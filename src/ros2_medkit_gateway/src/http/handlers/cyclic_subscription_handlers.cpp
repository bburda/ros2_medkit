// Copyright 2026 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_medkit_gateway/http/handlers/cyclic_subscription_handlers.hpp"

#include <regex>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Read the positional entity-id capture group from the typed request. The
/// legacy handlers used `req.matches[1]` without bounds checking; the typed
/// surface refuses out-of-range captures with ERR_INVALID_PARAMETER/400. This
/// path is unreachable in production because cpp-httplib routes only fire when
/// the regex matches, but the helper keeps the typed flow explicit.
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the positional subscription-id capture group.
tl::expected<std::string, ErrorInfo> read_subscription_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Build a typed CyclicSubscription DTO from CyclicSubscriptionInfo.
dto::CyclicSubscription subscription_to_dto(const CyclicSubscriptionInfo & info, const std::string & event_source) {
  dto::CyclicSubscription sub;
  sub.id = info.id;
  sub.observed_resource = info.resource_uri;
  sub.event_source = event_source;
  sub.protocol = info.protocol;
  sub.interval = interval_to_string(info.interval);
  return sub;
}

}  // namespace

CyclicSubscriptionHandlers::CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr,
                                                       ResourceSamplerRegistry & sampler_registry,
                                                       TransportRegistry & transport_registry, int max_duration_sec)
  : ctx_(ctx)
  , sub_mgr_(sub_mgr)
  , sampler_registry_(sampler_registry)
  , transport_registry_(transport_registry)
  , max_duration_sec_(max_duration_sec) {
}

// ---------------------------------------------------------------------------
// POST - create subscription
// ---------------------------------------------------------------------------
http::Result<std::pair<http::Created<dto::CyclicSubscription>, http::ResponseAttachments>>
CyclicSubscriptionHandlers::post_subscription(const http::TypedRequest & req,
                                              dto::CyclicSubscriptionCreateRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  // Validate protocol (optional, defaults to "sse")
  std::string protocol = body.protocol.value_or(std::string{"sse"});

  // Check transport is registered
  auto * transport = transport_registry_.get_transport(protocol);
  if (!transport) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_UNSUPPORTED_PROTOCOL,
                                     "Protocol '" + protocol + "' not available",
                                     json{{"parameter", "protocol"}, {"value", protocol}}));
  }

  // Parse interval
  CyclicInterval interval;
  try {
    interval = parse_interval(body.interval);
  } catch (const std::invalid_argument &) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Invalid interval. Must be 'fast', 'normal', or 'slow'.",
                                     json{{"parameter", "interval"}, {"value", body.interval}}));
  }

  // Validate duration
  int duration = body.duration;
  if (duration <= 0) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Duration must be a positive integer (seconds).",
                                     json{{"parameter", "duration"}, {"value", duration}}));
  }
  if (duration > max_duration_sec_) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Duration must not exceed " + std::to_string(max_duration_sec_) + " seconds.",
                                     json{{"parameter", "duration"}, {"max_value", max_duration_sec_}}));
  }

  // Parse resource URI to extract collection and resource path
  const std::string & resource = body.resource;
  auto parsed = parse_resource_uri(resource);
  if (!parsed) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_INVALID_RESOURCE_URI, "Invalid resource URI: " + parsed.error(),
                                     json{{"parameter", "resource"}, {"value", resource}}));
  }

  std::string entity_type = extract_entity_type(req.path());

  // Server-level resources (e.g. updates) skip entity-mismatch and collection checks
  bool is_server_level = parsed->entity_type.empty();

  if (!is_server_level) {
    // Validate resource URI references the same entity as the route
    if (parsed->entity_type != entity_type || parsed->entity_id != entity_id) {
      return tl::unexpected(make_error(400, ERR_X_MEDKIT_ENTITY_MISMATCH,
                                       "Resource URI must reference the same entity as the route",
                                       json{{"parameter", "resource"}, {"value", resource}}));
    }

    // Validate collection support
    auto known_collection = parse_resource_collection(parsed->collection);
    if (known_collection.has_value()) {
      // Known SOVD collection - check entity supports it
      if (!entity.supports_collection(*known_collection)) {
        return tl::unexpected(make_error(400, ERR_X_MEDKIT_COLLECTION_NOT_SUPPORTED,
                                         "Collection '" + parsed->collection + "' not supported for " + entity_type,
                                         json{{"collection", parsed->collection}, {"entity_type", entity_type}}));
      }
    } else if (parsed->collection.size() < 2 || parsed->collection.substr(0, 2) != "x-") {
      // Not a known collection and not a vendor extension
      return tl::unexpected(make_error(400, ERR_X_MEDKIT_INVALID_RESOURCE_URI,
                                       "Unknown collection '" + parsed->collection +
                                           "'. Use a known SOVD collection or 'x-' vendor extension.",
                                       json{{"collection", parsed->collection}}));
    }

    // Data collection requires a resource path (topic name)
    if (parsed->collection == "data" && parsed->resource_path.empty()) {
      return tl::unexpected(make_error(400, ERR_X_MEDKIT_INVALID_RESOURCE_URI,
                                       "Data collection requires a resource path (e.g. /api/v1/apps/{id}/data/{topic})",
                                       json{{"parameter", "resource"}, {"value", resource}}));
    }
  }

  // Check sampler is registered
  if (!sampler_registry_.has_sampler(parsed->collection)) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_COLLECTION_NOT_AVAILABLE,
                                     "No data provider for collection '" + parsed->collection + "'",
                                     json{{"collection", parsed->collection}}));
  }

  if (auto path_ok = validate_resource_path_support(sampler_registry_, *parsed, resource); !path_ok) {
    return tl::unexpected(path_ok.error());
  }

  // Create subscription
  auto result = sub_mgr_.create(entity_id, entity_type, resource, parsed->collection, parsed->resource_path, protocol,
                                interval, duration);
  if (!result) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, result.error()));
  }

  // Start transport delivery
  auto sampler = sampler_registry_.get_sampler(parsed->collection);
  auto event_source_result = transport->start(*result, *sampler, ctx_.node());
  if (!event_source_result) {
    sub_mgr_.remove(result->id);
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, event_source_result.error()));
  }

  auto sub_dto = subscription_to_dto(*result, *event_source_result);
  http::ResponseAttachments att;
  // The subscription is a child of the POST target, and `req.path()` already
  // carries the API prefix, so this is the same absolute form every `href` uses.
  att.with_location(child_resource_path(req.path(), sub_dto.id));
  return std::make_pair(http::Created<dto::CyclicSubscription>{std::move(sub_dto)}, std::move(att));
}

// ---------------------------------------------------------------------------
// GET - list subscriptions
// ---------------------------------------------------------------------------
http::Result<dto::Collection<dto::CyclicSubscription>>
CyclicSubscriptionHandlers::get_subscriptions(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto subs = sub_mgr_.list(entity_id);
  dto::Collection<dto::CyclicSubscription> response;
  for (const auto & sub : subs) {
    response.items.push_back(subscription_to_dto(sub, build_event_source(sub)));
  }

  return response;
}

// ---------------------------------------------------------------------------
// GET - get single subscription
// ---------------------------------------------------------------------------
http::Result<dto::CyclicSubscription> CyclicSubscriptionHandlers::get_subscription(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto sub_id_result = read_subscription_id(req);
  if (!sub_id_result) {
    return tl::unexpected(sub_id_result.error());
  }
  const std::string sub_id = *sub_id_result;

  auto sub = sub_mgr_.get(sub_id);
  if (!sub || sub->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  return subscription_to_dto(*sub, build_event_source(*sub));
}

// ---------------------------------------------------------------------------
// PUT - update subscription
// ---------------------------------------------------------------------------
http::Result<dto::CyclicSubscription>
CyclicSubscriptionHandlers::put_subscription(const http::TypedRequest & req,
                                             dto::CyclicSubscriptionUpdateRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto sub_id_result = read_subscription_id(req);
  if (!sub_id_result) {
    return tl::unexpected(sub_id_result.error());
  }
  const std::string sub_id = *sub_id_result;

  // Parse optional interval
  std::optional<CyclicInterval> new_interval;
  if (body.interval.has_value()) {
    try {
      new_interval = parse_interval(*body.interval);
    } catch (const std::invalid_argument &) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                       "Invalid interval. Must be 'fast', 'normal', or 'slow'.",
                                       json{{"parameter", "interval"}, {"value", *body.interval}}));
    }
  }

  // Validate optional duration
  std::optional<int> new_duration;
  if (body.duration.has_value()) {
    new_duration = *body.duration;
    if (*new_duration <= 0) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Duration must be a positive integer (seconds).",
                                       json{{"parameter", "duration"}, {"value", *new_duration}}));
    }
    if (*new_duration > max_duration_sec_) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                       "Duration must not exceed " + std::to_string(max_duration_sec_) + " seconds.",
                                       json{{"parameter", "duration"}, {"max_value", max_duration_sec_}}));
    }
  }

  // Verify subscription exists and belongs to this entity before updating
  auto existing = sub_mgr_.get(sub_id);
  if (!existing || existing->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  auto result = sub_mgr_.update(sub_id, new_interval, new_duration);
  if (!result) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  return subscription_to_dto(*result, build_event_source(*result));
}

// ---------------------------------------------------------------------------
// DELETE - remove subscription
// ---------------------------------------------------------------------------
http::Result<http::NoContent> CyclicSubscriptionHandlers::del_subscription(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto sub_id_result = read_subscription_id(req);
  if (!sub_id_result) {
    return tl::unexpected(sub_id_result.error());
  }
  const std::string sub_id = *sub_id_result;

  // Verify subscription exists and belongs to this entity before deleting
  auto existing = sub_mgr_.get(sub_id);
  if (!existing || existing->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  if (!sub_mgr_.remove(sub_id)) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  return http::NoContent{};
}

// ---------------------------------------------------------------------------
// GET /events - SSE stream (delegates to transport provider)
// ---------------------------------------------------------------------------
http::Result<http::SseStream> CyclicSubscriptionHandlers::sse_subscription_events(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto sub_id_result = read_subscription_id(req);
  if (!sub_id_result) {
    return tl::unexpected(sub_id_result.error());
  }
  const std::string sub_id = *sub_id_result;

  auto sub = sub_mgr_.get(sub_id);
  if (!sub) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  if (sub->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription not found", json{{"subscription_id", sub_id}}));
  }

  if (!sub_mgr_.is_active(sub_id)) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Subscription expired or inactive", json{{"subscription_id", sub_id}}));
  }

  auto * transport = transport_registry_.get_transport(sub->protocol);
  if (!transport) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_UNSUPPORTED_PROTOCOL,
                                     "Transport for protocol '" + sub->protocol + "' not available",
                                     json{{"protocol", sub->protocol}}));
  }

  // The transport returns an SseStream whose next_event closure carries the
  // sampler + tracker guard; the framework owns the chunked content provider
  // and the proxy-friendliness headers (Cache-Control: no-cache,
  // X-Accel-Buffering: no). Non-HTTP transports (MQTT, Zenoh, ...) return a
  // 501 ErrorInfo via the default implementation.
  return transport->make_sse_stream(sub_id);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string CyclicSubscriptionHandlers::build_event_source(const CyclicSubscriptionInfo & info) {
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/cyclic-subscriptions/" +
         info.id + "/events";
}

std::string CyclicSubscriptionHandlers::extract_entity_type(const std::string & path) {
  auto type = extract_entity_type_from_path(path);
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::FUNCTION:
      return "functions";
    case SovdEntityType::SERVER:
    case SovdEntityType::AREA:
    case SovdEntityType::UNKNOWN:
    default:
      RCLCPP_WARN(HandlerContext::logger(), "Unexpected entity type in cyclic subscription path: %s", path.c_str());
      return "apps";
  }
}

tl::expected<ParsedResourceUri, std::string>
CyclicSubscriptionHandlers::parse_resource_uri(const std::string & resource) {
  // Try entity-scoped format first: /api/v1/{entity_type}/{entity_id}/{collection}[/{resource_path}]
  static const std::regex entity_regex(R"(^/api/v1/(apps|components|functions)/([^/]+)/([^/]+)(/.*)?$)");
  std::smatch match;
  if (std::regex_match(resource, match, entity_regex)) {
    ParsedResourceUri parsed;
    parsed.entity_type = match[1].str();
    parsed.entity_id = match[2].str();
    parsed.collection = match[3].str();
    parsed.resource_path = match[4].matched ? match[4].str() : "";

    // Security: reject '..' as a path segment (not as substring of e.g. '/..foo')
    if (!parsed.resource_path.empty()) {
      std::string path = parsed.resource_path;
      size_t pos = 0;
      while (pos < path.size()) {
        size_t next = path.find('/', pos + 1);
        std::string segment = (next == std::string::npos) ? path.substr(pos) : path.substr(pos, next - pos);
        if (segment == "/.." || segment == "..") {
          return tl::make_unexpected("Resource path must not contain '..' as a path segment");
        }
        pos = (next == std::string::npos) ? path.size() : next;
      }
    }

    return parsed;
  }

  // Try server-level update status: /api/v1/updates/{update-package-id}/status
  static const std::regex update_status_regex(R"(^/api/v1/updates/([^/]+)/status$)");
  if (std::regex_match(resource, match, update_status_regex)) {
    ParsedResourceUri parsed;
    parsed.entity_type = "";  // server-level, no entity
    parsed.entity_id = "";
    parsed.collection = "updates";
    parsed.resource_path = match[1].str();  // update package ID
    return parsed;
  }

  return tl::make_unexpected(
      "Resource URI must match /api/v1/{apps|components|functions}/{id}/{collection}[/{path}] "
      "or /api/v1/updates/{id}/status");
}

tl::expected<void, ErrorInfo> CyclicSubscriptionHandlers::validate_resource_path_support(
    const ResourceSamplerRegistry & registry, const ParsedResourceUri & parsed, const std::string & resource) {
  if (parsed.resource_path.empty() || registry.honours_resource_path(parsed.collection)) {
    return {};
  }
  return tl::unexpected(make_error(
      400, ERR_X_MEDKIT_INVALID_RESOURCE_URI,
      "Collection '" + parsed.collection +
          "' is streamed as a whole and does not support a resource path. Subscribe to the collection URI without a "
          "trailing resource path.",
      json{{"parameter", "resource"}, {"value", resource}, {"collection", parsed.collection}}));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
