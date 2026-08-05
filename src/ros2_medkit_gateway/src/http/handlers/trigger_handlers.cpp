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

#include "ros2_medkit_gateway/core/http/handlers/trigger_handlers.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include <regex>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/data_point_suggest.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
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

/// Read the positional trigger-id capture group.
tl::expected<std::string, ErrorInfo> read_trigger_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Build a typed Trigger DTO from a TriggerInfo.
dto::Trigger trigger_info_to_dto(const TriggerInfo & info, const std::string & event_source) {
  dto::Trigger dto;
  dto.id = info.id;
  dto.status = (info.status == TriggerStatus::ACTIVE) ? "active" : "terminated";
  dto.observed_resource = info.resource_uri;
  dto.event_source = event_source;
  dto.protocol = info.protocol;

  // Build the flat trigger_condition JSON: condition_type + merged condition_params.
  json condition;
  condition["condition_type"] = info.condition_type;
  if (info.condition_params.is_object()) {
    for (auto & [key, val] : info.condition_params.items()) {
      condition[key] = val;
    }
  }
  dto.trigger_condition = condition;

  dto.multishot = info.multishot;
  dto.persistent = info.persistent;

  if (info.lifetime_sec.has_value()) {
    dto.lifetime = info.lifetime_sec.value();
  }

  if (!info.path.empty()) {
    dto.path = info.path;
  }

  if (info.log_settings.has_value()) {
    dto.log_settings = *info.log_settings;
  }

  return dto;
}

}  // namespace

TriggerHandlers::TriggerHandlers(HandlerContext & ctx, TriggerManager & trigger_mgr,
                                 std::shared_ptr<SSEClientTracker> client_tracker)
  : ctx_(ctx), trigger_mgr_(trigger_mgr), client_tracker_(std::move(client_tracker)) {
}

// ---------------------------------------------------------------------------
// POST - create trigger
// ---------------------------------------------------------------------------
http::Result<std::pair<http::Created<dto::Trigger>, http::ResponseAttachments>>
TriggerHandlers::post_trigger(const http::TypedRequest & req, dto::TriggerCreateRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  // Validate trigger_condition is an object.
  if (!body.trigger_condition.is_object()) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Missing or invalid 'trigger_condition'",
                                     json{{"parameter", "trigger_condition"}}));
  }

  auto trigger_condition_json = body.trigger_condition;
  if (!trigger_condition_json.contains("condition_type") || !trigger_condition_json["condition_type"].is_string()) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Missing or invalid 'condition_type' in trigger_condition",
                                     json{{"parameter", "trigger_condition.condition_type"}}));
  }

  std::string condition_type = trigger_condition_json["condition_type"].get<std::string>();

  // Extract condition_params (everything in trigger_condition except condition_type).
  json condition_params = trigger_condition_json;
  condition_params.erase("condition_type");

  // Parse resource URI.
  const std::string & resource = body.resource;
  auto parsed = parse_resource_uri(resource);
  if (!parsed) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_INVALID_RESOURCE_URI, "Invalid resource URI: " + parsed.error(),
                                     json{{"parameter", "resource"}, {"value", resource}}));
  }

  // Validate resource URI references the same entity as the route.
  std::string entity_type = extract_entity_type(req.path());
  if (parsed->entity_type != entity_type || parsed->entity_id != entity_id) {
    return tl::unexpected(make_error(400, ERR_X_MEDKIT_ENTITY_MISMATCH,
                                     "Resource URI must reference the same entity as the route",
                                     json{{"parameter", "resource"}, {"value", resource}}));
  }

  // Validate collection.
  static const std::unordered_set<std::string> known_collections = {"data", "faults", "operations", "updates", "logs"};
  if (known_collections.find(parsed->collection) == known_collections.end() &&
      parsed->collection.substr(0, 2) != "x-") {
    return tl::unexpected(
        make_error(400, ERR_INVALID_PARAMETER,
                   "Unknown collection. Supported: data, faults, operations, updates, logs, or x-* vendor extensions",
                   json{{"parameter", "resource"}, {"collection", parsed->collection}}));
  }

  // Validate path (JSON Pointer).
  std::string path = body.path.value_or(std::string{});
  if (!path.empty()) {
    if (path.size() > 1024) {
      return tl::unexpected(
          make_error(400, ERR_INVALID_PARAMETER, "Path too long (max 1024)", json{{"parameter", "path"}}));
    }
    try {
      (void)nlohmann::json::json_pointer(path);
    } catch (const nlohmann::json::exception &) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid JSON Pointer in 'path'",
                                       json{{"parameter", "path"}, {"value", path}}));
    }
  }

  // Validate protocol (default: sse).
  std::string protocol = body.protocol.value_or(std::string{"sse"});
  if (protocol != "sse") {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unsupported protocol. Supported: 'sse'",
                                     json{{"parameter", "protocol"}, {"value", protocol}}));
  }

  bool multishot = body.multishot.value_or(false);
  bool persistent = body.persistent.value_or(false);

  std::optional<int> lifetime_sec;
  if (body.lifetime.has_value()) {
    lifetime_sec = body.lifetime.value();
    if (*lifetime_sec <= 0) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Lifetime must be a positive integer (seconds)",
                                       json{{"parameter", "lifetime"}, {"value", *lifetime_sec}}));
    }
  }

  std::optional<json> log_settings;
  if (body.log_settings.has_value() && body.log_settings->is_object()) {
    log_settings = body.log_settings;
  }

  // For data triggers, resolve the resource_path URI segment to a full ROS 2 topic name.
  // The resource_path from parse_resource_uri is e.g. "/temperature" (URI segment), but the
  // ROS 2 topic name is e.g. "/sensor/temperature". We look up the entity's topics from the
  // cache and find the one whose name ends with the resource_path.
  std::string resolved_topic_name;
  if (parsed->collection == "data" && !parsed->resource_path.empty()) {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto aggregated = cache.get_entity_data(entity_id);
    for (const auto & topic : aggregated.topics) {
      // Match: topic.name ends with the resource_path (e.g., topic "/sensor/temperature"
      // ends with "/temperature"). Also accept exact match.
      if (topic.name == parsed->resource_path ||
          (topic.name.size() > parsed->resource_path.size() &&
           topic.name.compare(topic.name.size() - parsed->resource_path.size(), parsed->resource_path.size(),
                              parsed->resource_path) == 0)) {
        resolved_topic_name = topic.name;
        break;
      }
    }
    if (resolved_topic_name.empty() && entity_result->is_plugin) {
      // Plugin data points are enumerable right now, so a name the plugin does
      // not have can be rejected outright with a suggestion - parking it would
      // create a permanently ACTIVE trigger that never fires. DataProvider
      // first (covers plugin Components and DataProvider-only plugins), the
      // apps-scoped x-plc-data route as the fallback - the same order
      // data_handlers uses. Enumeration failure (plugin busy, link down, no
      // provider and no route) falls through to the late-binding path below,
      // as does an EXISTING point that merely failed to resolve: the entity
      // cache lags the plugin's live node map by up to one refresh cycle
      // (the retry budget is sized against it in gateway_node), and a
      // genuinely topic-less point surfaces through the retry-expiry warning.
      auto * pmgr = ctx_.node()->get_plugin_manager();
      std::optional<nlohmann::json> content;
      if (pmgr != nullptr) {
        if (auto * data_prov = pmgr->get_data_provider_for_entity(entity_id)) {
          try {
            if (auto result = data_prov->list_data(entity_id)) {
              content = result->content;
            }
          } catch (...) {
            // Enumeration is advisory here - a throwing provider must not
            // break trigger creation, it just skips the strict check.
          }
        }
        if (!content) {
          content = pmgr->fetch_entity_data_via_route(entity_id);
        }
      }
      // Same liveness guard as the fault-trigger enumeration in gateway_node:
      // a bridge with a dead PLC link answers 200 with connected=false and an
      // empty list, which must read as "unreadable right now", never as "this
      // entity has zero data points".
      if (content && EntityFreezeFrameCapture::content_has_live_data(*content)) {
        const auto values = EntityFreezeFrameCapture::values_from_list_content(*content);
        if (values.is_object()) {
          std::vector<std::string> names;
          names.reserve(values.size());
          for (auto it = values.begin(); it != values.end(); ++it) {
            names.push_back(it.key());
          }
          const std::string data_name = parsed->resource_path.substr(1);
          if (std::find(names.begin(), names.end(), data_name) == names.end()) {
            std::string msg = "data point '" + data_name + "' does not exist on entity '" + entity_id + "'";
            json params{
                {"parameter", "resource"}, {"resource_path", parsed->resource_path}, {"available_count", names.size()}};
            const std::string suggestion = suggest_data_point(data_name, names);
            if (!suggestion.empty()) {
              msg += " - did you mean '" + suggestion + "'?";
              params["did_you_mean"] = suggestion;
            }
            params["closest"] = closest_data_points(data_name, names, 20);
            return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, msg, params));
          }
        }
      }
    }
    if (resolved_topic_name.empty()) {
      RCLCPP_WARN(HandlerContext::logger(),
                  "Could not resolve resource_path '%s' to a ROS 2 topic for entity '%s'. "
                  "Topic subscription will be attempted when topic becomes available.",
                  parsed->resource_path.c_str(), entity_id.c_str());
    }
  }

  // Build create request.
  TriggerCreateRequest create_req;
  create_req.entity_id = entity_id;
  create_req.entity_type = entity_type;
  create_req.resource_uri = resource;
  create_req.collection = parsed->collection;
  create_req.resource_path = parsed->resource_path;
  create_req.resolved_topic_name = resolved_topic_name;
  create_req.path = path;
  create_req.condition_type = condition_type;
  create_req.condition_params = condition_params;
  create_req.protocol = protocol;
  create_req.multishot = multishot;
  create_req.persistent = persistent;
  create_req.lifetime_sec = lifetime_sec;
  create_req.log_settings = log_settings;

  auto result = trigger_mgr_.create(create_req);
  if (!result) {
    switch (result.error().code) {
      case TriggerError::CapacityExceeded:
      case TriggerError::PersistenceError:
        return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, result.error().message));
      case TriggerError::SubscribeFailed:
        return tl::unexpected(make_error(503, ERR_X_MEDKIT_SUBSCRIBE_FAILED, result.error().message));
      case TriggerError::NotFound:
        return tl::unexpected(make_error(404, ERR_ENTITY_NOT_FOUND, result.error().message));
      case TriggerError::ValidationError:
        return tl::unexpected(
            make_error(400, ERR_INVALID_PARAMETER, result.error().message, json{{"parameter", "trigger_condition"}}));
      default:
        // Defensive sentinel: a future TriggerError value that lands here was
        // missed when the enum was extended. Return 500 with a stable error
        // code so the omission is observable in tests instead of silently
        // mapping to 400.
        return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, result.error().message));
    }
  }

  auto event_source = build_event_source(*result);
  auto trigger_dto = trigger_info_to_dto(*result, event_source);
  http::ResponseAttachments att;
  // The trigger is a child of the POST target, and `req.path()` already carries
  // the API prefix, so this is the same absolute form every `href` uses.
  att.with_location(child_resource_path(req.path(), trigger_dto.id));
  return std::make_pair(http::Created<dto::Trigger>{std::move(trigger_dto)}, std::move(att));
}

// ---------------------------------------------------------------------------
// GET - list triggers
// ---------------------------------------------------------------------------
http::Result<dto::Collection<dto::Trigger>> TriggerHandlers::get_triggers(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto triggers = trigger_mgr_.list(entity_id);
  dto::Collection<dto::Trigger> response;
  for (const auto & trig : triggers) {
    response.items.push_back(trigger_info_to_dto(trig, build_event_source(trig)));
  }

  return response;
}

// ---------------------------------------------------------------------------
// GET - get single trigger
// ---------------------------------------------------------------------------
http::Result<dto::Trigger> TriggerHandlers::get_trigger(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto trigger_id_result = read_trigger_id(req);
  if (!trigger_id_result) {
    return tl::unexpected(trigger_id_result.error());
  }
  const std::string trigger_id = *trigger_id_result;

  auto trig = trigger_mgr_.get(trigger_id);
  if (!trig || trig->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  return trigger_info_to_dto(*trig, build_event_source(*trig));
}

// ---------------------------------------------------------------------------
// PUT - update trigger
// ---------------------------------------------------------------------------
http::Result<dto::Trigger> TriggerHandlers::put_trigger(const http::TypedRequest & req,
                                                        dto::TriggerUpdateRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto trigger_id_result = read_trigger_id(req);
  if (!trigger_id_result) {
    return tl::unexpected(trigger_id_result.error());
  }
  const std::string trigger_id = *trigger_id_result;

  int new_lifetime = body.lifetime;
  if (new_lifetime <= 0) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Lifetime must be a positive integer",
                                     json{{"parameter", "lifetime"}, {"value", new_lifetime}}));
  }

  // Verify trigger exists and belongs to this entity before updating.
  auto existing = trigger_mgr_.get(trigger_id);
  if (!existing || existing->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  auto result = trigger_mgr_.update(trigger_id, new_lifetime);
  if (!result) {
    if (result.error().code == TriggerError::NotFound) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, result.error().message, json{{"trigger_id", trigger_id}}));
    }
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, result.error().message,
                                     json{{"parameter", "lifetime"}, {"value", new_lifetime}}));
  }

  return trigger_info_to_dto(*result, build_event_source(*result));
}

// ---------------------------------------------------------------------------
// DELETE - remove trigger
// ---------------------------------------------------------------------------
http::Result<http::NoContent> TriggerHandlers::del_trigger(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto trigger_id_result = read_trigger_id(req);
  if (!trigger_id_result) {
    return tl::unexpected(trigger_id_result.error());
  }
  const std::string trigger_id = *trigger_id_result;

  // Verify trigger exists and belongs to this entity before deleting.
  auto existing = trigger_mgr_.get(trigger_id);
  if (!existing || existing->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  if (!trigger_mgr_.remove(trigger_id)) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  return http::NoContent{};
}

// ---------------------------------------------------------------------------
// GET /events - SSE stream
// ---------------------------------------------------------------------------
http::Result<http::SseStream> TriggerHandlers::sse_trigger_events(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto trigger_id_result = read_trigger_id(req);
  if (!trigger_id_result) {
    return tl::unexpected(trigger_id_result.error());
  }
  const std::string trigger_id = *trigger_id_result;

  auto trig = trigger_mgr_.get(trigger_id);
  if (!trig) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  if (trig->entity_id != entity_id) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", json{{"trigger_id", trigger_id}}));
  }

  if (!trigger_mgr_.is_active(trigger_id)) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Trigger expired or inactive", json{{"trigger_id", trigger_id}}));
  }

  if (!client_tracker_->try_connect()) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Maximum SSE client limit reached"));
  }

  // NOTE: trigger_mgr_ is captured by reference. TriggerHandlers must outlive all SSE
  // connections. This is guaranteed because GatewayNode destroys the REST server
  // (closing all connections) before destroying managers. See shutdown lifecycle in design spec.
  auto & mgr = trigger_mgr_;
  auto tracker = client_tracker_;

  // The SseStream's next_event closure is invoked once per cpp-httplib chunked
  // content tick. We keep the loop semantics of the legacy implementation:
  // wait for an event up to the keepalive timeout, then either write a
  // keepalive frame or drain every pending event (multishot triggers can
  // accumulate multiple events between wakeups).
  //
  // The tracker_guard releases the SSE client slot on closure destruction -
  // the framework holds the SseStream via a shared_ptr, so the guard's deleter
  // fires when the chunked content provider stops (either client disconnect
  // or end-of-stream).
  std::shared_ptr<void> tracker_guard(nullptr, [tracker](void *) {
    tracker->disconnect();
  });

  http::SseStream stream;
  stream.next_event = [&mgr, tid = trigger_id, tracker_guard,
                       sse_event_counter = static_cast<std::uint64_t>(0)](httplib::DataSink & sink) mutable -> bool {
    if (!mgr.is_active(tid)) {
      return false;
    }

    // Wait for event or timeout (15 seconds for keepalive).
    bool woken = mgr.wait_for_event(tid, std::chrono::milliseconds(15000));

    if (!mgr.is_active(tid)) {
      return false;
    }

    if (!woken) {
      // Timeout - send keepalive. The return value of sink.write doubles as the
      // "keep streaming" signal: write failure means the client disconnected,
      // so the closure terminates the stream.
      return sink.write(":keepalive\n\n", 12);
    }

    // Drain all pending events per wakeup (C2 fix: bounded queue can
    // accumulate multiple events between wakeups for multishot triggers).
    while (auto event = mgr.consume_pending_event(tid)) {
      ++sse_event_counter;
      std::string frame = "id: " + std::to_string(sse_event_counter) + "\n" + "data: " + event->dump() + "\n\n";
      if (!sink.write(frame.c_str(), frame.size())) {
        return false;
      }
    }
    return true;
  };

  return stream;
}

std::string TriggerHandlers::build_event_source(const TriggerInfo & info) {
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/triggers/" + info.id +
         "/events";
}

std::string TriggerHandlers::extract_entity_type(const std::string & path) {
  auto type = extract_entity_type_from_path(path);
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::AREA:
      return "areas";
    case SovdEntityType::FUNCTION:
      return "functions";
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      RCLCPP_WARN(HandlerContext::logger(), "Unexpected entity type in trigger path: %s", path.c_str());
      return "apps";
  }
}

tl::expected<TriggerParsedResourceUri, std::string> TriggerHandlers::parse_resource_uri(const std::string & resource) {
  // Entity-scoped format: /api/v1/{entity_type}/{entity_id}/{collection}[/{resource_path}]
  // Includes areas in addition to apps/components/functions
  static const std::regex entity_regex(R"(^/api/v1/(areas|apps|components|functions)/([^/]+)/([^/]+)(/.*)?$)");
  std::smatch match;
  if (std::regex_match(resource, match, entity_regex)) {
    TriggerParsedResourceUri parsed;
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
          return tl::make_unexpected(std::string("Resource path must not contain '..' as a path segment"));
        }
        pos = (next == std::string::npos) ? path.size() : next;
      }
    }

    return parsed;
  }

  return tl::make_unexpected(
      std::string("Resource URI must match /api/v1/{areas|apps|components|functions}/{id}/{collection}[/{path}]"));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
