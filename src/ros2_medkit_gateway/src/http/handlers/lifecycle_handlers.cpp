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

#include "ros2_medkit_gateway/core/http/handlers/lifecycle_handlers.hpp"

#include <memory>
#include <string>
#include <utility>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/lifecycle_provider.hpp"
#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Map a LifecycleProviderErrorInfo to a gateway ErrorInfo.
/// Modeled on operation_handlers' make_provider_error; NOT reused from there
/// because that helper is typed for OperationProviderErrorInfo and is
/// file-local to operation_handlers.cpp.
ErrorInfo to_error_info(const LifecycleProviderErrorInfo & e) {
  static constexpr size_t kMaxMsg = 512;
  int status = e.http_status < 400 ? 400 : (e.http_status > 599 ? 599 : e.http_status);
  std::string msg = e.message.size() > kMaxMsg ? e.message.substr(0, kMaxMsg) + "..." : e.message;

  switch (e.code) {
    case LifecycleProviderError::AccessDenied:
      return make_error(403, ERR_INSUFFICIENT_ACCESS_RIGHTS, std::move(msg));
    case LifecycleProviderError::PreconditionFailed:
      return make_error(409, ERR_PRECONDITION_NOT_FULFILLED, std::move(msg));
    case LifecycleProviderError::Unsupported:
      return make_error(501, ERR_NOT_IMPLEMENTED, std::move(msg));
    case LifecycleProviderError::EntityNotFound:
      return make_error(404, ERR_ENTITY_NOT_FOUND, std::move(msg));
    case LifecycleProviderError::TransportError:
    case LifecycleProviderError::Internal:
      return make_error(status, ERR_PLUGIN_ERROR, std::move(msg));
  }
  return make_error(status, ERR_PLUGIN_ERROR, std::move(msg));
}

}  // namespace

LifecycleHandlers::LifecycleHandlers(HandlerContext & ctx, PluginManager * plugin_mgr,
                                     std::shared_ptr<LifecycleStateReader> lifecycle_reader)
  : ctx_(ctx), plugin_mgr_(plugin_mgr), lifecycle_reader_(std::move(lifecycle_reader)) {
}

// =============================================================================
// GET /{entity}/status
// =============================================================================

http::Result<dto::LifecycleStatusResponse> LifecycleHandlers::handle_get_status(const http::TypedRequest & req) {
  auto id_raw = req.path_param("1");
  if (!id_raw) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
  }
  const std::string entity_id = *id_raw;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  const std::string base = api_path((entity.type == EntityType::APP ? "/apps/" : "/components/") + entity.id);

  // Delegate to LifecycleProvider when one is registered for this entity.
  if (plugin_mgr_) {
    auto * provider = plugin_mgr_->get_lifecycle_provider_for_entity(entity_id);
    if (provider) {
      try {
        auto result = provider->get_status(entity_id);
        if (!result) {
          return tl::make_unexpected(to_error_info(result.error()));
        }
        auto resp = *result;
        // The JSON writer only validates the status enum on read, not on write,
        // so guard the SOVD ready/notReady contract against a misbehaving plugin.
        if (resp.status != "ready" && resp.status != "notReady") {
          RCLCPP_ERROR(HandlerContext::logger(), "LifecycleProvider returned invalid status '%s' for entity '%s'",
                       resp.status.c_str(), entity_id.c_str());
          return tl::make_unexpected(make_error(500, ERR_PLUGIN_ERROR, "Plugin returned invalid lifecycle status"));
        }
        // Fill absolute transition URIs for each present field.
        if (resp.start.has_value()) {
          resp.start = base + "/status/start";
        }
        if (resp.restart.has_value()) {
          resp.restart = base + "/status/restart";
        }
        if (resp.force_restart.has_value()) {
          resp.force_restart = base + "/status/force-restart";
        }
        if (resp.shutdown.has_value()) {
          resp.shutdown = base + "/status/shutdown";
        }
        if (resp.force_shutdown.has_value()) {
          resp.force_shutdown = base + "/status/force-shutdown";
        }
        return resp;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(HandlerContext::logger(), "Plugin LifecycleProvider threw for entity '%s': %s", entity_id.c_str(),
                     e.what());
        return tl::make_unexpected(make_error(500, ERR_PLUGIN_ERROR, "Plugin threw exception"));
      } catch (...) {
        RCLCPP_ERROR(HandlerContext::logger(), "Plugin LifecycleProvider threw unknown exception for entity '%s'",
                     entity_id.c_str());
        return tl::make_unexpected(make_error(500, ERR_PLUGIN_ERROR, "Plugin threw unknown exception"));
      }
    }
  }

  // No provider registered - build default status from the entity cache.
  const auto & cache = ctx_.node()->get_thread_safe_cache();
  dto::LifecycleStatusResponse resp;

  if (entity.type == EntityType::APP) {
    auto app = cache.get_app(entity_id);
    std::optional<std::string> get_state_path;
    if (app && lifecycle_reader_) {
      get_state_path = find_lifecycle_get_state_path(app->services);
    }
    if (app && !app->is_online) {
      // An offline node cannot be "active", so skip the lifecycle read. This also
      // short-circuits a crashed managed node whose get_state/change_state services
      // still linger in the last-cached App::services: without this, the reader
      // would wait_for_service to exhaustion + spin to the timeout on every poll,
      // serializing concurrent /status reads on exactly the failure case operators
      // poll hardest. A plain offline node already short-circuited here.
      resp.status = "notReady";
    } else if (app && lifecycle_reader_ && get_state_path.has_value()) {
      // Managed lifecycle node: read the real state. "active" is the only ready state.
      auto state = lifecycle_reader_->get_state(*get_state_path);
      resp.status = lifecycle_status_from_state(state);
    } else {
      // Plain node, no reader wired, or a node exposing get_state without
      // change_state (deliberately treated as unmanaged): graph presence is the
      // best available signal.
      resp.status = (app && app->is_online) ? "ready" : "notReady";
    }
  } else {
    // Local component readiness. The synthetic host component (carrying
    // host_metadata) is always ready while we are serving this request: its
    // substrate is the gateway host, which is reachable by definition. Any
    // other local component derives readiness from its hosted Apps - a
    // component that hosts Apps is notReady when ALL of them are offline (its
    // subsystem is down), while a component with no hosted Apps is a pure
    // grouping and stays ready (a single down App is that App's own notReady,
    // not enough to mark the component down). Remote components are handled
    // earlier by aggregation forwarding.
    auto component = cache.get_component(entity_id);
    bool ready = true;
    if (component && !component->host_metadata.has_value()) {
      const auto app_ids = cache.get_apps_for_component(entity_id);
      if (!app_ids.empty()) {
        ready = false;
        for (const auto & app_id : app_ids) {
          auto hosted = cache.get_app(app_id);
          if (hosted && hosted->is_online) {
            ready = true;
            break;
          }
        }
      }
    }
    resp.status = ready ? "ready" : "notReady";
  }

  // No transition URIs for the default (no-provider) path.
  return resp;
}

// =============================================================================
// PUT /{entity}/status/{action}
// =============================================================================

http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
LifecycleHandlers::handle_transition(const http::TypedRequest & req, std::string_view transition) {
  auto id_raw = req.path_param("1");
  if (!id_raw) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
  }
  const std::string entity_id = *id_raw;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  const std::string base = api_path((entity.type == EntityType::APP ? "/apps/" : "/components/") + entity.id);

  if (plugin_mgr_) {
    auto * provider = plugin_mgr_->get_lifecycle_provider_for_entity(entity_id);
    if (provider) {
      try {
        auto result = provider->request_transition(entity_id, transition);
        if (!result) {
          return tl::make_unexpected(to_error_info(result.error()));
        }
        http::ResponseAttachments att;
        att.with_location(base + "/status");
        return std::make_pair(http::Accepted<http::NoContent>{http::NoContent{}}, std::move(att));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(HandlerContext::logger(), "Plugin LifecycleProvider threw for entity '%s': %s", entity_id.c_str(),
                     e.what());
        return tl::make_unexpected(make_error(500, ERR_PLUGIN_ERROR, "Plugin threw exception"));
      } catch (...) {
        RCLCPP_ERROR(HandlerContext::logger(), "Plugin LifecycleProvider threw unknown exception for entity '%s'",
                     entity_id.c_str());
        return tl::make_unexpected(make_error(500, ERR_PLUGIN_ERROR, "Plugin threw unknown exception"));
      }
    }
  }

  // No provider - lifecycle control not available.
  return tl::make_unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Lifecycle control not available for this entity"));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
