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

#include "ros2_medkit_gateway/core/http/handlers/operation_handlers.hpp"

#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/operation_provider.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/operations.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"
#include "ros2_medkit_serialization/type_introspection.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

// =============================================================================
// Helper free functions
// =============================================================================

/// Sanitize a plugin-supplied error into the standard `x-medkit-plugin-error`
/// shape: clamp HTTP status to [400, 599] and truncate message at 512 chars.
ErrorInfo make_plugin_error(int http_status, const std::string & message, json extra_params = {}) {
  static constexpr size_t kMaxMessageLength = 512;
  int status = http_status < 400 ? 400 : (http_status > 599 ? 599 : http_status);
  std::string msg = message.size() > kMaxMessageLength ? message.substr(0, kMaxMessageLength) + "..." : message;
  return make_error(status, ERR_PLUGIN_ERROR, std::move(msg), std::move(extra_params));
}

/// Read the first positional capture group (entity_id) with the same "missing
/// capture is treated as 400 invalid-request" semantics as the other migrated
/// handlers (matches the legacy `req.matches.size() < N` guard).
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the second positional capture group (operation_id).
tl::expected<std::string, ErrorInfo> read_operation_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the third positional capture group (execution_id).
tl::expected<std::string, ErrorInfo> read_execution_id(const http::TypedRequest & req) {
  auto raw = req.path_param("3");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Look up the entity_id -> EntityType -> AggregatedOperations triple. The
/// SOVD entity hierarchy (areas, components, apps, functions) all expose
/// operations but the cache lookup is per-type. Returns a typed error if the
/// entity type does not support operations (e.g. SERVER / UNKNOWN).
struct EntityOpsLookup {
  AggregatedOperations ops;
  std::string entity_type;  ///< "component" | "app" | "area" | "function"
};

tl::expected<EntityOpsLookup, ErrorInfo> resolve_entity_operations(const ThreadSafeEntityCache & cache,
                                                                   SovdEntityType type, const std::string & entity_id) {
  EntityOpsLookup out;
  switch (type) {
    case SovdEntityType::COMPONENT:
      out.ops = cache.get_component_operations(entity_id);
      out.entity_type = "component";
      return out;
    case SovdEntityType::APP:
      out.ops = cache.get_app_operations(entity_id);
      out.entity_type = "app";
      return out;
    case SovdEntityType::AREA:
      out.ops = cache.get_area_operations(entity_id);
      out.entity_type = "area";
      return out;
    case SovdEntityType::FUNCTION:
      out.ops = cache.get_function_operations(entity_id);
      out.entity_type = "function";
      return out;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      return tl::make_unexpected(make_error(404, ERR_ENTITY_NOT_FOUND, "Entity type does not support operations",
                                            json{{"entity_id", entity_id}}));
  }
}

/// Convert a ROS 2 action goal status into the SOVD `ExecutionStatus` enum
/// the gateway emits on the wire. Identical mapping to the legacy helper.
std::string sovd_status_from_ros2(ActionGoalStatus status) {
  switch (status) {
    case ActionGoalStatus::ACCEPTED:
    case ActionGoalStatus::EXECUTING:
    case ActionGoalStatus::CANCELING:
      return "running";
    case ActionGoalStatus::SUCCEEDED:
      return "completed";
    case ActionGoalStatus::CANCELED:
    case ActionGoalStatus::ABORTED:
      return "failed";
    case ActionGoalStatus::UNKNOWN:
    default:
      return "running";
  }
}

/// Build the `XMedkitOperationItem` block that decorates every OperationItem
/// returned by the runtime discovery branch. Shared by list_operations and
/// get_operation so the wire shape is identical for both.
dto::XMedkitOperationItem build_service_xmedkit(const ServiceInfo & svc, const std::string & entity_id,
                                                ros2_medkit_serialization::TypeIntrospection * type_introspection) {
  dto::XMedkitRos2 ros2;
  ros2.service = svc.full_path;
  ros2.type = svc.type;
  ros2.kind = "service";

  dto::XMedkitOperationItem x_medkit;
  x_medkit.ros2 = ros2;
  x_medkit.entity_id = entity_id;
  x_medkit.source = "ros2_medkit_gateway";

  if (type_introspection != nullptr && !svc.type.empty()) {
    try {
      // Schemas are assembled once per type and cached (shared) in
      // TypeIntrospection, so repeated /operations requests reuse them instead
      // of rebuilding + deep-copying (issue #442).
      x_medkit.type_info = *type_introspection->get_service_type_info(svc.type);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for service '%s': %s", svc.type.c_str(),
                   e.what());
    }
  }
  return x_medkit;
}

dto::XMedkitOperationItem build_action_xmedkit(const ActionInfo & act, const std::string & entity_id,
                                               ros2_medkit_serialization::TypeIntrospection * type_introspection) {
  dto::XMedkitRos2 ros2;
  ros2.action = act.full_path;
  ros2.type = act.type;
  ros2.kind = "action";

  dto::XMedkitOperationItem x_medkit;
  x_medkit.ros2 = ros2;
  x_medkit.entity_id = entity_id;
  x_medkit.source = "ros2_medkit_gateway";

  if (type_introspection != nullptr && !act.type.empty()) {
    try {
      x_medkit.type_info = *type_introspection->get_action_type_info(act.type);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for action '%s': %s", act.type.c_str(), e.what());
    }
  }
  return x_medkit;
}

/// Map an `OperationProviderErrorInfo` (from the typed plugin ABI) into the
/// SOVD `x-medkit-plugin-error` wire shape via `make_plugin_error`.
ErrorInfo make_provider_error(const OperationProviderErrorInfo & info, const std::string & entity_id,
                              const std::optional<std::string> & operation_id = std::nullopt) {
  json params{{"entity_id", entity_id}};
  if (operation_id.has_value()) {
    params["operation_id"] = *operation_id;
  }
  return make_plugin_error(info.http_status, info.message, std::move(params));
}

}  // namespace

namespace detail {

/// Shared outcome mapping for the two cancel entry points (DELETE execution
/// and PUT-stop) - issue #576:
/// - kOk: success.
/// - kTimeout: the CancelGoal response did not arrive, so the outcome is
///   UNKNOWN, not a rejection. Reconcile against the tracked goal fed by the
///   /_action/status stream: if the stream already shows CANCELING/CANCELED
///   the cancellation is in fact happening -> success. Otherwise 504 +
///   standard `not-responding` (SOVD: "no response from the underlying
///   entity in time"). The tracked status is NOT written on this path - the
///   status stream stays the authority.
/// - kServiceUnavailable: 503 - the action server is gone; retry may help.
/// - kTransportError: 500 - the request could not be delivered/parsed.
/// - kErrorResponse: 400 + `x-medkit-ros2-action-rejected` - the server
///   answered and definitively refused (return_code 1/2/3). This function is
///   the ONLY place that words those three codes: the transport deliberately
///   keeps no second copy, because only the HTTP layer knows whether the
///   client asked to cancel or to stop, and two hand-maintained tables for
///   the same protocol constants drift silently.
/// - kNotTracked: 404 - the execution no longer exists (evicted between the
///   handler's lookup and the manager's re-check); no request ever reached
///   the action server, so an availability code would misdirect the operator.
/// - kInvalidRequest: 400 + `invalid-parameter` - malformed execution id.
///
/// @param verb "Cancel" or "Stop" - keeps each entry point's message wording.
std::optional<CancelFailure> map_cancel_result(const ActionCancelResult & result, OperationManager & operation_mgr,
                                               const std::string & execution_id, const char * verb) {
  switch (result.outcome) {
    case CancelOutcome::kOk:
      return std::nullopt;
    case CancelOutcome::kTimeout: {
      auto tracked = operation_mgr.get_tracked_goal(execution_id);
      if (tracked.has_value() &&
          (tracked->status == ActionGoalStatus::CANCELING || tracked->status == ActionGoalStatus::CANCELED)) {
        return std::nullopt;
      }
      std::string message =
          std::string(verb) + " outcome unknown: the action server did not answer the cancel request in time. ";
      // CANCELED already reconciled above, so a terminal status here means
      // the goal finished on its own. Telling the client to watch for
      // progress would describe something that cannot happen - say what the
      // gateway already knows instead.
      if (tracked.has_value() &&
          (tracked->status == ActionGoalStatus::SUCCEEDED || tracked->status == ActionGoalStatus::ABORTED)) {
        message += "The execution status resource already reports the goal as " +
                   action_status_to_string(tracked->status) + ", so there is nothing left to cancel.";
      } else {
        message += "Poll the execution status resource to observe the goal's progress.";
      }
      return CancelFailure{504, ERR_NOT_RESPONDING, std::move(message)};
    }
    case CancelOutcome::kServiceUnavailable:
      return CancelFailure{503, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                           result.error_message.empty() ? "Cancel service not available" : result.error_message};
    case CancelOutcome::kTransportError:
      return CancelFailure{500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                           result.error_message.empty() ? std::string(verb) + " failed" : result.error_message};
    case CancelOutcome::kNotTracked:
      return CancelFailure{404, ERR_RESOURCE_NOT_FOUND, "Execution not found"};
    case CancelOutcome::kInvalidRequest:
      return CancelFailure{400, ERR_INVALID_PARAMETER,
                           result.error_message.empty() ? "Invalid execution id" : result.error_message};
    case CancelOutcome::kErrorResponse:
      break;
  }
  std::string message;
  switch (result.return_code) {
    case 1:
      message = std::string(verb) + " request rejected";
      break;
    case 2:
      message = "Unknown execution ID";
      break;
    case 3:
      message = "Execution already terminated";
      break;
    default:
      message = result.error_message.empty() ? std::string(verb) + " failed" : result.error_message;
  }
  return CancelFailure{400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, std::move(message)};
}

}  // namespace detail

// =============================================================================
// GET /{entity}/operations - list operations
// =============================================================================

http::Result<dto::Collection<dto::OperationItem>> OperationHandlers::list_operations(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Delegate to plugin OperationProvider for plugin-owned entities.
  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * op_prov = pmgr ? pmgr->get_operation_provider_for_entity(entity_id) : nullptr;
    if (op_prov == nullptr) {
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "No operation provider for plugin entity '" + entity_id + "'"));
    }
    try {
      auto result = op_prov->list_operations(entity_id);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id));
      }
      return *result;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto lookup = resolve_entity_operations(cache, entity_info.sovd_type(), entity_id);
  if (!lookup) {
    return tl::make_unexpected(lookup.error());
  }
  const auto & ops = lookup->ops;
  RCLCPP_DEBUG(HandlerContext::logger(), "Listing operations for %s '%s': %zu services, %zu actions",
               lookup->entity_type.c_str(), entity_id.c_str(), ops.services.size(), ops.actions.size());

  // Build OperationItem list (services + actions). The wire shape is shared
  // with handle_get_operation so build_service_xmedkit / build_action_xmedkit
  // are factored out into the anon-ns helpers above.
  dto::Collection<dto::OperationItem> collection;
  auto data_access_mgr = ctx_.node()->get_data_access_manager();
  auto type_introspection = data_access_mgr->get_type_introspection();

  for (const auto & svc : ops.services) {
    dto::OperationItem item;
    item.id = svc.name;
    item.name = svc.name;
    item.proximity_proof_required = false;
    item.asynchronous_execution = false;
    item.x_medkit = build_service_xmedkit(svc, entity_id, type_introspection);
    collection.items.push_back(std::move(item));
  }
  for (const auto & act : ops.actions) {
    dto::OperationItem item;
    item.id = act.name;
    item.name = act.name;
    item.proximity_proof_required = false;
    item.asynchronous_execution = true;
    item.x_medkit = build_action_xmedkit(act, entity_id, type_introspection);
    collection.items.push_back(std::move(item));
  }

  // Typed fan-out for the operations list. Replacement for the legacy raw-JSON
  // `merge_peer_items` mutator: peer items are parsed as `dto::OperationItem`
  // via JsonReader, malformed items are surfaced through `peer_dropped_items`
  // on the standard XMedkitCollection. The fan_out_collection helper still
  // operates on the raw cpp-httplib request - we use the typed-request escape
  // hatch deliberately here; a later commit will accept the typed request
  // directly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto fan_out = fan_out_collection<dto::OperationItem>(ctx_.aggregation_manager(), raw_req);
  for (auto & item : fan_out.items) {
    collection.items.push_back(std::move(item));
  }
  if (fan_out.partial || !fan_out.dropped_items.empty()) {
    dto::XMedkitCollection xm;
    if (fan_out.partial) {
      xm.partial = true;
      xm.failed_peers = fan_out.failed_peers;
    }
    if (!fan_out.dropped_items.empty()) {
      xm.peer_dropped_items = fan_out.dropped_items;
    }
    collection.x_medkit = std::move(xm);
  }
  return collection;
}

// =============================================================================
// GET /{entity}/operations/{op_id} - get operation details
// =============================================================================

http::Result<dto::OperationDetail> OperationHandlers::get_operation(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * op_prov = pmgr ? pmgr->get_operation_provider_for_entity(entity_id) : nullptr;
    if (op_prov == nullptr) {
      return tl::make_unexpected(
          make_error(404, ERR_OPERATION_NOT_FOUND, "No operation provider for plugin entity '" + entity_id + "'"));
    }
    try {
      auto result = op_prov->get_operation(entity_id, operation_id);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id, operation_id));
      }
      dto::OperationDetail detail;
      detail.item = std::move(*result);
      return detail;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto lookup = resolve_entity_operations(cache, entity_info.sovd_type(), entity_id);
  if (!lookup) {
    return tl::make_unexpected(lookup.error());
  }
  const auto & ops = lookup->ops;

  std::optional<ServiceInfo> service_info;
  std::optional<ActionInfo> action_info;
  for (const auto & svc : ops.services) {
    if (svc.name == operation_id) {
      service_info = svc;
      break;
    }
  }
  if (!service_info.has_value()) {
    for (const auto & act : ops.actions) {
      if (act.name == operation_id) {
        action_info = act;
        break;
      }
    }
  }
  if (!service_info.has_value() && !action_info.has_value()) {
    return tl::make_unexpected(make_error(404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                          json{{"entity_id", entity_id}, {"operation_id", operation_id}}));
  }

  auto data_access_mgr = ctx_.node()->get_data_access_manager();
  auto type_introspection = data_access_mgr->get_type_introspection();

  dto::OperationDetail detail;
  if (service_info.has_value()) {
    detail.item.id = service_info->name;
    detail.item.name = service_info->name;
    detail.item.proximity_proof_required = false;
    detail.item.asynchronous_execution = false;
    detail.item.x_medkit = build_service_xmedkit(*service_info, entity_id, type_introspection);
  } else {
    detail.item.id = action_info->name;
    detail.item.name = action_info->name;
    detail.item.proximity_proof_required = false;
    detail.item.asynchronous_execution = true;
    detail.item.x_medkit = build_action_xmedkit(*action_info, entity_id, type_introspection);
  }
  return detail;
}

// =============================================================================
// POST /{entity}/operations/{op_id}/executions - start execution
// =============================================================================

http::Result<
    std::pair<std::variant<dto::OperationExecutionResult, dto::ExecutionCreateAsync>, http::ResponseAttachments>>
OperationHandlers::create_execution(const http::TypedRequest & req, dto::ExecutionCreateRequest body) {
  using ResultVariant = std::variant<dto::OperationExecutionResult, dto::ExecutionCreateAsync>;
  using SuccessPair = std::pair<ResultVariant, http::ResponseAttachments>;

  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Locks gate all mutating operations, including plugin-owned entities.
  if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "operations"); !lock_err) {
    return tl::make_unexpected(lock_err.error());
  }

  // Plugin delegation: the plugin's `execute_operation` returns the typed
  // `OperationExecutionResult` envelope. Pre-typed plugins kept the raw JSON
  // body shape, so the wire format is unchanged.
  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * op_prov = pmgr ? pmgr->get_operation_provider_for_entity(entity_id) : nullptr;
    if (op_prov == nullptr) {
      return tl::make_unexpected(
          make_error(404, ERR_OPERATION_NOT_FOUND, "No operation provider for plugin entity '" + entity_id + "'"));
    }
    // Plugin ABI takes the raw parameter payload. Pre-typed handlers passed
    // the full request body JSON through (plugins read their own operation-
    // specific keys, e.g. OPC-UA reads `fault_code` / `comment`, UDS reads
    // `session_type` / `reset_type`). Preserve that contract by re-parsing
    // the raw body so callers do not have to wrap operation parameters in
    // a `parameters` / `goal` / `request` envelope when targeting plugin
    // entities. The typed `ExecutionCreateRequest` DTO still drives the ROS
    // runtime path below where the envelope is the SOVD-conforming shape.
    json params = json::object();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
    if (!raw_req.body.empty()) {
      auto parsed = json::parse(raw_req.body, nullptr, false);
      if (!parsed.is_discarded() && parsed.is_object()) {
        params = std::move(parsed);
      }
    }
    try {
      auto result = op_prov->execute_operation(entity_id, operation_id, params);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id, operation_id));
      }
      return SuccessPair{ResultVariant{std::move(*result)}, http::ResponseAttachments{}};
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin OperationProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  // Runtime discovery branch.
  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto lookup = resolve_entity_operations(cache, entity_info.sovd_type(), entity_id);
  if (!lookup) {
    return tl::make_unexpected(lookup.error());
  }
  const auto & ops = lookup->ops;
  const std::string id_field = (lookup->entity_type == "app") ? "app_id" : "component_id";

  std::optional<ServiceInfo> service_info;
  std::optional<ActionInfo> action_info;
  for (const auto & svc : ops.services) {
    if (svc.name == operation_id) {
      service_info = svc;
      break;
    }
  }
  if (!service_info.has_value()) {
    for (const auto & act : ops.actions) {
      if (act.name == operation_id) {
        action_info = act;
        break;
      }
    }
  }
  if (!service_info.has_value() && !action_info.has_value()) {
    return tl::make_unexpected(make_error(404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                          json{{"entity_id", entity_id}, {"operation_id", operation_id}}));
  }

  auto * operation_mgr = ctx_.node()->get_operation_manager();

  // ---- Action (asynchronous: 202 + Location) ----
  if (action_info.has_value()) {
    json goal_data = json::object();
    if (body.parameters.has_value()) {
      goal_data = *body.parameters;
    } else if (body.goal.has_value()) {
      goal_data = *body.goal;
    }
    std::string action_type = action_info->type;
    if (body.type.has_value()) {
      action_type = *body.type;
    }

    auto action_result = operation_mgr->send_action_goal(action_info->full_path, action_type, goal_data, entity_id);

    if (action_result.success && action_result.goal_accepted) {
      dto::ExecutionCreateAsync async_dto;
      async_dto.id = action_result.goal_id;
      async_dto.status = "running";

      const std::string base_path = (lookup->entity_type == "app") ? "/api/v1/apps/" : "/api/v1/components/";
      const std::string location =
          base_path + entity_id + "/operations/" + operation_id + "/executions/" + action_result.goal_id;

      http::ResponseAttachments att;
      att.with_header("Location", location);
      // dto_alternate_status<ExecutionCreateAsync> == 202, so the framework
      // emits the 202 status without an explicit override here.
      return SuccessPair{ResultVariant{std::move(async_dto)}, std::move(att)};
    }
    if (action_result.success && !action_result.goal_accepted) {
      return tl::make_unexpected(make_error(
          400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, "Goal rejected",
          json{{id_field, entity_id},
               {"operation_id", operation_id},
               {"details", action_result.error_message.empty() ? "Goal rejected" : action_result.error_message}}));
    }
    return tl::make_unexpected(make_error(
        500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE, "Action execution failed",
        json{{id_field, entity_id}, {"operation_id", operation_id}, {"details", action_result.error_message}}));
  }

  // ---- Service (synchronous: 200) ----
  // service_info.has_value() is guaranteed here because the !service && !action
  // 404 branch returned earlier.
  json request_data = json::object();
  if (body.parameters.has_value()) {
    request_data = *body.parameters;
  } else if (body.request.has_value()) {
    request_data = *body.request;
  }
  std::string service_type = service_info->type;
  if (body.type.has_value()) {
    service_type = *body.type;
  }

  auto svc_result = operation_mgr->call_service(service_info->full_path, service_type, request_data);
  if (svc_result.success) {
    // Wire shape: `{"parameters": <ros_response>}`. The DTO envelope
    // `OperationExecutionResult` is opaque so the bare object is emitted
    // verbatim by `JsonWriter<OperationExecutionResult>`.
    dto::OperationExecutionResult sync_result;
    sync_result.content = json{{"parameters", svc_result.response}};
    return SuccessPair{ResultVariant{std::move(sync_result)}, http::ResponseAttachments{}};
  }
  // Inline service-error path is replaced with the framework error channel:
  // `make_error` produces a SOVD GenericError that the typed wrapper renders.
  return tl::make_unexpected(
      make_error(500, ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE, "Service call failed",
                 json{{id_field, entity_id}, {"operation_id", operation_id}, {"details", svc_result.error_message}}));
}

// =============================================================================
// GET /{entity}/operations/{op_id}/executions - list executions
// =============================================================================

http::Result<dto::Collection<dto::ExecutionId>> OperationHandlers::list_executions(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                          json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  std::string namespace_path;
  bool entity_found = false;

  if (auto component = cache.get_component(entity_id)) {
    namespace_path = component->namespace_path;
    entity_found = true;
  }
  if (!entity_found) {
    if (auto app = cache.get_app(entity_id)) {
      for (const auto & act : app->actions) {
        if (act.name == operation_id) {
          namespace_path = act.full_path.substr(0, act.full_path.rfind('/'));
          entity_found = true;
          break;
        }
      }
    }
  }
  if (!entity_found) {
    return tl::make_unexpected(
        make_error(404, ERR_ENTITY_NOT_FOUND, "Entity not found", json{{"entity_id", entity_id}}));
  }

  const std::string action_path = namespace_path + "/" + operation_id;
  auto * operation_mgr = ctx_.node()->get_operation_manager();
  auto goals = operation_mgr->get_goals_for_action(action_path);

  // Typed Collection<ExecutionId> - replaces the legacy ad-hoc
  // `{"items": [{"id": "..."}]}` JSON literal. The wire shape is identical
  // (per JsonWriter<Collection<ExecutionId>>::write) but the per-item schema
  // is now enforced by JsonReader<ExecutionId> on round-trip.
  dto::Collection<dto::ExecutionId> collection;
  for (const auto & goal : goals) {
    dto::ExecutionId item;
    item.id = goal.goal_id;
    collection.items.push_back(std::move(item));
  }
  return collection;
}

// =============================================================================
// GET /{entity}/operations/{op_id}/executions/{exec_id} - execution status
// =============================================================================

http::Result<dto::OperationExecution> OperationHandlers::get_execution(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  auto exec_id_result = read_execution_id(req);
  if (!exec_id_result) {
    return tl::make_unexpected(exec_id_result.error());
  }
  const std::string execution_id = *exec_id_result;

  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                          json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  auto * operation_mgr = ctx_.node()->get_operation_manager();
  auto goal_info = operation_mgr->get_tracked_goal(execution_id);
  if (!goal_info.has_value()) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}}));
  }

  dto::OperationExecution exec_dto;
  exec_dto.status = sovd_status_from_ros2(goal_info->status);
  exec_dto.capability = "execute";

  if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
    exec_dto.parameters = goal_info->last_feedback;
  }

  dto::XMedkitRos2 exec_ros2;
  exec_ros2.action = goal_info->action_path;
  exec_ros2.type = goal_info->action_type;

  dto::XMedkitOperationExecution exec_x_medkit;
  exec_x_medkit.goal_id = execution_id;
  exec_x_medkit.ros2_status = action_status_to_string(goal_info->status);
  exec_x_medkit.ros2 = exec_ros2;
  exec_dto.x_medkit = exec_x_medkit;
  return exec_dto;
}

// =============================================================================
// DELETE /{entity}/operations/{op_id}/executions/{exec_id} - cancel
// =============================================================================

http::Result<http::NoContent> OperationHandlers::cancel_execution(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  auto exec_id_result = read_execution_id(req);
  if (!exec_id_result) {
    return tl::make_unexpected(exec_id_result.error());
  }
  const std::string execution_id = *exec_id_result;

  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                          json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  // Lock-check against the resolved entity (best-effort - if the entity
  // cannot be resolved here we still let the operation manager produce the
  // canonical 404 below).
  const auto entity_info = ctx_.get_entity_info(entity_id);
  if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "operations"); !lock_err) {
    return tl::make_unexpected(lock_err.error());
  }

  auto * operation_mgr = ctx_.node()->get_operation_manager();
  auto goal_info = operation_mgr->get_tracked_goal(execution_id);
  if (!goal_info.has_value()) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}}));
  }

  auto result = operation_mgr->cancel_action_goal(goal_info->action_path, execution_id);
  auto failure = detail::map_cancel_result(result, *operation_mgr, execution_id, "Cancel");
  if (!failure.has_value()) {
    return http::NoContent{};
  }
  json params{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}};
  // `return_code` only means something when the action server actually
  // answered; carrying a hardcoded 0 on the timeout / unavailable / guard
  // paths is exactly the ambiguity issue #576 is about.
  if (result.outcome == CancelOutcome::kErrorResponse) {
    params["return_code"] = result.return_code;
  }
  return tl::make_unexpected(
      make_error(failure->http_status, failure->error_code, failure->message, std::move(params)));
}

// =============================================================================
// PUT /{entity}/operations/{op_id}/executions/{exec_id} - update execution
// =============================================================================

http::Result<std::pair<dto::OperationExecution, http::ResponseAttachments>>
OperationHandlers::update_execution(const http::TypedRequest & req, const dto::ExecutionUpdateRequest & body) {
  using SuccessPair = std::pair<dto::OperationExecution, http::ResponseAttachments>;

  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto op_id_result = read_operation_id(req);
  if (!op_id_result) {
    return tl::make_unexpected(op_id_result.error());
  }
  const std::string operation_id = *op_id_result;

  auto exec_id_result = read_execution_id(req);
  if (!exec_id_result) {
    return tl::make_unexpected(exec_id_result.error());
  }
  const std::string execution_id = *exec_id_result;

  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                          json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  const auto entity_info = ctx_.get_entity_info(entity_id);
  if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "operations"); !lock_err) {
    return tl::make_unexpected(lock_err.error());
  }

  const std::string capability = body.capability;

  auto * operation_mgr = ctx_.node()->get_operation_manager();
  auto goal_info = operation_mgr->get_tracked_goal(execution_id);
  if (!goal_info.has_value()) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}}));
  }

  // SOVD capabilities: execute, freeze, reset, stop. ROS 2 actions only
  // implement stop (maps to cancel); the rest are 400 with an explicit
  // supported_capabilities hint.
  if (capability == "stop") {
    auto result = operation_mgr->cancel_action_goal(goal_info->action_path, execution_id);
    auto failure = detail::map_cancel_result(result, *operation_mgr, execution_id, "Stop");
    if (!failure.has_value()) {
      // The execution resource IS the request target for PUT, so echo the
      // requested path: the route is registered for apps, components, areas
      // and functions alike, and a hand-built apps/components pair sends an
      // areas client into the wrong collection.
      const std::string location(req.path());

      dto::OperationExecution exec_dto;
      exec_dto.id = execution_id;
      // Render the tracked status rather than assuming "running": the
      // reconcile set includes CANCELED, which GET reports as "failed", and
      // a 202 body must not contradict the resource Location points at.
      auto tracked = operation_mgr->get_tracked_goal(execution_id);
      exec_dto.status = tracked.has_value() ? sovd_status_from_ros2(tracked->status) : "running";

      http::ResponseAttachments att;
      att.with_status(202).with_header("Location", location);
      return SuccessPair{std::move(exec_dto), std::move(att)};
    }
    json params{{"entity_id", entity_id},
                {"operation_id", operation_id},
                {"execution_id", execution_id},
                {"capability", capability}};
    if (result.outcome == CancelOutcome::kErrorResponse) {
      params["return_code"] = result.return_code;
    }
    return tl::make_unexpected(
        make_error(failure->http_status, failure->error_code, failure->message, std::move(params)));
  }
  if (capability == "execute") {
    return tl::make_unexpected(
        make_error(409, ERR_INVALID_REQUEST,
                   "Cannot re-execute while operation is running. Cancel first, then start new execution.",
                   json{{"entity_id", entity_id},
                        {"operation_id", operation_id},
                        {"execution_id", execution_id},
                        {"capability", capability}}));
  }
  if (capability == "freeze" || capability == "reset") {
    return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Capability not supported for ROS 2 actions",
                                          json{{"entity_id", entity_id},
                                               {"operation_id", operation_id},
                                               {"execution_id", execution_id},
                                               {"capability", capability},
                                               {"supported_capabilities", json::array({"stop"})}}));
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unknown capability",
                                        json{{"entity_id", entity_id},
                                             {"operation_id", operation_id},
                                             {"execution_id", execution_id},
                                             {"capability", capability},
                                             {"supported_capabilities", json::array({"stop"})}}));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
