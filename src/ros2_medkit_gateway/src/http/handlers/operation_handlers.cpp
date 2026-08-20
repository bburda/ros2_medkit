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

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/member_qualified_id.hpp"
#include "ros2_medkit_gateway/core/http/operation_item_id.hpp"
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

/// True when `member_id` is one of the members that contributed to `ops`.
///
/// `source_ids` also carries the aggregating entity's own id, which is
/// harmless here: an id naming the entity itself resolves against exactly the
/// operations the entity contributed under its own name.
bool names_a_member(const AggregatedOperations & ops, const std::string & member_id) {
  return std::find(ops.source_ids.begin(), ops.source_ids.end(), member_id) != ops.source_ids.end();
}

/// The operation an item id resolves to inside one entity's operations.
struct ResolvedOperation {
  std::optional<ServiceInfo> service;
  std::optional<ActionInfo> action;

  bool found() const {
    return service.has_value() || action.has_value();
  }
};

/// Resolve `parsed` against the entity's operations.
///
/// A member half selects among same-named operations using the owner recorded
/// per full ROS path, and an item half that is a ROS path selects one outright.
/// A bare short name keeps the first match, which is the only thing it can mean
/// when it is unique and the only thing this gateway did before.
ResolvedOperation resolve_operation(const AggregatedOperations & ops, const http::MemberQualifiedId & parsed) {
  const auto owned_by_target = [&ops, &parsed](const std::string & full_path) {
    if (!parsed.has_member) {
      return true;
    }
    auto owner = ops.owner_by_path.find(full_path);
    return owner != ops.owner_by_path.end() && owner->second == parsed.member_id;
  };

  ResolvedOperation resolved;
  for (const auto & svc : ops.services) {
    if (http::operation_item_id_names(svc.name, svc.full_path, parsed.item_id) && owned_by_target(svc.full_path)) {
      resolved.service = svc;
      return resolved;
    }
  }
  for (const auto & act : ops.actions) {
    if (http::operation_item_id_names(act.name, act.full_path, parsed.item_id) && owned_by_target(act.full_path)) {
      resolved.action = act;
      return resolved;
    }
  }
  return resolved;
}

/// One operation `parsed` names: its ROS path, and the member that owns it if
/// the entity has members at all.
struct OperationMatch {
  std::string full_path;
  std::string member_id;  ///< empty when the entity exposes its operations directly
};

/// Everything `parsed` names, in collection order, using the same predicate
/// `resolve_operation` walks - so what is counted here is exactly what would
/// have been run. More than one match means the id does not identify an
/// operation, whether the extra copies belong to different members or to one
/// member that uses the same short name at two ROS paths.
std::vector<OperationMatch> matching_operations(const AggregatedOperations & ops,
                                                const http::MemberQualifiedId & parsed) {
  std::vector<OperationMatch> matches;
  const auto record = [&ops, &parsed, &matches](const std::string & full_path) {
    auto owner = ops.owner_by_path.find(full_path);
    const std::string member = owner != ops.owner_by_path.end() ? owner->second : std::string{};
    if (parsed.has_member && member != parsed.member_id) {
      return;
    }
    matches.push_back(OperationMatch{full_path, member});
  };
  for (const auto & svc : ops.services) {
    if (http::operation_item_id_names(svc.name, svc.full_path, parsed.item_id)) {
      record(svc.full_path);
    }
  }
  for (const auto & act : ops.actions) {
    if (http::operation_item_id_names(act.name, act.full_path, parsed.item_id)) {
      record(act.full_path);
    }
  }
  return matches;
}

/// The distinct members among `matches`, dropping the empty owner an entity
/// that exposes its own operations reports.
std::vector<std::string> distinct_members(const std::vector<OperationMatch> & matches) {
  std::vector<std::string> members;
  for (const auto & match : matches) {
    if (match.member_id.empty()) {
      continue;
    }
    if (std::find(members.begin(), members.end(), match.member_id) == members.end()) {
      members.push_back(match.member_id);
    }
  }
  return members;
}

/// The refusal `parsed` earns on this entity, or nothing when it names at most
/// one operation.
///
/// One construction for every route that resolves an operation id. A route that
/// accepted an id another route refuses would serve one of several operations
/// without ever saying which, and two separately built messages for the same
/// collision drift apart, so the caller is told a different remedy depending on
/// which verb it used.
///
/// `parameters.operation_ids` carries the ids that DO address what collided,
/// built by the rule the collection lists them under, so a caller sends one of
/// them back instead of deriving the form itself.
std::optional<ErrorInfo> refuse_if_ambiguous(const AggregatedOperations & ops, const http::MemberQualifiedId & parsed,
                                             const std::string & entity_id, const std::string & operation_id) {
  const std::vector<OperationMatch> matches = matching_operations(ops, parsed);
  if (matches.size() < 2) {
    return std::nullopt;
  }

  std::vector<std::string> paths;
  paths.reserve(matches.size());
  for (const auto & match : matches) {
    paths.push_back(match.full_path);
  }
  const std::vector<std::string> members = distinct_members(matches);
  json params{{"entity_id", entity_id}, {"operation_id", operation_id}, {"ros2_paths", paths}};
  if (!members.empty()) {
    params["member_ids"] = members;
  }

  const auto addressed_by_path = http::operation_paths_addressed_by_path(ops);
  std::vector<std::string> addressable;
  addressable.reserve(matches.size());
  for (const auto & match : matches) {
    std::string item = http::operation_item_half(parsed.item_id, match.full_path, addressed_by_path);
    if (ops.is_aggregated && !match.member_id.empty()) {
      item = http::make_member_qualified_id(match.member_id, item);
    }
    addressable.push_back(std::move(item));
  }
  params["operation_ids"] = addressable;

  if (members.size() > 1) {
    params["details"] = "Use format 'member_id:operation_id' to name the member that runs it";
    return make_error(400, ERR_INVALID_REQUEST, "Ambiguous operation id: more than one member provides it", params);
  }
  params["details"] =
      "One provider exposes this short name at more than one ROS path; address the one you mean by that "
      "path, without its leading slash";
  return make_error(400, ERR_INVALID_REQUEST, "Ambiguous operation id: it names more than one operation", params);
}

/// Settle which gateway holds the execution this request addresses.
///
/// An execution lives on the gateway that started it, which is the gateway that
/// owns the member behind the operation id - the same one POST was dispatched
/// to. The operation id is in the route, so the owner is resolvable by the rule
/// the executions collection already resolves it by, and the three follow-up
/// verbs reach the goals the collection hands out.
///
/// An id this gateway cannot resolve to exactly one owned operation is left to
/// the local path: the execution id is the key there, and the answer it gives
/// for an id naming no goal is the answer this route already owes. Resolving is
/// how the owner is found, never a second place for the route to refuse.
http::Result<MemberDispatch> dispatch_execution_to_owner(const HandlerContext & ctx, const http::TypedRequest & req,
                                                         const std::string & entity_id,
                                                         const std::string & operation_id,
                                                         const std::string & execution_id) {
  const auto entity_info = ctx.get_entity_info(entity_id);
  const auto & cache = ctx.node()->get_thread_safe_cache();
  auto lookup = resolve_entity_operations(cache, entity_info.sovd_type(), entity_id);
  if (!lookup) {
    return MemberDispatch::kServeLocally;
  }
  const auto & ops = lookup->ops;

  auto parsed = http::parse_member_qualified_id(operation_id, ops.is_aggregated);
  auto resolved = resolve_operation(ops, parsed);
  if (!resolved.found() || refuse_if_ambiguous(ops, parsed, entity_id, operation_id).has_value()) {
    return MemberDispatch::kServeLocally;
  }

  const std::string & full_path =
      resolved.service.has_value() ? resolved.service->full_path : resolved.action->full_path;
  auto owner = ops.owner_by_path.find(full_path);
  if (owner == ops.owner_by_path.end()) {
    return MemberDispatch::kServeLocally;
  }
  return ctx.dispatch_to_member(
      req, owner->second, "operations/" + parsed.item_id + "/executions/" + execution_id,
      json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}});
}

/// True for a member that is in the tree but whose gateway is silent.
///
/// A retained member is kept precisely so that the answer to a request does not
/// change when a link drops: the item is still addressable, and asking for it
/// says why it cannot be served right now. The alternatives are what this
/// replaces - quietly running a different member's operation, or a 200 with
/// nothing in it.
bool member_is_unreachable(const ThreadSafeEntityCache & cache, const std::string & member_id) {
  if (auto app = cache.get_app(member_id)) {
    return !app->available;
  }
  if (auto component = cache.get_component(member_id)) {
    return !component->available;
  }
  return false;
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
///
/// @param verb "Cancel" or "Stop" - keeps each entry point's message wording.
///   Every sentence this function builds is worded from `verb`: the same
///   mapping serves both routes, and a stop request answered with "the action
///   server did not answer the cancel request" names an operation the client
///   never issued. Where the underlying ROS 2 mechanism has to be named - a
///   stop IS carried out as an action cancel - it is named as the cause, after
///   the verb the client used, never in place of it.
std::optional<CancelFailure> map_cancel_result(const ActionCancelResult & result, OperationManager & operation_mgr,
                                               const std::string & execution_id, const char * verb) {
  // Derived rather than passed as a second parameter, so no call site can hand
  // in a verb and a noun that disagree.
  std::string lower_verb(verb);
  if (!lower_verb.empty()) {
    lower_verb[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(lower_verb[0])));
  }
  // "<Verb> failed: <cause>" - keeps the transport's diagnostic, which names
  // the ROS 2 entity that failed, while the sentence still opens on what the
  // client asked for. `fallback` is defensive: every producer of these outcomes
  // sets `error_message`, so it stands in only for a future one that forgets.
  auto attributed = [&result, verb](const char * fallback) {
    return std::string(verb) + " failed: " + (result.error_message.empty() ? fallback : result.error_message);
  };

  switch (result.outcome) {
    case CancelOutcome::kOk:
      return std::nullopt;
    case CancelOutcome::kTimeout: {
      auto tracked = operation_mgr.get_tracked_goal(execution_id);
      if (tracked.has_value() &&
          (tracked->status == ActionGoalStatus::CANCELING || tracked->status == ActionGoalStatus::CANCELED)) {
        return std::nullopt;
      }
      std::string message = std::string(verb) + " outcome unknown: the action server did not answer in time. ";
      // CANCELED already reconciled above, so a terminal status here means
      // the goal finished on its own. Telling the client to watch for
      // progress would describe something that cannot happen - say what the
      // gateway already knows instead, in the vocabulary the resource being
      // named actually answers in (`sovd_status_from_ros2`, not the raw ROS
      // word: the execution resource never emits "succeeded"/"aborted").
      if (tracked.has_value() &&
          (tracked->status == ActionGoalStatus::SUCCEEDED || tracked->status == ActionGoalStatus::ABORTED)) {
        message += "The execution status resource already reports the goal as " +
                   sovd_status_from_ros2(tracked->status) + ", so there is nothing left to " + lower_verb + ".";
      } else {
        // `status` alone cannot express the answer the client is asking for -
        // it renders CANCELED and ABORTED identically as "failed" - so name
        // the field that can.
        message += "Poll the execution status resource and read x-medkit.ros2_status to learn the goal's outcome.";
      }
      return CancelFailure{504, ERR_NOT_RESPONDING, std::move(message)};
    }
    case CancelOutcome::kServiceUnavailable:
      return CancelFailure{503, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                           attributed("the action server's cancel service is not available")};
    case CancelOutcome::kTransportError:
      return CancelFailure{500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                           attributed("the cancel request could not be completed")};
    case CancelOutcome::kNotTracked:
      return CancelFailure{404, ERR_RESOURCE_NOT_FOUND, "Execution not found"};
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
      // A code CancelGoal does not define. The transport's diagnostic quotes
      // the raw code and is the only thing that identifies it, so it is kept
      // as the cause - but it is worded around the action's own cancel, and on
      // the stop route that is not the operation the client issued.
      message = attributed("the action server refused it with a return code the protocol does not define");
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

  // How many operations the DECLARED tree carries under each short name. This
  // is the same count `create_execution` refuses on, and it is read here so the
  // listing and the execution cannot disagree: an id the tree calls ambiguous
  // is never offered bare, wherever the copy came from.
  std::unordered_map<std::string, size_t> declared_providers;
  for (const auto & svc : ops.services) {
    ++declared_providers[svc.name];
  }
  for (const auto & act : ops.actions) {
    ++declared_providers[act.name];
  }

  // The paths whose short name one provider carries twice. Their item half is
  // the path, because the member half they would otherwise be told apart by is
  // the same member for both.
  const std::unordered_set<std::string> path_addressed = http::operation_paths_addressed_by_path(ops);

  const auto contributed_by_peer = [&cache](const std::string & member_id) {
    static constexpr std::string_view kPeerPrefix = "peer:";
    if (auto app = cache.get_app(member_id)) {
      return app->source.rfind(kPeerPrefix, 0) == 0;
    }
    if (auto component = cache.get_component(member_id)) {
      return component->source.rfind(kPeerPrefix, 0) == 0;
    }
    return false;
  };
  const auto owner_is_remote = [&ops, &contributed_by_peer](const std::string & full_path) {
    auto owner = ops.owner_by_path.find(full_path);
    return owner != ops.owner_by_path.end() && contributed_by_peer(owner->second);
  };

  // The ROS path an item names, in the form an id carries it. Empty when the
  // item names no path, which is what a peer's malformed item looks like.
  const auto path_half_of = [](const dto::OperationItem & item) -> std::string {
    if (!item.x_medkit.has_value() || !item.x_medkit->ros2.has_value()) {
      return {};
    }
    const auto & ros2 = *item.x_medkit->ros2;
    return http::path_item_id(ros2.service.value_or(ros2.action.value_or(std::string{})));
  };

  // Qualify from the declared tree rather than by counting copies in this
  // response. A response can be short a copy - the caller suppressed fan-out,
  // or a peer did not answer - and counting copies would then hand back a bare
  // id that the execution refuses.
  //
  // The member half goes in front of whichever item half the id already
  // carries, short name or path, so an item this gateway addressed by path is
  // still addressed to the member that owns it.
  const auto qualify_from_declared_tree = [&declared_providers, &path_half_of](dto::OperationItem & item) {
    const std::string path_half = path_half_of(item);
    if (item.id != item.name && (path_half.empty() || item.id != path_half)) {
      return;  // already carries a member half, from us or from the peer that sent it
    }
    auto count = declared_providers.find(item.name);
    if (count == declared_providers.end() || count->second < 2) {
      return;
    }
    if (!item.x_medkit.has_value() || !item.x_medkit->member_ids.has_value() ||
        item.x_medkit->member_ids->size() != 1) {
      return;
    }
    item.id = http::make_member_qualified_id(item.x_medkit->member_ids->front(), item.id);
  };

  const auto build_item = [&](const auto & op, bool asynchronous) {
    dto::OperationItem item;
    item.id = http::operation_item_half(op.name, op.full_path, path_addressed);
    item.name = op.name;
    item.proximity_proof_required = false;
    item.asynchronous_execution = asynchronous;
    if constexpr (std::is_same_v<std::decay_t<decltype(op)>, ServiceInfo>) {
      item.x_medkit = build_service_xmedkit(op, entity_id, type_introspection);
    } else {
      item.x_medkit = build_action_xmedkit(op, entity_id, type_introspection);
    }
    if (auto owner = ops.owner_by_path.find(op.full_path); owner != ops.owner_by_path.end() && ops.is_aggregated) {
      item.x_medkit->member_ids = std::vector<std::string>{owner->second};
    }
    qualify_from_declared_tree(item);
    return item;
  };

  // A peer's operations are held here so ambiguity can be decided without
  // asking anyone. They are not reported from this walk while the peer is
  // reachable - the gateway that owns an operation is the one that reports it -
  // so they are set aside and only fall back into the list below, when the
  // fan-out that should have carried them did not.
  std::vector<dto::OperationItem> retained_from_peers;
  for (const auto & svc : ops.services) {
    auto item = build_item(svc, false);
    (owner_is_remote(svc.full_path) ? retained_from_peers : collection.items).push_back(std::move(item));
  }
  for (const auto & act : ops.actions) {
    auto item = build_item(act, true);
    (owner_is_remote(act.full_path) ? retained_from_peers : collection.items).push_back(std::move(item));
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
  // Two different reasons a peer's copy can be missing, and they are not the
  // same answer. The caller asking for no fan-out means the peers were never
  // consulted, and reporting their items anyway is what turns a bidirectionally
  // peered pair into a bounce. A fan-out that ran and came back without them
  // means the peer is not answering, and the tree still knows what it declared.
  const bool fan_out_suppressed = raw_req.has_header("X-Medkit-No-Fan-Out");
  auto fan_out = fan_out_collection<dto::OperationItem>(ctx_.aggregation_manager(), raw_req);
  std::unordered_set<std::string> paths_from_peers;
  for (auto & item : fan_out.items) {
    if (item.x_medkit.has_value() && item.x_medkit->ros2.has_value()) {
      const auto & ros2 = *item.x_medkit->ros2;
      auto path = ros2.service.value_or(ros2.action.value_or(std::string{}));
      if (!path.empty()) {
        paths_from_peers.insert(std::move(path));
      }
    }
    qualify_from_declared_tree(item);
    collection.items.push_back(std::move(item));
  }

  if (!fan_out_suppressed) {
    for (auto & item : retained_from_peers) {
      const auto & ros2 = *item.x_medkit->ros2;
      auto path = ros2.service.value_or(ros2.action.value_or(std::string{}));
      if (!path.empty() && paths_from_peers.count(path) > 0u) {
        continue;  // the owner answered for itself, which is the better copy
      }
      item.x_medkit->available = false;
      collection.items.push_back(std::move(item));
    }
  }

  // Catches ambiguity the declared tree has not seen: a copy that only reached
  // this gateway through the fan-out, from a member whose operations are not in
  // the local cache yet. An id only one item carries is left alone - it already
  // names one thing, and rewriting it would break every client sending it.
  http::qualify_ambiguous_ids(collection.items, [](const dto::OperationItem & item) {
    return item.x_medkit.has_value() && item.x_medkit->member_ids.has_value() ? &*item.x_medkit->member_ids : nullptr;
  });

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

  // A qualified id is accepted wherever the entity has members to name. A bare
  // id keeps resolving to the first match, so every client that sends the
  // short name - and the OpenAPI document this gateway generates - still work.
  auto parsed = http::parse_member_qualified_id(operation_id, ops.is_aggregated);
  if (parsed.has_member && !names_a_member(ops, parsed.member_id)) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Member not found in entity",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"member_id", parsed.member_id}}));
  }

  auto resolved = resolve_operation(ops, parsed);
  if (!resolved.found()) {
    return tl::make_unexpected(make_error(404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                          json{{"entity_id", entity_id}, {"operation_id", operation_id}}));
  }

  // A read refuses exactly what an execution refuses. Describing one of several
  // operations under an id that names them all tells the caller it holds an
  // address it does not hold, and the next thing it does with that id is run
  // it - where the same id is a 400. The collection no longer offers such an
  // id, so only a stale one arrives here, and it leaves with the ids that work.
  if (auto ambiguous = refuse_if_ambiguous(ops, parsed, entity_id, operation_id); ambiguous.has_value()) {
    return tl::make_unexpected(std::move(*ambiguous));
  }

  auto data_access_mgr = ctx_.node()->get_data_access_manager();
  auto type_introspection = data_access_mgr->get_type_introspection();

  dto::OperationDetail detail;
  // The id echoes what was requested, so a caller that took a qualified id out
  // of the collection sees the same id come back and can keep using it.
  detail.item.id = operation_id;
  if (resolved.service.has_value()) {
    detail.item.name = resolved.service->name;
    detail.item.proximity_proof_required = false;
    detail.item.asynchronous_execution = false;
    detail.item.x_medkit = build_service_xmedkit(*resolved.service, entity_id, type_introspection);
  } else {
    detail.item.name = resolved.action->name;
    detail.item.proximity_proof_required = false;
    detail.item.asynchronous_execution = true;
    detail.item.x_medkit = build_action_xmedkit(*resolved.action, entity_id, type_introspection);
  }

  // A retained member's operation is still described, because the description
  // is what was retained - but it says it cannot be served, so a client reading
  // the item on its own learns what the collection already told it, instead of
  // an absent field that means the opposite.
  const std::string & full_path =
      resolved.service.has_value() ? resolved.service->full_path : resolved.action->full_path;
  if (auto owner = ops.owner_by_path.find(full_path);
      owner != ops.owner_by_path.end() && member_is_unreachable(cache, owner->second)) {
    detail.item.x_medkit->available = false;
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

  auto parsed = http::parse_member_qualified_id(operation_id, ops.is_aggregated);
  if (parsed.has_member && !names_a_member(ops, parsed.member_id)) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Member not found in entity",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"member_id", parsed.member_id}}));
  }

  auto resolved = resolve_operation(ops, parsed);
  if (!resolved.found()) {
    return tl::make_unexpected(make_error(404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                          json{{"entity_id", entity_id}, {"operation_id", operation_id}}));
  }

  // An id that names more than one operation would run whichever was walked
  // first, and the caller would never learn which. An id that names a single
  // operation still executes, which is what every current client sends, and an
  // id that names nothing was already answered as not found above.
  //
  // The qualified form is checked too. Naming the member narrows the set, but
  // one member that uses the same short name at two ROS paths is still not
  // identified by it - there the item half has to be the ROS path, which is the
  // form the collection offers for exactly those copies.
  if (auto ambiguous = refuse_if_ambiguous(ops, parsed, entity_id, operation_id); ambiguous.has_value()) {
    return tl::make_unexpected(std::move(*ambiguous));
  }

  // Whoever ends up owning the resolved operation must be reachable, and must
  // run it themselves when they are another gateway. The service behind this id
  // lives on the owner's ROS graph; calling it from here finds nothing and
  // reports the member's own operation as unavailable.
  {
    const std::string & full_path =
        resolved.service.has_value() ? resolved.service->full_path : resolved.action->full_path;
    auto owner = ops.owner_by_path.find(full_path);
    if (owner != ops.owner_by_path.end()) {
      auto dispatch = ctx_.dispatch_to_member(req, owner->second, "operations/" + parsed.item_id + "/executions",
                                              json{{"entity_id", entity_id}, {"operation_id", operation_id}});
      if (!dispatch) {
        return tl::make_unexpected(dispatch.error());
      }
      if (*dispatch == MemberDispatch::kForwarded) {
        return tl::make_unexpected(HandlerContext::forwarded_sentinel_error());
      }
    }
  }

  const std::optional<ServiceInfo> & service_info = resolved.service;
  const std::optional<ActionInfo> & action_info = resolved.action;

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

      // The created execution is a sub-resource of the collection this POST
      // targeted, so append the new id to the request path. Hand-building the
      // prefix from an apps/components pair points areas and functions
      // clients - the route is registered for all four entity types - into
      // the components collection, and bypasses api_path() besides.
      const std::string location = req.path() + "/" + action_result.goal_id;

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

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Typed Collection<ExecutionId>; the wire shape is `{"items": [{"id": "..."}]}`
  // per JsonWriter<Collection<ExecutionId>>::write.
  dto::Collection<dto::ExecutionId> collection;

  // A plugin operation runs to completion inside execute_operation, so nothing
  // is ever tracked for it and the collection is empty by construction. The
  // provider still decides whether the operation exists at all, so a typo is a
  // miss here and not an empty success.
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
      return collection;
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

  // The executions of an operation are addressed by the id that addresses the
  // operation, resolved by the one rule the collection lists it under. Deriving
  // a ROS path by joining the entity's namespace to the id instead names a path
  // no member need have: an id can carry a member half or be a ROS path, and a
  // member's namespace is its own, not the aggregate's.
  auto parsed = http::parse_member_qualified_id(operation_id, ops.is_aggregated);
  if (parsed.has_member && !names_a_member(ops, parsed.member_id)) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Member not found in entity",
                   json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"member_id", parsed.member_id}}));
  }

  auto resolved = resolve_operation(ops, parsed);
  if (!resolved.found()) {
    return tl::make_unexpected(make_error(404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                          json{{"entity_id", entity_id}, {"operation_id", operation_id}}));
  }

  if (auto ambiguous = refuse_if_ambiguous(ops, parsed, entity_id, operation_id); ambiguous.has_value()) {
    return tl::make_unexpected(std::move(*ambiguous));
  }

  // The goals of an operation live on the gateway that sent them, which is the
  // one that owns the member - the same gateway POST reached to create them.
  // Answering from the local tracking map instead reports an empty collection
  // for goals that exist.
  const std::string & full_path =
      resolved.service.has_value() ? resolved.service->full_path : resolved.action->full_path;
  if (auto owner = ops.owner_by_path.find(full_path); owner != ops.owner_by_path.end()) {
    auto dispatch = ctx_.dispatch_to_member(req, owner->second, "operations/" + parsed.item_id + "/executions",
                                            json{{"entity_id", entity_id}, {"operation_id", operation_id}});
    if (!dispatch) {
      return tl::make_unexpected(dispatch.error());
    }
    if (*dispatch == MemberDispatch::kForwarded) {
      return tl::make_unexpected(HandlerContext::forwarded_sentinel_error());
    }
  }

  // Only an action produces an execution resource: a service call returns its
  // result inside the POST and leaves nothing to address afterwards. Its
  // executions collection therefore exists and is empty, which is a different
  // answer from the 404 an id naming no operation gets above - and the two have
  // to stay different, because a client reading a collection of executions
  // cannot otherwise tell a synchronous operation from a mistyped id.
  if (resolved.action.has_value()) {
    auto * operation_mgr = ctx_.node()->get_operation_manager();
    for (const auto & goal : operation_mgr->get_goals_for_action(resolved.action->full_path)) {
      dto::ExecutionId item;
      item.id = goal.goal_id;
      collection.items.push_back(std::move(item));
    }
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

  {
    auto dispatch = dispatch_execution_to_owner(ctx_, req, entity_id, operation_id, execution_id);
    if (!dispatch) {
      return tl::make_unexpected(dispatch.error());
    }
    if (*dispatch == MemberDispatch::kForwarded) {
      return tl::make_unexpected(HandlerContext::forwarded_sentinel_error());
    }
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

  {
    auto dispatch = dispatch_execution_to_owner(ctx_, req, entity_id, operation_id, execution_id);
    if (!dispatch) {
      return tl::make_unexpected(dispatch.error());
    }
    if (*dispatch == MemberDispatch::kForwarded) {
      return tl::make_unexpected(HandlerContext::forwarded_sentinel_error());
    }
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

  {
    auto dispatch = dispatch_execution_to_owner(ctx_, req, entity_id, operation_id, execution_id);
    if (!dispatch) {
      return tl::make_unexpected(dispatch.error());
    }
    if (*dispatch == MemberDispatch::kForwarded) {
      return tl::make_unexpected(HandlerContext::forwarded_sentinel_error());
    }
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
      const std::string & location = req.path();

      // Render the tracked status rather than assuming "running": the
      // reconcile set includes CANCELED, which GET reports as "failed", and
      // a 202 body must not contradict the resource Location points at.
      //
      // If the goal is gone by now - the cleanup timer can evict it between
      // the mapper's read and this one - then there is no status to render
      // and the Location we would hand out answers 404. Say that instead of
      // inventing "running", which would reproduce exactly the contradiction
      // this branch removed.
      auto tracked = operation_mgr->get_tracked_goal(execution_id);
      if (!tracked.has_value()) {
        return tl::make_unexpected(
            make_error(404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                       json{{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}}));
      }

      dto::OperationExecution exec_dto;
      exec_dto.id = execution_id;
      exec_dto.status = sovd_status_from_ros2(tracked->status);

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
    // `precondition-not-fulfilled` is the SOVD standard code for "prerequisites
    // not met" and is what the gateway already answers its lifecycle 409 with;
    // `invalid-request` is not in the standard code list at all.
    return tl::make_unexpected(
        make_error(409, ERR_PRECONDITION_NOT_FULFILLED,
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
