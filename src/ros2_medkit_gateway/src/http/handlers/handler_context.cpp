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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

#include <unordered_set>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/detail/forward_response_scope.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

tl::expected<void, std::string> HandlerContext::validate_entity_id(const std::string & entity_id) const {
  return ::ros2_medkit_gateway::validate_entity_id(entity_id);
}

tl::expected<std::string, std::string>
HandlerContext::get_component_namespace_path(const std::string & component_id) const {
  const auto & cache = node_->get_thread_safe_cache();
  auto component = cache.get_component(component_id);
  if (component) {
    return component->namespace_path;
  }
  return tl::unexpected("Component not found");
}

EntityInfo HandlerContext::get_entity_info(const std::string & entity_id, SovdEntityType expected_type) const {
  const auto & cache = node_->get_thread_safe_cache();
  EntityInfo info;
  info.id = entity_id;

  // Helper: check routing table for remote entity metadata
  auto apply_routing = [this](EntityInfo & ei) {
    if (aggregation_mgr_) {
      auto peer = aggregation_mgr_->find_peer_for_entity(ei.id);
      if (peer) {
        ei.is_remote = true;
        ei.peer_name = *peer;
        ei.peer_url = aggregation_mgr_->get_peer_url(*peer);
      }
    }
    // Check plugin ownership (only if not already a remote entity)
    if (!ei.is_remote) {
      auto * pmgr = node_->get_plugin_manager();
      if (pmgr) {
        auto owner = pmgr->get_entity_owner(ei.id);
        if (owner) {
          ei.is_plugin = true;
          ei.plugin_name = *owner;
        }
      }
    }
  };

  // If expected_type is specified, search ONLY in that collection
  // This prevents ID collisions (e.g., Area "powertrain" vs Component "powertrain")
  if (expected_type != SovdEntityType::UNKNOWN) {
    switch (expected_type) {
      case SovdEntityType::COMPONENT:
        if (auto component = cache.get_component(entity_id)) {
          info.type = EntityType::COMPONENT;
          info.namespace_path = component->namespace_path;
          info.fqn = component->fqn;
          info.id_field = "component_id";
          info.error_name = "Component";
          apply_routing(info);
          return info;
        }
        break;

      case SovdEntityType::APP:
        if (auto app = cache.get_app(entity_id)) {
          info.type = EntityType::APP;
          auto fqn = app->effective_fqn();
          info.namespace_path = fqn;
          info.fqn = fqn;
          info.id_field = "app_id";
          info.error_name = "App";
          apply_routing(info);
          return info;
        }
        break;

      case SovdEntityType::AREA:
        if (auto area = cache.get_area(entity_id)) {
          info.type = EntityType::AREA;
          info.namespace_path = area->namespace_path;
          info.fqn = area->namespace_path;
          info.id_field = "area_id";
          info.error_name = "Area";
          apply_routing(info);
          return info;
        }
        break;

      case SovdEntityType::FUNCTION:
        if (auto func = cache.get_function(entity_id)) {
          info.type = EntityType::FUNCTION;
          info.namespace_path = "";
          info.fqn = "";
          info.id_field = "function_id";
          info.error_name = "Function";
          apply_routing(info);
          return info;
        }
        break;

      case SovdEntityType::SERVER:
      case SovdEntityType::UNKNOWN:
      default:
        break;
    }
    // Not found in expected collection
    info.type = EntityType::UNKNOWN;
    info.error_name = "Entity";
    return info;
  }

  // No expected_type specified - search all collections in order (legacy behavior)
  // Search components first (O(1) lookup)
  if (auto component = cache.get_component(entity_id)) {
    info.type = EntityType::COMPONENT;
    info.namespace_path = component->namespace_path;
    info.fqn = component->fqn;
    info.id_field = "component_id";
    info.error_name = "Component";
    apply_routing(info);
    return info;
  }

  // Search apps (O(1) lookup)
  if (auto app = cache.get_app(entity_id)) {
    info.type = EntityType::APP;
    auto fqn = app->effective_fqn();
    info.namespace_path = fqn;
    info.fqn = fqn;
    info.id_field = "app_id";
    info.error_name = "App";
    apply_routing(info);
    return info;
  }

  // Search areas (O(1) lookup)
  if (auto area = cache.get_area(entity_id)) {
    info.type = EntityType::AREA;
    info.namespace_path = area->namespace_path;  // Use area's namespace for fault filtering
    info.fqn = area->namespace_path;
    info.id_field = "area_id";
    info.error_name = "Area";
    apply_routing(info);
    return info;
  }

  // Search functions (O(1) lookup)
  if (auto func = cache.get_function(entity_id)) {
    info.type = EntityType::FUNCTION;
    info.namespace_path = "";
    info.fqn = "";
    info.id_field = "function_id";
    info.error_name = "Function";
    apply_routing(info);
    return info;
  }

  // Not found - return UNKNOWN type
  info.type = EntityType::UNKNOWN;
  info.error_name = "Entity";
  return info;
}

tl::expected<void, ErrorInfo> HandlerContext::validate_collection_access_typed(const EntityInfo & entity,
                                                                               ResourceCollection collection) {
  auto caps = EntityCapabilities::for_type(entity.sovd_type());
  if (caps.supports_collection(collection)) {
    return {};
  }
  ErrorInfo err;
  err.code = ERR_COLLECTION_NOT_SUPPORTED;
  err.message = entity.error_name + " entities do not support " + to_string(collection) + " collection";
  err.http_status = 400;
  err.params = json{{"entity_id", entity.id}, {"collection", to_string(collection)}};
  return tl::unexpected(std::move(err));
}

tl::expected<void, ErrorInfo> HandlerContext::validate_lock_access(const http::TypedRequest & req,
                                                                   const EntityInfo & entity,
                                                                   const std::string & collection) {
  // Phase 1: If locking disabled, allow all access.
  if (!node_) {
    return {};
  }
  auto * lock_mgr = node_->get_lock_manager();
  if (!lock_mgr) {
    return {};
  }

  // Phase 2: Extract client_id from X-Client-Id header. TypedRequest::header
  // returns nullopt when the header is missing; LockManager::check_access
  // historically received the empty-string fall-back, so we preserve that.
  auto client_id_opt = req.header("X-Client-Id");
  std::string client_id = client_id_opt.value_or(std::string{});

  // Phase 3: Check lock access.
  auto result = lock_mgr->check_access(entity.id, client_id, collection);
  if (result.allowed) {
    return {};
  }

  ErrorInfo err;
  err.http_status = 409;
  err.message = result.denied_reason;
  if (result.denied_code == "lock-required") {
    err.code = ERR_INVALID_REQUEST;
    err.params = json{{"details", "A lock must be held to modify this resource collection"},
                      {"entity_id", entity.id},
                      {"collection", collection}};
  } else {
    err.code = ERR_LOCK_BROKEN;
    err.params = json{{"entity_id", entity.id}, {"collection", collection}};
    if (!result.denied_by_lock_id.empty()) {
      err.params["lock_id"] = result.denied_by_lock_id;
    }
  }
  return tl::unexpected(std::move(err));
}

namespace {

using http::detail::tl_forward_response;

}  // namespace

http::ValidatorResult<EntityInfo> HandlerContext::validate_entity_for_route(const http::TypedRequest & req,
                                                                            const std::string & entity_id) const {
  using ErrorVariant = std::variant<ErrorInfo, http::Forwarded>;

  // Step 1: Validate entity ID format.
  auto validation_result = validate_entity_id(entity_id);
  if (!validation_result) {
    ErrorInfo err;
    err.code = ERR_INVALID_PARAMETER;
    err.message = "Invalid entity ID";
    err.http_status = 400;
    err.params = json{{"details", validation_result.error()}, {"entity_id", entity_id}};
    return tl::unexpected(ErrorVariant{std::move(err)});
  }

  // Step 2: Get expected type from route path and look up entity. The raw
  // request is accessed through the framework-internal escape hatch on
  // TypedRequest because the path heuristic is part of the framework's
  // route-dispatch layer, not a handler concern. The deprecated-warning
  // suppression is intentional - the framework is the only legitimate caller
  // of raw_for_framework().
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto expected_type = extract_entity_type_from_path(raw_req.path);
  auto entity_info = get_entity_info(entity_id, expected_type);

  if (entity_info.type == EntityType::UNKNOWN) {
    // Step 3: Check if entity exists in ANY collection (for better error message).
    auto any_entity = get_entity_info(entity_id);
    ErrorInfo err;
    if (any_entity.type != EntityType::UNKNOWN) {
      // Entity exists but wrong type for this route -> 400.
      err.code = ERR_INVALID_PARAMETER;
      err.message = "Invalid entity type for route: expected " + to_string(expected_type) + ", got " +
                    to_string(any_entity.sovd_type());
      err.http_status = 400;
      err.params = json{{"entity_id", entity_id},
                        {"expected_type", to_string(expected_type)},
                        {"actual_type", to_string(any_entity.sovd_type())}};
    } else {
      // Entity doesn't exist at all -> 404.
      err.code = ERR_ENTITY_NOT_FOUND;
      err.message = "Entity not found";
      err.http_status = 404;
      err.params = json{{"entity_id", entity_id}};
    }
    return tl::unexpected(ErrorVariant{std::move(err)});
  }

  // Step 4: Forward to peer if entity is remote. This is the one place the
  // typed validator still mutates the underlying httplib::Response - peer
  // proxying needs to stream the response back, so the wire commit happens
  // inside the validator and the caller is signalled via Forwarded that no
  // further response work is allowed. The proxy response sink is supplied by
  // the framework via the thread-local set just before this call; when no
  // sink is set (e.g. a unit test that exercises the typed API without
  // aggregation) the forwarding path returns Forwarded without mutating any
  // wire - the framework guarantees a sink whenever aggregation is active.
  if (entity_info.is_remote && aggregation_mgr_) {
    if (tl_forward_response != nullptr) {
      aggregation_mgr_->forward_request(entity_info.peer_name, raw_req, *tl_forward_response);
    }
    return tl::unexpected(ErrorVariant{http::Forwarded{}});
  }

  return entity_info;
}

void HandlerContext::set_cors_headers(httplib::Response & res, const std::string & origin) const {
  res.set_header("Access-Control-Allow-Origin", origin);

  // Use pre-built header strings from CorsConfig
  if (!cors_config_.methods_header.empty()) {
    res.set_header("Access-Control-Allow-Methods", cors_config_.methods_header);
  }
  if (!cors_config_.headers_header.empty()) {
    res.set_header("Access-Control-Allow-Headers", cors_config_.headers_header);
  }

  // Expose headers that JavaScript needs access to (e.g., for file downloads)
  res.set_header("Access-Control-Expose-Headers", "Content-Disposition, Content-Length");

  // Set credentials header if enabled
  if (cors_config_.allow_credentials) {
    res.set_header("Access-Control-Allow-Credentials", "true");
  }
}

bool HandlerContext::is_origin_allowed(const std::string & origin) const {
  // Check if origin matches any allowed origin
  // Note: Wildcard "*" is allowed here but credentials+wildcard is blocked at startup
  // (see gateway_node.cpp validation). When wildcard is used, we echo back the actual
  // origin for security, as browsers require exact origin match with credentials.
  for (const auto & allowed : cors_config_.allowed_origins) {
    if (allowed == "*" || allowed == origin) {
      return true;
    }
  }
  return false;
}

std::set<std::string> HandlerContext::resolve_entity_source_fqns(const ThreadSafeEntityCache & cache,
                                                                 const EntityInfo & entity) {
  // Delegate to the neutral core helper so the HTTP fault handlers and the
  // ROS 2 plugin-context fault path share one entity -> source-FQN resolution.
  SovdEntityType type = SovdEntityType::UNKNOWN;
  switch (entity.type) {
    case EntityType::APP:
      type = SovdEntityType::APP;
      break;
    case EntityType::COMPONENT:
      type = SovdEntityType::COMPONENT;
      break;
    case EntityType::AREA:
      type = SovdEntityType::AREA;
      break;
    case EntityType::FUNCTION:
      type = SovdEntityType::FUNCTION;
      break;
    case EntityType::UNKNOWN:
      break;
  }
  return faults::resolve_entity_source_fqns(cache, type, entity.id);
}

std::vector<std::string> HandlerContext::resolve_app_host_fqns(const ThreadSafeEntityCache & cache,
                                                               const std::vector<std::string> & app_ids) {
  // Order-preserving dedup: two app_ids that resolve to the same effective_fqn
  // (e.g. manifest + runtime double-bind to the same node) must not produce
  // duplicate filters - downstream callers (LogManager::get_logs, bulkdata
  // source filters) would either query twice and merge, or dedup silently;
  // either way the duplicates are wasted CPU.
  std::vector<std::string> fqns;
  fqns.reserve(app_ids.size());
  std::unordered_set<std::string> seen;
  for (const auto & app_id : app_ids) {
    // Same app -> source resolution as the fault-scope path, so a plugin app
    // (external, no ROS binding) keeps its bare entity id here too. Resolving
    // by effective_fqn() alone dropped it, and the empty filter set made every
    // rosbag download and log query for its component match nothing.
    auto fqn = faults::resolve_app_source_fqn(cache, app_id);
    if (fqn.empty()) {
      continue;
    }
    if (seen.insert(fqn).second) {
      fqns.push_back(std::move(fqn));
    }
  }
  return fqns;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
