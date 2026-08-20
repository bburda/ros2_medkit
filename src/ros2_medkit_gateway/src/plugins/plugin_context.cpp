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

#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Map a SovdEntityType to the plural URL path segment used in the REST API.
/// Returns empty string for SERVER/UNKNOWN (no faults collection on those).
std::string entity_type_to_url_segment(SovdEntityType type) {
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
      return "";
  }
  return "";
}

}  // namespace

// ---- Concrete implementation ----

class GatewayPluginContext : public RosPluginContext {
 public:
  GatewayPluginContext(GatewayNode * node, FaultManager * fault_manager, ResourceSamplerRegistry * sampler_registry)
    : node_(node), fault_manager_(fault_manager), sampler_registry_(sampler_registry) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    const auto & cache = node_->get_thread_safe_cache();

    if (auto comp = cache.get_component(id)) {
      return PluginEntityInfo{SovdEntityType::COMPONENT, id, comp->namespace_path, comp->fqn};
    }
    if (auto app = cache.get_app(id)) {
      return PluginEntityInfo{SovdEntityType::APP, id, {}, app->effective_fqn()};
    }
    if (auto area = cache.get_area(id)) {
      return PluginEntityInfo{SovdEntityType::AREA, id, area->namespace_path, {}};
    }
    if (cache.get_function(id)) {
      return PluginEntityInfo{SovdEntityType::FUNCTION, id, {}, {}};
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & component_id) const override {
    std::vector<PluginEntityInfo> result;
    const auto & cache = node_->get_thread_safe_cache();
    auto app_ids = cache.get_apps_for_component(component_id);
    for (const auto & app_id : app_ids) {
      if (auto app = cache.get_app(app_id)) {
        result.push_back(PluginEntityInfo{SovdEntityType::APP, app_id,
                                          "",  // namespace_path not needed for Apps
                                          app->effective_fqn()});
      }
    }
    return result;
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    // Determine entity type (needed for both local query and peer path construction).
    auto entity = get_entity(entity_id);
    if (!entity) {
      return nlohmann::json::array();
    }

    nlohmann::json local_faults = nlohmann::json::array();

    if (fault_manager_ && fault_manager_->is_available()) {
      // Mirror the HTTP per-entity fault handler: fetch all faults, then scope
      // them to the entity's resolved App-FQN set (APP -> itself, COMPONENT ->
      // hosted apps, AREA -> descendant apps, FUNCTION -> host apps) via the
      // shared core helper. The fault manager returns a {"faults": [...]}
      // object; this method's contract is a bare array, so extract and filter
      // the inner array.
      auto result = fault_manager_->list_faults("");
      if (result.success && result.data.contains("faults") && result.data["faults"].is_array()) {
        const auto & cache = node_->get_thread_safe_cache();
        const auto source_fqns = faults::resolve_entity_source_fqns(cache, entity->type, entity_id);
        local_faults = faults::filter_faults_by_sources(result.data["faults"], source_fqns);
      }
    }

    // Fan-out to peer gateways: fetch faults for this entity from each peer.
    // The peer path mirrors the HTTP REST route: /api/v1/<type_plural>/<id>/faults.
    // Auth: the ROS 2 service path has no inbound HTTP Authorization header to
    // forward, so peers are queried with an empty credential - fan-out over this
    // path only surfaces faults from peers that do not gate their /faults route.
    auto * agg = node_->get_aggregation_manager();
    if (agg) {
      const std::string type_segment = entity_type_to_url_segment(entity->type);
      if (!type_segment.empty()) {
        const std::string peer_path = "/api/v1/" + type_segment + "/" + entity_id + "/faults";
        auto fan_result = agg->fan_out_get(peer_path, "");
        if (fan_result.merged_items.is_array()) {
          for (const auto & item : fan_result.merged_items) {
            local_faults.push_back(item);
          }
        }
        if (fan_result.is_partial) {
          RCLCPP_WARN(node_->get_logger(), "list_entity_faults('%s'): partial peer fan-out, %zu peer(s) failed",
                      entity_id.c_str(), fan_result.failed_peers.size());
        }
      }
    }

    return local_faults;
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & req, PluginResponse & res,
                                                            const std::string & entity_id) const override {
    // Validate entity ID format (shared validation with HandlerContext)
    if (auto err = validate_entity_id(entity_id); !err) {
      res.send_error(400, ERR_INVALID_PARAMETER, err.error());
      return std::nullopt;
    }

    // Determine expected type from route path (segment-boundary-aware matching)
    auto expected_type = extract_entity_type_from_path(req.path());

    auto entity = get_entity(entity_id);
    if (!entity) {
      res.send_error(404, ERR_ENTITY_NOT_FOUND, to_string(expected_type) + " not found: " + entity_id);
      return std::nullopt;
    }

    // Check type matches route
    if (expected_type != SovdEntityType::UNKNOWN && entity->type != expected_type) {
      res.send_error(400, ERR_INVALID_PARAMETER,
                     "Entity '" + entity_id + "' is a " + to_string(entity->type) + ", not a " +
                         to_string(expected_type));
      return std::nullopt;
    }

    return entity;
  }

  void register_capability(SovdEntityType entity_type, const std::string & capability_name) override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto & caps = type_capabilities_[entity_type];
    // Avoid duplicates
    for (const auto & c : caps) {
      if (c == capability_name) {
        return;
      }
    }
    caps.push_back(capability_name);
  }

  void register_entity_capability(const std::string & entity_id, const std::string & capability_name) override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto & caps = entity_capabilities_[entity_id];
    for (const auto & c : caps) {
      if (c == capability_name) {
        return;
      }
    }
    caps.push_back(capability_name);
  }

  std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto it = type_capabilities_.find(entity_type);
    if (it != type_capabilities_.end()) {
      return it->second;
    }
    return {};
  }

  std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto it = entity_capabilities_.find(entity_id);
    if (it != entity_capabilities_.end()) {
      return it->second;
    }
    return {};
  }

  LockAccessResult check_lock(const std::string & entity_id, const std::string & client_id,
                              const std::string & collection) const override {
    auto * lock_mgr = node_->get_lock_manager();
    if (!lock_mgr) {
      return LockAccessResult{true, "", "", ""};
    }
    return lock_mgr->check_access(entity_id, client_id, collection);
  }

  tl::expected<LockInfo, LockError> acquire_lock(const std::string & entity_id, const std::string & client_id,
                                                 const std::vector<std::string> & scopes,
                                                 int expiration_seconds) override {
    auto * lock_mgr = node_->get_lock_manager();
    if (!lock_mgr) {
      return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled", 503, std::nullopt});
    }
    return lock_mgr->acquire(entity_id, client_id, scopes, expiration_seconds);
  }

  tl::expected<void, LockError> release_lock(const std::string & entity_id, const std::string & client_id) override {
    auto * lock_mgr = node_->get_lock_manager();
    if (!lock_mgr) {
      return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled", 503, std::nullopt});
    }
    return lock_mgr->release(entity_id, client_id);
  }

  IntrospectionInput get_entity_snapshot() const override {
    IntrospectionInput input;
    const auto & cache = node_->get_thread_safe_cache();
    input.areas = cache.get_areas();
    input.components = cache.get_components();
    input.apps = cache.get_apps();
    input.functions = cache.get_functions();
    return input;
  }

  nlohmann::json list_all_faults() const override {
    nlohmann::json response = nlohmann::json::object();

    if (fault_manager_ && fault_manager_->is_available()) {
      auto result = fault_manager_->list_faults("");
      if (result.success) {
        response = result.data;
      }
    }

    // Fan-out to peer gateways: fetch all faults from /api/v1/faults on each peer.
    // Auth: see list_entity_faults - the ROS 2 service path forwards no inbound
    // Authorization header, so peers are queried with an empty credential.
    auto * agg = node_->get_aggregation_manager();
    if (agg) {
      auto fan_result = agg->fan_out_get("/api/v1/faults", "");
      if (fan_result.merged_items.is_array()) {
        // Ensure local response has a "faults" array to append to.
        if (!response.contains("faults") || !response["faults"].is_array()) {
          response["faults"] = nlohmann::json::array();
        }
        for (const auto & item : fan_result.merged_items) {
          response["faults"].push_back(item);
        }
      }
      if (fan_result.is_partial) {
        RCLCPP_WARN(node_->get_logger(), "list_all_faults: partial peer fan-out, %zu peer(s) failed",
                    fan_result.failed_peers.size());
      }
    }

    // "count" comes from the local-only query; after merging peer faults it
    // would undercount, so keep it consistent with the merged array.
    if (response.contains("count") && response.contains("faults") && response["faults"].is_array()) {
      response["count"] = response["faults"].size();
    }

    return response;
  }

  void register_sampler(
      const std::string & collection,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> & fn,
      bool honours_resource_path) override {
    if (sampler_registry_) {
      sampler_registry_->register_sampler(collection, fn, /*is_builtin=*/false, honours_resource_path);
    }
  }

  // ---- Trigger infrastructure ----

  ResourceChangeNotifier * get_resource_change_notifier() override {
    return node_->get_resource_change_notifier();
  }

  ConditionRegistry * get_condition_registry() override {
    return node_->get_condition_registry();
  }

  // ---- Entity surface notifications (plugin API v7) ----

  void notify_entities_changed(const EntityChangeScope & scope) override {
    // Forward the notification directly to GatewayNode for immediate
    // handling. This call is synchronous at this layer - the caller's
    // thread runs the full discovery refresh before returning. Any
    // serialization across concurrent plugin notifications is performed by
    // GatewayNode itself via its internal refresh mutex.
    node_->handle_entity_change_notification(scope);
  }

 private:
  GatewayNode * node_;
  FaultManager * fault_manager_;
  ResourceSamplerRegistry * sampler_registry_;

  mutable std::mutex capabilities_mutex_;
  std::unordered_map<SovdEntityType, std::vector<std::string>> type_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;
};

std::unique_ptr<RosPluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                              ResourceSamplerRegistry * sampler_registry) {
  return std::make_unique<GatewayPluginContext>(node, fault_manager, sampler_registry);
}

}  // namespace ros2_medkit_gateway
