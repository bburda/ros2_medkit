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

#pragma once

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/plugins/entity_change_scope.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

class ResourceSamplerRegistry;

// Forward declarations for trigger-related types
class ResourceChangeNotifier;
class ConditionRegistry;

/**
 * @brief Entity information exposed to plugins
 */
struct PluginEntityInfo {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  std::string id;
  std::string namespace_path;  ///< ROS 2 namespace (for component fault filtering)
  std::string fqn;             ///< Fully qualified ROS 2 node name
};

/**
 * @brief Middleware-neutral context interface for plugins.
 *
 * Provides plugins read-only access to gateway state (entities, faults,
 * locks) plus mutation hooks (capabilities, samplers, trigger registries)
 * without exposing any ROS 2 type. ROS-specific access (the underlying
 * rclcpp::Node *) lives on RosPluginContext, the derived interface in
 * ros2_medkit_gateway/plugins/ros_plugin_context.hpp.
 *
 * @note This interface is versioned alongside PLUGIN_API_VERSION. New methods may
 * be added in future versions (entity data access, configuration queries, etc.).
 *
 * @par Thread Safety
 * All methods are safe to call from any thread. Entity and fault queries use
 * the gateway's thread-safe caches internally.
 */
class PluginContext {
 public:
  virtual ~PluginContext() = default;

  // ---- Entity access (read-only) ----

  /// Look up an entity by ID. Returns nullopt if not found.
  virtual std::optional<PluginEntityInfo> get_entity(const std::string & id) const = 0;

  /// Get all Apps belonging to a Component (for aggregation endpoints)
  virtual std::vector<PluginEntityInfo> get_child_apps(const std::string & component_id) const = 0;

  // ---- Fault access ----

  /// List faults for a given entity. Returns JSON array of fault objects.
  /// Empty array if entity has no faults or fault manager is unavailable.
  virtual nlohmann::json list_entity_faults(const std::string & entity_id) const = 0;

  // ---- HTTP handler utilities (for entity-scoped routes) ----

  /**
   * @brief Validate entity exists and matches route type, sending SOVD error if not
   *
   * Use this in get_routes() handlers to validate entity IDs from path params.
   * On failure, an appropriate SOVD GenericError response is sent automatically.
   *
   * @param req Plugin request (extracts expected entity type from path)
   * @param res Plugin response (error sent here on failure)
   * @param entity_id Entity ID from path parameter (e.g., req.path_param(1))
   * @return Entity info if valid, nullopt if error was sent
   */
  virtual std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & req, PluginResponse & res,
                                                                    const std::string & entity_id) const = 0;

  // ---- Capability registration ----

  /**
   * @brief Register a custom capability for all entities of a given type
   *
   * The capability will appear in the entity's capabilities array with an
   * auto-generated href. For example, registering "x-medkit-traces" for
   * SovdEntityType::APP produces: {"name": "x-medkit-traces", "href": "/api/v1/apps/{id}/x-medkit-traces"}
   *
   * The plugin must also return a matching route from get_routes().
   *
   * @param entity_type Entity type to add the capability to
   * @param capability_name Capability name (use x- prefix for vendor extensions)
   */
  virtual void register_capability(SovdEntityType entity_type, const std::string & capability_name) = 0;

  /**
   * @brief Register a custom capability for a specific entity
   *
   * Like register_capability(entity_type, name) but scoped to a single entity.
   *
   * @param entity_id Specific entity ID
   * @param capability_name Capability name (use x- prefix for vendor extensions)
   */
  virtual void register_entity_capability(const std::string & entity_id, const std::string & capability_name) = 0;

  // ---- Capability query (used by discovery handlers) ----

  /// Get plugin-registered capabilities for an entity type
  virtual std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const = 0;

  /// Get plugin-registered capabilities for a specific entity
  virtual std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const = 0;

  // ---- Locking ----

  /**
   * @brief Check if a lock blocks access to a collection on an entity.
   *
   * @param entity_id Entity to check
   * @param client_id Client requesting access
   * @param collection Resource collection being accessed (e.g. "configurations")
   * @return LockAccessResult with allowed/denied status and details
   */
  virtual LockAccessResult check_lock(const std::string & entity_id, const std::string & client_id,
                                      const std::string & collection) const = 0;

  /**
   * @brief Acquire a lock on an entity.
   *
   * @param entity_id Entity to lock
   * @param client_id Client acquiring the lock
   * @param scopes Optional lock scopes (empty = all collections)
   * @param expiration_seconds Lock TTL in seconds
   * @return LockInfo on success, LockError on failure
   */
  virtual tl::expected<LockInfo, LockError> acquire_lock(const std::string & entity_id, const std::string & client_id,
                                                         const std::vector<std::string> & scopes,
                                                         int expiration_seconds) = 0;

  /**
   * @brief Release a lock on an entity.
   *
   * @param entity_id Entity to unlock
   * @param client_id Client releasing the lock
   * @return void on success, LockError on failure
   */
  virtual tl::expected<void, LockError> release_lock(const std::string & entity_id, const std::string & client_id) = 0;

  // ---- Entity bulk access ----

  /// Get a snapshot of all discovered entities (areas, components, apps, functions).
  /// Returns an IntrospectionInput populated from the current entity cache.
  virtual IntrospectionInput get_entity_snapshot() const {
    return {};
  }

  // ---- All-faults access ----

  /// List all faults across all entities. Returns JSON with "faults" array.
  /// Empty object if fault manager is unavailable.
  virtual nlohmann::json list_all_faults() const {
    return nlohmann::json::object();
  }

  // ---- Resource sampler registration ----

  /// Register a cyclic subscription sampler for a custom collection.
  virtual void register_sampler(
      const std::string & /*collection*/,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &
      /*fn*/) {
  }

  // ---- Trigger infrastructure access ----

  /**
   * @brief Get the ResourceChangeNotifier for publishing or subscribing to resource changes.
   *
   * Always returns a valid pointer - ResourceChangeNotifier is created unconditionally
   * in GatewayNode regardless of trigger configuration.
   */
  virtual ResourceChangeNotifier * get_resource_change_notifier() = 0;

  /**
   * @brief Get the ConditionRegistry for registering custom trigger condition evaluators.
   *
   * Always returns a valid pointer - ConditionRegistry is created unconditionally
   * in GatewayNode regardless of trigger configuration.
   */
  virtual ConditionRegistry * get_condition_registry() = 0;

  // ---- Entity surface notifications (plugin API v7) ----

  /**
   * @brief Tell the gateway that this plugin has finished mutating entities.
   *
   * Call this AFTER any change that adds, removes, or restructures entities
   * on disk or in the running runtime (deploying a new node under a manifest
   * fragment, removing a previously-deployed app, renaming a component).
   * The gateway responds by re-reading its manifest sources (including any
   * configured fragments directory) and running a discovery pass for the
   * affected scope. By the time this returns, subsequent `get_entity` and
   * HTTP discovery endpoints reflect the new surface.
   *
   * @param scope Hint that lets the gateway limit the rediscovery work.
   *              `EntityChangeScope::full_refresh()` asks for a global pass.
   *
   * @par Thread safety
   * Safe to call from any thread. The v7 reference implementation is
   * SYNCHRONOUS: the caller's thread drives the full manifest reload +
   * discovery refresh before the call returns, under an internal refresh
   * mutex that also serializes the periodic refresh timer. As a result:
   *   * the call may block for seconds on large manifests / slow plugins;
   *   * plugins MUST NOT hold their own mutexes across this call if any
   *     of their own callbacks (introspect, data/operation/fault providers)
   *     could try to reacquire that mutex - doing so deadlocks;
   *   * calling this method from within your own `IntrospectionProvider::
   *     introspect()` callback is detected and skipped with a warning log
   *     (see `GatewayNode::handle_entity_change_notification`). The design
   *     assumes introspect already runs inside a refresh pass, so a nested
   *     notification would be redundant.
   *
   * @par Backwards compatibility
   * Default implementation is a no-op. Plugin source written against plugin
   * API v6 compiles unchanged against v7 headers (source-compatible) - no
   * code changes are required. The plugin loader compares the exported
   * `plugin_api_version()` against the gateway's `PLUGIN_API_VERSION` with
   * strict equality, so a plugin `.so` pre-compiled against v6 IS rejected;
   * recompilation against v7 headers is required.
   *
   * @par Fragment file contract (when used with discovery.manifest.fragments_dir)
   * Plugins that deploy / remove manifest fragments on disk before calling
   * this method MUST publish the final content atomically. The gateway's
   * fragment scanner reads each file on the caller's thread once the
   * notification arrives; a partially-written fragment causes the reload
   * to fail, which rolls back the entire manifest merge (see the design
   * doc's "all-or-nothing fragment contract" section). The recommended
   * pattern is: write to `fragments_dir/.tmp-<id>.yaml`, `fsync`, then
   * `rename()` (POSIX atomic within the same filesystem) to
   * `fragments_dir/<id>.yaml`. The rename is the commit point that makes
   * the fragment visible to the next reload.
   */
  virtual void notify_entities_changed(const EntityChangeScope & /*scope*/) {
  }
};

}  // namespace ros2_medkit_gateway
