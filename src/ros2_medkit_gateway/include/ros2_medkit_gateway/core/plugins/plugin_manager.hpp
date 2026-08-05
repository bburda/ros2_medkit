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

#include "ros2_medkit_gateway/core/openapi/route_descriptions.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_loader.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/providers/lifecycle_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider_registry.hpp"
#include "ros2_medkit_gateway/core/providers/operation_provider.hpp"
#include "ros2_medkit_gateway/core/providers/script_provider.hpp"
#include "ros2_medkit_gateway/core/providers/update_provider.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace httplib {
class Server;
}

namespace ros2_medkit_gateway {

/**
 * @brief Orchestrates loading, lifecycle, and dispatch of gateway plugins
 *
 * Owns all plugin instances and dlopen handles. Subsystem managers receive
 * non-owning pointers via get_update_provider() / get_introspection_providers().
 *
 * Provider discovery uses extern "C" query functions from loaded .so files
 * (avoiding RTTI across dlopen boundary). For compile-time plugins added
 * via add_plugin(), dynamic_cast is used (safe within the same binary).
 *
 * Error isolation: every call to plugin code is wrapped in try/catch.
 * A failing plugin is disabled but does not crash the gateway.
 *
 * IMPORTANT: PluginManager must outlive all subsystem managers that hold
 * non-owning provider pointers (e.g. UpdateManager). In GatewayNode, declare
 * plugin_mgr_ BEFORE update_mgr_ so that destruction order is safe.
 */
class PluginManager : public LogProviderRegistry {
 public:
  PluginManager() = default;
  ~PluginManager() override;

  // Non-copyable, non-movable (owns dlopen handles)
  PluginManager(const PluginManager &) = delete;
  PluginManager & operator=(const PluginManager &) = delete;
  PluginManager(PluginManager &&) = delete;
  PluginManager & operator=(PluginManager &&) = delete;

  /**
   * @brief Add a plugin directly (for testing with compile-time plugins)
   *
   * Uses dynamic_cast for provider discovery (safe within same binary).
   *
   * @param plugin Plugin instance
   */
  void add_plugin(std::unique_ptr<GatewayPlugin> plugin);

  /**
   * @brief Load plugins from shared library paths
   * @param configs Plugin configurations with paths and per-plugin config
   * @return Number of successfully loaded plugins
   */
  size_t load_plugins(const std::vector<PluginConfig> & configs);

  /**
   * @brief Configure all loaded plugins
   *
   * Calls configure() on each plugin with its per-plugin config.
   * Plugins that throw are disabled.
   */
  void configure_plugins();

  /**
   * @brief Set plugin context on all plugins
   *
   * Passes the gateway context (entity cache, faults, ROS 2 node, HTTP utils)
   * to each plugin via set_context(). Replaces the old set_node() method.
   *
   * @param context Plugin context (must outlive all plugins)
   */
  void set_context(PluginContext & context);

  /**
   * @brief Register custom REST routes from all plugins
   * @param server httplib::Server instance
   * @param api_prefix API path prefix (e.g., "/api/v1")
   */
  void register_routes(httplib::Server & server, const std::string & api_prefix);

  /// Register a custom transport provider
  void register_transport(std::unique_ptr<SubscriptionTransportProvider> provider);

  /// Set registries (called during gateway init)
  void set_registries(ResourceSamplerRegistry & samplers, TransportRegistry & transports);

  /**
   * @brief Shutdown all plugins
   */
  void shutdown_all();

  // ---- Dispatch to subsystem managers ----

  /**
   * @brief Get the update provider (first plugin implementing UpdateProvider)
   * @return Non-owning pointer, or nullptr if no UpdateProvider plugin loaded
   */
  UpdateProvider * get_update_provider() const;

  /**
   * @brief Get all introspection providers
   * @return Non-owning pointers to all IntrospectionProvider plugins
   */
  std::vector<IntrospectionProvider *> get_introspection_providers() const;

  /**
   * @brief Get the first plugin implementing LogProvider, or nullptr if none loaded
   */
  LogProvider * get_log_provider() const;

  /**
   * @brief Get the first plugin implementing ScriptProvider, or nullptr if none loaded
   */
  ScriptProvider * get_script_provider() const;

  /**
   * @brief Get all plugins implementing LogProvider (for observer notifications)
   */
  std::vector<LogProvider *> get_log_observers() const;

  // LogProviderRegistry implementation - thin forwards over the existing
  // get_log_provider / get_log_observers accessors. Keeps the LogManager
  // dependency expressed via the neutral port so the manager body links
  // against gateway_core alone.
  LogProvider * primary_log_provider() const override {
    return get_log_provider();
  }
  std::vector<LogProvider *> log_observers() const override {
    return get_log_observers();
  }

  /**
   * @brief Get all introspection providers with their plugin names
   * @return (plugin_name, provider) pairs for all IntrospectionProvider plugins
   */
  std::vector<std::pair<std::string, IntrospectionProvider *>> get_named_introspection_providers() const;

  /// Collect OpenAPI route descriptions from all loaded plugins.
  /// Uses dlsym to check for optional "describe_plugin_routes" export.
  std::vector<openapi::RouteDescriptions> collect_route_descriptions() const;

  // ---- Capability queries (used by discovery handlers) ----

  /// Get plugin context (for capability queries from discovery handlers)
  PluginContext * get_context() const {
    return context_;
  }

  // ---- Entity ownership (per-entity provider routing) ----

  /// Register entity ownership for a plugin.
  /// Called after IntrospectionProvider::introspect() returns new entities.
  /// Maps entity IDs to the plugin that created them, enabling per-entity
  /// provider routing in handlers.
  void register_entity_ownership(const std::string & plugin_name, const std::vector<std::string> & entity_ids);

  /// Clear all entity ownership entries for a given plugin.
  /// Called before re-registering during refresh to remove stale entries.
  void clear_entity_ownership(const std::string & plugin_name);

  /// Get DataProvider for a specific entity (if plugin-owned)
  /// @return Non-owning pointer, or nullptr if entity is not plugin-owned
  ///         or owning plugin doesn't implement DataProvider
  DataProvider * get_data_provider_for_entity(const std::string & entity_id) const;

  /// Get OperationProvider for a specific entity (if plugin-owned)
  /// @return Non-owning pointer, or nullptr if entity is not plugin-owned
  ///         or owning plugin doesn't implement OperationProvider
  OperationProvider * get_operation_provider_for_entity(const std::string & entity_id) const;

  /// Get LifecycleProvider for a specific entity (if plugin-owned)
  /// @return Non-owning pointer, or nullptr if entity is not plugin-owned
  ///         or owning plugin doesn't implement LifecycleProvider
  LifecycleProvider * get_lifecycle_provider_for_entity(const std::string & entity_id) const;

  /// Get FaultProvider for a specific entity (if plugin-owned)
  FaultProvider * get_fault_provider_for_entity(const std::string & entity_id) const;

  /**
   * @brief Fetch an entity's current data by dispatching the owning plugin's
   *        own `apps/<entity_id>/x-plc-data` GET route in-process.
   *
   * Fallback for plugins that serve live values only through registered HTTP
   * routes (the commercial PLC bridges) instead of a DataProvider. The route
   * table is matched exactly like the HTTP server would and the handler is
   * invoked with a synthesized request - no loopback connection.
   *
   * The handler runs on the caller's thread, possibly concurrently with HTTP
   * server threads serving the same route - plugin route handlers must be
   * thread-safe (already the contract across concurrent HTTP workers; this
   * entry point just adds one more caller).
   *
   * @param item optional data point name; when non-empty the single-item
   *        route (x-plc-data/<item>) is dispatched instead of the list.
   *
   * @return Parsed JSON body on a 200 response; nullopt when the entity is
   *         not plugin-owned, no route matches, or the handler fails.
   */
  std::optional<nlohmann::json> fetch_entity_data_via_route(const std::string & entity_id,
                                                            const std::string & item = "");

  /// Check if an entity is owned by a plugin
  /// @return Plugin name if owned, nullopt otherwise
  std::optional<std::string> get_entity_owner(const std::string & entity_id) const;

  // ---- Info ----
  bool has_plugins() const;
  std::vector<std::string> plugin_names() const;

 private:
  static void setup_plugin_logging(GatewayPlugin & plugin);

  struct LoadedPlugin {
    GatewayPluginLoadResult load_result;
    nlohmann::json config;
    UpdateProvider * update_provider = nullptr;
    IntrospectionProvider * introspection_provider = nullptr;
    LogProvider * log_provider = nullptr;
    ScriptProvider * script_provider = nullptr;
    DataProvider * data_provider = nullptr;
    OperationProvider * operation_provider = nullptr;
    LifecycleProvider * lifecycle_provider = nullptr;
    FaultProvider * fault_provider = nullptr;
    /// get_routes() result, fetched once per plugin (contract: called once);
    /// shared by register_routes() and fetch_entity_data_via_route().
    std::vector<GatewayPlugin::PluginRoute> routes;
    bool routes_cached = false;
    /// Resource sampler collections this plugin registered via
    /// PluginContext::register_sampler() during set_context(). Captured by
    /// diffing ResourceSamplerRegistry::collection_names() around that call
    /// (see capture_new_sampler_collections) so they can be removed when the
    /// plugin is torn down - the sampler's callable captures the plugin
    /// instance, so a dangling entry after teardown is a use-after-free.
    std::vector<std::string> sampler_collections;
  };

  /// Fetch and cache lp.routes on first use (unique lock must be held).
  /// A throwing get_routes() disables the plugin. @return false when disabled.
  bool cache_routes_locked(LoadedPlugin & lp);

  /// Disable a plugin after a lifecycle error (nulls providers, resets plugin).
  /// Also clears provider pointers in load_result to prevent dangling references.
  void disable_plugin(LoadedPlugin & lp);

  /// Record which sampler collections a plugin's set_context() call just
  /// added, by comparing the registry's collection set before that call
  /// (`before`) to its current set. Called immediately after set_context()
  /// returns or throws, so partially-completed registrations are still
  /// captured. No-op if no ResourceSamplerRegistry is configured.
  void capture_new_sampler_collections(LoadedPlugin & lp, const std::vector<std::string> & before);

  /// Remove every sampler collection tracked for a plugin (see
  /// LoadedPlugin::sampler_collections) from the ResourceSamplerRegistry, and
  /// clear the tracked list. No-op if no ResourceSamplerRegistry is configured.
  void remove_plugin_samplers(LoadedPlugin & lp);

  mutable std::shared_mutex plugins_mutex_;
  std::vector<LoadedPlugin> plugins_;
  PluginContext * context_ = nullptr;
  UpdateProvider * first_update_provider_ = nullptr;
  LogProvider * first_log_provider_ = nullptr;
  ScriptProvider * first_script_provider_ = nullptr;
  ResourceSamplerRegistry * sampler_registry_ = nullptr;
  TransportRegistry * transport_registry_ = nullptr;
  /// Entity ID -> plugin name mapping (populated from IntrospectionProvider results)
  std::unordered_map<std::string, std::string> entity_ownership_;
  bool shutdown_called_ = false;
};

}  // namespace ros2_medkit_gateway
