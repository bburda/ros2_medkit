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

#include "ros2_medkit_gateway/core/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/core/discovery/layers/manifest_layer.hpp"
#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/discovery/service_action_resolver.hpp"
#include "ros2_medkit_gateway/core/providers/host_info_provider.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"
#include "ros2_medkit_gateway/ros2/providers/ros2_runtime_introspection.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_serialization {
class TypeIntrospection;
}  // namespace ros2_medkit_serialization

namespace ros2_medkit_gateway {

// Forward declarations
class TopicDataProvider;
class IntrospectionProvider;

/**
 * @brief Configuration for discovery
 */
struct DiscoveryConfig {
  DiscoveryMode mode{DiscoveryMode::RUNTIME_ONLY};
  std::string manifest_path;
  bool manifest_strict_validation{true};
  bool manifest_enabled{true};  // enable/disable manifest layer in hybrid mode
  bool runtime_enabled{true};   // enable/disable runtime layer in hybrid mode
  /// Directory scanned for manifest fragment yaml files on every manifest
  /// load / reload. Empty = disabled. Fragments may contribute apps,
  /// components, and functions; top-level fields (areas, metadata, config,
  /// scripts, capabilities, lock_overrides) are reserved for the base
  /// manifest. See `ManifestManager::set_fragments_dir`.
  std::string manifest_fragments_dir;

  /// CSV file describing manually inventoried assets. When non-empty, each row
  /// becomes a Component (identity populated) appended to the manifest on every
  /// load / reload and merged into the tree by id. Empty = disabled. See
  /// `ManifestManager::set_inventory_csv_path`.
  std::string inventory_csv_path;

  /**
   * @brief Runtime (heuristic) discovery options
   *
   * These options control how the heuristic discovery strategy
   * maps ROS 2 graph entities to SOVD entities.
   *
   * Note: Synthetic/heuristic Area and Component creation has been removed.
   * Areas come from manifest only. Components come from HostInfoProvider
   * or manifest. Namespaces create Function entities.
   */
  struct RuntimeOptions {
    /**
     * @brief Create Function entities from ROS 2 namespace grouping
     *
     * When true (default), namespaces are mapped to Function entities.
     * Each Function hosts the Apps in that namespace.
     * This is the SOVD-correct mapping: namespaces represent
     * functional grouping, not deployment topology.
     */
    bool create_functions_from_namespaces{true};

    /**
     * @brief Create a default Component from HostInfoProvider
     *
     * When true (default), a single host-level Component is created
     * from system info (hostname, OS, arch) instead of synthetic
     * per-namespace Components. All discovered Apps are linked to
     * this Component via the is-located-on relationship.
     * Only used in runtime_only mode.
     */
    bool default_component_enabled{true};

    /**
     * @brief Filter ROS 2 internal nodes from entity discovery
     *
     * When true (default), apps whose ID starts with underscore (_) are
     * filtered out. These are ROS 2 infrastructure nodes like
     * _param_client_node that should not appear as SOVD entities.
     * Applies to both local and peer-discovered apps.
     */
    bool filter_internal_nodes{true};
  } runtime;

  /**
   * @brief Merge pipeline configuration (hybrid mode only)
   */
  struct MergePipelineConfig {
    discovery::GapFillConfig gap_fill;
    /// Per-layer merge policy overrides: layer_name -> (field_group_name -> policy_name)
    /// e.g. {"manifest": {"live_data": "authoritative"}, "runtime": {"identity": "authoritative"}}
    std::map<std::string, std::map<std::string, std::string>> layer_policies;
  } merge_pipeline;
};

/**
 * @brief Orchestrates entity discovery using a configurable merge pipeline
 *
 * The discovery_mode parameter selects which layers the merge pipeline
 * activates:
 * - RUNTIME_ONLY: ROS 2 graph introspection only (no merge pipeline; the
 *   built-in Ros2RuntimeIntrospection provider is queried directly)
 * - MANIFEST_ONLY: Manifest as the sole source of truth
 * - HYBRID: Manifest + runtime + plugin layers merged through the pipeline
 *
 * The DiscoveryManager provides a unified interface for discovering:
 * - Areas: Logical groupings (manifest-only)
 * - Components: Software/hardware units (HostInfoProvider, manifest, plugins)
 * - Apps: Software applications (manifest, runtime nodes, plugins)
 * - Functions: Functional groupings (manifest, namespaces, plugins)
 *
 * @see ros2::Ros2RuntimeIntrospection
 * @see discovery::MergePipeline
 * @see discovery::ManifestManager
 */
class DiscoveryManager : public ServiceActionResolver {
 public:
  /**
   * @brief Construct the discovery manager
   * @param node ROS 2 node for graph introspection (must outlive this manager)
   */
  explicit DiscoveryManager(rclcpp::Node * node);

  /**
   * @brief Initialize with configuration
   *
   * Loads manifest if configured, creates appropriate strategy.
   * For RUNTIME_ONLY mode, this is a no-op.
   *
   * @param config Discovery configuration
   * @return true if initialization succeeded
   */
  bool initialize(const DiscoveryConfig & config);

  // =========================================================================
  // Main discovery methods (delegate to strategy)
  // =========================================================================

  /**
   * @brief Discover all areas
   * @return Vector of discovered Area entities
   */
  std::vector<Area> discover_areas();

  /**
   * @brief Discover all components
   * @return Vector of discovered Component entities
   */
  std::vector<Component> discover_components();

  /**
   * @brief Discover all apps
   * @return Vector of discovered App entities (empty in runtime-only mode)
   */
  std::vector<App> discover_apps();

  /**
   * @brief Discover all functions
   * @return Vector of discovered Function entities (empty in runtime-only mode)
   */
  std::vector<Function> discover_functions();

  /**
   * @brief Discover functions using pre-discovered apps
   *
   * Avoids redundant ROS 2 graph introspection when apps have already been
   * discovered in the same refresh cycle. Falls back to the no-arg overload
   * for modes that don't support this optimization (manifest-only, hybrid).
   *
   * @param apps Pre-discovered apps (used only in RUNTIME_ONLY mode)
   * @return Vector of discovered Function entities
   */
  std::vector<Function> discover_functions(const std::vector<App> & apps);

  // =========================================================================
  // Entity lookup by ID
  // =========================================================================

  /**
   * @brief Get area by ID
   * @param id Area identifier
   * @return Area if found
   */
  std::optional<Area> get_area(const std::string & id);

  /**
   * @brief Get component by ID
   * @param id Component identifier
   * @return Component if found
   */
  std::optional<Component> get_component(const std::string & id);

  /**
   * @brief Get app by ID
   * @param id App identifier
   * @return App if found
   */
  std::optional<App> get_app(const std::string & id);

  /**
   * @brief Get function by ID
   * @param id Function identifier
   * @return Function if found
   */
  std::optional<Function> get_function(const std::string & id);

  // =========================================================================
  // Relationship queries
  // =========================================================================

  /**
   * @brief Get subareas of an area
   * @param area_id Parent area ID
   * @return Vector of child areas
   */
  std::vector<Area> get_subareas(const std::string & area_id);

  /**
   * @brief Get subcomponents of a component
   * @param component_id Parent component ID
   * @return Vector of child components
   */
  std::vector<Component> get_subcomponents(const std::string & component_id);

  /**
   * @brief Get components in an area
   * @param area_id Area ID
   * @return Vector of components in the area
   */
  std::vector<Component> get_components_for_area(const std::string & area_id);

  /**
   * @brief Get apps for a component
   * @param component_id Component ID
   * @return Vector of apps associated with the component
   */
  std::vector<App> get_apps_for_component(const std::string & component_id);

  /**
   * @brief Get host component IDs for a function
   * @param function_id Function ID
   * @return Vector of component IDs that host the function
   */
  std::vector<std::string> get_hosts_for_function(const std::string & function_id);

  // =========================================================================
  // Runtime-specific methods (delegate to runtime strategy)
  // =========================================================================

  /**
   * @brief Discover all services in the system
   * @return Vector of ServiceInfo with schema information
   */
  std::vector<ServiceInfo> discover_services();

  /**
   * @brief Discover all actions in the system
   * @return Vector of ActionInfo with schema information
   */
  std::vector<ActionInfo> discover_actions();

  /**
   * @brief Find a service by component namespace and operation name
   * @param component_ns Component namespace
   * @param operation_name Service name
   * @return ServiceInfo if found, nullopt otherwise
   */
  std::optional<ServiceInfo> find_service(const std::string & component_ns,
                                          const std::string & operation_name) const override;

  /**
   * @brief Find an action by component namespace and operation name
   * @param component_ns Component namespace
   * @param operation_name Action name
   * @return ActionInfo if found, nullopt otherwise
   */
  std::optional<ActionInfo> find_action(const std::string & component_ns,
                                        const std::string & operation_name) const override;

  /**
   * @brief Set the topic sampler for component-topic mapping
   * @param provider Pointer to TopicDataProvider (must outlive DiscoveryManager)
   */
  void set_topic_data_provider(TopicDataProvider * provider);

  /**
   * @brief Set the type introspection for operation schema enrichment
   * @param introspection Pointer to TypeIntrospection (must outlive DiscoveryManager)
   */
  void set_type_introspection(ros2_medkit_serialization::TypeIntrospection * introspection);

  /**
   * @brief Refresh the cached topic map
   */
  void refresh_topic_map();

  /**
   * @brief Check if topic map has been built at least once
   * @return true if topic map is ready, false if not yet built
   */
  bool is_topic_map_ready() const;

  // =========================================================================
  // Manifest management
  // =========================================================================

  /**
   * @brief Add a plugin layer to the merge pipeline
   *
   * Wraps an IntrospectionProvider as a PluginLayer and adds it to
   * the pipeline. Only works in HYBRID mode.
   *
   * @param plugin_name Name of the plugin
   * @param provider Non-owning pointer to IntrospectionProvider
   */
  void add_plugin_layer(const std::string & plugin_name, IntrospectionProvider * provider);

  /**
   * @brief Register an introspection provider with the merge pipeline
   *
   * Equivalent to add_plugin_layer for shared-ownership providers; the manager
   * keeps the provider alive for the lifetime of the pipeline. Only meaningful
   * in HYBRID mode.
   */
  void register_introspection_provider(const std::string & name, std::shared_ptr<IntrospectionProvider> provider);

  /**
   * @brief Re-execute the merge pipeline (hybrid mode only)
   *
   * Call after adding plugin layers to trigger a single pipeline refresh.
   */
  void refresh_pipeline();

  /**
   * @brief Get the manifest manager
   * @return Pointer to manifest manager (nullptr if not using manifest)
   */
  discovery::ManifestManager * get_manifest_manager();

  /**
   * @brief Get the discovery configuration the manifest asked for
   *
   * Answers in every mode. There is no manifest manager in runtime_only, and
   * a hybrid manifest that fails to load resets it, so this returns a
   * default-constructed ManifestConfig in those cases - the same values the
   * pipeline itself runs with.
   */
  discovery::ManifestConfig get_manifest_config() const;

  /**
   * @brief Re-read the manifest configuration into the running pipeline
   *
   * Call after a successful manifest reload. `/health` reads the config live
   * from the ManifestManager, but the pipeline captured it when it was built,
   * so without this the served `unmanifested_policy` and the policy the
   * orphan filter actually runs drift apart. Keeps the existing layers - in
   * particular plugin layers added at runtime - and only refreshes the config
   * and the manifest layer's policies.
   */
  void refresh_manifest_config();

  // =========================================================================
  // Status
  // =========================================================================

  /**
   * @brief Check if a host info provider is active
   * @return true if default component is enabled and provider exists
   */
  bool has_host_info_provider() const;

  /**
   * @brief Get the default Component from HostInfoProvider
   * @return Component entity representing the local host, or nullopt if not enabled
   */
  std::optional<Component> get_default_component() const;

  /**
   * @brief Get current discovery mode
   * @return Active discovery mode
   */
  DiscoveryMode get_mode() const {
    return config_.mode;
  }

  /**
   * @brief Get a human-readable name for the active discovery configuration.
   * @return One of "runtime", "manifest", "hybrid".
   */
  std::string get_strategy_name() const;

  /**
   * @brief Get the last merge pipeline report (hybrid mode only)
   * @return MergeReport if in hybrid mode, nullopt otherwise
   */
  std::optional<discovery::MergeReport> get_merge_report() const;

  /**
   * @brief Get the last linking result (hybrid mode only)
   * @return LinkingResult if in hybrid mode, nullopt otherwise
   */
  std::optional<discovery::LinkingResult> get_linking_result() const;

 private:
  /**
   * @brief Build the merge pipeline for the active discovery mode.
   *
   * No-op in RUNTIME_ONLY and MANIFEST_ONLY modes; in HYBRID mode wires
   * ManifestLayer / RuntimeLayer based on per-layer enable flags.
   */
  void build_pipeline();

  /**
   * @brief Re-execute the pipeline and cache the merged result.
   *
   * Only called in HYBRID mode. RUNTIME_ONLY and MANIFEST_ONLY modes serve
   * entities directly from the runtime introspection provider or the
   * manifest manager.
   */
  void run_pipeline();

  /**
   * @brief Apply user-configured merge policy overrides to a layer
   */
  template <typename LayerT>
  void apply_layer_policy_overrides(const std::string & layer_name, LayerT & layer);

  /// Apply the manifest layer's two policy sources in their required order:
  /// the `allow_manifest_override` blanket demotion first, then the per-group
  /// gateway parameters, so an explicit parameter keeps beating the flag.
  void apply_manifest_layer_policies(discovery::ManifestLayer & manifest_layer);

  rclcpp::Node * node_;
  DiscoveryConfig config_;

  // Host info provider (created when default_component_enabled is true)
  std::unique_ptr<HostInfoProvider> host_info_provider_;

  // ROS 2 graph adapter implementing IntrospectionProvider
  std::unique_ptr<ros2::Ros2RuntimeIntrospection> runtime_introspection_;
  std::unique_ptr<discovery::ManifestManager> manifest_manager_;

  // HYBRID mode pipeline + cached merged result.
  std::unique_ptr<discovery::MergePipeline> pipeline_;
  discovery::MergeResult cached_result_;
  mutable std::mutex result_mutex_;

  // Plugin providers registered through register_introspection_provider keep
  // the IntrospectionProvider alive for the lifetime of the pipeline.
  std::vector<std::shared_ptr<IntrospectionProvider>> owned_providers_;
};

}  // namespace ros2_medkit_gateway
