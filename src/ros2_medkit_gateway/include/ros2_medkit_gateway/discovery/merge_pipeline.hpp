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

#include "ros2_medkit_gateway/core/discovery/discovery_layer.hpp"
#include "ros2_medkit_gateway/core/discovery/identity_merge.hpp"
#include "ros2_medkit_gateway/core/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Result of a merge pipeline execution
 */
struct MergeResult {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
  MergeReport report;
  LinkingResult linking_result;
};

/**
 * @brief Orchestrates multiple discovery layers with configurable merge policies
 *
 * Layers are added in priority order (first added = highest priority).
 * Each layer produces entities and declares per-field-group MergePolicy.
 * The pipeline merges entities by ID, resolving conflicts per policy.
 */
class MergePipeline {
 public:
  explicit MergePipeline(rclcpp::Logger logger = rclcpp::get_logger("merge_pipeline"));

  /**
   * @brief Add a discovery layer to the pipeline
   * @param layer Layer to add (priority = insertion order, first = highest)
   */
  void add_layer(std::unique_ptr<DiscoveryLayer> layer);

  /**
   * @brief Set the asset-identity merge configuration (source precedence).
   *
   * Identity authority is independent of the structural MergePolicy: this controls
   * which source wins per identity field, ranked on `Component.source`.
   */
  void set_identity_merge_config(IdentityMergeConfig config) {
    identity_config_ = std::move(config);
  }

  /**
   * @brief Execute all layers and merge results
   * @return Merged entities with diagnostics report
   */
  MergeResult execute();

  /**
   * @brief Get the last merge report (returned by value for thread safety)
   * @warning NOT thread-safe on its own. In production, access only through
   * DiscoveryManager which holds a mutex around the cached pipeline result.
   * Direct calls from multiple threads are a data race.
   */
  MergeReport get_last_report() const {
    return last_report_;
  }

  /**
   * @brief Set RuntimeLinker for post-merge app-to-node binding
   * @param linker RuntimeLinker instance
   * @param config ManifestConfig needed by RuntimeLinker::link() for unmanifested node policy
   */
  void set_linker(std::unique_ptr<RuntimeLinker> linker, const ManifestConfig & config);

  /**
   * @brief Replace the ManifestConfig the pipeline runs on, keeping the linker
   *
   * Used after a manifest reload. The config reached the pipeline once, at
   * construction, while `/health` reads it live from the ManifestManager - so
   * without this the two disagree after a reload and the documented
   * `orphan_count > 0 && unmanifested_policy == "error"` recipe reports a
   * policy the orphan filter is not running.
   *
   * Deliberately not "rebuild the pipeline": plugin layers are added at
   * runtime through DiscoveryManager::add_plugin_layer and a rebuild would
   * drop them.
   */
  void set_manifest_config(const ManifestConfig & config) {
    manifest_config_ = config;
  }

  /**
   * @brief Find a layer by name, or nullptr. Used to re-apply layer policies
   *        after a reload without disturbing layer order or membership.
   */
  DiscoveryLayer * find_layer(const std::string & name);

  /**
   * @brief Get the last linking result (returned by value for thread safety)
   * @warning NOT thread-safe on its own. In production, access only through
   * DiscoveryManager which holds a mutex around the cached pipeline result.
   */
  LinkingResult get_linking_result() const {
    return linking_result_;
  }

 private:
  /// Merge a vector of entities from multiple layers by ID
  template <typename Entity>
  std::vector<Entity> merge_entities(std::vector<std::pair<size_t, std::vector<Entity>>> & layer_entities,
                                     MergeReport & report);

  rclcpp::Logger logger_;
  std::vector<std::unique_ptr<DiscoveryLayer>> layers_;
  IdentityMergeConfig identity_config_;
  MergeReport last_report_;
  std::unique_ptr<RuntimeLinker> linker_;

  /// Ids of entities whose OWNING layer declares them protected from orphan
  /// suppression (see DiscoveryLayer::owns_protected_entities). Rebuilt from
  /// scratch on every execute() - a layer that stops contributing an entity
  /// must stop protecting it.
  std::unordered_set<std::string> protected_entity_ids_;
  ManifestConfig manifest_config_;
  LinkingResult linking_result_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
