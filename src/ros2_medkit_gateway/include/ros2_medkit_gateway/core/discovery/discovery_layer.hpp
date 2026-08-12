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

#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

// Forward declaration
struct IntrospectionInput;

namespace discovery {

/**
 * @brief Output produced by a discovery layer
 */
struct LayerOutput {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
  std::unordered_map<std::string, nlohmann::json> entity_metadata;  ///< entity id -> metadata
};

/**
 * @brief Interface for a pluggable discovery data source
 *
 * Each layer produces entities and declares how its data should be merged
 * with other layers via per-field-group MergePolicy.
 */
class DiscoveryLayer {
 public:
  virtual ~DiscoveryLayer() = default;

  /// Human-readable layer name (e.g., "manifest", "runtime", plugin name)
  virtual std::string name() const = 0;

  /// Discover entities from this layer's source
  virtual LayerOutput discover() = 0;

  /// Merge policy this layer uses for the given field group
  virtual MergePolicy policy_for(FieldGroup group) const = 0;

  /// Whether this layer provides runtime apps for post-merge linking.
  /// Only RuntimeLayer (or test doubles) should return true.
  /// True when entities this layer OWNS must survive orphan suppression.
  ///
  /// Protection used to be decided by matching `entity.source` against a list
  /// of known strings. That string is chosen by whoever produced the entity -
  /// a plugin provider may set anything, or nothing - so a provider that
  /// picked an unlisted tag (`beacon`, say) had its entities deleted. The
  /// decision belongs to the layer, which the pipeline knows for certain,
  /// rather than to a value the layer's data happens to carry.
  ///
  /// Declared here rather than answered with dynamic_cast to match how the
  /// pipeline already asks a layer what it is (see provides_runtime_apps).
  /// DiscoveryLayer is gateway-internal - plugins implement
  /// IntrospectionProvider, never this - so adding to it does not touch the
  /// plugin ABI.
  virtual bool owns_protected_entities() const {
    return false;
  }

  virtual bool provides_runtime_apps() const {
    return false;
  }

  /// Number of entities filtered by gap-fill in last discover(). Default 0.
  virtual size_t filtered_count() const {
    return 0;
  }

  /// Return unfiltered runtime apps for post-merge linking.
  /// Gap-fill may exclude apps from discover() output, but the linker needs
  /// all runtime apps to bind manifest apps to live nodes.
  /// Only RuntimeLayer (or test doubles) should override this.
  virtual std::vector<App> get_linking_apps() const {
    return {};
  }

  /// Provide the current discovery context (entities from previous layers).
  /// Called by MergePipeline before discover(). Default no-op.
  virtual void set_discovery_context(const IntrospectionInput & /*context*/) {
  }
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
