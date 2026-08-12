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
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include <string>
#include <unordered_map>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Discovery layer wrapping an IntrospectionProvider plugin
 *
 * Default policies: all ENRICHMENT (plugins enrich, they don't override)
 */
class PluginLayer : public DiscoveryLayer {
 public:
  PluginLayer(std::string plugin_name, IntrospectionProvider * provider);

  std::string name() const override {
    return name_;
  }

  /// A plugin declares entities the operator deliberately added; they are not
  /// runtime noise and the orphan sweep must not remove them, whatever
  /// `source` tag the provider chose to stamp on them.
  bool owns_protected_entities() const override {
    return true;
  }
  LayerOutput discover() override;
  MergePolicy policy_for(FieldGroup group) const override;
  void set_discovery_context(const IntrospectionInput & context) override;

  void set_policy(FieldGroup group, MergePolicy policy);

  /// Get per-entity metadata from last discover() call
  const std::unordered_map<std::string, nlohmann::json> & get_metadata() const {
    return last_metadata_;
  }

 private:
  std::string name_;
  IntrospectionProvider * provider_;
  rclcpp::Logger logger_;
  std::unordered_map<FieldGroup, MergePolicy> policies_;
  std::unordered_map<std::string, nlohmann::json> last_metadata_;
  IntrospectionInput discovery_context_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
