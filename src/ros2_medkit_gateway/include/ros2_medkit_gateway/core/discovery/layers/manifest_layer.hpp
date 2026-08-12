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
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"

#include <unordered_map>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Discovery layer wrapping ManifestManager
 *
 * Default policies: IDENTITY=AUTH, HIERARCHY=AUTH, LIVE_DATA=ENRICH,
 * STATUS=FALLBACK, METADATA=AUTH
 */
class ManifestLayer : public DiscoveryLayer {
 public:
  explicit ManifestLayer(ManifestManager * manifest_manager);

  std::string name() const override {
    return "manifest";
  }
  LayerOutput discover() override;
  MergePolicy policy_for(FieldGroup group) const override;

  void set_policy(FieldGroup group, MergePolicy policy);

  /// Restore the shipped policies. Needed on reload: a manifest that turns
  /// allow_manifest_override back on must undo the previous demotion.
  void reset_policies_to_defaults();

 private:
  ManifestManager * manifest_manager_;
  std::unordered_map<FieldGroup, MergePolicy> policies_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
