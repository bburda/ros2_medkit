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

#include "ros2_medkit_gateway/core/discovery/layers/manifest_layer.hpp"

namespace ros2_medkit_gateway {
namespace discovery {

ManifestLayer::ManifestLayer(ManifestManager * manifest_manager) : manifest_manager_(manifest_manager) {
  reset_policies_to_defaults();
}

void ManifestLayer::reset_policies_to_defaults() {
  policies_ = {{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
               {FieldGroup::HIERARCHY, MergePolicy::AUTHORITATIVE},
               {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
               {FieldGroup::STATUS, MergePolicy::FALLBACK},
               {FieldGroup::METADATA, MergePolicy::AUTHORITATIVE}};
}

LayerOutput ManifestLayer::discover() {
  LayerOutput output;
  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    return output;
  }
  output.areas = manifest_manager_->get_areas();
  output.components = manifest_manager_->get_components();
  output.apps = manifest_manager_->get_apps();
  output.functions = manifest_manager_->get_functions();
  return output;
}

MergePolicy ManifestLayer::policy_for(FieldGroup group) const {
  auto it = policies_.find(group);
  if (it != policies_.end()) {
    return it->second;
  }
  return MergePolicy::ENRICHMENT;
}

void ManifestLayer::set_policy(FieldGroup group, MergePolicy policy) {
  policies_[group] = policy;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
