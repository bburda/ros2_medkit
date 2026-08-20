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

#include "ros2_medkit_gateway/core/discovery/models/asset_identity.hpp"
#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Component entity - represents a software/hardware component (ROS 2 node)
 *
 * Components are derived from ROS 2 nodes or defined in manifests.
 * They expose operations (services/actions), data (topics), and configurations (parameters).
 */
struct Component {
  std::string id;                         ///< Unique identifier (node name)
  std::string name;                       ///< Human-readable name
  std::string namespace_path;             ///< ROS 2 namespace path
  std::string fqn;                        ///< Fully qualified name (namespace + id)
  std::string type = "Component";         ///< Entity type (always "Component")
  std::string area;                       ///< Parent area ID
  std::string source = "node";            ///< Discovery source: "node", "topic", or "manifest"
  std::string translation_id;             ///< Internationalization key
  std::string description;                ///< Human-readable description
  std::string variant;                    ///< Hardware variant identifier
  std::vector<std::string> tags;          ///< Tags for filtering
  std::string parent_component_id;        ///< Parent component ID for sub-components
  std::vector<std::string> depends_on;    ///< Component IDs this component depends on
  std::vector<std::string> contributors;  ///< Aggregation provenance: "local" and/or "peer:<name>"
  /// What the contributing gateway itself called this entity: "manifest",
  /// "runtime", "node" or "topic". `source` is overwritten with `peer:<name>`
  /// on arrival, because the identity-merge precedence keys on it, so the
  /// origin would otherwise be lost - and the origin is what decides whether
  /// the entity is retained when its peer stops answering. Empty for entities
  /// this gateway discovered itself.
  std::string declared_source;
  /// False while the peer contributing this entity is not answering and the
  /// entity is being retained from its last known declaration. A retained
  /// entity stays addressable and reports why it cannot be reached.
  bool available{true};
  std::vector<ServiceInfo> services;  ///< Services exposed by this component
  std::vector<ActionInfo> actions;    ///< Actions exposed by this component
  ComponentTopics topics;             ///< Topics this component publishes/subscribes
  std::optional<json> host_metadata;  ///< Host system metadata (for runtime default component)
  AssetIdentity identity;             ///< Asset-identity nameplate (merged across sources, per-field provenance)

  /// Tri-state: nullopt = no layer classified this component, true = non-ROS
  /// external asset (PLC/fieldbus/device), false = explicitly a ROS component.
  /// Mirrors App::external so a device Component owns its fault_manager faults
  /// without a synthetic child App (#516).
  std::optional<bool> external;

  /**
   * @brief Convert to JSON representation
   * @return JSON object with component data
   *
   * SOVD EntityReference fields: id, name, href, translation_id, tags
   * ROS 2 extensions in x-medkit: namespace, fqn, entityType, area, source, variant, etc.
   */
  json to_json() const {
    // Base fields
    json j = {{"id", id}};

    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }
    if (!tags.empty()) {
      j["tags"] = tags;
    }

    // ROS 2 extensions in x-medkit (SOVD vendor extension)
    json x_medkit = {
        {"entityType", type}, {"namespace", namespace_path}, {"fqn", fqn}, {"area", area}, {"source", source}};
    if (!description.empty()) {
      x_medkit["description"] = description;
    }
    if (!variant.empty()) {
      x_medkit["variant"] = variant;
    }
    if (!parent_component_id.empty()) {
      x_medkit["parentComponentId"] = parent_component_id;
    }
    if (!depends_on.empty()) {
      x_medkit["dependsOn"] = depends_on;
    }
    if (!contributors.empty()) {
      x_medkit["contributors"] = sorted_contributors(contributors);
    }
    x_medkit["topics"] = topics.to_json();
    if (host_metadata.has_value()) {
      x_medkit["host"] = host_metadata.value();
    }
    if (!identity.empty()) {
      x_medkit["identity"] = identity.to_json();
    }
    // Emit only when effectively external; omitted and explicit-false both leave
    // the field absent, keeping the wire shape stable (absence == not external).
    if (external.value_or(false)) {
      x_medkit["external"] = true;
    }
    j["x-medkit"] = x_medkit;

    // Add operations array combining services and actions
    json operations = json::array();
    for (const auto & svc : services) {
      operations.push_back(svc.to_json());
    }
    for (const auto & act : actions) {
      operations.push_back(act.to_json());
    }
    if (!operations.empty()) {
      j["operations"] = operations;
    }

    return j;
  }

  /**
   * @brief Create SOVD EntityReference format (strictly compliant)
   * @param base_url Base URL for self links
   * @return JSON object in EntityReference format: id, name, href, [translationId, tags]
   */
  json to_entity_reference(const std::string & base_url) const {
    json j = {{"id", id}, {"href", base_url + "/components/" + id}};
    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }
    if (!tags.empty()) {
      j["tags"] = tags;
    }
    return j;
  }

  /**
   * @brief Create SOVD Entity Capabilities format (strictly compliant)
   * @param base_url Base URL for capability links
   * @return JSON object listing available sub-resources
   */
  json to_capabilities(const std::string & base_url) const {
    std::string component_url = base_url + "/components/" + id;

    // Capabilities response
    json j = {{"id", id}};
    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }

    // Capabilities as URI references (SOVD compliant)
    if (!topics.publishes.empty() || !topics.subscribes.empty()) {
      j["data"] = component_url + "/data";
    }
    if (!services.empty() || !actions.empty()) {
      j["operations"] = component_url + "/operations";
    }
    // ROS 2 nodes always have parameters
    if (source == "node") {
      j["configurations"] = component_url + "/configurations";
    }
    j["faults"] = component_url + "/faults";
    j["subcomponents"] = component_url + "/subcomponents";
    j["related-apps"] = component_url + "/related-apps";
    if (!depends_on.empty()) {
      j["depends-on"] = component_url + "/depends-on";
    }

    // x-medkit extension for ROS 2 specific info
    j["x-medkit"] = {{"entityType", type}, {"namespace", namespace_path}, {"fqn", fqn}, {"source", source}};

    return j;
  }
};

inline bool operator==(const Component & a, const Component & b) {
  return a.id == b.id && a.name == b.name && a.namespace_path == b.namespace_path && a.fqn == b.fqn &&
         a.type == b.type && a.area == b.area && a.source == b.source && a.translation_id == b.translation_id &&
         a.description == b.description && a.variant == b.variant && a.tags == b.tags &&
         a.parent_component_id == b.parent_component_id && a.depends_on == b.depends_on &&
         a.contributors == b.contributors && a.services == b.services && a.actions == b.actions &&
         a.topics == b.topics && a.host_metadata == b.host_metadata && a.identity == b.identity &&
         a.external == b.external && a.declared_source == b.declared_source && a.available == b.available;
}
inline bool operator!=(const Component & a, const Component & b) {
  return !(a == b);
}

}  // namespace ros2_medkit_gateway
