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

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD App entity - represents a software application
 *
 * In the ROS 2 context, an App typically corresponds to a ROS node.
 * Apps are located on Components and can depend on other Apps.
 *
 * Apps can be:
 * - Manifest-defined: Declared in YAML manifest with ROS binding
 * - Runtime-discovered: Automatically created from ROS nodes (future)
 */
struct App {
  // === Required fields ===
  std::string id;    ///< Unique identifier
  std::string name;  ///< Human-readable name

  // === Optional SOVD fields ===
  std::string translation_id;     ///< For i18n support
  std::string description;        ///< Detailed description
  std::vector<std::string> tags;  ///< Tags for filtering

  // === Relationships ===
  std::string component_id;             ///< is-located-on relationship
  std::vector<std::string> depends_on;  ///< depends-on relationship (App IDs)

  // === ROS binding (for manifest) ===
  /**
   * @brief ROS 2 binding configuration for manifest-defined apps
   *
   * Specifies how to bind this App to a ROS 2 node at runtime.
   */
  struct RosBinding {
    std::string node_name;          ///< ROS node name to bind to
    std::string namespace_pattern;  ///< Namespace (can be "*" for wildcard)
    std::string topic_namespace;    ///< Alternative: bind by topic prefix

    bool is_empty() const {
      return node_name.empty() && topic_namespace.empty();
    }

    json to_json() const {
      json j;
      if (!node_name.empty()) {
        j["nodeName"] = node_name;
        if (!namespace_pattern.empty()) {
          j["namespace"] = namespace_pattern;
        }
      }
      if (!topic_namespace.empty()) {
        j["topicNamespace"] = topic_namespace;
      }
      return j;
    }
  };
  std::optional<RosBinding> ros_binding;

  friend bool operator==(const RosBinding & a, const RosBinding & b) {
    return a.node_name == b.node_name && a.namespace_pattern == b.namespace_pattern &&
           a.topic_namespace == b.topic_namespace;
  }
  friend bool operator!=(const RosBinding & a, const RosBinding & b) {
    return !(a == b);
  }

  // === Runtime state (populated after linking) ===
  std::optional<std::string> bound_fqn;  ///< Bound ROS node FQN
  bool is_online{false};                 ///< Whether the bound node is running
  /// Tri-state classification: nullopt = no layer classified this app,
  /// true = not a ROS node (external asset), false = explicitly a ROS node.
  /// A plain bool cannot distinguish "omitted" from "declared false", which let
  /// a manifest stub erase a plugin's introspected classification (#517).
  std::optional<bool> external;

  /// Get effective FQN: bound_fqn if available, otherwise derived from ros_binding.
  /// Returns empty string if neither is available. Used by handlers and samplers
  /// to resolve the ROS node FQN in manifest_only mode where runtime linking
  /// doesn't set bound_fqn.
  /// @note namespace_pattern must be empty or an exact namespace path (e.g. "/perception").
  /// Glob patterns like "**" or "prefix*" are not supported and will produce empty FQN.
  std::string effective_fqn() const {
    if (bound_fqn.has_value() && !bound_fqn->empty()) {
      return *bound_fqn;
    }
    if (ros_binding.has_value() && !ros_binding->node_name.empty()) {
      const auto & ns = ros_binding->namespace_pattern;
      // Wildcard or glob patterns cannot produce valid FQNs
      if (ns.find('*') != std::string::npos) {
        return "";
      }
      if (ns.empty()) {
        return "/" + ros_binding->node_name;
      }
      if (ns.back() != '/') {
        return ns + "/" + ros_binding->node_name;
      }
      return ns + ros_binding->node_name;
    }
    return "";
  }

  // === Resources (populated from bound node) ===
  ComponentTopics topics;
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;

  // === Discovery metadata ===
  std::string source = "manifest";        ///< "manifest" or "runtime"
  std::string original_id;                ///< Pre-rename ID when collision-prefixed by aggregation
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

  // === Serialization methods ===

  /**
   * @brief Serialize to full JSON representation
   */
  json to_json() const;

  /**
   * @brief Create SOVD EntityReference format
   * @param base_url Base URL for href (e.g., "/api/v1")
   */
  json to_entity_reference(const std::string & base_url) const;

  /**
   * @brief Create SOVD Entity Capabilities format
   * @param base_url Base URL for capability URIs
   */
  json to_capabilities(const std::string & base_url) const;
};

inline bool operator==(const App & a, const App & b) {
  return a.id == b.id && a.name == b.name && a.translation_id == b.translation_id && a.description == b.description &&
         a.tags == b.tags && a.component_id == b.component_id && a.depends_on == b.depends_on &&
         a.ros_binding == b.ros_binding && a.bound_fqn == b.bound_fqn && a.is_online == b.is_online &&
         a.external == b.external && a.topics == b.topics && a.services == b.services && a.actions == b.actions &&
         a.source == b.source && a.original_id == b.original_id && a.contributors == b.contributors &&
         a.declared_source == b.declared_source && a.available == b.available;
}
inline bool operator!=(const App & a, const App & b) {
  return !(a == b);
}

}  // namespace ros2_medkit_gateway
