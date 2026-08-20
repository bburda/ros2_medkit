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
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Area entity - represents a logical grouping (ROS 2 namespace)
 *
 * Areas are derived from ROS 2 namespaces or defined in manifests.
 * They provide a hierarchical organization of components.
 */
struct Area {
  std::string id;                         ///< Unique identifier (e.g., "powertrain")
  std::string name;                       ///< Human-readable name (e.g., "Powertrain System")
  std::string namespace_path;             ///< ROS 2 namespace path (e.g., "/powertrain")
  std::string type = "Area";              ///< Entity type (always "Area")
  std::string translation_id;             ///< Internationalization key
  std::string description;                ///< Human-readable description
  std::vector<std::string> tags;          ///< Tags for filtering
  std::string parent_area_id;             ///< Parent area ID for sub-areas
  std::string source;                     ///< Origin of this area (e.g., "manifest", "heuristic")
  std::vector<std::string> contributors;  ///< Aggregation provenance: "local" and/or "peer:<name>"
  /// What the contributing gateway itself called this entity: "manifest",
  /// "runtime", "node" or "topic". `source` is overwritten with `peer:<name>`
  /// on arrival, because the identity-merge precedence keys on it, so the
  /// origin would otherwise be lost - and the origin is what decides whether
  /// the entity is retained when its peer stops answering. Empty for entities
  /// this gateway discovered itself.
  std::string declared_source;

  /**
   * @brief Convert to JSON representation
   * @return JSON object with area data
   *
   * SOVD EntityReference fields: id, name, href, translation_id, tags
   * ROS 2 extensions in x-medkit: namespace, type, description, parentAreaId
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
    json x_medkit = {{"entityType", type}, {"namespace", namespace_path}};
    if (!description.empty()) {
      x_medkit["description"] = description;
    }
    if (!parent_area_id.empty()) {
      x_medkit["parentAreaId"] = parent_area_id;
    }
    if (!source.empty()) {
      x_medkit["source"] = source;
    }
    if (!contributors.empty()) {
      x_medkit["contributors"] = sorted_contributors(contributors);
    }
    j["x-medkit"] = x_medkit;

    return j;
  }

  /**
   * @brief Create SOVD EntityReference format (strictly compliant)
   * @param base_url Base URL for self links
   * @return JSON object in EntityReference format: id, name, href, [translationId, tags]
   */
  json to_entity_reference(const std::string & base_url) const {
    json j = {{"id", id}, {"href", base_url + "/areas/" + id}};
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
    std::string area_url = base_url + "/areas/" + id;

    // Capabilities response
    json j = {{"id", id}};
    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }

    // Capabilities as URI references (SOVD compliant)
    j["subareas"] = area_url + "/subareas";
    j["related-components"] = area_url + "/related-components";

    // x-medkit extension for ROS 2 specific info
    j["x-medkit"] = {{"entityType", type}, {"namespace", namespace_path}};

    return j;
  }
};

inline bool operator==(const Area & a, const Area & b) {
  return a.id == b.id && a.name == b.name && a.namespace_path == b.namespace_path && a.type == b.type &&
         a.translation_id == b.translation_id && a.description == b.description && a.tags == b.tags &&
         a.parent_area_id == b.parent_area_id && a.source == b.source && a.contributors == b.contributors &&
         a.declared_source == b.declared_source;
}
inline bool operator!=(const Area & a, const Area & b) {
  return !(a == b);
}

}  // namespace ros2_medkit_gateway
