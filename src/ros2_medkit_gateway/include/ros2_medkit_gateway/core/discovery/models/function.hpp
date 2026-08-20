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
 * @brief SOVD Function entity - represents a functional group
 *
 * Functions are higher-level abstractions that group Apps and/or Components
 * representing a complete capability (e.g., "Autonomous Navigation").
 *
 * Functions can be manifest-defined or created at runtime from namespace grouping.
 * They aggregate data, operations, and faults from their hosted entities.
 */
struct Function {
  // === Required fields ===
  std::string id;    ///< Unique identifier
  std::string name;  ///< Human-readable name

  // === Optional SOVD fields ===
  std::string translation_id;     ///< For i18n support
  std::string description;        ///< Detailed description
  std::vector<std::string> tags;  ///< Tags for filtering

  // === Relationships ===
  std::vector<std::string> hosts;       ///< App or Component IDs this function hosts
  std::vector<std::string> depends_on;  ///< depends-on relationship (Function IDs)

  // === Discovery metadata ===
  std::string source = "manifest";        ///< Discovery source: manifest or runtime
  std::vector<std::string> contributors;  ///< Aggregation provenance: "local" and/or "peer:<name>"
  /// What the contributing gateway itself called this entity: "manifest",
  /// "runtime", "node" or "topic". `source` is overwritten with `peer:<name>`
  /// on arrival, because the identity-merge precedence keys on it, so the
  /// origin would otherwise be lost - and the origin is what decides whether
  /// the entity is retained when its peer stops answering. Empty for entities
  /// this gateway discovered itself.
  std::string declared_source;

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

inline bool operator==(const Function & a, const Function & b) {
  return a.id == b.id && a.name == b.name && a.translation_id == b.translation_id && a.description == b.description &&
         a.tags == b.tags && a.hosts == b.hosts && a.depends_on == b.depends_on && a.source == b.source &&
         a.contributors == b.contributors && a.declared_source == b.declared_source;
}
inline bool operator!=(const Function & a, const Function & b) {
  return !(a == b);
}

}  // namespace ros2_medkit_gateway
