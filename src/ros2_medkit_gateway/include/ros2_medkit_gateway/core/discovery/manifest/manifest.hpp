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

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/script_types.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

using json = nlohmann::json;

/**
 * @brief Configuration for discovery behavior when manifest is present
 */
struct ManifestConfig {
  /**
   * @brief Policy for ROS nodes not declared in manifest
   */
  enum class UnmanifestedNodePolicy {
    IGNORE,  ///< Don't expose unmanifested nodes
    WARN,    ///< Log warning, include as orphan
    /// Same entity tree as WARN. Logs at error level and reports the orphan
    /// node FQNs on GET /health under the `unmanifested_nodes` warning code.
    /// Does NOT fail startup: the gateway keeps serving and status stays
    /// "healthy".
    ERROR,
    /// Same entity tree as WARN; only the log level differs (info rather than
    /// warning). No entity is tagged source="orphan" - nothing in discovery
    /// ever assigns that value.
    INCLUDE_AS_ORPHAN
  };

  UnmanifestedNodePolicy unmanifested_nodes{UnmanifestedNodePolicy::WARN};
  bool inherit_runtime_resources{true};  ///< Copy topics/services from runtime
  bool allow_manifest_override{true};    ///< Manifest can override runtime props

  /// Parse policy from string
  static UnmanifestedNodePolicy parse_policy(const std::string & str);
  /// Convert policy to string
  static std::string policy_to_string(UnmanifestedNodePolicy policy);
};

/**
 * @brief A parse-time notice that must obey the strictness dial
 *
 * ManifestManager copies these into ValidationResult as warnings, so
 * `discovery.manifest_strict_validation` decides whether they reject the
 * manifest or merely annotate it.
 */
struct ManifestParseNotice {
  std::string rule_id;
  std::string message;
};

/**
 * @brief Manifest metadata
 */
struct ManifestMetadata {
  std::string name;
  std::string description;
  std::string version;
  std::string created_at;
};

/**
 * @brief Full manifest structure
 *
 * Represents a parsed manifest YAML file containing entity definitions
 * and discovery configuration.
 */
/**
 * @brief Per-entity lock configuration from manifest
 */
struct ManifestLockConfig {
  std::vector<std::string> required_scopes;  ///< Scopes that must be locked (empty = no requirement)
  bool breakable = true;                     ///< Whether locks on this entity can be broken
  int max_expiration = 0;                    ///< Max expiration seconds (0 = use global default)
};

struct Manifest {
  std::string manifest_version;  ///< Must be "1.0"
  ManifestMetadata metadata;
  ManifestConfig config;

  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;

  /// Script entries loaded from manifest
  std::vector<ros2_medkit_gateway::ScriptEntryConfig> scripts;

  /// Custom capabilities overrides per entity
  std::unordered_map<std::string, json> capabilities;

  /// Per-entity lock configuration overrides (entity_id -> lock config)
  std::unordered_map<std::string, ManifestLockConfig> lock_overrides;

  /// Informational parse notices (deprecated key, key collision, unknown
  /// top-level key). Logged by ManifestManager; deliberately NEVER copied into
  /// ValidationResult, because strict mode turns validator warnings into load
  /// failures and would make a deprecation notice fatal under the default
  /// configuration.
  std::vector<std::string> log_notices;

  /// Parse notices that ARE correctness problems and must obey
  /// discovery.manifest_strict_validation.
  std::vector<ManifestParseNotice> validation_notices;

  /// Check if manifest has been loaded
  bool is_loaded() const {
    return !manifest_version.empty();
  }
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
