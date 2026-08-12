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

#include "ros2_medkit_gateway/core/discovery/manifest/manifest.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Forward declare YAML::Node to avoid including yaml-cpp in header
namespace YAML {
class Node;
}

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Parses manifest YAML files into Manifest structure
 *
 * The parser handles YAML loading and converts the document into
 * internal model structures (Area, Component, App, Function).
 */
class ManifestParser {
 public:
  /**
   * @brief Parse manifest from file
   * @param file_path Path to YAML file
   * @return Parsed manifest
   * @throws std::runtime_error if file cannot be read or parsed
   */
  Manifest parse_file(const std::string & file_path) const;

  /**
   * @brief Parse manifest from YAML string
   * @param yaml_content YAML content as string
   * @return Parsed manifest
   * @throws std::runtime_error if YAML is malformed
   */
  Manifest parse_string(const std::string & yaml_content) const;

  /// Maximum accepted size of a single fragment file in bytes. `fragments_dir`
  /// is user-configurable, so a misconfigured path (or a symlink pointing at a
  /// large log file) could otherwise feed an unbounded amount of data into
  /// `YAML::Load`, which has no builtin cap. 1 MiB is well above any realistic
  /// fragment (tens of entities) while keeping worst-case allocation bounded.
  static constexpr std::uintmax_t kMaxFragmentBytes = 1U << 20U;

  /**
   * @brief Parse a manifest fragment from file.
   *
   * Fragments declare a subset of a manifest (apps, components, functions)
   * intended to be merged on top of a base manifest at load time. A
   * fragment MAY declare `manifest_version` but is not required to - when
   * omitted, a synthetic `"1.0"` value is injected so the shared
   * `parse_string` pipeline still sees a well-formed manifest. Forbidden
   * top-level fields (`areas`, `metadata`, `config`, `discovery`, `scripts`,
   * `capabilities`, `lock_overrides`) are still parsed into the returned
   * `Manifest` so the caller (`ManifestManager::apply_fragments`) can
   * detect and reject them with a specific validation error.
   *
   * Files larger than `kMaxFragmentBytes` are rejected up-front without being
   * read into memory.
   *
   * @param file_path Path to fragment YAML
   * @return Parsed manifest with only the fragment-allowed sections populated
   * @throws std::runtime_error if file cannot be read or parsed, or if it
   *         exceeds `kMaxFragmentBytes`.
   */
  Manifest parse_fragment_file(const std::string & file_path) const;

 private:
  /// Recursively parse area and its nested subareas
  void parse_area_recursive(const YAML::Node & node, const std::string & parent_id, std::vector<Area> & areas) const;
  Component parse_component(const YAML::Node & node) const;
  /// Parse a manual-inventory asset entry into a Component (identity populated).
  Component parse_asset(const YAML::Node & node) const;
  /// Parse an optional `identity:` block into an AssetIdentity, stamping
  /// per-field provenance to "manifest".
  AssetIdentity parse_identity(const YAML::Node & node) const;
  App parse_app(const YAML::Node & node) const;
  Function parse_function(const YAML::Node & node) const;
  /// Validate the YAML kind of the discovery-configuration block (`config:`,
  /// or the deprecated `discovery:` alias), then parse it. A null block means
  /// "absent". A non-map block is reported under R015, which is a VALIDATION
  /// notice: under the shipped `manifest_strict_validation: true` that rejects
  /// the whole manifest, and only with strict validation off does the block
  /// get ignored and the defaults apply. Either way parse_config() only ever
  /// sees a map.
  ManifestConfig parse_config_block(const YAML::Node & node, const std::string & key,
                                    std::vector<ManifestParseNotice> & notices) const;
  /// Parse the settings inside an already-validated block. Values that are
  /// recognised but wrong - an unrecognised `unmanifested_nodes` (R014), or a
  /// setting of the wrong YAML kind (R015) - are appended to @p notices so the
  /// caller can put them through `discovery.manifest_strict_validation`.
  ManifestConfig parse_config(const YAML::Node & node, std::vector<ManifestParseNotice> & notices) const;
  /// Read one boolean discovery setting, leaving *target* alone when the key
  /// is absent or null and reporting R015 when it is present but not a
  /// boolean. Never throws: `.as<bool>()` would abort the whole manifest load.
  void read_bool_setting(const YAML::Node & node, const std::string & key, bool & target,
                         std::vector<ManifestParseNotice> & notices) const;
  ManifestMetadata parse_metadata(const YAML::Node & node) const;
  ManifestLockConfig parse_lock_config(const YAML::Node & node) const;
  ScriptEntryConfig parse_script_entry(const YAML::Node & node) const;

  /// Get string value with default
  std::string get_string(const YAML::Node & node, const std::string & key, const std::string & default_val = "") const;

  /// Get string vector
  std::vector<std::string> get_string_vector(const YAML::Node & node, const std::string & key) const;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
