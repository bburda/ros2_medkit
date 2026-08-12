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

#include "ros2_medkit_gateway/core/discovery/manifest/manifest_parser.hpp"

#include "ros2_medkit_gateway/core/discovery/identity_merge.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>
#include <unordered_set>

#include "ros2_medkit_gateway/core/discovery/manifest/asset_inventory.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {
namespace discovery {

namespace {
// A top-level block counts as present only when it carries a value. yaml-cpp
// reports a key with no value as defined-but-null, and treating that as
// present makes an empty `config:` outrank a populated `discovery:`.
bool is_present_block(const YAML::Node & node) {
  return static_cast<bool>(node) && !node.IsNull();
}

}  // namespace

namespace {

/// Every top-level key `parse_string` reads, plus the deprecated `discovery`
/// alias. Deliberately next to the parsing code: a new section cannot be added
/// to `parse_string` without extending this list, and anything outside it is
/// reported as ignored instead of being dropped in silence.
constexpr const char * kKnownTopLevelKeys[] = {
    "manifest_version", "metadata", "config",    "discovery", "areas",        "components",
    "assets",           "apps",     "functions", "scripts",   "capabilities",
};

bool is_known_top_level_key(const std::string & key) {
  for (const char * known : kKnownTopLevelKeys) {
    if (key == known) {
      return true;
    }
  }
  return false;
}

std::string known_top_level_keys_csv() {
  std::string joined;
  for (const char * known : kKnownTopLevelKeys) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += known;
  }
  return joined;
}

}  // namespace

Manifest ManifestParser::parse_file(const std::string & file_path) const {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open manifest file: " + file_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_string(buffer.str());
}

Manifest ManifestParser::parse_fragment_file(const std::string & file_path) const {
  // Read the yaml into a string, inject a synthetic manifest_version if the
  // fragment omits one (fragments are not required to declare it), then
  // reuse the main parse_string pipeline so every field is parsed with the
  // exact same logic as a full manifest. Anything forbidden in fragments
  // (areas, metadata, scripts, ...) is still parsed, which lets the caller
  // detect and reject it with a specific error message.

  // Cap file size before opening to avoid OOM on a misconfigured fragments_dir
  // (e.g., a symlink to a log file). yaml-cpp has no builtin size limit.
  std::error_code size_ec;
  auto size = std::filesystem::file_size(file_path, size_ec);
  if (size_ec) {
    throw std::runtime_error("Cannot stat manifest fragment '" + file_path + "': " + size_ec.message());
  }
  if (size > kMaxFragmentBytes) {
    throw std::runtime_error("Manifest fragment '" + file_path + "' exceeds " + std::to_string(kMaxFragmentBytes) +
                             "-byte limit (size=" + std::to_string(size) + ")");
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open manifest fragment: " + file_path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string contents = buffer.str();

  // Version prefix is appended-only when the source yaml does not set one.
  // Look for a TOP-LEVEL `manifest_version:` key - i.e., exactly at column 0
  // (no leading whitespace). An earlier revision tolerated any indentation,
  // which accidentally matched any nested sub-map key of the same name
  // (e.g., `ros_binding: { manifest_version: ... }`) and disabled the
  // synthetic injection, making otherwise-valid fragments fail to parse.
  // Sufficient for the yaml dialect we accept (no flow-style top-level
  // document, no document separators that would shift the top level).
  auto has_version_field = [](const std::string & s) {
    size_t pos = 0;
    while (pos < s.size()) {
      size_t end = s.find('\n', pos);
      std::string line = s.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
      // Require column 0: only an un-indented line with this exact key is
      // treated as a manifest-level declaration.
      if (line.compare(0, sizeof("manifest_version:") - 1, "manifest_version:") == 0) {
        return true;
      }
      if (end == std::string::npos) {
        break;
      }
      pos = end + 1;
    }
    return false;
  };
  if (!has_version_field(contents)) {
    contents = "manifest_version: \"1.0\"\n" + contents;
  }
  return parse_string(contents);
}

Manifest ManifestParser::parse_string(const std::string & yaml_content) const {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_content);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("YAML parse error: " + std::string(e.what()));
  }

  Manifest manifest;

  // Parse version (required)
  manifest.manifest_version = get_string(root, "manifest_version");
  if (manifest.manifest_version.empty()) {
    throw std::runtime_error("Missing required field: manifest_version");
  }

  // Parse metadata
  if (root["metadata"]) {
    manifest.metadata = parse_metadata(root["metadata"]);
  }

  // Parse discovery config. `config:` is the documented, canonical key.
  // `discovery:` is the key the parser originally read and is kept working as
  // a deprecated alias; when both are present, `config:` wins.
  // "Present" must mean the same thing here as it does in parse_config_block,
  // which treats a null value as absent. A bare `config:` with nothing under
  // it used to win the precedence test and then parse to nothing, so a
  // populated `discovery:` below it was skipped, the settings silently did not
  // apply, and the operator was told "'config:' is used" - which reads as
  // confirmation that it did.
  const bool has_config = is_present_block(root["config"]);
  const bool has_discovery_alias = is_present_block(root["discovery"]);
  // R014/R015 are ADVISORY for this release: collected here and emitted as log
  // notices rather than validation warnings. Reason: this branch is what makes
  // the block read at all, so a manifest carrying `unmanifested_nodes: Warn` or
  // `inherit_runtime_resources: 0` loaded fine before the upgrade and would be
  // REFUSED after it, under the shipped `manifest_strict_validation: true` -
  // taking hybrid discovery down to runtime_only for a file the operator never
  // touched. They become validation warnings in the next release; a wrong
  // value already falls back to the documented default either way.
  std::vector<ManifestParseNotice> config_notices;
  if (has_config) {
    manifest.config = parse_config_block(root["config"], "config", config_notices);
    if (has_discovery_alias) {
      manifest.log_notices.emplace_back(
          "Manifest declares both 'config:' and the deprecated 'discovery:' top-level keys; 'config:' is used and "
          "'discovery:' is ignored.");
    }
  } else if (has_discovery_alias) {
    manifest.config = parse_config_block(root["discovery"], "discovery", config_notices);
    manifest.log_notices.emplace_back(
        "Manifest uses the deprecated top-level 'discovery:' key for discovery configuration; rename it to 'config:'. "
        "The alias still works and will be removed in a future release.");
  }
  for (const auto & notice : config_notices) {
    manifest.log_notices.emplace_back("[" + notice.rule_id + "] " + notice.message +
                                      " (advisory in this release; becomes a validation warning in the next one)");
  }

  // Parse areas (with recursive subareas)
  if (root["areas"] && root["areas"].IsSequence()) {
    for (const auto & node : root["areas"]) {
      parse_area_recursive(node, "", manifest.areas);
    }
  }

  // Parse components
  if (root["components"] && root["components"].IsSequence()) {
    for (const auto & node : root["components"]) {
      auto comp = parse_component(node);
      if (node["lock"] && node["lock"].IsMap()) {
        manifest.lock_overrides[comp.id] = parse_lock_config(node["lock"]);
      }
      manifest.components.push_back(std::move(comp));
    }
  }

  // Parse assets (manual inventory list). Each asset becomes a Component with
  // identity populated, so it flows through the same merge-by-id path as any
  // other component and combines with protocol-discovered structure.
  if (root["assets"] && root["assets"].IsSequence()) {
    for (const auto & node : root["assets"]) {
      manifest.components.push_back(parse_asset(node));
    }
  }

  // Parse apps
  if (root["apps"] && root["apps"].IsSequence()) {
    for (const auto & node : root["apps"]) {
      auto app = parse_app(node);
      if (node["lock"] && node["lock"].IsMap()) {
        manifest.lock_overrides[app.id] = parse_lock_config(node["lock"]);
      }
      manifest.apps.push_back(std::move(app));
    }
  }

  // Parse functions
  if (root["functions"] && root["functions"].IsSequence()) {
    for (const auto & node : root["functions"]) {
      manifest.functions.push_back(parse_function(node));
    }
  }

  // Parse scripts
  if (root["scripts"] && root["scripts"].IsSequence()) {
    for (const auto & node : root["scripts"]) {
      manifest.scripts.push_back(parse_script_entry(node));
    }
  }

  // Parse capabilities (optional map)
  if (root["capabilities"] && root["capabilities"].IsMap()) {
    for (const auto & it : root["capabilities"]) {
      std::string entity_id = it.first.as<std::string>();
      // Store as empty JSON object for now
      // Full YAML->JSON conversion can be added if needed
      manifest.capabilities[entity_id] = json::object();
    }
  }

  // Anything the parser did not read is ignored - say so. A silently ignored
  // top-level key is how a whole configuration block can go missing without a
  // single line of output. Non-scalar keys are skipped, as the fragment path
  // does, because they cannot be rendered into a message.
  if (root.IsMap()) {
    for (const auto & it : root) {
      if (!it.first.IsScalar()) {
        continue;
      }
      std::string key = it.first.as<std::string>();
      if (is_known_top_level_key(key)) {
        continue;
      }
      manifest.log_notices.emplace_back("Manifest has unknown top-level key '" + key +
                                        "' - it is ignored. Known keys: " + known_top_level_keys_csv() + ".");
    }
  }

  return manifest;
}

ManifestMetadata ManifestParser::parse_metadata(const YAML::Node & node) const {
  ManifestMetadata meta;
  meta.name = get_string(node, "name");
  meta.description = get_string(node, "description");
  meta.version = get_string(node, "version");
  meta.created_at = get_string(node, "created_at");
  return meta;
}

// Gate the discovery-configuration block on its YAML kind before indexing it.
// parse_config() addresses the node by field name, and a node that is not a
// map cannot be addressed that way: yaml-cpp THROWS on a scalar (which would
// abandon the whole manifest, losing every entity in it over one bad key) and
// silently yields nothing on a sequence (which would drop the operator's
// configuration without a word). A key that carries no value at all is YAML
// null and means "absent": it loads in silence, here and for every setting
// inside the block (see read_bool_setting and parse_config).
//
// R015 is a VALIDATION notice, so what happens next depends on the strictness
// dial: with the shipped default `manifest_strict_validation: true` the
// manifest is rejected and hybrid mode degrades to runtime_only; with strict
// validation off the manifest loads, the defaults apply and the notice is
// logged. "The defaults apply" is only the non-strict half of the story.
ManifestConfig ManifestParser::parse_config_block(const YAML::Node & node, const std::string & key,
                                                  std::vector<ManifestParseNotice> & notices) const {
  if (node.IsMap()) {
    return parse_config(node, notices);
  }
  if (node.IsNull()) {
    return ManifestConfig{};  // present but empty == absent
  }

  const char * kind = node.IsSequence() ? "sequence" : "scalar";
  notices.push_back({"R015", "Manifest key '" + key + "' must be a mapping of discovery settings, but is a " +
                                 std::string(kind) + "; the block is ignored and the defaults apply."});
  return ManifestConfig{};
}

namespace {

// Describe a node's YAML kind for an operator-facing message.
const char * yaml_kind_name(const YAML::Node & node) {
  if (node.IsSequence()) {
    return "sequence";
  }
  if (node.IsMap()) {
    return "mapping";
  }
  return "scalar";
}

// True when a settings key is present and carries an actual value. A key with
// no value at all parses as YAML null, which means "not set" and must load in
// silence - writing `unmanifested_nodes:` and nothing else is not an error.
bool has_value(const YAML::Node & node, const std::string & key) {
  return static_cast<bool>(node[key]) && !node[key].IsNull();
}

}  // namespace

// Read one boolean setting. Same guard the block itself gets, one level down:
// `.as<bool>()` THROWS on anything it cannot convert - including a key left
// empty - and that exception would escape the whole manifest load, so a single
// mistyped flag would cost every entity in the file. Wrong kind or wrong text
// is reported under R015 and the default stands.
void ManifestParser::read_bool_setting(const YAML::Node & node, const std::string & key, bool & target,
                                       std::vector<ManifestParseNotice> & notices) const {
  if (!has_value(node, key)) {
    return;
  }
  const YAML::Node value = node[key];
  if (value.IsScalar()) {
    bool parsed = false;
    if (YAML::convert<bool>::decode(value, parsed)) {
      target = parsed;
      return;
    }
    notices.push_back({"R015", "Manifest setting '" + key + "' must be a boolean, but is '" + value.as<std::string>() +
                                   "'; the default (" + (target ? "true" : "false") + ") applies."});
    return;
  }
  notices.push_back({"R015", "Manifest setting '" + key + "' must be a boolean, but is a " + yaml_kind_name(value) +
                                 "; the default (" + (target ? "true" : "false") + ") applies."});
}

ManifestConfig ManifestParser::parse_config(const YAML::Node & node, std::vector<ManifestParseNotice> & notices) const {
  ManifestConfig config;

  // A key with no value means "not set", which is the struct default and not a
  // wrong value. Only a key that carries something is parsed - otherwise
  // get_string() renders YAML null as the literal text "null" and the operator
  // is told their empty key is an unknown policy.
  if (has_value(node, "unmanifested_nodes")) {
    const YAML::Node policy_node = node["unmanifested_nodes"];
    if (!policy_node.IsScalar()) {
      notices.push_back({"R015", "Manifest setting 'unmanifested_nodes' must be a string, but is a " +
                                     std::string(yaml_kind_name(policy_node)) + "; the default ('warn') applies."});
    } else if (const std::string policy = policy_node.as<std::string>(); !policy.empty()) {
      config.unmanifested_nodes = ManifestConfig::parse_policy(policy);
      // parse_policy is total - every unrecognised string maps to WARN - so at
      // this call site a typo is indistinguishable from a deliberate "warn".
      // Detect it by round-tripping the parsed value rather than by keeping a
      // second copy of the valid-value list here.
      if (ManifestConfig::policy_to_string(config.unmanifested_nodes) != policy) {
        notices.push_back({"R014", "Unknown unmanifested_nodes value '" + policy +
                                       "'; falling back to 'warn'. Valid values: ignore, warn, error, "
                                       "include_as_orphan (lower-case)."});
      }
    }
  }

  read_bool_setting(node, "inherit_runtime_resources", config.inherit_runtime_resources, notices);
  read_bool_setting(node, "allow_manifest_override", config.allow_manifest_override, notices);

  return config;
}

void ManifestParser::parse_area_recursive(const YAML::Node & node, const std::string & parent_id,
                                          std::vector<Area> & areas) const {
  Area area;
  area.id = get_string(node, "id");
  area.name = get_string(node, "name", area.id);  // Default to id if no name
  area.namespace_path = get_string(node, "namespace", "/" + area.id);
  area.translation_id = get_string(node, "translation_id");
  area.description = get_string(node, "description");
  area.tags = get_string_vector(node, "tags");
  // Set parent from recursive call, or from explicit parent_area field
  area.parent_area_id = parent_id.empty() ? get_string(node, "parent_area") : parent_id;
  area.source = "manifest";

  areas.push_back(area);

  // Recursively parse nested subareas
  if (node["subareas"] && node["subareas"].IsSequence()) {
    for (const auto & subarea_node : node["subareas"]) {
      parse_area_recursive(subarea_node, area.id, areas);
    }
  }
}

Component ManifestParser::parse_component(const YAML::Node & node) const {
  Component comp;
  comp.id = get_string(node, "id");
  comp.name = get_string(node, "name", comp.id);
  comp.namespace_path = get_string(node, "namespace");
  comp.area = get_string(node, "area");
  comp.translation_id = get_string(node, "translation_id");
  comp.description = get_string(node, "description");
  comp.variant = get_string(node, "variant");
  comp.tags = get_string_vector(node, "tags");
  comp.parent_component_id = get_string(node, "parent_component_id");
  comp.depends_on = get_string_vector(node, "depends_on");
  comp.source = "manifest";
  comp.identity = parse_identity(node);

  // Preserve the omitted-vs-explicit distinction: an absent `external:` key
  // leaves nullopt so the hybrid merge cannot let a stub's default erase a
  // plugin's introspected classification (#516, mirrors the App rule #517).
  if (node["external"]) {
    comp.external = node["external"].as<bool>();
  }

  // Parse type if provided (e.g., "controller", "sensor", "actuator")
  std::string type_val = get_string(node, "type");
  if (!type_val.empty()) {
    comp.type = type_val;
  }

  // Compute FQN if namespace and id are provided
  if (!comp.namespace_path.empty()) {
    comp.fqn = comp.namespace_path + "/" + comp.id;
  } else {
    comp.fqn = "/" + comp.id;
  }

  return comp;
}

Component ManifestParser::parse_asset(const YAML::Node & node) const {
  // Structural / presentation keys handled explicitly below. Identity keys are
  // routed through the shared asset_column alias table, so `assets:` accepts
  // exactly the names the CSV import does (serial / serial_number, ...); any
  // other scalar key is retained verbatim as an extra so operator-specific
  // columns are not lost.
  static const std::unordered_set<std::string> structural = {
      "tags", "parent_component_id", "name",       "description", "namespace", "variant",
      "type", "translation_id",      "depends_on", "external"};

  AssetEntry entry;
  if (node.IsMap()) {
    for (const auto & it : node) {
      if (!it.first.IsScalar() || !it.second.IsScalar()) {
        continue;
      }
      const std::string key = it.first.as<std::string>();
      if (structural.count(key) != 0) {
        continue;
      }
      assign_asset_field(entry, asset_column(key), key, it.second.as<std::string>());
    }
  }

  Component comp = asset_entry_to_component(entry);

  // Optional tree-placement and presentation overrides. Every key listed in
  // `structural` above must be consumed here; otherwise it is silently dropped.
  const std::string ns = get_string(node, "namespace");
  if (!ns.empty()) {
    // Operator-declared placement: compute an authoritative fqn like a real
    // component. Without a namespace, fqn stays empty so a merge with a
    // discovered node keeps the node's real path instead of a synthetic "/id".
    comp.namespace_path = ns;
    comp.fqn = ns + "/" + comp.id;
  }
  const std::string parent = get_string(node, "parent_component_id");
  if (!parent.empty()) {
    comp.parent_component_id = parent;
  }
  const std::string explicit_name = get_string(node, "name");
  if (!explicit_name.empty()) {
    comp.name = explicit_name;
  }
  const std::string explicit_description = get_string(node, "description");
  if (!explicit_description.empty()) {
    comp.description = explicit_description;
  }
  const std::string variant = get_string(node, "variant");
  if (!variant.empty()) {
    comp.variant = variant;  // hardware_rev stays on the identity; variant is explicit only
  }
  const std::string type_val = get_string(node, "type");
  if (!type_val.empty()) {
    comp.type = type_val;
  }
  const std::string translation_id = get_string(node, "translation_id");
  if (!translation_id.empty()) {
    comp.translation_id = translation_id;
  }
  for (const auto & dep : get_string_vector(node, "depends_on")) {
    comp.depends_on.push_back(dep);
  }
  for (const auto & tag : get_string_vector(node, "tags")) {
    comp.tags.push_back(tag);
  }

  // An `assets:` entry may classify the device as a non-ROS external asset,
  // mirroring `parse_component`. Tri-state: an absent key stays nullopt.
  // `external` is in the `structural` set above so it is not swallowed as an
  // identity extra (#516).
  if (node["external"]) {
    comp.external = node["external"].as<bool>();
  }

  return comp;
}

AssetIdentity ManifestParser::parse_identity(const YAML::Node & node) const {
  AssetIdentity identity;
  if (!node["identity"]) {
    return identity;
  }
  const YAML::Node & id_node = node["identity"];
  identity.manufacturer = get_string(id_node, "manufacturer");
  identity.model = get_string(id_node, "model");
  identity.order_code = get_string(id_node, "order_code");
  identity.serial_number = get_string(id_node, "serial_number");
  identity.hardware_revision = get_string(id_node, "hardware_revision");
  identity.firmware_version = get_string(id_node, "firmware_version");
  identity.software_version = get_string(id_node, "software_version");
  identity.network_endpoint = get_string(id_node, "network_endpoint");
  identity.role = get_string(id_node, "role");
  if (id_node["extra"] && id_node["extra"].IsMap()) {
    for (const auto & kv : id_node["extra"]) {
      identity.extra[kv.first.as<std::string>()] = kv.second.as<std::string>();
    }
  }
  // Pre-stamp provenance so the identity origin is recorded even in
  // MANIFEST_ONLY mode, where the merge pipeline (which otherwise seeds
  // provenance from Component.source) is bypassed. Only populated fields are
  // stamped; a merge later overrides per field with a higher-authority source.
  stamp_identity_provenance(identity, "manifest");
  return identity;
}

App ManifestParser::parse_app(const YAML::Node & node) const {
  App app;
  app.id = get_string(node, "id");
  app.name = get_string(node, "name", app.id);
  app.translation_id = get_string(node, "translation_id");
  app.description = get_string(node, "description");
  app.component_id = get_string(node, "is_located_on");
  app.depends_on = get_string_vector(node, "depends_on");
  app.tags = get_string_vector(node, "tags");
  // Preserve the omitted-vs-explicit distinction: an absent `external:` key
  // leaves nullopt (the manifest does not classify), so the hybrid merge cannot
  // let a stub's default erase a plugin's introspected classification (#517).
  if (node["external"]) {
    app.external = node["external"].as<bool>();
  }
  app.source = "manifest";

  // Parse ros_binding
  if (node["ros_binding"]) {
    App::RosBinding binding;
    binding.node_name = get_string(node["ros_binding"], "node_name");
    binding.namespace_pattern = get_string(node["ros_binding"], "namespace", "*");
    binding.topic_namespace = get_string(node["ros_binding"], "topic_namespace");
    app.ros_binding = binding;
  }

  return app;
}

Function ManifestParser::parse_function(const YAML::Node & node) const {
  Function func;
  func.id = get_string(node, "id");
  func.name = get_string(node, "name", func.id);
  func.translation_id = get_string(node, "translation_id");
  func.description = get_string(node, "description");
  // Support both "hosted_by" (manifest) and "hosts" (internal)
  func.hosts = get_string_vector(node, "hosted_by");
  if (func.hosts.empty()) {
    func.hosts = get_string_vector(node, "hosts");
  }
  func.depends_on = get_string_vector(node, "depends_on");
  func.tags = get_string_vector(node, "tags");
  func.source = "manifest";

  return func;
}

std::string ManifestParser::get_string(const YAML::Node & node, const std::string & key,
                                       const std::string & default_val) const {
  if (node[key]) {
    return node[key].as<std::string>();
  }
  return default_val;
}

std::vector<std::string> ManifestParser::get_string_vector(const YAML::Node & node, const std::string & key) const {
  std::vector<std::string> result;
  if (node[key] && node[key].IsSequence()) {
    for (const auto & item : node[key]) {
      result.push_back(item.as<std::string>());
    }
  }
  return result;
}

// ManifestConfig helper implementations
ManifestConfig::UnmanifestedNodePolicy ManifestConfig::parse_policy(const std::string & str) {
  if (str == "ignore") {
    return UnmanifestedNodePolicy::IGNORE;
  }
  if (str == "error") {
    return UnmanifestedNodePolicy::ERROR;
  }
  if (str == "include_as_orphan") {
    return UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN;
  }
  return UnmanifestedNodePolicy::WARN;  // Default
}

std::string ManifestConfig::policy_to_string(UnmanifestedNodePolicy policy) {
  switch (policy) {
    case UnmanifestedNodePolicy::IGNORE:
      return "ignore";
    case UnmanifestedNodePolicy::ERROR:
      return "error";
    case UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN:
      return "include_as_orphan";
    case UnmanifestedNodePolicy::WARN:
    default:
      return "warn";
  }
}

ros2_medkit_gateway::ScriptEntryConfig ManifestParser::parse_script_entry(const YAML::Node & node) const {
  ros2_medkit_gateway::ScriptEntryConfig entry;
  entry.id = get_string(node, "id");
  if (entry.id.empty()) {
    throw std::runtime_error("Script entry missing required field: id");
  }
  entry.name = get_string(node, "name", entry.id);
  entry.description = get_string(node, "description");
  entry.path = get_string(node, "path");
  if (entry.path.empty()) {
    throw std::runtime_error("Script entry '" + entry.id + "' missing required field: path");
  }
  entry.format = get_string(node, "format");
  if (entry.format.empty()) {
    throw std::runtime_error("Script entry '" + entry.id + "' missing required field: format");
  }
  if (entry.format != "bash" && entry.format != "python" && entry.format != "sh") {
    throw std::runtime_error("Script entry '" + entry.id + "' has unknown format: '" + entry.format +
                             "' (expected: bash, python, sh)");
  }
  if (node["timeout_sec"]) {
    entry.timeout_sec = std::max(1, node["timeout_sec"].as<int>());
  }
  entry.entity_filter = get_string_vector(node, "entity_filter");

  // Parse env map (skip non-scalar values to avoid yaml-cpp exceptions)
  if (node["env"] && node["env"].IsMap()) {
    for (const auto & it : node["env"]) {
      if (it.second.IsScalar()) {
        entry.env[it.first.as<std::string>()] = it.second.as<std::string>();
      }
    }
  }

  // Parse args (JSON array of {name, type, flag} objects)
  if (node["args"] && node["args"].IsSequence()) {
    entry.args = json::array();
    for (const auto & arg_node : node["args"]) {
      json arg_obj;
      if (arg_node["name"]) {
        arg_obj["name"] = arg_node["name"].as<std::string>();
      }
      if (arg_node["type"]) {
        arg_obj["type"] = arg_node["type"].as<std::string>();
      }
      if (arg_node["flag"]) {
        arg_obj["flag"] = arg_node["flag"].as<std::string>();
      }
      entry.args.push_back(arg_obj);
    }
  }

  // Parse parameters_schema via recursive YAML-to-JSON conversion
  if (node["parameters_schema"]) {
    auto schema = ros2_medkit_serialization::JsonSerializer::yaml_to_json(node["parameters_schema"]);
    if (!schema.is_null() && !schema.empty()) {
      entry.parameters_schema = schema;
    }
  }

  return entry;
}

ManifestLockConfig ManifestParser::parse_lock_config(const YAML::Node & node) const {
  ManifestLockConfig config;
  config.required_scopes = get_string_vector(node, "required_scopes");
  if (node["breakable"]) {
    config.breakable = node["breakable"].as<bool>();
  }
  if (node["max_expiration"]) {
    config.max_expiration = std::max(0, node["max_expiration"].as<int>());
  }
  return config;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
