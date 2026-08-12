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

/**
 * @file test_manifest_parser.cpp
 * @brief Unit tests for manifest YAML parser
 *
 * @verifies REQ_DISCOVERY_003 Manifest parsing
 */

#include <gtest/gtest.h>

#include <optional>

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/manifest/manifest_parser.hpp"

using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::discovery::ManifestConfig;
using ros2_medkit_gateway::discovery::ManifestParser;

namespace {

// R014 and R015 are ADVISORY this release: emitted on the log-only channel so
// the strictness dial cannot turn a manifest that loaded before the upgrade
// into a load failure after it. Returns the message with the rule-id prefix
// and advisory suffix stripped, so assertions stay about the message itself.
std::optional<std::string> find_notice(const ros2_medkit_gateway::discovery::Manifest & manifest,
                                       const std::string & rule_id) {
  const std::string prefix = "[" + rule_id + "] ";
  const std::string suffix = " (advisory in this release; becomes a validation warning in the next one)";
  for (const auto & notice : manifest.log_notices) {
    if (notice.rfind(prefix, 0) != 0) {
      continue;
    }
    std::string message = notice.substr(prefix.size());
    if (message.size() >= suffix.size() &&
        message.compare(message.size() - suffix.size(), suffix.size(), suffix) == 0) {
      message.erase(message.size() - suffix.size());
    }
    return message;
  }
  return std::nullopt;
}

}  // namespace

// =============================================================================
// Valid Manifest Parsing Tests
// =============================================================================

class ManifestParserTest : public ::testing::Test {
 protected:
  ManifestParser parser_;
};

TEST_F(ManifestParserTest, ParseMinimalManifest) {
  const std::string yaml = R"(
manifest_version: "1.0"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.manifest_version, "1.0");
  EXPECT_TRUE(manifest.is_loaded());
}

TEST_F(ManifestParserTest, ParseMetadata) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Test Manifest"
  description: "A test manifest"
  version: "1.0.0"
  created_at: "2025-01-15"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.metadata.name, "Test Manifest");
  EXPECT_EQ(manifest.metadata.description, "A test manifest");
  EXPECT_EQ(manifest.metadata.version, "1.0.0");
  EXPECT_EQ(manifest.metadata.created_at, "2025-01-15");
}

TEST_F(ManifestParserTest, ParseConfigBlock) {
  const std::string yaml = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: false
  allow_manifest_override: true
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(manifest.config.inherit_runtime_resources);
  EXPECT_TRUE(manifest.config.allow_manifest_override);
  EXPECT_TRUE(manifest.log_notices.empty()) << "canonical key must not produce a notice";
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, ParseConfigBlockDefaultPolicy) {
  const std::string yaml = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: "warn"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, ParseAreas) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "powertrain"
    name: "Powertrain System"
    namespace: "/powertrain"
    description: "Motor and drive systems"
    tags:
      - critical
      - automotive
  - id: "sensors"
    parent_area: "vehicle"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.areas.size(), 2);

  EXPECT_EQ(manifest.areas[0].id, "powertrain");
  EXPECT_EQ(manifest.areas[0].name, "Powertrain System");
  EXPECT_EQ(manifest.areas[0].namespace_path, "/powertrain");
  EXPECT_EQ(manifest.areas[0].description, "Motor and drive systems");
  EXPECT_EQ(manifest.areas[0].tags.size(), 2);

  EXPECT_EQ(manifest.areas[1].id, "sensors");
  EXPECT_EQ(manifest.areas[1].name, "sensors");  // Defaults to id
  EXPECT_EQ(manifest.areas[1].parent_area_id, "vehicle");
}

TEST_F(ManifestParserTest, ParseComponents) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "motor_controller"
    name: "Motor Controller"
    namespace: "/powertrain"
    area: "powertrain"
    description: "Controls the motor"
    variant: "v2"
    tags:
      - actuator
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.components.size(), 1);

  const auto & comp = manifest.components[0];
  EXPECT_EQ(comp.id, "motor_controller");
  EXPECT_EQ(comp.name, "Motor Controller");
  EXPECT_EQ(comp.namespace_path, "/powertrain");
  EXPECT_EQ(comp.fqn, "/powertrain/motor_controller");
  EXPECT_EQ(comp.area, "powertrain");
  EXPECT_EQ(comp.source, "manifest");
  EXPECT_EQ(comp.variant, "v2");
  EXPECT_EQ(comp.tags.size(), 1);
}

// INV2: a manifest component may declare an asset-identity nameplate. Parsed
// fields land on Component.identity with per-field provenance stamped
// "manifest" (so identity is attributable even in MANIFEST_ONLY mode).
TEST_F(ManifestParserTest, ParseComponentIdentity) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "plc_1"
    name: "Line PLC"
    identity:
      manufacturer: "Siemens"
      model: "S7-1500"
      order_code: "6ES7 672-5SC11-0YA0"
      serial_number: "SN-42"
      hardware_revision: "HW-3"
      firmware_version: "2.9.4"
      software_version: "app-1.0"
      network_endpoint: "opc.tcp://plc:4840"
      role: "plc"
      extra:
        slot: "3"
)";

  auto manifest = parser_.parse_string(yaml);
  ASSERT_EQ(manifest.components.size(), 1);
  const auto & id = manifest.components[0].identity;

  EXPECT_EQ(id.manufacturer, "Siemens");
  EXPECT_EQ(id.model, "S7-1500");
  EXPECT_EQ(id.order_code, "6ES7 672-5SC11-0YA0");
  EXPECT_EQ(id.serial_number, "SN-42");
  EXPECT_EQ(id.hardware_revision, "HW-3");
  EXPECT_EQ(id.firmware_version, "2.9.4");
  EXPECT_EQ(id.software_version, "app-1.0");
  EXPECT_EQ(id.network_endpoint, "opc.tcp://plc:4840");
  EXPECT_EQ(id.role, "plc");
  EXPECT_EQ(id.extra.at("slot"), "3");

  EXPECT_EQ(id.provenance.at("manufacturer"), "manifest");
  EXPECT_EQ(id.provenance.at("serial_number"), "manifest");
  EXPECT_EQ(id.provenance.at("extra.slot"), "manifest");
}

TEST_F(ManifestParserTest, ParseComponentWithoutIdentityIsEmpty) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "plain"
    name: "Plain"
)";
  auto manifest = parser_.parse_string(yaml);
  ASSERT_EQ(manifest.components.size(), 1);
  EXPECT_TRUE(manifest.components[0].identity.empty());
}

TEST_F(ManifestParserTest, ParseApps) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
    name: "Navigation 2"
    is_located_on: "navigation_server"
    description: "Navigation stack"
    depends_on:
      - localization
      - mapping
    tags:
      - navigation
    ros_binding:
      node_name: "nav2_controller"
      namespace: "/nav2"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);

  const auto & app = manifest.apps[0];
  EXPECT_EQ(app.id, "nav2");
  EXPECT_EQ(app.name, "Navigation 2");
  EXPECT_EQ(app.component_id, "navigation_server");
  EXPECT_EQ(app.description, "Navigation stack");
  EXPECT_EQ(app.depends_on.size(), 2);
  EXPECT_EQ(app.source, "manifest");

  ASSERT_TRUE(app.ros_binding.has_value());
  EXPECT_EQ(app.ros_binding->node_name, "nav2_controller");
  EXPECT_EQ(app.ros_binding->namespace_pattern, "/nav2");
}

TEST_F(ManifestParserTest, ParseAppExternal) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "external_api"
    name: "External API"
    external: true
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);
  ASSERT_TRUE(manifest.apps[0].external.has_value());
  EXPECT_TRUE(manifest.apps[0].external.value());
}

TEST_F(ManifestParserTest, ParseAppExternalOmittedStaysUnset) {
  // Tri-state: an absent `external:` key must leave the classification unset
  // (nullopt), not collapse to false - otherwise a manifest stub would erase a
  // plugin's introspected classification in the hybrid merge (#517).
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "ros_node"
    name: "ROS Node"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);
  EXPECT_FALSE(manifest.apps[0].external.has_value());
}

TEST_F(ManifestParserTest, ParseAppExternalExplicitFalseIsPreserved) {
  // An explicit `external: false` is authoritative and must be distinguishable
  // from omission, so the merge can let it override a wrong plugin `true`.
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "ros_node"
    name: "ROS Node"
    external: false
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);
  ASSERT_TRUE(manifest.apps[0].external.has_value());
  EXPECT_FALSE(manifest.apps[0].external.value());
}

TEST_F(ManifestParserTest, ParseComponent_ExternalKeyTriState) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata: { name: t, version: "1.0.0" }
components:
  - id: plc
    external: true
  - id: ros_node
    external: false
  - id: unclassified
)";
  auto manifest = parser_.parse_string(yaml);
  ASSERT_EQ(manifest.components.size(), 3u);
  auto by_id = [&](const std::string & id) -> const Component & {
    for (const auto & c : manifest.components) {
      if (c.id == id) {
        return c;
      }
    }
    ADD_FAILURE() << "component not found: " << id;
    static Component none;
    return none;
  };
  EXPECT_TRUE(by_id("plc").external.value_or(false));
  ASSERT_TRUE(by_id("ros_node").external.has_value());
  EXPECT_FALSE(by_id("ros_node").external.value());
  EXPECT_FALSE(by_id("unclassified").external.has_value());
}

TEST_F(ManifestParserTest, ParseFunctions) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "path_planning"
    name: "Path Planning"
    description: "Computes optimal paths"
    hosts:
      - nav2_planner
      - nav2_controller
    depends_on:
      - localization
    tags:
      - planning
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.functions.size(), 1);

  const auto & func = manifest.functions[0];
  EXPECT_EQ(func.id, "path_planning");
  EXPECT_EQ(func.name, "Path Planning");
  EXPECT_EQ(func.hosts.size(), 2);
  EXPECT_EQ(func.depends_on.size(), 1);
  EXPECT_EQ(func.source, "manifest");
}

TEST_F(ManifestParserTest, ParseFullManifest) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Robot System"
  version: "1.0.0"
config:
  unmanifested_nodes: "include_as_orphan"
areas:
  - id: "navigation"
  - id: "perception"
components:
  - id: "nav_server"
    area: "navigation"
  - id: "lidar_driver"
    area: "perception"
apps:
  - id: "nav2"
    is_located_on: "nav_server"
functions:
  - id: "path_planning"
    hosts:
      - nav_server
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.areas.size(), 2);
  EXPECT_EQ(manifest.components.size(), 2);
  EXPECT_EQ(manifest.apps.size(), 1);
  EXPECT_EQ(manifest.functions.size(), 1);
  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN);
}

// =============================================================================
// Top-level `config:` key, deprecated `discovery:` alias, parse notices
// =============================================================================

namespace {

constexpr const char * kDeprecatedDiscoveryNotice =
    "Manifest uses the deprecated top-level 'discovery:' key for discovery configuration; rename it to 'config:'. "
    "The alias still works and will be removed in a future release.";

constexpr const char * kBothKeysNotice =
    "Manifest declares both 'config:' and the deprecated 'discovery:' top-level keys; 'config:' is used and "
    "'discovery:' is ignored.";

std::string unknown_key_notice(const std::string & key) {
  return "Manifest has unknown top-level key '" + key +
         "' - it is ignored. Known keys: manifest_version, metadata, config, discovery, areas, components, assets, "
         "apps, functions, scripts, capabilities.";
}

/// Count how many notices contain `needle`. Substring rather than equality so
/// a caller can look for one key inside a set of notices.
size_t count_containing(const std::vector<std::string> & notices, const std::string & needle) {
  size_t n = 0;
  for (const auto & notice : notices) {
    if (notice.find(needle) != std::string::npos) {
      ++n;
    }
  }
  return n;
}

}  // namespace

TEST_F(ManifestParserTest, DeprecatedDiscoveryKeyStillAppliesItsValuesAndIsNoticed) {
  const std::string yaml = R"(
manifest_version: "1.0"
discovery:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: false
  allow_manifest_override: false
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(manifest.config.inherit_runtime_resources);
  EXPECT_FALSE(manifest.config.allow_manifest_override);
  ASSERT_EQ(manifest.log_notices.size(), 1u);
  EXPECT_EQ(manifest.log_notices[0], kDeprecatedDiscoveryNotice);
  // A deprecation must never reach the validator: strict validation is on by
  // default and would turn it into a load failure.
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, ConfigKeyWinsWhenBothBlocksArePresent) {
  const std::string yaml = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: true
discovery:
  unmanifested_nodes: "error"
  inherit_runtime_resources: false
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_TRUE(manifest.config.inherit_runtime_resources);
  ASSERT_EQ(manifest.log_notices.size(), 1u);
  EXPECT_EQ(manifest.log_notices[0], kBothKeysNotice);
}

TEST_F(ManifestParserTest, DeprecatedDiscoveryKeyAlsoReportsAnUnknownPolicyValue) {
  const std::string yaml = R"(
manifest_version: "1.0"
discovery:
  unmanifested_nodes: "quiet"
)";

  auto manifest = parser_.parse_string(yaml);

  // Advisory this release, so it lands on the log-only channel. The alias adds
  // its own deprecation notice there too, hence "at least one".
  EXPECT_TRUE(manifest.validation_notices.empty());
  EXPECT_TRUE(find_notice(manifest, "R014").has_value())
      << "the alias must report a bad policy value just as `config:` does";
}

TEST_F(ManifestParserTest, UnknownTopLevelKeyIsNoticedAndTheManifestStillLoads) {
  const std::string yaml = R"(
manifest_version: "1.0"
configuration:
  unmanifested_nodes: "ignore"
apps:
  - id: "nav2"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_TRUE(manifest.is_loaded());
  EXPECT_EQ(manifest.apps.size(), 1u);
  // The typo'd block is ignored, so the defaults stand.
  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  ASSERT_EQ(manifest.log_notices.size(), 1u);
  EXPECT_EQ(manifest.log_notices[0], unknown_key_notice("configuration"));
}

TEST_F(ManifestParserTest, EveryUnknownTopLevelKeyGetsItsOwnNotice) {
  const std::string yaml = R"(
manifest_version: "1.0"
configuration: {}
app:
  - id: "nav2"
area: []
lock_overrides: {}
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.log_notices.size(), 4u) << "expected one notice per unknown key, got: " << [&] {
    std::string joined;
    for (const auto & notice : manifest.log_notices) {
      joined += "\n  " + notice;
    }
    return joined;
  }();
  EXPECT_EQ(count_containing(manifest.log_notices, "'configuration'"), 1u);
  EXPECT_EQ(count_containing(manifest.log_notices, "'app'"), 1u);
  EXPECT_EQ(count_containing(manifest.log_notices, "'area'"), 1u);
  EXPECT_EQ(count_containing(manifest.log_notices, "'lock_overrides'"), 1u);
}

TEST_F(ManifestParserTest, EveryKnownTopLevelKeyIsAccepted) {
  // Every section parse_string reads, in one document. If a section is ever
  // added to the parser without being added to the known-key set, this test
  // starts reporting it as unknown.
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Everything"
config:
  unmanifested_nodes: "warn"
discovery:
  unmanifested_nodes: "warn"
areas:
  - id: "navigation"
components:
  - id: "nav_server"
    area: "navigation"
assets:
  - id: "cabinet-1"
apps:
  - id: "nav2"
    is_located_on: "nav_server"
functions:
  - id: "path_planning"
scripts:
  - id: "diag"
    path: "/opt/diag.sh"
    format: "bash"
capabilities:
  nav2:
    data: true
)";

  auto manifest = parser_.parse_string(yaml);

  // Only the both-keys collision notice; nothing reported as unknown.
  EXPECT_EQ(count_containing(manifest.log_notices, "unknown top-level key"), 0u);
}

TEST_F(ManifestParserTest, AllUnmanifestedNodePoliciesParseFromConfigBlock) {
  const struct {
    const char * text;
    ManifestConfig::UnmanifestedNodePolicy expected;
  } cases[] = {
      {"ignore", ManifestConfig::UnmanifestedNodePolicy::IGNORE},
      {"warn", ManifestConfig::UnmanifestedNodePolicy::WARN},
      {"error", ManifestConfig::UnmanifestedNodePolicy::ERROR},
      {"include_as_orphan", ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN},
  };

  for (const auto & tc : cases) {
    const std::string yaml =
        std::string("manifest_version: \"1.0\"\nconfig:\n  unmanifested_nodes: \"") + tc.text + "\"\n";
    auto manifest = parser_.parse_string(yaml);
    EXPECT_EQ(manifest.config.unmanifested_nodes, tc.expected) << "value: " << tc.text;
    EXPECT_TRUE(manifest.validation_notices.empty()) << "value: " << tc.text;
    EXPECT_TRUE(manifest.log_notices.empty()) << "value: " << tc.text;
  }
}

TEST_F(ManifestParserTest, UnknownPolicyValueProducesAnR014Notice) {
  const std::string yaml = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: "IGNORE"
)";

  auto manifest = parser_.parse_string(yaml);

  // Falls back to the default policy...
  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  // ...and says so. ADVISORY for this release: it must NOT reach
  // validation_notices, because the strictness dial defaults to true and would
  // refuse a manifest that loaded fine before this branch made the block read.
  EXPECT_TRUE(manifest.validation_notices.empty()) << "R014 must not be able to fail the load in this release";
  const auto notice = find_notice(manifest, "R014");
  ASSERT_TRUE(notice.has_value());
  // The value list says "lower-case" because that is the trap this very case
  // is: 'IGNORE' is a real policy word in the wrong case.
  EXPECT_EQ(*notice,
            "Unknown unmanifested_nodes value 'IGNORE'; falling back to 'warn'. "
            "Valid values: ignore, warn, error, include_as_orphan (lower-case).");
}

TEST_F(ManifestParserTest, AbsentConfigBlockUsesDefaults) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_TRUE(manifest.config.inherit_runtime_resources);
  EXPECT_TRUE(manifest.config.allow_manifest_override);
  EXPECT_TRUE(manifest.log_notices.empty());
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, EmptyConfigBlockUsesDefaults) {
  const std::string yaml = R"(
manifest_version: "1.0"
config: {}
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_TRUE(manifest.config.inherit_runtime_resources);
  EXPECT_TRUE(manifest.config.allow_manifest_override);
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, EmptyPolicyStringUsesTheDefaultWithoutANotice) {
  const std::string yaml = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: ""
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_TRUE(manifest.validation_notices.empty()) << "an omitted value is not a wrong value";
}

TEST_F(ManifestParserTest, BothBooleanFlagsSweepBothValues) {
  for (bool inherit : {false, true}) {
    for (bool allow_override : {false, true}) {
      const std::string yaml = std::string("manifest_version: \"1.0\"\nconfig:\n  inherit_runtime_resources: ") +
                               (inherit ? "true" : "false") +
                               "\n  allow_manifest_override: " + (allow_override ? "true" : "false") + "\n";
      auto manifest = parser_.parse_string(yaml);
      EXPECT_EQ(manifest.config.inherit_runtime_resources, inherit)
          << "inherit=" << inherit << " allow_override=" << allow_override;
      EXPECT_EQ(manifest.config.allow_manifest_override, allow_override)
          << "inherit=" << inherit << " allow_override=" << allow_override;
    }
  }
}

// =============================================================================
// Script Parsing Tests
// =============================================================================

// @verifies REQ_INTEROP_041
TEST_F(ManifestParserTest, ParseScripts) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "run-diagnostics"
    name: "Run Diagnostics"
    description: "Check health of all sensors"
    path: "/opt/scripts/run-diagnostics.sh"
    format: "bash"
    timeout_sec: 30
    entity_filter:
      - "components/*"
      - "apps/*"
    env:
      GATEWAY_URL: "http://localhost:8080"
      LOG_LEVEL: "debug"
  - id: "inject-nan"
    name: "Inject NaN Fault"
    path: "/opt/scripts/inject-nan.sh"
    format: "bash"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 2);

  const auto & s1 = manifest.scripts[0];
  EXPECT_EQ(s1.id, "run-diagnostics");
  EXPECT_EQ(s1.name, "Run Diagnostics");
  EXPECT_EQ(s1.description, "Check health of all sensors");
  EXPECT_EQ(s1.path, "/opt/scripts/run-diagnostics.sh");
  EXPECT_EQ(s1.format, "bash");
  EXPECT_EQ(s1.timeout_sec, 30);
  ASSERT_EQ(s1.entity_filter.size(), 2);
  EXPECT_EQ(s1.entity_filter[0], "components/*");
  EXPECT_EQ(s1.entity_filter[1], "apps/*");
  ASSERT_EQ(s1.env.size(), 2);
  EXPECT_EQ(s1.env.at("GATEWAY_URL"), "http://localhost:8080");
  EXPECT_EQ(s1.env.at("LOG_LEVEL"), "debug");

  const auto & s2 = manifest.scripts[1];
  EXPECT_EQ(s2.id, "inject-nan");
  EXPECT_EQ(s2.name, "Inject NaN Fault");
  EXPECT_EQ(s2.path, "/opt/scripts/inject-nan.sh");
  EXPECT_EQ(s2.format, "bash");
  EXPECT_EQ(s2.timeout_sec, 300);  // default
  EXPECT_TRUE(s2.entity_filter.empty());
  EXPECT_TRUE(s2.env.empty());
}

TEST_F(ManifestParserTest, ParseScriptsWithArgs) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "calibrate"
    path: "/opt/scripts/calibrate.py"
    format: "python"
    args:
      - name: "threshold"
        type: "float"
        flag: "--threshold"
      - name: "verbose"
        type: "bool"
        flag: "-v"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  const auto & s = manifest.scripts[0];
  EXPECT_EQ(s.name, "calibrate");  // defaults to id
  ASSERT_TRUE(s.args.is_array());
  ASSERT_EQ(s.args.size(), 2);
  EXPECT_EQ(s.args[0]["name"], "threshold");
  EXPECT_EQ(s.args[0]["type"], "float");
  EXPECT_EQ(s.args[0]["flag"], "--threshold");
  EXPECT_EQ(s.args[1]["name"], "verbose");
}

TEST_F(ManifestParserTest, ParseScriptsWithParametersSchema) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "test-script"
    path: "/opt/scripts/test.sh"
    format: "bash"
    parameters_schema:
      type: "object"
      required: "threshold"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  ASSERT_TRUE(manifest.scripts[0].parameters_schema.has_value());
  EXPECT_EQ(manifest.scripts[0].parameters_schema->at("type"), "object");
}

TEST_F(ManifestParserTest, ParseScriptsParametersSchemaPreservesNestedObjects) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "nested-schema"
    path: "/opt/scripts/test.sh"
    format: "bash"
    parameters_schema:
      type: "object"
      properties:
        threshold:
          type: "number"
      required:
        - threshold
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  ASSERT_TRUE(manifest.scripts[0].parameters_schema.has_value());
  const auto & schema = *manifest.scripts[0].parameters_schema;
  EXPECT_EQ(schema.at("type"), "object");
  // Nested objects are preserved via recursive YAML-to-JSON conversion
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_EQ(schema["properties"]["threshold"]["type"], "number");
  // Arrays are preserved
  ASSERT_TRUE(schema.contains("required"));
  ASSERT_TRUE(schema["required"].is_array());
  EXPECT_EQ(schema["required"][0], "threshold");
}

// @verifies REQ_INTEROP_041
TEST_F(ManifestParserTest, ParseScriptsMissingId) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - name: "No ID Script"
    path: "/opt/scripts/test.sh"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

// @verifies REQ_INTEROP_041
TEST_F(ManifestParserTest, ParseScriptsMissingPath) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "no-path"
    format: "bash"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

// @verifies REQ_INTEROP_041
TEST_F(ManifestParserTest, ParseScriptsMissingFormat) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "no-format"
    path: "/opt/scripts/test.sh"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

// @verifies REQ_INTEROP_041
TEST_F(ManifestParserTest, ParseScriptsTimeoutClampedToMinimum) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "clamped"
    path: "/opt/scripts/test.sh"
    format: "bash"
    timeout_sec: 0
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  EXPECT_EQ(manifest.scripts[0].timeout_sec, 1);  // clamped from 0
}

TEST_F(ManifestParserTest, ParseScriptsUnknownFormat) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "bad-format"
    path: "/opt/scripts/test.sh"
    format: "ruby"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

TEST_F(ManifestParserTest, ParseNoScriptsSection) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "test-area"
)";

  auto manifest = parser_.parse_string(yaml);
  EXPECT_TRUE(manifest.scripts.empty());
}

// =============================================================================
// Error Cases Tests
// =============================================================================

TEST_F(ManifestParserTest, ThrowsOnMissingVersion) {
  const std::string yaml = R"(
metadata:
  name: "Test"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

TEST_F(ManifestParserTest, ThrowsOnMalformedYaml) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: unclosed_string
    name: [invalid
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

// =============================================================================
// ManifestConfig Policy Tests
// =============================================================================

TEST(ManifestConfigTest, ParsePolicyStrings) {
  EXPECT_EQ(ManifestConfig::parse_policy("ignore"), ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_EQ(ManifestConfig::parse_policy("warn"), ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_EQ(ManifestConfig::parse_policy("error"), ManifestConfig::UnmanifestedNodePolicy::ERROR);
  EXPECT_EQ(ManifestConfig::parse_policy("include_as_orphan"),
            ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN);
  // Unknown defaults to WARN
  EXPECT_EQ(ManifestConfig::parse_policy("unknown"), ManifestConfig::UnmanifestedNodePolicy::WARN);
}

TEST(ManifestConfigTest, PolicyToString) {
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::IGNORE), "ignore");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::WARN), "warn");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::ERROR), "error");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN),
            "include_as_orphan");
}

// =============================================================================
// Malformed discovery-configuration block
//
// The block is addressed by key and then indexed by field. A key that is
// present but not a map cannot be indexed: yaml-cpp throws on a scalar, and
// silently yields nothing on a sequence. Neither may reach the field parser.
// Both spellings of the key are guarded, because both are read.
// =============================================================================

namespace {

// A manifest whose only variable is the value of the discovery-config block.
std::string manifest_with_config_block(const std::string & key, const std::string & value) {
  return "manifest_version: \"1.0\"\n" + key + ":" + value +
         "\ncomponents:\n"
         "  - id: mc-ecu\n"
         "    name: \"Malformed Config ECU\"\n";
}

}  // namespace

// A key with no value is "absent", not "malformed". This must keep working:
// it loads on the merge base and it loads here.
TEST_F(ManifestParserTest, NullConfigBlockIsTreatedAsAbsent) {
  for (const std::string & key : {std::string("config"), std::string("discovery")}) {
    auto manifest = parser_.parse_string(manifest_with_config_block(key, ""));

    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN) << key;
    EXPECT_TRUE(manifest.config.inherit_runtime_resources) << key;
    EXPECT_TRUE(manifest.config.allow_manifest_override) << key;
    EXPECT_FALSE(find_notice(manifest, "R015").has_value()) << key << ": a null block is absent, not malformed";
    ASSERT_EQ(manifest.components.size(), 1u) << key;
  }
}

TEST_F(ManifestParserTest, EmptyMapConfigBlockIsTreatedAsAbsent) {
  for (const std::string & key : {std::string("config"), std::string("discovery")}) {
    auto manifest = parser_.parse_string(manifest_with_config_block(key, " {}"));

    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN) << key;
    EXPECT_FALSE(find_notice(manifest, "R015").has_value()) << key;
    ASSERT_EQ(manifest.components.size(), 1u) << key;
  }
}

// The regression: a scalar value made parse_config throw out of the whole
// load, so the manifest was lost entirely rather than the block reported.
TEST_F(ManifestParserTest, ScalarConfigBlockIsReportedAndDoesNotAbortTheLoad) {
  for (const std::string & key : {std::string("config"), std::string("discovery")}) {
    ros2_medkit_gateway::discovery::Manifest manifest;
    ASSERT_NO_THROW(manifest = parser_.parse_string(manifest_with_config_block(key, " \"\"")))
        << key << ": a malformed block must not throw the manifest away";

    const auto notice = find_notice(manifest, "R015");
    ASSERT_TRUE(notice.has_value()) << key << ": a malformed block must be reported, not ignored";
    EXPECT_EQ(*notice, "Manifest key '" + key +
                           "' must be a mapping of discovery settings, but is a scalar; "
                           "the block is ignored and the defaults apply.");

    // The rest of the manifest still loads, and the settings fall back.
    ASSERT_EQ(manifest.components.size(), 1u) << key;
    EXPECT_EQ(manifest.components[0].id, "mc-ecu") << key;
    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN) << key;
  }
}

// A sequence never threw - it was indexed, yielded nothing and was silently
// dropped. Silent-drop of a configuration block is the bug class this branch
// exists to close, so it is reported too.
TEST_F(ManifestParserTest, SequenceConfigBlockIsReportedRatherThanSilentlyIgnored) {
  for (const std::string & key : {std::string("config"), std::string("discovery")}) {
    ros2_medkit_gateway::discovery::Manifest manifest;
    ASSERT_NO_THROW(manifest =
                        parser_.parse_string(manifest_with_config_block(key, "\n  - unmanifested_nodes: ignore")));

    const auto notice = find_notice(manifest, "R015");
    ASSERT_TRUE(notice.has_value()) << key << ": a sequence must not be dropped without a word";
    EXPECT_EQ(*notice, "Manifest key '" + key +
                           "' must be a mapping of discovery settings, but is a sequence; "
                           "the block is ignored and the defaults apply.");
    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN) << key;
  }
}

// A malformed canonical key must not silently promote the deprecated alias:
// `config:` present means `config:` is the block, well-formed or not.
TEST_F(ManifestParserTest, MalformedConfigDoesNotFallThroughToTheAlias) {
  const std::string yaml =
      "manifest_version: \"1.0\"\n"
      "config: \"\"\n"
      "discovery:\n"
      "  unmanifested_nodes: ignore\n";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_TRUE(find_notice(manifest, "R015").has_value());
  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN)
      << "the alias must not quietly supply the configuration the canonical key failed to";
}

// =============================================================================
// Malformed VALUES inside the discovery-configuration block
//
// The block guard above only proves the block is a mapping. Each setting
// inside it is read by kind too: `.as<bool>()` throws on anything it cannot
// convert, and a key left empty renders as the literal string "null" if it is
// read as text. Either would cost the whole manifest, or accuse the operator
// of a typo they did not make.
// =============================================================================

namespace {

std::string manifest_with_setting(const std::string & key, const std::string & value) {
  return "manifest_version: \"1.0\"\nconfig:\n  " + key + ":" + value +
         "\ncomponents:\n"
         "  - id: mc-ecu\n"
         "    name: \"Malformed Config ECU\"\n";
}

}  // namespace

// A key with no value is "not set". This is the case the block-level comment
// promised and the one all three settings used to get wrong.
TEST_F(ManifestParserTest, NullSettingValuesAreTreatedAsNotSet) {
  for (const std::string & key : {std::string("unmanifested_nodes"), std::string("inherit_runtime_resources"),
                                  std::string("allow_manifest_override")}) {
    ros2_medkit_gateway::discovery::Manifest manifest;
    ASSERT_NO_THROW(manifest = parser_.parse_string(manifest_with_setting(key, "")))
        << key << ": an empty value must not throw the manifest away";

    EXPECT_TRUE(manifest.validation_notices.empty())
        << key << ": an empty value is 'not set', not a wrong value; got "
        << (manifest.validation_notices.empty() ? std::string{} : manifest.validation_notices[0].message);
    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN) << key;
    EXPECT_TRUE(manifest.config.inherit_runtime_resources) << key;
    EXPECT_TRUE(manifest.config.allow_manifest_override) << key;
    ASSERT_EQ(manifest.components.size(), 1u) << key;
  }
}

TEST_F(ManifestParserTest, WellFormedSettingValuesStillParse) {
  auto manifest = parser_.parse_string(
      "manifest_version: \"1.0\"\n"
      "config:\n"
      "  unmanifested_nodes: ignore\n"
      "  inherit_runtime_resources: false\n"
      "  allow_manifest_override: false\n");

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(manifest.config.inherit_runtime_resources);
  EXPECT_FALSE(manifest.config.allow_manifest_override);
  EXPECT_TRUE(manifest.validation_notices.empty());
}

TEST_F(ManifestParserTest, NonBooleanSettingValuesAreReportedAndDefaulted) {
  struct Case {
    std::string value;
    std::string expected_kind;
  };
  const std::vector<Case> cases = {
      {" \"yes-please\"", "'yes-please'"},
      {"\n    - true", "a sequence"},
      {"\n    enabled: true", "a mapping"},
  };

  for (const std::string & key : {std::string("inherit_runtime_resources"), std::string("allow_manifest_override")}) {
    for (const auto & tc : cases) {
      ros2_medkit_gateway::discovery::Manifest manifest;
      ASSERT_NO_THROW(manifest = parser_.parse_string(manifest_with_setting(key, tc.value)))
          << key << " / " << tc.value;

      const auto notice = find_notice(manifest, "R015");
      ASSERT_TRUE(notice.has_value()) << key << " / " << tc.value << ": must be reported, not thrown or ignored";
      EXPECT_EQ(*notice, "Manifest setting '" + key + "' must be a boolean, but is " + tc.expected_kind +
                             "; the default (true) applies.");

      // The default stands and the rest of the manifest survives.
      EXPECT_TRUE(manifest.config.inherit_runtime_resources) << key;
      EXPECT_TRUE(manifest.config.allow_manifest_override) << key;
      ASSERT_EQ(manifest.components.size(), 1u) << key;
    }
  }
}

TEST_F(ManifestParserTest, NonScalarPolicyValueIsReportedAndDefaulted) {
  for (const std::string & value : {std::string("\n    - ignore"), std::string("\n    mode: ignore")}) {
    ros2_medkit_gateway::discovery::Manifest manifest;
    ASSERT_NO_THROW(manifest = parser_.parse_string(manifest_with_setting("unmanifested_nodes", value)));

    const auto notice = find_notice(manifest, "R015");
    ASSERT_TRUE(notice.has_value()) << value;
    EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
    // A wrong KIND is R015, not R014: R014 is for a scalar that is not one of
    // the known policy words.
    EXPECT_FALSE(find_notice(manifest, "R014").has_value()) << value;
  }
}

// The policy words are lower-case. Documented, and pinned here because under
// the shipped strict default this rejects the whole manifest.
TEST_F(ManifestParserTest, PolicyValueIsCaseSensitive) {
  auto manifest = parser_.parse_string(manifest_with_setting("unmanifested_nodes", " Warn"));

  const auto notice = find_notice(manifest, "R014");
  ASSERT_TRUE(notice.has_value()) << "'Warn' is not 'warn'";
  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
}

// =============================================================================
// Precedence between `config:` and the deprecated `discovery:` alias
//
// Both keys are read, so "which one is present" has to mean the same thing to
// the precedence test as it does to the block parser. It did not: a bare
// `config:` with no value counted as present, won, then parsed to nothing -
// so a populated `discovery:` under it was skipped in silence while the log
// said "'config:' is used", which reads as confirmation it applied.
// =============================================================================

namespace {

std::string manifest_with_blocks(const std::string & config_block, const std::string & alias_block) {
  return "manifest_version: \"1.0\"\n" + config_block + alias_block +
         "components:\n"
         "  - id: prec-ecu\n"
         "    name: \"Precedence ECU\"\n";
}

// The four shapes the canonical key can take.
const std::string kConfigAbsent;
const std::string kConfigNull = "config:\n";
const std::string kConfigEmptyMap = "config: {}\n";
const std::string kConfigPopulated = "config:\n  unmanifested_nodes: ignore\n";

const std::string kAliasAbsent;
const std::string kAliasPopulated = "discovery:\n  unmanifested_nodes: error\n";

bool has_collision_notice(const ros2_medkit_gateway::discovery::Manifest & manifest) {
  for (const auto & notice : manifest.log_notices) {
    if (notice.find("declares both") != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST_F(ManifestParserTest, ConfigAliasPrecedenceSweepsAllCombinations) {
  using Policy = ManifestConfig::UnmanifestedNodePolicy;
  struct Case {
    std::string name;
    std::string config_block;
    std::string alias_block;
    Policy expected;
    bool expect_collision_notice;
  };

  const std::vector<Case> cases = {
      // No canonical key: the alias applies, or nothing does.
      {"absent/absent", kConfigAbsent, kAliasAbsent, Policy::WARN, false},
      {"absent/alias", kConfigAbsent, kAliasPopulated, Policy::ERROR, false},
      // A null canonical key is NOT a configuration. It must not shadow the
      // alias, and it must not claim a collision.
      {"null/absent", kConfigNull, kAliasAbsent, Policy::WARN, false},
      {"null/alias", kConfigNull, kAliasPopulated, Policy::ERROR, false},
      // An empty MAP is a real (if empty) configuration block: it wins, and
      // the defaults apply.
      {"emptymap/absent", kConfigEmptyMap, kAliasAbsent, Policy::WARN, false},
      {"emptymap/alias", kConfigEmptyMap, kAliasPopulated, Policy::WARN, true},
      // A populated canonical key wins outright.
      {"populated/absent", kConfigPopulated, kAliasAbsent, Policy::IGNORE, false},
      {"populated/alias", kConfigPopulated, kAliasPopulated, Policy::IGNORE, true},
  };

  for (const auto & tc : cases) {
    auto manifest = parser_.parse_string(manifest_with_blocks(tc.config_block, tc.alias_block));

    EXPECT_EQ(manifest.config.unmanifested_nodes, tc.expected) << tc.name;
    EXPECT_EQ(has_collision_notice(manifest), tc.expect_collision_notice)
        << tc.name << ": the collision notice must fire only when `config:` really carries a block";
    ASSERT_EQ(manifest.components.size(), 1u) << tc.name;
  }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
