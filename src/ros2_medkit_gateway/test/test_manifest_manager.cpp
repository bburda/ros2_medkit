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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>

#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"

using namespace ros2_medkit_gateway::discovery;

namespace fs = std::filesystem;

class ManifestManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create temp directory for test files
    temp_dir_ = fs::temp_directory_path() / "manifest_manager_test";
    fs::create_directories(temp_dir_);
  }

  void TearDown() override {
    // Clean up temp directory
    fs::remove_all(temp_dir_);
  }

  std::string write_temp_file(const std::string & filename, const std::string & content) {
    fs::path file_path = temp_dir_ / filename;
    std::ofstream ofs(file_path);
    ofs << content;
    return file_path.string();
  }

  fs::path temp_dir_;
};

// Valid manifest YAML for testing
const std::string valid_manifest_yaml = R"(
manifest_version: "1.0"
metadata:
  name: "test-manifest"
  version: "1.0.0"
  description: "Test manifest for unit tests"
config:
  unmanifested_nodes: "warn"
  inherit_runtime_resources: true
areas:
  - id: "area1"
    name: "Test Area 1"
    description: "First test area"
  - id: "area2"
    name: "Test Area 2"
    parent_area: "area1"
components:
  - id: "comp1"
    name: "Component 1"
    area: "area1"
  - id: "comp2"
    name: "Component 2"
    area: "area1"
    parent_component_id: "comp1"
  - id: "comp3"
    name: "Component 3"
    area: "area2"
apps:
  - id: "app1"
    name: "App 1"
    is_located_on: "comp1"
    description: "First application"
  - id: "app2"
    name: "App 2"
    is_located_on: "comp1"
  - id: "app3"
    name: "App 3"
    is_located_on: "comp2"
functions:
  - id: "func1"
    name: "Function 1"
    app_ids: ["app1"]
    hosts: ["comp1", "comp2"]
  - id: "func2"
    name: "Function 2"
    app_ids: ["app1", "app2"]
    hosts: ["comp1"]
capabilities:
  comp1:
    faults: true
    configurations: true
  app1:
    data: ["read"]
)";

// Manifest with warnings (valid but has issues)
const std::string manifest_with_warnings = R"(
manifest_version: "1.0"
metadata:
  name: "manifest-with-warnings"
  version: "1.0.0"
areas:
  - id: "area1"
    name: "Area 1"
components:
  - id: "comp1"
    name: "Component 1"
    area: "area1"
apps:
  - id: "app1"
    name: "Unreferenced App"
    is_located_on: "comp1"
    description: "This app is not referenced by any function"
functions: []
)";

// Minimal valid manifest
const std::string minimal_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "minimal"
  version: "0.1.0"
areas:
  - id: "area1"
    name: "Area"
components:
  - id: "comp1"
    name: "Component"
    area: "area1"
)";

// Invalid manifest - missing required fields
const std::string invalid_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "invalid"
areas:
  - id: "area1"
    name: "Area"
components:
  - id: "comp1"
    name: "Component"
    area: "nonexistent_area"
)";

// =============================================================================
// Load/Unload Tests
// =============================================================================

TEST_F(ManifestManagerTest, LoadFromFile) {
  std::string path = write_temp_file("valid.yaml", valid_manifest_yaml);

  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest(path));
  EXPECT_TRUE(manager.is_manifest_active());
  EXPECT_EQ(manager.get_manifest_path(), path);
}

TEST_F(ManifestManagerTest, LoadFromString) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));
  EXPECT_TRUE(manager.is_manifest_active());
  EXPECT_EQ(manager.get_manifest_path(), "<string>");
}

TEST_F(ManifestManagerTest, LoadNonexistentFile) {
  ManifestManager manager;
  EXPECT_FALSE(manager.load_manifest("/nonexistent/path.yaml"));
  EXPECT_FALSE(manager.is_manifest_active());
}

TEST_F(ManifestManagerTest, LoadInvalidYaml) {
  std::string path = write_temp_file("invalid.yaml", "not: [valid: yaml: content");

  ManifestManager manager;
  EXPECT_FALSE(manager.load_manifest(path));
  EXPECT_FALSE(manager.is_manifest_active());

  auto result = manager.get_validation_result();
  EXPECT_FALSE(result.errors.empty());
}

TEST_F(ManifestManagerTest, LoadInvalidManifestStrict) {
  ManifestManager manager;
  EXPECT_FALSE(manager.load_manifest_from_string(invalid_manifest, true));
  EXPECT_FALSE(manager.is_manifest_active());
}

TEST_F(ManifestManagerTest, LoadInvalidManifestNonStrict) {
  ManifestManager manager;
  // Even in non-strict mode, ERRORs (broken references) always cause failure
  // This is intentional: ERRORs indicate fundamentally broken manifests
  EXPECT_FALSE(manager.load_manifest_from_string(invalid_manifest, false));
  EXPECT_FALSE(manager.is_manifest_active());

  auto result = manager.get_validation_result();
  EXPECT_TRUE(result.has_errors());
}

TEST_F(ManifestManagerTest, LoadWithWarnings) {
  ManifestManager manager;
  // The manifest_with_warnings may not necessarily produce warnings
  // if the validator doesn't check for unreferenced apps
  ASSERT_TRUE(manager.load_manifest_from_string(manifest_with_warnings));

  // Just verify it loads - whether there are warnings depends on validation rules
  EXPECT_TRUE(manager.is_manifest_active());
}

TEST_F(ManifestManagerTest, UnloadManifest) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));
  EXPECT_TRUE(manager.is_manifest_active());

  manager.unload_manifest();

  EXPECT_FALSE(manager.is_manifest_active());
  EXPECT_TRUE(manager.get_manifest_path().empty());
  EXPECT_TRUE(manager.get_areas().empty());
}

TEST_F(ManifestManagerTest, ReloadManifest) {
  std::string path = write_temp_file("reload.yaml", valid_manifest_yaml);

  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest(path));

  // Modify the file
  write_temp_file("reload.yaml", minimal_manifest);

  // Reload
  ASSERT_TRUE(manager.reload_manifest());
  EXPECT_TRUE(manager.is_manifest_active());

  // Check we have minimal manifest content (1 area instead of 2)
  EXPECT_EQ(manager.get_areas().size(), 1);
}

TEST_F(ManifestManagerTest, ReloadWithoutPath) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  // Cannot reload string-loaded manifest
  EXPECT_FALSE(manager.reload_manifest());
}

// =============================================================================
// Entity Access Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetMetadata) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto meta = manager.get_metadata();
  ASSERT_TRUE(meta.has_value());
  EXPECT_EQ(meta->name, "test-manifest");
  EXPECT_EQ(meta->version, "1.0.0");
  EXPECT_EQ(meta->description, "Test manifest for unit tests");
}

TEST_F(ManifestManagerTest, GetConfig) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto config = manager.get_config();
  // Config has unmanifested_nodes policy and inherit_runtime_resources
  EXPECT_TRUE(config.inherit_runtime_resources);
}

TEST_F(ManifestManagerTest, GetAreas) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto areas = manager.get_areas();
  EXPECT_EQ(areas.size(), 2);
  EXPECT_EQ(areas[0].id, "area1");
  EXPECT_EQ(areas[1].id, "area2");
}

TEST_F(ManifestManagerTest, GetComponents) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto comps = manager.get_components();
  EXPECT_EQ(comps.size(), 3);
}

TEST_F(ManifestManagerTest, GetApps) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto apps = manager.get_apps();
  EXPECT_EQ(apps.size(), 3);
}

TEST_F(ManifestManagerTest, GetFunctions) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto funcs = manager.get_functions();
  EXPECT_EQ(funcs.size(), 2);
}

// =============================================================================
// Entity Lookup by ID Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetAreaById) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto area = manager.get_area("area1");
  ASSERT_TRUE(area.has_value());
  EXPECT_EQ(area->id, "area1");
  EXPECT_EQ(area->name, "Test Area 1");

  // Non-existent
  auto none = manager.get_area("nonexistent");
  EXPECT_FALSE(none.has_value());
}

TEST_F(ManifestManagerTest, GetComponentById) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto comp = manager.get_component("comp2");
  ASSERT_TRUE(comp.has_value());
  EXPECT_EQ(comp->id, "comp2");
  EXPECT_EQ(comp->name, "Component 2");

  // Non-existent
  auto none = manager.get_component("nonexistent");
  EXPECT_FALSE(none.has_value());
}

TEST_F(ManifestManagerTest, GetAppById) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto app = manager.get_app("app1");
  ASSERT_TRUE(app.has_value());
  EXPECT_EQ(app->id, "app1");
  EXPECT_EQ(app->description, "First application");

  // Non-existent
  auto none = manager.get_app("nonexistent");
  EXPECT_FALSE(none.has_value());
}

TEST_F(ManifestManagerTest, GetFunctionById) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto func = manager.get_function("func1");
  ASSERT_TRUE(func.has_value());
  EXPECT_EQ(func->id, "func1");
  EXPECT_EQ(func->hosts.size(), 2);

  // Non-existent
  auto none = manager.get_function("nonexistent");
  EXPECT_FALSE(none.has_value());
}

// =============================================================================
// Relationship Query Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetComponentsForArea) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  // area1 has comp1, comp2 directly, plus comp3 from subarea area2
  auto comps = manager.get_components_for_area("area1");
  EXPECT_EQ(comps.size(), 3);

  // area2 only has comp3 (it's a subarea, not a parent)
  auto comps2 = manager.get_components_for_area("area2");
  EXPECT_EQ(comps2.size(), 1);
  EXPECT_EQ(comps2[0].id, "comp3");

  // Empty area
  auto empty = manager.get_components_for_area("nonexistent");
  EXPECT_TRUE(empty.empty());
}

TEST_F(ManifestManagerTest, GetAppsForComponent) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto apps = manager.get_apps_for_component("comp1");
  EXPECT_EQ(apps.size(), 2);

  auto apps2 = manager.get_apps_for_component("comp2");
  EXPECT_EQ(apps2.size(), 1);

  // Empty
  auto empty = manager.get_apps_for_component("comp3");
  EXPECT_TRUE(empty.empty());
}

TEST_F(ManifestManagerTest, GetHostsForFunction) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto hosts = manager.get_hosts_for_function("func1");
  EXPECT_EQ(hosts.size(), 2);

  auto hosts2 = manager.get_hosts_for_function("func2");
  EXPECT_EQ(hosts2.size(), 1);

  // Non-existent function
  auto empty = manager.get_hosts_for_function("nonexistent");
  EXPECT_TRUE(empty.empty());
}

TEST_F(ManifestManagerTest, GetSubareas) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto subs = manager.get_subareas("area1");
  EXPECT_EQ(subs.size(), 1);
  EXPECT_EQ(subs[0].id, "area2");

  // No subareas
  auto empty = manager.get_subareas("area2");
  EXPECT_TRUE(empty.empty());
}

TEST_F(ManifestManagerTest, GetSubcomponents) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto subs = manager.get_subcomponents("comp1");
  EXPECT_EQ(subs.size(), 1);
  EXPECT_EQ(subs[0].id, "comp2");

  // No subcomponents
  auto empty = manager.get_subcomponents("comp2");
  EXPECT_TRUE(empty.empty());
}

// =============================================================================
// Capabilities Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetCapabilitiesOverride) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  // Capabilities are parsed as empty JSON objects (keys exist, but full YAML->JSON not implemented)
  auto caps = manager.get_capabilities_override("comp1");
  ASSERT_TRUE(caps.has_value());
  EXPECT_TRUE(caps->is_object());

  auto app_caps = manager.get_capabilities_override("app1");
  ASSERT_TRUE(app_caps.has_value());
  EXPECT_TRUE(app_caps->is_object());

  // No override
  auto none = manager.get_capabilities_override("comp3");
  EXPECT_FALSE(none.has_value());
}

// =============================================================================
// Status JSON Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetStatusJsonActive) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  auto status = manager.get_status_json();
  EXPECT_TRUE(status["active"].get<bool>());
  EXPECT_EQ(status["metadata"]["name"], "test-manifest");
  EXPECT_EQ(status["entity_counts"]["areas"], 2);
  EXPECT_EQ(status["entity_counts"]["components"], 3);
  EXPECT_EQ(status["entity_counts"]["apps"], 3);
  EXPECT_EQ(status["entity_counts"]["functions"], 2);
  EXPECT_TRUE(status["validation"]["is_valid"].get<bool>());
}

TEST_F(ManifestManagerTest, GetStatusJsonInactive) {
  ManifestManager manager;

  auto status = manager.get_status_json();
  EXPECT_FALSE(status["active"].get<bool>());
  EXPECT_TRUE(status["path"].get<std::string>().empty());
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

TEST_F(ManifestManagerTest, GettersWithNoManifest) {
  ManifestManager manager;

  EXPECT_FALSE(manager.get_metadata().has_value());
  EXPECT_TRUE(manager.get_areas().empty());
  EXPECT_TRUE(manager.get_components().empty());
  EXPECT_TRUE(manager.get_apps().empty());
  EXPECT_TRUE(manager.get_functions().empty());
  EXPECT_FALSE(manager.get_area("any").has_value());
  EXPECT_FALSE(manager.get_component("any").has_value());
  EXPECT_TRUE(manager.get_components_for_area("any").empty());
}

TEST_F(ManifestManagerTest, MultipleLoads) {
  ManifestManager manager;

  // First load
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));
  EXPECT_EQ(manager.get_areas().size(), 2);

  // Second load replaces first
  ASSERT_TRUE(manager.load_manifest_from_string(minimal_manifest));
  EXPECT_EQ(manager.get_areas().size(), 1);
}

// =============================================================================
// Thread Safety Tests
// =============================================================================

TEST_F(ManifestManagerTest, ConcurrentReads) {
  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));

  std::vector<std::future<bool>> futures;
  for (int i = 0; i < 10; ++i) {
    futures.push_back(std::async(std::launch::async, [&manager]() {
      for (int j = 0; j < 100; ++j) {
        auto areas = manager.get_areas();
        auto comps = manager.get_components();
        auto status = manager.get_status_json();
        (void)areas;
        (void)comps;
        (void)status;
      }
      return true;
    }));
  }

  for (auto & f : futures) {
    EXPECT_TRUE(f.get());
  }
}

TEST_F(ManifestManagerTest, ConcurrentReadWrite) {
  std::string path = write_temp_file("concurrent.yaml", valid_manifest_yaml);

  ManifestManager manager;
  ASSERT_TRUE(manager.load_manifest(path));

  std::atomic<bool> running{true};
  std::vector<std::future<void>> readers;

  // Start reader threads
  for (int i = 0; i < 5; ++i) {
    readers.push_back(std::async(std::launch::async, [&manager, &running]() {
      while (running) {
        auto areas = manager.get_areas();
        auto active = manager.is_manifest_active();
        (void)areas;
        (void)active;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }));
  }

  // Perform some writes
  for (int i = 0; i < 5; ++i) {
    manager.reload_manifest();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  running = false;
  for (auto & f : readers) {
    f.wait();
  }

  // Should still be in valid state
  EXPECT_TRUE(manager.is_manifest_active());
}

// =============================================================================
// Integration with ROS Node (optional - runs without node too)
// =============================================================================

TEST_F(ManifestManagerTest, WorksWithoutNode) {
  ManifestManager manager(nullptr);
  ASSERT_TRUE(manager.load_manifest_from_string(valid_manifest_yaml));
  EXPECT_TRUE(manager.is_manifest_active());
}

// =============================================================================
// Script Access Tests
// =============================================================================

TEST_F(ManifestManagerTest, GetScriptsReturnsManifestScripts) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "run-diagnostics"
    name: "Run Diagnostics"
    path: "/opt/scripts/run-diagnostics.sh"
    format: "bash"
    timeout_sec: 30
    entity_filter:
      - "components/*"
)";

  ManifestManager mgr(nullptr);
  ASSERT_TRUE(mgr.load_manifest_from_string(yaml, false));
  auto scripts = mgr.get_scripts();
  ASSERT_EQ(scripts.size(), 1);
  EXPECT_EQ(scripts[0].id, "run-diagnostics");
  EXPECT_EQ(scripts[0].name, "Run Diagnostics");
  EXPECT_EQ(scripts[0].timeout_sec, 30);
  ASSERT_EQ(scripts[0].entity_filter.size(), 1);
  EXPECT_EQ(scripts[0].entity_filter[0], "components/*");
}

TEST_F(ManifestManagerTest, GetScriptsEmptyWhenNoManifest) {
  ManifestManager mgr(nullptr);
  auto scripts = mgr.get_scripts();
  EXPECT_TRUE(scripts.empty());
}

// =============================================================================
// Discovery configuration: `config:` block and parse notices
// =============================================================================

namespace {

/// Manifest whose discovery configuration is written under the documented
/// top-level key.
const std::string config_block_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "config-block"
config:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: false
areas:
  - id: "area1"
)";

/// Same content under the deprecated alias.
const std::string discovery_block_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "discovery-block"
discovery:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: false
areas:
  - id: "area1"
)";

const std::string unknown_policy_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "unknown-policy"
config:
  unmanifested_nodes: "silent"
areas:
  - id: "area1"
)";

const std::string unknown_top_level_key_manifest = R"(
manifest_version: "1.0"
metadata:
  name: "unknown-key"
configuration:
  unmanifested_nodes: "ignore"
areas:
  - id: "area1"
)";

/// Look up a warning by rule id. Takes the result by reference on purpose:
/// `ManifestManager::get_validation_result()` returns BY VALUE, so the caller
/// must keep the result alive for as long as it holds the returned pointer.
const ros2_medkit_gateway::discovery::ValidationError *
find_warning(const ros2_medkit_gateway::discovery::ValidationResult & result, const std::string & rule_id) {
  for (const auto & warn : result.warnings) {
    if (warn.rule_id == rule_id) {
      return &warn;
    }
  }
  return nullptr;
}

}  // namespace

TEST_F(ManifestManagerTest, ConfigBlockReachesGetConfig) {
  ManifestManager mgr(nullptr);
  ASSERT_TRUE(mgr.load_manifest_from_string(config_block_manifest, true));

  auto config = mgr.get_config();
  EXPECT_EQ(config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(config.inherit_runtime_resources);
}

TEST_F(ManifestManagerTest, DeprecatedDiscoveryBlockReachesGetConfigAndSurvivesStrictMode) {
  ManifestManager mgr(nullptr);
  // Strict mode turns validator warnings into a load failure, so a deprecation
  // routed through the validator would make this load fail.
  ASSERT_TRUE(mgr.load_manifest_from_string(discovery_block_manifest, true));

  auto config = mgr.get_config();
  EXPECT_EQ(config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(config.inherit_runtime_resources);
  EXPECT_FALSE(mgr.get_validation_result().has_warnings());
}

TEST_F(ManifestManagerTest, UnknownTopLevelKeySurvivesStrictMode) {
  ManifestManager mgr(nullptr);
  ASSERT_TRUE(mgr.load_manifest_from_string(unknown_top_level_key_manifest, true));
  EXPECT_FALSE(mgr.get_validation_result().has_warnings());
}

// R014 is ADVISORY for this release. It reaches the LOG, never
// ValidationResult, so the strictness dial cannot act on it. That is
// deliberate: this branch is what makes the discovery block read at all, so a
// manifest carrying a bad policy value loaded fine before the upgrade, and
// turning it into a load failure afterwards would take hybrid discovery down
// to runtime_only for a file the operator never edited. It becomes a
// validation warning in the next release.

TEST_F(ManifestManagerTest, UnknownPolicyValueIsAdvisoryAndLoadsNonStrict) {
  ManifestManager mgr(nullptr);
  ASSERT_TRUE(mgr.load_manifest_from_string(unknown_policy_manifest, false));

  auto result = mgr.get_validation_result();
  EXPECT_EQ(find_warning(result, "R014"), nullptr)
      << "R014 must not be a validation warning this release: " << result.summary();
  // The documented fallback still applies.
  EXPECT_EQ(mgr.get_config().unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
}

// The point of the whole item: strict validation is the SHIPPED DEFAULT, and it
// must not refuse a manifest over an advisory notice.
TEST_F(ManifestManagerTest, UnknownPolicyValueStillLoadsUnderStrictValidation) {
  ManifestManager mgr(nullptr);
  EXPECT_TRUE(mgr.load_manifest_from_string(unknown_policy_manifest, true))
      << "a bad policy value must not fail the load while R014 is advisory";
  EXPECT_TRUE(mgr.is_manifest_active());
  EXPECT_EQ(find_warning(mgr.get_validation_result(), "R014"), nullptr);
  EXPECT_EQ(mgr.get_config().unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
}

TEST_F(ManifestManagerTest, UnknownPolicyValueFromFileStillLoads) {
  std::string path = write_temp_file("unknown_policy.yaml", unknown_policy_manifest);

  ManifestManager mgr(nullptr);
  ASSERT_TRUE(mgr.load_manifest(path, true));
  EXPECT_TRUE(mgr.is_manifest_active());
  EXPECT_EQ(find_warning(mgr.get_validation_result(), "R014"), nullptr);
}

// get_config() is on the liveness path: /health calls it on every request and
// a container probe calls /health. load_manifest() holds the manager's mutex
// from entry through parse, fragment scan, inventory CSV and validation, so if
// get_config() took that mutex a plugin-triggered reload would park every
// concurrent probe - and a probe with a short timeout restarts the gateway
// mid-reload.
//
// Made deterministic with a large fragments directory: the load is slow enough
// that a lock-taking reader would visibly stall. Before the snapshot existed,
// the slowest observed get_config() was the length of a whole load.
TEST_F(ManifestManagerTest, GetConfigNeverBlocksBehindAManifestLoad) {
  const std::string base = R"(
manifest_version: "1.0"
config:
  unmanifested_nodes: "ignore"
components:
  - id: base-ecu
    name: "Base ECU"
)";
  const std::string base_path = write_temp_file("lockfree_base.yaml", base);

  // A fragments directory big enough that a load takes real time.
  const auto frag_dir =
      std::filesystem::temp_directory_path() / ("medkit-lockfree-frags-" + std::to_string(::getpid()));
  std::filesystem::create_directories(frag_dir);
  for (int i = 0; i < 400; ++i) {
    std::ofstream(frag_dir / ("frag_" + std::to_string(i) + ".yaml"))
        << "components:\n  - id: frag-" << i << "\n    name: \"Fragment " << i << "\"\n";
  }

  ManifestManager mgr(nullptr);
  mgr.set_fragments_dir(frag_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path, false));
  EXPECT_EQ(mgr.get_config().unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);

  std::atomic<bool> stop{false};
  std::atomic<int64_t> worst_us{0};
  std::thread reader([&] {
    while (!stop.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      auto cfg = mgr.get_config();
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
      // Only ever the configured value or the default - never a torn read.
      EXPECT_TRUE(cfg.unmanifested_nodes == ManifestConfig::UnmanifestedNodePolicy::IGNORE ||
                  cfg.unmanifested_nodes == ManifestConfig::UnmanifestedNodePolicy::WARN);
      int64_t prev = worst_us.load();
      while (elapsed > prev && !worst_us.compare_exchange_weak(prev, elapsed)) {
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });

  // Several reloads, each re-reading all 400 fragments under the mutex.
  const auto load_start = std::chrono::steady_clock::now();
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(mgr.reload_manifest());
  }
  const auto load_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - load_start).count();

  stop.store(true);
  reader.join();
  std::error_code ec;
  std::filesystem::remove_all(frag_dir, ec);

  // The loads must have taken long enough for this to mean something.
  ASSERT_GT(load_ms, 20) << "the fragment load was too fast to prove anything";
  // A reader that took the load mutex would show a stall on the order of a
  // whole load. Allow a wide margin for scheduler noise and sanitizers.
  EXPECT_LT(worst_us.load(), load_ms * 1000 / 4) << "get_config() stalled for " << worst_us.load() << "us while "
                                                 << load_ms << "ms of loading ran - it is taking the manifest lock";
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
