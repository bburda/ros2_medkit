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

// @verifies REQ_INTEROP_MANIFEST_FRAGMENTS

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"

using ros2_medkit_gateway::discovery::ManifestManager;

namespace {

/// Minimal valid base manifest with one area and one manifest-owned component.
/// Keeps apps empty so fragments have somewhere to contribute new entities.
constexpr const char * kBaseManifest = R"(
manifest_version: "1.0"
metadata:
  name: fragments-test
  version: "0.0.1"
areas:
  - id: vehicle
    name: Test vehicle
components:
  - id: ecu-primary
    name: Primary ECU
    area: vehicle
)";

class FragmentsFixture : public ::testing::Test {
 protected:
  std::filesystem::path base_path;
  std::filesystem::path fragments_dir;

  void SetUp() override {
    auto tmp = std::filesystem::temp_directory_path();
    std::string prefix = "medkit-fragments-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "-";
    int suffix = 0;
    do {
      fragments_dir = tmp / (prefix + std::to_string(suffix++));
    } while (std::filesystem::exists(fragments_dir));
    std::filesystem::create_directories(fragments_dir);
    base_path = fragments_dir.parent_path() / (prefix + "base.yaml");
    std::ofstream(base_path) << kBaseManifest;
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(fragments_dir, ec);
    std::filesystem::remove(base_path, ec);
  }

  void write_fragment(const std::string & name, const std::string & body) {
    std::ofstream(fragments_dir / name) << body;
  }
};

}  // namespace

TEST_F(FragmentsFixture, NoFragmentsDirLoadsBaseOnly) {
  ManifestManager mgr;
  // Leave fragments_dir unset -> behaves like before Chunk 2.
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  EXPECT_EQ(mgr.get_apps().size(), 0u);
  EXPECT_EQ(mgr.get_components().size(), 1u);
}

TEST_F(FragmentsFixture, MissingFragmentsDirIsNotAnError) {
  ManifestManager mgr;
  mgr.set_fragments_dir((fragments_dir / "does-not-exist").string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  EXPECT_EQ(mgr.get_apps().size(), 0u);
}

TEST_F(FragmentsFixture, SingleFragmentAppIsMerged) {
  write_fragment("hello.yaml", R"(
apps:
  - id: helloApp
    name: Hello World
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "helloApp");
}

TEST_F(FragmentsFixture, MultipleFragmentsMergeDeterministically) {
  // Names chosen so filesystem order is unstable without sorting but becomes
  // stable after the apply_fragments sort pass.
  write_fragment("z-second.yaml", R"(
apps:
  - id: secondApp
    name: Second
    is_located_on: ecu-primary
    ros_binding:
      node_name: secondApp
)");
  write_fragment("a-first.yaml", R"(
apps:
  - id: firstApp
    name: First
    is_located_on: ecu-primary
    ros_binding:
      node_name: firstApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 2u);
  // Alphabetical file order -> a-first processed before z-second.
  EXPECT_EQ(apps[0].id, "firstApp");
  EXPECT_EQ(apps[1].id, "secondApp");
}

TEST_F(FragmentsFixture, NonYamlFileIsIgnored) {
  write_fragment("ignore-me.txt", "totally not yaml");
  write_fragment("pick.yaml", R"(
apps:
  - id: pickedApp
    name: Picked
    is_located_on: ecu-primary
    ros_binding:
      node_name: pickedApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 1u);
}

TEST_F(FragmentsFixture, FragmentAreasAreRejected) {
  write_fragment("bad.yaml", R"(
areas:
  - id: rogue
    name: Rogue area
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for areas";
}

TEST_F(FragmentsFixture, FragmentMetadataDescriptionIsRejected) {
  // metadata.{description,created_at} are base-manifest-owned just like
  // metadata.{name,version} - a fragment that touches them must fail.
  write_fragment("bad.yaml", R"(
metadata:
  description: rogue fragment description
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for metadata.description";
}

TEST_F(FragmentsFixture, FragmentMetadataCreatedAtIsRejected) {
  write_fragment("bad.yaml", R"(
metadata:
  created_at: "2026-04-17T00:00:00Z"
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for metadata.created_at";
}

TEST_F(FragmentsFixture, FragmentDiscoveryConfigIsRejected) {
  // The top-level `discovery:` key maps to ManifestConfig which has
  // non-empty defaults, so the check relies on raw-YAML key detection.
  write_fragment("bad.yaml", R"(
discovery:
  unmanifested_nodes: error
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD" && err.message.find("discovery") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error naming 'discovery'";
}

TEST_F(FragmentsFixture, FragmentConfigBlockIsRejected) {
  // `config:` is the canonical spelling of the same block, so a fragment must
  // not be able to smuggle discovery configuration in under it either.
  write_fragment("bad.yaml", R"(
config:
  unmanifested_nodes: error
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD" && err.message.find("config") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error naming 'config'";
}

TEST_F(FragmentsFixture, DuplicateIdsAcrossFragmentsFailValidation) {
  write_fragment("a.yaml", R"(
apps:
  - id: clashApp
    name: A
    is_located_on: ecu-primary
    ros_binding:
      node_name: clashA
)");
  write_fragment("b.yaml", R"(
apps:
  - id: clashApp
    name: B
    is_located_on: ecu-primary
    ros_binding:
      node_name: clashB
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
}

TEST_F(FragmentsFixture, ReloadPicksUpNewFragments) {
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 0u);
  // Drop a fragment after initial load and reload.
  write_fragment("late.yaml", R"(
apps:
  - id: lateApp
    name: Late
    is_located_on: ecu-primary
    ros_binding:
      node_name: lateApp
)");
  ASSERT_TRUE(mgr.reload_manifest());
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "lateApp");
}

TEST_F(FragmentsFixture, ReloadPicksUpRemovedFragments) {
  write_fragment("temp.yaml", R"(
apps:
  - id: tempApp
    name: Temp
    is_located_on: ecu-primary
    ros_binding:
      node_name: tempApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 1u);

  std::filesystem::remove(fragments_dir / "temp.yaml");
  ASSERT_TRUE(mgr.reload_manifest());
  EXPECT_EQ(mgr.get_apps().size(), 0u);
}

TEST_F(FragmentsFixture, NestedKeyNamedLikeManifestVersionDoesNotBlockInjection) {
  // parse_fragment_file auto-injects `manifest_version: "1.0"` when the
  // fragment omits it. The detector that decides whether to inject scans
  // the raw YAML line-by-line; earlier revisions matched any line whose
  // first non-whitespace token was `manifest_version:`, meaning an
  // indented key of the same name anywhere in the fragment would disable
  // injection and cause a "Missing required field: manifest_version"
  // parse error - even though the fragment itself has no top-level
  // version. Fragments written by plugins that happen to use the word
  // in a nested position (e.g., inside custom metadata blocks) would
  // silently fail to merge.
  //
  // Pin the rule: only a column-0 `manifest_version:` disables injection.
  // Sub-map key `manifest_version:` inside `ros_binding:`. The schema
  // ignores unknown keys on ros_binding so the fragment is otherwise valid,
  // but the pre-fix line scanner matches the indented key and aborts the
  // synthetic version injection.
  write_fragment("nested.yaml", R"(
apps:
  - id: nestedApp
    name: Nested Key Test
    is_located_on: ecu-primary
    ros_binding:
      node_name: nestedApp
      manifest_version: "ignored-by-schema"
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false))
      << "injection should still happen when `manifest_version:` appears only as an indented value / list item";
  auto apps = mgr.get_apps();
  bool found = false;
  for (const auto & a : apps) {
    if (a.id == "nestedApp") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "nestedApp fragment should merge even though a component depends_on list mentions "
                        "'manifest_version'";
}

TEST_F(FragmentsFixture, SetFragmentsDirRoundTrip) {
  ManifestManager mgr;
  EXPECT_EQ(mgr.get_fragments_dir(), "");
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_EQ(mgr.get_fragments_dir(), fragments_dir.string());
  mgr.set_fragments_dir("");
  EXPECT_EQ(mgr.get_fragments_dir(), "");
}

TEST_F(FragmentsFixture, BaseAndFragmentDuplicateIdFailsValidation) {
  // The base manifest already declares `ecu-primary` as a component; adding
  // an app in a fragment that reuses the same id must be caught by the
  // regular validator pass after the merge (not by apply_fragments itself).
  write_fragment("clash.yaml", R"(
apps:
  - id: ecu-primary
    name: Clash
    is_located_on: ecu-primary
    ros_binding:
      node_name: clashApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  EXPECT_TRUE(vr.has_errors()) << "expected validator to reject duplicate id across base+fragment";
}

TEST_F(FragmentsFixture, OneBrokenFragmentFailsEntireLoad) {
  // All-or-nothing contract: even if one valid fragment would merge cleanly,
  // a sibling that fails to parse must fail the whole load so clients never
  // see a partially-applied manifest.
  write_fragment("valid.yaml", R"(
apps:
  - id: goodApp
    name: Good
    is_located_on: ecu-primary
    ros_binding:
      node_name: goodApp
)");
  write_fragment("broken.yaml", "apps: [ this is not valid yaml :\n  stray colon");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool saw_parse_error = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_PARSE") {
      saw_parse_error = true;
      break;
    }
  }
  EXPECT_TRUE(saw_parse_error);
  // `goodApp` must not leak into the live manifest - load failed so state
  // remains "no manifest loaded".
  EXPECT_EQ(mgr.get_apps().size(), 0u);
}

TEST_F(FragmentsFixture, EmptyReservedKeysAreRejected) {
  // Parsed-struct checks miss `areas: []` because the empty sequence parses
  // into an empty vector that is indistinguishable from "not declared". The
  // raw-YAML presence check must still catch it.
  write_fragment("empty-reserved.yaml", R"(
areas: []
metadata: {}
scripts: []
capabilities: {}
lock_overrides: {}
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  auto has_error_for = [&](const std::string & field) {
    for (const auto & err : vr.errors) {
      if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD" && err.message.find("'" + field + "'") != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  EXPECT_TRUE(has_error_for("areas"));
  EXPECT_TRUE(has_error_for("metadata"));
  EXPECT_TRUE(has_error_for("scripts"));
  EXPECT_TRUE(has_error_for("capabilities"));
  EXPECT_TRUE(has_error_for("lock_overrides"));
}

TEST_F(FragmentsFixture, UnknownTopLevelKeyIsIgnoredWithWarning) {
  // A typo like `app:` (singular) must not silently produce an empty
  // fragment - the manager logs a warning but still succeeds because no
  // forbidden key was declared.
  write_fragment("typo.yaml", R"(
app:
  - id: ignoredApp
    name: Ignored
    is_located_on: ecu-primary
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  EXPECT_EQ(mgr.get_apps().size(), 0u) << "typo'd top-level key contributes no entities";
}

TEST_F(FragmentsFixture, FragmentOverSizeLimitIsRejected) {
  // Size cap is enforced in parse_fragment_file before the file is read.
  // Build a yaml document comfortably larger than kMaxFragmentBytes (1 MiB).
  std::string huge = "apps:\n";
  huge.reserve((1U << 20U) + 4096);
  while (huge.size() < (1U << 20U) + 2048) {
    huge += "  - id: filler\n    name: Filler\n    is_located_on: ecu-primary\n";
  }
  write_fragment("huge.yaml", huge);
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool saw_parse_error = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_PARSE" && err.message.find("exceeds") != std::string::npos) {
      saw_parse_error = true;
      break;
    }
  }
  EXPECT_TRUE(saw_parse_error) << "expected a FRAGMENT_PARSE error mentioning the size limit";
}

TEST_F(FragmentsFixture, SymlinkEscapingFragmentsDirIsSkipped) {
  // Simulate the scenario: attacker drops `evil.yaml` as a symlink pointing
  // to a file outside fragments_dir. The gateway must skip it (with a warn)
  // rather than parsing the target contents.
  auto outside_dir = fragments_dir.parent_path() / (fragments_dir.filename().string() + "-outside");
  std::filesystem::create_directories(outside_dir);
  auto outside_path = outside_dir / "outside.yaml";
  std::ofstream(outside_path) << R"(
apps:
  - id: escapedApp
    name: Escaped
    is_located_on: ecu-primary
    ros_binding:
      node_name: escapedApp
)";
  std::error_code link_ec;
  std::filesystem::create_symlink(outside_path, fragments_dir / "evil.yaml", link_ec);
  if (link_ec) {
    GTEST_SKIP() << "symlink creation unsupported on this filesystem: " << link_ec.message();
  }

  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  // Symlink target was outside fragments_dir so the contribution must have
  // been skipped.
  for (const auto & app : mgr.get_apps()) {
    EXPECT_NE(app.id, "escapedApp") << "symlink escape must not contribute entities";
  }

  std::filesystem::remove_all(outside_dir, link_ec);
}
