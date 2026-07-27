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

#include <nlohmann/json.hpp>
#include <set>
#include <string>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
using ros2_medkit_gateway::handlers::BulkDataHandlers;

class BulkDataHandlersTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
  void TearDown() override {
  }
};

// === MIME type tests ===

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeMcap) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("mcap"), "application/x-mcap");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeSqlite3) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("sqlite3"), "application/x-sqlite3");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeDb3) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("db3"), "application/x-sqlite3");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeUnknown) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("unknown"), "application/octet-stream");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeEmpty) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype(""), "application/octet-stream");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeCasesSensitive) {
  // MCAP should not match mcap (case sensitive)
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("MCAP"), "application/octet-stream");
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("Mcap"), "application/octet-stream");
}

// === Shared-recording identifier tests ===
// Faults of one burst share a recording; each fault's descriptor carries
// x-medkit.recording_id = the bag directory basename, so clients can group
// the descriptors that serve the same bytes.

TEST_F(BulkDataHandlersTest, RecordingIdIsTheBagDirectoryBasename) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_MOTOR_OVERHEAT_1738664999000"),
            "fault_MOTOR_OVERHEAT_1738664999000");
}

TEST_F(BulkDataHandlersTest, RecordingIdIsTheSameForEveryFaultOfTheBurst) {
  // Two faults of one burst store rows pointing at the same path: their
  // descriptors must resolve to one identifier.
  const std::string shared_path = "/tmp/fault_ROOT_CAUSE_1700000000000";
  EXPECT_EQ(handlers::detail::rosbag_recording_id(shared_path), handlers::detail::rosbag_recording_id(shared_path));
  EXPECT_EQ(handlers::detail::rosbag_recording_id(shared_path), "fault_ROOT_CAUSE_1700000000000");
}

TEST_F(BulkDataHandlersTest, RecordingIdToleratesTrailingSlashAndEmptyPath) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_X_123/"), "fault_X_123");
  EXPECT_EQ(handlers::detail::rosbag_recording_id(""), "");
}

// === Shared timestamp utility tests ===

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsValidTimestamp) {
  // 2026-02-08T00:00:00.000Z
  int64_t ns = 1770458400000000000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find("2026") != std::string::npos);
  EXPECT_TRUE(result.find("T") != std::string::npos);
  EXPECT_TRUE(result.find("Z") != std::string::npos);
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsEpoch) {
  auto result = ros2_medkit_gateway::format_timestamp_ns(0);
  EXPECT_EQ(result, "1970-01-01T00:00:00.000Z");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsWithMilliseconds) {
  // 1 second + 123 ms
  int64_t ns = 1'000'000'000 + 123'000'000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find(".123Z") != std::string::npos);
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsNegativeFallback) {
  // Negative timestamps should return fallback
  auto result = ros2_medkit_gateway::format_timestamp_ns(-1);
  EXPECT_FALSE(result.empty());
  EXPECT_TRUE(result.find("Z") != std::string::npos);
}

// === Descriptor to JSON conversion tests ===

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonConversion) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "calibration_123_abcd1234";
  desc.name = "test.bin";
  desc.mime_type = "application/octet-stream";
  desc.size = 1024;
  desc.created = "2026-01-01T00:00:00.000Z";
  desc.description = "Test upload";
  desc.metadata = nlohmann::json::object();

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created},
                      {"description", desc.description}};

  EXPECT_EQ(j["id"], "calibration_123_abcd1234");
  EXPECT_EQ(j["name"], "test.bin");
  EXPECT_EQ(j["mimetype"], "application/octet-stream");
  EXPECT_EQ(j["size"], 1024);
  EXPECT_EQ(j["creation_date"], "2026-01-01T00:00:00.000Z");
  EXPECT_EQ(j["description"], "Test upload");
  EXPECT_FALSE(j.contains("x-medkit"));
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonWithMetadata) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "calibration_123_abcd1234";
  desc.name = "cal.bin";
  desc.mime_type = "application/octet-stream";
  desc.size = 512;
  desc.created = "2026-01-01T00:00:00.000Z";
  desc.description = "";
  desc.metadata = {{"sensor", "lidar"}, {"version", 2}};

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created},
                      {"description", desc.description}};
  if (!desc.metadata.empty()) {
    j["x-medkit"] = desc.metadata;
  }

  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["sensor"], "lidar");
  EXPECT_EQ(j["x-medkit"]["version"], 2);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonWithoutDescription) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "firmware_456_ef012345";
  desc.name = "fw.img";
  desc.mime_type = "application/octet-stream";
  desc.size = 2048;
  desc.created = "2026-06-15T12:00:00.000Z";
  desc.description = "";
  desc.metadata = nlohmann::json::object();

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created}};
  // Only add description if non-empty (matching handler pattern)
  if (!desc.description.empty()) {
    j["description"] = desc.description;
  }
  if (!desc.metadata.empty()) {
    j["x-medkit"] = desc.metadata;
  }

  EXPECT_FALSE(j.contains("description"));
  EXPECT_FALSE(j.contains("x-medkit"));
}

// === Error code tests ===

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, PayloadTooLargeErrorCodeDefined) {
  EXPECT_NE(ros2_medkit_gateway::ERR_PAYLOAD_TOO_LARGE, nullptr);
  EXPECT_STREQ(ros2_medkit_gateway::ERR_PAYLOAD_TOO_LARGE, "payload-too-large");
}

// =============================================================================
// compute_bulkdata_source_filters tests
//
// Pin the entity-type branching that drives rosbag descriptor lookups + the
// download ownership check. Crucial because synthetic / runtime-discovered
// components have empty fqn AND empty namespace_path: without aggregation
// from hosted apps the handler used to silently return zero source filters.
//
// Tested as a pure free function in detail:: against a directly-constructed
// ThreadSafeEntityCache so no GatewayNode / DDS context is needed.
// =============================================================================

namespace {

App make_test_app(const std::string & id, const std::string & node_name, const std::string & ns,
                  const std::string & component_id) {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = component_id;
  App::RosBinding rb;
  rb.node_name = node_name;
  rb.namespace_pattern = ns;
  a.ros_binding = rb;
  return a;
}

EntityInfo make_entity_info(EntityType type, const std::string & id, const std::string & namespace_path,
                            const std::string & fqn) {
  EntityInfo info;
  info.type = type;
  info.id = id;
  info.namespace_path = namespace_path;
  info.fqn = fqn;
  return info;
}

}  // namespace

class BulkDataSourceFiltersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Synthetic component: empty fqn AND empty namespace_path.
    Component synthetic;
    synthetic.id = "runtime_engine";
    synthetic.name = "Runtime Engine";
    synthetic.namespace_path = "";
    synthetic.fqn = "";

    // Manifest-only component: declares namespace but groups topics, not nodes.
    Component manifest_only;
    manifest_only.id = "topics_group";
    manifest_only.name = "Topics Group";
    manifest_only.namespace_path = "/topics/group";
    manifest_only.fqn = "/topics/group";

    auto app1 = make_test_app("temp_sensor", "temp_sensor", "/powertrain/engine", "runtime_engine");
    auto app2 = make_test_app("rpm_sensor", "rpm_sensor", "/powertrain/engine", "runtime_engine");

    Function func;
    func.id = "powertrain_diag";
    func.name = "Powertrain Diagnostics";
    func.hosts = {"temp_sensor", "rpm_sensor"};

    Function empty_func;
    empty_func.id = "empty_func";
    empty_func.name = "Empty Function";

    cache_.update_all({}, {synthetic, manifest_only}, {app1, app2}, {func, empty_func});
  }

  ThreadSafeEntityCache cache_;
};

// COMPONENT with hosted apps - returns app effective FQNs (synthetic component
// has empty fqn / namespace_path; without aggregation this would be {}).
TEST_F(BulkDataSourceFiltersTest, ComponentWithHostedAppsReturnsAppFqns) {
  auto entity = make_entity_info(EntityType::COMPONENT, "runtime_engine", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 2u);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("/powertrain/engine/temp_sensor"));
  EXPECT_TRUE(as_set.count("/powertrain/engine/rpm_sensor"));
}

// COMPONENT hosting plugin-provided apps - those apps have no ROS binding and
// report faults under their bare entity id, so that id must become the filter.
// Resolving by effective_fqn() alone yielded zero filters and the download
// ownership check answered "Bulk-data not found for this entity" for a bag that
// existed on disk.
TEST_F(BulkDataSourceFiltersTest, ComponentWithExternalAppsReturnsBareEntityIds) {
  App plc_app;
  plc_app.id = "plc_line1";
  plc_app.name = "PLC Line 1";
  plc_app.component_id = "plc_hw";
  plc_app.external = true;
  cache_.update_apps({plc_app});

  auto entity = make_entity_info(EntityType::COMPONENT, "plc_hw", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc_line1");
}

// APP provided by a protocol plugin - no ROS binding, so fqn and namespace_path
// are empty and the generic path returned no filter at all: the rosbag the fault
// detail advertises under /apps/<id>/bulk-data/rosbags/<code> belonged to nobody
// and every download 404'd. Its bare id is its reporting source.
TEST_F(BulkDataSourceFiltersTest, ExternalAppResolvesToItsBareEntityId) {
  App device;
  device.id = "twincat_runtime_device";
  device.name = "TwinCAT 3 Runtime (device)";
  device.component_id = "twincat_runtime";
  device.external = true;
  cache_.update_apps({device});

  auto entity = make_entity_info(EntityType::APP, "twincat_runtime_device", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "twincat_runtime_device");
}

// An external COMPONENT reports faults under its own id as well (a bridge raises
// PLC_COMMS_LOST there), so it owns that source alongside its hosted apps.
TEST_F(BulkDataSourceFiltersTest, ExternalComponentAlsoOwnsItsOwnId) {
  Component plc;
  plc.id = "plc_hw2";
  plc.name = "PLC";
  plc.external = true;
  cache_.update_components({plc});
  App child;
  child.id = "plc_hw2_device";
  child.name = "PLC (device)";
  child.component_id = "plc_hw2";
  child.external = true;
  cache_.update_apps({child});

  auto entity = make_entity_info(EntityType::COMPONENT, "plc_hw2", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("plc_hw2_device"));
  EXPECT_TRUE(as_set.count("plc_hw2"));
}

// COMPONENT with no hosted apps but non-empty fqn falls through to fqn path
// (manifest deployment grouping topics rather than nodes).
TEST_F(BulkDataSourceFiltersTest, ComponentManifestOnlyFallsThroughToFqn) {
  auto entity = make_entity_info(EntityType::COMPONENT, "topics_group", "/topics/group", "/topics/group");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/topics/group");
}

// COMPONENT with no hosted apps AND no fqn / namespace_path returns empty -
// nothing to query.
TEST_F(BulkDataSourceFiltersTest, ComponentSyntheticWithoutAppsReturnsEmpty) {
  auto entity = make_entity_info(EntityType::COMPONENT, "nonexistent_comp", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// FUNCTION with hosted apps - returns app effective FQNs. Crucially, FUNCTION
// must NOT fall through to namespace_path/fqn even when the host list is
// non-empty - functions are pure aggregated views.
TEST_F(BulkDataSourceFiltersTest, FunctionWithHostsReturnsAppFqns) {
  auto entity = make_entity_info(EntityType::FUNCTION, "powertrain_diag", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 2u);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("/powertrain/engine/temp_sensor"));
  EXPECT_TRUE(as_set.count("/powertrain/engine/rpm_sensor"));
}

// FUNCTION without hosted apps returns empty - no fall-through to fqn even if
// the entity carried one (regression guard for the original FUNCTION semantics
// after the COMPONENT/FUNCTION split).
TEST_F(BulkDataSourceFiltersTest, FunctionWithoutHostsReturnsEmptyEvenIfFqnSet) {
  auto entity = make_entity_info(EntityType::FUNCTION, "empty_func", "/some/ns", "/some/fqn");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// APP entity returns its own fqn as the single filter - no aggregation.
TEST_F(BulkDataSourceFiltersTest, AppReturnsSingleFqnFilter) {
  auto entity =
      make_entity_info(EntityType::APP, "temp_sensor", "/powertrain/engine", "/powertrain/engine/temp_sensor");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/powertrain/engine/temp_sensor");
}

// APP without fqn falls through to namespace_path filter.
TEST_F(BulkDataSourceFiltersTest, AppWithEmptyFqnFallsThroughToNamespacePath) {
  auto entity = make_entity_info(EntityType::APP, "some_app", "/the/namespace", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/the/namespace");
}

// APP with neither fqn nor namespace_path returns empty.
TEST_F(BulkDataSourceFiltersTest, AppWithEmptyFqnAndNamespaceReturnsEmpty) {
  auto entity = make_entity_info(EntityType::APP, "some_app", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// AREA not present in the cache (no hosted apps resolvable) falls back to fqn.
TEST_F(BulkDataSourceFiltersTest, AreaReturnsFqnAsFilter) {
  auto entity = make_entity_info(EntityType::AREA, "powertrain", "/powertrain", "/powertrain");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/powertrain");
}

// AREA with hosted apps resolves them like the fault scope does, recursing
// subareas and keeping external bare ids. The namespace fallback alone never
// matched a bag: list_rosbags_for_entity compares reporting sources exactly,
// so /areas/<id>/faults advertised a bulk_data_uri that 404'd.
TEST_F(BulkDataSourceFiltersTest, AreaResolvesHostedAppsIncludingSubareas) {
  Area cell;
  cell.id = "plc-cell";
  cell.name = "PLC Cell";
  cell.namespace_path = "/plc_cell";
  Area sub;
  sub.id = "cabinet";
  sub.name = "Cabinet";
  sub.namespace_path = "/plc_cell/cabinet";
  sub.parent_area_id = "plc-cell";

  Component plc;
  plc.id = "s7-plc";
  plc.name = "PLC";
  plc.area = "cabinet";
  App proc;
  proc.id = "plc-process";
  proc.name = "PLC Process";
  proc.component_id = "s7-plc";
  proc.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({cell, sub}, {plc}, {proc}, {});

  auto entity = make_entity_info(EntityType::AREA, "plc-cell", "/plc_cell", "/plc_cell");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc-process");
}

// FUNCTION whose host is a Component (not an app) resolves the component's
// apps. The app-index lookup dropped component hosts: the function listed the
// fault but its bag download 404'd.
TEST_F(BulkDataSourceFiltersTest, FunctionWithComponentHostResolvesComponentApps) {
  Component plc;
  plc.id = "plc_hw";
  plc.name = "PLC";
  App proc;
  proc.id = "plc-process";
  proc.name = "PLC Process";
  proc.component_id = "plc_hw";
  proc.external = true;
  Function func;
  func.id = "level-control";
  func.name = "Level Control";
  func.hosts = {"plc_hw"};

  ThreadSafeEntityCache cache;
  cache.update_all({}, {plc}, {proc}, {func});

  auto entity = make_entity_info(EntityType::FUNCTION, "level-control", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc-process");
}

// Download ownership uses fault_in_source_scope over the computed filters:
// exact match or '/'-boundary prefix only. A raw prefix match (the transport's
// get_fault(code, source) semantics) would let app id "plc" claim the bag of
// "plc_line1".
TEST_F(BulkDataSourceFiltersTest, DownloadOwnershipScopeRejectsPrefixSiblingApp) {
  App plc;
  plc.id = "plc";
  plc.name = "PLC";
  plc.external = true;
  App plc_line1;
  plc_line1.id = "plc_line1";
  plc_line1.name = "PLC Line 1";
  plc_line1.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, plc_line1}, {});

  auto entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc");
  std::set<std::string> scope(filters.begin(), filters.end());

  nlohmann::json sibling_fault = {{"reporting_sources", {"plc_line1"}}};
  EXPECT_FALSE(faults::fault_in_source_scope(sibling_fault, scope));

  nlohmann::json own_fault = {{"reporting_sources", {"plc"}}};
  EXPECT_TRUE(faults::fault_in_source_scope(own_fault, scope));

  nlohmann::json child_fault = {{"reporting_sources", {"plc/axis1"}}};
  EXPECT_TRUE(faults::fault_in_source_scope(child_fault, scope));
}
