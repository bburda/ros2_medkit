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

#include <algorithm>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

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
// No `using json = nlohmann::json` here: the namespace pulled in above already
// declares that alias, and redeclaring it shadows it.
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
// The bag directory basename is the recording's public name: it addresses the
// bag under /bulk-data/rosbags/{id} and groups the link rows serving the same
// bytes.

TEST_F(BulkDataHandlersTest, RecordingIdIsTheBagDirectoryBasename) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_MOTOR_OVERHEAT_1738664999000"),
            "fault_MOTOR_OVERHEAT_1738664999000");
}

TEST_F(BulkDataHandlersTest, RecordingIdIsTheSameForEveryFaultOfTheBurst) {
  // Rows of one burst carry different fault codes but the same path: their
  // descriptors must group under one id, distinct from any other bag's.
  const std::string burst_bag = "/tmp/fault_ROOT_CAUSE_1700000000000";
  const std::string other_bag = "/tmp/fault_UNRELATED_1700000000042";
  EXPECT_EQ(handlers::detail::rosbag_recording_id(burst_bag), "fault_ROOT_CAUSE_1700000000000");
  EXPECT_NE(handlers::detail::rosbag_recording_id(burst_bag), handlers::detail::rosbag_recording_id(other_bag));
}

TEST_F(BulkDataHandlersTest, RecordingIdToleratesTrailingSlashAndEmptyPath) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_X_123/"), "fault_X_123");
  EXPECT_EQ(handlers::detail::rosbag_recording_id(""), "");
}

// === Descriptor folding tests ===
// The fault manager returns one row per (fault, recording) link. A burst of
// correlated faults is several rows naming one bag, and one fault holding a
// history is several rows with distinct bags. Both shapes have to come out as
// one descriptor per recording.

namespace {

json rosbag_row(const std::string & fault_code, const std::string & recording_id, uint64_t size_bytes = 1024) {
  return json{{"fault_code", fault_code}, {"recording_id", recording_id}, {"file_path", "/var/bags/" + recording_id},
              {"format", "mcap"},         {"duration_sec", 5.0},          {"size_bytes", size_bytes}};
}

json fault_at(double first_occurred) {
  return json{{"first_occurred", first_occurred}};
}

/// A row carrying the recording's own timestamp, which is what the fault manager
/// sends now.
json rosbag_row_made_at(const std::string & fault_code, const std::string & recording_id, int64_t created_at_ns) {
  json row = rosbag_row(fault_code, recording_id);
  row["created_at_ns"] = created_at_ns;
  return row;
}

}  // namespace

TEST_F(BulkDataHandlersTest, OneFaultWithSeveralRecordingsYieldsOneDescriptorEach) {
  // The feature: a flapping fault keeps a history, and every recording in it
  // has to be separately addressable.
  const std::vector<json> rows{rosbag_row("FLAP", "fault_FLAP_3"), rosbag_row("FLAP", "fault_FLAP_2"),
                               rosbag_row("FLAP", "fault_FLAP_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 3u);
  EXPECT_EQ(descriptors[0].id, "fault_FLAP_3") << "order follows the fault manager's listing";
  EXPECT_EQ(descriptors[1].id, "fault_FLAP_2");
  EXPECT_EQ(descriptors[2].id, "fault_FLAP_1");
}

TEST_F(BulkDataHandlersTest, ABurstCollapsesToOneDescriptorCarryingEveryFault) {
  // Three rows, one bag. Emitting three items would repeat the id and report
  // the bag's size three times, which reads as three bags worth of storage.
  const std::vector<json> rows{rosbag_row("ROOT_CAUSE", "fault_ROOT_CAUSE_17"),
                               rosbag_row("DOWNSTREAM_B", "fault_ROOT_CAUSE_17"),
                               rosbag_row("DOWNSTREAM_A", "fault_ROOT_CAUSE_17")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_ROOT_CAUSE_17");
  EXPECT_EQ(descriptors[0].size, 1024u) << "the bag is counted once, not once per attached fault";

  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  const auto & x = *descriptors[0].x_medkit;
  ASSERT_TRUE(x.contains("fault_codes"));
  EXPECT_EQ(x["fault_codes"], (json{"DOWNSTREAM_A", "DOWNSTREAM_B", "ROOT_CAUSE"})) << "sorted, so output is stable";
  EXPECT_EQ(x["recording_id"], "fault_ROOT_CAUSE_17");
  EXPECT_EQ(x["format"], "mcap");
  EXPECT_DOUBLE_EQ(x["duration_sec"].get<double>(), 5.0);
}

TEST_F(BulkDataHandlersTest, TheSameFaultTwiceOnOneRecordingIsNotListedTwice) {
  // Two source filters can both resolve to the same app, so the same row can
  // arrive twice. A repeated code in fault_codes would be visible in the API.
  const std::vector<json> rows{rosbag_row("DUP", "fault_DUP_1"), rosbag_row("DUP", "fault_DUP_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  EXPECT_EQ((*descriptors[0].x_medkit)["fault_codes"], (json{"DUP"}));
}

TEST_F(BulkDataHandlersTest, ARecordingIsDatedByTheEarliestFaultOfItsBurst) {
  // Downstream faults confirm after the root cause, and the recording covers
  // the whole burst, so the earliest is the honest creation date.
  const std::vector<json> rows{rosbag_row("DOWNSTREAM", "fault_ROOT_9"), rosbag_row("ROOT", "fault_ROOT_9")};
  const std::unordered_map<std::string, json> faults{{"DOWNSTREAM", fault_at(1700000900.0)},
                                                     {"ROOT", fault_at(1700000000.0)}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, faults);
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000000} * 1'000'000'000));
}

TEST_F(BulkDataHandlersTest, EachRecordingOfOneFaultIsDatedByItsOwnCapture) {
  // Dating a recording by its fault gave every recording of that fault the same
  // date - and holding more than one is the whole point of the change, so the date
  // is exactly what tells the occurrences apart.
  const std::vector<json> rows{rosbag_row_made_at("FLAP", "fault_FLAP_2", int64_t{1700000900} * 1'000'000'000),
                               rosbag_row_made_at("FLAP", "fault_FLAP_1", int64_t{1700000000} * 1'000'000'000)};
  const std::unordered_map<std::string, json> faults{{"FLAP", fault_at(1700000000.0)}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, faults);
  ASSERT_EQ(descriptors.size(), 2u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000900} * 1'000'000'000));
  EXPECT_EQ(descriptors[1].creation_date, format_timestamp_ns(int64_t{1700000000} * 1'000'000'000));
  EXPECT_NE(descriptors[0].creation_date, descriptors[1].creation_date);
}

TEST_F(BulkDataHandlersTest, AnAcknowledgedFaultsRecordingsKeepTheirRealDate) {
  // list_faults excludes cleared faults by default, so an acknowledged fault is
  // absent from the map while its rows are still listed. Reading the date off the
  // fault dated those recordings 1970 - and this change is what makes an
  // acknowledged fault keep them in the first place.
  const std::vector<json> rows{rosbag_row_made_at("ACKED", "fault_ACKED_1", int64_t{1700000500} * 1'000'000'000)};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000500} * 1'000'000'000));
  EXPECT_EQ(descriptors[0].creation_date.rfind("1970", 0), std::string::npos) << "not the epoch";
}

TEST_F(BulkDataHandlersTest, DescriptorIdFallsBackToTheBasenameWhenTheRowHasNoRecordingId) {
  // A peer or a replay predating the stored field still has to be addressable.
  const std::vector<json> rows{json{{"fault_code", "OLD"}, {"file_path", "/var/bags/fault_OLD_5"}, {"format", "mcap"}}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_OLD_5");
}

TEST_F(BulkDataHandlersTest, ARowWithNeitherIdNorPathIsDroppedRatherThanAdvertised) {
  // An empty id would render as /bulk-data/rosbags/ - a 404 the client cannot
  // act on. Better absent than advertised and broken.
  const std::vector<json> rows{json{{"fault_code", "GHOST"}, {"format", "mcap"}}, rosbag_row("REAL", "fault_REAL_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_REAL_1");
}

TEST_F(BulkDataHandlersTest, DistinctRecordingsEachReportTheirOwnSize) {
  const std::vector<json> rows{rosbag_row("A", "fault_A_1", 2048), rosbag_row("B", "fault_B_1", 4096)};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 2u);
  EXPECT_EQ(descriptors[0].size, 2048u);
  EXPECT_EQ(descriptors[1].size, 4096u);
}

TEST_F(BulkDataHandlersTest, NoRowsYieldsNoDescriptors) {
  EXPECT_TRUE(handlers::detail::fold_rosbag_rows_into_descriptors({}, {}).empty());
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
// === Download authorization tests ===
// A recording is shared by a whole burst, so ownership is the union over its
// attached faults. The scope matcher itself is unchanged and pinned below; what
// is new is which codes get fed to it.

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesComeFromTheRecordingNotTheUrl) {
  const nlohmann::json rosbag = {{"file_path", "/var/bags/fault_ROOT_1"},
                                 {"recording_id", "fault_ROOT_1"},
                                 {"fault_codes", {"ROOT", "DOWNSTREAM_A", "DOWNSTREAM_B"}}};

  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(rosbag, "fault_ROOT_1"),
            (std::vector<std::string>{"ROOT", "DOWNSTREAM_A", "DOWNSTREAM_B"}));
}

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesFallBackToTheRequestedIdOnAnOlderPeer) {
  // The compatibility path: the id addressed was the fault code, and a peer
  // that predates the field sends no list. Authorizing against the requested id
  // is exactly the check that shipped before.
  const nlohmann::json rosbag = {{"file_path", "/var/bags/fault_MOTOR_1"}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(rosbag, "MOTOR_OVERHEAT"),
            (std::vector<std::string>{"MOTOR_OVERHEAT"}));
}

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesFallBackOnAnEmptyOrMalformedList) {
  // Never return empty: an empty list makes any_of vacuously false, which would
  // 404 a download the entity owns.
  const nlohmann::json empty_list = {{"fault_codes", nlohmann::json::array()}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(empty_list, "X"), (std::vector<std::string>{"X"}));

  const nlohmann::json not_an_array = {{"fault_codes", "X"}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(not_an_array, "X"), (std::vector<std::string>{"X"}));
}

TEST_F(BulkDataSourceFiltersTest, AFaultCodeUrlIsRecognisedAsTheCompatibilityPath) {
  // The segment named a fault; the answer is that fault's newest recording, whose
  // id is something else. Authorizing on the union alone would 200 a code the
  // entity does not own, so this path also demands the requested code in scope.
  const nlohmann::json resolved_by_code = {{"recording_id", "fault_MOTOR_OVERHEAT_1738664999000"},
                                           {"fault_codes", {"MOTOR_OVERHEAT", "MOTOR_STALL"}}};
  EXPECT_TRUE(handlers::detail::rosbag_resolved_by_fault_code(resolved_by_code, "MOTOR_OVERHEAT"));
}

TEST_F(BulkDataSourceFiltersTest, ARecordingIdUrlIsNotTheCompatibilityPath) {
  const nlohmann::json resolved_by_id = {{"recording_id", "fault_MOTOR_OVERHEAT_1738664999000"},
                                         {"fault_codes", {"MOTOR_OVERHEAT", "MOTOR_STALL"}}};
  EXPECT_FALSE(handlers::detail::rosbag_resolved_by_fault_code(resolved_by_id, "fault_MOTOR_OVERHEAT_1738664999000"));
}

TEST_F(BulkDataSourceFiltersTest, APeerWithoutRecordingIdsIsNotTreatedAsCompatibilityPath) {
  // An older peer answers by fault code only and sends no recording id. The
  // attached-codes fallback already reduces to the pre-#620 check there, so
  // demanding a second one would 404 downloads that used to work.
  const nlohmann::json older_peer = {{"file_path", "/var/bags/fault_MOTOR_1"}};
  EXPECT_FALSE(handlers::detail::rosbag_resolved_by_fault_code(older_peer, "MOTOR_OVERHEAT"));
}

TEST_F(BulkDataSourceFiltersTest, ABurstRecordingIsOwnedByAnyEntityOwningOneOfItsFaults) {
  // One bag, three faults, two apps. Each app reaches the bag through its own
  // fault - which is what it could already do when the bag was addressed by
  // fault code.
  App plc;
  plc.id = "plc";
  plc.external = true;
  App vision;
  vision.id = "vision";
  vision.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, vision}, {});

  auto plc_entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto plc_filters = handlers::detail::compute_bulkdata_source_filters(cache, plc_entity);
  std::set<std::string> plc_scope(plc_filters.begin(), plc_filters.end());

  const std::vector<nlohmann::json> burst_faults = {nlohmann::json{{"reporting_sources", {"vision"}}},
                                                    nlohmann::json{{"reporting_sources", {"plc"}}}};

  const bool plc_authorized = std::any_of(burst_faults.begin(), burst_faults.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, plc_scope);
  });
  EXPECT_TRUE(plc_authorized) << "the PLC owns one of the burst's faults";
}

TEST_F(BulkDataSourceFiltersTest, ABurstRecordingIsRejectedWhenNoAttachedFaultIsInScope) {
  App plc;
  plc.id = "plc";
  plc.external = true;
  App plc_line1;
  plc_line1.id = "plc_line1";
  plc_line1.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, plc_line1}, {});

  auto entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  std::set<std::string> scope(filters.begin(), filters.end());

  // The prefix-sibling rule has to survive the union: "plc" must not reach a
  // burst owned entirely by "plc_line1", no matter how many faults it holds.
  const std::vector<nlohmann::json> foreign_burst = {nlohmann::json{{"reporting_sources", {"plc_line1"}}},
                                                     nlohmann::json{{"reporting_sources", {"plc_line1/axis2"}}}};

  const bool authorized = std::any_of(foreign_burst.begin(), foreign_burst.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, scope);
  });
  EXPECT_FALSE(authorized);

  // ...while a descendant of the entity's own scope still reaches it.
  const std::vector<nlohmann::json> own_burst = {nlohmann::json{{"reporting_sources", {"plc_line1"}}},
                                                 nlohmann::json{{"reporting_sources", {"plc/axis1"}}}};
  EXPECT_TRUE(std::any_of(own_burst.begin(), own_burst.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, scope);
  }));
}

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
