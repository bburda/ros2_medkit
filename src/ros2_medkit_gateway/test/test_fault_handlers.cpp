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

#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/extended_data_records.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::handlers::FaultHandlers;
namespace conversions = ros2_medkit_gateway::ros2::conversions;
namespace dto = ros2_medkit_gateway::dto;

// The handler now consumes JSON shaped by the transport adapter. These tests
// drive that contract end-to-end by using the same conversions module the
// adapter uses to translate ros2_medkit_msgs into JSON, then call the handler
// to produce the final SOVD response (now a dto::FaultDetail struct).
//
// Tests convert the DTO back to JSON via JsonWriter for comparison so existing
// assertions can remain wire-level checks.

class FaultHandlersTest : public ::testing::Test {
 protected:
  static json fault_json(const ros2_medkit_msgs::msg::Fault & f) {
    return conversions::fault_to_json(f);
  }
  static json env_json(const ros2_medkit_msgs::msg::EnvironmentData & e) {
    return conversions::environment_data_to_json(e);
  }
  /// Convert the DTO returned by build_sovd_fault_response to JSON for
  /// wire-level assertions (keeps test bodies as close to original as possible).
  static json to_json(const dto::FaultDetail & detail) {
    return dto::JsonWriter<dto::FaultDetail>::write(detail);
  }
};

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseBasicFields) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEST_FAULT";
  fault.description = "Test fault description";
  fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;  // ERROR (2)
  fault.status = "CONFIRMED";                                     // Active/confirmed fault
  fault.occurrence_count = 5;
  fault.reporting_sources = {"/powertrain/motor_controller"};

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  env_data.extended_data_records.first_occurrence_ns = 1707044400000000000;
  env_data.extended_data_records.last_occurrence_ns = 1707044460000000000;

  auto response = to_json(
      FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor_controller"));

  // Verify item structure
  EXPECT_EQ(response["item"]["code"], "TEST_FAULT");
  EXPECT_EQ(response["item"]["fault_name"], "Test fault description");
  EXPECT_EQ(response["item"]["severity"], ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR);

  // Verify status object
  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "active");
  EXPECT_EQ(status["testFailed"], "1");
  EXPECT_EQ(status["confirmedDTC"], "1");
  EXPECT_EQ(status["pendingDTC"], "0");

  // Verify x-medkit
  EXPECT_EQ(response["x-medkit"]["occurrence_count"], 5);
  EXPECT_EQ(response["x-medkit"]["severity_label"], "ERROR");
  EXPECT_EQ(response["x-medkit"]["status_raw"],
            "CONFIRMED");  // Raw status in x-medkit, not in status object
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithFreezeFrame) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEMP_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 85.5, "variance": 0.1})";
  freeze_frame.topic = "/motor/temperature";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  freeze_frame.captured_at_ns = 1707044400000000000;
  env_data.snapshots.push_back(freeze_frame);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "temperature");
  EXPECT_DOUBLE_EQ(snap["data"].get<double>(), 85.5);  // Primary value extracted
  EXPECT_EQ(snap["x-medkit"]["topic"], "/motor/temperature");
  EXPECT_EQ(snap["x-medkit"]["message_type"], "sensor_msgs/msg/Temperature");
  EXPECT_FALSE(snap["x-medkit"].contains("capture_origin"));  // confirm-edge capture
}

// @verifies REQ_INTEROP_088
TEST_F(FaultHandlersTest, BuildSovdFaultResponsePropagatesCaptureOrigin) {
  // Startup catch-up frames (entity freeze-frames taken for faults that were
  // already confirmed at gateway start) carry capture_origin through to the
  // wire x-medkit block, so consumers can tell them from confirm-time frames.
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "STANDING_FAULT";

  json env_data = {{"snapshots", json::array({{{"type", "freeze_frame"},
                                               {"snapshot_type", "freeze_frame"},
                                               {"name", "plc_app"},
                                               {"data", R"({"level": 7.0})"},
                                               {"topic", ""},
                                               {"message_type", ""},
                                               {"captured_at_ns", 1234},
                                               {"capture_origin", "startup"}}})}};

  auto response = to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_data, "/apps/plc_app"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["x-medkit"]["capture_origin"], "startup");
}

// Conversion layer must emit an explicit "snapshot_type" discriminator so
// downstream consumers (handler, SSE, MCP) can dispatch on a single key
// regardless of which optional payload fields are present.
TEST_F(FaultHandlersTest, EnvironmentDataToJsonEmitsSnapshotTypeDiscriminator) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 85.5})";
  freeze_frame.topic = "/motor/temperature";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  env_data.snapshots.push_back(freeze_frame);

  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "fault_recording";
  rosbag.bulk_data_id = "ROSBAG_X";
  rosbag.size_bytes = 1234;
  rosbag.duration_sec = 1.0;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto j = env_json(env_data);
  ASSERT_TRUE(j.contains("snapshots"));
  ASSERT_EQ(j["snapshots"].size(), 2u);
  EXPECT_EQ(j["snapshots"][0]["snapshot_type"], "freeze_frame");
  EXPECT_EQ(j["snapshots"][1]["snapshot_type"], "rosbag");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithRosbag) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "ROSBAG_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "fault_recording";
  rosbag.bulk_data_id = "ROSBAG_FAULT";
  rosbag.size_bytes = 1234567;
  rosbag.duration_sec = 6.0;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto response = to_json(
      FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor_controller"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "rosbag");
  EXPECT_EQ(snap["bulk_data_uri"], "/apps/motor_controller/bulk-data/rosbags/ROSBAG_FAULT");
  EXPECT_EQ(snap["size_bytes"], 1234567);
  EXPECT_DOUBLE_EQ(snap["duration_sec"], 6.0);
  EXPECT_EQ(snap["format"], "mcap");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseNestedEntityPath) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "NESTED_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.bulk_data_id = "NESTED_FAULT";
  env_data.snapshots.push_back(rosbag);

  auto response = to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data),
                                                                   "/areas/perception/subareas/lidar"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["bulk_data_uri"], "/areas/perception/subareas/lidar/bulk-data/rosbags/NESTED_FAULT");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseStatusCleared) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "CLEARED_FAULT";
  fault.status = "CLEARED";  // Cleared status

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "cleared");
  EXPECT_EQ(status["testFailed"], "0");
  EXPECT_EQ(status["confirmedDTC"], "0");
  EXPECT_EQ(status["pendingDTC"], "0");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseStatusPassive) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "PASSIVE_FAULT";
  fault.status = "PREFAILED";  // Pending/passive status

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "passive");
  EXPECT_EQ(status["pendingDTC"], "1");
}

// @verifies REQ_INTEROP_013
// Severity labels must mirror ros2_medkit_msgs/msg/Fault.msg constants:
// SEVERITY_INFO=0, SEVERITY_WARN=1, SEVERITY_ERROR=2, SEVERITY_CRITICAL=3.
TEST_F(FaultHandlersTest, BuildSovdFaultResponseSeverityLabels) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Test INFO (0)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_INFO;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "INFO");
  }
  // Test WARN (1)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "WARN");
  }
  // Test ERROR (2)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "ERROR");
  }
  // Test CRITICAL (3)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "CRITICAL");
  }
  // Test UNKNOWN (255) - any value outside the SEVERITY_* range
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = 255;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "UNKNOWN");
  }
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithInvalidJson) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "INVALID_JSON_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "invalid_data";
  freeze_frame.data = "not valid json {";
  freeze_frame.topic = "/test";
  freeze_frame.message_type = "std_msgs/msg/String";
  env_data.snapshots.push_back(freeze_frame);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["data"], "not valid json {");  // Raw data returned
  EXPECT_TRUE(snap["x-medkit"].contains("parse_error"));
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseExtendedDataRecords) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEST_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  env_data.extended_data_records.first_occurrence_ns = 1770458400000000000;  // 2026-02-08
  env_data.extended_data_records.last_occurrence_ns = 1770458460000000000;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto & edr = response["environment_data"]["extended_data_records"];
  std::string first = edr["first_occurrence"].get<std::string>();
  std::string last = edr["last_occurrence"].get<std::string>();

  // Verify ISO 8601 format with milliseconds and Z suffix
  EXPECT_TRUE(first.find("2026") != std::string::npos);
  EXPECT_TRUE(first.find('Z') != std::string::npos);
  EXPECT_TRUE(first.find('T') != std::string::npos);
  EXPECT_TRUE(last.find('Z') != std::string::npos);
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponsePrimaryValueExtraction) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Test std_msgs/msg/Float64 - should extract "data" field
  {
    ros2_medkit_msgs::msg::Fault fault;
    ros2_medkit_msgs::msg::Snapshot snap;
    snap.type = "freeze_frame";
    snap.data = R"({"data": 42.5})";
    snap.message_type = "std_msgs/msg/Float64";
    env_data.snapshots.clear();
    env_data.snapshots.push_back(snap);

    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_DOUBLE_EQ(response["environment_data"]["snapshots"][0]["data"].get<double>(), 42.5);
  }

  // Test unknown message type - should return full data
  {
    ros2_medkit_msgs::msg::Fault fault;
    ros2_medkit_msgs::msg::Snapshot snap;
    snap.type = "freeze_frame";
    snap.data = R"({"foo": "bar", "baz": 123})";
    snap.message_type = "custom_msgs/msg/Unknown";
    env_data.snapshots.clear();
    env_data.snapshots.push_back(snap);

    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    auto data = response["environment_data"]["snapshots"][0]["data"];
    EXPECT_EQ(data["foo"], "bar");
    EXPECT_EQ(data["baz"], 123);
  }
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseMultipleSources) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "MULTI_SOURCE_FAULT";
  fault.reporting_sources = {"/perception/lidar", "/perception/camera", "/control/motor"};

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto sources = response["x-medkit"]["reporting_sources"];
  ASSERT_EQ(sources.size(), 3);
  EXPECT_EQ(sources[0], "/perception/lidar");
  EXPECT_EQ(sources[1], "/perception/camera");
  EXPECT_EQ(sources[2], "/control/motor");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseMixedSnapshots) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "MIXED_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Add freeze frame
  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 75.0})";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  env_data.snapshots.push_back(freeze_frame);

  // Add rosbag
  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "recording";
  rosbag.bulk_data_id = "MIXED_FAULT";
  rosbag.size_bytes = 1000;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/components/motor"));

  ASSERT_EQ(response["environment_data"]["snapshots"].size(), 2);

  // Verify freeze frame
  auto & snap0 = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap0["type"], "freeze_frame");
  EXPECT_DOUBLE_EQ(snap0["data"].get<double>(), 75.0);

  // Verify rosbag
  auto & snap1 = response["environment_data"]["snapshots"][1];
  EXPECT_EQ(snap1["type"], "rosbag");
  EXPECT_EQ(snap1["bulk_data_uri"], "/components/motor/bulk-data/rosbags/MIXED_FAULT");
}

// =============================================================================
// fault_in_source_scope tests (#395)
//
// Direct coverage for the scope-check logic that backs per-entity GET/DELETE
// and the collection-route filter. Integration tests pin the HTTP behavior;
// these tests pin the boundary semantics that decide what counts as "in
// scope" - especially the cases that motivated the post-review tightening:
// prefix-colliding FQNs and multi-source faults.
// =============================================================================

namespace {

json make_fault(const std::vector<std::string> & reporting_sources, const std::string & code = "F1") {
  json f;
  f["fault_code"] = code;
  f["reporting_sources"] = reporting_sources;
  return f;
}

}  // namespace

TEST(FaultInSourceScopeTest, SingleSourceExactMatchInScope) {
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor"}),
                                                   {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, SubpathOfScopedFqnIsInScope) {
  // A node nested under the entity's own FQN must still match - e.g. an app
  // FQN `/perception/lidar/lidar_sensor` and a reporter at
  // `/perception/lidar/lidar_sensor/diagnostic_updater` should be the same
  // app's sub-node, not a stranger.
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor/diagnostic_updater"}),
                                                   {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, PrefixCollidingNameIsNotInScope) {
  // `/ns/node_extra` shares the `/ns/node` prefix but is a distinct ROS node.
  // The pre-review check used `rfind(prefix, 0) == 0` and silently let it
  // through - this test pins the path-boundary fix.
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/ns/node_extra"}), {"/ns/node"}));
}

TEST(FaultInSourceScopeTest, MultiSourceAllInScopeIsInScope) {
  std::set<std::string> scope{"/perception/lidar/lidar_sensor", "/perception/rgb/rgb_camera"};
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(
      make_fault({"/perception/lidar/lidar_sensor", "/perception/rgb/rgb_camera"}), scope));
}

TEST(FaultInSourceScopeTest, MultiSourcePartiallyInScopeIsOutOfScope) {
  // The all-sources semantic blocks a cross-entity DELETE escalation: an
  // entity that owns only `/perception/lidar/lidar_sensor` must not be able
  // to clear (or read in detail) a fault that another entity in
  // `/telemetry/...` also reported.
  std::set<std::string> scope{"/perception/lidar/lidar_sensor"};
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(
      make_fault({"/perception/lidar/lidar_sensor", "/telemetry/telemetry_node"}), scope));
}

TEST(FaultInSourceScopeTest, EmptyScopeSetIsOutOfScope) {
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor"}), {}));
}

TEST(FaultInSourceScopeTest, EmptyReportingSourcesIsOutOfScope) {
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({}), {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, MissingReportingSourcesFieldIsOutOfScope) {
  json fault;
  fault["fault_code"] = "F1";
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(fault, {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, NonStringSourceEntryIsOutOfScope) {
  json fault;
  fault["fault_code"] = "F1";
  fault["reporting_sources"] = json::array();
  fault["reporting_sources"].push_back("/perception/lidar/lidar_sensor");
  fault["reporting_sources"].push_back(42);  // Malformed entry
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(fault, {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, RootFqnEdgeCase) {
  // Boundary check still works with "/" prefix candidates (artificial but
  // exercises the index arithmetic): "/" itself can't match "/anything"
  // unless the scope set actually contains "/" - which our resolver never
  // emits. This pin catches a future regression that special-cased "/".
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/something"}), {"/other"}));
}

// -----------------------------------------------------------------------------
// External protocol-plugin entity ownership: bare-id scope sets.
//
// An external entity (e.g. a PLC over OPC UA) has no ROS FQN, so fault-scope
// resolution grants it a scope set of exactly {its bare id}. These pins prove
// the guard treats a bare id like any other scope token: it owns faults
// reported under that id, and still blocks cross-entity disclosure (GET) and
// cross-entity clear (DELETE), which both flow through this predicate.
// -----------------------------------------------------------------------------

TEST(FaultInSourceScopeTest, BareEntityIdScopeMatchesOwnFault) {
  // The entity owns the fault it reported under its own id.
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"process"}), {"process"}));
}

TEST(FaultInSourceScopeTest, BareEntityIdScopeMatchesOwnSubPath) {
  // A sub-node under the entity id (path boundary) is still the same owner.
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"process/sub"}), {"process"}));
}

TEST(FaultInSourceScopeTest, BareEntityIdScopeRejectsOtherEntityFault) {
  // Cross-entity GET stays blocked: entity "process" must not see a fault that
  // another external entity ("s7_status") reported under its own id.
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"s7_status"}), {"process"}));
}

TEST(FaultInSourceScopeTest, BareEntityIdScopeRejectsMixedSourceFault) {
  // Cross-entity clear/disclosure stays blocked under the all-sources rule: a
  // fault co-reported by "process" and another entity is out of scope for
  // "process" alone.
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"process", "s7_status"}), {"process"}));
}

TEST(FaultInSourceScopeTest, BareEntityIdScopeRejectsPrefixCollision) {
  // "process_2" shares the "process" prefix but is a distinct entity id; the
  // path-boundary check must reject it (no bare-substring escalation).
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"process_2"}), {"process"}));
}

// The per-entity detail for an external plugin entity must carry the
// freeze-frame the fault_manager holds, mirroring the non-plugin detail path.
// This pins the shape build_sovd_fault_response produces for the PLC case:
// reporting_sources=[bare id] + a freeze-frame snapshot under a plugin entity
// path.
TEST_F(FaultHandlersTest, BuildSovdFaultResponseExternalEntityFreezeFrame) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "PLC_LEVEL_HIGH";
  fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  fault.status = "CONFIRMED";
  fault.reporting_sources = {"process"};  // bare external entity id

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "/plc/process/level";
  freeze_frame.data = R"({"data": 150.0})";
  freeze_frame.topic = "/plc/process/level";
  freeze_frame.message_type = "std_msgs/msg/Float64";
  freeze_frame.captured_at_ns = 1707044400000000000;
  env_data.snapshots.push_back(freeze_frame);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/process"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "/plc/process/level");
  EXPECT_DOUBLE_EQ(snap["data"].get<double>(), 150.0);
  EXPECT_EQ(response["x-medkit"]["reporting_sources"][0], "process");
}

// =============================================================================
// FaultListItem schema <-> fault_to_json producer contract.
//
// The fault list endpoints emit items verbatim from fault_to_json inside the
// opaque FaultListResult envelope (never through JsonWriter<FaultListItem>), so
// the published FaultListItem schema is otherwise an unverified claim about that
// wire. These tests pin the schema to its real in-tree producer: any field key,
// type, or enum drift in fault_to_json now fails here instead of silently
// diverging the spec from the wire (the drift the DTO contract exists to kill).
// =============================================================================
TEST(FaultListItemSchema, FaultToJsonConformsAndRoundTrips) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "BRAKE_PRESSURE_LOW";
  fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  fault.description = "Brake pressure below threshold";
  fault.occurrence_count = 3;
  fault.status = "active";
  fault.reporting_sources = {"brake_ecu", "abs_node"};
  fault.last_passed.sec = 1200;  // absent-when-zero covered separately below

  const json wire = conversions::fault_to_json(fault);

  // The published FaultListItem schema must accept the verbatim wire ...
  const auto parsed = dto::JsonReader<dto::FaultListItem>::read(wire);
  ASSERT_TRUE(parsed.has_value()) << "fault_to_json output does not conform to FaultListItem";
  // ... and round-trip back to identical wire (no field added or dropped).
  EXPECT_EQ(dto::JsonWriter<dto::FaultListItem>::write(parsed.value()), wire);
}

TEST(FaultListItemSchema, LastPassedOmittedWhenNeverPassed) {
  // last_passed carries the last PASSED instant; zero means the fault never
  // reported PASSED, and the wire says that by omitting the key entirely.
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "NEVER_PASSED";
  fault.status = "active";

  EXPECT_FALSE(conversions::fault_to_json(fault).contains("last_passed"));

  fault.last_passed.sec = 1200;
  fault.last_passed.nanosec = 500000000;
  EXPECT_DOUBLE_EQ(conversions::fault_to_json(fault)["last_passed"].get<double>(), 1200.5);
}

TEST(FaultListItemSchema, UnknownSeverityLabelIsAcceptedByEnum) {
  // fault_to_json maps any severity outside the four known levels to "UNKNOWN",
  // so the FaultListItem.severity_label enum vocabulary must include it.
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "MYSTERY";
  fault.severity = 99;  // outside SEVERITY_INFO..SEVERITY_CRITICAL
  fault.status = "active";

  const json wire = conversions::fault_to_json(fault);
  ASSERT_EQ(wire["severity_label"], "UNKNOWN");

  const auto parsed = dto::JsonReader<dto::FaultListItem>::read(wire);
  ASSERT_TRUE(parsed.has_value()) << "UNKNOWN severity_label rejected by FaultListItem enum";
  EXPECT_EQ(dto::JsonWriter<dto::FaultListItem>::write(parsed.value()), wire);
}

// =============================================================================
// classify_fault_failure - 404-vs-503 on a failed fault-manager call
// =============================================================================
//
// The split has to rest on which layer failed, not on how the failure reads.
// It rested on a substring match for "not found" over the store's message, so
// a refusal the fault manager worded any other way - its own `fault_code`
// validation among them - was served as 503, telling the client the gateway
// had a problem when the request did.

// A fault the store does not hold. The wording here is the fault manager's
// own, and the classification must not depend on it.
// @verifies REQ_INTEROP_013
// @verifies REQ_INTEROP_015
TEST(ClassifyFaultFailureTest, DeclinedIsAClientError) {
  const auto err =
      FaultHandlers::classify_fault_failure(ros2_medkit_gateway::FaultFailure::Declined, "Fault not found: NO_SUCH",
                                            "Failed to get fault", "app_id", "lidar_sensor", "NO_SUCH");
  EXPECT_EQ(err.http_status, 404);
  EXPECT_EQ(err.code, ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
  EXPECT_EQ(err.params["details"], "Fault not found: NO_SUCH");
  EXPECT_EQ(err.params["app_id"], "lidar_sensor");
  EXPECT_EQ(err.params["fault_code"], "NO_SUCH");
}

// The regression case: a refusal that never contains "not found". Before the
// classification was typed, this fell through the substring match to 503.
// @verifies REQ_INTEROP_013
// @verifies REQ_INTEROP_015
TEST(ClassifyFaultFailureTest, ARefusalWordedAnyOtherWayIsStillAClientError) {
  for (const char * message : {"fault_code exceeds maximum length of 256",
                               "fault_code contains invalid character '~'. Only alphanumeric, underscore, hyphen, "
                               "and dot are allowed",
                               "fault_code cannot contain '..'", "some wording nobody has written yet"}) {
    const auto err = FaultHandlers::classify_fault_failure(ros2_medkit_gateway::FaultFailure::Declined, message,
                                                           "Failed to get fault", "app_id", "lidar_sensor", "F~F");
    EXPECT_EQ(err.http_status, 404) << "declined refusal reported as " << err.http_status
                                    << " for message: " << message;
    EXPECT_LT(err.http_status, 500) << "a refusal must never be a server error: " << message;
  }
}

// The other direction must keep working: a fault manager that never answered
// is a real server-side failure and has to stay 503, or an outage would be
// indistinguishable from a missing fault.
// @verifies REQ_INTEROP_013
// @verifies REQ_INTEROP_015
TEST(ClassifyFaultFailureTest, UnavailableIsAServerError) {
  for (const char * message :
       {"GetFault service not available", "GetFault service call timed out", "GetFault transport not initialised"}) {
    const auto err = FaultHandlers::classify_fault_failure(ros2_medkit_gateway::FaultFailure::Unavailable, message,
                                                           "Failed to clear fault", "component_id", "host", "CODE");
    EXPECT_EQ(err.http_status, 503) << "message: " << message;
    EXPECT_EQ(err.code, ros2_medkit_gateway::ERR_SERVICE_UNAVAILABLE);
    EXPECT_EQ(err.message, "Failed to clear fault");
    EXPECT_EQ(err.params["details"], message);
  }
}
