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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_types.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

class MockFaultServiceTransport : public FaultServiceTransport {
 public:
  MockFaultServiceTransport() = default;
  ~MockFaultServiceTransport() override = default;
  MockFaultServiceTransport(const MockFaultServiceTransport &) = delete;
  MockFaultServiceTransport & operator=(const MockFaultServiceTransport &) = delete;
  MockFaultServiceTransport(MockFaultServiceTransport &&) = delete;
  MockFaultServiceTransport & operator=(MockFaultServiceTransport &&) = delete;

  FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                           const std::string & source_id) override {
    last_report_code_ = fault_code;
    last_report_severity_ = severity;
    last_report_description_ = description;
    last_report_source_ = source_id;
    ++report_calls_;
    FaultResult r;
    r.success = report_success_;
    r.data = report_data_;
    r.error_message = report_error_;
    return r;
  }

  FaultResult list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                          bool include_cleared, bool include_healed, bool include_muted,
                          bool include_clusters) override {
    last_list_source_ = source_id;
    last_include_prefailed_ = include_prefailed;
    last_include_confirmed_ = include_confirmed;
    last_include_cleared_ = include_cleared;
    last_include_healed_ = include_healed;
    last_include_muted_ = include_muted;
    last_include_clusters_ = include_clusters;
    ++list_calls_;
    FaultResult r;
    r.success = list_success_;
    r.data = list_data_;
    r.error_message = list_error_;
    return r;
  }

  FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code, const std::string & source_id) override {
    last_get_env_code_ = fault_code;
    last_get_env_source_ = source_id;
    ++get_env_calls_;
    FaultWithEnvJsonResult r;
    r.success = get_env_success_;
    r.data = get_env_data_;
    r.error_message = get_env_error_;
    return r;
  }

  FaultResult get_fault(const std::string & fault_code, const std::string & source_id) override {
    last_get_code_ = fault_code;
    last_get_source_ = source_id;
    ++get_calls_;
    FaultResult r;
    r.success = get_success_;
    r.data = get_data_;
    r.error_message = get_error_;
    return r;
  }

  FaultResult clear_fault(const std::string & fault_code, bool skip_correlation_auto_clear) override {
    last_clear_code_ = fault_code;
    last_clear_skip_correlation_ = skip_correlation_auto_clear;
    ++clear_calls_;
    FaultResult r;
    r.success = clear_success_;
    r.data = clear_data_;
    r.error_message = clear_error_;
    return r;
  }

  FaultResult get_snapshots(const std::string & fault_code, const std::string & topic) override {
    last_snapshots_code_ = fault_code;
    last_snapshots_topic_ = topic;
    ++snapshots_calls_;
    FaultResult r;
    r.success = snapshots_success_;
    r.data = snapshots_data_;
    r.error_message = snapshots_error_;
    return r;
  }

  FaultResult get_rosbag(const std::string & fault_code) override {
    last_rosbag_code_ = fault_code;
    ++rosbag_calls_;
    FaultResult r;
    r.success = rosbag_success_;
    r.data = rosbag_data_;
    r.error_message = rosbag_error_;
    return r;
  }

  FaultResult list_rosbags(const std::string & entity_fqn) override {
    last_list_rosbags_entity_ = entity_fqn;
    ++list_rosbags_calls_;
    FaultResult r;
    r.success = list_rosbags_success_;
    r.data = list_rosbags_data_;
    r.error_message = list_rosbags_error_;
    return r;
  }

  PlannedStopResult declare_planned_stop(const faults::PlannedStopWindow & request) override {
    last_declare_ = request;
    ++declare_calls_;
    PlannedStopResult r;
    r.success = planned_stop_success_;
    r.error_message = planned_stop_error_;
    r.failure = planned_stop_failure_;
    if (planned_stop_success_) {
      auto stored = request;
      stored.id = "declared-1";
      r.stops.push_back(stored);
    }
    return r;
  }

  PlannedStopResult end_planned_stop(const std::string & id) override {
    last_end_id_ = id;
    ++end_calls_;
    PlannedStopResult r;
    r.success = planned_stop_success_;
    r.error_message = planned_stop_error_;
    r.failure = planned_stop_failure_;
    if (planned_stop_success_) {
      faults::PlannedStopWindow ended;
      ended.id = id;
      ended.ended_early = true;
      r.stops.push_back(ended);
    }
    return r;
  }

  PlannedStopResult list_planned_stops(bool active_only) override {
    last_active_only_ = active_only;
    ++list_stops_calls_;
    PlannedStopResult r;
    r.success = planned_stop_success_;
    r.error_message = planned_stop_error_;
    r.failure = planned_stop_failure_;
    if (planned_stop_success_) {
      r.stops = planned_stops_;
    }
    return r;
  }

  bool wait_for_services(std::chrono::duration<double> timeout) override {
    last_wait_timeout_ = timeout.count();
    ++wait_calls_;
    return wait_result_;
  }

  bool is_available() const override {
    ++availability_calls_;
    return is_available_;
  }

  // Routing knobs / observation fields ---------------------------------------
  bool report_success_ = true;
  json report_data_ = json{{"accepted", true}};
  std::string report_error_;
  std::string last_report_code_;
  uint8_t last_report_severity_ = 0;
  std::string last_report_description_;
  std::string last_report_source_;
  int report_calls_ = 0;

  bool list_success_ = true;
  json list_data_ = json{{"faults", json::array()}, {"count", 0}};
  std::string list_error_;
  std::string last_list_source_;
  bool last_include_prefailed_ = false;
  bool last_include_confirmed_ = false;
  bool last_include_cleared_ = false;
  bool last_include_healed_ = false;
  bool last_include_muted_ = false;
  bool last_include_clusters_ = false;
  int list_calls_ = 0;

  bool get_env_success_ = true;
  json get_env_data_;
  std::string get_env_error_;
  std::string last_get_env_code_;
  std::string last_get_env_source_;
  int get_env_calls_ = 0;

  bool get_success_ = true;
  json get_data_;
  std::string get_error_;
  std::string last_get_code_;
  std::string last_get_source_;
  int get_calls_ = 0;

  bool clear_success_ = true;
  json clear_data_ = json{{"success", true}, {"message", "ok"}};
  std::string clear_error_;
  std::string last_clear_code_;
  bool last_clear_skip_correlation_ = false;
  int clear_calls_ = 0;

  bool snapshots_success_ = true;
  json snapshots_data_ = json::object();
  std::string snapshots_error_;
  std::string last_snapshots_code_;
  std::string last_snapshots_topic_;
  int snapshots_calls_ = 0;

  bool rosbag_success_ = true;
  json rosbag_data_ = json::object();
  std::string rosbag_error_;
  std::string last_rosbag_code_;
  int rosbag_calls_ = 0;

  bool list_rosbags_success_ = true;
  json list_rosbags_data_ = json{{"rosbags", json::array()}};
  std::string list_rosbags_error_;
  std::string last_list_rosbags_entity_;
  int list_rosbags_calls_ = 0;

  bool wait_result_ = true;
  double last_wait_timeout_ = 0.0;
  int wait_calls_ = 0;

  bool is_available_ = true;
  mutable int availability_calls_ = 0;
  // Planned-stop state. Kept beside the fault state so a test can make the
  // planned-stop half fail without disturbing the rest.
  faults::PlannedStopWindow last_declare_{};
  std::string last_end_id_;
  bool last_active_only_{false};
  std::vector<faults::PlannedStopWindow> planned_stops_;
  bool planned_stop_success_{true};
  std::string planned_stop_error_;
  FaultFailure planned_stop_failure_{FaultFailure::Declined};
  int declare_calls_{0};
  int end_calls_{0};
  int list_stops_calls_{0};
};

}  // namespace

TEST(FaultManagerRoutingTest, ReportFaultDelegatesAllArgsToTransport) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  FaultManager mgr(mock);

  auto r = mgr.report_fault("MOTOR_OVERHEAT", 3u, "engine reported overheat", "/powertrain/engine");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->report_calls_, 1);
  EXPECT_EQ(mock->last_report_code_, "MOTOR_OVERHEAT");
  EXPECT_EQ(mock->last_report_severity_, 3u);
  EXPECT_EQ(mock->last_report_description_, "engine reported overheat");
  EXPECT_EQ(mock->last_report_source_, "/powertrain/engine");
}

TEST(FaultManagerRoutingTest, ReportFaultPropagatesErrorMessage) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->report_success_ = false;
  mock->report_error_ = "ReportFault service not available";
  FaultManager mgr(mock);

  auto r = mgr.report_fault("X", 0u, "", "");

  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error_message, "ReportFault service not available");
}

TEST(FaultManagerRoutingTest, ListFaultsForwardsAllStatusFlags) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  FaultManager mgr(mock);

  auto r = mgr.list_faults("/cell_a", /*include_prefailed=*/true, /*include_confirmed=*/true,
                           /*include_cleared=*/true, /*include_healed=*/true, /*include_muted=*/true,
                           /*include_clusters=*/true);

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->list_calls_, 1);
  EXPECT_EQ(mock->last_list_source_, "/cell_a");
  EXPECT_TRUE(mock->last_include_prefailed_);
  EXPECT_TRUE(mock->last_include_confirmed_);
  EXPECT_TRUE(mock->last_include_cleared_);
  EXPECT_TRUE(mock->last_include_healed_);
  EXPECT_TRUE(mock->last_include_muted_);
  EXPECT_TRUE(mock->last_include_clusters_);
}

TEST(FaultManagerRoutingTest, GetFaultWithEnvReturnsTransportJsonShape) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  // Shape the transport returns: { "fault": {...}, "environment_data": {...} }
  // (rosbag snapshot intentionally omits bulk_data_uri - the handler adds it.)
  mock->get_env_data_ = {{"fault",
                          {{"fault_code", "F1"},
                           {"severity", 2},
                           {"severity_label", "WARN"},
                           {"status", "CONFIRMED"},
                           {"description", "test"}}},
                         {"environment_data",
                          {{"extended_data_records", {{"first_occurrence", "2026-01-01T00:00:00.000Z"}}},
                           {"snapshots", json::array({{{"type", "rosbag"},
                                                       {"name", "F1.bag"},
                                                       {"fault_code", "F1"},
                                                       {"size_bytes", 4096},
                                                       {"duration_sec", 5.0},
                                                       {"format", "mcap"}}})}}}};
  FaultManager mgr(mock);

  auto r = mgr.get_fault_with_env("F1", "/cell_a");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_get_env_code_, "F1");
  EXPECT_EQ(mock->last_get_env_source_, "/cell_a");
  ASSERT_TRUE(r.data.contains("fault"));
  ASSERT_TRUE(r.data.contains("environment_data"));
  EXPECT_EQ(r.data["fault"]["fault_code"], "F1");
  ASSERT_TRUE(r.data["environment_data"]["snapshots"].is_array());
  ASSERT_EQ(r.data["environment_data"]["snapshots"].size(), 1u);
  // Critical: the transport must NOT have populated bulk_data_uri at this stage.
  EXPECT_FALSE(r.data["environment_data"]["snapshots"][0].contains("bulk_data_uri"));
}

TEST(FaultManagerRoutingTest, GetFaultWithEnvCarriesFreezeFrameForExternalEntity) {
  // The per-entity detail route for an external plugin entity (e.g. a PLC over
  // OPC UA) reads the fault_manager's enriched record: reporting_sources=[bare
  // id] plus a freeze-frame snapshot. Pin that get_fault_with_env passes this
  // shape through verbatim so the handler can build the SOVD detail with the
  // freeze-frame.
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->get_env_data_ = {{"fault",
                          {{"fault_code", "PLC_LEVEL_HIGH"},
                           {"severity", 2},
                           {"severity_label", "ERROR"},
                           {"status", "CONFIRMED"},
                           {"reporting_sources", json::array({"process"})}}},
                         {"environment_data",
                          {{"snapshots", json::array({{{"type", "freeze_frame"},
                                                       {"snapshot_type", "freeze_frame"},
                                                       {"name", "/plc/process/level"},
                                                       {"topic", "/plc/process/level"},
                                                       {"message_type", "std_msgs/msg/Float64"},
                                                       {"data", R"({"data": 150.0})"}}})}}}};
  FaultManager mgr(mock);

  auto r = mgr.get_fault_with_env("PLC_LEVEL_HIGH", "");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_get_env_code_, "PLC_LEVEL_HIGH");
  ASSERT_TRUE(r.data.contains("fault"));
  EXPECT_EQ(r.data["fault"]["reporting_sources"][0], "process");
  ASSERT_TRUE(r.data["environment_data"]["snapshots"].is_array());
  ASSERT_EQ(r.data["environment_data"]["snapshots"].size(), 1u);
  EXPECT_EQ(r.data["environment_data"]["snapshots"][0]["type"], "freeze_frame");
}

TEST(FaultManagerRoutingTest, GetFaultWithEnvPropagatesErrorMessage) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->get_env_success_ = false;
  mock->get_env_error_ = "Fault not found";
  FaultManager mgr(mock);

  auto r = mgr.get_fault_with_env("MISSING");

  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error_message, "Fault not found");
  EXPECT_TRUE(r.data.empty());
}

TEST(FaultManagerRoutingTest, ClearFaultDelegatesAndReturnsAutoClearedCodes) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->clear_data_ = {{"success", true}, {"message", "ok"}, {"auto_cleared_codes", json::array({"S1", "S2"})}};
  FaultManager mgr(mock);

  auto r = mgr.clear_fault("ROOT");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_clear_code_, "ROOT");
  ASSERT_TRUE(r.data.contains("auto_cleared_codes"));
  EXPECT_EQ(r.data["auto_cleared_codes"].size(), 2u);
}

TEST(FaultManagerRoutingTest, ClearFaultDefaultsToCorrelationAutoClear) {
  // Global `DELETE /api/v1/faults` and any unscoped caller should preserve
  // the legacy auto-clear-with-root behaviour (skip flag defaults to false).
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->clear_success_ = true;
  FaultManager mgr(mock);

  mgr.clear_fault("ROOT");

  EXPECT_EQ(mock->last_clear_code_, "ROOT");
  EXPECT_FALSE(mock->last_clear_skip_correlation_);
}

TEST(FaultManagerRoutingTest, ClearFaultForwardsSkipCorrelationFlag) {
  // Per-entity DELETE routes call `clear_fault(code, /*skip=*/true)` so the
  // fault manager does NOT cascade-clear correlated symptoms reported by
  // apps outside the addressed entity. Pin the routing here.
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->clear_success_ = true;
  FaultManager mgr(mock);

  mgr.clear_fault("ROOT", /*skip_correlation_auto_clear=*/true);

  EXPECT_EQ(mock->last_clear_code_, "ROOT");
  EXPECT_TRUE(mock->last_clear_skip_correlation_);
}

TEST(FaultManagerRoutingTest, GetSnapshotsRoutesTopicFilter) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->snapshots_data_ = {{"topics", json::object()}};
  FaultManager mgr(mock);

  auto r = mgr.get_snapshots("F1", "/joint_states");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_snapshots_code_, "F1");
  EXPECT_EQ(mock->last_snapshots_topic_, "/joint_states");
}

TEST(FaultManagerRoutingTest, GetRosbagPropagatesErrorMessage) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->rosbag_success_ = false;
  mock->rosbag_error_ = "No rosbag file available for fault";
  FaultManager mgr(mock);

  auto r = mgr.get_rosbag("F1");

  EXPECT_FALSE(r.success);
  EXPECT_EQ(mock->last_rosbag_code_, "F1");
  EXPECT_EQ(r.error_message, "No rosbag file available for fault");
}

TEST(FaultManagerRoutingTest, ListRosbagsForwardsEntityFqn) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  FaultManager mgr(mock);

  auto r = mgr.list_rosbags("/sensors/lidar");

  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_list_rosbags_entity_, "/sensors/lidar");
}

TEST(FaultManagerRoutingTest, IsAvailableAndWaitForwardToTransport) {
  auto mock = std::make_shared<MockFaultServiceTransport>();
  mock->is_available_ = true;
  mock->wait_result_ = true;
  FaultManager mgr(mock);

  EXPECT_TRUE(mgr.is_available());
  EXPECT_GE(mock->availability_calls_, 1);

  EXPECT_TRUE(mgr.wait_for_services(std::chrono::duration<double>(2.5)));
  EXPECT_EQ(mock->wait_calls_, 1);
  EXPECT_DOUBLE_EQ(mock->last_wait_timeout_, 2.5);
}

}  // namespace ros2_medkit_gateway
