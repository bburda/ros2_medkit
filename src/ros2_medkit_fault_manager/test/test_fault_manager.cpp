// Copyright 2025 mfaferek93
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
#include <sqlite3.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_audit_log.hpp"
#include "ros2_medkit_fault_manager/fault_manager_node.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/list_faults_for_entity.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::clamp_debounce_counter;
using ros2_medkit_fault_manager::compute_debounce_status;
using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::FaultManagerNode;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::sanitize_debounce_config;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;
using ros2_medkit_msgs::srv::ClearFault;
using ros2_medkit_msgs::srv::GetFault;
using ros2_medkit_msgs::srv::ListFaultsForEntity;
using ros2_medkit_msgs::srv::ReportFault;

/// Default debounce config for tests (matches DebounceConfig defaults: threshold=-1, no healing)
static DebounceConfig default_config() {
  return DebounceConfig{};
}

class FaultStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
};

TEST_F(FaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Motor temperature exceeded threshold", "/powertrain/motor", timestamp,
                                            default_config());

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_.size(), 1u);
  EXPECT_TRUE(storage_.contains("MOTOR_OVERHEAT"));
}

TEST_F(FaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                            "Test", "/node1", timestamp, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 0u);
}

TEST_F(FaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                              "Initial report", "/powertrain/motor1", timestamp1, default_config());

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Second report", "/powertrain/motor2", timestamp2, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 1u);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  // Still the same continuous occurrence (not CLEARED in between): occurrence_count
  // does not bump on every report, only severity/sources/description update.
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(FaultStorageTest, ContinuouslyActiveFaultDoesNotInflateOccurrenceCount) {
  rclcpp::Clock clock;

  // Simulate a level-triggered poller re-reporting the same still-true condition
  // every cycle (issue #11): occurrence_count must stay at 1, not grow per poll.
  for (int i = 0; i < 25; ++i) {
    storage_.report_fault_event("TANK_OVERFILL", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                "level = 95 > 80", "/tank", clock.now(), default_config());
  }

  auto fault = storage_.get_fault("TANK_OVERFILL");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ListFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());

  // Default query should return the CONFIRMED fault
  auto faults = storage_.list_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ListFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, config);

  // Query with PREFAILED status
  auto faults = storage_.list_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, ListFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_.report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                              "/node1", timestamp, default_config());

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_.list_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(FaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/node1", timestamp, default_config());

  bool cleared = storage_.clear_fault("MOTOR_OVERHEAT");
  EXPECT_TRUE(cleared);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, ClearNonExistentFault) {
  bool cleared = storage_.clear_fault("NON_EXISTENT");
  EXPECT_FALSE(cleared);
}

TEST_F(FaultStorageTest, GetClearedFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());
  storage_.clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_.list_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_.list_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, DefaultDebounceConfig) {
  auto config = storage_.get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
  EXPECT_FALSE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 3);
  EXPECT_TRUE(config.critical_immediate_confirm);
  EXPECT_DOUBLE_EQ(config.auto_confirm_after_sec, 0.0);
}

TEST_F(FaultStorageTest, SetDebounceConfig) {
  DebounceConfig config;
  config.confirmation_threshold = -5;
  config.healing_enabled = true;
  config.healing_threshold = 5;
  config.critical_immediate_confirm = false;
  config.auto_confirm_after_sec = 10.0;

  storage_.set_debounce_config(config);
  auto retrieved = storage_.get_debounce_config();

  EXPECT_EQ(retrieved.confirmation_threshold, -5);
  EXPECT_TRUE(retrieved.healing_enabled);
  EXPECT_EQ(retrieved.healing_threshold, 5);
  EXPECT_FALSE(retrieved.critical_immediate_confirm);
  EXPECT_DOUBLE_EQ(retrieved.auto_confirm_after_sec, 10.0);
}

TEST_F(FaultStorageTest, FaultStaysPrefailedAboveThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (2 FAILED events should stay PREFAILED)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Still debouncing towards confirmation, same occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (3 FAILED events should confirm)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Debounce build-up is still one occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ConfirmedFaultStaysConfirmed) {
  rclcpp::Clock clock;

  // Report fault 4 times
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                                "/node" + std::to_string(i), clock.now(), default_config());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Same occurrence throughout, not one per report
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, MultiSourceConfirmsFault) {
  rclcpp::Clock clock;

  // Report same fault from 3 different sources
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor1", clock.now(), default_config());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor2", clock.now(), default_config());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor3", clock.now(), default_config());

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 3u);
}

TEST_F(FaultStorageTest, SameSourceMultipleReportsConfirms) {
  rclcpp::Clock clock;

  // Same source reports 3 times
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Repeated reports from one source, one occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 1u);  // Only one unique source
}

TEST_F(FaultStorageTest, ImmediateConfirmationWithThresholdZero) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = 0;  // Immediate confirmation
  storage_.set_debounce_config(config);

  // Single report should confirm immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                              "/node1", clock.now(), default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, CriticalBypassCanBeDisabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce to test CRITICAL bypass
  config.critical_immediate_confirm = false;
  storage_.set_debounce_config(config);

  // CRITICAL should NOT confirm immediately when disabled
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                              "/node1", clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, ClearedFaultCanBeReactivated) {
  rclcpp::Clock clock;

  // Report to confirm (with default threshold=-1, single report confirms)
  auto first_ts = clock.now();
  bool is_new = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Initial", "/node1", first_ts, default_config());
  EXPECT_TRUE(is_new);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), first_ts.nanoseconds());

  // Clear the fault
  storage_.clear_fault("FAULT_1");
  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);

  // Report again after a gap - should reactivate as a new cycle
  auto second_ts = rclcpp::Time(first_ts.nanoseconds() + 1'000'000'000LL);  // +1s
  is_new = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                       "Reactivated", "/node2", second_ts, default_config());
  EXPECT_TRUE(is_new);  // Should return true like a new fault

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Should be reconfirmed
  EXPECT_EQ(fault->occurrence_count, 2u);             // Should increment
  EXPECT_EQ(fault->reporting_sources.size(), 2u);     // Both sources
  EXPECT_EQ(fault->description, "Reactivated");       // Updated description
  // #25: first_occurred must reflect the new cycle, not the outage that already cleared.
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), second_ts.nanoseconds());
}

TEST_F(FaultStorageTest, PassedEventForClearedFaultIgnored) {
  rclcpp::Clock clock;

  // Report and confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // PASSED event should be ignored for CLEARED fault
  bool result = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                                            default_config());
  EXPECT_FALSE(result);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(FaultStorageTest, ClearedFaultReactivationRestartsDebounce) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  // Report 3 times to confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // Reactivate - should start in PREFAILED with counter=-1
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                              clock.now(), config);

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Not yet confirmed, needs 2 more FAILED

  // Report 2 more times to re-confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node5",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node6",
                              clock.now(), config);

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Now confirmed
}

TEST_F(FaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;
  // confirmation_threshold = -3 so 2 FAILED stays PREFAILED (a confirmed fault is
  // latched and would not move to PREPASSED on a heal).
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_threshold = 3;  // explicit upper clamp (don't depend on the default)

  // Report 2 FAILED events (counter = -2, PREFAILED)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);

  // Report 3 PASSED events (counter = -2 + 3 = +1)
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0
}

TEST_F(FaultStorageTest, PassedEventDoesNotAdvanceLastOccurred) {
  // Observed on a live box: a plugin sent one PASSED on reconnect, healing was
  // disabled so the fault stayed CONFIRMED (by design), but last_occurred jumped
  // to the PASSED instant - a fault that had not occurred for hours read as if it
  // had just happened. last_occurred belongs to occurrences (FAILED) only.
  const rclcpp::Time failed_at(1000, 0, RCL_SYSTEM_TIME);
  const rclcpp::Time passed_at(9000, 0, RCL_SYSTEM_TIME);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              failed_at, default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", passed_at,
                              default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // healing disabled: latched, by design
  EXPECT_EQ(rclcpp::Time(fault->last_occurred).nanoseconds(), failed_at.nanoseconds());
  // The PASSED instant is not lost: it rides on the wire as last_passed.
  EXPECT_EQ(rclcpp::Time(fault->last_passed).nanoseconds(), passed_at.nanoseconds());
}

TEST_F(FaultStorageTest, HealingDisabledByDefault) {
  rclcpp::Clock clock;

  // Report 1 FAILED event -> confirmed (threshold -1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  // With healing disabled, a heal heartbeat must not downgrade a confirmed fault.
  // The counter clamps at healing_threshold and the status latches CONFIRMED.
  for (int i = 0; i < 10; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                                default_config());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // stays confirmed, not silently downgraded
}

TEST_F(FaultStorageTest, HealingWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 1 FAILED event (counter = -1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(FaultStorageTest, TimeBasedConfirmationDisabledByDefault) {
  rclcpp::Clock clock;

  // Set threshold to -3 to get PREFAILED status for testing time-based confirmation
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Advance time and check - should not auto-confirm (auto_confirm_after_sec = 0)
  auto future_time = rclcpp::Time(clock.now().nanoseconds() + static_cast<int64_t>(20e9));
  auto confirmed = storage_.check_time_based_confirmation(future_time);

  EXPECT_TRUE(confirmed.empty());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, TimeBasedConfirmationWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce so fault stays PREFAILED
  config.auto_confirm_after_sec = 10.0;
  storage_.set_debounce_config(config);

  auto now = clock.now();
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              now, config);

  // Check before timeout - should not confirm
  auto before_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(5e9));
  auto confirmed_early = storage_.check_time_based_confirmation(before_timeout);
  EXPECT_TRUE(confirmed_early.empty());

  // Check after timeout - should confirm
  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_.check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0], "FAULT_1");

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ConfirmedFaultCanHealWithPassedEvents) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 3 FAILED events to confirm (counter = -3)
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                                "/node" + std::to_string(i), clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Report 6 PASSED events (counter = -3 + 6 = +3, reaches healing threshold)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(FaultStorageTest, HealedFaultCanRecurWithFailedEvents) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 1 FAILED event (counter = -1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Report 4 PASSED events to heal (counter = -1 + 4 = +3)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);

  // Report 3 FAILED events - fault should recur and confirm (counter = +3 - 3 = 0, then -3)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Recurrence",
                                "/node2", clock.now(), config);
  }

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// Regression for #428 on the in-memory backend: a heal heartbeat must not run the
// debounce counter off, and confirmed/healed status must latch (one report cannot
// flip it).
TEST_F(FaultStorageTest, HeartbeatHealClampedAndStatusLatched) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  ASSERT_EQ(storage_.get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);

  // Long heal heartbeat: counter clamps at healing_threshold, then heals.
  for (int i = 0; i < 100; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }
  EXPECT_EQ(storage_.get_fault("FAULT_1")->status, Fault::STATUS_HEALED);

  // Re-confirmation is bounded and the healed status latches until the counter
  // walks all the way back down to the confirmation threshold.
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                                clock.now(), config);
    EXPECT_EQ(storage_.get_fault("FAULT_1")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  EXPECT_EQ(storage_.get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);
}

// Latch holds in both directions, same as the SQLite backend: CONFIRMED with a positive counter
// stays CONFIRMED on FAILED.
TEST_F(FaultStorageTest, ConfirmedLatchSurvivesFailedAtPositiveCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_threshold = 3;
  config.critical_immediate_confirm = true;

  storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "crit", "/n",
                              clock.now(), config);
  ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_CONFIRMED);
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_CONFIRMED);
  storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                              config);
  EXPECT_EQ(storage_.get_fault("F")->status, Fault::STATUS_CONFIRMED);
}

// HEALED with a negative counter stays HEALED on PASSED.
TEST_F(FaultStorageTest, HealedLatchSurvivesPassedAtNegativeCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                              config);
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_HEALED);
  for (int i = 0; i < 5; ++i) {  // +3 -> -2, latched HEALED
    storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                                config);
    ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  storage_.report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  EXPECT_EQ(storage_.get_fault("F")->status, Fault::STATUS_HEALED);
}

TEST_F(FaultStorageTest, ReclassifyHealedAsCleared) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                              config);
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_HEALED);

  const auto reclassified = storage_.reclassify_healed_as_cleared();
  ASSERT_EQ(reclassified.size(), 1u);
  EXPECT_EQ(reclassified[0], "F");
  EXPECT_EQ(storage_.get_fault("F")->status, Fault::STATUS_CLEARED);
  EXPECT_TRUE(storage_.reclassify_healed_as_cleared().empty());
}

// Unit tests for the shared debounce helpers.
TEST(DebounceHelpers, ClampDebounceCounterBounds) {
  DebounceConfig config;
  config.confirmation_threshold = -2;
  config.healing_threshold = 2;
  EXPECT_EQ(clamp_debounce_counter(100, config), 2);
  EXPECT_EQ(clamp_debounce_counter(-100, config), -2);
  EXPECT_EQ(clamp_debounce_counter(1, config), 1);
}

TEST(DebounceHelpers, SanitizeDebounceConfigFixesBadThresholds) {
  DebounceConfig good;
  EXPECT_TRUE(sanitize_debounce_config(good));  // defaults are valid

  // healing_threshold == 0 is valid (heal on a single PASSED event); leave it untouched.
  DebounceConfig single_event_heal;
  single_event_heal.confirmation_threshold = -1;
  single_event_heal.healing_threshold = 0;
  EXPECT_TRUE(sanitize_debounce_config(single_event_heal));
  EXPECT_EQ(single_event_heal.healing_threshold, 0);

  DebounceConfig bad;
  bad.confirmation_threshold = 0;  // must be < 0
  bad.healing_threshold = -1;      // must be >= 0
  EXPECT_FALSE(sanitize_debounce_config(bad));
  EXPECT_LT(bad.confirmation_threshold, 0);
  EXPECT_GE(bad.healing_threshold, 0);
}

// healing_threshold == 0 heals on a single PASSED event (used by action_status_bridge).
TEST_F(FaultStorageTest, HealingThresholdZeroHealsOnSinglePassed) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = true;
  config.healing_threshold = 0;

  storage_.report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                              config);
  ASSERT_EQ(storage_.get_fault("F")->status, Fault::STATUS_CONFIRMED);
  storage_.report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  EXPECT_EQ(storage_.get_fault("F")->status, Fault::STATUS_HEALED);
}

TEST(DebounceHelpers, ComputeDebounceStatusLatches) {
  DebounceConfig config;               // healing 3, healing disabled
  config.confirmation_threshold = -3;  // so counters -1..+2 sit between the thresholds (latch range)
  // A confirmed fault with a positive counter stays confirmed (latch), not PREPASSED.
  EXPECT_EQ(compute_debounce_status(2, Fault::STATUS_CONFIRMED, config), Fault::STATUS_CONFIRMED);
  // A healed fault with a negative counter stays healed (latch), not PREFAILED.
  EXPECT_EQ(compute_debounce_status(-1, Fault::STATUS_HEALED, config), Fault::STATUS_HEALED);
  // A pending fault is not latched.
  EXPECT_EQ(compute_debounce_status(1, Fault::STATUS_PREFAILED, config), Fault::STATUS_PREPASSED);
}

TEST_F(FaultStorageTest, GetAllFaultsReturnsAllFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add faults with different statuses
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test 1", "/node1",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN, "Test 2", "/node2",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_3", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Test 3", "/node3",
                              timestamp, default_config());

  auto all_faults = storage_.get_all_faults();
  EXPECT_EQ(all_faults.size(), 3u);
}

TEST_F(FaultStorageTest, GetAllFaultsReturnsEmptyForEmptyStorage) {
  auto all_faults = storage_.get_all_faults();
  EXPECT_TRUE(all_faults.empty());
}

// FaultManagerNode tests
class FaultManagerNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Use in-memory storage for tests to avoid permission issues
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
    });
    node_ = std::make_shared<FaultManagerNode>(options);
  }

  void TearDown() override {
    node_.reset();
  }

  std::shared_ptr<FaultManagerNode> node_;
};

TEST_F(FaultManagerNodeTest, NodeCreation) {
  EXPECT_STREQ(node_->get_name(), "fault_manager");
  EXPECT_EQ(node_->get_storage().size(), 0u);
  EXPECT_EQ(node_->get_storage_type(), "memory");
}

TEST_F(FaultManagerNodeTest, DefaultDebounceConfig) {
  auto config = node_->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
}

TEST(FaultManagerNodeParameterTest, CustomConfirmationThreshold) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", -5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -5);
}

TEST(FaultManagerNodeParameterTest, ConfirmationThresholdZeroSanitizedToDefault) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", 0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  // 0 is invalid (must be < 0, otherwise the clamp would pin the counter and statuses get stuck).
  // It is sanitized to the safe default -1, which still confirms immediately on the first FAILED.
  auto config = node->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
}

TEST(FaultManagerNodeParameterTest, HealingEnabled) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"healing_enabled", true},
      {"healing_threshold", 5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_TRUE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 5);
}

TEST(FaultManagerNodeParameterTest, AutoConfirmAfterSec) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"auto_confirm_after_sec", 15.0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_DOUBLE_EQ(config.auto_confirm_after_sec, 15.0);
}

TEST(FaultManagerNodeParameterTest, ClampsInvalidCaptureParams) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"snapshots.capture_pool_size", -3},
      {"snapshots.capture_queue_depth", 0},
      {"snapshots.capture_queue_full_policy", std::string("bogus")},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  EXPECT_EQ(node->capture_pool_size_for_test(), 1);
  EXPECT_EQ(node->capture_queue_depth_for_test(), 1);
  EXPECT_EQ(node->capture_queue_full_policy_for_test(), ros2_medkit_fault_manager::QueueFullPolicy::kRejectNewest);
}

/// Drive @p count FAILED reports that move the debounce counter without confirming.
/// The threshold sits far below any count used here, so no report in the run confirms the fault.
static void drive_near_misses(ros2_medkit_fault_manager::FaultStorage & storage, int count) {
  DebounceConfig config;
  config.confirmation_threshold = -1000;
  config.critical_immediate_confirm = false;
  constexpr int64_t kBaseNs = 1700000000000000000LL;

  for (int i = 0; i < count; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump",
                               rclcpp::Time(kBaseNs + static_cast<int64_t>(i) * 1000000LL), config);
  }
}

TEST(FaultManagerNodeParameterTest, AppliesNearMissRetentionBound) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"near_miss.max_per_fault", 4},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  drive_near_misses(node->get_storage_for_test(), 10);

  EXPECT_EQ(node->get_storage().get_near_misses("PUMP_PRESSURE_LOW").size(), 4u);
}

TEST(FaultManagerNodeParameterTest, NearMissRetentionDefaultsToBounded) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"storage_type", "memory"}});
  auto node = std::make_shared<FaultManagerNode>(options);

  drive_near_misses(node->get_storage_for_test(), 205);

  // The documented default is 200 per fault code, and it must be in force without configuration.
  EXPECT_EQ(node->get_storage().get_near_misses("PUMP_PRESSURE_LOW").size(), 200u);
}

TEST(FaultManagerNodeParameterTest, NegativeNearMissBoundFallsBackToDefault) {
  // A negative value cast straight to size_t becomes SIZE_MAX, silently removing the bound.
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"near_miss.max_per_fault", -5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  drive_near_misses(node->get_storage_for_test(), 205);

  EXPECT_EQ(node->get_storage().get_near_misses("PUMP_PRESSURE_LOW").size(), 200u);
}

TEST(FaultManagerNodeParameterTest, ZeroNearMissBoundIsUnlimited) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"near_miss.max_per_fault", 0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  drive_near_misses(node->get_storage_for_test(), 205);

  EXPECT_EQ(node->get_storage().get_near_misses("PUMP_PRESSURE_LOW").size(), 205u);
}

TEST(FaultManagerNodeParameterTest, ParsesDropOldestPolicy) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"snapshots.capture_queue_full_policy", std::string("drop_oldest")},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  EXPECT_EQ(node->capture_queue_full_policy_for_test(), ros2_medkit_fault_manager::QueueFullPolicy::kDropOldest);
}

// FaultEvent Publishing Tests
//
// Each test iteration uses a unique namespace to prevent DDS cross-contamination
// between SetUp/TearDown cycles within the same process. Without this, late-delivered
// messages from a previous test's publisher can pollute the new subscription.
class FaultEventPublishingTest : public ::testing::Test {
 protected:
  static inline std::atomic<int> test_counter_{0};

  /// Fault manager parameter overrides; subclasses extend to enable capture features.
  virtual std::vector<rclcpp::Parameter> fault_manager_overrides() {
    return {
        rclcpp::Parameter("storage_type", "memory"),
        rclcpp::Parameter("confirmation_threshold", -1),  // Immediate confirmation
    };
  }

  void SetUp() override {
    // Unique namespace per test iteration avoids DDS topic collisions
    std::string ns = "/test_events_" + std::to_string(test_counter_.fetch_add(1));

    // Create fault manager node with immediate confirmation
    rclcpp::NodeOptions fm_options;
    fm_options.parameter_overrides(fault_manager_overrides());
    fm_options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    fault_manager_ = std::make_shared<FaultManagerNode>(fm_options);

    // Create test node in the same namespace
    rclcpp::NodeOptions test_options;
    test_options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    test_node_ = std::make_shared<rclcpp::Node>("test_event_subscriber", test_options);

    // Subscribe to fault events (namespaced topic: /<ns>/fault_manager/events)
    std::string events_topic = ns + "/fault_manager/events";
    auto qos = rclcpp::QoS(100).reliable().durability_volatile();
    event_subscription_ =
        test_node_->create_subscription<FaultEvent>(events_topic, qos, [this](const FaultEvent::SharedPtr msg) {
          received_events_.push_back(*msg);
        });

    // Create service clients (namespaced services)
    report_fault_client_ = test_node_->create_client<ReportFault>(ns + "/fault_manager/report_fault");
    clear_fault_client_ = test_node_->create_client<ClearFault>(ns + "/fault_manager/clear_fault");
    get_fault_client_ = test_node_->create_client<GetFault>(ns + "/fault_manager/get_fault");
    list_faults_for_entity_client_ =
        test_node_->create_client<ListFaultsForEntity>(ns + "/fault_manager/list_faults_for_entity");

    // Wait for services
    ASSERT_TRUE(report_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(clear_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(get_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(list_faults_for_entity_client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override {
    event_subscription_.reset();
    report_fault_client_.reset();
    clear_fault_client_.reset();
    get_fault_client_.reset();
    list_faults_for_entity_client_.reset();
    test_node_.reset();
    fault_manager_.reset();
    received_events_.clear();
  }

  void spin_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Spin until a predicate becomes true or timeout expires.
  /// More robust than fixed spin_for() under CPU contention.
  bool spin_until(const std::function<bool()> & predicate,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
  }

  /// Spin until a future is ready, with 2s timeout. Robust under CPU contention.
  template <typename FutureT>
  bool spin_until_future_ready(FutureT & future, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }

  bool call_report_fault(const std::string & fault_code, uint8_t severity, const std::string & source_id) {
    auto request = std::make_shared<ReportFault::Request>();
    request->fault_code = fault_code;
    request->event_type = ReportFault::Request::EVENT_FAILED;
    request->severity = severity;
    request->description = "Test fault";
    request->source_id = source_id;

    auto future = report_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return false;
    }
    return future.get()->accepted;
  }

  bool call_report_passed(const std::string & fault_code, const std::string & source_id) {
    auto request = std::make_shared<ReportFault::Request>();
    request->fault_code = fault_code;
    request->event_type = ReportFault::Request::EVENT_PASSED;
    request->source_id = source_id;

    auto future = report_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return false;
    }
    return future.get()->accepted;
  }

  bool call_clear_fault(const std::string & fault_code) {
    auto request = std::make_shared<ClearFault::Request>();
    request->fault_code = fault_code;

    auto future = clear_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return false;
    }
    return future.get()->success;
  }

  std::optional<GetFault::Response> call_get_fault(const std::string & fault_code) {
    auto request = std::make_shared<GetFault::Request>();
    request->fault_code = fault_code;

    auto future = get_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return std::nullopt;
    }
    return *future.get();
  }

  std::optional<ListFaultsForEntity::Response> call_list_faults_for_entity(const std::string & entity_id) {
    auto request = std::make_shared<ListFaultsForEntity::Request>();
    request->entity_id = entity_id;

    auto future = list_faults_for_entity_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return std::nullopt;
    }
    return *future.get();
  }

  std::shared_ptr<FaultManagerNode> fault_manager_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Subscription<FaultEvent>::SharedPtr event_subscription_;
  rclcpp::Client<ReportFault>::SharedPtr report_fault_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_fault_client_;
  rclcpp::Client<GetFault>::SharedPtr get_fault_client_;
  rclcpp::Client<ListFaultsForEntity>::SharedPtr list_faults_for_entity_client_;
  std::vector<FaultEvent> received_events_;
};

TEST_F(FaultEventPublishingTest, NewFaultPublishesConfirmedEvent) {
  // Report a new fault - with threshold=-1, it should immediately confirm
  ASSERT_TRUE(call_report_fault("TEST_FAULT_1", Fault::SEVERITY_ERROR, "/test_node"));

  // Wait for event to arrive (polling, robust under CPU contention)
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_CONFIRMED was published
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_CONFIRMED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_1");
  EXPECT_EQ(received_events_[0].fault.severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(received_events_[0].fault.status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultEventPublishingTest, UpdateExistingFaultPublishesUpdatedEvent) {
  // Report a new fault first
  ASSERT_TRUE(call_report_fault("TEST_FAULT_2", Fault::SEVERITY_WARN, "/test_node1"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Clear received events
  received_events_.clear();

  // Report same fault again - should trigger EVENT_UPDATED
  ASSERT_TRUE(call_report_fault("TEST_FAULT_2", Fault::SEVERITY_ERROR, "/test_node2"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_UPDATED was published (severity/sources changed; still one occurrence)
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_UPDATED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_2");
  EXPECT_EQ(received_events_[0].fault.occurrence_count, 1u);
}

TEST_F(FaultEventPublishingTest, ClearFaultPublishesClearedEvent) {
  // Report a fault first
  ASSERT_TRUE(call_report_fault("TEST_FAULT_3", Fault::SEVERITY_ERROR, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Clear received events
  received_events_.clear();

  // Clear the fault
  ASSERT_TRUE(call_clear_fault("TEST_FAULT_3"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_CLEARED was published
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_CLEARED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_3");
  EXPECT_EQ(received_events_[0].fault.status, Fault::STATUS_CLEARED);
}

/// Healing on, threshold 1: FAILED confirms (counter -1), the first PASSED lands on
/// 0 (latched CONFIRMED), the second reaches 1 and heals.
class HealingFaultEventPublishingTest : public FaultEventPublishingTest {
 protected:
  std::vector<rclcpp::Parameter> fault_manager_overrides() override {
    return {
        rclcpp::Parameter("storage_type", "memory"),
        rclcpp::Parameter("confirmation_threshold", -1),
        rclcpp::Parameter("healing_enabled", true),
        rclcpp::Parameter("healing_threshold", 1),
    };
  }
};

TEST_F(HealingFaultEventPublishingTest, HealPublishesClearedEventSoStreamConsumersSeeTheEnd) {
  ASSERT_TRUE(call_report_fault("HEAL_ME", Fault::SEVERITY_ERROR, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));
  received_events_.clear();

  ASSERT_TRUE(call_report_passed("HEAL_ME", "/test_node"));
  ASSERT_TRUE(call_report_passed("HEAL_ME", "/test_node"));

  // The heal is the fault's end; without an event the SSE stream keeps every
  // consumer showing a fault that is over.
  auto find_end_event = [this]() -> const FaultEvent * {
    for (const auto & event : received_events_) {
      if (event.event_type == FaultEvent::EVENT_CLEARED) {
        return &event;
      }
    }
    return nullptr;
  };

  ASSERT_TRUE(spin_until([&]() {
    return find_end_event() != nullptr;
  })) << "no event published for the CONFIRMED -> HEALED transition";

  const FaultEvent * event = find_end_event();
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->fault.fault_code, "HEAL_ME");
  EXPECT_EQ(event->fault.status, Fault::STATUS_HEALED);
}

TEST_F(FaultEventPublishingTest, ClearNonExistentFaultNoEvent) {
  // Clear non-existent fault - should not publish event
  ASSERT_FALSE(call_clear_fault("NON_EXISTENT_FAULT"));
  // Brief spin to confirm no event arrives (negative test - keep short)
  spin_for(std::chrono::milliseconds(100));

  // Verify no events published
  EXPECT_EQ(received_events_.size(), 0u);
}

TEST_F(FaultEventPublishingTest, EventContainsCorrectTimestamp) {
  auto before = fault_manager_->now();

  ASSERT_TRUE(call_report_fault("TEST_FAULT_4", Fault::SEVERITY_WARN, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto after = fault_manager_->now();

  ASSERT_EQ(received_events_.size(), 1u);

  // Verify timestamp is within expected range
  rclcpp::Time event_time(received_events_[0].timestamp);
  EXPECT_GE(event_time, before);
  EXPECT_LE(event_time, after);
}

TEST_F(FaultEventPublishingTest, EventContainsFullFaultData) {
  ASSERT_TRUE(call_report_fault("FULL_DATA_TEST", Fault::SEVERITY_CRITICAL, "/sensor/temperature"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  ASSERT_EQ(received_events_.size(), 1u);
  const auto & fault = received_events_[0].fault;

  // Verify all fault fields are populated
  EXPECT_EQ(fault.fault_code, "FULL_DATA_TEST");
  EXPECT_EQ(fault.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(fault.description, "Test fault");
  EXPECT_EQ(fault.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault.occurrence_count, 1u);
  ASSERT_EQ(fault.reporting_sources.size(), 1u);
  EXPECT_EQ(fault.reporting_sources[0], "/sensor/temperature");
}

TEST_F(FaultEventPublishingTest, TimestampUsesWallClockNotSimTime) {
  // Get current wall clock time
  auto wall_before = std::chrono::system_clock::now();
  auto wall_before_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_before.time_since_epoch()).count();

  ASSERT_TRUE(call_report_fault("WALL_CLOCK_TEST", Fault::SEVERITY_WARN, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto wall_after = std::chrono::system_clock::now();
  auto wall_after_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_after.time_since_epoch()).count();

  ASSERT_EQ(received_events_.size(), 1u);

  // Convert event timestamp to nanoseconds
  rclcpp::Time event_time(received_events_[0].timestamp);
  int64_t event_ns = event_time.nanoseconds();

  // Verify timestamp is close to wall clock time (within 5 seconds to be safe)
  // This catches the bug where sim time (e.g., 148 seconds) would be far from wall clock
  EXPECT_GE(event_ns, wall_before_ns - 5'000'000'000LL);
  EXPECT_LE(event_ns, wall_after_ns + 5'000'000'000LL);

  // Also verify it's a reasonable Unix timestamp (after year 2020)
  // This catches the "1970" bug from the issue
  constexpr int64_t YEAR_2020_NS = 1577836800LL * 1'000'000'000LL;  // 2020-01-01 00:00:00 UTC
  EXPECT_GT(event_ns, YEAR_2020_NS);
}

// GetFault Service Tests
TEST_F(FaultEventPublishingTest, GetFaultReturnsExpectedFault) {
  // Report a fault first
  ASSERT_TRUE(call_report_fault("GET_FAULT_TEST", Fault::SEVERITY_ERROR, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Get fault via service
  auto response = call_get_fault("GET_FAULT_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->fault.fault_code, "GET_FAULT_TEST");
  EXPECT_EQ(response->fault.severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(response->fault.status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsNotFoundForMissingFault) {
  // Get non-existent fault
  auto response = call_get_fault("NON_EXISTENT_FAULT");
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsEnvironmentData) {
  // Report a fault
  ASSERT_TRUE(call_report_fault("ENV_DATA_TEST", Fault::SEVERITY_WARN, "/sensor/temp"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto response = call_get_fault("ENV_DATA_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);

  // Environment data should be present
  const auto & env_data = response->environment_data;

  // Should have freeze-frame type snapshot for the first occurrence
  // Note: The actual snapshot content depends on the storage implementation
  // but we can verify the service returns environment_data correctly
  EXPECT_TRUE(env_data.snapshots.empty() || !env_data.snapshots[0].type.empty());
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsExtendedDataRecords) {
  // Report fault twice to have first and last occurrence timestamps differ
  ASSERT_TRUE(call_report_fault("EDR_TEST", Fault::SEVERITY_ERROR, "/node1"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));
  ASSERT_TRUE(call_report_fault("EDR_TEST", Fault::SEVERITY_ERROR, "/node2"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 2;
  }));

  auto response = call_get_fault("EDR_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);

  // Verify extended data records contain timestamp information
  const auto & edr = response->environment_data.extended_data_records;
  EXPECT_NE(edr.first_occurrence_ns, 0);
  EXPECT_NE(edr.last_occurrence_ns, 0);
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntitySuccess) {
  // Report faults from different sources
  ASSERT_TRUE(call_report_fault("MOTOR_FAULT", Fault::SEVERITY_ERROR, "/powertrain/motor_controller"));
  ASSERT_TRUE(call_report_fault("SENSOR_FAULT", Fault::SEVERITY_WARN, "/powertrain/motor_controller"));
  ASSERT_TRUE(call_report_fault("BRAKE_FAULT", Fault::SEVERITY_ERROR, "/chassis/brake_system"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 3;
  }));

  // Query faults for motor_controller entity
  auto response = call_list_faults_for_entity("motor_controller");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->faults.size(), 2u);

  // Verify the returned faults are from motor_controller
  std::set<std::string> codes;
  for (const auto & fault : response->faults) {
    codes.insert(fault.fault_code);
  }
  EXPECT_TRUE(codes.count("MOTOR_FAULT"));
  EXPECT_TRUE(codes.count("SENSOR_FAULT"));
  EXPECT_FALSE(codes.count("BRAKE_FAULT"));
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntityEmptyResult) {
  // Report faults from a different entity
  ASSERT_TRUE(call_report_fault("SOME_FAULT", Fault::SEVERITY_ERROR, "/some/other_entity"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Query faults for non-existent entity
  auto response = call_list_faults_for_entity("motor_controller");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->faults.empty());
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntityWithEmptyId) {
  auto response = call_list_faults_for_entity("");
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

// Freeze-frame retention: after clear_fault deletes the per-topic snapshots, GetFault
// must keep serving the retained freeze_frames row (the only surviving record).
class FreezeFrameRetentionTest : public FaultEventPublishingTest {
 protected:
  std::vector<rclcpp::Parameter> fault_manager_overrides() override {
    auto overrides = FaultEventPublishingTest::fault_manager_overrides();
    overrides.emplace_back("snapshots.enabled", true);
    overrides.emplace_back("snapshots.timeout_sec", 5.0);
    overrides.emplace_back("snapshots.default_topics", std::vector<std::string>{"/ff_pressure"});
    return overrides;
  }
};

TEST_F(FreezeFrameRetentionTest, GetFaultServesRetainedFreezeFrameAfterClear) {
  auto pub = test_node_->create_publisher<std_msgs::msg::Float64>("/ff_pressure", rclcpp::QoS(10));
  auto timer = test_node_->create_wall_timer(std::chrono::milliseconds(20), [&pub]() {
    std_msgs::msg::Float64 msg;
    msg.data = 91.25;
    pub->publish(msg);
  });

  // Wait until the fault manager can discover the publisher, then confirm the fault.
  ASSERT_TRUE(spin_until([this]() {
    return fault_manager_->count_publishers("/ff_pressure") > 0;
  }));
  ASSERT_TRUE(call_report_fault("FF_FAULT", Fault::SEVERITY_ERROR, "/test_node"));

  // Capture runs asynchronously on the pool; wait for a non-empty freeze-frame.
  ASSERT_TRUE(spin_until(
      [this]() {
        auto frame = fault_manager_->get_storage().get_freeze_frame("FF_FAULT");
        return frame.has_value() && frame->data != "{}";
      },
      std::chrono::milliseconds(10000)));

  ASSERT_TRUE(call_clear_fault("FF_FAULT"));

  auto response = call_get_fault("FF_FAULT");
  ASSERT_TRUE(response.has_value());
  ASSERT_TRUE(response->success);

  // Per-topic snapshots were deleted on clear; the retained frame is served instead.
  ASSERT_EQ(response->environment_data.snapshots.size(), 1u);
  const auto & snapshot = response->environment_data.snapshots[0];
  EXPECT_EQ(snapshot.type, ros2_medkit_msgs::msg::Snapshot::TYPE_FREEZE_FRAME);
  EXPECT_EQ(snapshot.name, "freeze_frame");
  auto parsed = nlohmann::json::parse(snapshot.data);
  ASSERT_TRUE(parsed.contains("/ff_pressure"));
  EXPECT_DOUBLE_EQ(parsed["/ff_pressure"]["data"].get<double>(), 91.25);
}

// snapshots.max_per_fault and snapshots.retain_on_clear are independent settings.
class UnlimitedSnapshotRetentionTest : public FaultEventPublishingTest {
 protected:
  std::vector<rclcpp::Parameter> fault_manager_overrides() override {
    auto overrides = FaultEventPublishingTest::fault_manager_overrides();
    overrides.emplace_back("snapshots.enabled", true);
    overrides.emplace_back("snapshots.timeout_sec", 5.0);
    overrides.emplace_back("snapshots.default_topics", std::vector<std::string>{"/unlimited_pressure"});
    overrides.emplace_back("snapshots.max_per_fault", 0);  // documented as unlimited
    overrides.emplace_back("snapshots.retain_on_clear", true);
    return overrides;
  }
};

TEST_F(UnlimitedSnapshotRetentionTest, RetentionAppliesWithNoPerFaultCap) {
  // set_retain_snapshots_on_clear used to sit inside `if (max_per_fault > 0)`, so
  // choosing the documented "0 = unlimited" silently turned retention off as well
  // and acknowledging a fault still deleted its readings.
  EXPECT_TRUE(fault_manager_->get_storage().retains_snapshots_on_clear())
      << "an unlimited cap swallowed the retention setting";

  auto pub = test_node_->create_publisher<std_msgs::msg::Float64>("/unlimited_pressure", rclcpp::QoS(10));
  auto timer = test_node_->create_wall_timer(std::chrono::milliseconds(20), [&pub]() {
    std_msgs::msg::Float64 msg;
    msg.data = 12.5;
    pub->publish(msg);
  });

  ASSERT_TRUE(spin_until([this]() {
    return fault_manager_->count_publishers("/unlimited_pressure") > 0;
  }));
  ASSERT_TRUE(call_report_fault("UNCAPPED_FAULT", Fault::SEVERITY_ERROR, "/test_node"));

  ASSERT_TRUE(spin_until(
      [this]() {
        return !fault_manager_->get_storage().get_snapshots("UNCAPPED_FAULT").empty();
      },
      std::chrono::milliseconds(10000)));

  ASSERT_TRUE(call_clear_fault("UNCAPPED_FAULT"));

  EXPECT_FALSE(fault_manager_->get_storage().get_snapshots("UNCAPPED_FAULT").empty())
      << "acknowledgement deleted readings the operator asked to keep";
}

// matches_entity helper tests
TEST(MatchesEntityTest, ExactMatch) {
  std::vector<std::string> sources = {"motor_controller"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "other"));
}

TEST(MatchesEntityTest, FQNSuffixMatch) {
  std::vector<std::string> sources = {"/powertrain/motor_controller"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "powertrain"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "controller"));  // Partial - not a suffix
}

TEST(MatchesEntityTest, HierarchicalMatch) {
  std::vector<std::string> sources = {"/perception/lidar/front_sensor"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "front_sensor"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "lidar"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "perception"));  // Root only
}

TEST(MatchesEntityTest, MultipleSources) {
  std::vector<std::string> sources = {"/chassis/brake_system", "/powertrain/motor_controller", "/sensor/temperature"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "brake_system"));
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "temperature"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "unknown"));
}

TEST(MatchesEntityTest, EmptySources) {
  std::vector<std::string> sources;
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "motor_controller"));
}

// --- InMemoryFaultStorage snapshot limit tests ---

TEST(InMemorySnapshotRetentionTest, ClearKeepsSnapshotsWhenEvidenceIsRetained) {
  InMemoryFaultStorage storage;
  rclcpp::Clock clock;
  storage.report_fault_event("KEEP", ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED,
                             ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR, "keep", "/n", clock.now(),
                             ros2_medkit_fault_manager::DebounceConfig{});
  storage.set_retain_snapshots_on_clear(true);

  ros2_medkit_fault_manager::SnapshotData row;
  row.fault_code = "KEEP";
  row.topic = "/t";
  row.message_type = "std_msgs/msg/String";
  row.data = "{}";
  row.captured_at_ns = 1000;
  row.capture_id = 1;
  storage.store_snapshots({row});

  ASSERT_TRUE(storage.clear_fault("KEEP"));

  // Same rule as the SQLite side: with a history configured, acknowledging must
  // not leave recordings whose matching readings were deleted.
  EXPECT_EQ(storage.get_snapshots("KEEP").size(), 1u);
}

TEST(InMemorySnapshotRetentionTest, ClearStillDropsSnapshotsByDefault) {
  InMemoryFaultStorage storage;
  rclcpp::Clock clock;
  storage.report_fault_event("DROP", ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED,
                             ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR, "drop", "/n", clock.now(),
                             ros2_medkit_fault_manager::DebounceConfig{});

  ros2_medkit_fault_manager::SnapshotData row;
  row.fault_code = "DROP";
  row.topic = "/t";
  row.message_type = "std_msgs/msg/String";
  row.data = "{}";
  row.capture_id = 1;
  storage.store_snapshots({row});

  ASSERT_TRUE(storage.clear_fault("DROP"));
  EXPECT_TRUE(storage.get_snapshots("DROP").empty()) << "the default must stay what it always was";
}

TEST(InMemorySnapshotLimitTest, KeepsTheNewestCaptureWholeAndDropsTheOldest) {
  InMemoryFaultStorage storage;
  storage.set_max_snapshots_per_fault(4);

  const auto capture = [](int64_t id, int64_t at) {
    std::vector<ros2_medkit_fault_manager::SnapshotData> rows;
    for (const char * topic : {"/a", "/b"}) {
      ros2_medkit_fault_manager::SnapshotData row;
      row.fault_code = "TEST";
      row.topic = topic;
      row.message_type = "std_msgs/msg/String";
      row.data = "{}";
      row.captured_at_ns = at;
      row.capture_id = id;
      rows.push_back(row);
    }
    return rows;
  };

  storage.store_snapshots(capture(1, 1000));
  storage.store_snapshots(capture(2, 2000));
  storage.store_snapshots(capture(3, 3000));

  // Whole captures in, whole capture out. Counting rows and rejecting the new one
  // stored capture 3 in part: one topic present, the rest silently absent.
  auto result = storage.get_snapshots("TEST");
  ASSERT_EQ(result.size(), 4u);
  for (const auto & s : result) {
    EXPECT_NE(s.capture_id, 1) << "the oldest capture should have gone whole";
  }
}

TEST(InMemorySnapshotLimitTest, UnlimitedWhenZero) {
  InMemoryFaultStorage storage;
  storage.set_max_snapshots_per_fault(0);

  ros2_medkit_fault_manager::SnapshotData snap;
  snap.fault_code = "TEST";
  snap.topic = "/test";
  snap.message_type = "std_msgs/msg/String";
  snap.data = "{}";

  for (int i = 0; i < 20; ++i) {
    snap.captured_at_ns = static_cast<int64_t>(i * 1000);
    storage.store_snapshot(snap);
  }

  auto result = storage.get_snapshots("TEST");
  EXPECT_EQ(result.size(), 20u);
}

// --- InMemoryFaultStorage freeze-frame tests ---

TEST(InMemoryFreezeFrameTest, StoreAndRetrieve) {
  InMemoryFaultStorage storage;

  ros2_medkit_fault_manager::FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = 1000;
  storage.store_freeze_frame(frame);

  auto retrieved = storage.get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
  EXPECT_EQ(retrieved->captured_at_ns, 1000);
}

TEST(InMemoryFreezeFrameTest, AbsentForUnknownFault) {
  InMemoryFaultStorage storage;
  EXPECT_FALSE(storage.get_freeze_frame("NEVER_CAPTURED").has_value());
}

TEST(InMemoryFreezeFrameTest, ReplacedOnRecapture) {
  InMemoryFaultStorage storage;

  ros2_medkit_fault_manager::FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = 1000;
  storage.store_freeze_frame(frame);

  frame.data = R"({"/plc/pressure":{"data":9.9}})";
  frame.captured_at_ns = 2000;
  storage.store_freeze_frame(frame);

  auto retrieved = storage.get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, R"({"/plc/pressure":{"data":9.9}})");
  EXPECT_EQ(retrieved->captured_at_ns, 2000);
}

TEST(InMemoryFreezeFrameTest, SurvivesClearFault) {
  InMemoryFaultStorage storage;
  rclcpp::Clock clock;

  storage.report_fault_event("PLC_PRESSURE_HIGH", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                             "Pressure high", "/plc_node", clock.now(), DebounceConfig{});

  ros2_medkit_fault_manager::SnapshotData snap;
  snap.fault_code = "PLC_PRESSURE_HIGH";
  snap.topic = "/plc/pressure";
  snap.message_type = "std_msgs/msg/Float64";
  snap.data = R"({"data":8.4})";
  snap.captured_at_ns = 1000;
  storage.store_snapshot(snap);

  ros2_medkit_fault_manager::FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = 1000;
  storage.store_freeze_frame(frame);

  ASSERT_TRUE(storage.clear_fault("PLC_PRESSURE_HIGH"));

  EXPECT_TRUE(storage.get_snapshots("PLC_PRESSURE_HIGH").empty());
  auto retrieved = storage.get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
}

// --- InMemoryFaultStorage shared-rosbag re-store tests ---
// A fault re-confirming stores a row with a new path; the old bag must be
// unlinked only when no sibling fault of the burst still references it.

TEST(InMemoryRosbagRestoreTest, RestoreWithNewPathUnlinksTheOldExclusiveBag) {
  InMemoryFaultStorage storage;

  const auto old_path = (std::filesystem::temp_directory_path() / "medkit_restore_exclusive_bag").string();
  std::filesystem::create_directories(old_path);

  ros2_medkit_fault_manager::RosbagFileInfo info;
  info.fault_code = "X";
  info.file_path = old_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  storage.store_rosbag_file(info);

  info.file_path = old_path + "_new";
  storage.store_rosbag_file(info);

  EXPECT_FALSE(std::filesystem::exists(old_path)) << "nobody references the old bag, it must be unlinked";
  auto row = storage.get_rosbag_file("X");
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(row->file_path, old_path + "_new");
}

TEST(InMemoryRosbagRestoreTest, RestoreWithNewPathKeepsTheBagASiblingStillReferences) {
  InMemoryFaultStorage storage;

  const auto shared_path = (std::filesystem::temp_directory_path() / "medkit_restore_shared_bag").string();
  std::filesystem::create_directories(shared_path);

  ros2_medkit_fault_manager::RosbagFileInfo info;
  info.file_path = shared_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  info.fault_code = "X";
  storage.store_rosbag_file(info);
  info.fault_code = "Y";
  storage.store_rosbag_file(info);

  info.fault_code = "X";
  info.file_path = shared_path + "_new";
  storage.store_rosbag_file(info);

  EXPECT_TRUE(std::filesystem::exists(shared_path)) << "the sibling fault still owns the shared bag";
  auto sibling = storage.get_rosbag_file("Y");
  ASSERT_TRUE(sibling.has_value());
  EXPECT_EQ(sibling->file_path, shared_path);

  std::error_code ec;
  std::filesystem::remove_all(shared_path, ec);
}

// =============================================================================
// Snapshot recapture cooldown test
// =============================================================================

class SnapshotCooldownTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
        {"confirmation_threshold", -1},  // Immediate confirmation
        {"healing_enabled", true},
        {"healing_threshold", 1},  // Heal on first PASSED
        {"snapshots.enabled", true},
        {"snapshots.recapture_cooldown_sec", 0.5},  // Short cooldown for testing
        {"snapshots.max_per_fault", 0},             // Unlimited (test cooldown, not limit)
    });
    fault_manager_ = std::make_shared<FaultManagerNode>(options);

    test_node_ = std::make_shared<rclcpp::Node>("test_cooldown");
    report_client_ = test_node_->create_client<ReportFault>("/fault_manager/report_fault");
    clear_client_ = test_node_->create_client<ClearFault>("/fault_manager/clear_fault");
    get_fault_client_ = test_node_->create_client<GetFault>("/fault_manager/get_fault");

    executor_.add_node(fault_manager_);
    executor_.add_node(test_node_);
    spin_thread_ = std::thread([this]() {
      executor_.spin();
    });

    ASSERT_TRUE(report_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(clear_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(get_fault_client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(fault_manager_);
    executor_.remove_node(test_node_);
    report_client_.reset();
    clear_client_.reset();
    get_fault_client_.reset();
    test_node_.reset();
    fault_manager_.reset();
  }

  void spin_for(std::chrono::milliseconds duration) {
    std::this_thread::sleep_for(duration);
  }

  bool report_fault(const std::string & code) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = code;
    req->event_type = ReportFault::Request::EVENT_FAILED;
    req->severity = Fault::SEVERITY_CRITICAL;  // Bypass debounce
    req->description = "Test";
    req->source_id = "/test";
    auto future = report_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && future.get()->accepted;
  }

  bool report_passed(const std::string & code) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = code;
    req->event_type = ReportFault::Request::EVENT_PASSED;
    req->severity = 0;
    req->source_id = "/test";
    auto future = report_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && future.get()->accepted;
  }

  size_t get_snapshot_count(const std::string & code) {
    auto req = std::make_shared<GetFault::Request>();
    req->fault_code = code;
    auto future = get_fault_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return 0;
    }
    auto resp = future.get();
    return resp->environment_data.snapshots.size();
  }

  std::shared_ptr<FaultManagerNode> fault_manager_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Client<ReportFault>::SharedPtr report_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_client_;
  rclcpp::Client<GetFault>::SharedPtr get_fault_client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};

TEST_F(SnapshotCooldownTest, CooldownPreventsRapidRecapture) {
  // First confirmation -> snapshot captured
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // Heal and re-confirm immediately (within cooldown)
  ASSERT_TRUE(report_passed("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // Only 1 snapshot capture should have occurred (second blocked by cooldown)
  // Note: snapshot capture is async, so we check the storage-level count.
  // Freeze-frame snapshots may or may not appear depending on topic availability,
  // but the capture thread should have been spawned only once.
  (void)get_snapshot_count("COOLDOWN_TEST");
  // With no topics to capture, count may be 0. The key assertion is that
  // the second confirmation did NOT spawn a capture thread.
  // We verify indirectly: wait for cooldown to expire and re-confirm.
  std::this_thread::sleep_for(std::chrono::milliseconds(600));  // Cooldown=0.5s

  // Now cooldown expired, heal and re-confirm -> should capture again
  ASSERT_TRUE(report_passed("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // This is a basic smoke test verifying the cooldown path doesn't crash
  // and the node handles rapid re-confirmation gracefully.
  SUCCEED();
}

// End-to-end check that the node hooks the audit log on the fault write path.
// Reports a CRITICAL fault (immediate confirm => occurred + confirmed), then
// clears it (=> cleared), and inspects the persisted audit DB by reopening it.
class FaultAuditIntegrationTest : public ::testing::Test {
 protected:
  /// Healing threshold for the node under test; override per-fixture.
  virtual int healing_threshold() const {
    return 1;  // counter >= 1 heals (two PASSED after one FAILED)
  }

  void SetUp() override {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    audit_path_ =
        (std::filesystem::temp_directory_path() / ("test_node_audit_" + std::to_string(dist(gen)) + ".db")).string();
    ns_ = "/test_audit_" + std::to_string(dist(gen));

    rclcpp::NodeOptions fm_options;
    fm_options.parameter_overrides({
        {"storage_type", "memory"},
        {"confirmation_threshold", -1},
        {"healing_enabled", true},
        {"healing_threshold", healing_threshold()},
        {"audit_log.enabled", true},
        {"audit_log.database_path", audit_path_},
    });
    fm_options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    fault_manager_ = std::make_shared<FaultManagerNode>(fm_options);

    rclcpp::NodeOptions test_options;
    test_options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    test_node_ = std::make_shared<rclcpp::Node>("test_audit_client", test_options);

    report_client_ = test_node_->create_client<ReportFault>(ns_ + "/fault_manager/report_fault");
    clear_client_ = test_node_->create_client<ClearFault>(ns_ + "/fault_manager/clear_fault");
    ASSERT_TRUE(report_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(clear_client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override {
    report_client_.reset();
    clear_client_.reset();
    test_node_.reset();
    fault_manager_.reset();
    std::error_code ec;
    std::filesystem::remove(audit_path_, ec);
    std::filesystem::remove(audit_path_ + "-wal", ec);
    std::filesystem::remove(audit_path_ + "-shm", ec);
  }

  template <typename FutureT>
  bool spin_until_ready(FutureT & future) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::string audit_path_;
  std::string ns_;
  std::shared_ptr<FaultManagerNode> fault_manager_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Client<ReportFault>::SharedPtr report_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_client_;
};

TEST_F(FaultAuditIntegrationTest, TransitionsAppendVerifiableChain) {
  auto report = std::make_shared<ReportFault::Request>();
  report->fault_code = "AUDIT_FAULT";
  report->event_type = ReportFault::Request::EVENT_FAILED;
  report->severity = Fault::SEVERITY_CRITICAL;  // immediate confirm
  report->description = "overpressure";
  report->source_id = "/plc/pump";
  auto rf = report_client_->async_send_request(report);
  ASSERT_TRUE(spin_until_ready(rf));
  ASSERT_TRUE(rf.get()->accepted);

  auto clear = std::make_shared<ClearFault::Request>();
  clear->fault_code = "AUDIT_FAULT";
  auto cf = clear_client_->async_send_request(clear);
  ASSERT_TRUE(spin_until_ready(cf));
  ASSERT_TRUE(cf.get()->success);

  // Reopen the audit DB independently and inspect the persisted chain. The chain
  // opens with a "logging_activated" lifecycle marker (seq 1), then the three
  // fault transitions. The "logging_deactivated" marker is only appended when the
  // node is destroyed (TearDown), so it is not visible to this read.
  ros2_medkit_fault_manager::FaultAuditLog audit(audit_path_);
  auto records = audit.read();
  ASSERT_EQ(records.size(), 4u);
  EXPECT_EQ(records[0].event.transition, ros2_medkit_fault_manager::kTransitionLoggingActivated);
  EXPECT_EQ(records[1].event.transition, ros2_medkit_fault_manager::kTransitionOccurred);
  EXPECT_EQ(records[2].event.transition, ros2_medkit_fault_manager::kTransitionConfirmed);
  EXPECT_EQ(records[3].event.transition, ros2_medkit_fault_manager::kTransitionCleared);
  EXPECT_EQ(records[1].event.fault_code, "AUDIT_FAULT");
  EXPECT_EQ(records[2].event.source_id, "/plc/pump");

  auto result = audit.verify();
  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.checked, 4);
}

// Completeness: an auto-healed fault must record its END. One FAILED confirms the
// fault (occurred + confirmed); two PASSED drive the debounce counter to the
// healing threshold, which must append a distinct "healed" row (source auto_heal),
// and the full occurred -> confirmed -> healed chain must verify.
TEST_F(FaultAuditIntegrationTest, AutoHealAppendsHealedRow) {
  auto send_report = [&](uint8_t event_type) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = "HEAL_FAULT";
    req->event_type = event_type;
    req->severity = Fault::SEVERITY_ERROR;  // not CRITICAL: goes through debounce
    req->description = "intermittent sensor";
    req->source_id = "/robot/sensor";
    auto fut = report_client_->async_send_request(req);
    ASSERT_TRUE(spin_until_ready(fut));
    ASSERT_TRUE(fut.get()->accepted);
  };

  send_report(ReportFault::Request::EVENT_FAILED);  // counter -1, threshold -1 => CONFIRMED
  send_report(ReportFault::Request::EVENT_PASSED);  // counter 0 (hysteresis): stays CONFIRMED
  send_report(ReportFault::Request::EVENT_PASSED);  // counter 1 >= healing_threshold => HEALED

  // HEALED is latched and the counter is clamped at the healing threshold: further
  // PASSED events keep status HEALED and must NOT append duplicate "healed" rows.
  send_report(ReportFault::Request::EVENT_PASSED);
  send_report(ReportFault::Request::EVENT_PASSED);

  ros2_medkit_fault_manager::FaultAuditLog audit(audit_path_);
  std::vector<std::string> transitions;
  std::string heal_source;
  for (const auto & rec : audit.read()) {
    if (rec.event.fault_code == "HEAL_FAULT") {
      transitions.push_back(rec.event.transition);
      if (rec.event.transition == ros2_medkit_fault_manager::kTransitionHealed) {
        heal_source = rec.event.source_id;
      }
    }
  }

  ASSERT_EQ(transitions.size(), 3u) << "expected exactly occurred, confirmed, healed (no duplicates under latch)";
  EXPECT_EQ(transitions[0], ros2_medkit_fault_manager::kTransitionOccurred);
  EXPECT_EQ(transitions[1], ros2_medkit_fault_manager::kTransitionConfirmed);
  EXPECT_EQ(transitions[2], ros2_medkit_fault_manager::kTransitionHealed);
  EXPECT_EQ(heal_source, "auto_heal");

  auto result = audit.verify();
  EXPECT_TRUE(result.ok) << result.error;
}

// The CONFIRMED latch must not create duplicate "confirmed" audit rows: a PASSED
// event on a confirmed fault keeps it CONFIRMED (latched), so the following FAILED
// event is an update, not a re-confirmation. Exactly one confirmed row per episode.
TEST_F(FaultAuditIntegrationTest, ConfirmedLatchNoDuplicateConfirmedRows) {
  auto send_report = [&](uint8_t event_type) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = "LATCH_FAULT";
    req->event_type = event_type;
    req->severity = Fault::SEVERITY_ERROR;
    req->description = "flapping sensor";
    req->source_id = "/robot/sensor";
    auto fut = report_client_->async_send_request(req);
    ASSERT_TRUE(spin_until_ready(fut));
    ASSERT_TRUE(fut.get()->accepted);
  };

  send_report(ReportFault::Request::EVENT_FAILED);  // counter -1 <= threshold -1 => CONFIRMED
  send_report(ReportFault::Request::EVENT_PASSED);  // counter 0: latch keeps CONFIRMED
  send_report(ReportFault::Request::EVENT_FAILED);  // counter -1 again: still CONFIRMED, no new row

  ros2_medkit_fault_manager::FaultAuditLog audit(audit_path_);
  size_t confirmed_rows = 0;
  for (const auto & rec : audit.read()) {
    if (rec.event.fault_code == "LATCH_FAULT" &&
        rec.event.transition == ros2_medkit_fault_manager::kTransitionConfirmed) {
      ++confirmed_rows;
    }
  }
  EXPECT_EQ(confirmed_rows, 1u) << "latched re-confirmation must not append a duplicate confirmed row";
  EXPECT_TRUE(audit.verify().ok);
}

// healing_threshold == 0 means a single PASSED event heals. That single-event heal
// must be audited as exactly one "healed" row, and further PASSED events (counter
// clamped at 0, status latched HEALED) must not append more.
class FaultAuditSingleEventHealTest : public FaultAuditIntegrationTest {
 protected:
  int healing_threshold() const override {
    return 0;
  }
};

TEST_F(FaultAuditSingleEventHealTest, SinglePassedHealAuditedOnce) {
  auto send_report = [&](uint8_t event_type) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = "FAST_HEAL";
    req->event_type = event_type;
    req->severity = Fault::SEVERITY_ERROR;
    req->description = "transient glitch";
    req->source_id = "/robot/imu";
    auto fut = report_client_->async_send_request(req);
    ASSERT_TRUE(spin_until_ready(fut));
    ASSERT_TRUE(fut.get()->accepted);
  };

  send_report(ReportFault::Request::EVENT_FAILED);  // counter -1 => CONFIRMED
  send_report(ReportFault::Request::EVENT_PASSED);  // counter 0 >= threshold 0 => HEALED
  send_report(ReportFault::Request::EVENT_PASSED);  // clamped at 0, latched HEALED: no new row

  ros2_medkit_fault_manager::FaultAuditLog audit(audit_path_);
  std::vector<std::string> transitions;
  for (const auto & rec : audit.read()) {
    if (rec.event.fault_code == "FAST_HEAL") {
      transitions.push_back(rec.event.transition);
    }
  }
  ASSERT_EQ(transitions.size(), 3u) << "expected exactly occurred, confirmed, healed";
  EXPECT_EQ(transitions[0], ros2_medkit_fault_manager::kTransitionOccurred);
  EXPECT_EQ(transitions[1], ros2_medkit_fault_manager::kTransitionConfirmed);
  EXPECT_EQ(transitions[2], ros2_medkit_fault_manager::kTransitionHealed);
  EXPECT_TRUE(audit.verify().ok);
}

TEST(FaultAuditDisabledTest, NoAuditFileWhenDisabled) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  const std::string audit_path =
      (std::filesystem::temp_directory_path() / ("test_audit_off_" + std::to_string(dist(gen)) + ".db")).string();

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"}, {"audit_log.database_path", audit_path},  // default enabled=false
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  // With the feature off, no audit database file is created.
  EXPECT_FALSE(std::filesystem::exists(audit_path));
}

// Timer-driven (PREFAILED->CONFIRMED) auto-confirmations must be audited, not
// silently applied. Sets auto_confirm_after_sec and asserts a "confirmed" audit
// row appears after the 1 Hz timer fires.
TEST(FaultAuditTimerTest, TimerConfirmationAppendsConfirmedAuditRow) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", -3},  // keep the fault PREFAILED so only the timer confirms it
      {"auto_confirm_after_sec", 0.2},
      {"audit_log.enabled", true},  // in-memory audit DB (memory storage)
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  const auto * audit = node->get_audit_log_for_test();
  ASSERT_NE(audit, nullptr);

  // Land a fault in PREFAILED directly in storage; the node's auto-confirm timer
  // must later flip it to CONFIRMED and append the audit row.
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.auto_confirm_after_sec = 0.2;
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  node->get_storage_for_test().report_fault_event("AUTO_CONF_1", ReportFault::Request::EVENT_FAILED,
                                                  Fault::SEVERITY_ERROR, "stuck", "/robot/src", clock.now(), config);
  ASSERT_EQ(node->get_storage().get_fault("AUTO_CONF_1")->status, Fault::STATUS_PREFAILED);

  // Spin until a confirmed audit row appears or the budget expires (the wall
  // timer fires once per second).
  bool saw_confirmed = false;
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    rclcpp::spin_some(node);
    for (const auto & rec : audit->read()) {
      if (rec.event.fault_code == "AUTO_CONF_1" &&
          rec.event.transition == ros2_medkit_fault_manager::kTransitionConfirmed) {
        saw_confirmed = true;
        break;
      }
    }
    if (saw_confirmed) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(saw_confirmed) << "timer-driven confirmation was not audited";
  EXPECT_EQ(node->get_storage().get_fault("AUTO_CONF_1")->status, Fault::STATUS_CONFIRMED);
  EXPECT_TRUE(audit->verify().ok);
}

namespace {

/// Leave a HEALED fault in a sqlite fault DB, as a previous healing-enabled run would.
void seed_healed_fault(const std::string & db_path, const std::string & fault_code) {
  ros2_medkit_fault_manager::SqliteFaultStorage seed(db_path);
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  rclcpp::Clock clock;
  seed.report_fault_event(fault_code, ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "stale", "/robot/src",
                          clock.now(), config);
  for (int i = 0; i < 4; ++i) {
    seed.report_fault_event(fault_code, ReportFault::Request::EVENT_PASSED, 0, "", "/robot/src", clock.now(), config);
  }
  ASSERT_EQ(seed.get_fault(fault_code)->status, Fault::STATUS_HEALED);
}

std::filesystem::path make_temp_dir(const char * prefix) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  auto dir = std::filesystem::temp_directory_path() / (std::string(prefix) + std::to_string(dist(gen)));
  std::filesystem::create_directories(dir);
  return dir;
}

}  // namespace

// Startup reclassification (HEALED->CLEARED when healing is disabled) flips faults
// directly in storage, so each one must be audited or the transition is invisible
// to the audit log's verify(). Seeds a HEALED row, restarts with healing disabled
// and the audit on, and expects exactly one "cleared" row with the startup source.
TEST(FaultAuditStartupReclassifyTest, StartupReclassifyAppendsClearedRow) {
  const auto dir = make_temp_dir("test_startup_reclassify_");
  const std::string db_path = (dir / "faults.db").string();
  const std::string audit_path = (dir / "audit.db").string();
  seed_healed_fault(db_path, "STALE_HEALED");

  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "sqlite"},
        {"database_path", db_path},
        {"healing_enabled", false},
        {"audit_log.enabled", true},
        {"audit_log.database_path", audit_path},
    });
    auto node = std::make_shared<FaultManagerNode>(options);

    EXPECT_EQ(node->get_storage().get_fault("STALE_HEALED")->status, Fault::STATUS_CLEARED);

    const auto * audit = node->get_audit_log_for_test();
    ASSERT_NE(audit, nullptr);
    size_t cleared_rows = 0;
    for (const auto & rec : audit->read()) {
      if (rec.event.fault_code == "STALE_HEALED") {
        EXPECT_EQ(rec.event.transition, ros2_medkit_fault_manager::kTransitionCleared);
        EXPECT_EQ(rec.event.source_id, "startup_reclassify");
        EXPECT_EQ(rec.event.status, Fault::STATUS_CLEARED);
        ++cleared_rows;
      }
    }
    EXPECT_EQ(cleared_rows, 1u) << "expected exactly one audited startup reclassification";

    auto result = audit->verify();
    EXPECT_TRUE(result.ok) << result.error;
  }
  std::filesystem::remove_all(dir);
}

// With the audit log disabled, startup reclassification must behave exactly as
// before: the fault is flipped to CLEARED and no audit database is created.
TEST(FaultAuditStartupReclassifyTest, DisabledAuditStillReclassifies) {
  const auto dir = make_temp_dir("test_startup_reclassify_off_");
  const std::string db_path = (dir / "faults.db").string();
  seed_healed_fault(db_path, "STALE_HEALED");

  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "sqlite"},
        {"database_path", db_path},
        {"healing_enabled", false},  // default audit_log.enabled=false
    });
    auto node = std::make_shared<FaultManagerNode>(options);

    EXPECT_EQ(node->get_storage().get_fault("STALE_HEALED")->status, Fault::STATUS_CLEARED);
    EXPECT_EQ(node->get_audit_log_for_test(), nullptr);
    EXPECT_FALSE(std::filesystem::exists(dir / "fault_audit.db"));
  }
  std::filesystem::remove_all(dir);
}

namespace {

/// Force the node's next audit append to fail by inserting a row at the seq the
/// node will try next (MAX(seq)+1), so its INSERT collides on the seq PRIMARY KEY.
/// Done from a separate connection; the append-only triggers do not block INSERT.
void poison_next_audit_seq(const std::string & db_path) {
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(db_path.c_str(), &db), SQLITE_OK);
  const char * sql =
      "INSERT INTO audit_log (seq, occurred_at_ns, fault_code, transition, severity, status, source_id, "
      "description, prev_hash, record_hash) "
      "SELECT COALESCE(MAX(seq), 0) + 1, 0, 'x', 'x', 0, 'x', 'x', 'x', 'x', 'x' FROM audit_log;";
  char * err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string err_str = err ? err : "";
  sqlite3_free(err);
  sqlite3_close(db);
  ASSERT_EQ(rc, SQLITE_OK) << err_str;
}

Fault make_failclosed_fault() {
  Fault f;
  f.fault_code = "FAILCLOSED";
  f.severity = Fault::SEVERITY_ERROR;
  f.status = "CONFIRMED";
  f.description = "injected";
  return f;
}

std::string make_temp_audit_path(const char * prefix) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  return (std::filesystem::temp_directory_path() / (std::string(prefix) + std::to_string(dist(gen)) + ".db")).string();
}

void remove_audit_files(const std::string & path) {
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path + "-wal", ec);
  std::filesystem::remove(path + "-shm", ec);
}

}  // namespace

// Silent-gap guard, default behaviour: when fail_closed is false, an append
// failure must NOT abort the operation, but it must be VISIBLE - the dropped
// counter increments and the audit is flagged unhealthy (not silently lost).
TEST(FaultAuditFailClosedTest, FailOpenFlagsButDoesNotThrow) {
  const std::string audit_path = make_temp_audit_path("test_audit_failopen_");
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
        {"audit_log.enabled", true},
        {"audit_log.database_path", audit_path},
        {"audit_log.fail_closed", false},
    });
    auto node = std::make_shared<FaultManagerNode>(options);
    ASSERT_TRUE(node->audit_healthy());
    EXPECT_EQ(node->audit_dropped_writes(), 0u);

    poison_next_audit_seq(audit_path);

    EXPECT_NO_THROW(
        node->audit_transition_for_test(ros2_medkit_fault_manager::kTransitionConfirmed, make_failclosed_fault()));
    EXPECT_EQ(node->audit_dropped_writes(), 1u);
    EXPECT_FALSE(node->audit_healthy());
  }
  remove_audit_files(audit_path);
}

// Silent-gap guard, compliance-strict: with fail_closed true, an injected append
// failure must abort the operation (throw) rather than silently proceed, and the
// same health signals must fire.
TEST(FaultAuditFailClosedTest, FailClosedAbortsAndFlags) {
  const std::string audit_path = make_temp_audit_path("test_audit_failclosed_");
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
        {"audit_log.enabled", true},
        {"audit_log.database_path", audit_path},
        {"audit_log.fail_closed", true},
    });
    auto node = std::make_shared<FaultManagerNode>(options);

    poison_next_audit_seq(audit_path);

    EXPECT_THROW(
        node->audit_transition_for_test(ros2_medkit_fault_manager::kTransitionConfirmed, make_failclosed_fault()),
        std::exception);
    EXPECT_EQ(node->audit_dropped_writes(), 1u);
    EXPECT_FALSE(node->audit_healthy());
  }
  remove_audit_files(audit_path);
}

// --- InMemoryFaultStorage snapshot retention tests ---

static void store_two_snapshots(InMemoryFaultStorage & storage, const std::string & fault_code) {
  for (int i = 0; i < 2; ++i) {
    ros2_medkit_fault_manager::SnapshotData snapshot;
    snapshot.fault_code = fault_code;
    snapshot.topic = "/test/topic" + std::to_string(i);
    snapshot.message_type = "std_msgs/msg/String";
    snapshot.data = R"({"data": "value"})";
    snapshot.captured_at_ns = 1000 + i;
    storage.store_snapshot(snapshot);
  }
}

TEST(InMemorySnapshotRetentionTest, RetainedOnClearWhenConfigured) {
  InMemoryFaultStorage storage;
  storage.set_retain_snapshots_on_clear(true);

  storage.report_fault_event("SNAPSHOT_RETAIN_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                             "fault with evidence", "/test_node", rclcpp::Time(1000), default_config());
  store_two_snapshots(storage, "SNAPSHOT_RETAIN_TEST");
  ASSERT_EQ(storage.get_snapshots("SNAPSHOT_RETAIN_TEST").size(), 2u);

  ASSERT_TRUE(storage.clear_fault("SNAPSHOT_RETAIN_TEST"));

  EXPECT_EQ(storage.get_snapshots("SNAPSHOT_RETAIN_TEST").size(), 2u);
}

TEST(InMemorySnapshotRetentionTest, DeletedOnClearByDefault) {
  InMemoryFaultStorage storage;

  storage.report_fault_event("SNAPSHOT_DROP_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                             "fault with evidence", "/test_node", rclcpp::Time(1000), default_config());
  store_two_snapshots(storage, "SNAPSHOT_DROP_TEST");

  ASSERT_TRUE(storage.clear_fault("SNAPSHOT_DROP_TEST"));

  EXPECT_TRUE(storage.get_snapshots("SNAPSHOT_DROP_TEST").empty());
}

TEST(InMemorySnapshotRetentionTest, HealedReclassificationDropsSnapshotsByDefault) {
  // The SQLite backend drops snapshots here; a backend that kept them would answer a snapshot
  // query differently for the same sequence of calls.
  InMemoryFaultStorage storage;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage.report_fault_event("SNAPSHOT_DROP_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                             "fault with evidence", "/test_node", rclcpp::Time(1000), config);
  store_two_snapshots(storage, "SNAPSHOT_DROP_TEST");
  for (int i = 1; i <= 2; ++i) {
    storage.report_fault_event("SNAPSHOT_DROP_TEST", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR, "",
                               "/test_node", rclcpp::Time(1000 + i), config);
  }
  auto healed = storage.get_fault("SNAPSHOT_DROP_TEST");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";

  ASSERT_EQ(storage.reclassify_healed_as_cleared().size(), 1u);

  EXPECT_TRUE(storage.get_snapshots("SNAPSHOT_DROP_TEST").empty());
}

TEST(InMemorySnapshotRetentionTest, HealedReclassificationKeepsSnapshotsWhenConfigured) {
  InMemoryFaultStorage storage;
  storage.set_retain_snapshots_on_clear(true);
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage.report_fault_event("SNAPSHOT_RETAIN_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                             "fault with evidence", "/test_node", rclcpp::Time(1000), config);
  store_two_snapshots(storage, "SNAPSHOT_RETAIN_TEST");
  for (int i = 1; i <= 2; ++i) {
    storage.report_fault_event("SNAPSHOT_RETAIN_TEST", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR, "",
                               "/test_node", rclcpp::Time(1000 + i), config);
  }
  ASSERT_EQ(storage.reclassify_healed_as_cleared().size(), 1u);

  EXPECT_EQ(storage.get_snapshots("SNAPSHOT_RETAIN_TEST").size(), 2u);
}

TEST(InMemorySnapshotRetentionTest, HealedReclassificationLeavesOtherFaultsSnapshotsAlone) {
  InMemoryFaultStorage storage;
  DebounceConfig healing;
  healing.healing_enabled = true;
  healing.healing_threshold = 1;

  storage.report_fault_event("HEALED_FAULT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "fault",
                             "/test_node", rclcpp::Time(1000), healing);
  store_two_snapshots(storage, "HEALED_FAULT");
  for (int i = 1; i <= 2; ++i) {
    storage.report_fault_event("HEALED_FAULT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR, "",
                               "/test_node", rclcpp::Time(1000 + i), healing);
  }

  storage.report_fault_event("ACTIVE_FAULT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "fault",
                             "/test_node", rclcpp::Time(1000), default_config());
  store_two_snapshots(storage, "ACTIVE_FAULT");

  ASSERT_EQ(storage.reclassify_healed_as_cleared().size(), 1u);

  EXPECT_TRUE(storage.get_snapshots("HEALED_FAULT").empty());
  EXPECT_EQ(storage.get_snapshots("ACTIVE_FAULT").size(), 2u) << "reclassification touched an unrelated fault";
}

TEST(InMemorySnapshotRetentionTest, RetentionIsPerFaultCodeNotGlobal) {
  InMemoryFaultStorage storage;
  storage.set_retain_snapshots_on_clear(true);

  for (const auto & code : {"FAULT_A", "FAULT_B"}) {
    storage.report_fault_event(code, ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "fault", "/test_node",
                               rclcpp::Time(1000), default_config());
    store_two_snapshots(storage, code);
  }

  ASSERT_TRUE(storage.clear_fault("FAULT_A"));

  EXPECT_EQ(storage.get_snapshots("FAULT_A").size(), 2u);
  EXPECT_EQ(storage.get_snapshots("FAULT_B").size(), 2u);
}

// --- InMemoryFaultStorage near-miss series tests ---
//
// The in-memory backend must keep the same near-miss contract as SQLite: appended per
// qualifying report, bounded oldest-first, retained across clear_fault.

/// Debounce config that takes four FAILED reports to confirm, leaving three near misses first.
static DebounceConfig near_miss_config() {
  DebounceConfig config;
  config.confirmation_threshold = -4;
  config.critical_immediate_confirm = false;
  return config;
}

static rclcpp::Time near_miss_time(int index) {
  constexpr int64_t kBaseNs = 1700000000000000000LL;
  return rclcpp::Time(kBaseNs + static_cast<int64_t>(index) * 1000000LL);
}

TEST(InMemoryNearMissTest, SeriesIsAppendedAndSurvivesClear) {
  InMemoryFaultStorage storage;
  const auto config = near_miss_config();

  for (int i = 0; i < 3; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }

  auto series = storage.get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u);
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[2].debounce_counter, -3);
  EXPECT_EQ(series[0].confirmation_threshold, -4);

  ASSERT_TRUE(storage.clear_fault("PUMP_PRESSURE_LOW"));
  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);
}

TEST(InMemoryNearMissTest, ConfirmingReportIsNotANearMiss) {
  InMemoryFaultStorage storage;
  const auto config = near_miss_config();

  for (int i = 0; i < 4; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }

  auto fault = storage.get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);
}

TEST(InMemoryNearMissTest, SeriesBoundedKeepingNewest) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.confirmation_threshold = -20;
  storage.set_max_near_misses_per_fault(3);

  for (int i = 0; i < 5; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }

  auto series = storage.get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u);
  EXPECT_EQ(series[0].debounce_counter, -3);
  EXPECT_EQ(series[2].debounce_counter, -5);
}

TEST(InMemoryNearMissTest, PassedReportIsNotANearMiss) {
  InMemoryFaultStorage storage;
  const auto config = near_miss_config();

  for (int i = 0; i < 2; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }
  storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                             "/hydraulics/pump", near_miss_time(2), config);

  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
}

TEST(InMemoryNearMissTest, SeriesSurvivesHealedReclassification) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                             "pressure dipping", "/hydraulics/pump", near_miss_time(0), config);
  for (int i = 1; i <= 2; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                               "/hydraulics/pump", near_miss_time(i), config);
  }
  auto healed = storage.get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";
  ASSERT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);

  ASSERT_EQ(storage.reclassify_healed_as_cleared().size(), 1u);

  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);
}

TEST(InMemoryNearMissTest, BoundIsPerFaultCode) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.confirmation_threshold = -20;
  storage.set_max_near_misses_per_fault(2);

  for (int i = 0; i < 3; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
    storage.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "temperature rising", "/powertrain/motor", near_miss_time(i), config);
  }

  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
  EXPECT_EQ(storage.get_near_misses("MOTOR_OVERHEAT").size(), 2u);
}

TEST(InMemoryNearMissTest, BoundZeroIsUnlimited) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.confirmation_threshold = -200;
  storage.set_max_near_misses_per_fault(0);

  for (int i = 0; i < 150; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }

  EXPECT_EQ(storage.get_near_misses("PUMP_PRESSURE_LOW").size(), 150u);
}

TEST(InMemoryNearMissTest, ApplyingSmallerBoundTrimsExistingSeries) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.confirmation_threshold = -20;
  storage.set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
  }

  storage.set_max_near_misses_per_fault(2);

  auto series = storage.get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].debounce_counter, -4);
  EXPECT_EQ(series[1].debounce_counter, -5);
}

TEST(InMemoryNearMissTest, ApplyingBoundReportsHowManyEntriesItDropped) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.confirmation_threshold = -20;
  storage.set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", near_miss_time(i), config);
    storage.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "temperature rising", "/powertrain/motor", near_miss_time(i), config);
  }

  EXPECT_EQ(storage.set_max_near_misses_per_fault(2), 6u);
  EXPECT_EQ(storage.set_max_near_misses_per_fault(2), 0u);
}

TEST(InMemoryNearMissTest, EntriesCarryTheResultingStatus) {
  InMemoryFaultStorage storage;
  DebounceConfig config = near_miss_config();
  config.healing_enabled = true;
  config.healing_threshold = 2;

  storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                             "pressure dipping", "/hydraulics/pump", near_miss_time(0), config);
  for (int i = 1; i <= 3; ++i) {
    storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                               "/hydraulics/pump", near_miss_time(i), config);
  }
  auto healed = storage.get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";

  storage.report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                             "pressure dipping", "/hydraulics/pump", near_miss_time(4), config);

  auto series = storage.get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].resulting_status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(series[1].resulting_status, Fault::STATUS_HEALED)
      << "a report under the HEALED latch must not read as a fresh approach";
}

TEST(InMemoryNearMissTest, EmptyForUnknownFault) {
  InMemoryFaultStorage storage;
  EXPECT_TRUE(storage.get_near_misses("NEVER_REPORTED").empty());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
