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

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::RosbagFileInfo;
using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

/// Default debounce config for tests (matches DebounceConfig defaults: threshold=-1, no healing)
static DebounceConfig default_config() {
  return DebounceConfig{};
}

class SqliteFaultStorageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a unique temp file for each test using random_device for better entropy
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    temp_db_path_ = std::filesystem::temp_directory_path() / ("test_faults_" + std::to_string(dist(gen)) + ".db");
    storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());
  }

  void TearDown() override {
    storage_.reset();
    // Clean up temp file
    std::filesystem::remove(temp_db_path_);
    // Also remove WAL and SHM files if they exist
    std::filesystem::remove(temp_db_path_.string() + "-wal");
    std::filesystem::remove(temp_db_path_.string() + "-shm");
  }

  std::filesystem::path temp_db_path_;
  std::unique_ptr<SqliteFaultStorage> storage_;
};

TEST_F(SqliteFaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED,
                                             Fault::SEVERITY_ERROR, "Motor temperature exceeded threshold",
                                             "/powertrain/motor", timestamp, default_config());

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_->size(), 1u);
  EXPECT_TRUE(storage_->contains("MOTOR_OVERHEAT"));
}

TEST_F(SqliteFaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                             "Test", "/node1", timestamp, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 0u);
}

TEST_F(SqliteFaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Initial report", "/powertrain/motor1", timestamp1, default_config());

  bool is_new =
      storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                   "Second report", "/powertrain/motor2", timestamp2, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 1u);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  // Still the same continuous occurrence (not CLEARED in between): occurrence_count
  // does not bump on every report, only severity/sources/description update.
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(SqliteFaultStorageTest, ContinuouslyActiveFaultDoesNotInflateOccurrenceCount) {
  rclcpp::Clock clock;

  // Simulate a level-triggered poller re-reporting the same still-true condition
  // every cycle (issue #11): occurrence_count must stay at 1, not grow per poll.
  for (int i = 0; i < 25; ++i) {
    storage_->report_fault_event("TANK_OVERFILL", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                 "level = 95 > 80", "/tank", clock.now(), default_config());
  }

  auto fault = storage_->get_fault("TANK_OVERFILL");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ListFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());

  // Default query should return the CONFIRMED fault
  auto faults = storage_->list_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ListFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, config);

  // Query with PREFAILED status
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(SqliteFaultStorageTest, ListFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_->report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                               timestamp, default_config());
  storage_->report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                               "/node1", timestamp, default_config());

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_->list_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(SqliteFaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                               "/node1", timestamp, default_config());

  bool cleared = storage_->clear_fault("MOTOR_OVERHEAT");
  EXPECT_TRUE(cleared);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteFaultStorageTest, ClearNonExistentFault) {
  bool cleared = storage_->clear_fault("NON_EXISTENT");
  EXPECT_FALSE(cleared);
}

TEST_F(SqliteFaultStorageTest, PassedEventDoesNotAdvanceLastOccurred) {
  // Same contract as the in-memory backend: a PASSED event ends a fault, it does
  // not re-date it. Guards against a stale CONFIRMED fault reading as freshly
  // active in /faults and in the SSE payload.
  const rclcpp::Time failed_at(1000, 0, RCL_SYSTEM_TIME);
  const rclcpp::Time passed_at(9000, 0, RCL_SYSTEM_TIME);

  storage_->report_fault_event("FAULT_LO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               failed_at, default_config());
  storage_->report_fault_event("FAULT_LO", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", passed_at,
                               default_config());

  auto fault = storage_->get_fault("FAULT_LO");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // healing disabled: latched, by design
  EXPECT_EQ(rclcpp::Time(fault->last_occurred).nanoseconds(), failed_at.nanoseconds());
  // The PASSED instant is not lost: it rides on the wire as last_passed.
  EXPECT_EQ(rclcpp::Time(fault->last_passed).nanoseconds(), passed_at.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, ReopenRepairsLastOccurredInflatedByOldPassedBug) {
  // Databases written by releases that advanced last_occurred_ns on PASSED
  // events hold inflated rows, and a latched CONFIRMED fault that never fails
  // again would keep the wrong timestamp forever. Opening the storage must
  // repair them from last_failed_ns.
  const rclcpp::Time failed_at(1000, 0, RCL_SYSTEM_TIME);
  storage_->report_fault_event("FAULT_MIG", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               failed_at, default_config());
  storage_.reset();

  // Simulate the old bug: a PASSED event at t=9000s re-dated the row.
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(temp_db_path_.c_str(), &db), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(db, "UPDATE faults SET last_occurred_ns = 9000000000000 WHERE fault_code = 'FAULT_MIG'",
                         nullptr, nullptr, nullptr),
            SQLITE_OK);
  sqlite3_close(db);

  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  auto fault = storage_->get_fault("FAULT_MIG");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(rclcpp::Time(fault->last_occurred).nanoseconds(), failed_at.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, GetClearedFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());
  storage_->clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteFaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_->list_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

// SQLite-specific persistence test
TEST_F(SqliteFaultStorageTest, PersistenceAcrossRestarts) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  // Report some faults
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Persistent fault 1", "/node1", timestamp, default_config());
  storage_->report_fault_event("FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Persistent fault 2", "/node2", timestamp, default_config());
  storage_->clear_fault("FAULT_2");

  // Verify initial state
  EXPECT_EQ(storage_->size(), 2u);

  // Close the storage
  storage_.reset();

  // Reopen the storage
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  // Verify faults persisted
  EXPECT_EQ(storage_->size(), 2u);
  EXPECT_TRUE(storage_->contains("FAULT_1"));
  EXPECT_TRUE(storage_->contains("FAULT_2"));

  auto fault1 = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault1.has_value());
  EXPECT_EQ(fault1->severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(fault1->status, Fault::STATUS_CONFIRMED);  // Immediately confirmed with threshold=-1
  EXPECT_EQ(fault1->description, "Persistent fault 1");

  auto fault2 = storage_->get_fault("FAULT_2");
  ASSERT_TRUE(fault2.has_value());
  EXPECT_EQ(fault2->status, Fault::STATUS_CLEARED);
}

// Test timestamp precision
TEST_F(SqliteFaultStorageTest, TimestampPrecision) {
  // Create a timestamp with nanosecond precision
  int64_t test_ns = 1735312456123456789LL;  // Specific nanosecond timestamp
  rclcpp::Time timestamp(test_ns, RCL_SYSTEM_TIME);

  storage_->report_fault_event("FAULT_TS", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Timestamp test",
                               "/node1", timestamp, default_config());

  auto fault = storage_->get_fault("FAULT_TS");
  ASSERT_TRUE(fault.has_value());

  // Convert builtin_interfaces::msg::Time back to rclcpp::Time for comparison
  rclcpp::Time first_ts(fault->first_occurred);
  rclcpp::Time last_ts(fault->last_occurred);

  // Verify nanosecond precision is preserved
  EXPECT_EQ(first_ts.nanoseconds(), test_ns);
  EXPECT_EQ(last_ts.nanoseconds(), test_ns);
}

// Test in-memory SQLite database
TEST(SqliteInMemoryTest, InMemoryDatabase) {
  SqliteFaultStorage storage(":memory:");
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage.report_fault_event("MEM_FAULT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN, "In-memory test",
                             "/test", timestamp, default_config());

  EXPECT_EQ(storage.size(), 1u);
  EXPECT_TRUE(storage.contains("MEM_FAULT"));
}

// Test reporting sources JSON handling
TEST_F(SqliteFaultStorageTest, ReportingSourcesJsonHandling) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add multiple sources for the same fault
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/node/path/with/slashes", timestamp, default_config());
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/another/node", timestamp, default_config());
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/special\"chars", timestamp, default_config());

  auto fault = storage_->get_fault("MULTI_SRC");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->reporting_sources.size(), 3u);

  // Verify all sources are present (order may vary due to set)
  std::set<std::string> sources(fault->reporting_sources.begin(), fault->reporting_sources.end());
  EXPECT_TRUE(sources.count("/node/path/with/slashes") > 0);
  EXPECT_TRUE(sources.count("/another/node") > 0);
  EXPECT_TRUE(sources.count("/special\"chars") > 0);
}

// Test database path accessor
TEST_F(SqliteFaultStorageTest, DbPathAccessor) {
  EXPECT_EQ(storage_->db_path(), temp_db_path_.string());
}

// Debounce config tests for SQLite storage
TEST_F(SqliteFaultStorageTest, DefaultDebounceConfig) {
  auto config = storage_->get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
  EXPECT_FALSE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 3);
  EXPECT_TRUE(config.critical_immediate_confirm);
}

TEST_F(SqliteFaultStorageTest, SetDebounceConfig) {
  DebounceConfig config;
  config.confirmation_threshold = -5;
  config.healing_enabled = true;
  config.healing_threshold = 5;
  config.critical_immediate_confirm = false;

  storage_->set_debounce_config(config);
  auto retrieved = storage_->get_debounce_config();

  EXPECT_EQ(retrieved.confirmation_threshold, -5);
  EXPECT_TRUE(retrieved.healing_enabled);
  EXPECT_EQ(retrieved.healing_threshold, 5);
  EXPECT_FALSE(retrieved.critical_immediate_confirm);
}

TEST_F(SqliteFaultStorageTest, FaultStaysPrefailedAboveThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (2 FAILED events should stay PREFAILED)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Still debouncing towards confirmation, same occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(SqliteFaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (3 FAILED events should confirm)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Debounce build-up is still one occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ImmediateConfirmationWithThresholdZero) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = 0;  // Immediate confirmation
  storage_->set_debounce_config(config);

  // Single report should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                               "/node1", clock.now(), default_config());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ClearedFaultCanBeReactivated) {
  rclcpp::Clock clock;

  // Report to confirm (with default threshold=-1, single report confirms)
  auto first_ts = clock.now();
  bool is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                             "Initial", "/node1", first_ts, default_config());
  EXPECT_TRUE(is_new);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), first_ts.nanoseconds());

  // Clear the fault
  storage_->clear_fault("FAULT_1");
  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);

  // Report again after a gap - should reactivate as a new cycle
  rclcpp::Time second_ts(first_ts.nanoseconds() + 1'000'000'000LL);  // +1s
  is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                        "Reactivated", "/node2", second_ts, default_config());
  EXPECT_TRUE(is_new);  // Should return true like a new fault

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Should be reconfirmed
  EXPECT_EQ(fault->occurrence_count, 2u);             // Should increment
  EXPECT_EQ(fault->reporting_sources.size(), 2u);     // Both sources
  EXPECT_EQ(fault->description, "Reactivated");       // Updated description
  // #25: first_occurred must reflect the new cycle, not the outage that already cleared.
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), second_ts.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, PassedEventForClearedFaultIgnored) {
  rclcpp::Clock clock;

  // Report and confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), default_config());

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // PASSED event should be ignored for CLEARED fault
  bool result = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1",
                                             clock.now(), default_config());
  EXPECT_FALSE(result);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(SqliteFaultStorageTest, ClearedFaultReactivationRestartsDebounce) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // Reactivate - should start in PREFAILED with counter=-1
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                               clock.now(), config);

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Not yet confirmed, needs 2 more FAILED

  // Report 2 more times to re-confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node5",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node6",
                               clock.now(), config);

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Now confirmed
}

TEST_F(SqliteFaultStorageTest, ConfirmationPersistsAfterReopen) {
  rclcpp::Clock clock;

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), default_config());

  // Close and reopen storage
  storage_.reset();
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  // Verify status persisted
  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;
  // confirmation_threshold = -3 so 2 FAILED stays PREFAILED (not CONFIRMED). A
  // confirmed fault is latched and would not move to PREPASSED on a heal.
  DebounceConfig config;
  config.confirmation_threshold = -3;

  // Report 2 FAILED events (counter -2, PREFAILED)
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);

  // Report 3 PASSED events (counter -2 -> +1)
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0, never confirmed so not latched
}

// Regression for #428: a periodic heal heartbeat on a healthy system used to push
// the debounce counter toward INT32_MAX, so a real fault then took a huge number of
// reports to confirm. The counter must clamp at the thresholds, and a confirmed or
// healed status must latch (one report must not flip it).
TEST_F(SqliteFaultStorageTest, HeartbeatHealClampedAndStatusLatched) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);

  // Long heal heartbeat: counter clamps at healing_threshold instead of running off.
  for (int i = 0; i < 100; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }
  auto healed = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(healed.has_value());
  EXPECT_EQ(healed->status, Fault::STATUS_HEALED);

  // Re-confirmation is now bounded to (healing_threshold - confirmation_threshold)
  // reports, and the healed status latches until the counter walks all the way down.
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                                 clock.now(), config);
    EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  auto reconfirmed = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(reconfirmed.has_value());
  EXPECT_EQ(reconfirmed->status, Fault::STATUS_CONFIRMED);
}

// A confirmed fault must not be un-confirmed by a heal heartbeat when auto-healing
// is off, and the counter must stay bounded.
TEST_F(SqliteFaultStorageTest, ConfirmedFaultSurvivesHealHeartbeat) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = false;
  config.healing_threshold = 3;  // upper clamp on the counter, even with healing disabled

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);

  for (int i = 0; i < 20; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
    EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED) << "un-confirmed after " << (i + 1);
  }

  // The counter is now positive (clamped at +3). One FAILED must keep it CONFIRMED, not flip it to
  // PREPASSED via the `counter > 0` branch - that was the asymmetric-latch bug.
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);
}

// Latch must hold in BOTH directions and identically on both backends (regression for the
// asymmetric per-branch latch). A CONFIRMED fault with a positive counter stays CONFIRMED on FAILED.
TEST_F(SqliteFaultStorageTest, ConfirmedLatchSurvivesFailedAtPositiveCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_threshold = 3;
  config.critical_immediate_confirm = true;

  // CRITICAL confirms immediately at counter -1.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "crit", "/n",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  // PASSED events raise the counter into positive territory; latch keeps it CONFIRMED.
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  // One normal FAILED: counter still positive, must NOT become PREPASSED.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
}

// A HEALED fault with a negative counter stays HEALED on PASSED (opposite direction of the bug above).
TEST_F(SqliteFaultStorageTest, HealedLatchSurvivesPassedAtNegativeCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  for (int i = 0; i < 4; ++i) {  // counter -1 -> +3 -> HEALED
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);
  // FAILED events drive the counter negative while HEALED is latched (not yet at confirmation -3).
  for (int i = 0; i < 5; ++i) {  // +3 -> -2
    storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                                 config);
    ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  // One PASSED: counter goes -2 -> -1, still negative; must stay HEALED, not flip to PREFAILED.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);
}

// A database written by an older build can hold a runaway counter. It must be clamped back on first
// touch so re-confirmation is bounded, not ~INT32 events away.
TEST_F(SqliteFaultStorageTest, RunawayCounterFromOldDbRecovers) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  // Simulate the old bug: poke a huge counter directly into the DB behind the storage's back.
  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(temp_db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw, "UPDATE faults SET debounce_counter = 100000, status = 'PREPASSED'", nullptr, nullptr,
                           nullptr),
              SQLITE_OK);
    sqlite3_close(raw);
  }
  // The counter is clamped back into range on first touch, so a bounded number of FAILED events
  // re-confirms (healing_threshold - confirmation_threshold = 4), not the ~100000 the runaway implied.
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                                 config);
  }
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ReclassifyHealedAsCleared) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);

  const auto reclassified = storage_->reclassify_healed_as_cleared();
  ASSERT_EQ(reclassified.size(), 1u);
  EXPECT_EQ(reclassified[0], "F");
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CLEARED);
  EXPECT_TRUE(storage_->reclassify_healed_as_cleared().empty());
}

TEST_F(SqliteFaultStorageTest, HealingWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_->set_debounce_config(config);

  // Report 1 FAILED event
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(SqliteFaultStorageTest, TimeBasedConfirmationWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce so fault stays PREFAILED
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               now, config);

  // Check before timeout - should not confirm
  auto before_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(5e9));
  auto confirmed_early = storage_->check_time_based_confirmation(before_timeout);
  EXPECT_TRUE(confirmed_early.empty());

  // Check after timeout - should confirm
  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_->check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0], "FAULT_1");

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// confirmed_at_ns column (consumed by the compliance timeline exporter)

namespace {
int64_t read_confirmed_at(const std::filesystem::path & db_path, const std::string & fault_code) {
  sqlite3 * raw = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.string().c_str(), &raw), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  EXPECT_EQ(sqlite3_prepare_v2(raw, "SELECT confirmed_at_ns FROM faults WHERE fault_code = ?", -1, &stmt, nullptr),
            SQLITE_OK);
  sqlite3_bind_text(stmt, 1, fault_code.c_str(), -1, SQLITE_TRANSIENT);
  int64_t value = -1;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    value = sqlite3_column_int64(stmt, 0);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(raw);
  return value;
}
}  // namespace

TEST_F(SqliteFaultStorageTest, ConfirmedAtRecordedOnImmediateConfirmation) {
  rclcpp::Clock clock;
  auto t = clock.now();
  // Default config confirms on the first FAILED event.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t,
                               default_config());
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "F"), t.nanoseconds());

  // A later FAILED on an already-confirmed fault must NOT move the timestamp.
  auto t2 = rclcpp::Time(t.nanoseconds() + static_cast<int64_t>(5e9));
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t2,
                               default_config());
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "F"), t.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, ConfirmedAtRecordedOnDebouncedConfirmation) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -2;  // second FAILED event confirms

  auto t1 = clock.now();
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t1, config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "F"), 0) << "not confirmed yet: no confirmation timestamp";

  auto t2 = rclcpp::Time(t1.nanoseconds() + static_cast<int64_t>(1e9));
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t2, config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "F"), t2.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, ConfirmedAtRecordedOnTimeBasedConfirmation) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", now, config);

  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_->check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "F"), after_timeout.nanoseconds());
}

TEST_F(SqliteFaultStorageTest, ConfirmedAtColumnMigratedIntoOldDatabase) {
  // Rebuild the faults table without confirmed_at_ns (a pre-migration DB),
  // then reopen: the storage must add the column and treat old rows as 0.
  storage_.reset();
  std::filesystem::remove(temp_db_path_);
  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(temp_db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw,
                           "CREATE TABLE faults (fault_code TEXT PRIMARY KEY, severity INTEGER NOT NULL, "
                           "description TEXT NOT NULL, first_occurred_ns INTEGER NOT NULL, "
                           "last_occurred_ns INTEGER NOT NULL, occurrence_count INTEGER NOT NULL, "
                           "status TEXT NOT NULL, reporting_sources TEXT NOT NULL, "
                           "debounce_counter INTEGER NOT NULL DEFAULT 0, "
                           "last_failed_ns INTEGER NOT NULL DEFAULT 0, last_passed_ns INTEGER NOT NULL DEFAULT 0); "
                           "INSERT INTO faults VALUES ('OLD', 2, 'd', 1, 1, 1, 'CONFIRMED', '[\"/n\"]', -1, 1, 0);",
                           nullptr, nullptr, nullptr),
              SQLITE_OK);
    sqlite3_close(raw);
  }
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());
  EXPECT_TRUE(storage_->contains("OLD"));
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "OLD"), 0) << "pre-migration rows carry no confirmation time";

  rclcpp::Clock clock;
  auto t = clock.now();
  storage_->report_fault_event("NEW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t,
                               default_config());
  EXPECT_EQ(read_confirmed_at(temp_db_path_, "NEW"), t.nanoseconds());
}

// --- #620: many recordings per fault -----------------------------------------

namespace {

/// Read PRAGMA index_list and report whether any index came from a table/column
/// UNIQUE constraint (origin 'u'), which is the thing ALTER TABLE cannot drop.
bool has_unique_constraint_index(const std::filesystem::path & db_path, const std::string & table) {
  sqlite3 * raw = nullptr;
  EXPECT_EQ(sqlite3_open(db_path.string().c_str(), &raw), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  const std::string sql = "PRAGMA index_list(" + table + ")";
  EXPECT_EQ(sqlite3_prepare_v2(raw, sql.c_str(), -1, &stmt, nullptr), SQLITE_OK);
  bool found = false;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const auto * origin = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
    if (origin != nullptr && std::string(origin) == "u") {
      found = true;
    }
  }
  sqlite3_finalize(stmt);
  sqlite3_close(raw);
  return found;
}

RosbagFileInfo make_rosbag(const std::string & code, const std::string & path, int64_t created_ns,
                           size_t bytes = 1024) {
  RosbagFileInfo info;
  info.fault_code = code;
  info.file_path = path;
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = bytes;
  info.created_at_ns = created_ns;
  return info;  // recording_id deliberately left empty - the backend must fill it
}

}  // namespace

TEST_F(SqliteFaultStorageTest, LegacyUniqueConstraintIsRebuiltAwayAndRecordingIdBackfilled) {
  // A database from before #620: fault_code carries a column-level UNIQUE and there
  // is no recording_id column at all. Opening it must rebuild the table, keep every
  // row, and derive recording_id from each path's basename.
  storage_.reset();
  std::filesystem::remove(temp_db_path_);
  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(temp_db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw,
                           "CREATE TABLE rosbag_files ("
                           " id INTEGER PRIMARY KEY AUTOINCREMENT,"
                           " fault_code TEXT NOT NULL UNIQUE,"
                           " file_path TEXT NOT NULL,"
                           " format TEXT NOT NULL,"
                           " duration_sec REAL NOT NULL,"
                           " size_bytes INTEGER NOT NULL,"
                           " created_at_ns INTEGER NOT NULL);"
                           "INSERT INTO rosbag_files (fault_code, file_path, format, duration_sec, size_bytes, "
                           "created_at_ns) VALUES ('A', '/bags/fault_A_100', 'mcap', 6.0, 10, 100);"
                           "INSERT INTO rosbag_files (fault_code, file_path, format, duration_sec, size_bytes, "
                           "created_at_ns) VALUES ('B', '/bags/fault_B_200', 'sqlite3', 6.0, 20, 200);",
                           nullptr, nullptr, nullptr),
              SQLITE_OK);
    sqlite3_close(raw);
  }
  ASSERT_TRUE(has_unique_constraint_index(temp_db_path_, "rosbag_files")) << "fixture must start constrained";

  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  EXPECT_FALSE(has_unique_constraint_index(temp_db_path_, "rosbag_files"))
      << "the column-level UNIQUE must be gone, replaced by a named index";

  const auto all = storage_->get_all_rosbag_files();
  ASSERT_EQ(all.size(), 2u) << "no row may be lost in the rebuild";
  // Oldest first, so the original insertion order survived the copy.
  EXPECT_EQ(all[0].fault_code, "A");
  EXPECT_EQ(all[1].fault_code, "B");
  EXPECT_EQ(all[0].recording_id, "fault_A_100") << "backfilled from the path basename";
  EXPECT_EQ(all[1].recording_id, "fault_B_200");
}

TEST_F(SqliteFaultStorageTest, ReopeningAMigratedDatabaseIsANoOp) {
  // The migration runs on every open, so it has to be idempotent - a second and
  // third open must not rebuild again, lose rows or re-backfill.
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_100", 100));
  const auto before = storage_->get_all_rosbag_files();

  for (int i = 0; i < 2; ++i) {
    storage_.reset();
    storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());
    const auto after = storage_->get_all_rosbag_files();
    ASSERT_EQ(after.size(), before.size()) << "reopen #" << i + 1 << " changed the row count";
    EXPECT_EQ(after[0].recording_id, before[0].recording_id);
    EXPECT_FALSE(has_unique_constraint_index(temp_db_path_, "rosbag_files"));
  }
}

TEST_F(SqliteFaultStorageTest, OneFaultKeepsSeveralRecordingsWhenTheCapAllows) {
  storage_->set_max_rosbags_per_fault(3);
  storage_->store_rosbag_file(make_rosbag("FLAP", "/bags/fault_FLAP_100", 100));
  storage_->store_rosbag_file(make_rosbag("FLAP", "/bags/fault_FLAP_200", 200));

  const auto rows = storage_->get_rosbag_files("FLAP");
  ASSERT_EQ(rows.size(), 2u) << "the second recording must not replace the first";
  EXPECT_EQ(rows[0].recording_id, "fault_FLAP_200") << "newest first";
  EXPECT_EQ(rows[1].recording_id, "fault_FLAP_100");

  // get_rosbag_file is "the newest", deterministically.
  const auto newest = storage_->get_rosbag_file("FLAP");
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->recording_id, "fault_FLAP_200");
}

TEST_F(SqliteFaultStorageTest, CapKeepsTheNewestAndUnlinksTheEvictedBag) {
  const auto bags = temp_db_path_.parent_path() / (temp_db_path_.stem().string() + "_bags");
  const auto dir_a = bags / "fault_CAP_100";
  const auto dir_b = bags / "fault_CAP_200";
  std::filesystem::create_directories(dir_a);
  std::filesystem::create_directories(dir_b);

  storage_->set_max_rosbags_per_fault(1);
  storage_->store_rosbag_file(make_rosbag("CAP", dir_a.string(), 100));
  storage_->store_rosbag_file(make_rosbag("CAP", dir_b.string(), 200));

  const auto rows = storage_->get_rosbag_files("CAP");
  ASSERT_EQ(rows.size(), 1u) << "cap 1 is the historical behaviour: one recording per fault";
  EXPECT_EQ(rows[0].file_path, dir_b.string());
  EXPECT_FALSE(std::filesystem::exists(dir_a)) << "the evicted bag must be unlinked";
  EXPECT_TRUE(std::filesystem::exists(dir_b));
}

TEST_F(SqliteFaultStorageTest, EvictingOneFaultsLinkKeepsABagASiblingStillReferences) {
  // A burst shares one recording. Evicting it for one fault must not take the bytes
  // the other fault still points at.
  const auto bags = temp_db_path_.parent_path() / (temp_db_path_.stem().string() + "_bags");
  const auto shared = bags / "fault_SHARED_100";
  const auto later = bags / "fault_A_200";
  std::filesystem::create_directories(shared);
  std::filesystem::create_directories(later);

  storage_->set_max_rosbags_per_fault(1);
  storage_->store_rosbag_files({make_rosbag("A", shared.string(), 100), make_rosbag("B", shared.string(), 100)});
  // A re-confirms with its own new bag, so A's link to the shared one is evicted.
  storage_->store_rosbag_file(make_rosbag("A", later.string(), 200));

  EXPECT_TRUE(std::filesystem::exists(shared)) << "B still references it";
  const auto b_rows = storage_->get_rosbag_files("B");
  ASSERT_EQ(b_rows.size(), 1u);
  EXPECT_EQ(b_rows[0].file_path, shared.string());
}

TEST_F(SqliteFaultStorageTest, RecordingLookupReturnsEveryFaultOfTheBurst) {
  storage_->store_rosbag_files({make_rosbag("A", "/bags/fault_A_100", 100), make_rosbag("B", "/bags/fault_A_100", 100),
                                make_rosbag("C", "/bags/fault_A_100", 100)});

  const auto rows = storage_->get_rosbag_files_by_recording("fault_A_100");
  ASSERT_EQ(rows.size(), 3u) << "the download authorizes against this set";
  EXPECT_EQ(rows[0].fault_code, "A");
  EXPECT_EQ(rows[2].fault_code, "C");
  EXPECT_TRUE(storage_->get_rosbag_files_by_recording("fault_NOPE_1").empty());
}

TEST_F(SqliteFaultStorageTest, DeleteRecordingRemovesEveryLinkAndTheBag) {
  const auto bags = temp_db_path_.parent_path() / (temp_db_path_.stem().string() + "_bags");
  const auto dir = bags / "fault_A_100";
  std::filesystem::create_directories(dir);
  storage_->store_rosbag_files({make_rosbag("A", dir.string(), 100), make_rosbag("B", dir.string(), 100)});

  EXPECT_EQ(storage_->delete_rosbag_recording("fault_A_100"), 2u);
  EXPECT_TRUE(storage_->get_rosbag_files("A").empty());
  EXPECT_TRUE(storage_->get_rosbag_files("B").empty());
  EXPECT_FALSE(std::filesystem::exists(dir));
}

TEST_F(SqliteFaultStorageTest, DeletingAFaultDropsAllItsRecordings) {
  storage_->set_max_rosbags_per_fault(0);
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_100", 100));
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_200", 200));

  EXPECT_TRUE(storage_->delete_rosbag_file("A"));
  EXPECT_TRUE(storage_->get_rosbag_files("A").empty()) << "auto_cleanup drops the fault's whole history";
}

TEST_F(SqliteFaultStorageTest, SharedRecordingStillCountsOnceTowardsStorageWithSeveralRecordings) {
  storage_->set_max_rosbags_per_fault(0);
  storage_->store_rosbag_files(
      {make_rosbag("A", "/bags/fault_A_100", 100, 4096), make_rosbag("B", "/bags/fault_A_100", 100, 4096)});
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_200", 200, 1024));

  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 4096u + 1024u) << "bytes belong to file_path, not to the row";
}

// Snapshot storage tests
// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, StoreAndRetrieveSnapshot) {
  using ros2_medkit_fault_manager::SnapshotData;

  // First, create a fault to associate the snapshot with
  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Store a snapshot
  SnapshotData snapshot;
  snapshot.fault_code = "MOTOR_OVERHEAT";
  snapshot.topic = "/motor/temperature";
  snapshot.message_type = "sensor_msgs/msg/Temperature";
  snapshot.data = R"({"temperature": 85.5, "variance": 0.1})";
  snapshot.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot);

  // Retrieve snapshots
  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  ASSERT_EQ(snapshots.size(), 1u);

  EXPECT_EQ(snapshots[0].fault_code, "MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots[0].topic, "/motor/temperature");
  EXPECT_EQ(snapshots[0].message_type, "sensor_msgs/msg/Temperature");
  EXPECT_EQ(snapshots[0].data, R"({"temperature": 85.5, "variance": 0.1})");
  EXPECT_EQ(snapshots[0].captured_at_ns, snapshot.captured_at_ns);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, MultipleSnapshotsForSameFault) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Store multiple snapshots for the same fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots.size(), 2u);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, FilterSnapshotsByTopic) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  // Filter by topic
  auto filtered = storage_->get_snapshots("MOTOR_OVERHEAT", "/motor/temperature");
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered[0].topic, "/motor/temperature");
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, NoSnapshotsForUnknownFault) {
  auto snapshots = storage_->get_snapshots("UNKNOWN_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, ClearFaultDeletesAssociatedSnapshots) {
  using ros2_medkit_fault_manager::SnapshotData;
  rclcpp::Clock clock;

  // Create a fault using report_fault_event
  storage_->report_fault_event("SNAPSHOT_CLEAR_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Test fault for snapshot cleanup", "/test_node", clock.now(), default_config());

  // Store snapshots for this fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot1.topic = "/test/topic1";
  snapshot1.message_type = "std_msgs/msg/String";
  snapshot1.data = R"({"data": "test1"})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot1);

  SnapshotData snapshot2;
  snapshot2.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot2.topic = "/test/topic2";
  snapshot2.message_type = "std_msgs/msg/String";
  snapshot2.data = R"({"data": "test2"})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot2);

  // Verify snapshots exist
  auto snapshots_before = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  ASSERT_EQ(snapshots_before.size(), 2u);

  // Clear the fault
  bool cleared = storage_->clear_fault("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(cleared);

  // Verify snapshots are deleted
  auto snapshots_after = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(snapshots_after.empty());
}

// Freeze-frame storage tests
// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, ClearFaultKeepsSnapshotsWhenEvidenceIsRetained) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("KEEP", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "keep", "/n",
                               clock.now(), default_config());
  storage_->set_retain_snapshots_on_clear(true);

  SnapshotData row;
  row.fault_code = "KEEP";
  row.topic = "/t";
  row.message_type = "std_msgs/msg/Float64";
  row.data = R"({"data": 1.0})";
  row.captured_at_ns = 1000;
  row.capture_id = 1;
  storage_->store_snapshots({row});

  ASSERT_TRUE(storage_->clear_fault("KEEP"));

  // Recordings survive an acknowledgement once a history is configured, so the
  // readings captured beside them have to as well - otherwise the fault is left
  // holding bags whose values are gone, which is worse than losing both.
  EXPECT_EQ(storage_->get_snapshots("KEEP").size(), 1u);
}

TEST_F(SqliteFaultStorageTest, StoreAndRetrieveFreezeFrame) {
  using ros2_medkit_fault_manager::FreezeFrameData;
  rclcpp::Clock clock;

  storage_->report_fault_event("PLC_PRESSURE_HIGH", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Pressure high", "/plc_node", clock.now(), default_config());

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4},"/plc/valve":{"data":true}})";
  frame.captured_at_ns = clock.now().nanoseconds();
  storage_->store_freeze_frame(frame);

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->fault_code, "PLC_PRESSURE_HIGH");
  EXPECT_EQ(retrieved->data, frame.data);
  EXPECT_EQ(retrieved->captured_at_ns, frame.captured_at_ns);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, NoFreezeFrameForUnknownFault) {
  auto retrieved = storage_->get_freeze_frame("NEVER_CAPTURED");
  EXPECT_FALSE(retrieved.has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, FreezeFrameReplacedOnRecapture) {
  using ros2_medkit_fault_manager::FreezeFrameData;
  rclcpp::Clock clock;

  FreezeFrameData first;
  first.fault_code = "PLC_PRESSURE_HIGH";
  first.data = R"({"/plc/pressure":{"data":8.4}})";
  first.captured_at_ns = 1000;
  storage_->store_freeze_frame(first);

  FreezeFrameData second;
  second.fault_code = "PLC_PRESSURE_HIGH";
  second.data = R"({"/plc/pressure":{"data":9.9}})";
  second.captured_at_ns = 2000;
  storage_->store_freeze_frame(second);

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, second.data);
  EXPECT_EQ(retrieved->captured_at_ns, 2000);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, FreezeFrameSurvivesClearFault) {
  using ros2_medkit_fault_manager::FreezeFrameData;
  using ros2_medkit_fault_manager::SnapshotData;
  rclcpp::Clock clock;

  storage_->report_fault_event("PLC_PRESSURE_HIGH", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Pressure high", "/plc_node", clock.now(), default_config());

  // A per-topic snapshot (removed on clear) plus a freeze-frame (retained on clear).
  SnapshotData snapshot;
  snapshot.fault_code = "PLC_PRESSURE_HIGH";
  snapshot.topic = "/plc/pressure";
  snapshot.message_type = "std_msgs/msg/Float64";
  snapshot.data = R"({"data":8.4})";
  snapshot.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot);

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = clock.now().nanoseconds();
  storage_->store_freeze_frame(frame);

  ASSERT_TRUE(storage_->clear_fault("PLC_PRESSURE_HIGH"));

  // Snapshots are wiped on clear, the freeze-frame is retained and still retrievable.
  EXPECT_TRUE(storage_->get_snapshots("PLC_PRESSURE_HIGH").empty());
  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, FreezeFramePersistsAcrossReopen) {
  using ros2_medkit_fault_manager::FreezeFrameData;

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = 4242;
  storage_->store_freeze_frame(frame);

  // Reopen the same database file.
  storage_.reset();
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
  EXPECT_EQ(retrieved->captured_at_ns, 4242);
}

// Rosbag entity-scoped listing tests

// @verifies REQ_INTEROP_071
TEST_F(SqliteFaultStorageTest, ListRosbagsForEntityFiltersCorrectly) {
  using ros2_medkit_fault_manager::RosbagFileInfo;
  rclcpp::Clock clock;

  // Create fault with reporting source for entity
  storage_->report_fault_event("ENTITY_FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Fault from entity", "/powertrain/motor", clock.now(), default_config());

  // Create another fault with different reporting source
  storage_->report_fault_event("ENTITY_FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Fault from other entity", "/chassis/brake", clock.now(), default_config());

  // Store rosbags for both faults
  RosbagFileInfo info1;
  info1.fault_code = "ENTITY_FAULT_1";
  info1.file_path = "/tmp/entity1.mcap";
  info1.format = "mcap";
  info1.duration_sec = 5.0;
  info1.size_bytes = 1024;
  info1.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "ENTITY_FAULT_2";
  info2.file_path = "/tmp/entity2.mcap";
  info2.format = "mcap";
  info2.duration_sec = 3.0;
  info2.size_bytes = 512;
  info2.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info2);

  // Get rosbags for motor entity
  auto rosbags = storage_->list_rosbags_for_entity("/powertrain/motor");
  ASSERT_EQ(rosbags.size(), 1u);
  EXPECT_EQ(rosbags[0].fault_code, "ENTITY_FAULT_1");

  // Get rosbags for brake entity
  auto brake_rosbags = storage_->list_rosbags_for_entity("/chassis/brake");
  ASSERT_EQ(brake_rosbags.size(), 1u);
  EXPECT_EQ(brake_rosbags[0].fault_code, "ENTITY_FAULT_2");

  // Get rosbags for unknown entity
  auto unknown_rosbags = storage_->list_rosbags_for_entity("/unknown/entity");
  EXPECT_TRUE(unknown_rosbags.empty());
}

TEST_F(SqliteFaultStorageTest, AFailingRowDeleteKeepsTheBagOnDisk) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  // delete_rosbag_file() has to survive a DELETE that throws - a busy, full or
  // read-only database - without having already unlinked the bag. A surviving row
  // whose directory is gone fails every later retrieval and keeps its bytes charged
  // against the quota, which sums rows. The batch sibling states this invariant in
  // its own comment; this pins it for the single-row path too.
  const auto bag_dir = std::filesystem::temp_directory_path() / "test_failing_delete_bag";
  std::filesystem::create_directories(bag_dir);
  { std::ofstream(bag_dir / "payload.mcap") << "data"; }

  RosbagFileInfo info;
  info.fault_code = "DELETE_FAILS";
  info.file_path = bag_dir.string();
  info.format = "mcap";
  info.duration_sec = 1.0;
  info.size_bytes = 4;
  info.created_at_ns = 1000;
  storage_->store_rosbag_file(info);

  // Make the DELETE, and only the DELETE, fail. A dropped or renamed table would
  // take the preceding SELECT with it, and the unlink under test comes after that.
  sqlite3 * raw = nullptr;
  ASSERT_EQ(sqlite3_open(temp_db_path_.string().c_str(), &raw), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(raw,
                         "CREATE TRIGGER block_rosbag_delete BEFORE DELETE ON rosbag_files "
                         "BEGIN SELECT RAISE(ABORT, 'delete blocked'); END;",
                         nullptr, nullptr, nullptr),
            SQLITE_OK);
  sqlite3_close(raw);

  EXPECT_THROW(storage_->delete_rosbag_file("DELETE_FAILS"), std::runtime_error);

  EXPECT_TRUE(storage_->get_rosbag_file("DELETE_FAILS").has_value()) << "the DELETE failed, so its row must remain";
  EXPECT_TRUE(std::filesystem::exists(bag_dir))
      << "the bag was unlinked before its row was deleted, so the surviving row now points at nothing";

  std::error_code ec;
  std::filesystem::remove_all(bag_dir, ec);
}

// @verifies REQ_INTEROP_073
TEST_F(SqliteFaultStorageTest, GetAllRosbagFilesReturnsSortedByCreatedAt) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  RosbagFileInfo info1;
  info1.fault_code = "FAULT_A";
  info1.file_path = "/tmp/a.mcap";
  info1.format = "mcap";
  info1.duration_sec = 1.0;
  info1.size_bytes = 100;
  info1.created_at_ns = 1000;
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "FAULT_B";
  info2.file_path = "/tmp/b.mcap";
  info2.format = "mcap";
  info2.duration_sec = 2.0;
  info2.size_bytes = 200;
  info2.created_at_ns = 2000;
  storage_->store_rosbag_file(info2);

  auto all_rosbags = storage_->get_all_rosbag_files();
  ASSERT_EQ(all_rosbags.size(), 2u);

  // Should be sorted by created_at_ns (oldest first)
  EXPECT_EQ(all_rosbags[0].fault_code, "FAULT_A");
  EXPECT_EQ(all_rosbags[1].fault_code, "FAULT_B");
}

// Shared-recording tests: a burst of correlated faults confirming inside one
// post-roll window all reference the same bag, so the file must outlive every
// record but one, and the quota must not count it once per fault.

TEST_F(SqliteFaultStorageTest, SharedRosbagSurvivesUntilTheLastFaultIsDeleted) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  auto bag_path = temp_db_path_.string() + "_shared_bag";
  std::filesystem::create_directories(bag_path);

  RosbagFileInfo info;
  info.fault_code = "ROOT_CAUSE";
  info.file_path = bag_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;
  storage_->store_rosbag_file(info);

  info.fault_code = "CORRELATED";
  storage_->store_rosbag_file(info);

  EXPECT_TRUE(storage_->delete_rosbag_file("CORRELATED"));
  EXPECT_TRUE(std::filesystem::exists(bag_path));
  EXPECT_TRUE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());

  EXPECT_TRUE(storage_->delete_rosbag_file("ROOT_CAUSE"));
  EXPECT_FALSE(std::filesystem::exists(bag_path));
}

TEST_F(SqliteFaultStorageTest, SharedRosbagCountsOnceTowardsStorageTotal) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  // Rows of one recording can transiently disagree on size while it is being
  // finalised, so the total takes MAX(size_bytes) per path. Two shared bags
  // with opposite insert orderings, so neither first- nor last-write-wins can
  // fake the MAX (mirrors the in-memory SharedBagTotalTakesTheLargestRowPerPath).
  auto store = [this](const char * code, const char * path, size_t bytes) {
    RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = path;
    info.format = "mcap";
    info.duration_sec = 5.0;
    info.size_bytes = bytes;
    info.created_at_ns = 1000;
    storage_->store_rosbag_file(info);
  };

  store("BIG_FIRST", "/tmp/shared_a.mcap", 1000);
  store("SMALL_SECOND", "/tmp/shared_a.mcap", 300);
  store("SMALL_FIRST", "/tmp/shared_b.mcap", 200);
  store("BIG_SECOND", "/tmp/shared_b.mcap", 800);
  store("UNRELATED", "/tmp/other.mcap", 500);

  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 1000u + 800u + 500u);
}

// Re-store guard: a fault re-confirming stores a row with a new path, and the
// old bag must be unlinked only when no sibling fault still references it.

TEST_F(SqliteFaultStorageTest, RestoreWithNewPathUnlinksTheOldExclusiveBag) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto old_path = temp_db_path_.string() + "_exclusive_bag";
  std::filesystem::create_directories(old_path);

  RosbagFileInfo info;
  info.fault_code = "X";
  info.file_path = old_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  storage_->store_rosbag_file(info);

  info.file_path = old_path + "_new";
  storage_->store_rosbag_file(info);

  EXPECT_FALSE(std::filesystem::exists(old_path)) << "nobody references the old bag, it must be unlinked";
  auto row = storage_->get_rosbag_file("X");
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(row->file_path, old_path + "_new");
}

TEST_F(SqliteFaultStorageTest, RestoreWithNewPathKeepsTheBagASiblingStillReferences) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto shared_path = temp_db_path_.string() + "_shared_bag";
  std::filesystem::create_directories(shared_path);

  RosbagFileInfo info;
  info.file_path = shared_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  info.fault_code = "X";
  storage_->store_rosbag_file(info);
  info.fault_code = "Y";
  storage_->store_rosbag_file(info);

  info.fault_code = "X";
  info.file_path = shared_path + "_new";
  storage_->store_rosbag_file(info);

  EXPECT_TRUE(std::filesystem::exists(shared_path)) << "the sibling fault still owns the shared bag";
  auto sibling = storage_->get_rosbag_file("Y");
  ASSERT_TRUE(sibling.has_value());
  EXPECT_EQ(sibling->file_path, shared_path);

  std::error_code ec;
  std::filesystem::remove_all(shared_path, ec);
}

// Burst-level batch operations (one SQLite transaction per burst).

TEST_F(SqliteFaultStorageTest, BulkStoreRegistersEveryFaultOfTheBurst) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  RosbagFileInfo info;
  info.file_path = "/tmp/burst_bag";
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;

  std::vector<RosbagFileInfo> rows;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    info.fault_code = code;
    rows.push_back(info);
  }
  storage_->store_rosbag_files(rows);

  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    auto row = storage_->get_rosbag_file(code);
    ASSERT_TRUE(row.has_value()) << code;
    EXPECT_EQ(row->file_path, "/tmp/burst_bag");
  }
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 4096u);
}

TEST_F(SqliteFaultStorageTest, BulkDeleteRemovesTheBurstAndUnlinksTheBagOnce) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto bag_path = temp_db_path_.string() + "_burst_bag";
  std::filesystem::create_directories(bag_path);

  RosbagFileInfo info;
  info.file_path = bag_path;
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;
  std::vector<RosbagFileInfo> rows;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    info.fault_code = code;
    rows.push_back(info);
  }
  storage_->store_rosbag_files(rows);

  // A partial delete leaves the bag on disk for the remaining fault.
  EXPECT_EQ(storage_->delete_rosbag_files({"CORRELATED_A", "CORRELATED_B", "NEVER_STORED"}), 2u);
  EXPECT_TRUE(std::filesystem::exists(bag_path));
  EXPECT_TRUE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());
  EXPECT_FALSE(storage_->get_rosbag_file("CORRELATED_A").has_value());

  // The last reference going away unlinks the directory.
  EXPECT_EQ(storage_->delete_rosbag_files({"ROOT_CAUSE"}), 1u);
  EXPECT_FALSE(std::filesystem::exists(bag_path));
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 0u);
}

// =============================================================================
// Snapshot limit tests (issue #308)
// =============================================================================

TEST_F(SqliteFaultStorageTest, SnapshotCapKeepsTheNewestCaptureWholeAndDropsTheOldest) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Two topics per capture, room for two captures.
  storage_->set_max_snapshots_per_fault(4);

  const auto capture = [](int64_t id, int64_t at) {
    std::vector<SnapshotData> rows;
    for (const char * topic : {"/motor/temp", "/motor/rpm"}) {
      SnapshotData row;
      row.fault_code = "MOTOR_OVERHEAT";
      row.topic = topic;
      row.message_type = "std_msgs/msg/Float64";
      row.data = R"({"data": 1.0})";
      row.captured_at_ns = at;
      row.capture_id = id;
      rows.push_back(row);
    }
    return rows;
  };

  storage_->store_snapshots(capture(1, 1000));
  storage_->store_snapshots(capture(2, 2000));
  storage_->store_snapshots(capture(3, 3000));

  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");

  // The third capture is stored WHOLE and the first goes whole. The old rule
  // counted rows and rejected the new one once full, so capture 3 landed with one
  // topic present and the other silently missing - a freeze frame with a hole in
  // it that reads exactly like "that topic was not publishing".
  ASSERT_EQ(snapshots.size(), 4u);
  std::set<int64_t> captures;
  for (const auto & s : snapshots) {
    captures.insert(s.capture_id);
  }
  EXPECT_EQ(captures, (std::set<int64_t>{2, 3}));
  for (int64_t id : {2, 3}) {
    EXPECT_EQ(std::count_if(snapshots.begin(), snapshots.end(),
                            [id](const SnapshotData & s) {
                              return s.capture_id == id;
                            }),
              2)
        << "capture " << id << " was stored in part";
  }
}

TEST_F(SqliteFaultStorageTest, ACaptureLargerThanTheCapIsKeptWholeRatherThanTorn) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("WIDE", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "many topics", "/n",
                               clock.now(), default_config());
  storage_->set_max_snapshots_per_fault(2);

  std::vector<SnapshotData> rows;
  for (int i = 0; i < 4; ++i) {
    SnapshotData row;
    row.fault_code = "WIDE";
    row.topic = "/t" + std::to_string(i);
    row.message_type = "std_msgs/msg/Float64";
    row.data = "{}";
    row.captured_at_ns = 1000;
    row.capture_id = 7;
    rows.push_back(row);
  }
  storage_->store_snapshots(rows);

  // The cap is smaller than this fault's topic count. Trimming to it would mean
  // storing the reading with holes, which is the failure being fixed; the capture
  // stays whole and the operator can see the cap is too small.
  EXPECT_EQ(storage_->get_snapshots("WIDE").size(), 4u);
}

TEST_F(SqliteFaultStorageTest, SnapshotLimitZeroMeansUnlimited) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("FAULT_A", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());

  // Default (0) = unlimited
  storage_->set_max_snapshots_per_fault(0);

  for (int i = 0; i < 20; ++i) {
    SnapshotData snap;
    snap.fault_code = "FAULT_A";
    snap.topic = "/topic";
    snap.message_type = "std_msgs/msg/String";
    snap.data = "{}";
    snap.captured_at_ns = i * 1000;
    storage_->store_snapshot(snap);
  }

  auto snapshots = storage_->get_snapshots("FAULT_A");
  EXPECT_EQ(snapshots.size(), 20u) << "Unlimited mode should store all snapshots";
}

TEST_F(SqliteFaultStorageTest, SnapshotLimitPerFaultNotGlobal) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("FAULT_A", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_B", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());

  storage_->set_max_snapshots_per_fault(1);

  SnapshotData snap_a;
  snap_a.fault_code = "FAULT_A";
  snap_a.topic = "/topic";
  snap_a.message_type = "std_msgs/msg/String";
  snap_a.data = "{}";
  snap_a.captured_at_ns = 1000;

  SnapshotData snap_b = snap_a;
  snap_b.fault_code = "FAULT_B";

  storage_->store_snapshot(snap_a);
  storage_->store_snapshot(snap_b);

  // Both faults should have 1 snapshot each (limit is per-fault)
  EXPECT_EQ(storage_->get_snapshots("FAULT_A").size(), 1u);
  EXPECT_EQ(storage_->get_snapshots("FAULT_B").size(), 1u);
}

// --- Near-miss series ---
//
// A near miss is a FAILED report that moved the debounce counter without the fault ending up
// CONFIRMED. The series is append-only and must survive clear_fault, because acknowledging a
// fault cycle must not erase how often that code approached confirmation.

/// Debounce config that takes four FAILED reports to confirm, leaving three near misses first.
static DebounceConfig four_strike_config() {
  DebounceConfig config;
  config.confirmation_threshold = -4;
  config.critical_immediate_confirm = false;
  return config;
}

/// Deterministic timestamps 1 ms apart, so series ordering is checkable.
static rclcpp::Time nth_report_time(int index) {
  constexpr int64_t kBaseNs = 1700000000000000000LL;
  return rclcpp::Time(kBaseNs + static_cast<int64_t>(index) * 1000000LL);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesIsAppendedNotOverwritten) {
  const auto config = four_strike_config();

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_NE(fault->status, Fault::STATUS_CONFIRMED) << "test setup: these reports must not confirm";

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "each near miss must append an entry, not overwrite the last";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[1].debounce_counter, -2);
  EXPECT_EQ(series[2].debounce_counter, -3);
  EXPECT_EQ(series[0].confirmation_threshold, -4);
  EXPECT_EQ(series[0].fault_code, "PUMP_PRESSURE_LOW");
  EXPECT_EQ(series[0].source_id, "/hydraulics/pump");
  EXPECT_EQ(series[0].severity, Fault::SEVERITY_WARN);
  EXPECT_EQ(series[0].occurred_at_ns, nth_report_time(0).nanoseconds());
  EXPECT_LT(series[0].occurred_at_ns, series[2].occurred_at_ns);
}

TEST_F(SqliteFaultStorageTest, ConfirmingReportIsNotANearMiss) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // The fourth report is the fault happening, not nearly happening.
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesSurvivesClearFault) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);

  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "acknowledging the fault destroyed the near-miss record";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[2].debounce_counter, -3);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesContinuesAcrossReactivation) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  // A new outage cycle starts: the reactivating report resets the counter to -1 without confirming.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping again", "/hydraulics/pump", nth_report_time(10), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 4u) << "the series must span fault cycles, one entry per occurrence";
  EXPECT_EQ(series[3].debounce_counter, -1);
  EXPECT_EQ(series[3].occurred_at_ns, nth_report_time(10).nanoseconds());
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesSurvivesReopen) {
  const auto config = four_strike_config();

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  storage_.reset();
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "the series must outlive the process, not just the fault cycle";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[2].debounce_counter, -3);
}

TEST_F(SqliteFaultStorageTest, PassedReportIsNotANearMiss) {
  const auto config = four_strike_config();

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);

  // A PASSED report moves the counter in the healing direction: the fault receding, not nearing.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                               "/hydraulics/pump", nth_report_time(2), config);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
}

TEST_F(SqliteFaultStorageTest, CriticalImmediateConfirmIsNotANearMiss) {
  DebounceConfig config = four_strike_config();
  config.critical_immediate_confirm = true;

  storage_->report_fault_event("BATTERY_THERMAL_RUNAWAY", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL,
                               "cell over temperature", "/power/bms", nth_report_time(0), config);

  auto fault = storage_->get_fault("BATTERY_THERMAL_RUNAWAY");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_TRUE(storage_->get_near_misses("BATTERY_THERMAL_RUNAWAY").empty());
}

TEST_F(SqliteFaultStorageTest, ImmediateConfirmThresholdRecordsNoNearMiss) {
  // Endpoint of the documented range: confirmation_threshold = -1 confirms on the first report,
  // so a fault under this config never has a near miss to record.
  const auto config = default_config();

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "pressure lost", "/hydraulics/pump", nth_report_time(0), config);

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_TRUE(storage_->get_near_misses("PUMP_PRESSURE_LOW").empty());
}

TEST_F(SqliteFaultStorageTest, FailedReportUnderHealedLatchIsNearMiss) {
  DebounceConfig config = four_strike_config();
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);

  for (int i = 1; i <= 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                                 "/hydraulics/pump", nth_report_time(i), config);
  }
  auto healed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";

  // The latch keeps the status at HEALED, but the counter moved toward confirmation: a near miss.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(3), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[1].debounce_counter, 0);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesBoundedKeepingNewest) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(3);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u);
  // Oldest-first eviction: a series frozen at boot would say nothing about a trend.
  EXPECT_EQ(series[0].debounce_counter, -3);
  EXPECT_EQ(series[1].debounce_counter, -4);
  EXPECT_EQ(series[2].debounce_counter, -5);
}

TEST_F(SqliteFaultStorageTest, NearMissBoundOfOneKeepsLatestOnly) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(1);

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 1u);
  EXPECT_EQ(series[0].debounce_counter, -4);
}

TEST_F(SqliteFaultStorageTest, NearMissBoundIsPerFaultCode) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(2);

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
  EXPECT_EQ(storage_->get_near_misses("MOTOR_OVERHEAT").size(), 2u);
}

TEST_F(SqliteFaultStorageTest, NearMissBoundZeroIsUnlimited) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -200;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 150; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 150u);
}

TEST_F(SqliteFaultStorageTest, NearMissTableCreatedOnDatabaseFromOlderBuild) {
  // The appliances this matters for already have a faults.db written by a build with no
  // near_misses table. Opening one must add the table, not fail and not skip recording.
  const auto config = four_strike_config();
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  storage_.reset();

  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(temp_db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw, "DROP TABLE near_misses", nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(raw);
  }

  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());
  EXPECT_TRUE(storage_->get_near_misses("PUMP_PRESSURE_LOW").empty());

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(1), config);
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesSurvivesHealedReclassification) {
  // Startup reclassification is the other place that drops a fault's captured data; it must
  // leave the series alone for the same reason clear_fault does.
  DebounceConfig config = four_strike_config();
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  for (int i = 1; i <= 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                                 "/hydraulics/pump", nth_report_time(i), config);
  }
  auto healed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);

  ASSERT_EQ(storage_->reclassify_healed_as_cleared().size(), 1u);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesUsesArrivalOrderNotTimestamps) {
  // Reporters carry their own clocks, so a report can arrive with a timestamp behind one already
  // stored. Ordering eviction by timestamp would delete the row that was just appended.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(2);

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(10 + i), config);
  }
  // Arrives third, but carries the earliest timestamp of the three.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].occurred_at_ns, nth_report_time(11).nanoseconds());
  EXPECT_EQ(series[1].occurred_at_ns, nth_report_time(0).nanoseconds())
      << "the report that just arrived must not be the one evicted";
  EXPECT_EQ(series[1].debounce_counter, -3);
}

TEST_F(SqliteFaultStorageTest, NearMissEntriesDescribeTheirOwnReport) {
  // Each entry must describe the report that produced it, not the fault's current state, or a
  // series spanning a threshold change or a new reporting source reads as if nothing changed.
  DebounceConfig first = four_strike_config();
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), first);

  DebounceConfig second = four_strike_config();
  second.confirmation_threshold = -8;
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "pressure dipping", "/hydraulics/backup_pump", nth_report_time(1), second);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].confirmation_threshold, -4);
  EXPECT_EQ(series[0].severity, Fault::SEVERITY_WARN);
  EXPECT_EQ(series[0].source_id, "/hydraulics/pump");
  EXPECT_EQ(series[1].confirmation_threshold, -8);
  EXPECT_EQ(series[1].severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(series[1].source_id, "/hydraulics/backup_pump");
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesContinuesAfterReopen) {
  const auto config = four_strike_config();

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  storage_.reset();
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(2), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "a restart must extend the series, not restart or reorder it";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[1].debounce_counter, -2);
  EXPECT_EQ(series[2].debounce_counter, -3);
  EXPECT_EQ(series[2].occurred_at_ns, nth_report_time(2).nanoseconds());
}

TEST_F(SqliteFaultStorageTest, ApplyingSmallerBoundTrimsExistingSeries) {
  // A database that grew under a larger bound, or none, must come back inside the bound as soon
  // as it is applied. Waiting for the next near miss leaves a quiet fault code over the bound for
  // good.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 5u);

  storage_->set_max_near_misses_per_fault(2);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u) << "applying the bound left the stored series over it";
  EXPECT_EQ(series[0].debounce_counter, -4);
  EXPECT_EQ(series[1].debounce_counter, -5);
  EXPECT_EQ(storage_->get_near_misses("MOTOR_OVERHEAT").size(), 2u) << "the bound is applied per fault code";
}

TEST_F(SqliteFaultStorageTest, ApplyingUnlimitedBoundKeepsExistingSeries) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(4);

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  storage_->set_max_near_misses_per_fault(0);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 4u);
}

TEST_F(SqliteFaultStorageTest, UnlimitedBoundSpeltAsSizeMaxKeepsTheSeries) {
  // SIZE_MAX is the idiomatic spelling of "no limit". Bound straight into an int64 it becomes -1,
  // and every row then compares as beyond the bound, which would empty the table.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->set_max_near_misses_per_fault(std::numeric_limits<size_t>::max()), 0u);
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(3), config);
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 4u);
}

TEST_F(SqliteFaultStorageTest, ApplyingBoundReportsHowManyEntriesItDropped) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }

  // Two codes, five entries each, bound of 2: three dropped per code.
  EXPECT_EQ(storage_->set_max_near_misses_per_fault(2), 6u);
  // Applying the same bound again has nothing left to drop.
  EXPECT_EQ(storage_->set_max_near_misses_per_fault(2), 0u);
}

TEST_F(SqliteFaultStorageTest, PassedReportOnUnknownFaultWritesNothing) {
  // A heal heartbeat for a fault that does not exist must stay a read: it writes no row and must
  // not take the writer lock on the way to finding that out.
  EXPECT_FALSE(storage_->report_fault_event("NEVER_REPORTED", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN,
                                            "", "/test_node", nth_report_time(0), default_config()));
  EXPECT_EQ(storage_->size(), 0u);
  EXPECT_TRUE(storage_->get_near_misses("NEVER_REPORTED").empty());
}

TEST_F(SqliteFaultStorageTest, NearMissSeriesEmptyForUnknownFault) {
  EXPECT_TRUE(storage_->get_near_misses("NEVER_REPORTED").empty());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
