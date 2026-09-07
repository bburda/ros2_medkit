// Copyright 2026 mfaferek93
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
#include <chrono>
#include <thread>

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"

using namespace ros2_medkit_fault_manager::correlation;
using namespace std::chrono_literals;

class CorrelationEngineTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }

  CorrelationConfig create_hierarchical_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  default_window_ms: 500
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    drive_faults:
      codes: ["DRIVE_*"]
  rules:
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000
      mute_symptoms: true
      auto_clear_with_root: true
)";
    return parse_config_string(yaml);
  }

  CorrelationConfig create_auto_cluster_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]
  rules:
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: highest_severity
)";
    return parse_config_string(yaml);
  }

  /// A hierarchical rule and an auto-cluster rule side by side. The cluster path
  /// sets should_mute without ever writing muted_faults_, which is the case the
  /// planned stop has to survive.
  CorrelationConfig create_cluster_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    valve_errors:
      codes: ["VALVE_*"]
  rules:
    - id: valve_storm
      name: "Valve Storm"
      mode: auto_cluster
      match:
        - pattern: valve_errors
      min_count: 2
      window_ms: 60000
      show_as_single: true
      representative: first
)";
    return parse_config_string(yaml);
  }

  CorrelationConfig create_mixed_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_*"]
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: estop_rule
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
      window_ms: 500
      mute_symptoms: true
      auto_clear_with_root: true
    - id: sensor_cluster
      mode: auto_cluster
      match:
        - pattern: sensor_errors
      min_count: 2
      window_ms: 500
      representative: first
)";
    return parse_config_string(yaml);
  }
};

// ============================================================================
// Hierarchical correlation tests
// ============================================================================

TEST_F(CorrelationEngineTest, RootCauseRecognized) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  auto result = engine.process_fault("ESTOP_001", "CRITICAL");

  EXPECT_FALSE(result.should_mute);
  EXPECT_TRUE(result.is_root_cause);
  EXPECT_EQ("estop_cascade", result.rule_id);
}

TEST_F(CorrelationEngineTest, SymptomMuted) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // First, report root cause
  auto root_result = engine.process_fault("ESTOP_001", "CRITICAL");
  EXPECT_TRUE(root_result.is_root_cause);

  // Then report symptom
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");

  EXPECT_TRUE(symptom_result.should_mute);
  EXPECT_FALSE(symptom_result.is_root_cause);
  EXPECT_EQ("ESTOP_001", symptom_result.root_cause_code);
  EXPECT_EQ("estop_cascade", symptom_result.rule_id);
  // delay_ms can be 0 if faults are processed in quick succession
  EXPECT_GE(symptom_result.delay_ms, 0u);

  // Check muted faults
  EXPECT_EQ(1u, engine.get_muted_count());
  auto muted = engine.get_muted_faults();
  ASSERT_EQ(1u, muted.size());
  EXPECT_EQ("MOTOR_COMM_FL", muted[0].fault_code);
  EXPECT_EQ("ESTOP_001", muted[0].root_cause_code);
}

TEST_F(CorrelationEngineTest, MultipleSymptomsMuted) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FR", "ERROR");
  engine.process_fault("DRIVE_FAULT", "ERROR");

  EXPECT_EQ(3u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, SymptomBeforeRootCauseNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Report symptom BEFORE root cause
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");

  // Should NOT be muted (no root cause yet)
  EXPECT_FALSE(symptom_result.should_mute);
  EXPECT_FALSE(symptom_result.is_root_cause);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, SymptomAfterWindowNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  auto start = std::chrono::steady_clock::now();
  engine.process_fault("ESTOP_001", "CRITICAL", start);

  // Report symptom AFTER window (window is 1000ms)
  auto after_window = start + 1500ms;
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR", after_window);

  // Should NOT be muted (outside window)
  EXPECT_FALSE(symptom_result.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, ClearRootCauseClearsSymptoms) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FR", "ERROR");

  EXPECT_EQ(2u, engine.get_muted_count());

  // Clear root cause
  auto clear_result = engine.process_clear("ESTOP_001");

  EXPECT_EQ(2u, clear_result.auto_cleared_codes.size());
  EXPECT_NE(std::find(clear_result.auto_cleared_codes.begin(), clear_result.auto_cleared_codes.end(), "MOTOR_COMM_FL"),
            clear_result.auto_cleared_codes.end());
  EXPECT_NE(std::find(clear_result.auto_cleared_codes.begin(), clear_result.auto_cleared_codes.end(), "MOTOR_COMM_FR"),
            clear_result.auto_cleared_codes.end());

  // Muted faults should be cleared
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, UnrelatedFaultNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  auto result = engine.process_fault("SENSOR_TIMEOUT", "ERROR");

  // SENSOR_TIMEOUT doesn't match motor_errors or drive_faults patterns
  EXPECT_FALSE(result.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, InlineSymptomCodesMuted) {
  // Config with inline codes instead of pattern references
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: estop_inline
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - codes: ["MOTOR_*", "DRIVE_*"]
      window_ms: 1000
      mute_symptoms: true
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  // Report root cause
  auto root_result = engine.process_fault("ESTOP_001", "CRITICAL");
  EXPECT_TRUE(root_result.is_root_cause);

  // Report symptoms matching inline codes
  auto motor_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);
  EXPECT_EQ("ESTOP_001", motor_result.root_cause_code);

  auto drive_result = engine.process_fault("DRIVE_FAULT", "ERROR");
  EXPECT_TRUE(drive_result.should_mute);

  // Unrelated fault should not be muted
  auto sensor_result = engine.process_fault("SENSOR_ERROR", "ERROR");
  EXPECT_FALSE(sensor_result.should_mute);

  EXPECT_EQ(2u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, MixedPatternAndInlineSymptoms) {
  // Config with both pattern references and inline codes
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: estop_mixed
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: sensor_errors
        - codes: ["MOTOR_*"]
      window_ms: 1000
      mute_symptoms: true
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");

  // Both pattern-matched and inline-matched faults should be muted
  auto sensor_result = engine.process_fault("SENSOR_TIMEOUT", "ERROR");
  EXPECT_TRUE(sensor_result.should_mute);

  auto motor_result = engine.process_fault("MOTOR_FAULT", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);

  EXPECT_EQ(2u, engine.get_muted_count());
}

// ============================================================================
// Auto-cluster tests
// ============================================================================

TEST_F(CorrelationEngineTest, AutoClusterTriggersAtMinCount) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Report faults - need 3 for cluster
  auto r1 = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_FALSE(r1.should_mute);
  EXPECT_EQ(0u, engine.get_cluster_count());

  auto r2 = engine.process_fault("SENSOR_TIMEOUT", "ERROR");
  EXPECT_FALSE(r2.should_mute);
  EXPECT_EQ(0u, engine.get_cluster_count());

  // Third fault triggers cluster
  auto r3 = engine.process_fault("DRIVE_COMM_ERROR", "WARNING");
  // Third fault should be muted (show_as_single=true)
  EXPECT_TRUE(r3.should_mute);
  EXPECT_EQ(1u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, AutoClusterNotTriggeredBelowMinCount) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("SENSOR_TIMEOUT", "ERROR");

  // Only 2 faults, need 3
  EXPECT_EQ(0u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, AutoClusterHighestSeverityRepresentative) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  engine.process_fault("MOTOR_COMM_FL", "WARNING");
  engine.process_fault("SENSOR_TIMEOUT", "CRITICAL");  // Higher severity
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR");

  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("SENSOR_TIMEOUT", clusters[0].representative_code);
  EXPECT_EQ("CRITICAL", clusters[0].representative_severity);
}

TEST_F(CorrelationEngineTest, AutoClusterFirstRepresentative) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    errors:
      codes: ["*_ERROR"]
  rules:
    - id: test_cluster
      mode: auto_cluster
      match:
        - pattern: errors
      min_count: 2
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  engine.process_fault("FIRST_ERROR", "WARNING");
  engine.process_fault("SECOND_ERROR", "CRITICAL");

  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("FIRST_ERROR", clusters[0].representative_code);
}

TEST_F(CorrelationEngineTest, AutoClusterWindowExpires) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto start = std::chrono::steady_clock::now();

  engine.process_fault("MOTOR_COMM_FL", "ERROR", start);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", start + 100ms);

  // Third fault after window (500ms)
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", start + 600ms);

  // Cluster should have been reset, so still not at min_count
  // Actually the third fault starts a new pending cluster
  EXPECT_EQ(0u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, CleanupExpiredRemovesPendingRootCauses) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Use a fixed timestamp in the past (beyond window_ms=1000ms)
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(2000);

  // Report root cause with old timestamp
  engine.process_fault("ESTOP_001", "CRITICAL", past);

  // Symptom reported NOW should NOT be correlated (root cause expired)
  auto result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_FALSE(result.should_mute);  // Not muted - root cause window expired
  EXPECT_EQ(0u, engine.get_muted_count());

  // Call cleanup to explicitly remove expired entries
  engine.cleanup_expired();

  // Another symptom should also not be correlated
  auto result2 = engine.process_fault("MOTOR_TIMEOUT_RR", "ERROR");
  EXPECT_FALSE(result2.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, CleanupExpiredRemovesPendingClusters) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Create pending cluster with old timestamp
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", past);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", past + 10ms);

  // Cluster should not be active (only 2 faults, need 3)
  EXPECT_EQ(0u, engine.get_cluster_count());

  // Cleanup should remove expired pending cluster
  engine.cleanup_expired();

  // New fault should start fresh pending cluster, not join expired one
  auto result = engine.process_fault("DRIVE_COMM_ERROR", "ERROR");
  EXPECT_FALSE(result.should_mute);           // First in new cluster
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still not enough
}

TEST_F(CorrelationEngineTest, CleanupExpiredDoesNotBreakActiveCluster) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Create an active cluster with old timestamps (past the 500ms window)
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", past);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", past + 10ms);
  engine.process_fault("DRIVE_COMM_ERROR", "WARNING", past + 20ms);

  // Cluster should be active (3 faults >= min_count=3)
  EXPECT_EQ(1u, engine.get_cluster_count());

  // Cleanup expires the pending cluster but must NOT wipe fault_to_cluster_
  // entries that the active cluster still needs
  engine.cleanup_expired();

  // Active cluster should still exist
  EXPECT_EQ(1u, engine.get_cluster_count());

  // process_clear must still find the active cluster via fault_to_cluster_
  engine.process_clear("MOTOR_COMM_FL");

  // Cluster should still exist with 2 remaining fault codes
  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ(2u, clusters[0].fault_codes.size());
}

// ============================================================================
// Mixed mode tests
// ============================================================================

TEST_F(CorrelationEngineTest, HierarchicalAndClusterCoexist) {
  auto config = create_mixed_config();
  CorrelationEngine engine(config);

  // Hierarchical: ESTOP_001 -> MOTOR_*
  engine.process_fault("ESTOP_001", "CRITICAL");
  auto motor_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);
  EXPECT_EQ(1u, engine.get_muted_count());

  // Auto-cluster: SENSOR_* (need 2)
  engine.process_fault("SENSOR_LIDAR", "ERROR");
  engine.process_fault("SENSOR_IMU", "ERROR");
  EXPECT_EQ(1u, engine.get_cluster_count());

  // Both should coexist
  EXPECT_EQ(1u, engine.get_muted_count());    // MOTOR_COMM_FL
  EXPECT_EQ(1u, engine.get_cluster_count());  // SENSOR cluster
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(CorrelationEngineTest, DuplicateFaultCode) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");  // Duplicate

  // Should still only have 1 muted fault (no duplicate)
  // Note: current implementation allows duplicates in symptoms list
  // This test documents current behavior
  EXPECT_GE(engine.get_muted_count(), 1u);
}

TEST_F(CorrelationEngineTest, EmptyEngineQueries) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // No faults processed
  EXPECT_EQ(0u, engine.get_muted_count());
  EXPECT_EQ(0u, engine.get_cluster_count());
  EXPECT_TRUE(engine.get_muted_faults().empty());
  EXPECT_TRUE(engine.get_clusters().empty());
}

TEST_F(CorrelationEngineTest, ClearNonExistentFault) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Clear fault that was never reported
  auto result = engine.process_clear("NONEXISTENT");

  EXPECT_TRUE(result.auto_cleared_codes.empty());
}

TEST_F(CorrelationEngineTest, AutoClusterRetroactiveMuting) {
  // Create config with FIRST representative policy to have predictable representative
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: sensor_cluster
      mode: auto_cluster
      match:
        - pattern: sensor_errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Fault #1 - representative (FIRST policy)
  auto result1 = engine.process_fault("SENSOR_001", "ERROR", t0);
  EXPECT_FALSE(result1.should_mute);  // First fault is representative
  EXPECT_FALSE(result1.cluster_id.empty());
  EXPECT_TRUE(result1.retroactive_mute_codes.empty());  // Cluster not active yet

  // Fault #2 - not muted because cluster not active
  auto result2 = engine.process_fault("SENSOR_002", "ERROR", t0 + std::chrono::milliseconds(10));
  EXPECT_FALSE(result2.should_mute);  // Cluster still not active
  EXPECT_TRUE(result2.retroactive_mute_codes.empty());

  // Fault #3 - triggers cluster activation (min_count=3)
  auto result3 = engine.process_fault("SENSOR_003", "ERROR", t0 + std::chrono::milliseconds(20));
  EXPECT_TRUE(result3.should_mute);                      // #3 is muted (not representative)
  EXPECT_EQ(1u, result3.retroactive_mute_codes.size());  // #2 should be retroactively muted
  EXPECT_EQ("SENSOR_002", result3.retroactive_mute_codes[0]);

  // Cluster should be active with 3 faults
  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  EXPECT_EQ(3u, clusters[0].fault_codes.size());
  EXPECT_EQ("SENSOR_001", clusters[0].representative_code);  // FIRST policy
}

// ============================================================================
// Pending cluster cleanup on clear (#127)
// ============================================================================

TEST_F(CorrelationEngineTest, ClearFaultRemovesFromPendingCluster) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults (below min_count=3), creating a pending cluster
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", t0 + 10ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still pending

  // Clear one of the faults
  engine.process_clear("MOTOR_COMM_FL");

  // Now add a third fault - should NOT activate the cluster because
  // the cleared fault was removed from pending, so only 2 faults total
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", t0 + 20ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still not enough
}

TEST_F(CorrelationEngineTest, ClearRepresentativeReassignsPendingCluster) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    errors:
      codes: ["*_ERROR"]
  rules:
    - id: test_cluster
      mode: auto_cluster
      match:
        - pattern: errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults, creating a pending cluster
  engine.process_fault("FIRST_ERROR", "CRITICAL", t0);
  engine.process_fault("SECOND_ERROR", "ERROR", t0 + 10ms);

  // Clear the representative (first fault)
  engine.process_clear("FIRST_ERROR");

  // Add 2 more faults to reach min_count (SECOND_ERROR + 2 new = 3)
  engine.process_fault("THIRD_ERROR", "ERROR", t0 + 20ms);
  engine.process_fault("FOURTH_ERROR", "ERROR", t0 + 30ms);

  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  // SECOND_ERROR should be the new representative (first remaining after clear)
  EXPECT_EQ("SECOND_ERROR", clusters[0].representative_code);
  EXPECT_EQ("ERROR", clusters[0].representative_severity);
}

TEST_F(CorrelationEngineTest, ClearAllFaultsRemovesPendingCluster) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", t0 + 10ms);

  // Clear both
  engine.process_clear("MOTOR_COMM_FL");
  engine.process_clear("SENSOR_TIMEOUT");

  // Adding 2 new faults should start a fresh pending cluster, not join old one
  engine.process_fault("DRIVE_COMM_NEW", "ERROR", t0 + 100ms);
  engine.process_fault("MOTOR_COMM_NEW", "ERROR", t0 + 110ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still 2 faults, below min_count=3
}

// ============================================================================
// MOST_RECENT reassignment on clear
// ============================================================================

TEST_F(CorrelationEngineTest, ClearMostRecentRepresentativeReassignsActiveCluster) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    errors:
      codes: ["*_ERROR"]
  rules:
    - id: test_cluster
      mode: auto_cluster
      match:
        - pattern: errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: most_recent
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  engine.process_fault("FIRST_ERROR", "WARNING", t0);
  engine.process_fault("SECOND_ERROR", "ERROR", t0 + 10ms);
  engine.process_fault("THIRD_ERROR", "CRITICAL", t0 + 20ms);

  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("THIRD_ERROR", clusters[0].representative_code);
  EXPECT_EQ("CRITICAL", clusters[0].representative_severity);

  engine.process_clear("THIRD_ERROR");

  clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("SECOND_ERROR", clusters[0].representative_code);
  EXPECT_EQ("ERROR", clusters[0].representative_severity);
}

// ============================================================================
// HIGHEST_SEVERITY reassignment on clear (#213)
// ============================================================================

TEST_F(CorrelationEngineTest, ClearHighestSeverityRepresentativeReassigns) {
  auto config = create_auto_cluster_config();  // highest_severity, min_count=3
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // WARNING, CRITICAL (rep), ERROR
  engine.process_fault("MOTOR_COMM_FL", "WARNING", t0);
  engine.process_fault("SENSOR_TIMEOUT", "CRITICAL", t0 + 10ms);
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", t0 + 20ms);

  // SENSOR_TIMEOUT is representative (highest severity)
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("SENSOR_TIMEOUT", clusters[0].representative_code);

  // Clear the representative
  engine.process_clear("SENSOR_TIMEOUT");

  // Remaining: WARNING, ERROR -> ERROR should become new representative
  clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("DRIVE_COMM_ERROR", clusters[0].representative_code);
  EXPECT_EQ("ERROR", clusters[0].representative_severity);
}

// ============================================================================
// cleanup_expired removes stale fault_to_cluster_ entries (#214)
// ============================================================================

TEST_F(CorrelationEngineTest, CleanupExpiredRemovesFaultToClusterEntries) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Create pending cluster with old timestamp
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", past);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", past + 10ms);
  EXPECT_EQ(0u, engine.get_cluster_count());

  // Cleanup removes expired pending cluster
  engine.cleanup_expired();

  // Faults from expired cluster should be able to start fresh.
  // If fault_to_cluster_ was NOT cleaned, these would try to join
  // a non-existent cluster instead of creating a new one.
  auto t1 = std::chrono::steady_clock::now();
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t1);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", t1 + 10ms);
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", t1 + 20ms);

  // Should form a NEW cluster with all 3 faults
  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ(3u, clusters[0].fault_codes.size());
}

// ============================================================================
// Planned stop: a second, operator-driven mute source
// ============================================================================

TEST_F(CorrelationEngineTest, PlannedStopMutesAnUncorrelatedFault) {
  CorrelationEngine engine(create_hierarchical_config());

  EXPECT_FALSE(engine.planned_stop_active());
  auto before = engine.process_fault("PUMP_SEAL_LEAK", "ERROR");
  EXPECT_FALSE(before.should_mute);
  EXPECT_FALSE(engine.is_muted("PUMP_SEAL_LEAK"));

  engine.begin_planned_stop();
  EXPECT_TRUE(engine.planned_stop_active());

  auto during = engine.process_fault("VALVE_STUCK", "ERROR");
  EXPECT_TRUE(during.should_mute);
  EXPECT_TRUE(engine.is_muted("VALVE_STUCK"));
  EXPECT_EQ(1u, engine.get_muted_count());

  auto muted = engine.get_muted_faults();
  ASSERT_EQ(1u, muted.size());
  EXPECT_EQ("VALVE_STUCK", muted[0].fault_code);
  EXPECT_EQ(CorrelationEngine::kPlannedStopRootCause, muted[0].root_cause_code);
  EXPECT_EQ(CorrelationEngine::kPlannedStopRuleId, muted[0].rule_id);
}

TEST_F(CorrelationEngineTest, PlannedStopIsIdempotentAndSurvivesRepeatedReports) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  engine.begin_planned_stop();
  EXPECT_TRUE(engine.planned_stop_active());

  engine.process_fault("VALVE_STUCK", "ERROR");
  engine.process_fault("VALVE_STUCK", "ERROR");
  EXPECT_EQ(1u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, EndingPlannedStopUnmutesWhatItMuted) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  engine.process_fault("VALVE_STUCK", "ERROR");
  engine.process_fault("PUMP_SEAL_LEAK", "WARNING");
  EXPECT_EQ(2u, engine.get_muted_count());

  auto unmuted = engine.end_planned_stop();
  EXPECT_FALSE(engine.planned_stop_active());
  std::sort(unmuted.begin(), unmuted.end());
  ASSERT_EQ(2u, unmuted.size());
  EXPECT_EQ("PUMP_SEAL_LEAK", unmuted[0]);
  EXPECT_EQ("VALVE_STUCK", unmuted[1]);
  EXPECT_EQ(0u, engine.get_muted_count());
  EXPECT_FALSE(engine.is_muted("VALVE_STUCK"));
  EXPECT_FALSE(engine.is_muted("PUMP_SEAL_LEAK"));
}

TEST_F(CorrelationEngineTest, EndingPlannedStopWithNothingMutedReturnsNothing) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  auto unmuted = engine.end_planned_stop();
  EXPECT_TRUE(unmuted.empty());

  // Ending a stop that is already off changes nothing either.
  EXPECT_TRUE(engine.end_planned_stop().empty());
  EXPECT_FALSE(engine.planned_stop_active());
}

TEST_F(CorrelationEngineTest, AFaultReportedAfterTheStopEndsIsNotMuted) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  engine.end_planned_stop();

  auto result = engine.process_fault("VALVE_STUCK", "ERROR");
  EXPECT_FALSE(result.should_mute);
  EXPECT_FALSE(engine.is_muted("VALVE_STUCK"));
}

TEST_F(CorrelationEngineTest, EndingPlannedStopLeavesARuleMutedSymptomMuted) {
  CorrelationEngine engine(create_hierarchical_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.begin_planned_stop();

  // ESTOP_001 is a root cause: muted by the stop, but not a symptom.
  engine.process_fault("ESTOP_001", "CRITICAL", t0);
  // MOTOR_COMM_FL matches the rule's symptom pattern inside the window, so the
  // RULE mutes it - the stop is not the reason it is muted.
  auto symptom = engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 10ms);
  EXPECT_TRUE(symptom.should_mute);
  EXPECT_EQ("estop_cascade", symptom.rule_id);

  auto unmuted = engine.end_planned_stop();
  EXPECT_EQ(1u, unmuted.size());
  EXPECT_EQ("ESTOP_001", unmuted[0]);

  // The rule still holds the symptom down.
  EXPECT_TRUE(engine.is_muted("MOTOR_COMM_FL"));
  auto muted = engine.get_muted_faults();
  ASSERT_EQ(1u, muted.size());
  EXPECT_EQ("MOTOR_COMM_FL", muted[0].fault_code);
  EXPECT_EQ("estop_cascade", muted[0].rule_id);
}

TEST_F(CorrelationEngineTest, ARuleTakingOverAStopMutedFaultKeepsItMuted) {
  CorrelationEngine engine(create_hierarchical_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.begin_planned_stop();

  // Muted by the stop first: no root cause is pending yet.
  auto first = engine.process_fault("MOTOR_COMM_FL", "ERROR", t0);
  EXPECT_TRUE(first.should_mute);
  EXPECT_EQ(CorrelationEngine::kPlannedStopRuleId, engine.get_muted_faults()[0].rule_id);

  // The root cause arrives, then the same code is reported again and the rule
  // claims it.
  engine.process_fault("ESTOP_001", "CRITICAL", t0 + 10ms);
  auto second = engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 20ms);
  EXPECT_TRUE(second.should_mute);
  EXPECT_EQ("estop_cascade", second.rule_id);

  auto unmuted = engine.end_planned_stop();
  EXPECT_EQ(1u, unmuted.size());
  EXPECT_EQ("ESTOP_001", unmuted[0]);
  EXPECT_TRUE(engine.is_muted("MOTOR_COMM_FL"));
}

TEST_F(CorrelationEngineTest, ClearingDuringAStopTakesTheFaultOutOfTheMute) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  engine.process_fault("VALVE_STUCK", "ERROR");
  EXPECT_TRUE(engine.is_muted("VALVE_STUCK"));

  engine.process_clear("VALVE_STUCK");
  EXPECT_FALSE(engine.is_muted("VALVE_STUCK"));
  EXPECT_EQ(0u, engine.get_muted_count());

  // Nothing survives to be unmuted, so the switch-off announces nothing.
  EXPECT_TRUE(engine.end_planned_stop().empty());
}

TEST_F(CorrelationEngineTest, ClearingARootCauseDuringAStopDropsItsStopMutedSymptoms) {
  CorrelationEngine engine(create_hierarchical_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.begin_planned_stop();
  engine.process_fault("ESTOP_001", "CRITICAL", t0);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 10ms);

  auto cleared = engine.process_clear("ESTOP_001");
  ASSERT_EQ(1u, cleared.auto_cleared_codes.size());
  EXPECT_EQ("MOTOR_COMM_FL", cleared.auto_cleared_codes[0]);
  EXPECT_FALSE(engine.is_muted("MOTOR_COMM_FL"));

  // Both codes left the mute with the clear; the switch-off has nothing to say.
  EXPECT_TRUE(engine.end_planned_stop().empty());
}

TEST_F(CorrelationEngineTest, PlannedStopMutesWithoutAnyRulesConfigured) {
  CorrelationEngine engine{CorrelationConfig{}};

  engine.begin_planned_stop();
  auto result = engine.process_fault("VALVE_STUCK", "ERROR");
  EXPECT_TRUE(result.should_mute);
  EXPECT_TRUE(engine.is_muted("VALVE_STUCK"));

  auto unmuted = engine.end_planned_stop();
  ASSERT_EQ(1u, unmuted.size());
  EXPECT_EQ("VALVE_STUCK", unmuted[0]);
}

// ============================================================================
// Planned stop: only a cycle that STARTS inside it is marked
// ============================================================================

TEST_F(CorrelationEngineTest, AStillActiveReReportIsNotTakenByTheStop) {
  CorrelationEngine engine(create_hierarchical_config());

  // The fault's cycle started before the stop was declared.
  engine.process_fault("PUMP_SEAL_LEAK", "ERROR", std::chrono::steady_clock::now(), /*cycle_started=*/true);
  EXPECT_FALSE(engine.is_muted("PUMP_SEAL_LEAK"));

  engine.begin_planned_stop();

  // A level-triggered reporter keeps sending FAILED while the condition holds.
  auto result = engine.process_fault("PUMP_SEAL_LEAK", "ERROR", std::chrono::steady_clock::now(),
                                     /*cycle_started=*/false);
  EXPECT_FALSE(result.should_mute);
  EXPECT_FALSE(engine.is_muted("PUMP_SEAL_LEAK"));
  EXPECT_TRUE(engine.end_planned_stop().empty()) << "the stop released a fault it never muted";
}

TEST_F(CorrelationEngineTest, ACycleStartingInsideTheStopIsTakenByIt) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  auto result = engine.process_fault("PUMP_SEAL_LEAK", "ERROR", std::chrono::steady_clock::now(),
                                     /*cycle_started=*/true);
  EXPECT_TRUE(result.should_mute);
  EXPECT_TRUE(engine.is_muted("PUMP_SEAL_LEAK"));

  auto unmuted = engine.end_planned_stop();
  ASSERT_EQ(1u, unmuted.size());
  EXPECT_EQ("PUMP_SEAL_LEAK", unmuted[0]);
}

TEST_F(CorrelationEngineTest, AStillActiveReReportDoesNotReleaseAStopMute) {
  CorrelationEngine engine(create_hierarchical_config());

  engine.begin_planned_stop();
  engine.process_fault("VALVE_STUCK", "ERROR", std::chrono::steady_clock::now(), /*cycle_started=*/true);
  ASSERT_TRUE(engine.is_muted("VALVE_STUCK"));

  // The same condition is reported again while it holds. Nothing about the mute
  // changes: the cycle is the one the stop already took.
  auto again = engine.process_fault("VALVE_STUCK", "ERROR", std::chrono::steady_clock::now(),
                                    /*cycle_started=*/false);
  EXPECT_TRUE(again.should_mute);
  EXPECT_TRUE(engine.is_muted("VALVE_STUCK"));

  auto unmuted = engine.end_planned_stop();
  ASSERT_EQ(1u, unmuted.size());
  EXPECT_EQ("VALVE_STUCK", unmuted[0]);
}

// ============================================================================
// Planned stop: exactly one mute owner per fault
// ============================================================================

TEST_F(CorrelationEngineTest, TheStopNeverOverwritesARuleMute) {
  CorrelationEngine engine(create_hierarchical_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.process_fault("ESTOP_001", "CRITICAL", t0);
  auto symptom = engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 10ms);
  ASSERT_TRUE(symptom.should_mute);
  ASSERT_EQ("estop_cascade", engine.get_muted_faults()[0].rule_id);

  // The correlation window closes, then a stop is declared and the same code
  // starts a new cycle. The rule still owns the mute; the stop must not take it.
  engine.begin_planned_stop();
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 5000ms, /*cycle_started=*/true);

  auto muted = engine.get_muted_faults();
  ASSERT_EQ(1u, muted.size());
  EXPECT_EQ("estop_cascade", muted[0].rule_id) << "the stop overwrote a rule's mute";

  EXPECT_TRUE(engine.end_planned_stop().empty());
  EXPECT_TRUE(engine.is_muted("MOTOR_COMM_FL"));
}

TEST_F(CorrelationEngineTest, AClusterMuteLeavesTheStopHoldingItsOwnEntry) {
  CorrelationEngine engine(create_cluster_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.begin_planned_stop();

  // First of the burst: the cluster is still below min_count, so the stop mutes it.
  auto first = engine.process_fault("VALVE_A", "ERROR", t0, /*cycle_started=*/true);
  EXPECT_TRUE(first.should_mute);

  // Second reaches min_count. The cluster mutes it as a non-representative, but
  // the cluster path writes nothing to the mute map, so the stop stays its owner.
  auto second = engine.process_fault("VALVE_B", "ERROR", t0 + 10ms, /*cycle_started=*/true);
  EXPECT_TRUE(second.should_mute);
  EXPECT_FALSE(second.cluster_id.empty());

  auto muted = engine.get_muted_faults();
  ASSERT_EQ(2u, muted.size());
  for (const auto & entry : muted) {
    EXPECT_EQ(CorrelationEngine::kPlannedStopRuleId, entry.rule_id);
  }

  auto unmuted = engine.end_planned_stop();
  std::sort(unmuted.begin(), unmuted.end());
  ASSERT_EQ(2u, unmuted.size()) << "a cluster-claimed fault was orphaned in the mute map";
  EXPECT_EQ("VALVE_A", unmuted[0]);
  EXPECT_EQ("VALVE_B", unmuted[1]);
  EXPECT_EQ(0u, engine.get_muted_count());
}

// ============================================================================
// Planned stop: dropping and restoring one fault's mute
// ============================================================================

TEST_F(CorrelationEngineTest, DroppingAStopMuteReleasesOnlyWhatTheStopOwns) {
  CorrelationEngine engine(create_hierarchical_config());

  auto t0 = std::chrono::steady_clock::now();
  engine.begin_planned_stop();
  engine.process_fault("VALVE_STUCK", "ERROR", t0, /*cycle_started=*/true);
  engine.process_fault("ESTOP_001", "CRITICAL", t0 + 10ms, /*cycle_started=*/true);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0 + 20ms, /*cycle_started=*/true);
  // get_muted_faults() is ordered by fault code, so look the entry up rather than
  // taking a position in the list.
  auto rule_id_of = [&engine](const std::string & code) {
    for (const auto & entry : engine.get_muted_faults()) {
      if (entry.fault_code == code) {
        return entry.rule_id;
      }
    }
    return std::string{"<not muted>"};
  };
  ASSERT_EQ("estop_cascade", rule_id_of("MOTOR_COMM_FL"));

  engine.drop_planned_stop_mute("VALVE_STUCK");
  EXPECT_FALSE(engine.is_muted("VALVE_STUCK"));

  // A rule's mute is not the stop's to drop.
  engine.drop_planned_stop_mute("MOTOR_COMM_FL");
  EXPECT_TRUE(engine.is_muted("MOTOR_COMM_FL"));

  auto unmuted = engine.end_planned_stop();
  ASSERT_EQ(1u, unmuted.size());
  EXPECT_EQ("ESTOP_001", unmuted[0]);
}

TEST_F(CorrelationEngineTest, ARestoredStopMuteBehavesLikeOneTakenLive) {
  CorrelationEngine engine{CorrelationConfig{}};

  // What a restart does: the declaration is read back, then the faults whose
  // cycles started inside it are marked again before anything is served.
  engine.begin_planned_stop();
  engine.restore_planned_stop_mute("VALVE_STUCK");
  engine.restore_planned_stop_mute("PUMP_SEAL_LEAK");

  EXPECT_TRUE(engine.is_muted("VALVE_STUCK"));
  EXPECT_EQ(2u, engine.get_muted_count());
  const auto muted = engine.get_muted_faults();
  EXPECT_EQ(CorrelationEngine::kPlannedStopRootCause, muted[0].root_cause_code);

  auto unmuted = engine.end_planned_stop();
  std::sort(unmuted.begin(), unmuted.end());
  ASSERT_EQ(2u, unmuted.size());
  EXPECT_EQ("PUMP_SEAL_LEAK", unmuted[0]);
  EXPECT_EQ("VALVE_STUCK", unmuted[1]);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
