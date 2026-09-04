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

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/capture_thread_pool.hpp"
#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"
#include "ros2_medkit_fault_manager/entity_threshold_resolver.hpp"
#include "ros2_medkit_fault_manager/fault_audit_log.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/rosbag_capture.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"
#include "ros2_medkit_msgs/srv/list_faults_for_entity.hpp"
#include "ros2_medkit_msgs/srv/list_rosbags.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_manager {

/// Central fault manager node
///
/// Provides service interfaces for fault reporting, querying, and clearing.
/// Supports configurable storage backends (memory or SQLite) via ROS parameters.
///
/// Parameters:
/// - storage_type (string): "memory" or "sqlite" (default: "sqlite")
/// - database_path (string): Path to SQLite database file (default: "/var/lib/ros2_medkit/faults.db")
///   Use ":memory:" for in-memory SQLite database (useful for testing)
class FaultManagerNode : public rclcpp::Node {
 public:
  explicit FaultManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FaultManagerNode() override;
  FaultManagerNode(const FaultManagerNode &) = delete;
  FaultManagerNode & operator=(const FaultManagerNode &) = delete;
  FaultManagerNode(FaultManagerNode &&) = delete;
  FaultManagerNode & operator=(FaultManagerNode &&) = delete;

  /// Get read-only access to fault storage (for testing)
  const FaultStorage & get_storage() const {
    return *storage_;
  }

  /// Get mutable access to fault storage (for testing only).
  FaultStorage & get_storage_for_test() {
    return *storage_;
  }

  /// Get the tamper-evident audit log (nullptr when disabled), for testing only.
  const FaultAuditLog * get_audit_log_for_test() const {
    return audit_log_.get();
  }

  /// Health signal: number of audit transitions that failed to append (and were
  /// therefore lost from the chain). 0 when the audit is healthy or disabled.
  /// IN-PROCESS ONLY: this is a C++ getter, not exposed over any service/REST/
  /// health surface yet, so an operator cannot read it at runtime in this
  /// revision (a runtime read/verify/health surface is future work).
  uint64_t audit_dropped_writes() const {
    return audit_dropped_writes_.load(std::memory_order_relaxed);
  }

  /// Health signal: false once any audit append has failed. A compliance-strict
  /// deployment should treat a false here (or a non-zero audit_dropped_writes())
  /// as the audit chain being incomplete. IN-PROCESS ONLY (see above): not yet
  /// observable at runtime by an operator.
  bool audit_healthy() const {
    return audit_healthy_.load(std::memory_order_relaxed);
  }

  /// Test-only: drive one audit append through the same failure-handling path the
  /// service handlers use. Throws when audit_log.fail_closed is set and the append
  /// fails; otherwise observe audit_dropped_writes()/audit_healthy().
  void audit_transition_for_test(const char * transition, const ros2_medkit_msgs::msg::Fault & fault) {
    audit_transition(transition, fault, "test", 0);
  }

  /// Get the storage type being used
  const std::string & get_storage_type() const {
    return storage_type_;
  }

  int capture_pool_size_for_test() const {
    return capture_pool_size_;
  }
  int capture_queue_depth_for_test() const {
    return capture_queue_depth_;
  }
  QueueFullPolicy capture_queue_full_policy_for_test() const {
    return capture_queue_full_policy_;
  }

  /// Check if entity matches any reporting source
  /// @param reporting_sources List of reporting sources from fault
  /// @param entity_id Entity ID to match (exact match or as suffix of FQN)
  /// @return true if entity_id matches any source
  static bool matches_entity(const std::vector<std::string> & reporting_sources, const std::string & entity_id);

 private:
  /// Create storage backend based on configuration
  std::unique_ptr<FaultStorage> create_storage();

  /// Handle ReportFault service request
  void handle_report_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                           const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response);

  /// Handle ListFaults service request
  void handle_list_faults(const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
                          const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response);

  /// Handle GetFault service request (single fault with environment_data)
  void handle_get_fault(const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                        const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response);

  /// Handle ClearFault service request
  void handle_clear_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                          const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response);

  /// Handle GetSnapshots service request
  void handle_get_snapshots(const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
                            const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response);

  /// Handle GetRosbag service request
  void handle_get_rosbag(const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                         const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response);

  /// Handle ListRosbags batch service request
  void handle_list_rosbags(const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Request> & request,
                           const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Response> & response);

  /// Handle ListFaultsForEntity service request
  void
  handle_list_faults_for_entity(const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
                                const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response);

  /// Create snapshot configuration from parameters
  SnapshotConfig create_snapshot_config();

  /// Load snapshot configuration from YAML file
  /// @param config_file Path to the YAML configuration file
  /// @param config SnapshotConfig to populate with loaded values
  void load_snapshot_config_from_yaml(const std::string & config_file, SnapshotConfig & config);

  /// Initialize correlation engine from configuration file
  /// @return CorrelationEngine instance if enabled and config is valid, nullptr otherwise
  std::unique_ptr<correlation::CorrelationEngine> create_correlation_engine();

  /// Publish a fault event to the events topic
  /// @param event_type One of FaultEvent::EVENT_CONFIRMED, EVENT_CLEARED, EVENT_UPDATED
  /// @param fault The fault data associated with this event
  /// @param auto_cleared_codes Optional list of auto-cleared symptom fault codes (for EVENT_CLEARED)
  void publish_fault_event(const std::string & event_type, const ros2_medkit_msgs::msg::Fault & fault,
                           const std::vector<std::string> & auto_cleared_codes = {});

  /// Validate severity value
  static bool is_valid_severity(uint8_t severity);

  /// Extract topic name from full topic path (last segment)
  static std::string extract_topic_name(const std::string & topic_path);

  /// Resolve debounce config for a given source_id using entity threshold resolver.
  /// Falls back to global config if no entity-specific overrides match.
  DebounceConfig resolve_config(const std::string & source_id) const;

  /// Create the tamper-evident audit log from parameters (nullptr if disabled).
  std::unique_ptr<FaultAuditLog> create_audit_log();

  /// Append a fault state-transition to the audit log when enabled. No-op when
  /// the log is off, or when only confirmations are logged and this is not one.
  /// @param transition One of the kTransition* constants.
  /// @param fault The fault state at the time of the transition.
  /// @param source_id The reporting source that drove the transition.
  /// @param occurred_at_ns Wall-clock timestamp of the transition.
  void audit_transition(const char * transition, const ros2_medkit_msgs::msg::Fault & fault,
                        const std::string & source_id, int64_t occurred_at_ns);

  std::string storage_type_;
  std::string database_path_;
  std::string database_url_;
  int32_t confirmation_threshold_{-1};
  bool healing_enabled_{false};
  int32_t healing_threshold_{3};
  double auto_confirm_after_sec_{0.0};
  double snapshot_recapture_cooldown_sec_{60.0};
  int capture_pool_size_{2};
  int capture_queue_depth_{16};
  QueueFullPolicy capture_queue_full_policy_{QueueFullPolicy::kRejectNewest};
  DebounceConfig global_config_;  ///< Global debounce config (built from ROS params)
  std::unique_ptr<FaultStorage> storage_;
  std::unique_ptr<EntityThresholdResolver> threshold_resolver_;  ///< Per-entity threshold overrides

  /// Tamper-evident audit log of fault transitions (nullptr when disabled).
  std::unique_ptr<FaultAuditLog> audit_log_;
  bool audit_confirmed_only_{false};  ///< When true, only "confirmed" transitions are logged
  /// When true, an audit append failure is re-raised as a fail-FAST signal that
  /// the chain is broken (operator must act). It does NOT roll back the already-
  /// committed fault-state change (the fault store is a separate DB); it is not
  /// atomicity.
  bool audit_fail_closed_{false};

  std::atomic<uint64_t> audit_dropped_writes_{0};  ///< Count of audit appends that failed (lost rows)
  std::atomic<bool> audit_healthy_{true};          ///< Cleared on the first failed audit append

  rclcpp::Service<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ListFaults>::SharedPtr list_faults_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFault>::SharedPtr get_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetSnapshots>::SharedPtr get_snapshots_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetRosbag>::SharedPtr get_rosbag_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ListRosbags>::SharedPtr list_rosbags_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ListFaultsForEntity>::SharedPtr list_faults_for_entity_srv_;
  rclcpp::TimerBase::SharedPtr auto_confirm_timer_;

  /// Timer for periodic cleanup of expired correlation data
  rclcpp::TimerBase::SharedPtr correlation_cleanup_timer_;

  /// Publisher for fault events (SSE streaming via gateway)
  rclcpp::Publisher<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr event_publisher_;

  /// Snapshot capture for capturing topic data on fault confirmation.
  /// shared_ptr to allow safe capture-by-value in the pool's capture jobs.
  std::shared_ptr<SnapshotCapture> snapshot_capture_;

  /// Rosbag capture for time-window recording (nullptr if disabled).
  /// shared_ptr to allow safe capture-by-value in the pool's capture jobs.
  std::shared_ptr<RosbagCapture> rosbag_capture_;

  /// Correlation engine for fault correlation/muting (nullptr if disabled)
  std::unique_ptr<correlation::CorrelationEngine> correlation_engine_;

  /// Bounded pool that runs capture jobs off the service thread (issue #441).
  /// Declared after snapshot_capture_/rosbag_capture_ so it is destroyed first.
  std::unique_ptr<CaptureThreadPool> capture_pool_;

  /// Per-fault cooldown tracking for snapshot recapture
  std::mutex last_capture_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_capture_times_;
};

}  // namespace ros2_medkit_fault_manager
