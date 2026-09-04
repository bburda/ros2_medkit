// Copyright 2026 gstavrinos
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

#include <pqxx/pqxx>

#include <mutex>
#include <optional>
#include <string>

#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// PostgreSQL-based fault storage implementation with persistence
/// Thread-safe implementation using mutex protection on connection access
class PgFaultStorage : public FaultStorage {
 public:
  /// Create PostgreSQL fault storage
  /// @param conn_info Connection string or DSN (e.g.,
  /// "postgresql://user:password@localhost:5432/ros2_medkit_faults_database")
  /// @throws std::runtime_error if database cannot be connected to or initialized
  explicit PgFaultStorage(const std::string & conn_info);

  /// Destructor - closes database connection
  ~PgFaultStorage() override;

  // Non-copyable, non-movable (owns PostgreSQL connection)
  PgFaultStorage(const PgFaultStorage &) = delete;
  PgFaultStorage & operator=(const PgFaultStorage &) = delete;
  PgFaultStorage(PgFaultStorage &&) = delete;
  PgFaultStorage & operator=(PgFaultStorage &&) = delete;

  void set_debounce_config(const DebounceConfig & config) override;
  DebounceConfig get_debounce_config() const override;

  bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                          const std::string & description, const std::string & source_id,
                          const rclcpp::Time & timestamp, const DebounceConfig & config) override;

  std::vector<ros2_medkit_msgs::msg::Fault> list_faults(bool filter_by_severity, uint8_t severity,
                                                        const std::vector<std::string> & statuses) const override;

  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const override;

  bool clear_fault(const std::string & fault_code) override;

  size_t size() const override;

  bool contains(const std::string & fault_code) const override;

  std::vector<std::string> check_time_based_confirmation(const rclcpp::Time & current_time) override;

  void set_max_snapshots_per_fault(size_t max_count) override;
  void set_retain_snapshots_on_clear(bool retain) override;
  bool retains_snapshots_on_clear() const override;

  void set_max_rosbags_per_fault(size_t max_count) override;

  void store_snapshot(const SnapshotData & snapshot) override;
  void store_snapshots(const std::vector<SnapshotData> & snapshots) override;
  std::vector<SnapshotData> get_snapshots(const std::string & fault_code,
                                          const std::string & topic_filter = "") const override;
  int64_t get_max_capture_id() const override;

  void store_freeze_frame(const FreezeFrameData & frame) override;
  std::optional<FreezeFrameData> get_freeze_frame(const std::string & fault_code) const override;
  size_t set_max_near_misses_per_fault(size_t max_count) override;
  std::vector<NearMissRecord> get_near_misses(const std::string & fault_code) const override;

  void store_rosbag_file(const RosbagFileInfo & info) override;
  void store_rosbag_files(const std::vector<RosbagFileInfo> & infos) override;
  std::optional<RosbagFileInfo> get_rosbag_file(const std::string & fault_code) const override;
  std::vector<RosbagFileInfo> get_rosbag_files(const std::string & fault_code) const override;
  std::vector<RosbagFileInfo> get_rosbag_files_by_recording(const std::string & recording_id) const override;
  bool delete_rosbag_file(const std::string & fault_code) override;
  size_t delete_rosbag_recording(const std::string & recording_id) override;
  size_t delete_rosbag_files(const std::vector<std::string> & fault_codes) override;
  size_t get_total_rosbag_storage_bytes() const override;
  std::vector<RosbagFileInfo> get_all_rosbag_files() const override;
  std::vector<RosbagFileInfo> list_rosbags_for_entity(const std::string & entity_fqn) const override;
  std::vector<ros2_medkit_msgs::msg::Fault> get_all_faults() const override;
  std::vector<std::string> reclassify_healed_as_cleared() override;

  /// Get the connection info string used to initialize the database
  const std::string & conn_info() const {
    return conn_info_;
  }

 private:
  /// Initialize database schema (create tables if they don't exist)
  void initialize_schema();

  /// Whether any fault at all still references @p file_path. Caller holds mutex_.
  bool path_referenced(const std::string & file_path) const;

  /// store_rosbag_file body without taking mutex_. Caller holds mutex_ and
  /// manages transaction scope. Returns replaced bag path if applicable.
  std::vector<std::string> store_rosbag_file_locked(const RosbagFileInfo & info, pqxx::work & tx);

  /// report_fault_event body without taking mutex_ or opening a transaction. Caller holds mutex_
  /// and supplies the transaction, so the fault row and any near-miss row commit together.
  bool report_fault_event_locked(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                 const std::string & description, const std::string & source_id,
                                 const rclcpp::Time & timestamp, const DebounceConfig & config, pqxx::work & tx);

  /// Append one entry to the near-miss series and evict the oldest entries beyond
  /// max_near_misses_per_fault_. Caller holds mutex_ and has already written the fault row.
  /// @param fault_code The fault code that nearly confirmed
  /// @param occurred_at_ns Timestamp of the report
  /// @param debounce_counter Counter value after the report
  /// @param config Debounce config the report was evaluated against
  /// @param severity Severity carried by the report
  /// @param source_id Reporting source
  /// @param resulting_status Fault status after the report was applied
  /// @param tx Transaction the fault row was written in
  void record_near_miss_locked(const std::string & fault_code, int64_t occurred_at_ns, int32_t debounce_counter,
                               const DebounceConfig & config, uint8_t severity, const std::string & source_id,
                               const std::string & resulting_status, pqxx::work & tx);

  /// Deserialize JSON array string from PostgreSQL TEXT/JSONB field
  static std::vector<std::string> parse_json_array(const std::string & json_str);

  /// Serialize vector of strings to JSON array string (for PostgreSQL JSONB)
  static std::string serialize_json_array(const std::vector<std::string> & vec);

  std::string conn_info_;
  std::unique_ptr<pqxx::connection> db_conn_;
  mutable std::mutex mutex_;
  DebounceConfig config_;
  size_t max_snapshots_per_fault_{0};    ///< 0 = unlimited
  size_t max_near_misses_per_fault_{0};  ///< 0 = unlimited
  bool retain_snapshots_on_clear_{false};
  /// Defaults to 1, the pre-#620 behaviour: a new recording replaces the old one.
  /// 0 = unlimited, bounded only by max_total_storage_mb.
  size_t max_rosbags_per_fault_{1};
};

}  // namespace ros2_medkit_fault_manager
