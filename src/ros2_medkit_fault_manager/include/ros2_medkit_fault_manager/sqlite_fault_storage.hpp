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

#include <sqlite3.h>

#include <mutex>
#include <string>

#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// SQLite-based fault storage implementation with persistence
/// Thread-safe implementation using mutex protection
class SqliteFaultStorage : public FaultStorage {
 public:
  /// Create SQLite fault storage
  /// @param db_path Path to SQLite database file. Use ":memory:" for in-memory database.
  /// @throws std::runtime_error if database cannot be opened or initialized
  explicit SqliteFaultStorage(const std::string & db_path);

  /// Destructor - closes database connection
  ~SqliteFaultStorage() override;

  // Non-copyable, non-movable (owns SQLite connection)
  SqliteFaultStorage(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage & operator=(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage(SqliteFaultStorage &&) = delete;
  SqliteFaultStorage & operator=(SqliteFaultStorage &&) = delete;

  void set_debounce_config(const DebounceConfig & config) override;
  DebounceConfig get_debounce_config() const override;

  bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                          const std::string & description, const std::string & source_id,
                          const rclcpp::Time & timestamp, const DebounceConfig & config,
                          const std::string & supersedes_source_id = "") override;

  std::vector<ros2_medkit_msgs::msg::Fault> list_faults(bool filter_by_severity, uint8_t severity,
                                                        const std::vector<std::string> & statuses) const override;

  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const override;

  bool clear_fault(const std::string & fault_code) override;

  size_t size() const override;

  bool contains(const std::string & fault_code) const override;

  std::vector<std::string> check_time_based_confirmation(const rclcpp::Time & current_time) override;

  void set_max_snapshots_per_fault(size_t max_count) override;

  void store_snapshot(const SnapshotData & snapshot) override;
  std::vector<SnapshotData> get_snapshots(const std::string & fault_code,
                                          const std::string & topic_filter = "") const override;

  void store_freeze_frame(const FreezeFrameData & frame) override;
  std::optional<FreezeFrameData> get_freeze_frame(const std::string & fault_code) const override;

  void store_rosbag_file(const RosbagFileInfo & info) override;
  std::optional<RosbagFileInfo> get_rosbag_file(const std::string & fault_code) const override;
  bool delete_rosbag_file(const std::string & fault_code) override;
  size_t get_total_rosbag_storage_bytes() const override;
  std::vector<RosbagFileInfo> get_all_rosbag_files() const override;
  std::vector<RosbagFileInfo> list_rosbags_for_entity(const std::string & entity_fqn) const override;
  std::vector<ros2_medkit_msgs::msg::Fault> get_all_faults() const override;
  std::vector<std::string> reclassify_healed_as_cleared() override;

  /// Get the database path
  const std::string & db_path() const {
    return db_path_;
  }

 private:
  /// Initialize database schema
  void initialize_schema();

  /// Parse JSON array string to vector of strings
  static std::vector<std::string> parse_json_array(const std::string & json_str);

  /// Serialize vector of strings to JSON array string
  static std::string serialize_json_array(const std::vector<std::string> & vec);

  std::string db_path_;
  sqlite3 * db_{nullptr};
  mutable std::mutex mutex_;
  DebounceConfig config_;
  size_t max_snapshots_per_fault_{0};  ///< 0 = unlimited
};

}  // namespace ros2_medkit_fault_manager
