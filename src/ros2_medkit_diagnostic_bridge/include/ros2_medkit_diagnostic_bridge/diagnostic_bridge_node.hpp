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

#pragma once

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

namespace ros2_medkit_diagnostic_bridge {

/// Bridge node that converts ROS2 /diagnostics messages to FaultManager faults.
///
/// Subscribes to /diagnostics topic and forwards diagnostic status messages
/// to the FaultManager via the ReportFault service.
///
/// Severity mapping:
///   - OK (0)    -> PASSED event (healing)
///   - WARN (1)  -> WARN severity (1)
///   - ERROR (2) -> ERROR severity (2)
///   - STALE (3) -> CRITICAL severity (3)
///
/// Example launch configuration:
/// @code{.yaml}
/// diagnostic_bridge:
///   ros__parameters:
///     diagnostics_topic: "/diagnostics"
///     auto_generate_codes: true
///     # Custom mappings: "name_to_code.<diagnostic_name>": "<FAULT_CODE>"
///     "name_to_code.motor_controller: Status": "MOTOR_001"
/// @endcode
class DiagnosticBridgeNode : public rclcpp::Node {
 public:
  explicit DiagnosticBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Map diagnostic status to fault code
  /// Mapping order:
  /// 1. Custom name_to_code mapping (parameter overrides)
  /// 2. Check if diagnostic contains any key-value pairs with keys in keyvalue_codes
  /// 3. Auto-generate from diagnostic name (if auto_generate_codes is true)
  /// 4. Return empty string if no mapping found
  std::string map_to_fault_code(const diagnostic_msgs::msg::DiagnosticStatus & status) const;

  /// Map DiagnosticStatus level to Fault severity
  /// Returns std::nullopt if level is OK (should send PASSED instead)
  static std::optional<uint8_t> map_to_severity(uint8_t diagnostic_level);

  /// Check if diagnostic level indicates OK status
  static bool is_ok_level(uint8_t diagnostic_level);

  /// Get (creating on first use) the FaultReporter for a given source_id.
  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & source_id);

  /// Resolve fault source_id for a diagnostic status.
  /// Uses a slash-containing hardware_id when enabled, else falls back to bridge FQN.
  std::string source_id_for(const diagnostic_msgs::msg::DiagnosticStatus & status) const;

  /// Number of currently tracked source reporters.
  size_t tracked_reporter_count();

 private:
  /// Callback for /diagnostics messages
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg);

  /// Process a single DiagnosticStatus
  void process_diagnostic(const diagnostic_msgs::msg::DiagnosticStatus & status);

  /// Auto-generate fault code from diagnostic name
  /// Converts to UPPER_SNAKE_CASE: "motor temp" -> "MOTOR_TEMP"
  static std::string generate_fault_code(const std::string & diagnostic_name);

  /// Load parameters from ROS2 parameter server
  void load_parameters();

  // ROS2 components
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  // One FaultReporter per source_id, bounded with LRU eviction.
  struct ReporterEntry {
    std::string source_id;
    std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter;
  };
  std::list<ReporterEntry> reporters_lru_;
  std::unordered_map<std::string, std::list<ReporterEntry>::iterator> reporters_;
  std::mutex reporters_mutex_;

  // Configuration
  std::string diagnostics_topic_;
  bool auto_generate_codes_;
  bool use_hardware_id_as_source_id_{false};
  int max_tracked_sources_{512};
  std::map<std::string, std::string> name_to_code_;
  std::vector<std::string> keyvalue_codes_;
};

}  // namespace ros2_medkit_diagnostic_bridge
