// Copyright 2026 mfaferek93, bburda
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
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

namespace ros2_medkit_log_bridge {

/// Bridge node that promotes ROS 2 /rosout log entries to FaultManager faults.
///
/// Subscribes to /rosout (rcl_interfaces/msg/Log) and forwards entries at or
/// above a configurable severity floor to the FaultManager, attributing each
/// fault to the originating node's fully-qualified name via a per-source
/// FaultReporter. Drop-in compat adapter, same category as
/// ros2_medkit_diagnostic_bridge: native FaultReporter instrumentation stays
/// the canonical path; this bridge is the fallback for nodes that only log.
/// Level mapping and the WARN/LocalFilter caveat are documented in README.md.
///
/// Hard limitation by construction: only sees rclcpp logs that reach /rosout
/// from a still-alive node. Console-only loggers and crash-before-flush are out
/// of reach.
class LogBridgeNode : public rclcpp::Node {
 public:
  explicit LogBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Map an rcl_interfaces/msg/Log level to a Fault severity.
  /// Returns false when the level is below the floor / not promotable.
  static bool map_level_to_severity(uint8_t log_level, uint8_t severity_floor, uint8_t * severity_out);

  /// Auto-generate a stable fault code from the originating node's FQN and the
  /// log message. Numbers/hex/paths in the message are normalized away so the
  /// same logical message maps to the same code across occurrences.
  /// Format: <PREFIX>_<NODE>_<HASH>, clamped to medkit's [A-Z0-9_] / 64-char rule.
  std::string generate_fault_code(const std::string & source_id, const std::string & message) const;

  /// Normalize a log message into a stable template (lowercased, digit/hex/path
  /// runs stripped, whitespace collapsed, isolated single-letter tokens dropped).
  /// Exposed for unit testing.
  static std::string normalize_message(const std::string & message);

  /// Whether a given originating node should be promoted, honouring the
  /// include/exclude lists. Exposed for unit testing.
  bool node_is_eligible(const std::string & source_id) const;

  /// Whether a logger name belongs to the medkit stack's own infrastructure
  /// (fault_manager, gateway, the other bridges). Matched on the raw logger
  /// name so namespaced nodes are caught. Exposed for unit testing.
  static bool is_medkit_stack_logger(const std::string & logger_name);

  /// Whether this log should be skipped because it comes from the medkit stack
  /// itself - applies the exclude_medkit_stack toggle to is_medkit_stack_logger.
  /// Exposed for unit testing the toggle.
  bool skips_medkit_stack(const std::string & logger_name) const;

  /// Map an rcl_interfaces/msg/Log.name (a logger name, e.g. "bt_navigator" or
  /// "controller_manager.resource_manager") to the originating node's
  /// fully-qualified name ("/bt_navigator", "/controller_manager"). The gateway
  /// discovers runtime entities by node FQN, so the fault's source_id must use
  /// the same form for faults (and their snapshots / rosbag) to associate with
  /// the entity in the SOVD tree. Exposed for unit testing.
  static std::string node_source_id(const std::string & log_name);

  /// FNV-1a 32-bit hash, fixed spec, emitted as 8 lowercase hex chars. Exposed
  /// for unit testing so a known input asserts a known constant.
  static std::string fnv1a_hex(const std::string & in);

  /// Whether a (fault_code, severity) may be forwarded now under the cooldown
  /// (first occurrence passes; same code+severity within report_cooldown_sec is
  /// suppressed; 0.0 disables). Keyed by severity so a WARN never suppresses a
  /// same-message ERROR escalation. Exposed for unit testing.
  bool cooldown_allows(const std::string & fault_code, uint8_t severity, const rclcpp::Time & now);

  /// Fetch (or lazily create) the per-source FaultReporter for an originating
  /// node, so the fault's source_id is the node that logged, not the bridge.
  /// Bounded by max_tracked_nodes_ with LRU eviction. Exposed for unit testing.
  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & source_id);

  /// Number of currently tracked per-node reporters. Exposed for unit testing.
  size_t tracked_reporter_count();

 private:
  void log_callback(const rcl_interfaces::msg::Log::ConstSharedPtr & msg);

  void load_parameters();

  static std::string to_upper_snake(const std::string & in, size_t max_len);

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;

  // One FaultReporter per originating node (correct source_id + own LocalFilter),
  // LRU-ordered: front() is the least-recently-used entry.
  struct ReporterEntry {
    std::string source_id;
    std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter;
  };
  std::list<ReporterEntry> reporters_lru_;
  std::unordered_map<std::string, std::list<ReporterEntry>::iterator> reporters_;
  std::mutex reporters_mutex_;

  // Per-fault_code forward cooldown: last time a code was forwarded.
  std::unordered_map<std::string, rclcpp::Time> last_forward_;
  std::mutex cooldown_mutex_;

  // Configuration
  std::string rosout_topic_;
  uint8_t severity_floor_;
  std::string code_prefix_;
  std::vector<std::string> exclude_nodes_;
  std::vector<std::string> include_only_nodes_;
  int max_tracked_nodes_;
  double report_cooldown_sec_;
  std::string own_node_name_;
  // When true (default), never promote logs from the medkit stack's own
  // infrastructure nodes (fault_manager, gateway, the other bridges) - else
  // their /rosout lines feed back into faults about medkit itself.
  bool exclude_medkit_stack_;
};

}  // namespace ros2_medkit_log_bridge
