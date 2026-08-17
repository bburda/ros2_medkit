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

#include "ros2_medkit_diagnostic_bridge/diagnostic_bridge_node.hpp"

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <limits>

#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_diagnostic_bridge {

DiagnosticBridgeNode::DiagnosticBridgeNode(const rclcpp::NodeOptions & options) : Node("diagnostic_bridge", options) {
  load_parameters();

  // Subscribe to diagnostics - reporter created lazily on first callback
  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10), [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg) {
        diagnostics_callback(msg);
      });

  RCLCPP_INFO(get_logger(),
              "DiagnosticBridge started (topic=%s, auto_generate=%s, use_hardware_id_as_source_id=%s, mappings=%zu)",
              diagnostics_topic_.c_str(), auto_generate_codes_ ? "true" : "false",
              use_hardware_id_as_source_id_ ? "true" : "false", name_to_code_.size());
}

void DiagnosticBridgeNode::load_parameters() {
  diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  auto_generate_codes_ = declare_parameter<bool>("auto_generate_codes", true);
  use_hardware_id_as_source_id_ = declare_parameter<bool>("use_hardware_id_as_source_id", false);
  const int64_t max_tracked_sources = declare_parameter<int64_t>("max_tracked_sources", 512);
  const int64_t max_tracked_sources_used =
      std::clamp(max_tracked_sources, INT64_C(1), static_cast<int64_t>(std::numeric_limits<int>::max()));
  if (max_tracked_sources_used != max_tracked_sources) {
    RCLCPP_WARN(get_logger(), "max_tracked_sources=%" PRId64 " clamped to %" PRId64, max_tracked_sources,
                max_tracked_sources_used);
  }
  max_tracked_sources_ = static_cast<int>(max_tracked_sources_used);

  std::vector<std::string> keyvalue_codes =
      declare_parameter<std::vector<std::string>>("keyvalue_codes", std::vector<std::string>());
  std::copy_if(keyvalue_codes.begin(), keyvalue_codes.end(), std::back_inserter(keyvalue_codes_),
               [](const std::string & s) {
                 return !s.empty();
               });

  // Load custom name_to_code mappings from parameter overrides
  // Format: name_to_code.<diagnostic_name> = <fault_code>
  // Example: --ros-args -p "name_to_code.motor_temp:=MOTOR_OVERHEAT"
  auto params = get_node_parameters_interface()->get_parameter_overrides();
  const std::string prefix = "name_to_code.";
  for (const auto & [name, value] : params) {
    if (name.rfind(prefix, 0) == 0 && value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      std::string diag_name = name.substr(prefix.length());
      name_to_code_[diag_name] = value.get<std::string>();
      RCLCPP_DEBUG(get_logger(), "Loaded mapping: '%s' -> '%s'", diag_name.c_str(), name_to_code_[diag_name].c_str());
    }
  }
}

void DiagnosticBridgeNode::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg) {
  for (const auto & status : msg->status) {
    process_diagnostic(status);
  }
}

ros2_medkit_fault_reporter::FaultReporter * DiagnosticBridgeNode::reporter_for(const std::string & source_id) {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  auto it = reporters_.find(source_id);
  if (it != reporters_.end()) {
    reporters_lru_.splice(reporters_lru_.end(), reporters_lru_, it->second);
    return it->second->reporter.get();
  }

  // Multiple FaultReporters on one node are safe: FaultReporter guards
  // parameter declaration with has_parameter().
  auto reporter = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(this->shared_from_this(), source_id);
  auto * raw = reporter.get();
  reporters_lru_.push_back(ReporterEntry{source_id, std::move(reporter)});
  reporters_[source_id] = std::prev(reporters_lru_.end());

  if (static_cast<int>(reporters_lru_.size()) > max_tracked_sources_) {
    RCLCPP_WARN_ONCE(get_logger(), "max_tracked_sources (%d) reached; evicting least-recently-used reporters",
                     max_tracked_sources_);
    reporters_.erase(reporters_lru_.front().source_id);
    reporters_lru_.pop_front();
  }
  return raw;
}

size_t DiagnosticBridgeNode::tracked_reporter_count() {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  return reporters_.size();
}

std::string DiagnosticBridgeNode::source_id_for(const diagnostic_msgs::msg::DiagnosticStatus & status) const {
  const std::string fqn = get_fully_qualified_name();
  if (!use_hardware_id_as_source_id_) {
    return fqn;
  }

  if (!status.hardware_id.empty() && status.hardware_id.find('/') != std::string::npos) {
    return status.hardware_id;
  }

  // Not RCLCPP_WARN_THROTTLE: this method is const, so get_clock() yields a const
  // Clock, and Clock::now() is non-const on Humble. Once per process is also the
  // right cadence here - the throttle keeps one static per call site, so it would
  // name a single diagnostic every 10s and never the others.
  RCLCPP_WARN_ONCE(get_logger(),
                   "Diagnostic '%s' hardware_id '%s' is not a slash-containing node ID, using bridge source_id '%s'",
                   status.name.c_str(), status.hardware_id.c_str(), fqn.c_str());
  return fqn;
}

void DiagnosticBridgeNode::process_diagnostic(const diagnostic_msgs::msg::DiagnosticStatus & status) {
  const std::string fault_code = map_to_fault_code(status);

  // Skip if no mapping and auto-generate disabled
  if (fault_code.empty()) {
    return;
  }

  const std::string source_id = source_id_for(status);
  auto * reporter = reporter_for(source_id);
  if (reporter == nullptr) {
    return;
  }

  if (is_ok_level(status.level)) {
    // OK status -> send PASSED event for healing
    reporter->report_passed(fault_code);
    RCLCPP_DEBUG(get_logger(), "Diagnostic OK: %s -> PASSED for %s", status.name.c_str(), fault_code.c_str());
  } else {
    // WARN, ERROR, STALE -> send FAILED event
    auto severity = map_to_severity(status.level);
    // severity is guaranteed to have value here (not OK level)
    reporter->report(fault_code, *severity, status.message);
    RCLCPP_DEBUG(get_logger(), "Diagnostic %s: %s -> fault %s (severity=%d)", status.name.c_str(),
                 status.message.c_str(), fault_code.c_str(), *severity);
  }
}

std::string DiagnosticBridgeNode::map_to_fault_code(const diagnostic_msgs::msg::DiagnosticStatus & status) const {
  const std::string & diagnostic_name = status.name;
  // Check custom mappings first
  const auto it = name_to_code_.find(diagnostic_name);
  if (it != name_to_code_.end()) {
    return it->second;
  }

  // Check if the diagnostic contains any attributes that contain a code
  if (!keyvalue_codes_.empty()) {
    for (const auto & configured_key : keyvalue_codes_) {
      for (const auto & kv : status.values) {
        if (kv.key == configured_key) {
          return kv.value;
        }
      }
    }
  }

  // Auto-generate if enabled
  if (auto_generate_codes_) {
    return generate_fault_code(diagnostic_name);
  }

  // Log warning and return empty string if no mapping and auto-generate disabled
  // Use a mutable clock copy — Humble's RCLCPP_WARN_THROTTLE requires non-const Clock
  rclcpp::Clock clock(*get_clock());
  RCLCPP_WARN_THROTTLE(get_logger(), clock, 5000, "No mapping for diagnostic '%s' and auto_generate_codes is disabled",
                       diagnostic_name.c_str());
  return "";
}

std::optional<uint8_t> DiagnosticBridgeNode::map_to_severity(uint8_t diagnostic_level) {
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using Fault = ros2_medkit_msgs::msg::Fault;

  switch (diagnostic_level) {
    case DiagStatus::OK:
      return std::nullopt;  // Use is_ok_level() and send PASSED instead
    case DiagStatus::WARN:
      return Fault::SEVERITY_WARN;
    case DiagStatus::ERROR:
      return Fault::SEVERITY_ERROR;
    case DiagStatus::STALE:
      return Fault::SEVERITY_CRITICAL;
    default:
      return Fault::SEVERITY_ERROR;  // Unknown level -> ERROR
  }
}

bool DiagnosticBridgeNode::is_ok_level(uint8_t diagnostic_level) {
  return diagnostic_level == diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string DiagnosticBridgeNode::generate_fault_code(const std::string & diagnostic_name) {
  std::string result;
  result.reserve(diagnostic_name.size());

  bool last_was_separator = true;  // Start true to avoid leading underscore
  for (char c : diagnostic_name) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      result += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      last_was_separator = false;
    } else if (!last_was_separator) {
      // Replace non-alphanumeric with underscore (but avoid doubles)
      result += '_';
      last_was_separator = true;
    }
  }

  // Remove trailing underscore if present
  if (!result.empty() && result.back() == '_') {
    result.pop_back();
  }

  return result;
}

}  // namespace ros2_medkit_diagnostic_bridge
