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
#pragma once

#include <cstdint>
#include <string>

#include <ros2_medkit_msgs/srv/report_fault.hpp>

// ReportFault request builders. Header-only (depends only on the msg package) so
// the raise/clear payload is unit-testable without rclcpp or a PluginContext.
// fault_manager rejects any ReportFault with an empty source_id, so both builders
// MUST populate it from the reporting entity.
namespace ros2_medkit_graph_watchdog {

/// Build a fault RAISE request (FAILED event).
inline ros2_medkit_msgs::srv::ReportFault::Request make_fault_report(const std::string & source_id,
                                                                     const std::string & fault_code, uint8_t severity,
                                                                     const std::string & description) {
  ros2_medkit_msgs::srv::ReportFault::Request req;
  req.fault_code = fault_code;
  req.event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
  req.severity = severity;
  req.description = description;
  req.source_id = source_id;
  return req;
}

/// Build a fault CLEAR request (PASSED event). source_id must match the raise.
inline ros2_medkit_msgs::srv::ReportFault::Request make_fault_clear(const std::string & source_id,
                                                                    const std::string & fault_code) {
  ros2_medkit_msgs::srv::ReportFault::Request req;
  req.fault_code = fault_code;
  req.event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED;
  req.source_id = source_id;
  return req;
}

}  // namespace ros2_medkit_graph_watchdog
