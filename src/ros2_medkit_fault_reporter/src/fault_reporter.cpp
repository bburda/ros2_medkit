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

#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

namespace ros2_medkit_fault_reporter {

FaultReporter::FaultReporter(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
                             rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                             rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
                             rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
                             rclcpp::Logger logger, const std::string & source_id, const std::string & service_name)
  : source_id_(source_id), logger_(logger) {
  // Validate source_id
  if (source_id_.empty()) {
    RCLCPP_WARN(logger_, "FaultReporter created with empty source_id, fault origins will be difficult to trace");
  }

  // Create service client (stored as shared_ptr, independent of node lifetime)
  client_ = rclcpp::create_client<ros2_medkit_msgs::srv::ReportFault>(
      node_base, node_graph, node_services, service_name, rmw_qos_profile_services_default, nullptr);

  // Load configuration from parameters (node only needed during construction)
  load_parameters(node_params);

  RCLCPP_DEBUG(logger_, "FaultReporter initialized for source: %s", source_id_.c_str());
}

FaultReporter::FaultReporter(rclcpp::Node::SharedPtr node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                  node->get_node_services_interface(), node->get_node_parameters_interface(), node->get_logger(),
                  source_id, service_name) {
}

FaultReporter::FaultReporter(rclcpp::Node & node, const std::string & source_id, const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_logger(), source_id, service_name) {
}

FaultReporter::FaultReporter(rclcpp_lifecycle::LifecycleNode & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_logger(), source_id, service_name) {
}

void FaultReporter::load_parameters(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params) {
  FilterConfig config;

  // Declare parameters with defaults if not already declared
  if (!node_params->has_parameter("fault_reporter.local_filtering.enabled")) {
    node_params->declare_parameter("fault_reporter.local_filtering.enabled", rclcpp::ParameterValue(config.enabled));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_threshold")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_threshold",
                                   rclcpp::ParameterValue(config.default_threshold));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_window_sec")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_window_sec",
                                   rclcpp::ParameterValue(config.default_window_sec));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.bypass_severity")) {
    node_params->declare_parameter("fault_reporter.local_filtering.bypass_severity",
                                   rclcpp::ParameterValue(static_cast<int>(config.bypass_severity)));
  }

  config.enabled = node_params->get_parameter("fault_reporter.local_filtering.enabled").as_bool();
  config.default_threshold =
      static_cast<int>(node_params->get_parameter("fault_reporter.local_filtering.default_threshold").as_int());
  config.default_window_sec =
      node_params->get_parameter("fault_reporter.local_filtering.default_window_sec").as_double();
  config.bypass_severity =
      static_cast<uint8_t>(node_params->get_parameter("fault_reporter.local_filtering.bypass_severity").as_int());

  filter_.set_config(config);

  RCLCPP_DEBUG(logger_, "Filter config: enabled=%d, threshold=%d, window=%.1fs, bypass_severity=%d", config.enabled,
               config.default_threshold, config.default_window_sec, config.bypass_severity);
}

void FaultReporter::report(const std::string & fault_code, uint8_t severity, const std::string & description) {
  // Validate fault_code
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report fault with empty fault_code, ignoring");
    return;
  }

  // Check if filter allows forwarding
  if (!filter_.should_forward(fault_code, severity)) {
    RCLCPP_DEBUG(logger_, "Fault '%s' filtered (threshold not met)", fault_code.c_str());
    return;
  }

  send_report(fault_code, ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED, severity, description);
}

void FaultReporter::report_passed(const std::string & fault_code) {
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report PASSED with empty fault_code, ignoring");
    return;
  }

  if (!filter_.should_forward_passed(fault_code)) {
    RCLCPP_DEBUG(logger_, "PASSED for '%s' filtered (threshold not met)", fault_code.c_str());
    return;
  }

  send_report(fault_code, ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED, 0, "");
}

bool FaultReporter::is_service_ready() const {
  return client_->service_is_ready();
}

void FaultReporter::send_report(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                const std::string & description) {
  if (!client_->service_is_ready()) {
    // Use WARN level for high-severity faults that would bypass filtering
    if (event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED &&
        severity >= filter_.config().bypass_severity) {
      RCLCPP_WARN(logger_, "FaultManager service not available, dropping high-severity fault '%s'", fault_code.c_str());
    } else {
      RCLCPP_DEBUG(logger_, "FaultManager service not available, skipping report for '%s'", fault_code.c_str());
    }
    return;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->event_type = event_type;
  request->severity = severity;
  request->description = description;
  request->source_id = source_id_;

  // Fire and forget - don't block on response
  client_->async_send_request(request);

  if (event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED) {
    RCLCPP_DEBUG(logger_, "Reported FAILED: %s (severity=%d)", fault_code.c_str(), severity);
  } else {
    RCLCPP_DEBUG(logger_, "Reported PASSED: %s", fault_code.c_str());
  }
}

}  // namespace ros2_medkit_fault_reporter
