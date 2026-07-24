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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_medkit_fault_reporter/local_filter.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_reporter {

/// Client library for reporting faults to the central FaultManager
///
/// Provides a simple API for ROS 2 nodes to report faults with optional
/// local filtering to reduce noise from repeated fault occurrences.
///
/// Example usage:
/// @code
/// class MyNode : public rclcpp::Node {
///  public:
///   MyNode() : Node("my_node") {
///     reporter_ = std::make_unique<FaultReporter>(
///         shared_from_this(), get_fully_qualified_name());
///   }
///
///   void on_error() {
///     reporter_->report("SENSOR_FAILURE", Fault::SEVERITY_ERROR, "Sensor timeout");
///   }
///
///  private:
///   std::unique_ptr<FaultReporter> reporter_;
/// };
/// @endcode
///
/// Inside a lifecycle node, construct it from the node directly (no shared_from_this needed):
/// @code
/// class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
///   CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
///     reporter_ = std::make_unique<FaultReporter>(*this, get_fully_qualified_name());
///     return CallbackReturn::SUCCESS;
///   }
/// };
/// @endcode
class FaultReporter {
 public:
  /// Base Constructor for FaultReporter
  ///
  /// @param node_base ROS2 Node Base Interface for creating the service client
  /// @param node_graph ROS2 Node Graph Interface for creating the service client
  /// @param node_services ROS2 Node Services Interface for creating the service client
  /// @param node_params ROS2 Node Parameters Interface for loading parameters
  /// @param logger ROS2 Node Logger for reporting logs
  /// @param source_id Identifier for this reporter (typically node's FQN)
  /// @param service_name Name of the ReportFault service (default: /fault_manager/report_fault)
  FaultReporter(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
                rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params, rclcpp::Logger logger,
                const std::string & source_id, const std::string & service_name = "/fault_manager/report_fault");

  /// Construct a FaultReporter via Node Shared Ptr (maintain backwards compatibiilty)
  ///
  /// @param node The ROS 2 node to use for service client and parameters
  /// @param source_id Identifier for this reporter (typically node's FQN)
  /// @param service_name Name of the ReportFault service (default: /fault_manager/report_fault)
  FaultReporter(rclcpp::Node::SharedPtr node, const std::string & source_id,
                const std::string & service_name = "/fault_manager/report_fault");

  /// Construct a FaultReporter via Node
  ///
  /// @param node The ROS 2 node to use for service client and parameters
  /// @param source_id Identifier for this reporter (typically node's FQN)
  /// @param service_name Name of the ReportFault service (default: /fault_manager/report_fault)
  FaultReporter(rclcpp::Node & node, const std::string & source_id,
                const std::string & service_name = "/fault_manager/report_fault");

  /// Construct a FaultReporter via Lifecycle Node
  ///
  /// @param node The ROS 2 node to use for service client and parameters
  /// @param source_id Identifier for this reporter (typically node's FQN)
  /// @param service_name Name of the ReportFault service (default: /fault_manager/report_fault)
  FaultReporter(rclcpp_lifecycle::LifecycleNode & node, const std::string & source_id,
                const std::string & service_name = "/fault_manager/report_fault");

  /// Report a FAILED event (fault occurrence)
  ///
  /// The fault will be forwarded to the FaultManager if local filtering
  /// allows it (threshold met within time window, or high severity).
  ///
  /// @param fault_code Global fault identifier (e.g., "MOTOR_OVERHEAT")
  /// @param severity Severity level (use Fault::SEVERITY_* constants)
  /// @param description Human-readable fault description
  void report(const std::string & fault_code, uint8_t severity, const std::string & description);

  /// Report a PASSED event (fault condition cleared)
  ///
  /// PASSED events bypass local filtering and are always forwarded to FaultManager.
  /// Use this when the condition that caused a fault is no longer present.
  ///
  /// @param fault_code Global fault identifier (must match a previously reported fault)
  void report_passed(const std::string & fault_code);

  /// Check if the FaultManager service is available
  bool is_service_ready() const;

  /// Get read-only access to the local filter (for testing)
  const LocalFilter & filter() const {
    return filter_;
  }

 private:
  /// Load filter configuration from ROS parameters
  void load_parameters(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params);

  /// Send the fault report to FaultManager (async, fire-and-forget)
  void send_report(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                   const std::string & description);

  std::string source_id_;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr client_;
  LocalFilter filter_;
  rclcpp::Logger logger_;
};

}  // namespace ros2_medkit_fault_reporter
