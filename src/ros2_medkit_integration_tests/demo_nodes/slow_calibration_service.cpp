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

/**
 * @file slow_calibration_service.cpp
 * @brief Demo calibration service that takes a settable time to answer.
 *
 * The fixture for a budget. Every other service node in this package answers
 * immediately, which cannot tell a budget that is too small from one that is
 * right - a test written against an instant service passes on both sides of a
 * timeout change and measures nothing.
 *
 * `response_delay_sec` is settable at runtime, so one test can hold a call
 * open past one budget and inside another without relaunching the node.
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class SlowCalibrationService : public rclcpp::Node {
 public:
  SlowCalibrationService() : Node("slow_calibration_service") {
    rcl_interfaces::msg::ParameterDescriptor delay_desc;
    delay_desc.description = "Seconds the calibrate service waits before answering";
    declare_parameter("response_delay_sec", 0.0, delay_desc);

    rcl_interfaces::msg::ParameterDescriptor offset_desc;
    offset_desc.description = "Calibration offset value";
    declare_parameter("calibration_offset", 0.0, offset_desc);

    // No callback group and no QoS override. run_demo_node spins a
    // SingleThreadedExecutor, so callbacks serialise whatever group they are
    // in, and the QoS argument to create_service is a different type on Humble
    // than on Jazzy - naming either buys nothing here and costs a distro.
    calibration_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/calibrate", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const auto delay = std::chrono::duration<double>(get_parameter("response_delay_sec").as_double());
          const auto deadline = std::chrono::steady_clock::now() + delay;
          // Slept in slices rather than in one call so SIGINT during a long
          // delay still ends the process inside the launch_testing grace
          // window; a single sleep_for would hold the executor for the whole
          // delay whatever the context says.
          while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          response->success = true;
          response->message =
              "Calibration complete after " + std::to_string(get_parameter("response_delay_sec").as_double()) + "s";
        });

    RCLCPP_INFO(get_logger(), "Slow calibration service started");
  }

 private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_srv_;
};

int main(int argc, char ** argv) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, [] {
    return std::make_shared<SlowCalibrationService>();
  });
}
