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
 * @file dual_calibration_service.cpp
 * @brief One node exposing two services whose ROS paths differ only above the
 *        last segment.
 *
 * `left/calibrate` and `right/calibrate` under the node's namespace are two
 * different services with one short name, and the short name is the wire id the
 * operations collection uses. One provider carries both, so the member half of
 * a qualified id names the same thing for each - which is the case the ROS path
 * has to address instead.
 *
 * Each side answers with a message naming itself, so a caller can tell which
 * service ran from the response rather than from the status alone.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class DualCalibrationService : public rclcpp::Node {
 public:
  DualCalibrationService() : Node("dual_calibration") {
    left_srv_ = make_side("left");
    right_srv_ = make_side("right");

    RCLCPP_INFO(this->get_logger(), "Dual calibration services started");
  }

  // The callbacks capture `this`, so the services have to go before any member
  // they touch does.
  ~DualCalibrationService() override {
    left_srv_.reset();
    right_srv_.reset();
  }

  DualCalibrationService(const DualCalibrationService &) = delete;
  DualCalibrationService & operator=(const DualCalibrationService &) = delete;
  DualCalibrationService(DualCalibrationService &&) = delete;
  DualCalibrationService & operator=(DualCalibrationService &&) = delete;

 private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr make_side(const std::string & side) {
    return this->create_service<std_srvs::srv::Trigger>(
        side + "/calibrate", [this, side](const std::shared_ptr<std_srvs::srv::Trigger::Request> & request,
                                          const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
          (void)request;  // Trigger has no request fields
          response->success = true;
          response->message = side + " side calibrated";
          RCLCPP_INFO(this->get_logger(), "Calibration requested: %s", response->message.c_str());
        });
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr left_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr right_srv_;
};

int main(int argc, char * argv[]) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, []() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<DualCalibrationService>();
  });
}
