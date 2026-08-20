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
 * @brief One node exposing two services AND two actions whose ROS paths differ
 *        only above the last segment.
 *
 * `left/calibrate` and `right/calibrate` under the node's namespace are two
 * different services with one short name, and the short name is the wire id the
 * operations collection uses. One provider carries both, so the member half of
 * a qualified id names the same thing for each - which is the case the ROS path
 * has to address instead.
 *
 * `left/sweep` and `right/sweep` are the same collision on the action side, and
 * it has to exist separately: only an action produces executions, so a ROS path
 * is the only id under which a goal of a twice-named operation can be listed at
 * all. A service answers inside its call and leaves nothing to address.
 *
 * Each side answers with a message naming itself, so a caller can tell which
 * service ran from the response rather than from the status alone. Each sweep
 * runs one step per goal `order` at 2 Hz, so a goal started with a large order
 * is still running when the execution that started it is looked up.
 */

#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class DualCalibrationService : public rclcpp::Node {
 public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  DualCalibrationService() : Node("dual_calibration") {
    left_srv_ = make_side("left");
    right_srv_ = make_side("right");
    left_sweep_ = make_sweep("left");
    right_sweep_ = make_sweep("right");

    RCLCPP_INFO(this->get_logger(), "Dual calibration services and sweeps started");
  }

  void prepare_shutdown() {
    shutdown_.store(true);
    if (left_thread_.joinable()) {
      left_thread_.join();
    }
    if (right_thread_.joinable()) {
      right_thread_.join();
    }
    left_sweep_.reset();
    right_sweep_.reset();
  }

  // The callbacks capture `this`, so the services and the goal-executing
  // threads have to go before any member they touch does.
  ~DualCalibrationService() override {
    prepare_shutdown();
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

  rclcpp_action::Server<Fibonacci>::SharedPtr make_sweep(const std::string & side) {
    return rclcpp_action::create_server<Fibonacci>(
        this, side + "/sweep",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandleFibonacci> &) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, side](const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
          std::thread & slot = side == "left" ? left_thread_ : right_thread_;
          if (slot.joinable()) {
            slot.join();
          }
          slot = std::thread(&DualCalibrationService::sweep, this, side, goal_handle);
        });
  }

  /// One sweep step per goal `order` at 2 Hz, so a goal ordered generously is
  /// still running while the caller looks it up.
  ///
  /// A goal in flight when the node goes down is left alone: rclcpp_action
  /// clears its own tracking during shutdown, and reporting an outcome into
  /// that window throws.
  void sweep(const std::string & side, const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
    try {
      auto feedback = std::make_shared<Fibonacci::Feedback>();
      auto result = std::make_shared<Fibonacci::Result>();
      feedback->sequence.push_back(0);
      rclcpp::Rate rate(2);

      for (int step = 1; step < goal_handle->get_goal()->order && rclcpp::ok() && !shutdown_.load(); ++step) {
        if (goal_handle->is_canceling()) {
          result->sequence = feedback->sequence;
          goal_handle->canceled(result);
          return;
        }
        feedback->sequence.push_back(step);
        goal_handle->publish_feedback(feedback);
        rate.sleep();
      }

      if (!rclcpp::ok() || shutdown_.load()) {
        return;
      }
      result->sequence = feedback->sequence;
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "%s side swept", side.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Sweep interrupted: %s", e.what());
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr left_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr right_srv_;
  rclcpp_action::Server<Fibonacci>::SharedPtr left_sweep_;
  rclcpp_action::Server<Fibonacci>::SharedPtr right_sweep_;
  std::thread left_thread_;
  std::thread right_thread_;
  std::atomic<bool> shutdown_{false};
};

int main(int argc, char * argv[]) {
  // rclcpp_action may throw "Asked to publish result for goal that does not
  // exist" during SIGINT shutdown if a goal is in flight: the executor can run
  // a goal state callback after the action server has cleared its tracking.
  // Exiting cleanly there keeps a torn-down demo node from reading as a crash.
  std::set_terminate([]() {
    _exit(0);
  });
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, []() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<DualCalibrationService>();
  });
}
