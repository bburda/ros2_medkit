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
 * @file shared_leaf.cpp
 * @brief The node behind a leaf two gateways declare under one id.
 *
 * An App id declared on both sides of an aggregating pair is renamed on merge
 * (`<peer>__<id>`), and that rename only means something where an id built from
 * the leaf id addresses something. So this node carries one resource of each
 * kind that is addressed that way: `calibrate`, whose short name is the wire id
 * of an operation, and `reading`, whose path is the wire id of a data item.
 *
 * The service answers with the node's fully qualified name. Two copies of this
 * node both answer 200, so the name in the response is the only thing on the
 * wire that says WHICH copy ran.
 */

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class SharedLeaf : public rclcpp::Node {
 public:
  SharedLeaf() : Node("shared_leaf") {
    reading_pub_ = this->create_publisher<std_msgs::msg::Float32>("reading", 10);

    calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> & request,
                            const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
          (void)request;  // Trigger has no request fields
          response->success = true;
          response->message = std::string(this->get_fully_qualified_name()) + " calibrated";
          RCLCPP_INFO(this->get_logger(), "Calibration requested: %s", response->message.c_str());
        });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      publish_reading();
    });

    RCLCPP_INFO(this->get_logger(), "Shared leaf started");
  }

  // The service callback and the timer callback both capture `this`, so they
  // have to be torn down before any member they touch is.
  ~SharedLeaf() override {
    timer_->cancel();
    std::lock_guard<std::mutex> lock(callback_mutex_);
    timer_.reset();
    calibrate_srv_.reset();
    reading_pub_.reset();
  }

  SharedLeaf(const SharedLeaf &) = delete;
  SharedLeaf & operator=(const SharedLeaf &) = delete;
  SharedLeaf(SharedLeaf &&) = delete;
  SharedLeaf & operator=(SharedLeaf &&) = delete;

 private:
  void publish_reading() {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!reading_pub_) {
      return;
    }
    reading_ += 1.0;
    if (reading_ > 100.0) {
      reading_ = 1.0;
    }

    auto msg = std_msgs::msg::Float32();
    msg.data = static_cast<float>(reading_);
    reading_pub_->publish(msg);
  }

  std::mutex callback_mutex_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr reading_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  double reading_ = 1.0;
};

int main(int argc, char * argv[]) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, []() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<SharedLeaf>();
  });
}
