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
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "ros2_medkit_graph_watchdog/watchdog_clock.hpp"

class WatchdogClockTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("watchdog_clock_test_node");
  }
  void TearDown() override {
    node_.reset();
  }
  rclcpp::Node::SharedPtr node_;
};

TEST_F(WatchdogClockTest, IsMonotonicNonDecreasing) {
  ros2_medkit_graph_watchdog::WatchdogClock clock(node_.get());
  const rclcpp::Time t1 = clock.now();
  const rclcpp::Time t2 = clock.now();
  EXPECT_GE(t2.nanoseconds(), t1.nanoseconds());
}

// Exercises that WatchdogClock actually honors use_sim_time (rather than just
// asserting the clock type matches the node's own clock, which is true by
// construction and proves nothing about sim-time wiring). Publishes on /clock
// and asserts now() reports the published sim time, not wall-clock time.
TEST_F(WatchdogClockTest, HonorsSimTime) {
  auto sim_node = std::make_shared<rclcpp::Node>(
      "watchdog_clock_sim_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  auto clock_pub = sim_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  ros2_medkit_graph_watchdog::WatchdogClock wclock(sim_node.get());

  rosgraph_msgs::msg::Clock msg;
  msg.clock = rclcpp::Time(12345, 678000000, RCL_ROS_TIME);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sim_node);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  double t = 0.0;
  while (std::chrono::steady_clock::now() < deadline) {
    clock_pub->publish(msg);
    exec.spin_some();
    t = wclock.now().seconds();
    if (t > 12000.0 && t < 13000.0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_GT(t, 12000.0);  // sim time ~12345s, not wall-clock ~1.7e9s
  EXPECT_LT(t, 13000.0);
}

TEST_F(WatchdogClockTest, SystemTimeAlwaysValid) {
  ros2_medkit_graph_watchdog::WatchdogClock clock(node_.get());  // use_sim_time not set -> false
  clock.mark_tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  clock.mark_tick();
  EXPECT_TRUE(clock.time_is_valid());
}

TEST_F(WatchdogClockTest, StalledSimClockIsInvalid) {
  auto sim_node = std::make_shared<rclcpp::Node>(
      "watchdog_clock_stall_node",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  // No /clock ever published -> sim time stays at 0 while wall time advances.
  ros2_medkit_graph_watchdog::WatchdogClock clock(sim_node.get());
  clock.mark_tick();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  clock.mark_tick();
  EXPECT_FALSE(clock.time_is_valid());  // sim time did not advance while wall time did
}
