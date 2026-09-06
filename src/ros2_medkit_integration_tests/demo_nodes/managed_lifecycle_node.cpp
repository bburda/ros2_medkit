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

#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "ros2_medkit_integration_tests/crash_backtrace.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// A managed lifecycle node for status integration tests. It DEFAULTS to staying
// unconfigured (auto_activate:=false): it is in the ROS graph (so is_online would
// wrongly say "ready") but its lifecycle state is "unconfigured", so accurate
// status must read "notReady". This is the case that proves the lifecycle reader
// is consulted instead of is_online. With auto_activate:=true it configures and
// activates itself shortly after start (status -> ready).
class ManagedLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  ManagedLifecycleNode() : rclcpp_lifecycle::LifecycleNode("managed_lifecycle") {
    auto_activate_ = this->declare_parameter<bool>("auto_activate", false);
    if (auto_activate_) {
      timer_ = this->create_wall_timer(std::chrono::milliseconds(300), [this]() {
        timer_->cancel();
        this->configure();
        this->activate();
      });
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override {
    return CallbackReturn::SUCCESS;
  }

 private:
  bool auto_activate_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  // This node does not go through run_demo_node(), so it installs the
  // crash reporter itself - otherwise it would be the one demo binary that
  // still dies without leaving a stack behind.
  ros2_medkit_integration_tests::install_crash_backtrace();
  // Wrap the body so no exception escapes main (bugprone-exception-escape): the
  // run_demo_node helper used by the other demo nodes only accepts an
  // rclcpp::Node, so a LifecycleNode needs its own main.
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManagedLifecycleNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    fprintf(stderr, "managed_lifecycle: %s\n", e.what());
    return 1;
  } catch (...) {
    fprintf(stderr, "managed_lifecycle: unknown exception\n");
    return 1;
  }
  return 0;
}
