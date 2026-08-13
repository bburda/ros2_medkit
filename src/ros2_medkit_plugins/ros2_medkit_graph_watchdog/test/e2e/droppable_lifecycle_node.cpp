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
 * @file droppable_lifecycle_node.cpp
 * @brief Demo node that looks managed, answers a chosen lifecycle label, and can STOP
 *        looking managed on command.
 *
 * Proof fixture for two claims the lifecycle_expectation detector makes that no other
 * fixture in this workspace can reach:
 *
 * 1. A present, healthy, managed node whose `get_state` path is missing from a sweep reads
 *    "not a managed lifecycle node" for a tick or two. If that happens to be the last thing
 *    observed before the node shuts down cleanly, the departure carve-out must NOT turn a
 *    healthy departure into a fault. Dropping the services on command and then being killed
 *    is exactly that sequence, produced deliberately instead of waited for.
 * 2. The same drop, held for longer, is a node that is GENUINELY not managed when it leaves -
 *    which must still be reported. The only difference between the two runs is how long the
 *    dropped state is held, which is what makes the pair discriminating.
 *
 * It also supplies the one state no other fixture can: a node that reads a NON-active
 * lifecycle label and can then leave the managed set WITHOUT becoming healthy and WITHOUT
 * leaving the graph. `managed_lifecycle_node.cpp` is a real rclcpp_lifecycle::LifecycleNode
 * and cannot un-advertise its lifecycle services; `unreadable_lifecycle_node.cpp` never
 * answers at all, so it can never be measured not-active in the first place.
 *
 * Like `unreadable_lifecycle_node.cpp`, this is a plain rclcpp::Node that only has to look
 * managed to the gateway's discovery layer: `find_lifecycle_get_state_path()` matches purely
 * on discovered SERVICE TYPE (one `lifecycle_msgs/srv/GetState` plus one
 * `lifecycle_msgs/srv/ChangeState`), never on the node's C++ type. Unlike that fixture, this
 * one answers `get_state` immediately - the interesting variable here is whether the services
 * EXIST, not whether they respond.
 *
 * Parameters:
 * - `state_label` (string, default "active"): the label `get_state` answers with. "active"
 *   also reports the ACTIVE state id; anything else reports UNCONFIGURED's id with the
 *   given label, which is what the detector classifies as a violation.
 * - `drop_services` (bool, default false): setting it true at runtime - through the node's
 *   own auto-started `~/set_parameters` service - destroys both lifecycle services, so the
 *   node stays in the graph but stops being a managed lifecycle node. It is a parameter
 *   rather than a timer so the test controls exactly WHEN, after it has independently
 *   confirmed the starting state, instead of racing a fixed delay against the detector's own
 *   horizons. Setting it back to false does not re-advertise: this fixture models a node
 *   losing its lifecycle interface, and a re-advertise would need a second, unrelated
 *   discovery round trip to be meaningful.
 */

#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

class DroppableLifecycleNode : public rclcpp::Node {
 public:
  DroppableLifecycleNode() : Node("droppable_lifecycle") {
    state_label_ = this->declare_parameter<std::string>("state_label", "active");
    const bool drop_at_start = this->declare_parameter<bool>("drop_services", false);

    advertise();
    if (drop_at_start) {
      drop_services();
    }

    param_callback_handle_ =
        this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto & param : params) {
            if (param.get_name() == "drop_services" && param.as_bool()) {
              drop_services();
            }
          }
          return result;
        });

    RCLCPP_INFO(get_logger(),
                "droppable_lifecycle started: get_state answers '%s'; set drop_services:=true to stop advertising "
                "the lifecycle services",
                state_label_.c_str());
  }

  ~DroppableLifecycleNode() override {
    get_state_service_.reset();
    change_state_service_.reset();
    param_callback_handle_.reset();
  }
  DroppableLifecycleNode(const DroppableLifecycleNode &) = delete;
  DroppableLifecycleNode & operator=(const DroppableLifecycleNode &) = delete;
  DroppableLifecycleNode(DroppableLifecycleNode &&) = delete;
  DroppableLifecycleNode & operator=(DroppableLifecycleNode &&) = delete;

 private:
  void advertise() {
    get_state_service_ = this->create_service<lifecycle_msgs::srv::GetState>(
        "~/get_state", [this](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*request*/,
                              const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & response) {
          response->current_state.id = state_label_ == "active"
                                           ? lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
                                           : lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
          response->current_state.label = state_label_;
        });
    change_state_service_ = this->create_service<lifecycle_msgs::srv::ChangeState>(
        "~/change_state", [](const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> & /*request*/,
                             const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> & response) {
          // Never driven: only find_lifecycle_get_state_path()'s type check needs it to exist.
          response->success = true;
        });
  }

  void drop_services() {
    if (!get_state_service_ && !change_state_service_) {
      return;  // already dropped - a second drop_services:=true is a no-op
    }
    get_state_service_.reset();
    change_state_service_.reset();
    RCLCPP_INFO(get_logger(),
                "droppable_lifecycle dropped its lifecycle services: still in the graph, no longer a "
                "managed lifecycle node");
  }

  std::string state_label_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_service_;
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroppableLifecycleNode>());
  rclcpp::shutdown();
  return 0;
}
