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

// Pins the callback-group contract behind server.executor_threads (issue
// #575): the blocking-RPC clients must dispatch on a Reentrant group (so a
// second executor thread can deliver a response while a default-group
// callback runs) and the per-action status subscriptions must dispatch on a
// dedicated MutuallyExclusive group (in-order delivery, decoupled from the
// default group). A regression that silently reverted either group to the
// node default would pass every functional test on an idle system - this
// suite makes the wiring itself falsifiable.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "ros2_medkit_gateway/ros2/transports/ros2_action_transport.hpp"
#include "ros2_medkit_gateway/ros2_common/callback_groups.hpp"

using ros2_medkit_gateway::ros2::Ros2ActionTransport;
using ros2_medkit_gateway::ros2_common::create_gateway_callback_groups;
using ros2_medkit_gateway::ros2_common::GatewayCallbackGroups;

class CallbackGroupsTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_callback_groups_node");
  }

  void TearDown() override {
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(CallbackGroupsTest, RpcGroupIsReentrantStatusGroupIsMutuallyExclusive) {
  auto groups = create_gateway_callback_groups(*node_);

  ASSERT_NE(groups.rpc_reentrant, nullptr);
  ASSERT_NE(groups.action_status, nullptr);
  EXPECT_EQ(groups.rpc_reentrant->type(), rclcpp::CallbackGroupType::Reentrant);
  EXPECT_EQ(groups.action_status->type(), rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NE(groups.rpc_reentrant.get(), groups.action_status.get());
}

TEST_F(CallbackGroupsTest, GroupsAreDispatchedByTheNodeExecutor) {
  // automatically_add_to_executor_with_node makes whichever executor spins
  // the node also service these groups - without it, every entity in them
  // would silently never fire.
  auto groups = create_gateway_callback_groups(*node_);
  EXPECT_TRUE(groups.rpc_reentrant->automatically_add_to_executor_with_node());
  EXPECT_TRUE(groups.action_status->automatically_add_to_executor_with_node());
}

TEST_F(CallbackGroupsTest, ActionStatusSubscriptionLandsInStatusGroup) {
  auto groups = create_gateway_callback_groups(*node_);
  Ros2ActionTransport transport(node_.get(), groups.rpc_reentrant, groups.action_status);

  transport.subscribe_status("/powertrain/engine/phantom",
                             [](const std::string &, const std::string &, ros2_medkit_gateway::ActionGoalStatus) {});

  // The status subscription must be registered into the shared status group,
  // not the node default - otherwise a long default-group callback delays
  // goal-status tracking again. Match by topic name: the default group also
  // holds rclcpp-internal subscriptions (e.g. /parameter_events), so a bare
  // count would not isolate the status subscription.
  const std::string status_topic = "/powertrain/engine/phantom/_action/status";
  auto is_status_sub = [&status_topic](const rclcpp::SubscriptionBase::SharedPtr & sub) {
    return sub != nullptr && status_topic == sub->get_topic_name();
  };

  EXPECT_NE(groups.action_status->find_subscription_ptrs_if(is_status_sub), nullptr)
      << "status subscription not registered into the shared status group";
  EXPECT_EQ(node_->get_node_base_interface()->get_default_callback_group()->find_subscription_ptrs_if(is_status_sub),
            nullptr)
      << "status subscription must not land in the node default group";
}
