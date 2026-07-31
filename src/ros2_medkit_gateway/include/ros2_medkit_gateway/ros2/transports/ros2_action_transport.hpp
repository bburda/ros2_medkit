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

#pragma once

#include <action_msgs/msg/goal_status_array.hpp>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>

#include "ros2_medkit_gateway/compat/generic_client_compat.hpp"
#include "ros2_medkit_gateway/core/transports/action_transport.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing ActionTransport.
 *
 * Owns the action-internal-service client cache (send_goal / get_result /
 * cancel_goal) and the GoalStatusArray subscription cache that
 * OperationManager previously held directly. Subscriptions captured via
 * lambda are torn down before any other member destructs to satisfy the
 * subscription-destructor pattern (see CLAUDE.md: callbacks must not fire
 * on a partially-destroyed object).
 */
class Ros2ActionTransport : public ActionTransport {
 public:
  /**
   * @param node Non-owning ROS node used for client + subscription creation.
   * @param rpc_group Callback group for the send_goal / get_result /
   *        cancel_goal clients' response callbacks - the shared Reentrant
   *        RPC group from ros2_common::create_gateway_callback_groups()
   *        (issue #575), so responses can be delivered while default-group
   *        callbacks run.
   * @param status_group Callback group for the `/_action/status`
   *        subscriptions - the shared MutuallyExclusive group, preserving
   *        per-subscription in-order delivery while decoupling status
   *        tracking from the default group.
   *        Both group shared_ptrs are kept alive by this transport (the
   *        node only holds weak references to its callback groups).
   */
  Ros2ActionTransport(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr rpc_group,
                      rclcpp::CallbackGroup::SharedPtr status_group);

  ~Ros2ActionTransport() override;

  Ros2ActionTransport(const Ros2ActionTransport &) = delete;
  Ros2ActionTransport & operator=(const Ros2ActionTransport &) = delete;
  Ros2ActionTransport(Ros2ActionTransport &&) = delete;
  Ros2ActionTransport & operator=(Ros2ActionTransport &&) = delete;

  ActionSendGoalResult send_goal(const std::string & action_path, const std::string & action_type, const json & goal,
                                 std::chrono::duration<double> timeout) override;

  ActionCancelResult cancel_goal(const std::string & action_path, const std::string & goal_id,
                                 std::chrono::duration<double> timeout) override;

  ActionGetResultResult get_result(const std::string & action_path, const std::string & action_type,
                                   const std::string & goal_id, std::chrono::duration<double> timeout) override;

  void subscribe_status(const std::string & action_path, StatusCallback callback) override;

  void unsubscribe_status(const std::string & action_path) override;

 private:
  /// Cached internal-service clients for one action path.
  struct ActionClientSet {
    compat::GenericServiceClient::SharedPtr send_goal_client;
    compat::GenericServiceClient::SharedPtr get_result_client;
    compat::GenericServiceClient::SharedPtr cancel_goal_client;
    std::string action_type;
  };

  ActionClientSet & get_or_create_clients(const std::string & action_path, const std::string & action_type);

  void on_status_msg(const std::string & action_path, const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg);

  rclcpp::Node * node_;
  /// Shared Reentrant group for RPC response dispatch and shared
  /// MutuallyExclusive group for status subscriptions; owned here because
  /// the node keeps only weak references to its callback groups.
  rclcpp::CallbackGroup::SharedPtr rpc_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  /// Set on the first shutdown signal so callbacks short-circuit while the
  /// adapter is being torn down.
  std::atomic<bool> shutdown_requested_{false};

  mutable std::shared_mutex clients_mutex_;
  std::map<std::string, ActionClientSet> action_clients_;

  /// Status subscriptions and the user-provided callbacks they fan into.
  /// Cleared in the destructor (and in unsubscribe_status) before any other
  /// member destructs.
  mutable std::mutex subscriptions_mutex_;
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> status_subscriptions_;
  std::map<std::string, StatusCallback> status_callbacks_;
};

}  // namespace ros2_medkit_gateway::ros2
