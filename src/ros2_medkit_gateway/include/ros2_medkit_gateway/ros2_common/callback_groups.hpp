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

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway::ros2_common {

/**
 * @brief Callback groups for the gateway node's blocking-RPC clients and
 * action-status subscriptions (issue #575).
 *
 * WHY THIS LIVES IN ros2_common/: creating callback groups is one of the
 * node-mutating rclcpp calls covered by the issue-#375 regression gate
 * (`scripts/check_no_naked_subscriptions.sh` bans `create_callback_group`
 * outside `ros2_common/`). Group creation is therefore funnelled through
 * this factory instead of being scattered across transports.
 *
 * Thread-safety contract:
 * - `create_gateway_callback_groups()` is called exactly ONCE, on the single
 *   startup thread, before the executor spins and before any RESTServer
 *   worker exists - group creation itself never races other node mutations.
 * - The returned shared_ptrs are handed to the transports as constructor
 *   dependencies and may afterwards be passed to entity-creation calls
 *   (`create_generic_client` / `create_subscription`) from HTTP worker
 *   threads. Re-homing those entities into these groups adds no new
 *   concurrency: registration into a shared group is the same class of
 *   operation as the pre-existing registration into the shared *default*
 *   group, and stays serialized by the per-transport client mutexes.
 * - The transports keep their group shared_ptr for their own lifetime; the
 *   node only holds weak references to its groups, so dropping the last
 *   shared_ptr would silently stop dispatch for every entity in the group.
 */
struct GatewayCallbackGroups {
  /// Shared Reentrant group for the response callbacks of blocking RPC
  /// clients: the generic service clients behind `/operations` executions
  /// and the per-action send_goal / get_result / cancel_goal client trio.
  /// Reentrant so a response can be dispatched on any executor thread even
  /// while a default-group callback (e.g. a discovery refresh pass) is
  /// running - this is what makes `server.executor_threads` > 1 buy actual
  /// RPC-response parallelism. A Reentrant group does not *require* a
  /// second thread; a single-threaded executor still services it.
  rclcpp::CallbackGroup::SharedPtr rpc_reentrant;

  /// Shared MutuallyExclusive group for the per-action `/_action/status`
  /// subscriptions. Mutually exclusive so GoalStatusArray callbacks keep
  /// their in-order delivery (status transitions must not be observed out
  /// of order), while being decoupled from the default group so a long
  /// refresh pass cannot delay goal-status tracking.
  rclcpp::CallbackGroup::SharedPtr action_status;
};

/**
 * @brief Create the gateway's shared callback groups on @p node.
 *
 * Must be called once, from the startup thread, before the node is added to
 * a spinning executor (see the thread-safety contract above). Both groups
 * are created with `automatically_add_to_executor_with_node = true`, so
 * whichever executor the node is added to dispatches them.
 */
GatewayCallbackGroups create_gateway_callback_groups(rclcpp::Node & node);

}  // namespace ros2_medkit_gateway::ros2_common
