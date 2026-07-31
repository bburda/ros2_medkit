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

#include "ros2_medkit_gateway/ros2_common/callback_groups.hpp"

namespace ros2_medkit_gateway::ros2_common {

GatewayCallbackGroups create_gateway_callback_groups(rclcpp::Node & node) {
  GatewayCallbackGroups groups;
  groups.rpc_reentrant = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  groups.action_status = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  return groups;
}

}  // namespace ros2_medkit_gateway::ros2_common
