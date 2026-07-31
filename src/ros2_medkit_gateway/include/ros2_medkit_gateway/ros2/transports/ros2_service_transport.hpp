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

#include <chrono>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>

#include "ros2_medkit_gateway/compat/generic_client_compat.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing ServiceTransport.
 *
 * Owns the GenericServiceClient cache and the JsonSerializer that
 * OperationManager previously held directly. Service-call body lifted
 * verbatim from the legacy OperationManager::call_service.
 */
class Ros2ServiceTransport : public ServiceTransport {
 public:
  /**
   * @param node Non-owning ROS node used for client creation.
   * @param rpc_group Callback group the clients' response callbacks are
   *        dispatched on - the shared Reentrant RPC group from
   *        ros2_common::create_gateway_callback_groups() (issue #575), so a
   *        response can be delivered while default-group callbacks run. The
   *        transport keeps the shared_ptr alive for its own lifetime (the
   *        node only holds a weak reference to the group).
   */
  Ros2ServiceTransport(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr rpc_group);

  ~Ros2ServiceTransport() override;

  Ros2ServiceTransport(const Ros2ServiceTransport &) = delete;
  Ros2ServiceTransport & operator=(const Ros2ServiceTransport &) = delete;
  Ros2ServiceTransport(Ros2ServiceTransport &&) = delete;
  Ros2ServiceTransport & operator=(Ros2ServiceTransport &&) = delete;

  ServiceCallResult call(const std::string & service_path, const std::string & service_type, const json & request,
                         std::chrono::duration<double> timeout) override;

 private:
  static std::string make_client_key(const std::string & service_path, const std::string & service_type);

  compat::GenericServiceClient::SharedPtr get_or_create_client(const std::string & service_path,
                                                               const std::string & service_type);

  rclcpp::Node * node_;
  /// Shared Reentrant group for response dispatch; owned here because the
  /// node keeps only a weak reference to its callback groups.
  rclcpp::CallbackGroup::SharedPtr rpc_group_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  mutable std::shared_mutex clients_mutex_;
  std::map<std::string, compat::GenericServiceClient::SharedPtr> clients_;
};

}  // namespace ros2_medkit_gateway::ros2
