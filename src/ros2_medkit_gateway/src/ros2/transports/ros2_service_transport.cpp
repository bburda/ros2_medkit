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

#include "ros2_medkit_gateway/ros2/transports/ros2_service_transport.hpp"

#include <chrono>
#include <utility>

#include "ros2_medkit_serialization/message_cleanup.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"
#include "ros2_medkit_serialization/service_action_types.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_gateway::ros2 {

Ros2ServiceTransport::Ros2ServiceTransport(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr rpc_group)
  : node_(node)
  , rpc_group_(std::move(rpc_group))
  , serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>()) {
  RCLCPP_INFO(node_->get_logger(), "Ros2ServiceTransport initialised (native serialization)");
}

Ros2ServiceTransport::~Ros2ServiceTransport() {
  // Drop all cached clients so any outstanding promise references release
  // before rclcpp shuts the underlying executor down. The compat shim
  // (Humble) terminates if a future still references shared state at
  // destruction; the wiring contract guarantees this destructor runs while
  // the executor is still spinnable.
  std::unique_lock<std::shared_mutex> lock(clients_mutex_);
  clients_.clear();
}

std::string Ros2ServiceTransport::make_client_key(const std::string & service_path, const std::string & service_type) {
  return service_path + "|" + service_type;
}

compat::GenericServiceClient::SharedPtr Ros2ServiceTransport::get_or_create_client(const std::string & service_path,
                                                                                   const std::string & service_type) {
  const std::string key = make_client_key(service_path, service_type);

  {
    std::shared_lock<std::shared_mutex> lock(clients_mutex_);
    auto it = clients_.find(key);
    if (it != clients_.end()) {
      return it->second;
    }
  }

  std::unique_lock<std::shared_mutex> lock(clients_mutex_);
  auto it = clients_.find(key);
  if (it != clients_.end()) {
    return it->second;
  }

  auto client = compat::create_generic_service_client(node_, service_path, service_type, rpc_group_);
  clients_[key] = client;

  RCLCPP_DEBUG(node_->get_logger(), "Created generic client for %s (%s)", service_path.c_str(), service_type.c_str());

  return client;
}

ServiceCallResult Ros2ServiceTransport::call(const std::string & service_path, const std::string & service_type,
                                             const json & request, std::chrono::duration<double> timeout) {
  using ros2_medkit_serialization::ServiceActionTypes;
  ServiceCallResult result;

  try {
    auto client = get_or_create_client(service_path, service_type);

    // Bound the availability wait by the caller-provided timeout so a missing
    // service fails within the caller's budget rather than the previous
    // hardcoded 5 s. The async call below has its own independent timeout
    // via future.wait_for().
    const auto wait_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (!client->wait_for_service(wait_timeout)) {
      result.success = false;
      result.error_message = "Service not available: " + service_path;
      return result;
    }

    std::string request_type = ServiceActionTypes::get_service_request_type(service_type);
    std::string response_type = ServiceActionTypes::get_service_response_type(service_type);

    json request_data = request.empty() || request.is_null() ? json::object() : request;

    try {
      json defaults = serializer_->get_defaults(request_type);
      for (auto it = request_data.begin(); it != request_data.end(); ++it) {
        defaults[it.key()] = it.value();
      }
      request_data = defaults;
    } catch (const ros2_medkit_serialization::TypeNotFoundError &) {
      // If we can't get defaults, just use provided data.
    }

    RosMessage_Cpp ros_request = serializer_->from_json(request_type, request_data);

    RCLCPP_INFO(node_->get_logger(), "Calling service: %s (type: %s)", service_path.c_str(), service_type.c_str());

    auto future_and_id = client->async_send_request(ros_request.data);

    const auto timeout_secs =
        std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout.count(), 0.0) * 1000.0)};
    auto future_status = future_and_id.wait_for(timeout_secs);

    if (future_status != std::future_status::ready) {
      client->remove_pending_request(future_and_id.request_id);
      ros2_medkit_serialization::destroy_ros_message(&ros_request);
      result.success = false;
      // Format the wait budget with millisecond precision so a sub-second
      // timeout (e.g. 0.1s) is not silently rendered as "0s".
      const auto timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_secs).count();
      result.error_message = "Service call timed out (" + std::to_string(timeout_ms) + "ms): " + service_path;
      return result;
    }

    ros2_medkit_serialization::destroy_ros_message(&ros_request);

    auto response_ptr = future_and_id.get();
    if (response_ptr != nullptr) {
      try {
        auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
        if (type_info != nullptr) {
          result.response = serializer_->to_json(type_info, response_ptr.get());
          result.success = true;
          RCLCPP_DEBUG(node_->get_logger(), "Service call succeeded: %s", result.response.dump().c_str());
        } else {
          result.success = false;
          result.error_message = "Unknown response type: " + response_type;
        }
      } catch (const std::exception & e) {
        result.success = false;
        result.error_message = std::string("Failed to deserialize response: ") + e.what();
      }
    } else {
      result.success = false;
      result.error_message = "Service returned null response";
    }

  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Type not found for service '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = std::string("Unknown service type: ") + e.what();
  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Serialization failed for service '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = std::string("Invalid request format: ") + e.what();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed for '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

}  // namespace ros2_medkit_gateway::ros2
