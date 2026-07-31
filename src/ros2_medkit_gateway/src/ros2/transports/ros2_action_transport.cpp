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

#include "ros2_medkit_gateway/ros2/transports/ros2_action_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <random>
#include <sstream>
#include <utility>

#include "ros2_medkit_serialization/message_cleanup.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"
#include "ros2_medkit_serialization/service_action_types.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_gateway::ros2 {

namespace {

/// Convert UUID bytes to lowercase hex string.
std::string uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid) {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

/// Parse a UUID hex string into a JSON byte array (16 elements).
json uuid_hex_to_json_array(const std::string & uuid_hex) {
  json array = json::array();
  for (size_t i = 0; i + 1 < uuid_hex.length() && i < 32; i += 2) {
    int byte_val = std::stoi(uuid_hex.substr(i, 2), nullptr, 16);
    array.push_back(byte_val);
  }
  return array;
}

/// Generate a version-4 random UUID. The engine is mutex-guarded rather
/// than `thread_local`: this translation unit is part of `gateway_lib`,
/// which is also linked into the `test_gateway_plugin.so` MODULE target,
/// and TLS variables in MODULE binaries use initial-exec (TPOFF32) which
/// is incompatible with shared objects. UUID generation is one-per-goal
/// so the lock is uncontended in practice.
std::array<uint8_t, 16> generate_uuid() {
  static std::mutex rng_mutex;
  static std::mt19937 rng{std::random_device{}()};
  std::array<uint8_t, 16> uuid;
  {
    std::lock_guard<std::mutex> lock(rng_mutex);
    std::uniform_int_distribution<int> dist(0, 255);
    for (auto & byte : uuid) {
      byte = static_cast<uint8_t>(dist(rng));
    }
  }
  uuid[6] = (uuid[6] & 0x0f) | 0x40;  // version 4
  uuid[8] = (uuid[8] & 0x3f) | 0x80;  // variant bits
  return uuid;
}

json uuid_bytes_to_json_array(const std::array<uint8_t, 16> & uuid) {
  json array = json::array();
  for (auto byte : uuid) {
    array.push_back(static_cast<int>(byte));
  }
  return array;
}

ActionGoalStatus from_status_byte(int8_t status) {
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      return ActionGoalStatus::ACCEPTED;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      return ActionGoalStatus::EXECUTING;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      return ActionGoalStatus::CANCELING;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return ActionGoalStatus::SUCCEEDED;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return ActionGoalStatus::CANCELED;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return ActionGoalStatus::ABORTED;
    default:
      return ActionGoalStatus::UNKNOWN;
  }
}

}  // namespace

Ros2ActionTransport::Ros2ActionTransport(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr rpc_group,
                                         rclcpp::CallbackGroup::SharedPtr status_group)
  : node_(node)
  , rpc_group_(std::move(rpc_group))
  , status_group_(std::move(status_group))
  , serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>()) {
  RCLCPP_INFO(node_->get_logger(), "Ros2ActionTransport initialised (native serialization)");
}

Ros2ActionTransport::~Ros2ActionTransport() {
  // Mark shutdown so any in-flight callback short-circuits before touching
  // members that are about to destruct.
  shutdown_requested_.store(true);

  // Drop subscriptions first - their callbacks capture `this`, so they must
  // not fire after this destructor begins.
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    status_subscriptions_.clear();
    status_callbacks_.clear();
  }

  // Drop cached service clients to release outstanding promise references
  // before the executor tears down.
  std::unique_lock<std::shared_mutex> lock(clients_mutex_);
  action_clients_.clear();
}

Ros2ActionTransport::ActionClientSet & Ros2ActionTransport::get_or_create_clients(const std::string & action_path,
                                                                                  const std::string & action_type) {
  std::unique_lock<std::shared_mutex> lock(clients_mutex_);

  auto it = action_clients_.find(action_path);
  if (it != action_clients_.end()) {
    return it->second;
  }

  using ros2_medkit_serialization::ServiceActionTypes;

  ActionClientSet clients;
  clients.action_type = action_type;

  std::string send_goal_service = action_path + "/_action/send_goal";
  std::string send_goal_type = ServiceActionTypes::get_action_send_goal_service_type(action_type);
  clients.send_goal_client =
      compat::create_generic_service_client(node_, send_goal_service, send_goal_type, rpc_group_);

  std::string get_result_service = action_path + "/_action/get_result";
  std::string get_result_type = ServiceActionTypes::get_action_get_result_service_type(action_type);
  clients.get_result_client =
      compat::create_generic_service_client(node_, get_result_service, get_result_type, rpc_group_);

  std::string cancel_service = action_path + "/_action/cancel_goal";
  clients.cancel_goal_client =
      compat::create_generic_service_client(node_, cancel_service, "action_msgs/srv/CancelGoal", rpc_group_);

  RCLCPP_DEBUG(node_->get_logger(), "Created action clients for %s (type: %s)", action_path.c_str(),
               action_type.c_str());

  action_clients_[action_path] = std::move(clients);
  return action_clients_[action_path];
}

ActionSendGoalResult Ros2ActionTransport::send_goal(const std::string & action_path, const std::string & action_type,
                                                    const json & goal, std::chrono::duration<double> timeout) {
  ActionSendGoalResult result;
  result.success = false;
  result.goal_accepted = false;

  try {
    using ros2_medkit_serialization::ServiceActionTypes;

    auto & clients = get_or_create_clients(action_path, action_type);

    // Bound the availability wait by the caller-provided timeout so a missing
    // action server fails within the caller's budget rather than the previous
    // hardcoded 5 s. The async send_goal below has its own timeout via
    // future.wait_for().
    const auto wait_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (!clients.send_goal_client->wait_for_service(wait_timeout)) {
      result.error_message = "Action server not available: " + action_path;
      return result;
    }

    auto uuid_bytes = generate_uuid();

    json send_goal_request;
    send_goal_request["goal_id"]["uuid"] = uuid_bytes_to_json_array(uuid_bytes);
    send_goal_request["goal"] = goal.empty() || goal.is_null() ? json::object() : goal;

    std::string request_type = ServiceActionTypes::get_action_send_goal_request_type(action_type);

    RCLCPP_INFO(node_->get_logger(), "SendGoal request type: %s, JSON: %s", request_type.c_str(),
                send_goal_request.dump().c_str());

    RosMessage_Cpp ros_request = serializer_->from_json(request_type, send_goal_request);

    RCLCPP_INFO(node_->get_logger(), "Sending action goal: %s (type: %s)", action_path.c_str(), action_type.c_str());

    auto future_and_id = clients.send_goal_client->async_send_request(ros_request.data);

    const auto timeout_ms =
        std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout.count(), 0.0) * 1000.0)};
    auto future_status = future_and_id.wait_for(timeout_ms);

    if (future_status != std::future_status::ready) {
      clients.send_goal_client->remove_pending_request(future_and_id.request_id);
      ros2_medkit_serialization::destroy_ros_message(&ros_request);
      result.error_message = "Send goal timed out";
      return result;
    }

    ros2_medkit_serialization::destroy_ros_message(&ros_request);

    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Send goal returned null response";
      return result;
    }

    std::string response_type = ServiceActionTypes::get_action_send_goal_response_type(action_type);
    auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: " + response_type;
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    result.success = true;
    result.goal_accepted = response.value("accepted", false);

    if (result.goal_accepted) {
      result.goal_id = uuid_bytes_to_hex(uuid_bytes);
      RCLCPP_INFO(node_->get_logger(), "Action goal accepted with ID: %s", result.goal_id.c_str());
    } else {
      result.error_message = "Goal rejected by action server";
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Send action goal failed for '%s': %s", action_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

ActionCancelResult Ros2ActionTransport::cancel_goal(const std::string & action_path, const std::string & goal_id,
                                                    std::chrono::duration<double> timeout) {
  ActionCancelResult result;
  result.success = false;
  result.return_code = 0;

  try {
    // Cancel does not need an action_type to be valid; reuse cached clients
    // by passing whatever the cache already has if present, or a placeholder.
    std::string cached_type;
    {
      std::shared_lock<std::shared_mutex> lock(clients_mutex_);
      auto it = action_clients_.find(action_path);
      if (it != action_clients_.end()) {
        cached_type = it->second.action_type;
      }
    }

    if (cached_type.empty()) {
      // No prior clients exist for this action - we can't reliably build the
      // send_goal/get_result types, but cancel only needs the cancel client
      // with a fixed type. Use a marker; get_or_create_clients tolerates
      // any non-empty type for the cancel-only path because the cancel
      // client uses the standard action_msgs/srv/CancelGoal type.
      cached_type = "action_msgs/action/CancelOnly";
    }

    auto & clients = get_or_create_clients(action_path, cached_type);

    const auto timeout_ms =
        std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout.count(), 0.0) * 1000.0)};

    if (!clients.cancel_goal_client->wait_for_service(std::chrono::seconds(2))) {
      result.error_message = "Cancel service not available";
      return result;
    }

    json cancel_request;
    cancel_request["goal_info"]["goal_id"]["uuid"] = uuid_hex_to_json_array(goal_id);

    RosMessage_Cpp ros_request = serializer_->from_json("action_msgs/srv/CancelGoal_Request", cancel_request);

    RCLCPP_INFO(node_->get_logger(), "Canceling action goal: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    auto future_and_id = clients.cancel_goal_client->async_send_request(ros_request.data);

    auto future_status = future_and_id.wait_for(timeout_ms);
    if (future_status != std::future_status::ready) {
      clients.cancel_goal_client->remove_pending_request(future_and_id.request_id);
      ros2_medkit_serialization::destroy_ros_message(&ros_request);
      result.error_message = "Cancel request timed out";
      return result;
    }
    ros2_medkit_serialization::destroy_ros_message(&ros_request);

    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Cancel returned null response";
      return result;
    }

    auto type_info =
        ros2_medkit_serialization::TypeCache::instance().get_message_type_info("action_msgs/srv/CancelGoal_Response");
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: action_msgs/srv/CancelGoal_Response";
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    result.success = true;
    result.return_code = static_cast<int8_t>(response.value("return_code", 0));

    if (result.return_code == 0) {
      RCLCPP_INFO(node_->get_logger(), "Cancel request accepted for goal: %s", goal_id.c_str());
    } else {
      switch (result.return_code) {
        case 1:
          result.error_message = "Cancel request rejected";
          break;
        case 2:
          result.error_message = "Unknown goal ID";
          break;
        case 3:
          result.error_message = "Goal already terminated";
          break;
        default:
          result.error_message = "Unknown cancel error";
          break;
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Action cancel failed for '%s': %s", action_path.c_str(), e.what());
    result.error_message = e.what();
  }

  return result;
}

ActionGetResultResult Ros2ActionTransport::get_result(const std::string & action_path, const std::string & action_type,
                                                      const std::string & goal_id,
                                                      std::chrono::duration<double> timeout) {
  ActionGetResultResult result;
  result.success = false;
  result.status = ActionGoalStatus::UNKNOWN;

  try {
    using ros2_medkit_serialization::ServiceActionTypes;

    auto & clients = get_or_create_clients(action_path, action_type);

    if (!clients.get_result_client->wait_for_service(std::chrono::seconds(2))) {
      result.error_message = "Get result service not available";
      return result;
    }

    json get_result_request;
    get_result_request["goal_id"]["uuid"] = uuid_hex_to_json_array(goal_id);

    std::string request_type = ServiceActionTypes::get_action_get_result_request_type(action_type);

    RosMessage_Cpp ros_request = serializer_->from_json(request_type, get_result_request);

    RCLCPP_INFO(node_->get_logger(), "Getting action result: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    auto future_and_id = clients.get_result_client->async_send_request(ros_request.data);

    const auto timeout_ms =
        std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout.count(), 0.0) * 1000.0)};
    auto future_status = future_and_id.wait_for(timeout_ms);
    if (future_status != std::future_status::ready) {
      clients.get_result_client->remove_pending_request(future_and_id.request_id);
      ros2_medkit_serialization::destroy_ros_message(&ros_request);
      result.error_message = "Get result timed out";
      return result;
    }
    ros2_medkit_serialization::destroy_ros_message(&ros_request);

    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Get result returned null response";
      return result;
    }

    std::string response_type = ServiceActionTypes::get_action_get_result_response_type(action_type);
    auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: " + response_type;
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    result.success = true;

    if (response.contains("status")) {
      int status_val = response["status"].get<int>();
      result.status = static_cast<ActionGoalStatus>(status_val);
    }

    if (response.contains("result")) {
      result.result = response["result"];
    } else {
      result.result = response;
    }

    RCLCPP_INFO(node_->get_logger(), "Got action result for %s: status=%s", goal_id.c_str(),
                action_status_to_string(result.status).c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Get action result failed for '%s': %s", action_path.c_str(), e.what());
    result.error_message = e.what();
  }

  return result;
}

void Ros2ActionTransport::subscribe_status(const std::string & action_path, StatusCallback callback) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  // Idempotent: re-subscribing on the same path is a no-op (preserves the
  // first registered callback, matching today's manager-level behaviour).
  if (status_subscriptions_.count(action_path) > 0) {
    return;
  }

  status_callbacks_[action_path] = std::move(callback);

  std::string status_topic = action_path + "/_action/status";

  auto cb = [this, action_path](const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
    if (shutdown_requested_.load(std::memory_order_acquire)) {
      return;
    }
    on_status_msg(action_path, msg);
  };

  // Dispatch on the shared MutuallyExclusive status group (issue #575):
  // in-order delivery per subscription is preserved, but status tracking is
  // no longer serialized behind the node's default group.
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = status_group_;
  auto subscription = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      status_topic, rclcpp::QoS(10).best_effort(), cb, sub_options);

  status_subscriptions_[action_path] = subscription;
  RCLCPP_INFO(node_->get_logger(), "Subscribed to action status: %s", status_topic.c_str());
}

void Ros2ActionTransport::unsubscribe_status(const std::string & action_path) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  auto it = status_subscriptions_.find(action_path);
  if (it != status_subscriptions_.end()) {
    status_subscriptions_.erase(it);
    status_callbacks_.erase(action_path);
    RCLCPP_INFO(node_->get_logger(), "Unsubscribed from action status: %s/_action/status", action_path.c_str());
  }
}

void Ros2ActionTransport::on_status_msg(const std::string & action_path,
                                        const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
  StatusCallback callback;
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto it = status_callbacks_.find(action_path);
    if (it == status_callbacks_.end()) {
      return;
    }
    callback = it->second;
  }

  if (!callback) {
    return;
  }

  for (const auto & status : msg->status_list) {
    std::string goal_id = uuid_bytes_to_hex(status.goal_info.goal_id.uuid);
    callback(action_path, goal_id, from_status_byte(status.status));
  }
}

}  // namespace ros2_medkit_gateway::ros2
