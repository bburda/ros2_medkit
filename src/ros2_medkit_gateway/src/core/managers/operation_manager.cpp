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

#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <utility>

#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

namespace ros2_medkit_gateway {

namespace {

/// UUID hex string length (16 bytes = 32 hex characters).
constexpr size_t kUuidHexLength = 32;

/// Multiplier applied to max_age before force-evicting a non-terminal goal
/// whose last_update has stopped advancing. Two times the normal expiry gives
/// well-behaved action servers ample headroom to publish a terminal state
/// while still bounding tracked_goals_ when an action server crashes.
constexpr int kStuckGoalAgeMultiplier = 2;

bool is_terminal_status(ActionGoalStatus status) {
  return status == ActionGoalStatus::SUCCEEDED || status == ActionGoalStatus::CANCELED ||
         status == ActionGoalStatus::ABORTED;
}

}  // namespace

OperationManager::OperationManager(std::shared_ptr<ServiceTransport> service_transport,
                                   std::shared_ptr<ActionTransport> action_transport, ServiceActionResolver * resolver,
                                   int service_call_timeout_sec, LogSink log_sink)
  : service_transport_(std::move(service_transport))
  , action_transport_(std::move(action_transport))
  , resolver_(resolver)
  // notifier_ defaults to nullptr (initialized in-class)
  , log_sink_(std::move(log_sink))
  , rng_(std::random_device{}())
  , service_call_timeout_sec_(service_call_timeout_sec) {
}

OperationManager::~OperationManager() {
  shutdown();
}

void OperationManager::shutdown() {
  // Drop status subscriptions through the transport so the transport can
  // tear down its underlying ROS state safely (idempotent on the transport
  // side too).
  std::map<std::string, bool> paths_to_drop;
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    paths_to_drop.swap(subscribed_paths_);
  }
  for (const auto & [path, _] : paths_to_drop) {
    if (action_transport_) {
      action_transport_->unsubscribe_status(path);
    }
  }

  // Clear tracked goals.
  std::lock_guard<std::mutex> lock(goals_mutex_);
  tracked_goals_.clear();
}

void OperationManager::set_notifier(ResourceChangeNotifier * notifier) {
  notifier_ = notifier;
}

bool OperationManager::is_valid_message_type(const std::string & type) {
  static const std::regex type_regex(R"(^[a-zA-Z_][a-zA-Z0-9_]*/(srv|action|msg)/[a-zA-Z_][a-zA-Z0-9_]*$)");
  return std::regex_match(type, type_regex);
}

bool OperationManager::is_service_type(const std::string & type) {
  return type.find("/srv/") != std::string::npos;
}

bool OperationManager::is_action_type(const std::string & type) {
  return type.find("/action/") != std::string::npos;
}

bool OperationManager::is_valid_uuid_hex(const std::string & uuid_hex) {
  if (uuid_hex.length() != kUuidHexLength) {
    return false;
  }
  return std::all_of(uuid_hex.begin(), uuid_hex.end(), [](char c) {
    return std::isxdigit(static_cast<unsigned char>(c));
  });
}

ServiceCallResult OperationManager::call_service(const std::string & service_path, const std::string & service_type,
                                                 const json & request) {
  if (!service_transport_) {
    ServiceCallResult result;
    result.success = false;
    result.error_message = "ServiceTransport not configured";
    return result;
  }
  return service_transport_->call(service_path, service_type, request,
                                  std::chrono::duration<double>(service_call_timeout_sec_));
}

ServiceCallResult OperationManager::call_component_service(const std::string & component_ns,
                                                           const std::string & operation_name,
                                                           const std::optional<std::string> & service_type,
                                                           const json & request) {
  ServiceCallResult result;
  std::string resolved_type;
  std::string service_path;

  if (service_type.has_value() && !service_type->empty()) {
    resolved_type = *service_type;
    service_path = component_ns;
    if (!service_path.empty() && service_path.back() != '/') {
      service_path += "/";
    }
    service_path += operation_name;
  } else {
    if (resolver_ == nullptr) {
      result.success = false;
      result.error_message = "Service not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    auto service_info = resolver_->find_service(component_ns, operation_name);
    if (!service_info.has_value()) {
      result.success = false;
      result.error_message = "Service not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = service_info->type;
    service_path = service_info->full_path;
  }

  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid service type format: " + resolved_type;
    return result;
  }

  if (!is_service_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not a service type: " + resolved_type;
    return result;
  }

  return call_service(service_path, resolved_type, request);
}

// ==================== ACTION OPERATIONS ====================

std::array<uint8_t, 16> OperationManager::generate_uuid() {
  std::lock_guard<std::mutex> lock(rng_mutex_);
  std::array<uint8_t, 16> uuid;
  std::uniform_int_distribution<int> dist(0, 255);
  for (auto & byte : uuid) {
    byte = static_cast<uint8_t>(dist(rng_));
  }
  // Set version 4 (random UUID).
  uuid[6] = (uuid[6] & 0x0f) | 0x40;
  // Set variant bits.
  uuid[8] = (uuid[8] & 0x3f) | 0x80;
  return uuid;
}

json OperationManager::uuid_bytes_to_json_array(const std::array<uint8_t, 16> & uuid) {
  json array = json::array();
  for (auto byte : uuid) {
    array.push_back(static_cast<int>(byte));
  }
  return array;
}

json OperationManager::uuid_hex_to_json_array(const std::string & uuid_hex) {
  json array = json::array();
  for (size_t i = 0; i + 1 < uuid_hex.length() && i < kUuidHexLength; i += 2) {
    int byte_val = std::stoi(uuid_hex.substr(i, 2), nullptr, 16);
    array.push_back(byte_val);
  }
  return array;
}

std::string OperationManager::uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid) {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

void OperationManager::inject_tracked_goal_for_testing(ActionGoalInfo info) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  tracked_goals_[info.goal_id] = std::move(info);
}

void OperationManager::track_goal(const std::string & goal_id, const std::string & action_path,
                                  const std::string & action_type, const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  ActionGoalInfo info;
  info.goal_id = goal_id;
  info.action_path = action_path;
  info.action_type = action_type;
  info.entity_id = entity_id;
  info.status = ActionGoalStatus::ACCEPTED;
  info.created_at = std::chrono::system_clock::now();
  info.last_update = info.created_at;
  tracked_goals_[goal_id] = info;
}

ActionSendGoalResult OperationManager::send_action_goal(const std::string & action_path,
                                                        const std::string & action_type, const json & goal,
                                                        const std::string & entity_id) {
  ActionSendGoalResult result;
  result.success = false;
  result.goal_accepted = false;

  if (!action_transport_) {
    result.error_message = "ActionTransport not configured";
    return result;
  }

  result = action_transport_->send_goal(action_path, action_type, goal,
                                        std::chrono::duration<double>(service_call_timeout_sec_));

  if (result.success && result.goal_accepted && !result.goal_id.empty()) {
    track_goal(result.goal_id, action_path, action_type, entity_id);
    subscribe_to_action_status(action_path);
    update_goal_status(result.goal_id, ActionGoalStatus::EXECUTING);
  }

  return result;
}

ActionSendGoalResult OperationManager::send_component_action_goal(const std::string & component_ns,
                                                                  const std::string & operation_name,
                                                                  const std::optional<std::string> & action_type,
                                                                  const json & goal, const std::string & entity_id) {
  ActionSendGoalResult result;
  std::string resolved_type;
  std::string action_path;

  if (action_type.has_value() && !action_type->empty()) {
    resolved_type = *action_type;
    action_path = component_ns;
    if (!action_path.empty() && action_path.back() != '/') {
      action_path += "/";
    }
    action_path += operation_name;
  } else {
    if (resolver_ == nullptr) {
      result.success = false;
      result.error_message = "Action not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    auto action_info = resolver_->find_action(component_ns, operation_name);
    if (!action_info.has_value()) {
      result.success = false;
      result.error_message = "Action not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = action_info->type;
    action_path = action_info->full_path;
  }

  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid action type format: " + resolved_type;
    return result;
  }

  if (!is_action_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not an action type: " + resolved_type;
    return result;
  }

  return send_action_goal(action_path, resolved_type, goal, entity_id);
}

ActionCancelResult OperationManager::cancel_action_goal(const std::string & action_path, const std::string & goal_id) {
  ActionCancelResult result;
  result.success = false;
  result.return_code = 0;

  // Each guard classifies itself: they all short-circuit before the request
  // reaches the action server, so riding the struct's kTransportError default
  // would tell the client the action server is unavailable when it is fine.
  if (!is_valid_uuid_hex(goal_id)) {
    result.outcome = CancelOutcome::kInvalidRequest;
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  auto goal_info = get_tracked_goal(goal_id);
  if (!goal_info) {
    result.outcome = CancelOutcome::kNotTracked;
    result.error_message = "Unknown goal_id - not tracked";
    return result;
  }

  if (!action_transport_) {
    // A missing transport IS a gateway-side transport failure - the only
    // guard for which the default classification is the right one.
    result.outcome = CancelOutcome::kTransportError;
    result.error_message = "ActionTransport not configured";
    return result;
  }

  // Cancel gets the same budget as every other action RPC. A timed-out
  // cancel is no longer conflated with a rejection (issue #576): the
  // transport reports it as CancelOutcome::kTimeout and the HTTP layer
  // reconciles it against the /_action/status stream, so there is no need
  // for the old 15s floor that papered over slow dispatch.
  result =
      action_transport_->cancel_goal(action_path, goal_id, std::chrono::duration<double>(service_call_timeout_sec_));

  if (result.outcome == CancelOutcome::kOk) {
    update_goal_status(goal_id, ActionGoalStatus::CANCELING);
  }
  return result;
}

ActionGetResultResult OperationManager::get_action_result(const std::string & action_path,
                                                          const std::string & action_type,
                                                          const std::string & goal_id) {
  ActionGetResultResult result;
  result.success = false;
  result.status = ActionGoalStatus::UNKNOWN;

  if (!is_valid_uuid_hex(goal_id)) {
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  if (!action_transport_) {
    result.error_message = "ActionTransport not configured";
    return result;
  }

  result = action_transport_->get_result(action_path, action_type, goal_id,
                                         std::chrono::duration<double>(service_call_timeout_sec_));

  if (result.success) {
    update_goal_status(goal_id, result.status);
  }
  return result;
}

std::optional<ActionGoalInfo> OperationManager::get_tracked_goal(const std::string & goal_id) const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<ActionGoalInfo> OperationManager::list_tracked_goals() const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  std::vector<ActionGoalInfo> goals;
  goals.reserve(tracked_goals_.size());
  for (const auto & [id, info] : tracked_goals_) {
    goals.push_back(info);
  }
  return goals;
}

std::vector<ActionGoalInfo> OperationManager::get_goals_for_action(const std::string & action_path) const {
  std::vector<ActionGoalInfo> goals;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    for (const auto & [id, info] : tracked_goals_) {
      if (info.action_path == action_path) {
        goals.push_back(info);
      }
    }
  }
  std::sort(goals.begin(), goals.end(), [](const ActionGoalInfo & a, const ActionGoalInfo & b) {
    return a.created_at > b.created_at;
  });
  return goals;
}

bool OperationManager::has_goals_for_action(const std::string & action_path) const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  return std::any_of(tracked_goals_.begin(), tracked_goals_.end(), [&action_path](const auto & entry) {
    return entry.second.action_path == action_path;
  });
}

std::optional<ActionGoalInfo> OperationManager::get_latest_goal_for_action(const std::string & action_path) const {
  auto goals = get_goals_for_action(action_path);
  if (goals.empty()) {
    return std::nullopt;
  }
  return goals.front();
}

void OperationManager::update_goal_status(const std::string & goal_id, ActionGoalStatus status) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it == tracked_goals_.end()) {
    return;
  }
  // A terminal state is final in the ROS 2 action protocol, and every caller
  // of this method is an RPC completion (goal accepted -> EXECUTING, cancel
  // accepted -> CANCELING, get_result -> reported status) that can land after
  // the /_action/status stream has already delivered the terminal state. The
  // guard lives here rather than at the cancel call site because the hazard
  // is the same for all three: nothing ever corrects a backwards write - the
  // server publishes no further status frame, so the goal would be reported
  // as still running until the stuck-goal path force-evicts it with a false
  // "action server crashed" warning. The stream stays the authority; it
  // writes directly under this lock in on_status_callback and is deliberately
  // not routed through here.
  if (is_terminal_status(it->second.status)) {
    return;
  }
  it->second.status = status;
  it->second.last_update = std::chrono::system_clock::now();
}

void OperationManager::update_goal_feedback(const std::string & goal_id, const json & feedback) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    it->second.last_feedback = feedback;
    it->second.last_update = std::chrono::system_clock::now();
  }
}

void OperationManager::cleanup_old_goals(std::chrono::seconds max_age) {
  std::set<std::string> actions_to_check;
  // Goals whose status never reached a terminal state but whose last_update
  // has frozen for an excessive period (e.g. action server crashed). Collected
  // under the mutex and logged after release to avoid holding the lock across
  // RCLCPP_WARN, which can take a non-trivial amount of time.
  std::vector<std::pair<std::string, ActionGoalStatus>> stuck_evictions;

  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto now = std::chrono::system_clock::now();
    const auto stuck_age = max_age * kStuckGoalAgeMultiplier;

    for (auto it = tracked_goals_.begin(); it != tracked_goals_.end();) {
      const auto age = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update);

      if (is_terminal_status(it->second.status)) {
        if (age > max_age) {
          actions_to_check.insert(it->second.action_path);
          it = tracked_goals_.erase(it);
          continue;
        }
      } else if (age > stuck_age) {
        // Non-terminal goal that has gone silent past 2x max_age - assume
        // the action server crashed without publishing a terminal state.
        // Force-evict to keep tracked_goals_ bounded.
        stuck_evictions.emplace_back(it->first, it->second.status);
        actions_to_check.insert(it->second.action_path);
        it = tracked_goals_.erase(it);
        continue;
      }
      ++it;
    }
  }

  for (const auto & [goal_id, status] : stuck_evictions) {
    std::ostringstream msg;
    msg << "Evicting stuck action goal '" << goal_id << "' (status=" << action_status_to_string(status)
        << ") - last_update older than " << kStuckGoalAgeMultiplier << "x max_age";
    emit_log(kLogLevelWarn, msg.str());
  }

  for (const auto & action_path : actions_to_check) {
    auto remaining_goals = get_goals_for_action(action_path);
    if (remaining_goals.empty()) {
      unsubscribe_from_action_status(action_path);
    }
  }
}

void OperationManager::subscribe_to_action_status(const std::string & action_path) {
  if (!action_transport_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    if (subscribed_paths_.count(action_path) > 0) {
      return;
    }
    subscribed_paths_[action_path] = true;
  }

  action_transport_->subscribe_status(
      action_path, [this](const std::string & path, const std::string & goal_id, ActionGoalStatus status) {
        on_status_callback(path, goal_id, status);
      });
}

void OperationManager::unsubscribe_from_action_status(const std::string & action_path) {
  bool was_subscribed = false;
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    // cleanup_old_goals reads the goal count first and unsubscribes after, so
    // a goal sent in between would keep a subscription that is about to be
    // destroyed: tracked, but with no status stream. Since #576 the stream
    // decides 204-vs-504 for a timed-out cancel, so re-check under the same
    // lock that guards the erase and let a live goal veto the unsubscribe.
    if (has_goals_for_action(action_path)) {
      return;
    }
    auto it = subscribed_paths_.find(action_path);
    if (it != subscribed_paths_.end()) {
      subscribed_paths_.erase(it);
      was_subscribed = true;
    }
  }
  if (was_subscribed && action_transport_) {
    action_transport_->unsubscribe_status(action_path);
  }
}

void OperationManager::on_status_callback(const std::string & action_path, const std::string & goal_id,
                                          ActionGoalStatus status) {
  ActionGoalStatus previous = ActionGoalStatus::UNKNOWN;
  std::string entity_id;
  bool changed = false;
  bool match = false;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto it = tracked_goals_.find(goal_id);
    if (it == tracked_goals_.end() || it->second.action_path != action_path) {
      return;
    }
    match = true;
    previous = it->second.status;
    if (previous != status) {
      it->second.status = status;
      it->second.last_update = std::chrono::system_clock::now();
      changed = true;
      entity_id = it->second.entity_id;
    }
  }

  if (match && changed && notifier_ != nullptr) {
    json goal_json;
    goal_json["goal_id"] = goal_id;
    goal_json["action_path"] = action_path;
    goal_json["status"] = action_status_to_string(status);
    notifier_->notify("operations", entity_id, goal_id, goal_json);
  }
}

}  // namespace ros2_medkit_gateway
