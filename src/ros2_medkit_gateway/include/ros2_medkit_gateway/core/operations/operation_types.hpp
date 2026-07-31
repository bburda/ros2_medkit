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
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Result of a synchronous service call.
struct ServiceCallResult {
  bool success;
  json response;
  std::string error_message;
};

/// Action goal status (matches ROS 2 action_msgs/msg/GoalStatus enum values
/// but is a neutral C++ enum that does not require action_msgs include).
enum class ActionGoalStatus : int8_t {
  UNKNOWN = 0,
  ACCEPTED = 1,
  EXECUTING = 2,
  CANCELING = 3,
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6
};

/// Convert status enum to lowercase string ("succeeded", "canceled", etc.).
std::string action_status_to_string(ActionGoalStatus status);

/// Result of sending an action goal.
struct ActionSendGoalResult {
  bool success;
  std::string goal_id;  ///< UUID hex string
  bool goal_accepted;
  std::string error_message;
};

/// Outcome of a CancelGoal round-trip (issue #576). Distinguishes "the
/// server answered" from "no answer arrived in time": a timed-out cancel has
/// an UNKNOWN outcome (the request may well have been accepted) and must not
/// be reported as a definitive rejection.
enum class CancelOutcome : uint8_t {
  kOk,                  ///< Server answered with return_code 0 (cancel accepted).
  kTimeout,             ///< No response within the budget - outcome unknown.
  kServiceUnavailable,  ///< cancel_goal service not discoverable (server gone).
  kTransportError,      ///< Null response / unknown type / exception / precondition failure.
  kErrorResponse,       ///< Server answered with return_code 1/2/3 (definitive).
};

/// Result of canceling an action goal.
struct ActionCancelResult {
  bool success = false;
  int8_t return_code = 0;  ///< 0=accepted, 1=rejected, 2=unknown_id, 3=terminated
  /// Defaults to kTransportError so any early-return path that forgets to
  /// classify itself reads as a transport failure, never as a rejection.
  CancelOutcome outcome = CancelOutcome::kTransportError;
  std::string error_message;
};

/// Result of getting an action result.
struct ActionGetResultResult {
  bool success;
  ActionGoalStatus status;
  json result;
  std::string error_message;
};

/// Tracked action goal info (stored locally).
struct ActionGoalInfo {
  std::string goal_id;
  std::string action_path;  ///< e.g., /powertrain/engine/long_calibration
  std::string action_type;  ///< e.g., example_interfaces/action/Fibonacci
  std::string entity_id;    ///< SOVD entity ID (e.g., engine) for trigger notifications
  ActionGoalStatus status;
  json last_feedback;
  std::chrono::system_clock::time_point created_at;
  std::chrono::system_clock::time_point last_update;
};

}  // namespace ros2_medkit_gateway
