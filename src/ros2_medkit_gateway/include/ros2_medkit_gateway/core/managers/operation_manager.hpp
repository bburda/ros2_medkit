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

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/service_action_resolver.hpp"
#include "ros2_medkit_gateway/core/operations/operation_types.hpp"
#include "ros2_medkit_gateway/core/transports/action_transport.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"

namespace ros2_medkit_gateway {

class ResourceChangeNotifier;

using json = nlohmann::json;

/**
 * @brief Application service for ROS 2 service / action operations.
 *
 * Pure C++; ROS-side I/O is performed by the injected ServiceTransport and
 * ActionTransport adapters. Goal tracking, UUID generation, type validation
 * and component-namespace resolution remain in the manager body.
 */
class OperationManager {
 public:
  /// Severity used by the OperationManager's log_sink callback. Numeric values
  /// match rcl/rcutils log levels (30=WARN, 40=ERROR) so an adapter can
  /// forward to RCLCPP_* macros without translation. Mirrors LogManager's
  /// kLogLevelWarn / kLogLevelError contract.
  static constexpr int kLogLevelWarn = 30;
  static constexpr int kLogLevelError = 40;

  /// Sink for internal diagnostics (stuck-goal eviction, etc.). Keeps
  /// OperationManager middleware-neutral while preserving observability when
  /// the gateway adapter forwards to /rosout. May be null - the sink is then
  /// a no-op, which is the production behaviour today.
  using LogSink = std::function<void(int level, std::string_view message)>;

  /**
   * @param service_transport Concrete ServiceTransport adapter (typically Ros2ServiceTransport).
   * @param action_transport Concrete ActionTransport adapter (typically Ros2ActionTransport).
   * @param resolver Service / action lookup interface (DiscoveryManager implements it).
   *                 Must outlive this manager. May be nullptr in tests that do not exercise
   *                 the component-name resolution paths.
   * @param service_call_timeout_sec Timeout in seconds applied to every service / action call.
   * @param log_sink Optional sink for internal diagnostics. Pass nullptr to silence
   *                 stuck-goal eviction warnings (default).
   */
  OperationManager(std::shared_ptr<ServiceTransport> service_transport,
                   std::shared_ptr<ActionTransport> action_transport, ServiceActionResolver * resolver,
                   int service_call_timeout_sec = 10, LogSink log_sink = nullptr);

  ~OperationManager();

  OperationManager(const OperationManager &) = delete;
  OperationManager & operator=(const OperationManager &) = delete;
  OperationManager(OperationManager &&) = delete;
  OperationManager & operator=(OperationManager &&) = delete;

  /// Idempotent teardown. Clears tracked goals and unsubscribes from
  /// status topics. Subsequent calls are no-ops. Transport teardown is
  /// owned by the transport destructors.
  void shutdown();

  /// Set optional notifier for broadcasting operation status changes to trigger subsystem.
  void set_notifier(ResourceChangeNotifier * notifier);

  /// Call a ROS 2 service synchronously through the ServiceTransport.
  ServiceCallResult call_service(const std::string & service_path, const std::string & service_type,
                                 const json & request);

  /// Find and call a service by component and operation name.
  /// Resolves the path / type via the ServiceActionResolver when type is unset.
  ServiceCallResult call_component_service(const std::string & component_ns, const std::string & operation_name,
                                           const std::optional<std::string> & service_type, const json & request);

  /// Validate message type format (package/srv/Type or package/action/Type or package/msg/Type).
  static bool is_valid_message_type(const std::string & type);

  /// Validate UUID hex string format (32 hex characters).
  static bool is_valid_uuid_hex(const std::string & uuid_hex);

  /// Check if type is a service type (contains /srv/).
  static bool is_service_type(const std::string & type);

  /// Check if type is an action type (contains /action/).
  static bool is_action_type(const std::string & type);

  // ==================== ACTION OPERATIONS ====================

  /// Send a goal to an action server through the ActionTransport.
  ActionSendGoalResult send_action_goal(const std::string & action_path, const std::string & action_type,
                                        const json & goal, const std::string & entity_id = "");

  /// Send a goal using component namespace + operation name. Resolves through
  /// the ServiceActionResolver when type is unset.
  ActionSendGoalResult send_component_action_goal(const std::string & component_ns, const std::string & operation_name,
                                                  const std::optional<std::string> & action_type, const json & goal,
                                                  const std::string & entity_id = "");

  /// Cancel a running action goal.
  ///
  /// PRECONDITION: the caller has already resolved `goal_id` through
  /// get_tracked_goal() and taken `action_path` from the resulting
  /// ActionGoalInfo. Both HTTP entry points do exactly that and answer 404
  /// themselves when the lookup fails, so the only guard below that a
  /// contract-respecting caller can trip is the tracked-goal re-check - and
  /// only by racing the cleanup timer, which is why that one alone carries a
  /// wire classification (CancelOutcome::kNotTracked -> 404). A malformed
  /// goal_id cannot reach here at all: every key in the tracking map comes
  /// from uuid_bytes_to_hex().
  ActionCancelResult cancel_action_goal(const std::string & action_path, const std::string & goal_id);

  /// Get the result of a completed action.
  ActionGetResultResult get_action_result(const std::string & action_path, const std::string & action_type,
                                          const std::string & goal_id);

  /// Get tracked goal info by goal_id.
  std::optional<ActionGoalInfo> get_tracked_goal(const std::string & goal_id) const;

  /// List all tracked goals.
  std::vector<ActionGoalInfo> list_tracked_goals() const;

  /// Get all goals for a specific action path, sorted by created_at (newest first).
  std::vector<ActionGoalInfo> get_goals_for_action(const std::string & action_path) const;

  /// Get the most recent goal for a specific action path.
  std::optional<ActionGoalInfo> get_latest_goal_for_action(const std::string & action_path) const;

  /// Update goal status in tracking. No-op if the goal is unknown.
  void update_goal_status(const std::string & goal_id, ActionGoalStatus status);

  /// Update goal feedback in tracking. No-op if the goal is unknown.
  void update_goal_feedback(const std::string & goal_id, const json & feedback);

  /// Remove completed goals older than max_age. Forwards the unsubscribe
  /// request to the action transport for each path that becomes empty.
  void cleanup_old_goals(std::chrono::seconds max_age = std::chrono::seconds(300));

  /// Subscribe (idempotently) to action status updates. Wires the transport
  /// callback into update_goal_status() so external status changes update
  /// the tracking map on the transport's executor thread.
  void subscribe_to_action_status(const std::string & action_path);

  /// Unsubscribe from action status updates. Idempotent, and a no-op while
  /// any goal for the path is still tracked - a tracked goal without its
  /// status stream can never be reconciled by the cancel-timeout path.
  void unsubscribe_from_action_status(const std::string & action_path);

  /// Resolved service / action call budget in seconds. Every service call and
  /// every action RPC (send_goal, cancel_goal, get_result) is bounded by it,
  /// so the value the gateway actually applies is observable rather than
  /// merely configured.
  int service_call_timeout_sec() const {
    return service_call_timeout_sec_;
  }

  /// Test-only helper: inject a fully-formed ActionGoalInfo directly into the
  /// tracking map. Used by unit tests to exercise paths (e.g. stuck-goal
  /// eviction) without driving real action server traffic. Not part of the
  /// production API.
  void inject_tracked_goal_for_testing(ActionGoalInfo info);

 private:
  /// Convert UUID hex string to JSON array of byte values.
  static json uuid_hex_to_json_array(const std::string & uuid_hex);

  /// Generate a random version-4 UUID (16 bytes).
  std::array<uint8_t, 16> generate_uuid();

  /// Convert UUID bytes to JSON array.
  static json uuid_bytes_to_json_array(const std::array<uint8_t, 16> & uuid);

  /// Convert UUID bytes to lowercase hex string (no separators).
  static std::string uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid);

  /// Track a new goal in the local map.
  void track_goal(const std::string & goal_id, const std::string & action_path, const std::string & action_type,
                  const std::string & entity_id);

  /// True while at least one goal for `action_path` is tracked. Cheaper than
  /// get_goals_for_action (no copy, no sort) and safe to call while holding
  /// subscriptions_mutex_ - nothing takes goals_mutex_ before that one.
  bool has_goals_for_action(const std::string & action_path) const;

  /// Propagate a goal status change observed via the action transport into
  /// the tracking map, firing the resource-change notifier on transitions.
  void on_status_callback(const std::string & action_path, const std::string & goal_id, ActionGoalStatus status);

  /// Forward an internal diagnostic to the injected log_sink (if any).
  void emit_log(int level, std::string_view msg) const {
    if (log_sink_) {
      log_sink_(level, msg);
    }
  }

  std::shared_ptr<ServiceTransport> service_transport_;
  std::shared_ptr<ActionTransport> action_transport_;
  ServiceActionResolver * resolver_;
  ResourceChangeNotifier * notifier_ = nullptr;
  LogSink log_sink_;

  /// RNG for UUID generation. Guarded by rng_mutex_.
  std::mutex rng_mutex_;
  std::mt19937 rng_;

  /// Service / action call timeout in seconds.
  int service_call_timeout_sec_;

  /// Map of goal_id -> ActionGoalInfo for tracking active goals.
  mutable std::mutex goals_mutex_;
  std::map<std::string, ActionGoalInfo> tracked_goals_;

  /// Set of action paths the manager has asked the transport to subscribe to.
  /// Used so cleanup_old_goals can issue a single unsubscribe per path.
  mutable std::mutex subscriptions_mutex_;
  std::map<std::string, bool> subscribed_paths_;
};

}  // namespace ros2_medkit_gateway
