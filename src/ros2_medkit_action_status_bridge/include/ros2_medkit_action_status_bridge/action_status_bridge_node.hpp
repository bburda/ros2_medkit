// Copyright 2026 mfaferek93, bburda
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
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

namespace ros2_medkit_action_status_bridge {

/// Bridge node that turns terminal ROS 2 action goal states into FaultManager
/// faults. Generic across every action: it observes the per-action
/// `/<action>/_action/status` topic (`action_msgs/msg/GoalStatusArray`) that
/// every action server publishes, so no per-project code is needed.
///
/// This catches the authoritative "the goal failed" verdict that neither the
/// /diagnostics bridge nor the /rosout log bridge can see - e.g. a Nav2
/// NavigateToPose aborting or a MoveIt MoveGroup goal aborting. The *reason*
/// (action-specific error code in the result) is a separate enrichment concern;
/// this bridge delivers the generic terminal status.
///
/// Status mapping (action_msgs/msg/GoalStatus):
///   - ABORTED (6)  -> fault (severity configurable, default ERROR), UNLESS the
///                     goal was preempted - see `goal_was_preempted`
///   - CANCELED (5) -> fault only if canceled_is_fault (usually intentional)
///   - SUCCEEDED (4)-> PASSED (heals the per-action fault code) if enabled
///
/// Fault state is per-ACTION, not per-goal: every message is scanned for the net
/// state of the whole GoalStatusArray and a fault is raised/healed only on the
/// action-level transition. See `derive_state`.
///
/// PREEMPTION IS NOT A FAILURE, AND ABORTED IS THE ONLY STATUS IT EVER WEARS.
/// A server that terminates a running goal because a newer one displaced it has
/// no separate status to say so: `action_msgs/GoalStatus` has no PREEMPTED, and
/// the canonical single-goal server (`nav2_util::SimpleActionServer::
/// accept_pending_goal`) calls `current_handle_->abort()` on the goal it is
/// dropping. So CANCELED never fires for a preemption and `canceled_is_fault`
/// cannot suppress one. The one witness that survives on the status topic is the
/// ARRAY the abort is published in: the displacing goal is accepted (and, for a
/// server returning ACCEPT_AND_EXECUTE, already EXECUTING) and its status is
/// published BEFORE the old goal is aborted, so the array carrying the abort
/// also carries its live successor. `goal_was_preempted` reads exactly that.
class ActionStatusBridgeNode : public rclcpp::Node {
 public:
  explicit ActionStatusBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Resets the rescan timer and status subscriptions before the rest of the node
  // is destroyed: their callbacks capture `this`, so firing on a partially
  // destroyed object would crash (subscription destructor pattern).
  ~ActionStatusBridgeNode() override;
  ActionStatusBridgeNode(const ActionStatusBridgeNode &) = delete;
  ActionStatusBridgeNode & operator=(const ActionStatusBridgeNode &) = delete;
  ActionStatusBridgeNode(ActionStatusBridgeNode &&) = delete;
  ActionStatusBridgeNode & operator=(ActionStatusBridgeNode &&) = delete;

  /// Net fault state of an action, derived from a whole GoalStatusArray.
  ///   - kUnknown: no terminal goal in the array yet (no transition)
  ///   - kHealthy: array has terminal goals and none of them are failing
  ///   - kFailed:  at least one goal is failing (ABORTED, or CANCELED when
  ///               canceled_is_fault)
  enum class ActionState { kUnknown, kHealthy, kFailed };

  /// Derive the action name from a `/<action>/_action/status` topic name.
  /// Returns empty when the topic is not an action status topic.
  static std::string action_name_from_status_topic(const std::string & topic);

  /// Build a node FQN from a publisher endpoint's node name + namespace, or ""
  /// if unresolved. During DDS discovery rcl reports the placeholders
  /// "_NODE_NAME_UNKNOWN_" / "_NODE_NAMESPACE_UNKNOWN_" before the real name
  /// propagates; those (and an empty name) are treated as unresolved so a fault
  /// is never attributed to the placeholder.
  static std::string server_fqn_from_endpoint(const std::string & node_name, const std::string & node_namespace);

  /// Build the fault code for an action name and terminal status.
  /// Format: <PREFIX>_<ACTION>_<ABORTED|CANCELED>, charset/length per medkit
  /// rules. `canceled` selects the CANCELED suffix.
  std::string fault_code_for(const std::string & action_name, bool canceled) const;

  /// Lowercase hex of a 16-byte goal UUID, for dedup keys and short display.
  static std::string uuid_to_hex(const std::array<uint8_t, 16> & uuid);

  /// Why one goal in an array counts (or does not count) towards the action's
  /// failure. Recorded per goal id the FIRST time that goal is seen terminal.
  enum class GoalVerdict { kNotFailing, kFailingAborted, kFailingCanceled };

  /// Per-action memory of the goals whose terminal status has already been
  /// judged, keyed by goal-id hex. It exists because the preemption test reads
  /// the array a goal terminated IN, and that array keeps changing afterwards:
  /// a GENUINE abort acquires a newer active successor as soon as the client
  /// retries (measured live at ~1 s), and re-judging it then would read the
  /// retry as a preemption and heal the fault the abort had just raised. A
  /// verdict is therefore taken once, at the terminal transition, and only
  /// forgotten when the goal ages out of the array.
  using GoalLedger = std::map<std::string, GoalVerdict>;

  /// True when the ABORTED goal at `index` was displaced by a newer goal rather
  /// than failing on its own: the goal with the smallest goal_info.stamp
  /// strictly greater than this one's exists in the SAME array and is still
  /// active (ACCEPTED / EXECUTING / CANCELING).
  ///
  /// The IMMEDIATE successor is the test, not "any newer active goal", so that a
  /// cold start against the latched array reads correctly: a snapshot holding
  /// several old genuine aborts plus one goal executing now would classify all
  /// of them as preempted under the looser test, whereas each of those aborts is
  /// in fact followed by another TERMINAL goal and only the last one is followed
  /// by the live one.
  static bool goal_was_preempted(const action_msgs::msg::GoalStatusArray & msg, size_t index);

  /// Scan a whole GoalStatusArray and return the action-level net state, judging
  /// each goal's terminal status once and remembering the verdict in `ledger`
  /// (which is also pruned to the goals still present in this array, so an
  /// aged-out failure stops counting exactly as it did before). Order of the
  /// goals in the array does not affect the result (any failing goal wins).
  /// `canceled_is_fault` decides whether CANCELED counts as failing;
  /// `preempted_is_fault` true restores the pre-preemption-aware behaviour in
  /// which every ABORTED counts.
  static ActionState derive_state(const action_msgs::msg::GoalStatusArray & msg, bool canceled_is_fault,
                                  bool preempted_is_fault, GoalLedger & ledger);

  /// True when every failing goal recorded in `ledger` failed by CANCELED, so
  /// the emitted fault code's suffix matches the description. ABORTED dominates
  /// if both occur; false when nothing is failing.
  static bool failure_is_cancel(const GoalLedger & ledger);

  /// Update the desired per-action state from a message and reconcile it,
  /// acting on the transition only. Returns the state that was reported on
  /// (kFailed on raise, kHealthy on heal) or kUnknown when nothing was reported
  /// - including when the report could not be delivered yet (the FaultManager
  /// service is not discovered), in which case the transition stays pending and
  /// is retried on the next rescan. A null `reporter` is the unit-test seam:
  /// delivery is a no-op treated as successful, so the transition decision and
  /// stored per-action state still update and tests assert on the return value.
  ActionState apply_message(const std::string & action_name, const action_msgs::msg::GoalStatusArray & msg,
                            ros2_medkit_fault_reporter::FaultReporter * reporter);

 private:
  void rescan_actions();
  void status_callback(const std::string & action_name, const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg);

  /// Deliver the pending raise/heal for one action when the desired state
  /// differs from what the FaultManager was last told AND the report is
  /// deliverable (a null reporter is the test seam, treated as deliverable; a
  /// real reporter must have a discovered service). Commits `last_reported_state_`
  /// only after delivery, so a report that cannot go out yet stays pending and
  /// is retried later instead of being silently dropped. Returns the state
  /// reported on this call (kFailed/kHealthy) or kUnknown when nothing was sent.
  ActionState reconcile(const std::string & action_name, ros2_medkit_fault_reporter::FaultReporter * reporter);

  /// Re-attempt every action whose desired state has not been delivered to the
  /// FaultManager yet, then arm/disarm the retry timer. Driven from the fast
  /// retry timer (and the rescan as a backstop) so a report deferred during a
  /// discovery window is delivered within one short tick of the service
  /// appearing - keeping the FaultManager freeze-frame snapshot contemporaneous.
  void reconcile_pending();

  /// Arm the fast retry timer when at least one transition is undelivered, and
  /// disarm it once everything is delivered, so an idle bridge does no periodic
  /// work. Caller must not hold state_mutex_.
  void update_retry_timer();

  /// Get (creating on first use) the FaultReporter for an action. The reporter's
  /// source_id is fixed when first created: the resolved server FQN if discovery
  /// has settled, otherwise the action name as a fallback so the fault still fires
  /// on time. It is never re-attributed afterwards (reporting_sources is
  /// append-only and the per-entity scope filter is strict-AND).
  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & action_name);

  /// Resolve the action server's node FQN from its status-topic publisher, for
  /// use as the fault source_id so faults associate with the gateway's SOVD
  /// entity. Returns "" if no publisher with a discovery-resolved node name is
  /// visible yet (the caller falls back and re-resolves later).
  std::string server_fqn_for_action(const std::string & action_name);

  /// Returns true if this (goal, status) pair was not logged before, marking it
  /// logged. Bounded to avoid unbounded growth. Suppresses duplicate LOG lines
  /// only; never gates the action-level state transition.
  bool mark_logged(const std::string & goal_status_key);

  bool action_is_eligible(const std::string & action_name) const;

  void load_parameters();

  /// Drop subscriptions, reporters and per-action state for actions whose status
  /// topic has vanished from `present_topics`.
  void prune_vanished(const std::map<std::string, std::string> & present_topics);

  static std::string to_upper_snake(const std::string & in, size_t max_len);

  rclcpp::TimerBase::SharedPtr rescan_timer_;
  // Fast, lightweight timer that retries undelivered reports. Decoupled from the
  // (slow, expensive) rescan so a fault deferred because the FaultManager service
  // was not yet discovered is delivered within one short tick of it appearing -
  // keeping the FaultManager's freeze-frame snapshot as close to the event as the
  // discovery floor allows. See the delivery-latency note in the README.
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> subs_;

  // Per-action FaultReporter, created lazily on the first report. The source_id is
  // fixed at creation (server FQN if resolved, else the action-name fallback) and
  // never changed. Guarded by reporters_mutex_.
  std::map<std::string, std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter>> reporters_;
  std::mutex reporters_mutex_;

  // Desired (latest observed) action-level state derived from status messages:
  // the source of truth the bridge tries to make the FaultManager reflect.
  // `canceled` records whether a failure was a CANCELED (vs ABORTED), to pick
  // the right fault code when the report is delivered later.
  struct DesiredState {
    ActionState net{ActionState::kUnknown};
    bool canceled{false};
  };

  // Desired vs last-reported state, both keyed by action name and guarded by
  // state_mutex_. The transition raises/heals only when they differ; a report
  // that cannot be delivered yet leaves last_reported_state_ behind desired_,
  // so reconcile_pending() retries it on the next rescan instead of dropping it.
  std::map<std::string, DesiredState> desired_state_;
  std::map<std::string, ActionState> last_reported_state_;
  // Per-action goal verdict memory (see GoalLedger). Same key space and same
  // lock as the two maps above, and dropped with them in prune_vanished().
  std::map<std::string, GoalLedger> goal_ledgers_;
  std::mutex state_mutex_;

  // Bounded dedup of logged (goal_id:status) keys (LOG suppression only).
  std::unordered_set<std::string> logged_;
  std::deque<std::string> logged_order_;
  std::mutex logged_mutex_;
  size_t logged_capacity_;

  // Configuration
  uint8_t aborted_severity_;
  bool canceled_is_fault_;
  bool preempted_is_fault_;
  bool heal_on_succeeded_;
  double rescan_period_sec_;
  double retry_period_sec_;
  std::string code_prefix_;
  std::vector<std::string> exclude_actions_;
  std::vector<std::string> include_only_actions_;

  friend class ActionStatusBridgeTestAccess;
};

/// Test-only accessor for the bridge's private maps and prune path. Lets unit
/// tests exercise rescan add+prune without a live action graph.
class ActionStatusBridgeTestAccess {
 public:
  explicit ActionStatusBridgeTestAccess(ActionStatusBridgeNode * node) : node_(node) {
  }

  /// Seed a watched action (subscription placeholder + per-action state) as if
  /// rescan had discovered its `/<action>/_action/status` topic.
  void add_watched(const std::string & action_name);

  /// Run the prune pass against a set of still-present action names.
  void prune_to(const std::vector<std::string> & present_action_names);

  bool is_watched(const std::string & action_name) const;
  bool has_state(const std::string & action_name) const;

  /// True if the FaultManager was actually told this action is failed (the
  /// last-reported state was committed, i.e. the report was delivered).
  bool reported_failed(const std::string & action_name) const;

  /// True if a failed state is observed but not yet delivered: the desired state
  /// is failed while the last-reported state is not. This is the "pending retry"
  /// condition - the fault is remembered, not dropped.
  bool pending_failed(const std::string & action_name) const;

  /// The FaultReporter for an action (created on first call). Lets a test drive
  /// the deferred-delivery path with a real reporter whose service is not ready.
  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & action_name);

  /// Run the retry pass (what the retry timer fires) so a test can exercise the
  /// deferred-then-delivered loop without a live timer.
  void run_reconcile_pending();

  /// Reconcile one action treating delivery as available (the null-reporter
  /// seam), so a test can prove a previously deferred transition is delivered
  /// once the service becomes ready.
  ActionStatusBridgeNode::ActionState reconcile_deliverable(const std::string & action_name);

  /// Identity of the FaultReporter for an action (created on first call). Lets a
  /// test assert the reporter is created once and never swapped out.
  const void * reporter_identity(const std::string & action_name);

 private:
  ActionStatusBridgeNode * node_;
};

}  // namespace ros2_medkit_action_status_bridge
