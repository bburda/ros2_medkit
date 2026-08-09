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

#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <functional>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_action_status_bridge {

namespace {

constexpr char kStatusSuffix[] = "/_action/status";
constexpr char kStatusType[] = "action_msgs/msg/GoalStatusArray";

// Total fault-code length clamp (medkit rule). Reserve room for the status
// suffix so a long action name cannot let the clamp eat "_ABORTED"/"_CANCELED".
constexpr size_t kMaxCodeLen = 64;
constexpr size_t kMaxActionPart = 48;
// Longest status suffix that may follow the action part, "_CANCELED".
constexpr size_t kMaxStatusSuffix = 9;

bool contains_substr(const std::vector<std::string> & list, const std::string & value) {
  for (const auto & item : list) {
    if (!item.empty() && value.find(item) != std::string::npos) {
      return true;
    }
  }
  return false;
}

void strip_trailing_underscore(std::string & s) {
  while (!s.empty() && s.back() == '_') {
    s.pop_back();
  }
}

// A goal that has reached a final state. Anything else (ACCEPTED, EXECUTING,
// CANCELING) is still live and can therefore be the goal that displaced another.
bool is_terminal_status(int8_t status) {
  using GoalStatus = action_msgs::msg::GoalStatus;
  return status == GoalStatus::STATUS_SUCCEEDED || status == GoalStatus::STATUS_CANCELED ||
         status == GoalStatus::STATUS_ABORTED;
}

// goal_info.stamp in nanoseconds. The action SERVER stamps this at acceptance
// (rcl_action_accept_new_goal), so within one action it orders goals by the
// order that server took them - which is what "displaced by a newer goal" means.
int64_t stamp_ns(const action_msgs::msg::GoalStatus & gs) {
  return static_cast<int64_t>(gs.goal_info.stamp.sec) * 1000000000LL + static_cast<int64_t>(gs.goal_info.stamp.nanosec);
}

}  // namespace

bool ActionStatusBridgeNode::goal_was_preempted(const action_msgs::msg::GoalStatusArray & msg, size_t index) {
  using GoalStatus = action_msgs::msg::GoalStatus;
  if (index >= msg.status_list.size() || msg.status_list[index].status != GoalStatus::STATUS_ABORTED) {
    return false;
  }
  const int64_t self_stamp = stamp_ns(msg.status_list[index]);

  // The immediate successor by stamp: the smallest stamp strictly greater than
  // this goal's. A tie is NOT a successor - unstamped arrays (the unit tests,
  // and any server that does not stamp) must fall through to "not preempted",
  // which is the conservative direction.
  const action_msgs::msg::GoalStatus * successor = nullptr;
  int64_t successor_stamp = 0;
  for (size_t i = 0; i < msg.status_list.size(); ++i) {
    if (i == index) {
      continue;
    }
    const int64_t s = stamp_ns(msg.status_list[i]);
    if (s <= self_stamp) {
      continue;
    }
    if (successor == nullptr || s < successor_stamp) {
      successor = &msg.status_list[i];
      successor_stamp = s;
    }
  }
  return successor != nullptr && !is_terminal_status(successor->status);
}

ActionStatusBridgeNode::ActionState ActionStatusBridgeNode::derive_state(const action_msgs::msg::GoalStatusArray & msg,
                                                                         bool canceled_is_fault,
                                                                         bool preempted_is_fault, GoalLedger & ledger) {
  using GoalStatus = action_msgs::msg::GoalStatus;

  GoalLedger next;
  bool any_terminal = false;
  for (size_t i = 0; i < msg.status_list.size(); ++i) {
    const auto & gs = msg.status_list[i];
    std::array<uint8_t, 16> uuid{};
    std::copy(gs.goal_info.goal_id.uuid.begin(), gs.goal_info.goal_id.uuid.end(), uuid.begin());
    const std::string key = uuid_to_hex(uuid);

    if (!is_terminal_status(gs.status)) {
      continue;  // still live: contributes nothing and carries no verdict
    }

    // A verdict already taken for this goal is never revised - see GoalLedger.
    // Carrying it into `next` is also what prunes: a goal absent from this
    // array simply does not make it across.
    const auto seen = ledger.find(key);
    if (seen != ledger.end()) {
      next.emplace(key, seen->second);
    } else if (gs.status == GoalStatus::STATUS_ABORTED) {
      const bool suppressed = !preempted_is_fault && goal_was_preempted(msg, i);
      next.emplace(key, suppressed ? GoalVerdict::kNotFailing : GoalVerdict::kFailingAborted);
    } else if (gs.status == GoalStatus::STATUS_CANCELED && canceled_is_fault) {
      next.emplace(key, GoalVerdict::kFailingCanceled);
    } else {
      next.emplace(key, GoalVerdict::kNotFailing);
    }

    // SUCCEEDED, or CANCELED when it is not a fault, is a non-failing terminal
    // and is what licenses a heal. A SUPPRESSED (preempted) abort deliberately
    // does not: a preemption is evidence about nothing, and letting it stand in
    // for a success would heal a real fault raised moments earlier.
    if (gs.status == GoalStatus::STATUS_SUCCEEDED || gs.status == GoalStatus::STATUS_CANCELED) {
      any_terminal = true;
    }
  }

  ledger = std::move(next);
  for (const auto & [key, verdict] : ledger) {
    (void)key;
    if (verdict != GoalVerdict::kNotFailing) {
      return ActionState::kFailed;  // any failing goal fails the action (order-independent)
    }
  }
  return any_terminal ? ActionState::kHealthy : ActionState::kUnknown;
}

bool ActionStatusBridgeNode::failure_is_cancel(const GoalLedger & ledger) {
  bool saw_canceled = false;
  for (const auto & [key, verdict] : ledger) {
    (void)key;
    if (verdict == GoalVerdict::kFailingAborted) {
      return false;
    }
    if (verdict == GoalVerdict::kFailingCanceled) {
      saw_canceled = true;
    }
  }
  return saw_canceled;
}

ActionStatusBridgeNode::ActionStatusBridgeNode(const rclcpp::NodeOptions & options)
  : Node("action_status_bridge", options) {
  load_parameters();

  // Fast retry timer, created up front but kept disarmed (cancelled) until a
  // report is actually deferred, so an idle bridge does no periodic work.
  retry_timer_ = create_wall_timer(std::chrono::duration<double>(retry_period_sec_), [this]() {
    reconcile_pending();
  });
  retry_timer_->cancel();

  rescan_actions();
  rescan_timer_ = create_wall_timer(std::chrono::duration<double>(rescan_period_sec_), [this]() {
    rescan_actions();
  });

  RCLCPP_INFO(get_logger(),
              "ActionStatusBridge started (prefix=%s, aborted_severity=%u, canceled_is_fault=%s, "
              "preempted_is_fault=%s, rescan=%.1fs, retry=%.0fms)",
              code_prefix_.c_str(), static_cast<unsigned>(aborted_severity_), canceled_is_fault_ ? "true" : "false",
              preempted_is_fault_ ? "true" : "false", rescan_period_sec_, retry_period_sec_ * 1000.0);
}

ActionStatusBridgeNode::~ActionStatusBridgeNode() {
  // Stop the rescan timer first so no new subscription is created during
  // teardown, then drop all status subscriptions. Their callbacks capture
  // `this`; letting them fire on a partially destroyed node causes SIGABRT
  // (subscription destructor pattern). subs_ is only mutated from the (now
  // stopped) rescan path, so no lock is needed here.
  rescan_timer_.reset();
  retry_timer_.reset();
  subs_.clear();
}

void ActionStatusBridgeNode::load_parameters() {
  const int aborted_severity =
      static_cast<int>(declare_parameter<int>("aborted_severity", ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR));
  if (aborted_severity < ros2_medkit_msgs::msg::Fault::SEVERITY_INFO ||
      aborted_severity > ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
    RCLCPP_WARN(get_logger(), "aborted_severity %d out of range [0,3]; clamping to ERROR", aborted_severity);
    aborted_severity_ = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  } else {
    aborted_severity_ = static_cast<uint8_t>(aborted_severity);
  }

  canceled_is_fault_ = declare_parameter<bool>("canceled_is_fault", false);
  // A preempted goal is terminated by its server as ABORTED with no status of
  // its own (see this class's header), so it is suppressed by default: on a
  // fleet that re-plans, preemptions are routine and a fault per preemption
  // buries the failures the tree exists to name. Set true to count every
  // ABORTED, whatever displaced it.
  preempted_is_fault_ = declare_parameter<bool>("preempted_is_fault", false);
  heal_on_succeeded_ = declare_parameter<bool>("heal_on_succeeded", true);
  rescan_period_sec_ = declare_parameter<double>("rescan_period_sec", 2.0);
  if (rescan_period_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "rescan_period_sec %.3f not positive; using 2.0", rescan_period_sec_);
    rescan_period_sec_ = 2.0;
  }

  // Fast retry cadence for delivering reports deferred while the FaultManager
  // service was undiscovered. Kept short (default 50 ms) so the FaultManager
  // freeze-frame snapshot is taken as close to the event as the discovery floor
  // allows; the timer is armed only while a delivery is pending.
  retry_period_sec_ = declare_parameter<double>("retry_period_sec", 0.05);
  if (retry_period_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "retry_period_sec %.3f not positive; using 0.05", retry_period_sec_);
    retry_period_sec_ = 0.05;
  }

  code_prefix_ = to_upper_snake(declare_parameter<std::string>("code_prefix", "ACTION"), kMaxActionPart);
  if (code_prefix_.empty()) {
    RCLCPP_WARN(get_logger(), "code_prefix empty after normalization; using ACTION");
    code_prefix_ = "ACTION";
  }

  exclude_actions_ = declare_parameter<std::vector<std::string>>("exclude_actions", std::vector<std::string>{});
  include_only_actions_ =
      declare_parameter<std::vector<std::string>>("include_only_actions", std::vector<std::string>{});

  const int dedup_capacity = static_cast<int>(declare_parameter<int>("dedup_capacity", 4096));
  if (dedup_capacity <= 0) {
    RCLCPP_WARN(get_logger(), "dedup_capacity %d not positive; using 4096", dedup_capacity);
    logged_capacity_ = 4096;
  } else {
    logged_capacity_ = static_cast<size_t>(dedup_capacity);
  }

  // Action terminal states are discrete, authoritative events. The action-level
  // state model only transitions on a net-state change, so it does not flap;
  // retries that issue a fresh goal_id each attempt do not re-raise while the
  // action stays failed. Pre-declare the FaultReporter LocalFilter as disabled
  // so a raise reports immediately and, crucially, PASSED (healing on a later
  // SUCCEEDED) is not throttled by the default occurrence threshold. The
  // reporter ctor's has_parameter guard honours this value. Override via
  // fault_reporter.local_filtering.enabled.
  if (!has_parameter("fault_reporter.local_filtering.enabled")) {
    declare_parameter("fault_reporter.local_filtering.enabled", false);
  }
}

void ActionStatusBridgeNode::rescan_actions() {
  const auto topics = get_topic_names_and_types();
  // Action status QoS is reliable + transient_local + keep_last; match it so we
  // receive the latched terminal status of a goal.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

  // Warn (don't silently drop faults) when a server publishes a non-standard
  // (volatile/best-effort) status QoS that is incompatible with our request.
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.incompatible_qos_callback = [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
    RCLCPP_WARN(get_logger(),
                "Incompatible action status QoS (policy %d); faults may be missed for a non-standard server",
                static_cast<int>(info.last_policy_kind));
  };

  // Track which eligible status topics exist this scan, to prune the rest.
  std::map<std::string, std::string> present;

  for (const auto & [topic, types] : topics) {
    const std::string action_name = action_name_from_status_topic(topic);
    if (action_name.empty()) {
      continue;
    }
    if (std::find(types.begin(), types.end(), kStatusType) == types.end()) {
      continue;
    }
    if (!action_is_eligible(action_name)) {
      continue;
    }
    present.emplace(topic, action_name);
    if (subs_.find(topic) != subs_.end()) {
      continue;  // already subscribed
    }
    auto sub = create_subscription<action_msgs::msg::GoalStatusArray>(
        topic, qos,
        [this, action_name](const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
          status_callback(action_name, msg);
        },
        sub_options);
    subs_.emplace(topic, sub);
    RCLCPP_INFO(get_logger(), "Watching action '%s' (topic %s)", action_name.c_str(), topic.c_str());
  }

  prune_vanished(present);

  // Backstop the fast retry timer: re-attempt any deferred delivery and clean up
  // healed-and-vanished actions on the (slow) rescan tick too, so a missed timer
  // arming can never strand a pending report. The fast timer is what keeps the
  // FaultManager freeze-frame contemporaneous; this is just belt-and-suspenders.
  reconcile_pending();
}

void ActionStatusBridgeNode::prune_vanished(const std::map<std::string, std::string> & present_topics) {
  for (auto it = subs_.begin(); it != subs_.end();) {
    const std::string & topic = it->first;
    if (present_topics.find(topic) != present_topics.end()) {
      ++it;
      continue;
    }
    const std::string action_name = action_name_from_status_topic(topic);

    bool was_failed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto sit = last_reported_state_.find(action_name);
      was_failed = sit != last_reported_state_.end() && sit->second == ActionState::kFailed;
    }
    {
      std::lock_guard<std::mutex> lock(reporters_mutex_);
      // If a still-failed action vanishes (e.g. Nav2 lifecycle deactivate), heal
      // its fault before forgetting it - otherwise the fault stays stuck active
      // in the FaultManager and the bridge can no longer clear it. Gate the heal
      // on service readiness: unlike a live heal, a vanished action cannot be
      // retried (its state is dropped here), so if the FaultManager service is
      // not discovered at this instant the heal cannot be delivered. Surface that
      // narrow case with a warning instead of dropping it silently. See the
      // delivery-latency note in the README.
      auto rit = reporters_.find(action_name);
      if (was_failed && heal_on_succeeded_ && rit != reporters_.end()) {
        if (rit->second->is_service_ready()) {
          rit->second->report_passed(fault_code_for(action_name, false));
          if (canceled_is_fault_) {
            rit->second->report_passed(fault_code_for(action_name, true));
          }
          RCLCPP_INFO(get_logger(), "Action '%s' vanished while failed; healed before dropping", action_name.c_str());
        } else {
          RCLCPP_WARN(get_logger(),
                      "Action '%s' vanished while failed but FaultManager service is not ready; its heal could not be "
                      "delivered and the fault may remain active",
                      action_name.c_str());
        }
      }
      reporters_.erase(action_name);
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_reported_state_.erase(action_name);
      desired_state_.erase(action_name);
      goal_ledgers_.erase(action_name);
    }
    RCLCPP_INFO(get_logger(), "Action '%s' vanished; dropped (topic %s)", action_name.c_str(), topic.c_str());
    it = subs_.erase(it);
  }
}

ActionStatusBridgeNode::ActionState
ActionStatusBridgeNode::apply_message(const std::string & action_name, const action_msgs::msg::GoalStatusArray & msg,
                                      ros2_medkit_fault_reporter::FaultReporter * reporter) {
  // Record the latest observed action-level state as the desired state, then
  // reconcile it against what the FaultManager was last told. A SUCCEEDED while
  // healing is disabled leaves the desired state untouched, so a prior failure
  // stays failed (the no-heal contract). The derivation itself now carries
  // per-action memory (GoalLedger), so it runs under state_mutex_ with the maps
  // it belongs to.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ActionState net = derive_state(msg, canceled_is_fault_, preempted_is_fault_, goal_ledgers_[action_name]);
    if (net == ActionState::kUnknown) {
      return ActionState::kUnknown;  // no terminal verdict in this message
    }
    auto & desired = desired_state_[action_name];
    if (net == ActionState::kFailed) {
      desired.net = ActionState::kFailed;
      desired.canceled = failure_is_cancel(goal_ledgers_[action_name]);
    } else if (heal_on_succeeded_) {  // net == kHealthy
      desired.net = ActionState::kHealthy;
    }
  }

  const ActionState delivered = reconcile(action_name, reporter);
  update_retry_timer();  // arm the fast retry if this could not be delivered yet
  return delivered;
}

ActionStatusBridgeNode::ActionState
ActionStatusBridgeNode::reconcile(const std::string & action_name,
                                  ros2_medkit_fault_reporter::FaultReporter * reporter) {
  // state_mutex_ is held across the gate, the report() call and the commit so a
  // second caller (the status callback and the retry timer both reconcile)
  // cannot pass the gate concurrently and double-report under a multi-threaded
  // executor. report()/report_passed() are async (fire-and-forget), so the
  // critical section stays short. Callers must NOT hold state_mutex_.
  std::lock_guard<std::mutex> lock(state_mutex_);

  auto dit = desired_state_.find(action_name);
  if (dit == desired_state_.end() || dit->second.net == ActionState::kUnknown) {
    return ActionState::kUnknown;  // nothing desired yet
  }
  const DesiredState desired = dit->second;
  auto rit = last_reported_state_.find(action_name);
  // Absent reported state is treated as healthy: a first SUCCEEDED must not heal
  // a fault that was never raised.
  const ActionState reported = (rit == last_reported_state_.end()) ? ActionState::kHealthy : rit->second;

  if (desired.net == reported) {
    return ActionState::kUnknown;  // already in sync, nothing to report
  }

  // A real reporter can only deliver once the FaultManager service is
  // discovered; until then keep the transition pending (the fast retry timer
  // re-attempts it) instead of dropping it. A null reporter is the unit-test
  // decision-only seam: delivery is a no-op that still commits the transition.
  const bool deliverable = (reporter == nullptr) || reporter->is_service_ready();
  if (!deliverable) {
    RCLCPP_DEBUG(get_logger(), "FaultManager service not ready; deferring %s for action %s",
                 desired.net == ActionState::kFailed ? "fault" : "heal", action_name.c_str());
    return ActionState::kUnknown;
  }

  if (desired.net == ActionState::kFailed) {
    if (reporter != nullptr) {
      const std::string code = fault_code_for(action_name, desired.canceled);
      reporter->report(code, aborted_severity_,
                       std::string("Action ") + action_name + (desired.canceled ? " canceled" : " aborted"));
      RCLCPP_INFO(get_logger(), "Action %s -> fault %s", action_name.c_str(), code.c_str());
    }
    last_reported_state_[action_name] = ActionState::kFailed;
    return ActionState::kFailed;
  }

  // desired.net == kHealthy, and healing was enabled when it was set.
  if (reporter != nullptr) {
    // Heal both possible codes; only the one previously raised exists.
    reporter->report_passed(fault_code_for(action_name, false));
    if (canceled_is_fault_) {
      reporter->report_passed(fault_code_for(action_name, true));
    }
    RCLCPP_INFO(get_logger(), "Action %s healed", action_name.c_str());
  }
  last_reported_state_[action_name] = ActionState::kHealthy;
  return ActionState::kHealthy;
}

void ActionStatusBridgeNode::reconcile_pending() {
  // Collect actions whose desired state has not been delivered to the
  // FaultManager, then reconcile each outside the lock (reconcile() takes
  // state_mutex_ itself). reporter_for() is only called for actions with a real
  // pending transition, so this never creates reporters speculatively.
  std::vector<std::string> pending;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & [action_name, desired] : desired_state_) {
      if (desired.net == ActionState::kUnknown) {
        continue;
      }
      auto rit = last_reported_state_.find(action_name);
      const ActionState reported = (rit == last_reported_state_.end()) ? ActionState::kHealthy : rit->second;
      if (desired.net != reported) {
        pending.push_back(action_name);
      }
    }
  }
  for (const auto & action_name : pending) {
    reconcile(action_name, reporter_for(action_name));
  }

  update_retry_timer();
}

void ActionStatusBridgeNode::update_retry_timer() {
  if (!retry_timer_) {
    return;
  }
  bool any_pending = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & [action_name, desired] : desired_state_) {
      if (desired.net == ActionState::kUnknown) {
        continue;
      }
      auto rit = last_reported_state_.find(action_name);
      const ActionState reported = (rit == last_reported_state_.end()) ? ActionState::kHealthy : rit->second;
      if (desired.net != reported) {
        any_pending = true;
        break;
      }
    }
  }
  // Arm only on the no-pending -> pending edge so an already-running timer is not
  // restarted (which would push out its next fire); disarm once drained.
  if (any_pending) {
    if (retry_timer_->is_canceled()) {
      retry_timer_->reset();
    }
  } else {
    retry_timer_->cancel();
  }
}

void ActionStatusBridgeNode::status_callback(const std::string & action_name,
                                             const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
  using GoalStatus = action_msgs::msg::GoalStatus;

  // Per-goal dedup is LOG-only here; it must not gate the state transition.
  for (size_t i = 0; i < msg->status_list.size(); ++i) {
    const auto & gs = msg->status_list[i];
    if (gs.status != GoalStatus::STATUS_ABORTED && gs.status != GoalStatus::STATUS_CANCELED &&
        gs.status != GoalStatus::STATUS_SUCCEEDED) {
      continue;
    }
    std::array<uint8_t, 16> uuid{};
    std::copy(gs.goal_info.goal_id.uuid.begin(), gs.goal_info.goal_id.uuid.end(), uuid.begin());
    const std::string uuid_hex = uuid_to_hex(uuid);
    const std::string key = uuid_hex + ":" + std::to_string(static_cast<int>(gs.status));
    if (!mark_logged(key)) {
      continue;
    }
    // A suppressed preemption is INFO, not DEBUG: it is the one case where the
    // bridge deliberately says nothing to the FaultManager about an ABORTED
    // goal, so the decision has to be visible somewhere. Logged only on the
    // first sight of that goal's abort - the same dedup key everything else
    // here uses - so a fleet that re-plans does not flood the log.
    if (!preempted_is_fault_ && gs.status == GoalStatus::STATUS_ABORTED && goal_was_preempted(*msg, i)) {
      RCLCPP_INFO(get_logger(), "Action %s goal %s aborted by preemption (a newer goal was live); not a fault",
                  action_name.c_str(), uuid_hex.substr(0, 8).c_str());
      continue;
    }
    RCLCPP_DEBUG(get_logger(), "Action %s goal %s status %d", action_name.c_str(), uuid_hex.substr(0, 8).c_str(),
                 static_cast<int>(gs.status));
  }

  // Single-threaded rclcpp::spin (see main.cpp): status callbacks and the rescan
  // timer run on the same thread, so the raw FaultReporter* returned here stays
  // valid for this call - prune_vanished() only erases reporters_ from that same
  // thread, never concurrently. This invariant must hold if a multi-threaded
  // executor is ever introduced.
  apply_message(action_name, *msg, reporter_for(action_name));
}

ros2_medkit_fault_reporter::FaultReporter * ActionStatusBridgeNode::reporter_for(const std::string & action_name) {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  auto it = reporters_.find(action_name);
  if (it != reporters_.end()) {
    return it->second.get();  // created on the first report; source is fixed thereafter
  }
  // First report for this action: attribute it to the action SERVER's node FQN so
  // the gateway can resolve the fault to its SOVD entity. During DDS discovery the
  // FQN may be unresolved (rcl reports placeholders); fall back to the action name
  // so the fault still fires on time. Reporting the fault on time takes priority
  // over entity attribution when discovery is slow.
  //
  // The source is NOT re-attributed later: reporting_sources is append-only on the
  // manager side and the per-entity /faults scope filter is strict-AND, so a
  // provisional action-name source cannot be swapped for the FQN afterwards.
  // Correct attribution for the slow-discovery case is a separate concern (it
  // needs a way to supersede a provisional source).
  const std::string fqn = server_fqn_for_action(action_name);
  const std::string & source_id = fqn.empty() ? action_name : fqn;
  auto reporter = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(this->shared_from_this(), source_id);
  auto * raw = reporter.get();
  reporters_[action_name] = std::move(reporter);
  return raw;
}

std::string ActionStatusBridgeNode::server_fqn_from_endpoint(const std::string & node_name,
                                                             const std::string & node_namespace) {
  // During DDS discovery the participant is known before its node name/namespace
  // propagate; rcl reports these placeholders meanwhile. Treat them (and an
  // empty name) as unresolved so a fault is never attributed to the placeholder.
  constexpr const char * kUnknownName = "_NODE_NAME_UNKNOWN_";
  constexpr const char * kUnknownNamespace = "_NODE_NAMESPACE_UNKNOWN_";
  if (node_name.empty() || node_name == kUnknownName || node_namespace == kUnknownNamespace) {
    return "";
  }
  return (node_namespace.empty() || node_namespace == "/") ? "/" + node_name : node_namespace + "/" + node_name;
}

std::string ActionStatusBridgeNode::server_fqn_for_action(const std::string & action_name) {
  // The action server publishes <action>/_action/status, so its node FQN is the
  // publisher's. Returns "" until discovery resolves a real node name.
  const auto pubs = get_publishers_info_by_topic(action_name + "/_action/status");
  for (const auto & p : pubs) {
    const std::string fqn = server_fqn_from_endpoint(p.node_name(), p.node_namespace());
    if (!fqn.empty()) {
      return fqn;
    }
  }
  return "";  // unresolved; caller falls back and re-resolves on a later message
}

bool ActionStatusBridgeNode::mark_logged(const std::string & goal_status_key) {
  std::lock_guard<std::mutex> lock(logged_mutex_);
  if (logged_.find(goal_status_key) != logged_.end()) {
    return false;
  }
  logged_.insert(goal_status_key);
  logged_order_.push_back(goal_status_key);
  while (logged_order_.size() > logged_capacity_) {
    logged_.erase(logged_order_.front());
    logged_order_.pop_front();
  }
  return true;
}

bool ActionStatusBridgeNode::action_is_eligible(const std::string & action_name) const {
  if (!include_only_actions_.empty() && !contains_substr(include_only_actions_, action_name)) {
    return false;
  }
  if (contains_substr(exclude_actions_, action_name)) {
    return false;
  }
  return true;
}

std::string ActionStatusBridgeNode::action_name_from_status_topic(const std::string & topic) {
  const std::string suffix = kStatusSuffix;
  if (topic.size() <= suffix.size()) {
    return "";
  }
  if (topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return "";
  }
  return topic.substr(0, topic.size() - suffix.size());
}

std::string ActionStatusBridgeNode::uuid_to_hex(const std::array<uint8_t, 16> & uuid) {
  std::string out;
  out.reserve(32);
  char buf[3];
  for (uint8_t b : uuid) {
    std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(b));
    out += buf;
  }
  return out;
}

std::string ActionStatusBridgeNode::to_upper_snake(const std::string & in, size_t max_len) {
  std::string result;
  result.reserve(in.size());
  bool last_sep = true;
  for (char c : in) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      result += static_cast<char>(std::toupper(uc));
      last_sep = false;
    } else if (!last_sep) {
      result += '_';
      last_sep = true;
    }
  }
  strip_trailing_underscore(result);
  if (result.size() > max_len) {
    // Truncate but reserve room for a stable disambiguator so two action names
    // sharing the same `max_len` prefix do not collide.
    constexpr size_t kDisambigLen = 5;  // '_' + 4 hex
    const size_t keep = (max_len > kDisambigLen) ? (max_len - kDisambigLen) : 0;
    const std::string full = result;
    result.resize(keep);
    strip_trailing_underscore(result);
    const uint16_t h = static_cast<uint16_t>(std::hash<std::string>{}(full));
    char buf[6];
    std::snprintf(buf, sizeof(buf), "_%04X", static_cast<unsigned>(h));
    result += buf;
  }
  return result;
}

std::string ActionStatusBridgeNode::fault_code_for(const std::string & action_name, bool canceled) const {
  const char * suffix = canceled ? "_CANCELED" : "_ABORTED";
  // Budget the action part so the status suffix always survives the 64-char clamp.
  const size_t suffix_budget = kMaxStatusSuffix + 1;  // suffix + leading '_' to action part
  const size_t prefix_len = code_prefix_.size() + 1;  // prefix + '_'
  size_t action_budget = kMaxActionPart;
  if (prefix_len + action_budget + suffix_budget > kMaxCodeLen) {
    action_budget = (kMaxCodeLen > prefix_len + suffix_budget) ? (kMaxCodeLen - prefix_len - suffix_budget) : 0;
  }
  const std::string action_part = to_upper_snake(action_name, action_budget);

  std::string code = code_prefix_;
  if (!action_part.empty()) {
    code += '_';
    code += action_part;
  }
  code += suffix;
  if (code.size() > kMaxCodeLen) {
    code.resize(kMaxCodeLen);
    strip_trailing_underscore(code);
  }
  return code;
}

void ActionStatusBridgeTestAccess::add_watched(const std::string & action_name) {
  // nullptr subscription is sufficient: prune keys off the topic name only.
  node_->subs_.emplace(action_name + kStatusSuffix, nullptr);
  std::lock_guard<std::mutex> lock(node_->state_mutex_);
  node_->last_reported_state_[action_name] = ActionStatusBridgeNode::ActionState::kFailed;
}

void ActionStatusBridgeTestAccess::prune_to(const std::vector<std::string> & present_action_names) {
  std::map<std::string, std::string> present;
  for (const auto & a : present_action_names) {
    present.emplace(a + kStatusSuffix, a);
  }
  node_->prune_vanished(present);
}

bool ActionStatusBridgeTestAccess::is_watched(const std::string & action_name) const {
  return node_->subs_.find(action_name + kStatusSuffix) != node_->subs_.end();
}

bool ActionStatusBridgeTestAccess::has_state(const std::string & action_name) const {
  std::lock_guard<std::mutex> lock(node_->state_mutex_);
  return node_->last_reported_state_.find(action_name) != node_->last_reported_state_.end();
}

bool ActionStatusBridgeTestAccess::reported_failed(const std::string & action_name) const {
  std::lock_guard<std::mutex> lock(node_->state_mutex_);
  auto it = node_->last_reported_state_.find(action_name);
  return it != node_->last_reported_state_.end() && it->second == ActionStatusBridgeNode::ActionState::kFailed;
}

bool ActionStatusBridgeTestAccess::pending_failed(const std::string & action_name) const {
  std::lock_guard<std::mutex> lock(node_->state_mutex_);
  auto dit = node_->desired_state_.find(action_name);
  if (dit == node_->desired_state_.end() || dit->second.net != ActionStatusBridgeNode::ActionState::kFailed) {
    return false;
  }
  auto rit = node_->last_reported_state_.find(action_name);
  const auto reported =
      (rit == node_->last_reported_state_.end()) ? ActionStatusBridgeNode::ActionState::kHealthy : rit->second;
  return reported != ActionStatusBridgeNode::ActionState::kFailed;
}

ros2_medkit_fault_reporter::FaultReporter *
ActionStatusBridgeTestAccess::reporter_for(const std::string & action_name) {
  return node_->reporter_for(action_name);
}

void ActionStatusBridgeTestAccess::run_reconcile_pending() {
  node_->reconcile_pending();
}

ActionStatusBridgeNode::ActionState
ActionStatusBridgeTestAccess::reconcile_deliverable(const std::string & action_name) {
  return node_->reconcile(action_name, nullptr);
}

const void * ActionStatusBridgeTestAccess::reporter_identity(const std::string & action_name) {
  return node_->reporter_for(action_name);
}

}  // namespace ros2_medkit_action_status_bridge
