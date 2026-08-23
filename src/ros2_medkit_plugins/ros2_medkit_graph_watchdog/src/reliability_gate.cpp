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
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

namespace ros2_medkit_graph_watchdog {

ReliabilityGate::ReliabilityGate(int warmup_cycles, rclcpp::Node * gateway_node, std::mutex * node_mutex,
                                 int departed_retention_ticks)
  : warmup_(warmup_cycles)
  , lifecycle_(gateway_node, node_mutex, departed_retention_ticks)
  , warmup_cycles_(warmup_cycles < 0 ? 0 : warmup_cycles) {
}

void ReliabilityGate::update(const ros2_medkit_gateway::IntrospectionInput & snapshot, uint64_t tick) {
  std::vector<std::string> ids;
  ids.reserve(snapshot.apps.size());
  for (const auto & app : snapshot.apps) {
    ids.push_back(app.id);
  }
  // LifecycleWatcher is internally synchronized and does BLOCKING GetState reads: update
  // it OUTSIDE gate_mutex_ so the status handler is never blocked behind a blocking read.
  // The tick counter stays uint64_t end to end: narrowing it to int here would wrap after
  // INT_MAX ticks (~13.6 years at a 200 ms cadence) and corrupt departed-entry retention on a
  // long-lived gateway. Cheap to keep wide, so do not rely on "it never gets that far".
  lifecycle_.update(snapshot, tick);

  // warmup_ and the bringup scalars have no internal lock; the tick runs on the plugin
  // worker thread while status_json() reads them on an HTTP thread, so gate_mutex_ makes
  // this writer and that reader mutually exclusive.
  std::lock_guard<std::mutex> lock(gate_mutex_);
  // Re-arm the global bringup grace whenever the graph empties out. Without this,
  // graph_seen_ latches on the first-ever non-empty snapshot and never re-arms, so
  // after a full-stack restart (apps -> empty -> refill) last_tick_ - graph_first_tick_
  // already exceeds warmup_cycles_ and unknown (topic/graph) sources get no grace on the
  // second bringup - defeating the exact false-positive suppression this gate exists for.
  // This keeps the global grace in step with WarmupTracker's per-entity re-warm.
  if (ids.empty()) {
    graph_seen_ = false;
  } else if (!graph_seen_) {
    graph_seen_ = true;
    graph_first_tick_ = tick;
  }
  warmup_.update(ids, tick);
  last_tick_ = tick;
}

void ReliabilityGate::pump_lifecycle_events(std::chrono::nanoseconds budget) {
  // Deliberately NOT under gate_mutex_, for the same reason lifecycle_.update() is not:
  // this executes subscription callbacks that take the watcher's own state mutex, and
  // holding gate_mutex_ across them would park the status handler behind them.
  lifecycle_.pump_events(budget);
}

bool ReliabilityGate::allows_raise(const std::string & source_id) const {
  if (warmup_.is_known(source_id)) {
    return warmup_.is_armed(source_id, last_tick_) && lifecycle_.node_ok(source_id);
  }
  // Unknown source_id: global bringup grace only.
  if (!graph_seen_) {
    return false;
  }
  return (last_tick_ - graph_first_tick_) >= static_cast<uint64_t>(warmup_cycles_);
}

PresenceOwnership ReliabilityGate::presence_ownership(const std::string & source_id) const {
  // Read without gate_mutex_, for the same reason allows_raise() takes none: lifecycle_ is
  // separately self-synchronized, and both run on the tick thread, sequentially after update().
  // Two reads rather than one combined lock: the only transition either can observe mid-call is
  // an empty label becoming a real one, and both orderings of that give the same answer here.
  const auto label = lifecycle_.state_of(source_id);

  // Decided BEFORE arming, and that order is the point. A measured non-active label is
  // knowledge about who the node belongs to, and it is the only negative answer a caller may
  // act on by giving a key up. Warmup is not: an entity that has not armed yet, or has gone
  // back to warming after a restart, has been measured as nothing at all. Asking allows_raise()
  // first would collapse the two, because node_ok() already refuses a managed non-active node -
  // so every re-warming node would read exactly like a node the graph had disowned.
  if (label.has_value() && !label->empty() && *label != "active") {
    return PresenceOwnership::kDisowned;
  }
  if (!allows_raise(source_id)) {
    return PresenceOwnership::kUnclaimed;
  }
  if (!label.has_value()) {
    // No managed record: a plain node. Nothing about it can ever say it belonged elsewhere,
    // so this is knowledge, not a gap in it.
    return PresenceOwnership::kEarned;
  }
  if (label->empty()) {
    // Ignorance, and it is bounded. While the watcher still has GetState attempts to spend, a
    // measurement may be one tick away and taking ownership now would race it. Once they are
    // spent, nothing here will ASK again - but ~/transition_event is still subscribed and can
    // still deliver a label, so the grant is provisional rather than final.
    return lifecycle_.measurement_pending(source_id) ? PresenceOwnership::kUnclaimed : PresenceOwnership::kProvisional;
  }
  return PresenceOwnership::kEarned;  // "active", the one label that says this node is ours
}

nlohmann::json ReliabilityGate::status_json() const {
  // Reader side of gate_mutex_ (see update()). Held across the lifecycle_ reads too - they
  // take the lifecycle SharedState mutex (order gate_mutex_ -> lifecycle mutex, never the
  // reverse) and do no blocking I/O, so this cannot stall.
  std::lock_guard<std::mutex> lock(gate_mutex_);
  nlohmann::json entities = nlohmann::json::array();
  for (const auto & [id, entry] : warmup_.entries()) {
    const bool armed = warmup_.is_armed(id, last_tick_);
    const auto lc = lifecycle_.state_of(id);
    const bool suppressed = !armed || !lifecycle_.node_ok(id);
    entities.push_back({{"id", id},
                        {"first_seen_tick", entry.first_seen},
                        {"armed", armed},
                        {"state", suppressed ? "warming_up" : "armed"},
                        {"lifecycle", lc.has_value() ? nlohmann::json(*lc) : nlohmann::json(nullptr)},
                        // The other half of an unread label, and the half no caller can infer
                        // from elapsed time: whether the watcher still intends to ask. Without
                        // it an operator (and a test) cannot tell a node that is about to be
                        // measured from one that never will be - see measurement_pending().
                        {"measurement_pending", lifecycle_.measurement_pending(id)}});
  }
  const bool global_armed = graph_seen_ && (last_tick_ - graph_first_tick_) >= static_cast<uint64_t>(warmup_cycles_);
  return {{"x-medkit-watchdog",
           {{"schema_version", "1.0.0"},
            {"warmup_cycles", warmup_cycles_},
            {"global_state", global_armed ? "armed" : "warming_up"},
            {"entities", std::move(entities)}}}};
}

std::optional<std::string> ReliabilityGate::lifecycle_state_of(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  return lifecycle_.state_of(app_id);
}

std::optional<DepartedLifecycle> ReliabilityGate::departed_lifecycle_state_of(const std::string & fqn) const {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  return lifecycle_.departed_state_of(fqn);
}

void ReliabilityGate::reset() {
  lifecycle_.reset();
}

void ReliabilityGate::set_lifecycle_state_for_test(const std::string & app_id, const std::string & label,
                                                   int reseeds_remaining) {
  lifecycle_.set_state_for_test(app_id, label, reseeds_remaining);
}

int ReliabilityGate::lifecycle_reseeds_remaining_for_test(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  return lifecycle_.reseeds_remaining_for_test(app_id);
}

void ReliabilityGate::set_departed_lifecycle_state_for_test(const std::string & fqn, const std::string & label,
                                                            bool saw_transition, bool error_terminated) {
  lifecycle_.set_departed_state_for_test(fqn, label, saw_transition, error_terminated);
}

bool reliability_allows(const ReliabilityGate * gate, const std::string & source_id) {
  return gate == nullptr || gate->allows_raise(source_id);
}

PresenceOwnership presence_ownership(const ReliabilityGate * gate, const std::string & source_id) {
  // A null gate is not yet wired (a bare-context test), and every other predicate here treats
  // that as fully permissive. kEarned rather than kProvisional: there is no watcher to ever
  // withdraw the grant, so a provisional answer would leave a caller waiting for news that
  // cannot come.
  return gate == nullptr ? PresenceOwnership::kEarned : gate->presence_ownership(source_id);
}

}  // namespace ros2_medkit_graph_watchdog
