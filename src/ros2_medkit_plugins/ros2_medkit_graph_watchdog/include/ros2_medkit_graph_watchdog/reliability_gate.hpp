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
#include <mutex>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"  // IntrospectionInput
#include "ros2_medkit_graph_watchdog/lifecycle_watcher.hpp"
#include "ros2_medkit_graph_watchdog/presence_ownership.hpp"
#include "ros2_medkit_graph_watchdog/warmup_tracker.hpp"

namespace ros2_medkit_graph_watchdog {

/// Central gate composing per-entity warmup (WarmupTracker) with ROS lifecycle state
/// (LifecycleWatcher). An entity is allowed to raise a fault once it has been armed
/// (continuously present for warmup_cycles ticks) and its lifecycle state is not a KNOWN
/// non-active one: a managed node whose GetState has never answered still passes, because
/// gating on an unread label would silence every detector for it (LifecycleWatcher::node_ok()).
/// Unknown source_ids (e.g. topics, not tracked apps) fall back to a global bringup grace
/// period keyed off the first tick the graph was non-empty. presence_ownership() below asks
/// the narrower question the presence class needs, without moving this one.
class ReliabilityGate {
 public:
  ReliabilityGate(int warmup_cycles, rclcpp::Node * gateway_node, std::mutex * node_mutex,
                  int departed_retention_ticks = kDefaultDepartedRetentionTicks);

  /// Feed a fresh introspection snapshot at `tick`: updates per-entity warmup,
  /// lifecycle tracking, and the global bringup marker.
  void update(const ros2_medkit_gateway::IntrospectionInput & snapshot, uint64_t tick);

  /// Drain the LifecycleWatcher's pending ~/transition_event callbacks (see
  /// LifecycleWatcher::pump_events). Nothing else runs those subscriptions, so the tick
  /// thread has to poll this between ticks; it MUST be the same thread that calls
  /// update().
  void pump_lifecycle_events(std::chrono::nanoseconds budget);

  /// True if `source_id` is allowed to raise a fault: known entities must be armed
  /// and lifecycle-ok; unknown entities fall back to the global warmup window.
  bool allows_raise(const std::string & source_id) const;

  /// On what grounds the PRESENCE detector (node_death) owns this source's departure, listed
  /// in the order the cases are decided, because that order is itself load-bearing:
  ///
  ///   1. a label that is neither empty nor "active"  -> kDisowned    (measured elsewhere)
  ///   2. not allowed to raise                        -> kUnclaimed   (warming up)
  ///   3. no managed record at all (a plain node)     -> kEarned
  ///   4. empty label, GetState attempts still to run -> kUnclaimed   (still asking)
  ///   5. empty label, every attempt spent            -> kProvisional (asked, and failed)
  ///   6. the label reads "active"                    -> kEarned
  ///
  /// Case 1 is deliberately decided BEFORE arming, and not merely for tidiness: node_ok()
  /// already refuses a managed non-active node, so asking allows_raise() first would answer
  /// the same for a node the graph has measured as inactive and for one that has simply not
  /// armed yet. A caller that gives up a key on the negative answer would then drop every
  /// node that restarts, since a returning node is un-armed for its whole re-warm - see
  /// PresenceOwnership for why only kDisowned may take a key away.
  ///
  /// Narrower than allows_raise(), and deliberately so. allows_raise() answers "may this
  /// entity raise", and for a managed node whose label has never been read it answers yes,
  /// because gating every detector on an unread label would silence a node whose GetState
  /// never answers (see LifecycleWatcher::node_ok()). Ownership needs knowledge rather than
  /// permission, so an unread label answers kUnclaimed while that ignorance can still resolve.
  ///
  /// It does NOT stay kUnclaimed forever: `lifecycle_expectation` is the only other detector
  /// that could report such a node's departure, and it looks at nothing unless an operator
  /// named the node in `require_active`, which defaults to empty. So once the watcher has
  /// stopped asking, the answer becomes kProvisional - owned, because otherwise nobody
  /// reports this death, and provisional, because the label may still arrive over
  /// ~/transition_event (that subscription outlives the seed budget) and would then say the
  /// node was never this detector's. See PresenceOwnership for what each caller owes the
  /// distinction.
  PresenceOwnership presence_ownership(const std::string & source_id) const;

  /// Raw lifecycle state label for `app_id` (delegates to the internal
  /// LifecycleWatcher), or nullopt if `app_id` is not a tracked managed lifecycle
  /// node. Additive, const, plugin-internal seam for detectors that need the raw
  /// label rather than the gated node_ok()/allows_raise() boolean (e.g.
  /// lifecycle_expectation_detector, which enforces its OWN grace instead of the
  /// gate's - see that detector's design note on why it cannot use
  /// reliability_allows()).
  std::optional<std::string> lifecycle_state_of(const std::string & app_id) const;

  /// How a node that departed the graph within the gate's departed-retention horizon
  /// left it, keyed by its stable fqn (delegates to the internal LifecycleWatcher).
  /// nullopt if the fqn never departed, or has aged out of retention. Additive, const,
  /// plugin-internal seam for LifecycleShutdownSuppressor.
  std::optional<DepartedLifecycle> departed_lifecycle_state_of(const std::string & fqn) const;

  /// SOVD `x-medkit-watchdog` extension payload: schema_version, warmup_cycles, global_state,
  /// and per entity id/first_seen_tick/armed/state/lifecycle/measurement_pending. The last two
  /// are a pair on purpose: `lifecycle` is null for a node with no managed record and "" for
  /// one whose GetState has not answered, and `measurement_pending` says whether that second
  /// case is still being asked about - which nothing else on the route distinguishes.
  nlohmann::json status_json() const;

  /// Drop lifecycle subscriptions (shutdown path).
  void reset();

  /// Test seam: inject a lifecycle state label for `app_id` so a test can drive the
  /// composed warmup + lifecycle path (an armed entity whose lifecycle is non-active)
  /// through allows_raise()/status_json() without a live managed node. `reseeds_remaining`
  /// stages the OTHER half of an unmeasured node's state (see
  /// LifecycleWatcher::measurement_pending): the default is what a freshly discovered node
  /// carries, and 0 is the settled form.
  void set_lifecycle_state_for_test(const std::string & app_id, const std::string & label,
                                    int reseeds_remaining = LifecycleWatcher::kReseedAttempts);

  /// Test seam: GetState re-seed budget the internal LifecycleWatcher has left for
  /// `app_id`, or -1 if it is not tracked. Lets a test tie the ownership answer to the
  /// budget actually running out rather than to a tick count that happens to correlate.
  int lifecycle_reseeds_remaining_for_test(const std::string & app_id) const;

  /// Test-only seam: inject a departed-lifecycle entry directly (delegates to the
  /// internal LifecycleWatcher) so a suppressor unit test can drive
  /// departed_lifecycle_state_of() without a live GetState/transition_event round-trip
  /// or a real node removal.
  void set_departed_lifecycle_state_for_test(const std::string & fqn, const std::string & label,
                                             bool saw_transition = true, bool error_terminated = false);

 private:
  // update() runs on the plugin's tick thread; status_json() runs on an HTTP handler
  // thread. warmup_ (an unordered_map) and the scalars below have no internal lock, so
  // gate_mutex_ serializes the writer (update) against the cross-thread reader
  // (status_json). lifecycle_ is separately self-synchronized and is deliberately updated
  // OUTSIDE gate_mutex_ (it does blocking reads); allows_raise() and presence_ownership()
  // are readers that only run on the tick thread (sequentially after update), so they need no
  // lock - the lifecycle_ reads inside them take that class's own mutex.
  mutable std::mutex gate_mutex_;
  WarmupTracker warmup_;
  LifecycleWatcher lifecycle_;
  int warmup_cycles_;
  uint64_t last_tick_ = 0;
  bool graph_seen_ = false;
  uint64_t graph_first_tick_ = 0;
};

}  // namespace ros2_medkit_graph_watchdog
