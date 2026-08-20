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
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"  // IntrospectionInput
#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"

namespace ros2_medkit_graph_watchdog {

/// Fallback departed-lifecycle retention horizon (ticks) when a caller does not compute
/// its own (e.g. a unit test constructing a LifecycleWatcher/ReliabilityGate directly).
/// Production wiring (GraphWatchdogPlugin::set_context()) always passes an explicit
/// value derived from the plugin's own prune_grace / node_death's miss_grace.
inline constexpr int kDefaultDepartedRetentionTicks = 60;

/// What a departure looked like, as far as ~/transition_event let us observe it. The last
/// label alone cannot tell a lifecycle-manager shutdown from an error termination:
/// `finalized` is the resting state of BOTH a completed shutdown and a failed on_error
/// (ON_ERROR_FAILURE/ON_ERROR_ERROR out of `errorprocessing`), which is exactly how a driver
/// signals an unrecoverable hardware fault. So carry the extra evidence.
struct DepartedLifecycle {
  std::string label;              ///< last cached lifecycle label at the moment of departure
  bool saw_transition = false;    ///< at least one ~/transition_event arrived for this node
  bool error_terminated = false;  ///< the node passed through `errorprocessing`
};

/// Tracks managed lifecycle nodes and caches their state label ("active" etc.),
/// seeded via GetState and kept fresh via ~/transition_event. Non-managed nodes are
/// never tracked (node_ok returns true for them).
///
/// Threading contract - this is what makes the ~/transition_event subscriptions safe to
/// create and destroy on a live gateway node. The subscriptions are created on the shared
/// gateway node but placed in a callback group of this watcher's own, created with
/// `automatically_add_to_executor_with_node = false` and added ONLY to the private
/// executor below. No gateway executor ever collects them, so no gateway executor thread
/// can ever hold a reference to one, and `~Subscription` (which mutates the node's rcl
/// entity registry, the same structure `create_subscription` mutates) can only run on the
/// thread that owns this watcher.
///
/// Concretely, `update()` and `pump_events()` MUST be called from one and the same thread
/// (the plugin's tick thread), and `reset()`/the destructor MUST NOT run while either is
/// in flight (the plugin joins its tick thread before tearing the gate down). The const
/// readers - node_ok(), state_of(), measurement_pending(), departed_state_of() - are the
/// only members safe to call from another thread; they take the shared-state mutex and
/// touch no ROS entity.
class LifecycleWatcher {
 public:
  /// How many extra GetState re-seeds a tracked-but-non-active node gets to self-heal a
  /// missed activation. Two ticks of coverage comfortably outlast the tens-of-ms DDS
  /// endpoint-matching window; after that a matched subscription catches every transition.
  /// Public because it is also the bound on how long an unmeasured node's ignorance can
  /// still resolve - see measurement_pending().
  static constexpr int kReseedAttempts = 2;

  LifecycleWatcher(rclcpp::Node * gateway_node, std::mutex * node_mutex,
                   int departed_retention_ticks = kDefaultDepartedRetentionTicks);
  ~LifecycleWatcher();
  LifecycleWatcher(const LifecycleWatcher &) = delete;
  LifecycleWatcher & operator=(const LifecycleWatcher &) = delete;
  LifecycleWatcher(LifecycleWatcher &&) = delete;
  LifecycleWatcher & operator=(LifecycleWatcher &&) = delete;

  /// Discover managed nodes from the snapshot (find_lifecycle_get_state_path over
  /// App::services): seed+subscribe new ones, drop vanished ones, and drop + re-seed
  /// tracked ids whose BINDING moved (same App::id, different fqn / get_state path -
  /// see Tracked::get_state_path). `tick` is the plugin's own tick counter (threaded
  /// through from ReliabilityGate::update()) - used to timestamp a departure into
  /// `recently_departed_` and to prune stale entries from it.
  void update(const ros2_medkit_gateway::IntrospectionInput & snapshot, std::uint64_t tick);

  /// Execute the ~/transition_event callbacks that are already pending, for at most
  /// `budget` of wall clock. Nothing else runs these subscriptions (see the class note),
  /// so the owner has to call this or transition events never land - and calling it once
  /// per tick would delay every event by up to a full tick interval and let a bringup
  /// burst overrun the subscription queue. The plugin therefore polls it while it waits
  /// for the next tick. MUST be called from the same thread as update(); it never blocks
  /// waiting for work, so `budget` only caps a pathological burst.
  void pump_events(std::chrono::nanoseconds budget);

  bool node_ok(const std::string & app_id) const;  // true if untracked or "active"
  std::optional<std::string> state_of(const std::string & app_id) const;
  /// True while `app_id` is a tracked managed node whose state is not measured YET and
  /// still can be: the label is empty and the GetState self-heal budget has attempts left.
  ///
  /// This is what separates transient ignorance from settled ignorance, and the two must
  /// not be treated alike. `reseeds_remaining` is charged only for a read that actually
  /// ran (see update()), so it answers "have we finished asking" rather than "how many
  /// ticks have passed". False for an untracked node, false for one carrying a real label,
  /// and false once the budget is spent: at that point nothing in this class will ever
  /// issue another GetState for it, so the label can only still change through a
  /// ~/transition_event the node may never publish. A caller deciding who OWNS such a
  /// node's departure must treat that as settled and take it, or the node is reported by
  /// nobody - see ReliabilityGate::presence_ownership().
  bool measurement_pending(const std::string & app_id) const;
  /// Last cached lifecycle label of a node that WAS tracked but is no longer present in
  /// the snapshot (departed within `departed_retention_ticks` ticks of "now"), keyed by
  /// its stable fqn (App::effective_fqn(), NOT App::id - mirrors node_death's own keying,
  /// see node_death_detector.cpp). nullopt if the fqn never departed or has aged out of
  /// retention. Does NOT return a still-present node's state - node_death only ever
  /// queries fqns already absent from the graph.
  std::optional<DepartedLifecycle> departed_state_of(const std::string & fqn) const;
  /// Shutdown: tear down the private executor and drop every subscription. Callers must
  /// have stopped update()/pump_events() first (see the class note); the destructor calls
  /// it too, and it is idempotent.
  void reset();

  /// Test seam: stage a tracked entry directly. `reseeds_remaining` defaults to the same
  /// budget a freshly discovered node gets, so an injected EMPTY label reads as transient
  /// ignorance (the state a node is in right after discovery); pass 0 to stage the settled
  /// form a node reaches once its GetState has been asked and never answered.
  void set_state_for_test(const std::string & app_id, const std::string & label,
                          int reseeds_remaining = kReseedAttempts);
  /// Test-only: how much of the bounded GetState self-heal budget a tracked node has left.
  /// -1 if the node is not tracked. Lets a test assert that a re-seed job cut by the per-tick
  /// blocking budget was not charged for a read it never issued.
  int reseeds_remaining_for_test(const std::string & app_id) const;
  /// Test-only seam: inject a departed-lifecycle entry directly, without driving a
  /// managed node through discovery + real removal - lets a suppressor unit test drive
  /// departed_state_of() without a live GetState/transition_event round-trip. Defaults
  /// describe an observed, non-error departure; pass error_terminated to stage the
  /// failed-on_error shape.
  void set_departed_state_for_test(const std::string & fqn, const std::string & label, bool saw_transition = true,
                                   bool error_terminated = false);

 private:
  struct Tracked {
    std::string state_label;
    /// Stable fqn (App::effective_fqn()), captured at first sighting - carried into
    /// recently_departed_ on departure so it can be looked up by the same key
    /// node_death itself uses (App::id is not stable across a sibling's death, see
    /// node_death_detector.cpp).
    std::string fqn;
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub;
    /// Bounded self-heal budget: re-seed via GetState while the cached label is
    /// non-active, to recover an `active` transition_event lost during the DDS
    /// endpoint-matching window right after the (volatile) subscription is created.
    int reseeds_remaining = 0;
    /// The GetState path this entry was created from. Together with `fqn` it is the
    /// entry's BINDING identity, which update() re-checks every tick: the subscription
    /// topic derives from this path, so if either piece moves under an unchanged
    /// App::id, the entry describes a different node than the snapshot now binds and
    /// must be dropped + re-seeded (an id kept across such a move would keep enforcing
    /// the old node's label against the new binding).
    std::string get_state_path;
    /// Set by the ~/transition_event callback - see DepartedLifecycle for why the last
    /// label alone is not enough to classify a departure.
    bool saw_transition = false;
    bool error_terminated = false;
  };
  /// Everything the volatile ~/transition_event callback touches lives behind this
  /// shared_ptr. The callback captures a weak_ptr (never a raw `this`): a callback that
  /// races teardown either locked first (keeping this alive for its duration) or finds
  /// lock() empty and bails, so it never dereferences a destroyed mutex/map. The private
  /// executor runs those callbacks on the same thread that calls update(), so a callback
  /// cannot actually be in flight while an entry is erased - the weak_ptr guard is kept
  /// as the standing defence for the shared state, not as the fix for the entity race.
  struct SharedState {
    std::mutex mutex;
    std::unordered_map<std::string, Tracked> tracked;  // key = App::id
    /// A departed node's observed departure, keyed by its stable fqn (NOT App::id) -
    /// value = {departure, departed_tick}. Populated on the drop path in update() just
    /// before a tracked entry is erased - whether the id left the managed set or its
    /// binding moved (either way, the OLD binding departed); pruned (in update(), keyed
    /// off `tick`) once `tick - departed_tick > retention_ticks_`.
    std::map<std::string, std::pair<DepartedLifecycle, std::uint64_t>> recently_departed_;
    bool shutdown_requested = false;
  };
  rclcpp::Node * node_;
  std::mutex * node_mutex_;
  std::shared_ptr<ros2_medkit_gateway::Ros2LifecycleStateReader> reader_;
  std::shared_ptr<SharedState> state_;
  int retention_ticks_;
  /// The lifecycle subscriptions' own callback group, created with automatic executor
  /// registration DISABLED so no executor the gateway node is added to ever collects
  /// them - that is what keeps every reference to them on this watcher's own thread.
  rclcpp::CallbackGroup::SharedPtr lifecycle_group_;
  /// The only executor that group is ever added to. Declared last so it is torn down
  /// before the group (and, in a reset()-less path, before `state_`'s subscriptions).
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> lifecycle_executor_;
};

}  // namespace ros2_medkit_graph_watchdog
