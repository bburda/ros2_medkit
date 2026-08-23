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
#include "ros2_medkit_graph_watchdog/lifecycle_watcher.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"  // find_lifecycle_get_state_path

namespace ros2_medkit_graph_watchdog {

namespace {

/// Cap the number of blocking GetState calls per update() so a batch bringup (Nav2 /
/// MoveIt activating many lifecycle nodes at once) cannot stall the tick loop - and the
/// x-medkit-watchdog handler that shares tick_mutex_ - for seconds. Nodes beyond the cap
/// are still subscribed (so no transition is missed); they seed on a later tick.
constexpr int kMaxGetStateSeedsPerTick = 8;

/// A count cap alone does not bound the STALL: each GetState blocks up to the reader's
/// timeout (500 ms default) when a managed node is slow/unresponsive, so 8 timed-out
/// seeds still freezes the tick loop for ~4 s. On a real Nav2 stack (~15 lifecycle nodes,
/// one stuck mid-transition) this stalled detection for tens of seconds at bringup. So
/// also bound the total blocking time per tick: stop seeding once this wall-clock budget
/// is spent and defer the rest to the next tick. Worst-case blocking is then one in-flight
/// timeout (~500 ms), never N of them; async ~/transition_event keeps state fresh meanwhile.
constexpr std::chrono::milliseconds kGetStateBudgetPerTick{150};

/// The error branch of the default lifecycle state machine. A node reaches it from any
/// primary state via ON_ERROR, and leaves it either back to `unconfigured` (ON_ERROR_SUCCESS,
/// the default on_error) or into `finalized` (ON_ERROR_FAILURE / ON_ERROR_ERROR).
constexpr const char * kErrorProcessingLabel = "errorprocessing";

/// The GetState service for a managed lifecycle node is always published at
/// "<node>/get_state" (rclcpp_lifecycle's "~/get_state" resolved relative to the
/// node); ~/transition_event resolves the same way, so swapping the trailing
/// segment derives the topic without needing a second discovery pass.
std::string derive_transition_event_topic(const std::string & get_state_path) {
  static constexpr char kGetStateSuffix[] = "/get_state";
  static constexpr char kTransitionEventSuffix[] = "/transition_event";
  const std::size_t suffix_len = std::char_traits<char>::length(kGetStateSuffix);
  if (get_state_path.size() >= suffix_len &&
      get_state_path.compare(get_state_path.size() - suffix_len, suffix_len, kGetStateSuffix) == 0) {
    return get_state_path.substr(0, get_state_path.size() - suffix_len) + kTransitionEventSuffix;
  }
  // Shouldn't happen: find_lifecycle_get_state_path always returns a ".../get_state"
  // path. Degrade gracefully (append) rather than throw on an unexpected shape.
  return get_state_path + kTransitionEventSuffix;
}

}  // namespace

LifecycleWatcher::LifecycleWatcher(rclcpp::Node * gateway_node, std::mutex * node_mutex, int departed_retention_ticks)
  : node_(gateway_node)
  , node_mutex_(node_mutex)
  , reader_(std::make_shared<ros2_medkit_gateway::Ros2LifecycleStateReader>(gateway_node))
  , state_(std::make_shared<SharedState>())
  , retention_ticks_(departed_retention_ticks) {
  // One group for every lifecycle subscription, created once here rather than per node:
  // create_callback_group() mutates the node's own group registry and is no more
  // thread-safe than create_subscription(), so it goes under the shared node_mutex_ too.
  //
  // `automatically_add_to_executor_with_node = false` is the load-bearing argument. With
  // it, a gateway executor that has this node added never collects these subscriptions,
  // so no executor thread ever holds a reference to one and none of them can be destroyed
  // on an executor thread. Adding the group to a private executor instead keeps creation,
  // destruction AND callback execution on the single thread that drives this watcher.
  std::lock_guard<std::mutex> node_lock(*node_mutex_);
  lifecycle_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                                  /*automatically_add_to_executor_with_node=*/false);
  lifecycle_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  lifecycle_executor_->add_callback_group(lifecycle_group_, node_->get_node_base_interface());
}

LifecycleWatcher::~LifecycleWatcher() {
  reset();
}

void LifecycleWatcher::update(const ros2_medkit_gateway::IntrospectionInput & snapshot, std::uint64_t tick) {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->shutdown_requested) {
      return;
    }
    // Prune stale departed-lifecycle entries BEFORE this tick's own departures (if any)
    // are recorded below, so a fqn that departs on this very tick is never immediately
    // pruned by the same pass.
    for (auto it = state_->recently_departed_.begin(); it != state_->recently_departed_.end();) {
      const std::uint64_t departed_tick = it->second.second;
      // tick is monotonic, so tick >= departed_tick and the unsigned subtraction cannot wrap.
      if (tick - departed_tick > static_cast<std::uint64_t>(retention_ticks_)) {
        it = state_->recently_departed_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Current set of managed (GetState + ChangeState) node ids, their GetState path, and
  // stable fqn (App::effective_fqn()) - the fqn is captured into a new Tracked entry
  // below and carried into recently_departed_ on departure.
  std::unordered_map<std::string, std::string> current_paths;
  std::unordered_map<std::string, std::string> current_fqns;
  for (const auto & app : snapshot.apps) {
    const auto get_state_path = ros2_medkit_gateway::find_lifecycle_get_state_path(app.services);
    if (!get_state_path) {
      continue;
    }
    current_paths.emplace(app.id, *get_state_path);
    current_fqns.emplace(app.id, app.effective_fqn());
  }

  // Drop nodes that are no longer managed - and tracked ids whose BINDING moved. An
  // entry's binding identity is (fqn, get_state path), both captured at first sighting:
  // App::id alone is not it, because an id can survive a graph sweep while pointing at a
  // DIFFERENT node (id assignment shifts under bare-name collisions). Keeping such an
  // entry would keep enforcing the old node's label - and its old ~/transition_event
  // subscription - against the new binding, so a moved binding is two events at once:
  // the OLD binding departed (recorded under ITS fqn, same record and retention as a
  // vanish), and the id is new again (erased here, so the selection below re-seeds it
  // through the ordinary new-node path: fresh GetState, fresh subscription, fresh
  // self-heal budget).
  //
  // Erasing the entry runs ~Subscription, which mutates node_'s entity registry and
  // waitset - the same structures create_subscription mutates. Two things make that safe,
  // and BOTH are needed:
  //
  //   1. The subscription is in lifecycle_group_, which no gateway executor collects, so
  //      the only references to it live on this thread. A subscription in the gateway's
  //      executor would not have that property: rclcpp::AnyExecutable holds a STRONG ref
  //      to the subscription it dispatches and is a local of the executor thread, so the
  //      last reference is routinely released - and ~Subscription routinely runs - on an
  //      executor thread no lock here can reach.
  //   2. node_mutex_ then state->mutex, mirroring the creation-side lock order
  //      (node_mutex_ -> state->mutex), serializes this against the other entity creation
  //      this plugin does on node_. The callback only takes state->mutex, so no inversion.
  {
    std::lock_guard<std::mutex> node_lock(*node_mutex_);
    std::lock_guard<std::mutex> state_lock(state_->mutex);
    for (auto it = state_->tracked.begin(); it != state_->tracked.end();) {
      const auto path_it = current_paths.find(it->first);
      bool drop = (path_it == current_paths.end());
      if (!drop) {
        const auto fqn_it = current_fqns.find(it->first);
        drop = fqn_it == current_fqns.end() || fqn_it->second != it->second.fqn ||
               path_it->second != it->second.get_state_path;
      }
      if (drop) {
        state_->recently_departed_[it->second.fqn] = {
            DepartedLifecycle{it->second.state_label, it->second.saw_transition, it->second.error_terminated}, tick};
        it = state_->tracked.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Select the GetState work for this tick (bounded), and the new nodes to subscribe.
  // A node needs a (blocking) GetState if it is newly managed, or if it is tracked but
  // still cached non-active with self-heal budget left - re-seeding recovers an `active`
  // transition_event that was published during the sub's DDS matching window and lost.
  struct SeedJob {
    std::string id;
    std::string path;
    bool is_new;
  };
  std::vector<SeedJob> jobs;
  std::vector<std::pair<std::string, std::string>> new_nodes;  // (id, get_state_path)
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    for (const auto & [id, path] : current_paths) {
      auto it = state_->tracked.find(id);
      if (it == state_->tracked.end()) {
        new_nodes.emplace_back(id, path);
        if (static_cast<int>(jobs.size()) < kMaxGetStateSeedsPerTick) {
          jobs.push_back({id, path, true});
        }
      } else if (it->second.state_label != "active" && it->second.reseeds_remaining > 0 &&
                 static_cast<int>(jobs.size()) < kMaxGetStateSeedsPerTick) {
        // NOT charged here: the blocking loop below can break on its wall-clock budget before
        // reaching this job, and a job that never issued a GetState must not spend an attempt.
        // One node stuck mid-transition eats the whole 150ms budget on its own timeout, so every
        // job queued behind it that tick used to be skipped AND charged - two ticks of that
        // drained both attempts with zero reads. The consequences are permanent for the process:
        // a node whose seed never ran keeps label "" (never gated, and a require_active
        // expectation on it can never be CONFIRMED - GRAPH_NODE_UNREADABLE is what reports that
        // instead, once its own hold expires), and one seeded non-active that then activates
        // inside the subscription's matching window keeps that label forever (every fault
        // suppressed, plus a permanent false GRAPH_NODE_INACTIVE).
        jobs.push_back({id, path, false});
      }
    }
  }

  // Blocking GetState calls - run WITHOUT any lock held (the reader spins a private
  // executor inline up to its timeout; holding state->mutex would serialize every
  // concurrent status read behind it).
  std::unordered_map<std::string, std::string> seeded;
  std::vector<std::string> reseeds_performed;  // re-seed jobs whose GetState actually ran
  const auto seed_deadline = std::chrono::steady_clock::now() + kGetStateBudgetPerTick;
  for (const auto & job : jobs) {
    seeded[job.id] = reader_->get_state(job.path).value_or("");
    if (!job.is_new) {
      reseeds_performed.push_back(job.id);
    }
    // Stop once the per-tick blocking budget is spent; the unseeded jobs are already
    // subscribed (new nodes) or keep their reseed budget (tracked nodes), so they are
    // re-picked and seeded on a later tick without the tick loop ever stalling here.
    if (std::chrono::steady_clock::now() >= seed_deadline) {
      break;
    }
  }

  // Subscribe newly-managed nodes and apply their seed. rclcpp_lifecycle publishes
  // ~/transition_event reliable + volatile (rcl_lifecycle uses the default publisher QoS
  // with no durability override). A transient_local subscriber would be QoS-incompatible
  // with a volatile publisher and silently receive zero messages, so stay volatile. The
  // callback captures a weak_ptr to SharedState (never a raw `this`) so it stays safe if
  // the watcher is torn down while a callback is in flight.
  for (const auto & new_node : new_nodes) {
    // Bind to named references (not a captured structured binding, which is a C++20
    // extension) so the callback below can capture `id` under C++17.
    const std::string & id = new_node.first;
    const std::string & get_state_path = new_node.second;
    const std::string topic = derive_transition_event_topic(get_state_path);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    std::weak_ptr<SharedState> weak_state = state_;
    // Every lifecycle subscription goes into the watcher's own group, never the node's
    // default one - see the constructor for why that is what makes teardown safe.
    rclcpp::SubscriptionOptions options;
    options.callback_group = lifecycle_group_;

    std::lock_guard<std::mutex> node_lock(*node_mutex_);
    std::lock_guard<std::mutex> state_lock(state_->mutex);
    if (state_->shutdown_requested) {
      return;
    }
    auto sub = node_->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        topic, qos,
        [weak_state, id](const lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr & msg) {
          auto shared = weak_state.lock();
          if (!shared) {
            return;  // watcher torn down; nothing to update
          }
          std::lock_guard<std::mutex> cb_lock(shared->mutex);
          if (shared->shutdown_requested) {
            return;
          }
          // No re-check that this entry is still the binding the subscription was created
          // for: a moved binding erases the entry, and erasing it destroys THIS
          // subscription, on this very thread (see the class note in the header - the
          // private executor that runs this callback is pumped by the same thread that
          // runs update()). A message from a binding that is already gone therefore has
          // nowhere to arrive from.
          auto it = shared->tracked.find(id);
          if (it != shared->tracked.end()) {
            it->second.state_label = msg->goal_state.label;
            it->second.saw_transition = true;
            // `errorprocessing` on either end of the edge means this node took the error
            // branch. It is sticky for the node's whole tracked lifetime: ON_ERROR_FAILURE
            // and ON_ERROR_ERROR both land in `finalized`, so by the time we see the final
            // label the error edge is already behind us.
            if (msg->start_state.label == kErrorProcessingLabel || msg->goal_state.label == kErrorProcessingLabel) {
              it->second.error_terminated = true;
            }
          }
        },
        options);
    auto & tracked = state_->tracked[id];
    tracked.sub = std::move(sub);
    tracked.reseeds_remaining = LifecycleWatcher::kReseedAttempts;
    const auto seed_it = seeded.find(id);
    tracked.state_label = (seed_it != seeded.end()) ? seed_it->second : "";
    // Captured at first sighting: current_fqns is built from the same snapshot pass
    // that produced current_paths, so an id here is always present in current_fqns too.
    // Together, fqn + get_state_path are the binding identity the drop loop re-checks.
    const auto fqn_it = current_fqns.find(id);
    tracked.fqn = (fqn_it != current_fqns.end()) ? fqn_it->second : "";
    tracked.get_state_path = get_state_path;
  }

  // Apply re-seed results to already-tracked nodes. Never overwrite a good cached label
  // with an empty (failed/timed-out) seed. The read above ran with no lock held, but no
  // ~/transition_event can have overtaken it: the private executor that delivers them is
  // pumped by this same thread, between ticks, so nothing writes state_label while a seed
  // is in flight (see the class note in the header).
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    // Charge the self-heal budget only for reads that actually happened (see the selection
    // loop). A node whose job was cut by the budget keeps its attempts and is re-picked next tick.
    for (const auto & id : reseeds_performed) {
      auto it = state_->tracked.find(id);
      if (it != state_->tracked.end() && it->second.reseeds_remaining > 0) {
        it->second.reseeds_remaining -= 1;
      }
    }
    for (const auto & job : jobs) {
      if (job.is_new) {
        continue;
      }
      const auto seed_it = seeded.find(job.id);
      if (seed_it == seeded.end() || seed_it->second.empty()) {
        continue;
      }
      auto it = state_->tracked.find(job.id);
      if (it != state_->tracked.end()) {
        it->second.state_label = seed_it->second;
      }
    }
  }
}

void LifecycleWatcher::pump_events(std::chrono::nanoseconds budget) {
  // No lock: this runs the ~/transition_event callbacks, and those take state->mutex
  // themselves. Holding either mutex across the spin would deadlock the callback, and
  // holding node_mutex_ would stall every other entity creation for the whole drain.
  // Single-thread safety comes from the calling contract (same thread as update(); see
  // the class note), not from a lock.
  //
  // spin_some() collects once and returns as soon as the ready work is drained - it never
  // blocks waiting for new work - so this is a poll, not a spin. It is also the point
  // where the private executor takes and releases its own transient strong references to
  // the subscriptions, which is precisely why it must run on the same thread that erases
  // them.
  if (lifecycle_executor_) {
    lifecycle_executor_->spin_some(budget);
  }
}

bool LifecycleWatcher::node_ok(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto it = state_->tracked.find(app_id);
  if (it == state_->tracked.end()) {
    return true;
  }
  // An empty label means the one-shot GetState seed failed and no transition_event
  // has arrived. Treat unknown as "do not gate": transition_event is volatile and
  // never backfills an already-active node, so gating on an empty label would
  // suppress that node's faults forever. Only a KNOWN non-active label gates.
  return it->second.state_label.empty() || it->second.state_label == "active";
}

bool LifecycleWatcher::measurement_pending(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto it = state_->tracked.find(app_id);
  if (it == state_->tracked.end()) {
    return false;
  }
  // Both conjuncts matter. A non-empty label is a measurement, however stale. An empty one
  // with no attempts left is a measurement that will never arrive: update() only queues a
  // re-seed while `reseeds_remaining > 0`, so nothing here will issue another GetState for
  // this entry for the rest of its tracked life.
  return it->second.state_label.empty() && it->second.reseeds_remaining > 0;
}

std::optional<std::string> LifecycleWatcher::state_of(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto it = state_->tracked.find(app_id);
  if (it == state_->tracked.end()) {
    return std::nullopt;
  }
  return it->second.state_label;
}

std::optional<DepartedLifecycle> LifecycleWatcher::departed_state_of(const std::string & fqn) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto it = state_->recently_departed_.find(fqn);
  if (it == state_->recently_departed_.end()) {
    return std::nullopt;
  }
  return it->second.first;
}

void LifecycleWatcher::reset() {
  // Symmetric with subscription creation (node_mutex_ then state->mutex): clearing
  // tracked drops the subs, and ~Subscription mutates node_'s entity registry.
  //
  // Unlike update(), this does NOT run on the tick thread - the plugin calls it from
  // whoever is tearing the plugin down, after that thread is joined. Destroying the
  // private executor FIRST is what keeps the destruction single-threaded anyway: while it
  // is alive it may still hold its own strong reference to a collected subscription, so
  // clearing `tracked` would not necessarily release the last one, and the release would
  // then happen at some later, less obvious point. Tearing the executor down here means
  // every reference is gone by the time `tracked` is cleared.
  //
  // node_mutex_ is held across the whole body, which is what excludes a concurrent
  // create_subscription in update() (it takes the same mutex). The two teardown steps are
  // each idempotent - a null unique_ptr and an already-empty map - so the second call (the
  // destructor, after the plugin already called reset() through the gate) does nothing.
  std::lock_guard<std::mutex> node_lock(*node_mutex_);
  lifecycle_executor_.reset();
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  state_->shutdown_requested = true;
  state_->tracked.clear();
}

int LifecycleWatcher::reseeds_remaining_for_test(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto it = state_->tracked.find(app_id);
  return it == state_->tracked.end() ? -1 : it->second.reseeds_remaining;
}

void LifecycleWatcher::set_state_for_test(const std::string & app_id, const std::string & label,
                                          int reseeds_remaining) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  auto & tracked = state_->tracked[app_id];
  tracked.state_label = label;
  tracked.reseeds_remaining = reseeds_remaining;
  // This seam stands in for a live ~/transition_event round-trip, so it must record the same
  // thing that callback does: we WATCHED the node reach this label. Leaving saw_transition false
  // would make every shim-driven departure look like a node found already sitting in its final
  // state, which is a different (and deliberately unclassifiable) case. Use
  // set_departed_state_for_test() to stage that one.
  tracked.saw_transition = true;
}

void LifecycleWatcher::set_departed_state_for_test(const std::string & fqn, const std::string & label,
                                                   bool saw_transition, bool error_terminated) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->recently_departed_[fqn] = {DepartedLifecycle{label, saw_transition, error_terminated}, 0};
}

}  // namespace ros2_medkit_graph_watchdog
