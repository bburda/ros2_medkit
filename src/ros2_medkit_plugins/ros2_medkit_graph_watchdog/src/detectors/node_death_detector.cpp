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
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/allowlist_suppressor.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/lifecycle_shutdown_suppressor.hpp"
#include "ros2_medkit_graph_watchdog/node_liveness_tracker.hpp"
#include "ros2_medkit_graph_watchdog/suppressor.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// A dead node is a total, silent loss of whatever it provided - as severe as QoS starvation.
constexpr std::uint8_t kNodeDeathSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;

// Ticks a tracked node may be absent before it is reported dead. Mirrors
// GraphWatchdogPlugin's own kDefaultNodeDeathMissGrace, which needs this value before
// this detector's own configure() has run (see that plugin's
// compute_departed_retention_ticks()).
constexpr int kDefaultMissGrace = 2;

// Standalone fallback only. A real launch always injects "prune_grace" (the plugin's own
// default, also 60) once wired through GraphWatchdogPlugin::set_context() - this only
// matters for a bare configure() call that never goes through the plugin, e.g. a unit test.
constexpr int kDefaultPruneGrace = 60;

// Bounds for detectors.node_death.tracked_node_cap - same range, same reasoning, as
// lifecycle_expectation_detector.cpp's identical kMinTrackedNodeCap/kMaxTrackedNodeCap: the
// range check runs on the WIDE integer before narrowing, for the same reason miss_grace and
// prune_grace above do. Zero is refused rather than clamped: a cap of nothing means this
// detector tracks nothing, which is exactly the silence it exists to prevent.
constexpr std::int64_t kMinTrackedNodeCap = 1;
constexpr std::int64_t kMaxTrackedNodeCap = 16384;

/// Whether `fqn`'s leaf is one of ROS 2's own short-lived CLI helper nodes.
///
/// `ros2 topic echo`, `ros2 param get` and every other `ros2` CLI invocation spin up a
/// real node named `_ros2cli_<pid>` - there is no hidden-node filter in discovery, so the
/// gateway turns each one into an ordinary App. A few seconds of one of those is enough to
/// arm it, and the moment the CLI process exits looks exactly like a death; because every
/// invocation gets a fresh pid, letting these through would accumulate one permanently
/// "dead" entry per CLI invocation for the life of the gateway. The prefix is rcl's own
/// naming convention for hidden nodes, so matching it is an exact structural check, not a
/// heuristic over operator-chosen names.
/// Whether the gate can currently read this app's lifecycle label as "active" - the one
/// positive measurement that says a node is the presence detector's. Distinct from a kEarned
/// answer, which is also given to a node that carries no lifecycle at all.
bool reads_active(const ReliabilityGate * gate, const std::string & app_id) {
  if (gate == nullptr) {
    return true;  // not wired (a bare-context test): every other predicate here is permissive
  }
  const auto label = gate->lifecycle_state_of(app_id);
  return label.has_value() && *label == "active";
}

bool is_ros2cli_node(const std::string & fqn) {
  const auto slash = fqn.rfind('/');
  const std::string leaf = slash == std::string::npos ? fqn : fqn.substr(slash + 1);
  return leaf.rfind("_ros2cli_", 0) == 0;
}
}  // namespace

/// Watches every App the graph carries for one that was armed and then vanishes, and
/// raises GRAPH_NODE_DISAPPEARED naming it. Zero-config: unlike lifecycle_expectation's
/// operator-declared require_active list, every App in scope is a candidate - see the
/// package design note this mirrors.
///
/// Liveness is `App::is_online`, never mere membership in the snapshot. In runtime
/// discovery a dead node's App leaves the snapshot entirely, so the two happen to
/// coincide; but a manifest keeps a bound App present with only `is_online` cleared once
/// its node disappears, so counting membership alone would make a manifest node immortal.
/// A managed lifecycle node that merely deactivates keeps `is_online=true` (its process is
/// still alive; only its ROS 2 lifecycle state changed), so that is never mistaken for
/// death either - lifecycle state is lifecycle_expectation's concern, not this one's.
///
/// A key is tracked - and so can ever be reported dead - only once the reliability gate has
/// said this detector OWNS its departure, which needs it armed AND its lifecycle state
/// either known not to be a managed-non-active one or asked for as often as it ever will be.
/// A manifest App that never comes online starts `is_online=false` and is skipped above
/// regardless, so it can never be falsely called dead; a node still inside its warmup window
/// gets the same protection; and a MANAGED node whose state has not been read YET is left to
/// lifecycle_expectation while the watcher still has GetState attempts to spend on it. Once
/// those are spent the node comes back here, because nothing will ASK again and
/// lifecycle_expectation looks at nothing unless an operator named it in `require_active`.
/// That last admission is PROVISIONAL: the ~/transition_event subscription outlives the seed
/// budget, so an unasked-for label can still arrive, and one that reads non-active means the
/// node was never this detector's - it is released while still alive rather than reported dead
/// afterwards. The gate itself stays permissive about an unread label throughout (see
/// LifecycleWatcher::node_ok()), so this whole decision is made here, not there.
///
/// Composable/component nodes hosted in one process die together: if the container
/// process dies, every node it hosted vanishes from the snapshot on the same tick, and all
/// of them are folded into the one aggregated fault rather than one fault each (see
/// AggregatedFault) - individually named up to AggregatedFault::kMaxDescriptionChars and
/// NodeLivenessTracker::kMaxNamedDepartedEntries, whichever runs out first; past either
/// limit the remainder are still counted, just not named (see NodeLivenessTracker's own
/// collapsed-count doc).
class NodeDeathDetector : public Detector {
 public:
  std::string id() const override {
    return "node_death";
  }

  void configure(const nlohmann::json & config) override {
    std::vector<std::string> warnings;

    // tick_interval_ms is plugin plumbing (see plugin_injected_detector_keys()), but the
    // miss_grace floor below is computed FROM it, so a malformed value here would corrupt
    // that derivation silently unless it too is validated and named.
    int tick_interval_ms = kDefaultTickIntervalMs;
    if (config.contains("tick_interval_ms")) {
      const auto & value = config["tick_interval_ms"];
      // ROS parameters are int64: get the wide value and range-check it BEFORE narrowing,
      // or a value above INT_MAX wraps back into the accepted band and passes silently.
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : 0;
      if (wide > 0 && wide <= std::numeric_limits<int>::max()) {
        tick_interval_ms = static_cast<int>(wide);
      } else {
        warnings.push_back("'tick_interval_ms' must be a positive integer up to " +
                           std::to_string(std::numeric_limits<int>::max()) + "; keeping " +
                           std::to_string(tick_interval_ms));
      }
    }

    int miss_grace = kDefaultMissGrace;
    if (config.contains("miss_grace")) {
      const auto & value = config["miss_grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxNodeDeathGraceTicks) {
        miss_grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'miss_grace' must be an integer in 0.." + std::to_string(kMaxNodeDeathGraceTicks) +
                           "; keeping " + std::to_string(kDefaultMissGrace));
      }
    }
    // The wall-clock floor: a death is reported once misses EXCEED miss_grace, i.e. after
    // miss_grace + 1 ticks, so raise the tick count until that window spans at least
    // kMinNodeDeathWindowMs - and say so, since a silently-shortened tolerance is exactly
    // how a fast tick turns one stale graph-cache generation into an immediate false death.
    const int floor = min_node_death_miss_grace(tick_interval_ms);
    if (miss_grace < floor) {
      warnings.push_back("'miss_grace' " + std::to_string(miss_grace) + " spans only " +
                         std::to_string((miss_grace + 1) * tick_interval_ms) + "ms at a " +
                         std::to_string(tick_interval_ms) + "ms tick, under the " +
                         std::to_string(kMinNodeDeathWindowMs) + "ms floor one graph-cache refresh needs; using " +
                         std::to_string(floor));
      miss_grace = floor;
    }

    int prune_grace = kDefaultPruneGrace;
    if (config.contains("prune_grace")) {
      const auto & value = config["prune_grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxNodeDeathGraceTicks) {
        prune_grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'prune_grace' must be an integer in 0.." + std::to_string(kMaxNodeDeathGraceTicks) +
                           "; keeping " + std::to_string(kDefaultPruneGrace));
      }
    }

    // The hard bound on how many keys tracker_ keeps state for at once. Same validation
    // shape as miss_grace/prune_grace above; see NodeLivenessTracker::kDefaultTrackedKeyCap
    // for why 512 is comfortable even against this detector's full-graph scope.
    int tracked_node_cap = NodeLivenessTracker::kDefaultTrackedKeyCap;
    if (config.contains("tracked_node_cap")) {
      const auto & value = config["tracked_node_cap"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= kMinTrackedNodeCap && wide <= kMaxTrackedNodeCap) {
        tracked_node_cap = static_cast<int>(wide);
      } else {
        warnings.push_back("'tracked_node_cap' must be an integer in " + std::to_string(kMinTrackedNodeCap) + ".." +
                           std::to_string(kMaxTrackedNodeCap) + "; keeping the default (" +
                           std::to_string(NodeLivenessTracker::kDefaultTrackedKeyCap) + ")");
      }
    }

    build_suppressor_chain(config, warnings);
    // A remembered id-match belongs to the allowlist that computed it; build_suppressor_chain
    // just rebuilt (or dropped) that allowlist above, so a value captured under the OLD
    // config would misrepresent the new one - clear rather than carry it forward.
    id_allowlisted_.clear();
    // Deliberately NOT reset here, unlike id_allowlisted_ above: ever_raised_ is a fact about
    // this PROCESS's history ("has this instance ever put a genuine FAILED on the wire"),
    // not about the current config. A live reconfigure (an operator editing the allowlist,
    // say) can run with a death still outstanding and absent; resetting the flag would make
    // the freshly-rebuilt tracker unable to ever re-create evidence for a node that is not
    // currently online and armed, so the standing fault would get neither a further FAILED
    // NOR a PASSED once the node genuinely returns - stuck for the rest of the process's
    // life. A real process restart needs no reset either: it gets a brand-new
    // NodeDeathDetector object, and ever_raised_'s own member initializer already starts
    // false - see tick()'s own comment for the guard this flag drives.
    collect_unknown_detector_keys(config, known_keys(), warnings);
    warnings_ = std::move(warnings);
    warnings_logged_ = false;

    // A death can only ever be reported once misses_ exceeds miss_grace, so prune_ticks
    // must be at least miss_grace + 1 - otherwise a durably-suppressed key could be
    // reclaimed the very tick it would first have been reported, and the report would be
    // silently lost rather than silently suppressed.
    const int prune_ticks = std::max(prune_grace, miss_grace + 1);
    tracker_ = NodeLivenessTracker(miss_grace, prune_ticks, tracked_node_cap);
    tracked_node_cap_.store(tracked_node_cap);
  }

  std::size_t tracked_count_for_test() const override {
    return tracked_count_.load();
  }

  /// status_json() runs on an HTTP handler thread concurrently with tick() on the plugin's
  /// tick thread (see the base class doc), and NodeLivenessTracker itself carries no
  /// synchronization of its own - reading tracker_ directly from here would race tick()'s
  /// writes to it. tick() publishes into these atomics once per sweep instead, so this
  /// method touches nothing tick() also touches.
  ///
  /// `tracking_saturated` is the one an operator acts on: it means the number of DEPARTED
  /// (not yet confirmed dead, or already collapsed) identities exceeds tracked_node_cap even
  /// after collapsing every confirmed death it safely could - so a newly confirmed death
  /// among them may not get an individual name, only a place in the collapsed count. An
  /// armed/present key is never refused tracking by this cap (see NodeLivenessTracker's own
  /// class doc), so this is never "a node is going unchecked." The fix is a
  /// require_active-style narrower allowlist entry set is not available here (this detector
  /// is zero-config by design), so it is either less simultaneous departure churn or a larger
  /// tracked_node_cap - both of which need tracked_count/tracked_node_cap alongside it to
  /// reason about.
  nlohmann::json status_json() const override {
    return {{"tracked_count", tracked_count_.load()},
            {"tracking_saturated", saturated_.load()},
            {"tracked_node_cap", tracked_node_cap_.load()}};
  }

  void tick(DetectorContext & ctx) override {
    if (!ctx.snapshot) {
      return;
    }
    log_warnings_once(ctx);

    std::set<std::string> present;
    std::set<std::string> armed;
    std::set<std::string> handed_back;  // provisionally admitted, and the ground has gone
    for (const auto & app : ctx.snapshot->apps) {
      // A peer-aggregated app carries no ROS binding of its own (PeerClient::parse_app
      // never reads x-medkit.ros2.node), so effective_fqn() is empty for every one of
      // them. Tracking them would collapse the whole peer fleet onto a single "" key: one
      // online peer app would mask every other peer app's departure.
      if (app.source.rfind("peer:", 0) == 0) {
        continue;
      }
      // Liveness is the bound node actually running, not mere presence in the snapshot -
      // see the class doc for why membership alone would make a manifest node immortal.
      if (!app.is_online) {
        continue;
      }
      // Keyed on the STABLE fqn, not App::id: id is recomputed every sweep and only gets a
      // namespace prefix once a bare-name collision currently exists anywhere in the
      // graph, so a live node's id can change out from under a key built from it.
      const std::string key = app.effective_fqn();
      if (key.empty() || is_ros2cli_node(key)) {
        continue;
      }
      present.insert(key);
      // Tracking follows OWNERSHIP rather than permission, and the two are not the same
      // question: the gate answers "may this entity raise" permissively for a managed node
      // whose lifecycle label has never been read. What the gate returns here is the GROUND,
      // and the ground decides whether admission is permanent. kEarned came from a
      // measurement (or from a node with no lifecycle to measure) and is latched exactly as
      // this detector has always latched an armed key. kProvisional came only from the
      // watcher giving up on asking, so it holds only while that stays true: if a label
      // arrives afterwards - the ~/transition_event subscription outlives the seed budget -
      // the node turns out to have been another detector's all along, and it is handed back
      // below rather than reported dead later. The two NEGATIVE grounds are just as different
      // from each other: kDisowned is that measurement arriving, and kUnclaimed is the absence
      // of any measurement at all, which is what a re-warming node answers - so only the first
      // withdraws a key. The gate is keyed by App::id.
      switch (presence_ownership(ctx.gate, app.id)) {
        case PresenceOwnership::kEarned:
          // kEarned has two grounds and only one of them is a measurement: "the label reads
          // active" is, "there is no managed record at all" is not. For a key this detector has
          // already handed back that difference decides everything, because a dying managed
          // node produces the second shape on its way out - its lifecycle services leave the
          // snapshot a sweep before the App does, the watcher drops the record, and a node with
          // no record reads exactly like a plain one. Re-admitting on that would let the act of
          // dying erase the measurement that disowned the node, and the death would be reported
          // by the detector the label had already disqualified. So a released key waits for the
          // graph to say ACTIVE again; the disappearance of what disowned it is not news that
          // the node became ours.
          if (released_.count(key) != 0 && !reads_active(ctx.gate, app.id)) {
            break;
          }
          released_.erase(key);
          earned_.insert(key);  // knowledge, once had, is never withdrawn
          armed.insert(key);
          break;
        case PresenceOwnership::kProvisional:
          // Same rule, same reason: the asking starting over on a re-bound entry is the absence
          // of knowledge, not the arrival of it, so it cannot readmit a key already handed back.
          if (released_.count(key) != 0) {
            break;
          }
          armed.insert(key);
          break;
        case PresenceOwnership::kDisowned:
          // The graph has measured this node as another detector's. If this detector never
          // earned it, hand it back while it is still alive. A key that WAS earned keeps its
          // admission - a node that ran and then deactivated is still a death when it stops
          // running.
          //
          // This runs inside the per-app loop, so the hand-back needs a tick on which the app
          // is BOTH still present and already disowned. A node that dies within one such tick
          // of its label arriving is reported by this detector after all - measured at 210 ms
          // between the label reaching the status route and the release, against a ~1 s entity
          // cache refresh. The window cannot be closed by deciding at REPORT time instead: by
          // then the key has departed, `earned_` has been pruned for it (that prune is what
          // bounds this map against identity churn), and the departed record carries only the
          // LAST label - which reads "inactive" both for a node that was earned and then
          // deactivated, which must be reported, and for one that was only ever provisional,
          // which must not. Telling those apart at report time needs per-key ownership history
          // that survives the departure, which is the unbounded state the prune exists to
          // prevent. The window is stated in the README rather than papered over.
          if (earned_.count(key) == 0) {
            handed_back.insert(key);
            released_.insert(key);
          }
          break;
        case PresenceOwnership::kUnclaimed:
          // Nothing is known about this node yet - it is warming up, or the watcher is still
          // asking. That is not a reason to admit it, and it is emphatically not a reason to
          // give up a key already admitted: a node that dies, is reported, comes back and dies
          // again inside its re-warm window reads exactly like this on the tick it returns, and
          // releasing it there means its second death is tracked by nothing and every tick of
          // the outage reports PASSED.
          break;
      }
      // Capture the allowlist's id-form verdict WHILE the entity is still present and
      // app.id is in hand - suppresses() alone can never do this for a dead key, since by
      // the time one is checked below the entity has already left ctx.snapshot entirely.
      // See allowlist_suppressor.hpp's class doc. Overwritten every tick rather than
      // latched, so a key whose id later stops colliding (and so stops matching) does not
      // stay exempt on a stale reading - App::id, unlike the fqn, is not stable.
      if (allowlist_ != nullptr) {
        if (allowlist_->allows(app.id)) {
          id_allowlisted_.insert(key);
        } else {
          id_allowlisted_.erase(key);
        }
      }
    }

    // Before update(), so a key handed back this tick cannot appear in this tick's report:
    // the whole point is that it is not this detector's node, and a death it was never
    // entitled to report must not be reported once on the way out either.
    for (const auto & key : handed_back) {
      tracker_.release(key);
    }
    // `earned_` is a fact about keys in the CURRENT graph, so it is pruned to them: a key
    // that departs and later returns is a new incarnation, and it re-earns (or does not) from
    // whatever the gate then says. Without the prune this would grow with identity churn for
    // the life of the process, which is the growth the tracker's own cap exists to bound.
    for (auto it = earned_.begin(); it != earned_.end();) {
      it = present.count(*it) == 0 ? earned_.erase(it) : std::next(it);
    }
    // Bounded the same way and for the same reason: a key that leaves the graph is a fact about
    // a node that is gone, and the incarnation that returns re-derives its own ground.
    for (auto it = released_.begin(); it != released_.end();) {
      it = present.count(*it) == 0 ? released_.erase(it) : std::next(it);
    }

    auto report = tracker_.update(present, armed);

    std::set<std::string> keys_before_suppression;
    for (const auto & [key, detail] : report.dead) {
      (void)detail;
      keys_before_suppression.insert(key);
    }
    // Suppress via the generic chain_ (each suppressor gets just the fqn key - covers
    // AllowlistSuppressor's own fqn/leaf forms and LifecycleShutdownSuppressor's fqn-keyed
    // departed-state lookup) OR via a remembered allowlist id-match captured above while the
    // key was still present. The id form bypasses chain_ dispatch entirely rather than
    // going through Suppressor::suppresses() - see allowlist_suppressor.hpp for why.
    for (auto it = report.dead.begin(); it != report.dead.end();) {
      bool suppressed = id_allowlisted_.count(it->first) > 0;
      if (!suppressed) {
        for (const Suppressor * s : chain_) {
          if (s != nullptr && s->suppresses(it->first, ctx)) {
            suppressed = true;
            break;
          }
        }
      }
      if (suppressed) {
        it = report.dead.erase(it);
      } else {
        ++it;
      }
    }

    // Only a DURABLE suppressor's veto may feed prune() (see suppressor.hpp). Re-checking
    // against durable_chain_ specifically - rather than simply "no longer in report.dead"
    // - is required because a non-durable suppressor could remove a key from this tick's
    // report even while chain_ also carries a durable one; durability is a property of
    // WHICH suppressor vetoed a key, not of whether the key survived the filter. The id-form
    // allowlist match is exactly as durable as its fqn/leaf forms (both read the same
    // AllowlistSuppressor::durable()==true), so it feeds prune() the same way.
    std::set<std::string> suppressed_for_prune;
    for (const auto & key : keys_before_suppression) {
      if (report.dead.count(key) > 0) {
        continue;  // survived the filter - nothing suppressed it this tick
      }
      if (id_allowlisted_.count(key) > 0) {
        suppressed_for_prune.insert(key);
        continue;
      }
      for (const Suppressor * s : durable_chain_) {
        if (s != nullptr && s->suppresses(key, ctx)) {
          suppressed_for_prune.insert(key);
          break;
        }
      }
    }
    tracker_.prune(suppressed_for_prune);
    // Bound id_allowlisted_ the same way tracker_ itself is bounded: drop any key the
    // tracker no longer knows about at all, so a key reclaimed by prune() above does not
    // leave a same-tick orphan entry that would otherwise sit here forever (this map is
    // never itself consulted for a key outside known_keys(), but an unbounded map under
    // identity churn is exactly the failure N11 exists to rule out for tracker_ itself).
    for (auto it = id_allowlisted_.begin(); it != id_allowlisted_.end();) {
      it = tracker_.known_keys().count(*it) > 0 ? std::next(it) : id_allowlisted_.erase(it);
    }
    // Published once per sweep, after prune() has settled this tick's count - see
    // status_json()'s own doc for why this is the only cross-thread-safe way to read it.
    tracked_count_.store(tracker_.tracked_count());
    saturated_.store(report.tracking_saturated);
    if (report.saturation_started && ctx.gateway_node) {
      // Once per EPISODE, not once per process: the latch re-arms when saturation ends (see
      // NodeLivenessTracker's own saturated_last_tick_), so a later, real saturation is not
      // silent because an earlier one already spent the warning. Mirrors
      // lifecycle_expectation_detector.cpp's identical saturation_started handling.
      RCLCPP_WARN(ctx.gateway_node->get_logger(),
                  "graph_watchdog node_death: %d departed identities exceed tracked_node_cap even after "
                  "collapsing every confirmed death - a newly confirmed death may not be named "
                  "individually until some of these clear or are collapsed in turn. Every armed/present "
                  "node is still being checked; raise 'tracked_node_cap' if this graph genuinely churns "
                  "this many simultaneous departures at once",
                  tracked_node_cap_.load());
    }

    // ctx.clear_fault(), unlike ctx.raise_fault(), carries no reliability-gate check of its
    // own, so a level-triggered "affected is empty" reaches the fault manager as a genuine
    // PASSED unconditionally the moment the fault client is ready - and this detector is
    // zero-config, so unlike lifecycle_expectation's require_active it has no fixed,
    // enumerable set of entries to ask "have I re-observed all of them yet" before trusting
    // its own silence. ever_raised_ stands in for that: this process may only clear
    // GRAPH_NODE_DISAPPEARED once it has ITSELF actually handed a FAILED request for it to
    // the fault client at least once - not merely decided one was warranted. See
    // DetectorContext::raise_fault's own doc for exactly what that return value does and
    // does not prove.
    //
    // Tracked-key COUNT is deliberately not the signal, though it may look like an equally
    // good proxy: any App elsewhere in the graph - the gateway's own node included - tends
    // to arm within a tick or two of startup, which would make tracker_.tracked_count() go
    // non-zero long before the SPECIFIC node a stored, outstanding fault names has been
    // re-observed at all. A guard keyed on that count would open on an unrelated node's
    // arming and clear a fault this process has not actually re-verified - exactly the
    // silent, ungated heal this guard exists to rule out. report.dead's own emptiness says
    // nothing that specific either (every dead key is merged into one aggregate), which is
    // why the guard is keyed on whether THIS instance has raised at all, not on what it
    // currently knows about.
    if (report.dead.empty() && !ever_raised_) {
      return;
    }
    // emit_ordered()'s own return is what earns ever_raised_, not report.dead's mere
    // non-emptiness: raise_fault() can still decline to send even with a non-empty report -
    // Advisory/Off mode, no client wired yet, or (concretely) the fault_manager service not
    // being ready right at this tick, none of which are visible from here without reading
    // the return value. Setting the flag from intent rather than delivery is the exact
    // shape of false-clear this guard exists to rule out, one level deeper: kill a node
    // while the service is down, wait past miss_grace, restore the service, let the node
    // return - a flag set from "the report was non-empty" would already be true by then
    // though async_send_request() had never actually been called for it, and the empty
    // report on return would then emit a PASSED for an occurrence never even attempted.
    // Retried
    // every tick the report stays non-empty (emit_ordered() runs unconditionally above the
    // guard once report.dead is non-empty, whether or not ever_raised_ is already true), so
    // a raise that fails here is simply attempted again next tick rather than lost.
    const bool sent = aggregated_.emit_ordered(ctx, report.dead, report.keys_by_freshness);
    if (!report.dead.empty() && sent) {
      ever_raised_ = true;
    }
  }

 private:
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"allowlist", "miss_grace",       "prune_grace",
                                            "suppress",  "tick_interval_ms", "tracked_node_cap"};
    return keys;
  }

  void log_warnings_once(const DetectorContext & ctx) {
    if (warnings_.empty() || warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    for (const auto & warning : warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "graph_watchdog node_death: %s", warning.c_str());
    }
    warnings_logged_ = true;
  }

  /// Rebuild owned_suppressors_/chain_/durable_chain_ from this detector's own config
  /// slice. `suppress` is a list of mechanism names, dispatched here:
  ///   "allowlist"  - an AllowlistSuppressor over the configured `allowlist` set.
  ///   "lifecycle"  - a (stateless) LifecycleShutdownSuppressor.
  ///   anything else - named but unrecognised; warned about, not built.
  ///
  /// Both mechanisms are opt-in: a configured `allowlist` that `suppress` never names has
  /// NO effect, and says so - naming an entry on the list is not, by itself, a request to
  /// suppress it. Every field is re-validated from scratch on every call (type, then array
  /// element shape), and every rejection is named rather than silently dropped, so a typo'd
  /// key or a malformed entry never reads as "working" from the operator's side.
  ///
  /// Rebuilds all three containers unconditionally, so a re-configure() never leaves
  /// chain_/durable_chain_ holding a pointer into a freed owned_suppressors_ entry.
  void build_suppressor_chain(const nlohmann::json & config, std::vector<std::string> & warnings) {
    std::set<std::string> allow_set;
    if (config.contains("allowlist")) {
      const auto & value = config["allowlist"];
      if (value.is_array()) {
        for (const auto & entry : value) {
          if (entry.is_string() && !entry.get<std::string>().empty()) {
            allow_set.insert(entry.get<std::string>());
          } else {
            warnings.push_back("'allowlist' entries must be non-empty strings; skipping " + entry.dump());
          }
        }
      } else {
        warnings.push_back("'allowlist' must be an array of strings; ignoring it");
      }
    }

    owned_suppressors_.clear();
    chain_.clear();
    durable_chain_.clear();
    allowlist_ = nullptr;  // owned_suppressors_.clear() just freed whatever this pointed to

    bool allowlist_named = false;
    if (config.contains("suppress")) {
      const auto & value = config["suppress"];
      if (value.is_array()) {
        for (const auto & entry : value) {
          if (!entry.is_string()) {
            warnings.push_back("'suppress' entries must be strings; skipping " + entry.dump());
            continue;
          }
          const std::string name = entry.get<std::string>();
          if (name == "allowlist") {
            allowlist_named = true;
          } else if (name == "lifecycle") {
            owned_suppressors_.push_back(std::make_unique<LifecycleShutdownSuppressor>());
          } else {
            warnings.push_back("unknown 'suppress' entry '" + name + "'; ignoring it");
          }
        }
      } else {
        warnings.push_back("'suppress' must be an array of strings; ignoring it");
      }
    }

    if (allowlist_named) {
      auto suppressor = std::make_unique<AllowlistSuppressor>(std::move(allow_set));
      // Kept as a second, typed pointer alongside the polymorphic chain_ entry below -
      // node_death_detector.cpp's own tick() needs to call allows() directly (an
      // AllowlistSuppressor-specific method, not part of the generic Suppressor interface)
      // while an entity is still present. See tick()'s own comment and
      // allowlist_suppressor.hpp's class doc for why.
      allowlist_ = suppressor.get();
      owned_suppressors_.push_back(std::move(suppressor));
    } else if (!allow_set.empty()) {
      // A configured allowlist that suppress does not name has no effect - naming it in
      // suppress is what opts it in, matching every other suppression mechanism this
      // framework carries.
      warnings.push_back("'allowlist' has " + std::to_string(allow_set.size()) + " entr" +
                         (allow_set.size() == 1 ? std::string("y") : std::string("ies")) +
                         " configured but 'suppress' does not name \"allowlist\"; the list is inert");
    }

    for (const auto & s : owned_suppressors_) {
      chain_.push_back(s.get());
      if (s->durable()) {
        durable_chain_.push_back(s.get());
      }
    }
  }

  NodeLivenessTracker tracker_{kDefaultMissGrace};
  std::vector<std::string> warnings_;
  bool warnings_logged_ = false;
  AggregatedFault aggregated_{graph_fault_codes::kNodeDisappeared, kNodeDeathSeverity};
  std::vector<std::unique_ptr<Suppressor>> owned_suppressors_;
  std::vector<const Suppressor *> chain_;
  std::vector<const Suppressor *> durable_chain_;  ///< Subset of chain_ safe to feed prune(). See tick().
  /// Non-owning; points into owned_suppressors_ when "allowlist" is named in suppress, null
  /// otherwise. Typed (not just another chain_ entry) because tick() needs allows(), which
  /// is not part of the generic Suppressor interface - see allowlist_suppressor.hpp.
  AllowlistSuppressor * allowlist_ = nullptr;
  /// fqn -> "the last App::id observed for this key, while present, matched the allowlist".
  /// Refreshed every tick a key is present (see tick()); the value from its last live tick
  /// is what a dead key is judged by, since id is unavailable once the entity has left
  /// ctx.snapshot. Bounded to tracker_.known_keys() at the end of every tick.
  /// Keys this detector handed back on a measured disown and has not been given back by a
  /// measurement since. Read only by the two admission branches in tick(), where it is what
  /// stops the loss of a lifecycle record - which is what dying looks like from the snapshot -
  /// counting as the node becoming this detector's. Cleared by a label reading "active", and
  /// pruned to the live graph every tick.
  std::set<std::string> released_;
  /// Keys whose ownership the gate granted on kEarned grounds while they were present. Read
  /// only by the kDisowned branch in tick(), which is the one place the difference between an
  /// admission earned from a measurement and one granted provisionally decides anything.
  /// Pruned to the live graph every tick.
  std::set<std::string> earned_;
  std::set<std::string> id_allowlisted_;
  /// Whether THIS detector instance has itself genuinely raised GRAPH_NODE_DISAPPEARED at
  /// least once - the ungated-clear guard. See tick()'s own comment for why this, and not
  /// tracker_.tracked_count(), is the right granularity.
  bool ever_raised_ = false;
  std::atomic<std::size_t> tracked_count_{0};  ///< Cross-thread-safe mirror of tracker_.tracked_count().
  std::atomic<bool> saturated_{false};         ///< tracked_node_cap refused a key on the last tick
  std::atomic<int> tracked_node_cap_{NodeLivenessTracker::kDefaultTrackedKeyCap};  ///< as of the last configure()
};

REGISTER_DETECTOR(NodeDeathDetector, "node_death")

}  // namespace ros2_medkit_graph_watchdog
