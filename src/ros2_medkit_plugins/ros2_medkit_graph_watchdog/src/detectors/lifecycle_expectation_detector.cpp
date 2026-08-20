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
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// Three independent fault codes, each fixed at its own severity - mirroring orphan_detector
// and param_drift_detector's fixed-severity AggregatedFault. GRAPH_NODE_INACTIVE (a
// required-active node CONFIRMED stuck inactive) means the robot silently will not act to
// the outside world - as severe as node death. GRAPH_NODE_UNREADABLE and
// GRAPH_NODE_NOT_MANAGED (a required node whose lifecycle promise has never been measured,
// for one of two reasons) are lesser, independent claims: nothing has been measured wrong
// about either, the promise is merely UNVERIFIED - and like every non-CRITICAL severity
// here they settle through the ordinary debounce path rather than bypassing it (see
// ros2_medkit_msgs/msg/Fault.msg's CRITICAL contract).
constexpr std::uint8_t kConfirmedInactiveSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
constexpr std::uint8_t kUnmeasuredSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
// Consecutive not-active ticks a required node tolerates before being reported inactive.
// Config-overridable via "grace" - gives a managed node bringup time to reach "active"
// (configure + activate) before the operator's expectation is enforced.
constexpr int kDefaultGrace = 5;
// Widest `grace` accepted, in ticks (five minutes at the shipped 1 s cadence). Range-checked
// on the WIDE integer before narrowing for the same reason prune_grace is (see
// kMaxPruneGrace): get<int>() truncates first, so 4294967296 would arrive as 0 and pass the
// >= 0 check - turning the documented "non-negative integer" contract into a hair-trigger
// that reports a node on its very first not-active tick, with no warning and no default.
//
// The upper end used to be INT_MAX - 1, which is not a wide tolerance but an off switch with
// no warning attached. `grace` bounds how long a node that STAYS IN THE GRAPH may read
// not-active before being reported, and how long a node that returns from a departure still
// inactive takes to (re-)mature - both advance on present ticks, which this value directly
// caps. At INT_MAX - 1 that is roughly 24 days at the shipped cadence either way; five minutes
// is already an extravagant allowance for a managed node to reach `active`, and it makes the
// worst-case PRESENT withhold something an operator can reason about. A deployment that
// genuinely needs longer wants a slower tick, not a detector that is silent for weeks.
//
// Whether it also bounds a node that leaves the graph before its streak matures depends on
// whether that node was ever ARMED (see LifecycleExpectationTracker's own class doc). For one
// the reliability gate has armed at least once, `grace` does NOT bound it: absence holds a
// below-grace streak rather than advancing it, so that entry sits in the tracker's pending
// set for as long as the node stays gone, however long that is - GRAPH_NODE_DISAPPEARED can
// report that departure instead, so nothing here needs to hurry it. That is not a new way to
// withhold GRAPH_NODE_INACTIVE's clear for every OTHER node - the clear was already gated on
// every required node's status being settled, so one node this indecisive already blocked it
// before this bound existed. For a node that was NEVER armed - node_death only ever tracks a
// node the gate has armed, so this is the one departure nothing else can report - `grace`
// DOES still bound it: absence keeps advancing the streak too, so it matures within `grace` +
// `absence_grace` + 1 ticks either way.
constexpr std::int64_t kMaxGrace = 300;
/// Consecutive ticks an entry must match ONLY unmanaged nodes before the typo warning
/// fires. One transient tick is not evidence: discover_apps() wraps the per-node service
/// enumeration in a try/catch and pushes the app regardless, so a sweep that races service
/// discovery yields an app with no services and no tracked lifecycle state.
constexpr int kUnmanagedWarnTicks = 5;
// Fallback plugin-scope prune_grace (ticks) when the config doesn't carry one - mirrors
// GraphWatchdogPlugin's own default.
constexpr int kDefaultPruneGrace = 60;
// Widest prune_grace accepted, in ticks (one hour at the shipped 1 s cadence). The value is
// range-checked on the WIDE integer before narrowing: get<int>() truncates first, so
// 4294967296 would arrive as 0 and reclaim bookkeeping on the first absent tick, and 2^63-1
// would arrive as -1 and be dropped in silence.
constexpr std::int64_t kMaxPruneGrace = 3600;
// Widest tracked_node_cap accepted. 32x the shipped default of 512: one tracked node really
// costs on the order of 500 bytes (the NodeState itself, its map node, the fqn key and the
// "required by" entry strings), so a map full at this cap is roughly 8 MB - the most this
// detector may spend on bookkeeping inside a gateway that also holds the entity cache and the
// HTTP server, on hardware that is often an embedded board. A deployment with more than
// sixteen thousand DISTINCT required-node identities alive at once is not a large fleet, it is
// identity churn, and churn is what the cap exists to bound rather than to accommodate. The
// range check runs on the WIDE integer before narrowing, for the same reason grace and
// prune_grace do. Zero is refused rather than clamped: a cap of nothing means the detector
// checks nothing, which is exactly the silence it exists to prevent.
constexpr std::int64_t kMinTrackedNodeCap = 1;
constexpr std::int64_t kMaxTrackedNodeCap = 16384;
/// Consecutive withheld ticks before the reason is put in the log. Withholding is right -
/// health that was never measured is not health - but from the outside it is
/// indistinguishable from a detector that is working and finding nothing, so once per
/// episode the detector says which it is. Ten ticks: label seeding normally completes
/// within a tick or two of the watcher seeing the node, so a hold that lives this long is
/// worth explaining.
constexpr int kWithheldClearReportTicks = 10;
// Ties kMaxLifecycleDetailChars (lifecycle_expectation_tracker.hpp) to the real
// AggregatedFault::kMaxDescriptionChars it is budgeted against - the tracker header does
// not include aggregated_fault.hpp (kept "Pure.", see its class doc), so this is the one
// place that can verify the arithmetic in the tracker's own comment against the actual
// cap: at least 3 maximally-long details must still fit under it.
static_assert(3 * kMaxLifecycleDetailChars + 2 * 2 <= AggregatedFault::kMaxDescriptionChars,
              "kMaxLifecycleDetailChars must leave room for at least 3 worst-case details under "
              "AggregatedFault::kMaxDescriptionChars");
}  // namespace

/// Watches operator-declared "must be active" nodes and raises GRAPH_NODE_INACTIVE for
/// one that is present in the graph (alive) but sitting in a non-active lifecycle state
/// for more than `grace` consecutive ticks, GRAPH_NODE_UNREADABLE for one whose lifecycle
/// label this run has never been able to read, and GRAPH_NODE_NOT_MANAGED for one that
/// carries no tracked lifecycle at all. The three are independent, level-triggered
/// aggregates: each one's content comes from its own measurement, and one raising or
/// healing never forces or blocks another. GRAPH_NODE_INACTIVE's own clear is not simply
/// "nothing CONFIRMED inactive this tick", though: it is withheld while any required
/// node's status is UNSETTLED (see the withheld-clear guard in tick()) - the original
/// withheld-clear guarantee this detector always gave, now scoped to GRAPH_NODE_INACTIVE
/// alone. GRAPH_NODE_UNREADABLE and GRAPH_NODE_NOT_MANAGED have no such guard: each one's
/// own clear needs nothing beyond its own content going empty. A node can only ever be
/// content of one of the three at a time - see LifecycleExpectationTracker's own class doc
/// for the state machine (one observed state per node per tick, two clocks) that makes
/// that guarantee structural rather than something this detector has to re-derive.
/// Distinct from the presence class (GRAPH_NODE_DISAPPEARED, pure presence): here the
/// process is ALIVE and in the graph, but a node the operator declared critical-active is
/// either stuck inactive/unconfigured/finalized (GRAPH_NODE_INACTIVE), or its lifecycle
/// promise is simply UNVERIFIED (GRAPH_NODE_UNREADABLE / GRAPH_NODE_NOT_MANAGED) - on a
/// Nav2 stack a controller_server stuck inactive means the robot silently will not act, no
/// crash, no log. Safe default: require_active is empty, so nothing is checked and there
/// are zero false positives until an operator opts specific nodes in (mirrors param_drift
/// being config-scoped). See lifecycle_expectation_tracker.hpp for the pure tracking core
/// and the design doc.
///
/// Design note (why grace, not the reliability gate): this detector cannot gate its
/// raise on reliability_allows() - it ANDs lifecycle_.node_ok(), which is FALSE for
/// exactly the inactive node this detector exists to catch, so gating on it would
/// suppress the signal forever. Instead the tracker counts consecutive not-active ticks
/// itself and raises only past its own grace, independent of the central gate (which
/// still applies to the aggregated RAISE itself via ctx.raise_fault - see
/// DetectorContext - for warmup/bringup-quiesce, just not for the lifecycle-inactive
/// state this detector is specifically built to report).
class LifecycleExpectationDetector : public Detector {
 public:
  std::string id() const override {
    return "lifecycle_expectation";
  }

  void configure(const nlohmann::json & config) override {
    std::vector<std::string> warnings;
    std::set<std::string> require_active;
    if (config.contains("require_active")) {
      if (config["require_active"].is_array()) {
        for (const auto & entry : config["require_active"]) {
          if (entry.is_string() && !entry.get<std::string>().empty()) {
            require_active.insert(entry.get<std::string>());
          } else {
            // Skip "" - it would match every unbound app's empty leaf. Silently dropping the
            // entry leaves the operator believing the node is covered, so it has to say so.
            warnings.push_back("'require_active' entries must be non-empty node names; skipping one");
          }
        }
      } else {
        warnings.push_back("'require_active' must be a string array of node names; ignoring it");
      }
    }
    int grace = kDefaultGrace;
    if (config.contains("grace")) {
      const auto & value = config["grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxGrace) {
        grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'grace' must be an integer in 0.." + std::to_string(kMaxGrace) + "; keeping the default (" +
                           std::to_string(kDefaultGrace) + ")");
      }
    }
    // prune_grace is injected into every detector's config by the plugin and is exempt from the
    // unknown-key warning, so ignoring a bad value here would accept the key with no warning and
    // no effect - the exact silent failure the README says cannot happen. Range-check on the
    // WIDE value before narrowing (see kMaxPruneGrace).
    int prune_grace = kDefaultPruneGrace;
    if (config.contains("prune_grace")) {
      const auto & value = config["prune_grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxPruneGrace) {
        prune_grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'prune_grace' must be an integer in 0.." + std::to_string(kMaxPruneGrace) + "; keeping " +
                           std::to_string(kDefaultPruneGrace));
      }
    }
    // The hard bound on how many nodes the tracker keeps state for. Configurable for the same
    // reason every other bound in this detector is: the README already presents 512 as a number
    // an operator reasons about, and a deployment whose require_active entries legitimately
    // match more than 512 PRESENT nodes at once otherwise has no lever at all - the detector
    // simply stops checking the excess and says so once. Same wide-integer range check as
    // grace and prune_grace above.
    int tracked_node_cap = kDefaultTrackedNodeCap;
    if (config.contains("tracked_node_cap")) {
      const auto & value = config["tracked_node_cap"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= kMinTrackedNodeCap && wide <= kMaxTrackedNodeCap) {
        tracked_node_cap = static_cast<int>(wide);
      } else {
        warnings.push_back("'tracked_node_cap' must be an integer in " + std::to_string(kMinTrackedNodeCap) + ".." +
                           std::to_string(kMaxTrackedNodeCap) + "; keeping the default (" +
                           std::to_string(kDefaultTrackedNodeCap) + ")");
      }
    }
    collect_unknown_detector_keys(config, known_keys(), warnings);
    require_active_ = require_active;
    // prune_grace goes through unclamped: it is the age horizon for IDLE bookkeeping ONLY
    // (both clocks at zero, no matured ownership, so nothing to lose). A node carrying
    // evidence is never reclaimed by age at all, so there is no longer anything for a
    // grace-derived floor to protect - what bounds the map when nothing is idle is
    // kDefaultTrackedNodeCap. See LifecycleExpectationTracker's "Bounded by evidence, not
    // by age".
    tracker_ = LifecycleExpectationTracker(require_active_, grace, kDefaultAbsenceGrace, kDefaultNoMatchWarnTicks,
                                           prune_grace, kDefaultUnmeasuredHoldTicks, tracked_node_cap);
    tracked_node_cap_ = tracked_node_cap;
    unmanaged_streak_.clear();
    // The one-time warning is scoped to the CURRENT config. Without this, an entry that
    // already warned stays permanently silenced across a reconfigure - including after it
    // was removed and re-added, which is exactly when the operator wants to hear that it
    // still names nothing managed.
    warned_.clear();
    warnings_ = std::move(warnings);
    warnings_logged_ = false;
    // The withheld-clear guard restarts with the config: a reconfigure rebuilds the
    // tracker above (its own map is the ENTIRE per-node state machine now), so the guard
    // has nothing left to carry over even without a separate clear() of its own.
    ever_matched_.clear();
    unmatched_ticks_ = 0;
    withheld_ticks_ = 0;
    withheld_reported_ = false;
  }

  /// Delegates straight to the tracker: every per-node fact (both clocks, absence,
  /// entries, cause) lives in its one map now, so there is no second bookkeeping
  /// structure in this class to reconcile against.
  std::size_t tracked_count_for_test() const override {
    return tracker_.tracked_count();
  }

  /// Surfaced under `detectors.lifecycle_expectation` on GET /x-medkit-watchdog. Built from
  /// atomics only: this runs on an HTTP handler thread while tick() writes them on the
  /// plugin's tick thread, and the handler holds no lock this detector takes.
  ///
  /// `tracking_saturated` is the one an operator acts on. It means the detector has REFUSED
  /// to track a required node because `tracked_node_cap` is full of present nodes that are
  /// all carrying evidence - so that node is going unchecked, and GRAPH_NODE_INACTIVE's
  /// clear is withheld for as long as it lasts. The fix is either a `require_active` entry
  /// that matches fewer identities (a bare name on a graph whose nodes respawn under
  /// ever-new namespaces matches an unbounded set) or a larger `tracked_node_cap`, which is
  /// why the cap and the current count are reported beside it.
  nlohmann::json status_json() const override {
    return nlohmann::json{{"tracking_saturated", saturated_.load()},
                          {"tracked_nodes", tracked_nodes_.load()},
                          {"tracked_node_cap", tracked_node_cap_.load()}};
  }

  void tick(DetectorContext & ctx) override {
    // Before the zero-config early return, deliberately: the misspelt key most worth
    // reporting is `require_activ`, and that typo is exactly the config in which
    // require_active_ is empty.
    log_warnings_once(ctx);
    if (require_active_.empty()) {
      // require_active_ is empty - a config that never set it, or a reconfigure that
      // dropped it - so there is nothing to check and nothing below this line ever runs
      // while it stays empty (preserves EmptyConfigNeverContactsTheFaultManager /
      // ExplicitEmptyRequireActiveNeverContactsTheFaultManager). A record this detector
      // already raised under an OLDER, non-empty config is left exactly where it is:
      // GraphWatchdogPlugin::set_context() creates and configures every detector exactly
      // once, so no caller can reach this branch after a prior non-empty configure() in
      // the same process, and a stale record left behind when an operator removes a
      // detector's configuration is a question about who clears a fault whose detector no
      // longer exists - true of every detector in this package, and not solved here.
      return;
    }
    if (!ctx.snapshot) {
      return;  // zero-config safe default handled above; here there is just nothing to check yet
    }

    // Match each configured require_active entry against a live app by App::id OR its stable
    // effective_fqn() OR the bare leaf name of that fqn. App::id alone is unstable: it is
    // recomputed each sweep and gets a namespace prefix once a same-bare-name collision exists
    // anywhere in the graph, so a bare-name config would silently stop matching on a
    // multi-robot graph - the worst failure mode for a silent-fault detector. A bare name
    // therefore matches every namespace's node of that name; use a full FQN to pin one.
    std::vector<LifecycleMatch> matches;
    std::set<std::string> entry_has_managed_match;  // entries matching >=1 app with a tracked lifecycle state
    for (const auto & app : ctx.snapshot->apps) {
      const std::string fqn = app.effective_fqn();
      if (fqn.empty()) {
        continue;  // an unnamed entry can never be useful in a fault
      }
      const std::string leaf = fqn.substr(fqn.find_last_of('/') + 1);
      for (const auto & id : require_active_) {
        if (id != app.id && id != fqn && id != leaf) {
          continue;
        }
        const std::optional<std::string> state = ctx.gate ? ctx.gate->lifecycle_state_of(app.id) : std::nullopt;
        if (state.has_value()) {
          entry_has_managed_match.insert(id);
        }
        // Whether the gate says the presence detector OWNS this node's departure, for this
        // SAME app.id. It is what node_death's tracking decision consults, not the whole of
        // it: that detector applies its own filters first (peer-aggregated apps, apps that
        // are not online, an empty fqn, ros2cli helper names - see node_death_detector.cpp),
        // so a true answer here means the GATE would let it track this node, not that it
        // will. Reading it from the one gate both detectors share on the same tick is what
        // makes "armed" trustworthy rather than a guess: it is not derived from this node's
        // own lifecycle label, it IS the fact the presence detector would itself consult if
        // asked right now. The tracker needs it to tell a node the presence detector could
        // someday report from one it structurally never can - see LifecycleExpectationTracker's
        // own class doc. Deliberately NOT reliability_allows(): that one is permissive about a
        // managed node whose lifecycle label has never been read, so taking its permission for
        // ownership would switch this detector's absence path off for a node the presence class
        // does not actually hold - either because the watcher has not finished asking about it,
        // or because it asked, gave up, and then a label arrived and took the node back.
        //
        // kEarned only, and the two exclusions are for different reasons.
        //
        // A PROVISIONAL grant is deliberately not enough: the tracker latches this value for
        // the node's whole life, and a provisional grant is the one node_death itself gives up
        // the moment a real label arrives. Latching it here would leave this detector believing
        // the presence class had a node it had already handed back - the same silence in the
        // opposite direction.
        //
        // ANDed with is_online because node_death skips an app that is not online before it
        // ever consults the gate, and the gate has no notion of online-ness: a manifest app
        // whose node never started is armed, carries no lifecycle record, and would otherwise
        // read as owned by a detector that will never look at it. The remaining filters
        // node_death applies need no mirror here: a peer-aggregated app and one with no binding
        // both have an empty fqn and are already skipped above.
        const bool armed = app.is_online && presence_ownership(ctx.gate, app.id) == PresenceOwnership::kEarned;
        // One match per (entry, node): the tracker keys violations by NODE, so two
        // namesakes are both reported instead of one silently replacing the other.
        matches.push_back(LifecycleMatch{id, fqn, state, armed});
      }
    }
    std::set<std::string> matched_entries;
    for (const auto & match : matches) {
      matched_entries.insert(match.entry);
    }
    // Third leg of the withheld-clear guard: an entry that has NEVER matched a node since
    // configure(). The tracker's own pending/affected/*_affected maps are all keyed by a
    // matched node, so in the window before the first match they are empty, the tracker
    // reports nothing settled either way, and the clear flows about a node the detector has
    // not once looked at. That window is not exotic - it is every restart: the plugin ticks
    // as soon as it is loaded, while the entity snapshot is still catching up with the
    // graph. Deliberately NOT the same as an entry whose node VANISHES after having
    // matched: that is a presence problem and the clear is correct there (the tracker makes
    // the same handoff). Bounded by the same hold as the unmeasured legs, so a misspelt
    // entry cannot block healing for the process lifetime.
    ever_matched_.insert(matched_entries.begin(), matched_entries.end());
    const bool any_never_matched = ever_matched_.size() < require_active_.size();
    if (!any_never_matched) {
      unmatched_ticks_ = 0;
    } else if (unmatched_ticks_ <= kDefaultUnmeasuredHoldTicks) {
      ++unmatched_ticks_;  // frozen one past the hold, like the tracker's own clocks
    }
    const bool unmatched_blocking =
        any_never_matched && unmatched_ticks_ >= 1 && unmatched_ticks_ <= kDefaultUnmeasuredHoldTicks;

    // An entry that matches present nodes but NONE with a tracked lifecycle state is
    // probably a typo (or a plain, non-managed node). Requiring N CONSECUTIVE such ticks
    // before saying so is what keeps that from latching on a transient: LifecycleWatcher
    // only tracks a node whose get_state service was seen on THIS sweep, and
    // discover_apps() can yield an app with empty services when a sweep races service
    // enumeration - one such tick used to pin the accusation for the process lifetime,
    // since warned_ is only cleared by configure() and configure() runs once.
    //
    // Requires ctx.gate: with no gate wired, lifecycle_state_of() is never consulted and
    // EVERY entry looks unmanaged, so this would accuse every correct entry of being a typo.
    if (ctx.gateway_node && ctx.gate) {
      for (const auto & id : matched_entries) {
        if (entry_has_managed_match.count(id) != 0) {
          unmanaged_streak_.erase(id);  // resolved: it may warn again if it regresses
          warned_.erase(id);
          continue;
        }
        if (++unmanaged_streak_[id] > kUnmanagedWarnTicks && warned_.insert(id).second) {
          RCLCPP_WARN(ctx.gateway_node->get_logger(),
                      "graph_watchdog lifecycle_expectation: require_active entry '%s' has matched a present node "
                      "with no tracked lifecycle state for %d consecutive ticks (not a managed rclcpp_lifecycle "
                      "node?) - check for a typo",
                      id.c_str(), kUnmanagedWarnTicks + 1);
        }
      }
    }

    auto report = tracker_.update(matches);
    // Published for GET /x-medkit-watchdog. Written here on the plugin's tick thread and
    // read by status_json() on an HTTP handler thread, which holds no lock this detector
    // takes - hence atomics rather than plain members.
    saturated_.store(report.tracking_saturated);
    tracked_nodes_.store(tracker_.tracked_count());
    // An entry that matches nothing at all is reported by the tracker, not here: the loop
    // above only sees entries that DID match a present node.
    if (ctx.gateway_node) {
      if (report.saturation_started) {
        // Once per EPISODE, not once per process: the latch re-arms when saturation ends, so
        // a later, real saturation is not silent because an earlier one spent the one
        // warning. Refusing the newcomer is the safe direction (no live violation is evicted
        // to make room), but a required node is going unchecked, and that must never be
        // silent.
        RCLCPP_WARN(ctx.gateway_node->get_logger(),
                    "graph_watchdog lifecycle_expectation: already tracking %d required nodes - the most this "
                    "detector keeps state for - and every one of them is carrying evidence, so a newly matched "
                    "node is NOT being checked. A require_active entry is matching an unbounded set of node "
                    "identities (a bare name on a graph whose nodes respawn under ever-new namespaces?); pin "
                    "the nodes you mean with full FQNs, or raise 'tracked_node_cap'",
                    tracked_node_cap_.load());
      }
      for (const auto & entry : report.entries_matching_nothing) {
        // The tracker counts CONSECUTIVE no-match ticks, so it also surfaces an entry whose
        // node matched and later left the graph. The warning below says the opposite in two
        // places ("no node at all since startup", and that the presence class cannot report
        // it), and for a node that WAS present both statements are false - a departed node
        // is precisely what GRAPH_NODE_DISAPPEARED owns. Same handoff the never-matched
        // hold makes above: an entry that has matched once is never treated as missing
        // again.
        if (ever_matched_.count(entry) != 0) {
          continue;
        }
        RCLCPP_WARN(ctx.gateway_node->get_logger(),
                    "graph_watchdog lifecycle_expectation: require_active entry '%s' has matched no node at all "
                    "since startup - a misspelt entry, or a required node that never came up (the presence class "
                    "GRAPH_NODE_DISAPPEARED cannot report it either: it only tracks nodes that were present at "
                    "least once)",
                    entry.c_str());
      }
    }

    // GRAPH_NODE_UNREADABLE and GRAPH_NODE_NOT_MANAGED: independent of GRAPH_NODE_INACTIVE
    // and never withheld by its guard - a node whose unmeasured clock matured is a settled
    // fact for as long as it stays matured, not something to wait on further. Both fixed
    // SEVERITY_WARN. Level-triggered every tick, same as every other aggregate here:
    // non-empty content raises (or re-raises), empty content clears.
    aggregated_unreadable_.emit_ordered(ctx, report.unreadable_affected, report.newly_unreadable);
    aggregated_not_managed_.emit_ordered(ctx, report.not_managed_affected, report.newly_not_managed);

    // Withheld-clear guard - for GRAPH_NODE_INACTIVE ONLY. The emitter is level-triggered:
    // an empty affected map is a clear, and a clear asserts that every required node is
    // healthy. Two things can make `report.affected` empty without that being true: an
    // entry that has never matched anything yet (`unmatched_blocking`, entry-keyed - see
    // above), and any node whose status the tracker itself reports UNSETTLED this tick
    // (`report.pending`, node-keyed - see LifecycleExpectationTracker's own class doc for
    // exactly what feeds it: a violation streak that has not yet passed grace, an
    // unmeasured clock still climbing, or a streak held while the node is inside an
    // unmeasured spell). Absence is NOT one of them any more: content follows the clocks
    // rather than the snapshot, so a node already past grace stays in `affected` while it
    // blinks instead of emptying the map. A node whose
    // unmeasured clock has MATURED is neither: ownership passed to its own fault code and
    // its violation streak was released, so it is not pending either - which is exactly
    // what lets GRAPH_NODE_INACTIVE's clear flow the moment a node's "cannot measure this"
    // status is fully resolved into content elsewhere, rather than continuing to poison
    // this fault's withhold decision the way a shared record used to.
    //
    // Saturation is the third reason, and unlike the other two it is not a transient: once a
    // departed entry can no longer crowd out a present one, a refusal means there are
    // genuinely more required PRESENT nodes than `tracked_node_cap` allows. That is a
    // capacity condition an operator resolves by raising the key, so it gets no bounded hold
    // of its own - unlike `unmatched_blocking`, which is bounded precisely because a misspelt
    // entry must not block healing forever. A detector that declined to check a required
    // node cannot assert that every required node is healthy, for as long as it keeps
    // declining.
    if (report.affected.empty() && (!report.pending.empty() || unmatched_blocking || report.tracking_saturated)) {
      report_withheld_clear(ctx, report.pending_violation, report.pending_unreadable, report.pending_not_managed,
                            unmatched_blocking, report.tracking_saturated);
      return;  // GRAPH_NODE_INACTIVE: nothing measured - and a young streak - is not the same as healthy
    }
    withheld_ticks_ = 0;
    withheld_reported_ = false;
    // GRAPH_NODE_INACTIVE: fixed SEVERITY_ERROR, like orphan_detector and
    // param_drift_detector's AggregatedFault members - `report.affected` can no longer be
    // anything but a CONFIRMED violation, so there is nothing left for a per-tick severity
    // choice to decide.
    aggregated_inactive_.emit_ordered(ctx, report.affected, report.newly_affected);
  }

 private:
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"require_active", "grace", "tracked_node_cap"};
    return keys;
  }

  void log_warnings_once(const DetectorContext & ctx) {
    if (warnings_.empty() || warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    for (const auto & warning : warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "graph_watchdog lifecycle_expectation: %s", warning.c_str());
    }
    warnings_logged_ = true;
  }

  /// Say, once per withhold episode, that the clear is being withheld and why.
  /// Withholding is correct - health that was never measured is not health, and neither is
  /// a violation the counter has not finished counting - but from the outside it is
  /// indistinguishable from a detector that is working and finding nothing: no raise, no
  /// clear, and a GRAPH_NODE_INACTIVE already in the store simply never heals. Once per
  /// episode, past a horizon the normal label seeding and a normal grace both fit inside.
  /// Every reason is named separately because each releases on a different condition -
  /// `not_managed` and `unreadable` each release into their OWN fault code now, while
  /// `violation` releases either into content (GRAPH_NODE_INACTIVE itself) or into health,
  /// so the three cannot share one sentence.
  void report_withheld_clear(const DetectorContext & ctx, const std::set<std::string> & pending_violation,
                             const std::set<std::string> & pending_unreadable,
                             const std::set<std::string> & pending_not_managed, bool unmatched, bool saturated) {
    ++withheld_ticks_;
    if (withheld_ticks_ <= kWithheldClearReportTicks || withheld_reported_ || !ctx.gateway_node) {
      return;
    }
    withheld_reported_ = true;
    std::string reason;
    const auto add = [&reason](const std::string & phrase) {
      if (!reason.empty()) {
        reason += " and ";
      }
      reason += phrase;
    };
    if (saturated) {
      add("a required node could not be tracked at all because 'tracked_node_cap' (" +
          std::to_string(tracked_node_cap_.load()) +
          ") is full of present nodes carrying evidence (that hold releases when a slot frees up, or when the "
          "cap is raised)");
    }
    if (unmatched) {
      add("a require_active entry has not matched any node yet (that hold releases as soon as it matches, "
          "or by itself after " +
          std::to_string(kDefaultUnmeasuredHoldTicks) + " ticks)");
    }
    if (!pending_not_managed.empty()) {
      add(std::to_string(pending_not_managed.size()) +
          " required node(s) have had no lifecycle label read this run (starting with '" +
          *pending_not_managed.begin() + "'; after " + std::to_string(kDefaultUnmeasuredHoldTicks) +
          " ticks that hold releases by itself and the node is reported under GRAPH_NODE_NOT_MANAGED instead)");
    }
    if (!pending_unreadable.empty()) {
      add(std::to_string(pending_unreadable.size()) +
          " required node(s) are matched to a managed node whose lifecycle label has never answered "
          "(starting with '" +
          *pending_unreadable.begin() + "'; after " + std::to_string(kDefaultUnmeasuredHoldTicks) +
          " unanswered ticks that hold releases by itself and the node is reported under "
          "GRAPH_NODE_UNREADABLE instead)");
    }
    if (!pending_violation.empty()) {
      add(std::to_string(pending_violation.size()) +
          " required node(s) are measured not-active but still within grace " + "(starting with '" +
          *pending_violation.begin() + "'; that hold releases as soon as the node reads active or " +
          "its streak passes grace)");
    }
    RCLCPP_WARN(ctx.gateway_node->get_logger(),
                "graph_watchdog lifecycle_expectation: nothing is reported inactive, but GRAPH_NODE_INACTIVE is "
                "withheld from clearing because %s",
                reason.c_str());
  }

  std::set<std::string> require_active_;
  std::set<std::string> warned_;                 // require_active entries already warned about (no lifecycle state)
  std::map<std::string, int> unmanaged_streak_;  // entry -> consecutive ticks matching only unmanaged nodes
  std::vector<std::string> warnings_;            // configure()-time findings, surfaced once per config
  bool warnings_logged_ = false;
  std::set<std::string> ever_matched_;  // require_active entries that have matched >=1 node this config
  int unmatched_ticks_ = 0;             // consecutive ticks with an entry that has never matched (frozen past the hold)
  int withheld_ticks_ = 0;              // consecutive ticks the clear has been withheld
  bool withheld_reported_ = false;      // withhold already explained this episode
  /// Written by configure()/tick() on the plugin's tick thread, read by status_json() on an
  /// HTTP handler thread - atomics, because that route takes no lock this detector holds.
  std::atomic<int> tracked_node_cap_{kDefaultTrackedNodeCap};
  std::atomic<bool> saturated_{false};         ///< the cap refused a required node on the last tick
  std::atomic<std::size_t> tracked_nodes_{0};  ///< tracker map size as of the last tick
  LifecycleExpectationTracker tracker_{{}, kDefaultGrace};
  // Fixed-severity AggregatedFault members, one per code - the same shape orphan_detector
  // and param_drift_detector use, now that each code's content decides its own emission
  // independently.
  AggregatedFault aggregated_inactive_{graph_fault_codes::kNodeInactive, kConfirmedInactiveSeverity};
  AggregatedFault aggregated_unreadable_{graph_fault_codes::kNodeUnreadable, kUnmeasuredSeverity};
  AggregatedFault aggregated_not_managed_{graph_fault_codes::kNodeNotManaged, kUnmeasuredSeverity};
};

REGISTER_DETECTOR(LifecycleExpectationDetector, "lifecycle_expectation")

}  // namespace ros2_medkit_graph_watchdog
