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

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_graph_watchdog {

/// Consecutive ticks a require_active entry may match NO node at all before it is worth
/// warning about. A required node that never came up is invisible to the presence class
/// (GRAPH_NODE_DISAPPEARED needs the node present and online first), so nothing else
/// reports it.
inline constexpr int kDefaultNoMatchWarnTicks = 10;
/// Consecutive ticks a matched node may be genuinely ABSENT from the snapshot before its
/// clocks start moving again. Inside this budget a node's whole state is simply HELD:
/// zeroing it on the first blink would mean a node dropping out of one snapshot in every
/// few never accumulates enough consecutive present ticks on either clock below to be
/// reported at all. PAST this budget absence still never DISCARDS a clock, but it may no
/// longer be the one thing MOVING it either: an already-matured fault CONTINUES exactly as
/// it did on its last present tick, while a violation streak that has not yet matured is
/// simply held at whatever it already reached, for a node the presence detector could also
/// have reported - only a present tick can still advance it there. A node the presence
/// detector could structurally never report gets no such backstop: absence keeps advancing
/// its streak too, exactly like a still-climbing unmeasured clock does. See "Absence
/// continues, it never erases" in LifecycleExpectationTracker's class doc.
inline constexpr int kDefaultAbsenceGrace = 3;

/// Default bound on how many nodes this tracker keeps state for at once - the operator's
/// `tracked_node_cap`. Evidence is never reclaimed by AGE (an age horizon is itself an
/// evasion for a node that touches it periodically), so this is what bounds the map: at the
/// cap, IDLE entries - both clocks at zero and no matured ownership, so nothing to lose -
/// are reclaimed first, then DEPARTED entries are collapsed into a count (see
/// kMaxNamedDepartedEntries), and only if every tracked node is PRESENT and carrying
/// evidence is a newly-seen node refused and the saturation surfaced to the operator. 512
/// against a realistic graph: this map only ever holds nodes a require_active entry MATCHED,
/// and a large single-robot ROS 2 graph (a full Nav2 stack plus perception) runs to roughly
/// a hundred nodes while a ten-robot fleet sharing one domain runs to a few hundred - so
/// even an operator whose entries match every node in the graph stays well under the cap, at
/// a cost of a few hundred KB to hold it. Growth past it can only come from identity CHURN
/// (respawns under ever-new fqns), which is exactly what the cap exists to bound. Same
/// value, for the same reason, as ros2_medkit_log_bridge's own max_tracked_nodes default.
inline constexpr int kDefaultTrackedNodeCap = 512;

/// The most DEPARTED entries kept individually NAMED when the cap has to make room for a
/// present node; the rest are collapsed into a per-code count (see "A present node always
/// wins a slot" in the class doc). Three, and the number falls straight out of the
/// description budget: a detail is capped at kMaxLifecycleDetailChars and
/// AggregatedFault::kMaxDescriptionChars holds at most three of them
/// (3 * 150 + 2 * 2 = 454 <= 480, while a fourth would not fit - the arithmetic
/// lifecycle_expectation_detector.cpp already static_asserts). So a fourth named departed
/// entry could never appear in any description however the ordering fell out; past three
/// they are pure cost, and the cost they impose is a slot a PRESENT node needs.
inline constexpr int kMaxNamedDepartedEntries = 3;

/// Consecutive matched ticks an UNMEASURED observation (kUnreadable or kNotManaged) must
/// hold before ABSENCE is allowed to continue it - see "Absence continues the last
/// CORROBORATED observation" in the class doc. A present, healthy, managed node reads
/// kNotManaged for a tick whenever its get_state path is missing from one sweep
/// (LifecycleWatcher::update() drops a tracked id whose path is absent from the current
/// sweep, and discover_apps() can yield an app with no services when a sweep races service
/// enumeration), so a single such reading must never be what decides that a node departed
/// unmeasurable. Six: no single racing sweep produces six consecutive readings, it is the
/// same bar the detector's own typo warning already sets (kUnmanagedWarnTicks fires on the
/// sixth consecutive unmanaged tick, guarding against this exact transient), and it is a
/// tenth of kDefaultUnmeasuredHoldTicks - so a node that is genuinely unmeasurable when it
/// leaves has corroborated that long before the hold which would report it.
inline constexpr int kDefaultObservationSettleTicks = 6;

/// Consecutive ticks a node's UNMEASURED clock (see LifecycleObservedState below) may
/// climb before the cause it is currently climbing under takes ownership of the node and
/// gets its own fault code. "Frozen one past itself" like every other bounded clock here:
/// past the bound the stored counter stops incrementing, so it only needs to distinguish
/// "still climbing" from "matured", never how far past. Not configurable via detector
/// config (mirrors the previous kUnmeasuredHoldTicks, which was a detector-file-local
/// constant) - exposed here as a constructor default so a test can shrink it without
/// waiting out 60 real ticks.
inline constexpr int kDefaultUnmeasuredHoldTicks = 60;

/// Longest label a REAL lifecycle implementation can produce on `~/transition_event`: the
/// four primary states this detector enforces against (`unconfigured`, `inactive`,
/// `active`, `finalized`) plus the six transition states lifecycle_msgs/msg/State.msg
/// defines (`configuring`, `cleaningup`, `shuttingdown`, `activating`, `deactivating`,
/// `errorprocessing`). The longest of the ten is `errorprocessing` at 15 characters.
/// kMaxLifecycleLabelChars is more than double that, so no conforming label is ever
/// affected by the trim below - only a value from a non-conforming remote publisher (the
/// label arrives verbatim off a remote `~/transition_event`, see lifecycle_watcher.cpp)
/// spends the budget. This is the PRIMARY guard (R13): the label is the untrusted,
/// unbounded part of the detail, so it is trimmed on its own rather than as part of a
/// whole-detail head-and-tail that would also mangle the node name.
inline constexpr std::size_t kMaxLifecycleLabelChars = 32;

/// Backstop on the WHOLE per-node detail string (after the label above is already
/// trimmed), for a pathological fqn or a long `require_active` "required by" list rather
/// than the label - R13's secondary guard. Sized so at least 3 maximally-long details
/// still fit inside one description under AggregatedFault::kMaxDescriptionChars (480,
/// joined by "; "): 3 * 150 + 2 * 2 = 454 <= 480, while 4 maximally-long details would not:
/// 4 * 150 + 3 * 2 = 606 > 480. lifecycle_expectation_detector.cpp static_asserts this
/// arithmetic against the real AggregatedFault::kMaxDescriptionChars constant, since this
/// header does not include aggregated_fault.hpp (see its own "Pure." class doc).
inline constexpr std::size_t kMaxLifecycleDetailChars = 150;

/// Truncates `text` to at most `max_chars`, replacing the tail with "..." when it does not
/// fit whole - the same suffix-ellipsis convention AggregatedFault::describe_ordered uses
/// for the overall description, so a trimmed label or detail reads the same way a trimmed
/// description does. Shared by every detail builder below.
inline std::string trim_to(const std::string & text, std::size_t max_chars) {
  if (text.size() <= max_chars) {
    return text;
  }
  constexpr std::size_t kMarkerChars = 3;  // "..."
  const std::size_t keep = max_chars > kMarkerChars ? max_chars - kMarkerChars : 0;
  return text.substr(0, keep) + "...";
}

/// One live node matched by a require_active entry, with its lifecycle state.
struct LifecycleMatch {
  std::string entry;  ///< the require_active entry that matched
  std::string fqn;    ///< the node's stable App::effective_fqn()
  std::optional<std::string> state;
  /// Whether the presence detector will OWN this node's departure. Narrower than "may
  /// raise": it needs the node's lifecycle state either known not to be a managed-non-active
  /// one or asked for as often as it ever will be, so a managed node whose state has not
  /// been read YET answers false here even though the gate would permit it to raise - and
  /// answers true again once the watcher has stopped asking, since nothing else would report
  /// that node at all. It also carries the caller's own online check, because node_death
  /// skips an app that is not online before it consults the gate. Read off the gate and the
  /// snapshot by the caller, not re-derived here: it composes warmup, the lifecycle watcher's
  /// record and its re-seed budget, none of which this header has access to or may
  /// reimplement. Defaults true - the narrower of the two behaviours
  /// NodeState::ever_armed then selects between - so a caller that does not wire this
  /// through keeps the already-shipped absence handling rather than silently gaining a new
  /// reporting path.
  bool armed = true;
};

/// The one observed fact this whole state machine runs on: what a MATCHED node's read
/// tells us this tick. A node that did not match anything this tick never reaches
/// classify_observed_state() at all - it is ABSENT, and absence is handled by a wholly
/// separate path (the per-node absence budget in update()), not by this enum, because
/// absence carries no label to classify.
enum class LifecycleObservedState {
  kActive,      ///< label == "active"
  kInactive,    ///< label non-empty and != "active" - the one state that ever counts
                ///< toward the violation clock
  kUnreadable,  ///< optional("") - a MANAGED node whose label has never been read
  kNotManaged,  ///< nullopt - no tracked lifecycle at all this tick
};

/// Pure function of the label alone.
inline LifecycleObservedState classify_observed_state(const std::optional<std::string> & state) {
  if (!state.has_value()) {
    return LifecycleObservedState::kNotManaged;
  }
  if (state->empty()) {
    return LifecycleObservedState::kUnreadable;
  }
  return *state == "active" ? LifecycleObservedState::kActive : LifecycleObservedState::kInactive;
}

/// Which of the two "I cannot measure this" causes a node's unmeasured clock most
/// recently climbed under. Meaningful in two different ways depending on
/// `NodeState::unmeasured_matured` (private below): while the clock is still climbing it
/// is simply "what did the last unmeasured tick look like" (refreshed every such tick, for
/// the withheld-clear log's own bookkeeping); once matured it is frozen - see "Sticky vs.
/// current-cause-wins" in update() for why.
enum class LifecycleUnmeasuredCause {
  kUnreadable,
  kNotManaged,
};

/// What one update() sweep found. Ownership across the three fault-shaped maps below is
/// exclusive: a node appears in at most one of `affected`, `unreadable_affected` and
/// `not_managed_affected` on any given tick - see "Ownership is exclusive" in update().
struct LifecycleExpectationReport {
  /// fqn -> detail phrase, for GRAPH_NODE_INACTIVE: a node CONFIRMED non-active (its
  /// violation clock crossed `grace`) and not currently owned by the unmeasured clock.
  std::map<std::string, std::string> affected;
  /// fqns whose violation clock crossed `grace` on THIS update() - i.e. were ADDED to
  /// `affected` this tick, not merely still in it from an earlier one. Lexicographic by
  /// fqn (this tracker's own node map is a std::map) when several cross together, so the
  /// caller's new-first ordering is deterministic even in the degenerate `grace: 0` case.
  std::vector<std::string> newly_affected;

  /// fqn -> detail phrase, for GRAPH_NODE_UNREADABLE: the unmeasured clock matured with
  /// cause kUnreadable and still holds that cause (see the sticky rule in update()).
  std::map<std::string, std::string> unreadable_affected;
  std::vector<std::string> newly_unreadable;

  /// fqn -> detail phrase, for the not-managed fault code: the unmeasured clock matured
  /// with cause kNotManaged and still holds that cause.
  std::map<std::string, std::string> not_managed_affected;
  std::vector<std::string> newly_not_managed;

  /// fqns whose GRAPH_NODE_INACTIVE status is UNSETTLED this tick - a violation streak that
  /// has not yet passed `grace`, or an unmeasured clock still climbing (not yet matured),
  /// or a streak HELD while the node is inside an unmeasured spell. The caller must not
  /// assert "nothing is inactive" while any of these exist: a level-triggered clear
  /// whenever `affected` is empty would otherwise heal a fault whose node's status this
  /// tracker simply has not settled yet. Once a node's unmeasured clock MATURES it leaves
  /// this set entirely (ownership passes to its own fault code, and GRAPH_NODE_INACTIVE has
  /// nothing left to say about it either way - see "the violation streak is RELEASED" in
  /// advance_unmeasured_clock()). Absence never puts a node here on its own: content
  /// follows the clocks, not the snapshot, so a node already past `grace` stays in
  /// `affected` through a blink rather than dropping into a withheld limbo.
  std::set<std::string> pending;
  /// Breakdown of `pending`, for the withheld-clear LOG LINE only - never the withhold
  /// decision itself, which is `!pending.empty()`. A node can appear in more than one
  /// (e.g. a held violation streak while ALSO climbing the unmeasured clock), so these are
  /// not a partition and must not be summed against `pending.size()`.
  std::set<std::string> pending_violation;    ///< below-grace violation streak
  std::set<std::string> pending_unreadable;   ///< unmeasured clock climbing, cause kUnreadable
  std::set<std::string> pending_not_managed;  ///< unmeasured clock climbing, cause kNotManaged

  /// Entries that have matched nothing for more CONSECUTIVE ticks than the no-match
  /// threshold and have not been reported yet. Consecutive, not "never": an entry whose node
  /// matched and later left the graph surfaces here too, and for that one the caller's
  /// "never came up, and the presence class cannot see it either" framing is false - so the
  /// caller filters on whether the entry has ever matched before it says anything.
  std::vector<std::string> entries_matching_nothing;

  /// LEVEL, not an edge: true on every tick a newly matched node had to be refused because
  /// the tracked-node cap is full of PRESENT nodes all carrying evidence, and nothing could
  /// be reclaimed or collapsed. It stays true for as long as the condition lasts, because a
  /// refused node is still matched on the following tick and refused again. Refusing the
  /// newcomer rather than evicting a live violation is the safe direction, but it means a
  /// required node is going unchecked, so the caller must NOT clear GRAPH_NODE_INACTIVE
  /// while it holds - a detector that declined to check a required node cannot assert that
  /// every required node is healthy.
  bool tracking_saturated = false;
  /// EDGE: the first tick of a saturation episode, for the log line. Re-arms when the
  /// episode ends, so a later, real saturation is not silent because an earlier one spent
  /// the one warning.
  bool saturation_started = false;
};

/// Enforces "these nodes must be active" and, inseparably from it, "I could not tell
/// whether these nodes are active" - one state machine per matched node rather than two
/// systems that have to agree.
///
/// **The model.** Per node, per tick, exactly one LifecycleObservedState (or ABSENT, its
/// own separate case - see below). Two clocks, and only two:
///
/// - **Violation streak** (`NodeState::violation_streak`). Advances on kInactive. Resets
///   ONLY on kActive. Past `grace`, the node is in `affected` (GRAPH_NODE_INACTIVE).
/// - **Unmeasured clock** (`NodeState::unmeasured_clock`). Advances on kUnreadable OR
///   kNotManaged. Resets ONLY on a real measurement - kActive or kInactive. It is BLIND to
///   which of the two causes it saw on any given tick: a node alternating between
///   "matched but never read" and "not a tracked lifecycle node at all" keeps this single
///   clock climbing instead of each cause resetting the other's own counter - which is
///   what let such a node evade both codes forever in the design this replaces (a
///   MANAGED-node-whose-label-never-arrives counter and a NOT-MANAGED counter, each zeroed
///   by the other's tick). Past `unmeasured_hold_ticks`, the node's UNMEASURED clock
///   matures and takes ownership - see "Ownership is exclusive" below.
///
/// **Ownership is exclusive.** A node is content of at most one of `affected`,
/// `unreadable_affected` or `not_managed_affected` at a time. The moment the unmeasured
/// clock matures, the violation streak is RELEASED (reset to 0) - not merely excluded from
/// `affected` THIS tick, but actually zeroed - so GRAPH_NODE_INACTIVE has nothing left to
/// hold and its clear is free to flow. A node returning to kInactive afterwards therefore
/// re-earns `grace` from zero, exactly as a node that had never gone unmeasured would.
/// Below maturity the two still do not overlap: a node inside an unmeasured spell
/// (`unmeasured_clock > 0`) is never `affected` content however far its streak had already
/// climbed - "I cannot measure this right now" is not a violation - but the streak is HELD,
/// not reset, and resumes the instant a real read comes back.
///
/// **Content follows the clocks, not the snapshot.** Whether a node happened to be in this
/// tick's matches decides nothing about what it reports. A node whose streak is past
/// `grace` stays in `affected` while it blinks, and a matured node stays in its own map,
/// because a fault's content is what the tracker has MEASURED about the node, and a missing
/// snapshot entry measures nothing either way. What absence does change is the detail
/// phrase: past `absence_grace` it says the node has left the graph, so an operator is
/// never sent to look at a node that is no longer there.
///
/// **Sticky vs. current-cause-wins (the one design choice this class makes on its own).**
/// The unmeasured clock is cause-blind, but the two causes map to two different fault
/// codes. What should a node report once its clock has MATURED, if the cause it is
/// observed under later changes (unreadable this tick, not-managed the next, without ever
/// crossing back to a real measurement)? This class is STICKY: `NodeState::cause` freezes
/// at whatever it was on the tick the clock matured, and nothing but a real measurement
/// (kActive/kInactive, which resets the whole clock) ever changes it again - so a node that
/// matured as GRAPH_NODE_UNREADABLE keeps reporting under that code even if it is later
/// observed NOT_MANAGED for a while, and vice versa. The rejected alternative is
/// current-cause-wins: report whichever cause the LIVE tick shows, so the fault code can
/// flip between the two on every alternating tick. Current-cause-wins is the more literal
/// reading of "exactly what is true right now", but it makes a node that flaps between the
/// two causes flap the FAULT SURFACE too - a raise/clear/raise churn on two different fault
/// codes for a node whose actual situation (still cannot be measured, whichever way you
/// slice it) never changed. Sticky keeps the surface calm: once "I cannot measure this"
/// has been reported, which specific unmeasured-flavor it is stays fixed until something
/// REAL is learned about the node, matching how `grace` and `unmeasured_hold_ticks`
/// already treat a real measurement as the only thing that resets a clock.
///
/// **Absence continues the last CORROBORATED observation, it never erases.** A node not
/// matched at all this tick is ABSENT - a wholly separate fact from any
/// LifecycleObservedState, since there is no label to classify. For up to `absence_grace`
/// consecutive absent ticks a node's whole state (both clocks, whichever fault - if any -
/// currently owns it) is simply held, unchanged, so a node that blinks out of one snapshot
/// in every few is not indistinguishable from one that never stayed present long enough to
/// be measured at all. PAST `absence_grace` absence advances whichever clock the node's
/// `NodeState::settled_observed` says it was on, and resets nothing:
///
/// - settled kInactive, ALREADY REPORTED under GRAPH_NODE_INACTIVE (its streak past
///   `grace`): the streak continues exactly as it did on its last present tick - frozen at
///   `grace` + 1 either way, see advance_violation_streak's own "frozen one past itself" -
///   so a node measured not-active and confirmed that then vanishes keeps its fault raised,
///   still naming a node that is no longer there. That is the honest reading of the
///   expectation the operator wrote: the node must be ACTIVE, it was not and was already
///   said so, and now it is gone too.
/// - settled kInactive, NOT YET REPORTED (streak at or below `grace`), on a node that WAS
///   ARMED at some point (`NodeState::ever_armed`): the streak is HELD, neither advanced
///   nor erased. Maturing it here would raise GRAPH_NODE_INACTIVE from ticks gathered while
///   the node could not be observed at all - evidence the operator's own graph never
///   witnessed - and the presence detector is ABLE to report this exact departure instead:
///   node_death tracks any node the gate has admitted for presence OWNERSHIP at least once
///   (NodeLivenessTracker's own "a key becomes TRACKED the first time it is armed"), so
///   starting a NEW violation from a departure nobody here could watch is its job, not this
///   one's. The entry keeps what it already earned, so a node that RETURNS still inactive
///   resumes its streak rather than re-earning `grace` from zero.
/// - settled kInactive, NOT YET REPORTED, on a node that was NEVER armed: the streak keeps
///   climbing on absence exactly as it would on a present tick. The reasoning above
///   inverts: node_death only ever tracks a node the gate has admitted for presence
///   ownership at least once, so one that never reached that bar is structurally invisible
///   to the presence detector no matter what happens to it afterwards. Two shapes reach
///   that bar and fail it: a `require_active` entry that comes up `unconfigured` and is
///   killed before its own `grace` elapses, and a node that is not online, which node_death
///   skips before it ever asks the gate. A managed node whose state is merely UNREAD is a
///   third, and a temporary one: the gate withholds ownership while it is still asking, then
///   grants it PROVISIONALLY once it stops - and that grant is not latched here, because the
///   presence detector itself gives it up the moment a real label arrives. Holding
///   this streak too would mean nothing in the plugin ever reports the departure. This
///   keeps the same bound a still-climbing UNMEASURED clock already has regardless of
///   arming (see "Bounded by evidence, not by age" below): the streak matures within
///   `grace` + `absence_grace` + 1 ticks, never staying unsettled forever the way an ARMED
///   node's below-grace streak may.
///
///   Either way, any UNCORROBORATED unmeasured spell the node had also started is dropped
///   here rather than held: nothing is left to read, and a clock that can never mature
///   would keep the entry non-idle - and therefore `pending` - for the life of the process.
/// - settled kUnreadable or kNotManaged: the unmeasured clock keeps climbing, under the
///   cause that observation set - absence carries no label, so it cannot change one.
/// - settled kActive: nothing advances, and anything the node had started but not
///   corroborated is released, so the entry becomes idle and is reclaimed silently. An
///   entry that is already CONTENT (a matured unmeasured clock, or a streak past `grace`)
///   is never released - a departure heals nothing. Starting a NEW violation from a healthy
///   departure is the presence class's job (GRAPH_NODE_DISAPPEARED, this package's own
///   node_death detector), not this class's - a far narrower boundary than "a node absent
///   past the grace is invisible whichever clock it was on".
///
/// **What "settled" means, and the alternatives rejected for it.** A real measurement
/// (kActive or kInactive) settles IMMEDIATELY: a lifecycle label is a fact about the node,
/// and no sweep artifact can invent one. An UNMEASURED observation settles immediately too
/// on a node NOTHING HAS EVER MEASURED, because it is then the only thing known about it -
/// which is what keeps a node that is unreadable or unmanaged whenever it is present, and
/// absent the rest of the time, accumulating toward its report instead of being released on
/// every absence. It is only when an unmeasured reading would OVERRIDE a real measurement
/// that it has to be corroborated first, over `kDefaultObservationSettleTicks` consecutive
/// matched ticks. The reason is that kNotManaged and kUnreadable are BOTH producible by the
/// discovery layer missing a node's service path for one sweep, on a node that is present and
/// perfectly healthy - and if that sweep is the last one before a clean shutdown, "absence
/// continues the last observation" would mature a healthy departure into a permanent
/// GRAPH_NODE_NOT_MANAGED.
///
/// The rejected alternative is the literal one this replaces: continue whatever the single
/// last observation was. It is simpler and it is wrong for exactly the reading it cannot tell
/// apart from a real one. Rejected as well: continuing the MOST INFORMATIVE observation of
/// the recent window, which is worse still here - an unmeasured reading outranks a healthy
/// one (see observed_rank), so a single bad sweep would decide the carve-out by design rather
/// than by accident. And rejected as a shortcut: requiring corroboration unconditionally,
/// which reopens the restart-loop evasion above for any node whose uptime is shorter than the
/// settling budget. Corroboration does not weaken the promise it guards: a node that is
/// genuinely unmeasurable when it leaves has been so for far more than six ticks, since the
/// hold that would report it is sixty.
///
/// The design this replaces reset both clocks and released ownership past `absence_grace`.
/// That made every erasure horizon an evasion: a node in a restart loop - start, crash,
/// respawn delay, start - touches absence periodically BY CONSTRUCTION, so it could
/// alternate `(UNREADABLE, ABSENT x N)`, `(NOT_MANAGED, ABSENT x N)` or `(INACTIVE,
/// ABSENT x N)` forever and never accumulate enough of anything to be reported, which is
/// precisely the node this detector most exists to catch.
///
/// **Bounded by evidence, not by age.** Since absence no longer erases anything a MATURED
/// entry carries, that entry is never reclaimed by age - a departure must not heal what it
/// cannot un-happen. `prune_ticks` therefore reclaims only IDLE entries (both clocks zero,
/// no matured ownership, no live-but-unmatured streak), which carry nothing to lose. What
/// bounds the map's SIZE instead is `tracked_node_cap`.
///
/// A node whose UNMEASURED clock is still climbing keeps the bound this class shipped with:
/// past `absence_grace` it advances every tick regardless of maturity - unlike the violation
/// streak above, this clock's own absence behaviour did not change, see the two unmeasured
/// causes above - so it matures within at most `unmeasured_hold_ticks` + `absence_grace` + 1
/// ticks and is reported, which is also the longest GRAPH_NODE_UNREADABLE or
/// GRAPH_NODE_NOT_MANAGED's clear can be withheld by one departed node.
///
/// A VIOLATION streak on a node the presence detector was never able to own shares that same
/// bound, for the same reason it advances at all while absent (see the kInactive case above):
/// it matures within at most `grace` + `absence_grace` + 1 ticks, because nothing else will
/// ever report that node's departure either. Only once a node HAS been owned does its
/// below-grace streak lose the bound: absence then holds it rather than advancing it, so it
/// neither matures nor becomes idle for as long as the node is away, however long that is.
/// This is not a new way to withhold GRAPH_NODE_INACTIVE's clear - an aggregate,
/// level-triggered fault already could not assert "every required node is healthy" while any
/// one of them was unsettled, whether that unsettled node was HELD (`pending`) or climbing
/// toward CONTENT (`affected`), so a node this indecisive already blocked the same clear
/// before this design. What changes is only that GRAPH_NODE_INACTIVE never NAMES an OWNED
/// node's departure: that node's evidence belongs to GRAPH_NODE_DISAPPEARED, not to an entry
/// nobody here could measure.
///
/// **A present node always wins a slot.** At the cap the order is: reclaim IDLE entries
/// (they carry nothing); then collapse DEPARTED entries - lexicographically last first,
/// keeping at most `kMaxNamedDepartedEntries` of them named - into per-code COUNTS; and only
/// if every remaining entry is PRESENT and carrying evidence is the newcomer refused, with
/// `LifecycleExpectationReport::tracking_saturated` saying so on every such tick.
///
/// Collapsing rather than holding follows from something easy to miss: **the fault is keyed
/// by CODE, not by node.** Five hundred entries for dead identities keep exactly the same
/// one fault raised that a single entry would, and the description budget can only ever name
/// a handful of them - so holding them buys nothing, while the slots they occupy can cost
/// total blindness. Under identity churn (each robot's nodes appearing under their own
/// namespace and leaving for good) a cap full of the dead would refuse a genuinely broken
/// PRESENT node: `track()` returns nullptr, the node never enters `affected` or `pending`,
/// its entry has matched so the caller's never-matched hold does not apply either, and
/// GRAPH_NODE_INACTIVE emits a level-triggered CLEAR every tick while that node reads
/// not-active. The detector would report health it had refused to check.
///
/// **What the operator sees when a count is non-zero.** The collapsed count enters that
/// code's content as one extra line - "and N more required node(s) left the graph ..." -
/// ahead of the individually named entries but behind anything crossing on this tick, so a
/// node that just broke is never displaced by it while the summary is not itself truncated
/// away by a list of names that are all equally uninformative. Its
/// presence means two things at once: those N nodes left the graph carrying evidence and
/// their fault is still correctly raised (a departure heals nothing), and the tracked-node
/// cap is under enough pressure that they could not be kept individually - i.e. `require_active`
/// is matching an unbounded set of identities, or `tracked_node_cap` is too small for the
/// fleet. The count only ever grows within one tracker lifetime: once an entry is collapsed
/// the tracker no longer knows which fqn it was, so a node returning under that same fqn is
/// tracked afresh, measured afresh, and cannot decrement it. That is deliberate rather than
/// overlooked - it is "a departure never heals a fault" applied to an entry that can no
/// longer be named - and it is bounded the same way the promise itself is: a gateway restart
/// re-baselines everything.
///
/// **Keyed by node, not by config entry.** The bare-name form of require_active is
/// deliberately fleet-wide ("all controller_servers must be active"), so one entry
/// legitimately covers N nodes; two entries (a fleet-wide bare name plus a pinned FQN) can
/// also both name the SAME node, in which case its clocks each advance once per tick, not
/// once per matching entry (counting per match would silently halve whatever budget the
/// operator configured). Every entry that named a node is still carried as "required by"
/// context in that node's detail string.
class LifecycleExpectationTracker {
 public:
  /// kNoPrune keeps every IDLE entry forever too - the default, so a short-lived unit test
  /// is unaffected. Production wiring passes the operator's own `prune_grace`.
  static constexpr int kNoPrune = std::numeric_limits<int>::max();

  /// prune_ticks, unmeasured_hold_ticks and tracked_node_cap are LAST on purpose: every
  /// parameter here is an int, so inserting one in the middle keeps existing positional
  /// calls compiling while silently rebinding them.
  LifecycleExpectationTracker(std::set<std::string> require_active, int grace, int absence_grace = kDefaultAbsenceGrace,
                              int no_match_warn_ticks = kDefaultNoMatchWarnTicks, int prune_ticks = kNoPrune,
                              int unmeasured_hold_ticks = kDefaultUnmeasuredHoldTicks,
                              int tracked_node_cap = kDefaultTrackedNodeCap)
    : require_active_(std::move(require_active))
    , grace_(grace)
    , absence_grace_(absence_grace)
    , no_match_warn_ticks_(no_match_warn_ticks)
    , prune_ticks_(prune_ticks)
    , unmeasured_hold_ticks_(unmeasured_hold_ticks)
    , tracked_node_cap_(tracked_node_cap < 1 ? 1 : tracked_node_cap) {
  }

  /// Nodes whose bookkeeping is still held - for tests asserting the map stays bounded.
  /// Every per-node fact (both clocks, absence count, cause, entries) now lives in ONE map,
  /// so this is simply its size - no second map to reconcile against.
  std::size_t tracked_count() const {
    return nodes_.size();
  }

  LifecycleExpectationReport update(const std::vector<LifecycleMatch> & matches) {
    LifecycleExpectationReport report;

    // Collapse the per-(entry, node) matches to one record per NODE before any clock
    // moves - see "Keyed by node, not by config entry" above.
    struct NodeTick {
      std::vector<std::string> entries;  ///< every entry that named this node, in match order
      std::optional<std::string> state;  ///< the label enforced for it this tick
      bool armed = false;                ///< whether the gate allowed a raise for this node on THIS tick
    };
    std::set<std::string> matched_entries;
    std::map<std::string, NodeTick> nodes;
    for (const auto & match : matches) {
      matched_entries.insert(match.entry);
      const auto inserted = nodes.emplace(match.fqn, NodeTick{});
      NodeTick & node = inserted.first->second;
      if (std::find(node.entries.begin(), node.entries.end(), match.entry) == node.entries.end()) {
        node.entries.push_back(match.entry);
      }
      // Duplicate matches for one node carry the same label by construction (the caller
      // reads the label once per node), but if they ever disagree the tie-break below picks
      // the MOST INFORMATIVE reading: a confirmed violation beats an unmeasured read, which
      // beats a merely-not-tracked one, which beats a healthy one - a detector for silent
      // faults must not let a duplicate match throw information away.
      if (inserted.second ||
          observed_rank(classify_observed_state(match.state)) > observed_rank(classify_observed_state(node.state))) {
        node.state = match.state;
      }
      // Same app.id, same tick, so every duplicate match necessarily carries the same
      // arming fact in practice - OR rather than overwrite only so a future caller that
      // ever legitimately disagrees cannot make this node look LESS armed than any one
      // match already said it was.
      node.armed = node.armed || match.armed;
    }

    // Clocks that cross their bound on THIS tick. Collected here rather than pushed
    // straight into report.newly_*, so the content pass below - one lexicographic sweep of
    // nodes_ - is the only place that decides ordering, whether the node that crossed was
    // matched this tick or merely absent past its blink tolerance.
    std::set<std::string> crossed_violation;
    std::set<std::string> crossed_unmeasured;

    std::set<std::string> seen_fqns;
    for (const auto & entry : nodes) {
      const std::string & fqn = entry.first;
      const NodeTick & node_tick = entry.second;
      NodeState * const tracked = track(fqn, report);
      if (tracked == nullptr) {
        continue;  // every tracked node carries evidence and the cap is full - see track()
      }
      seen_fqns.insert(fqn);
      NodeState & node = *tracked;
      node.absent_ticks = 0;
      node.entries = node_tick.entries;  // refreshed on every matched tick, held through a blink
      // Sticky: once the gate has admitted this node for presence OWNERSHIP on any one tick,
      // the presence detector could have picked up its departure from that tick onward, for
      // the rest of this node's life - see NodeState::ever_armed. Latching the stricter fact
      // is what lets a node that went active, deactivated and then died stay node_death's,
      // while one whose state was never measured at all never becomes its.
      node.ever_armed = node.ever_armed || node_tick.armed;

      const LifecycleObservedState state = classify_observed_state(node_tick.state);
      switch (state) {
        case LifecycleObservedState::kActive:
          // A real measurement: both clocks reset, ownership released, nothing left to say.
          // A label is a fact about the node, so it settles at once - see "What 'settled'
          // means" in the class doc.
          node.violation_streak = 0;
          node.unmeasured_clock = 0;
          node.unmeasured_matured = false;
          node.last_label.clear();
          node.settled_observed = state;
          node.ever_measured = true;
          break;
        case LifecycleObservedState::kInactive:
          // A real measurement resets the unmeasured clock and releases unmeasured
          // ownership (a node returning to a real read - even a bad one - is no longer
          // "cannot be measured"), THEN the violation clock advances under its own rules.
          node.unmeasured_clock = 0;
          node.unmeasured_matured = false;
          // Trimmed on the way IN, so the untrusted remote-supplied label bounds the memory
          // this tracker holds as well as the detail it later builds (R13's primary guard).
          node.last_label = trim_to(*node_tick.state, kMaxLifecycleLabelChars);
          node.settled_observed = state;
          node.ever_measured = true;
          advance_violation_streak(node, fqn, crossed_violation);
          break;
        case LifecycleObservedState::kUnreadable:
        case LifecycleObservedState::kNotManaged:
          // Cause-blind advance: this clock does not care which of the two it is seeing,
          // only that a real measurement has not happened. While still climbing, `cause`
          // tracks the live cause (for the withheld-clear log, and so that whatever is live
          // AT MATURITY is what freezes); once matured it is left untouched - see the class
          // doc's "Sticky vs. current-cause-wins".
          if (!node.unmeasured_matured) {
            node.cause = state == LifecycleObservedState::kUnreadable ? LifecycleUnmeasuredCause::kUnreadable
                                                                      : LifecycleUnmeasuredCause::kNotManaged;
          }
          advance_unmeasured_clock(node, fqn, crossed_unmeasured);
          // An unmeasured reading has to be CORROBORATED before absence may continue it - but
          // only when there is a real measurement for it to override. On a node nothing has
          // ever measured, it is the only thing known, so it settles at once. The clock IS
          // the run length, since only a real measurement resets it. See "What 'settled'
          // means" in the class doc.
          if (!node.unmeasured_matured &&
              (!node.ever_measured || node.unmeasured_clock >= kDefaultObservationSettleTicks)) {
            node.settled_observed = state;
          }
          break;
      }
    }

    // Absence loop: every tracked node NOT matched this tick. Inside absence_grace_
    // everything is HELD unchanged (a blink). Past it, absence resets nothing but no longer
    // advances everything either - see "Absence continues, it never erases" in the class
    // doc. A node last measured kInactive continues an ALREADY-matured violation exactly as
    // it was, but a below-grace one is simply held, never matured on the strength of absence
    // alone. A node last measured kUnreadable/kNotManaged keeps climbing its unmeasured
    // clock regardless of maturity - that clock's own absence behaviour did not change. A node
    // last measured kActive continues nothing: a healthy node shutting down is
    // GRAPH_NODE_DISAPPEARED's business, which this package's node_death detector now
    // covers. Only IDLE bookkeeping is reclaimed by age, because only an idle entry has
    // nothing to lose.
    for (auto it = nodes_.begin(); it != nodes_.end();) {
      const std::string & fqn = it->first;
      NodeState & node = it->second;
      if (seen_fqns.count(fqn) != 0) {
        ++it;
        continue;
      }
      if (node.absent_ticks < std::numeric_limits<int>::max()) {
        ++node.absent_ticks;  // saturating: past both horizons the exact count stops mattering
      }
      if (node.absent_ticks > absence_grace_) {
        switch (node.settled_observed) {
          case LifecycleObservedState::kInactive:
            // An unmeasured spell that never lasted long enough to be corroborated dies with
            // the node: there is nothing left to read, and a clock that can never mature
            // would keep this entry non-idle - and therefore `pending` - forever.
            node.unmeasured_clock = 0;
            // Two independent questions decide whether this tick may advance the streak.
            // First, is_content(node): "has this node's violation already been reported",
            // not "how many ticks has it accumulated" - already reported, absence continues
            // it exactly like any other tick would (advance_violation_streak() no-ops past
            // `grace` regardless, see its own "frozen one past itself"), so this call only
            // documents that maturity, once earned, survives a departure. Second,
            // `!node.ever_armed`: who ELSE could ever report this node's departure. A node
            // the presence detector could have tracked (admitted for ownership at least
            // once) needs no help from here even below grace: maturing it would raise
            // GRAPH_NODE_INACTIVE from ticks gathered while nobody could observe the node,
            // evidence the presence detector owns instead (node_death tracks any node the
            // gate has admitted for ownership at least once - NodeLivenessTracker's own "a
            // key becomes TRACKED the first time it is armed") - so the streak is left
            // untouched: neither advanced (no fault born from an absence the presence class
            // will report anyway) nor erased (a node that RETURNS still inactive resumes
            // from here rather than re-earning `grace` from zero). A node that was never
            // admitted is structurally invisible to the presence detector no matter what
            // happens to it afterwards - a managed node whose lifecycle state was never
            // measured is one of them, since ownership needs the state positively known -
            // so nothing else in the plugin will ever report this departure either: absence
            // keeps advancing the streak for it exactly as a present tick would, the same
            // way it already does for a still-climbing unmeasured clock below.
            if (is_content(node) || !node.ever_armed) {
              advance_violation_streak(node, fqn, crossed_violation);
            }
            break;
          case LifecycleObservedState::kUnreadable:
          case LifecycleObservedState::kNotManaged:
            advance_unmeasured_clock(node, fqn, crossed_unmeasured);
            break;
          case LifecycleObservedState::kActive:
            release_uncorroborated(node);  // healthy departure: nothing here starts a violation
            break;
        }
      }
      if (node.absent_ticks > prune_ticks_ && is_idle(node)) {
        it = nodes_.erase(it);  // idle: carries no evidence, so reclaiming it loses nothing
      } else {
        ++it;
      }
    }

    // Content pass: EVERY tracked node, present or not - content follows the clocks, not
    // the snapshot (see the class doc). One lexicographic sweep, so the newly_* orderings
    // are deterministic even when several nodes cross together - but PRESENT crossings are
    // collected apart from DEPARTED ones and go first in each newly_* list. The caller feeds
    // newly_* to AggregatedFault::emit_ordered, which stops once the description budget is
    // spent, so on a tick where a departed node and a present, newly-broken one both cross,
    // a single lexicographic list could name the one that LEFT and truncate away the one
    // that just broke.
    std::vector<std::string> departed_affected, departed_unreadable, departed_not_managed;
    for (auto & [fqn, node] : nodes_) {
      const bool departed = node.absent_ticks > absence_grace_;
      if (node.unmeasured_matured) {
        const std::string detail = unmeasured_detail(fqn, node, departed);
        const bool crossed = crossed_unmeasured.count(fqn) != 0;
        if (node.cause == LifecycleUnmeasuredCause::kUnreadable) {
          report.unreadable_affected[fqn] = detail;
          if (crossed) {
            (departed ? departed_unreadable : report.newly_unreadable).push_back(fqn);
          }
        } else {
          report.not_managed_affected[fqn] = detail;
          if (crossed) {
            (departed ? departed_not_managed : report.newly_not_managed).push_back(fqn);
          }
        }
        continue;  // matured: never pending for GRAPH_NODE_INACTIVE (ownership released it)
      }
      // A node inside an unmeasured spell is not confirmed content however far its streak
      // had climbed - the streak is HELD, not reported, until a real read comes back.
      if (node.unmeasured_clock == 0 && node.violation_streak > grace_) {
        report.affected[fqn] = inactive_detail(fqn, node, departed);
        if (crossed_violation.count(fqn) != 0) {
          (departed ? departed_affected : report.newly_affected).push_back(fqn);
        }
      }
      if (node.violation_streak > 0 && report.affected.count(fqn) == 0) {
        report.pending.insert(fqn);
        report.pending_violation.insert(fqn);
      }
      if (node.unmeasured_clock > 0) {
        report.pending.insert(fqn);
        if (node.cause == LifecycleUnmeasuredCause::kUnreadable) {
          report.pending_unreadable.insert(fqn);
        } else {
          report.pending_not_managed.insert(fqn);
        }
      }
    }
    append_all(report.newly_affected, departed_affected);
    append_all(report.newly_unreadable, departed_unreadable);
    append_all(report.newly_not_managed, departed_not_managed);

    // Departed entries collapsed to free slots for present nodes still count as content, or
    // freeing the slot would heal a fault a departure must never heal. Sorted after every
    // real fqn (see kCollapsedKeyPrefix), so this never takes description budget from a
    // node that is still named.
    add_collapsed_content(report.affected, collapsed_inactive_, "not active");
    add_collapsed_content(report.unreadable_affected, collapsed_unreadable_, "with an unread lifecycle state");
    add_collapsed_content(report.not_managed_affected, collapsed_not_managed_, "unmanaged");

    report.saturation_started = report.tracking_saturated && !saturated_last_tick_;
    saturated_last_tick_ = report.tracking_saturated;

    // An entry matching nothing is silent in every other path: it never reaches the
    // violation branch above, and the presence class never starts tracking a node that was
    // never present, so a misspelt entry - or one whose node crashed during launch -
    // produces no fault and no log line anywhere.
    for (const auto & entry : require_active_) {
      if (matched_entries.count(entry) != 0) {
        no_match_[entry] = 0;
        continue;
      }
      if (++no_match_[entry] > no_match_warn_ticks_ && reported_no_match_.insert(entry).second) {
        report.entries_matching_nothing.push_back(entry);
      }
    }
    return report;
  }

 private:
  /// Per-node bookkeeping - the entire state machine for one fqn. Everything that used to
  /// be spread across two counter maps in this class plus two more in the detector's own
  /// guard now lives here.
  struct NodeState {
    int violation_streak = 0;         ///< consecutive kInactive ticks since the last kActive
    int unmeasured_clock = 0;         ///< consecutive kUnreadable-or-kNotManaged ticks since
                                      ///< the last real measurement (frozen one past the hold)
    bool unmeasured_matured = false;  ///< the unmeasured clock has crossed the hold and not
                                      ///< yet been reset by a real measurement
    LifecycleUnmeasuredCause cause = LifecycleUnmeasuredCause::kUnreadable;  ///< live while
                                                                             ///< climbing, frozen (sticky) once
                                                                             ///< unmeasured_matured - see the class doc
    /// What absence continues past absence_grace: the last observation of this node that
    /// COUNTS - a real measurement immediately, an unmeasured reading either immediately (if
    /// there is no real measurement for it to override) or once it has held for
    /// kDefaultObservationSettleTicks consecutive matched ticks. kActive is the right initial
    /// value: it continues nothing, and it is only ever read while `ever_measured` is true.
    LifecycleObservedState settled_observed = LifecycleObservedState::kActive;
    /// Whether a REAL measurement (kActive or kInactive) has ever been taken of this node.
    /// It is what an unmeasured reading has to be corroborated against: with no real
    /// measurement on record an unmeasured reading is simply the best - and only - thing
    /// known about the node, so it settles at once. Without this distinction a node that is
    /// unreadable or unmanaged whenever it is present and absent the rest of the time (a
    /// restart loop, the case this detector most exists to catch) would have its clock
    /// released on every absence and never mature.
    bool ever_measured = false;
    /// Whether the reliability gate has admitted this node for presence OWNERSHIP on at
    /// least one matched tick, ever. Sticky: once true it stays true, because the fact it
    /// stands in for - "the presence detector could report this node's departure" - is
    /// itself sticky (NodeLivenessTracker tracks a key from its first admitted tick onward,
    /// regardless of its arm state afterwards). Read here rather than re-derived from
    /// `settled_observed` or the live label: this node's LIFECYCLE state alone cannot say
    /// whether the gate admitted it, since admission also needs warmup, which this class
    /// does not track - guessing from state would either miss a warming-up node that later
    /// arms, or wrongly treat a node this class never measured active as owned. See the
    /// absence loop's kInactive case for the one place this decides anything.
    bool ever_armed = false;
    /// The label of the last kInactive observation, already trimmed to
    /// kMaxLifecycleLabelChars. Kept because the detail for a CONFIRMED violation names the
    /// state the node is stuck in, and absence carries no label of its own to name. Cleared
    /// by kActive, the one observation that also clears the streak, so it can never outlive
    /// the streak it describes.
    std::string last_label;
    std::vector<std::string> entries;  ///< "required by" entries, refreshed on every matched tick
    int absent_ticks = 0;              ///< consecutive ticks this fqn was not matched at all (saturating)
  };

  /// An entry with nothing to lose: no streak, no unmeasured clock, no matured ownership.
  /// The only kind that may be reclaimed - by age past prune_ticks_, or to make room at the
  /// tracked-node cap.
  static bool is_idle(const NodeState & node) {
    return node.violation_streak == 0 && node.unmeasured_clock == 0 && !node.unmeasured_matured;
  }

  /// An entry that is already a fault's CONTENT - a matured unmeasured clock, or a violation
  /// streak past `grace`. Nothing may release one: a departure heals no fault.
  bool is_content(const NodeState & node) const {
    return node.unmeasured_matured || node.violation_streak > grace_;
  }

  /// An entry whose absence has outlived the blink tolerance - the node is gone, not
  /// flickering.
  bool is_departed(const NodeState & node) const {
    return node.absent_ticks > absence_grace_;
  }

  /// A departed node whose last CORROBORATED observation was healthy has nothing for absence
  /// to continue, so anything it had started but not corroborated goes with it - otherwise a
  /// single missed sweep before a clean shutdown leaves a clock that can never mature and an
  /// entry that is never idle, i.e. `pending` for the life of the process. An entry that is
  /// already CONTENT is never touched.
  void release_uncorroborated(NodeState & node) const {
    if (is_content(node)) {
      return;
    }
    node.violation_streak = 0;
    node.unmeasured_clock = 0;
  }

  /// The tracked entry for `fqn`, creating it if a slot can be found. Returns nullptr only
  /// when every tracked node is PRESENT and carrying evidence - never evicting a live
  /// violation for a newcomer - and the report says so on every such tick, so the caller can
  /// withhold the clear it must not emit while a required node is going unchecked.
  NodeState * track(const std::string & fqn, LifecycleExpectationReport & report) {
    const auto existing = nodes_.find(fqn);
    if (existing != nodes_.end()) {
      return &existing->second;
    }
    if (nodes_.size() >= static_cast<std::size_t>(tracked_node_cap_) && !make_room()) {
      report.tracking_saturated = true;
      return nullptr;
    }
    return &nodes_[fqn];
  }

  /// Frees a slot for a PRESENT node - see "A present node always wins a slot" in the class
  /// doc. Idle entries first (they carry nothing), then departed entries collapsed into
  /// per-code counts, keeping at most kMaxNamedDepartedEntries of them named; if even that
  /// leaves no room the named ones go too, because a present required node going unchecked
  /// is a worse outcome than a departed one losing its name. False when nothing could be
  /// freed, i.e. every entry is present and carrying evidence.
  bool make_room() {
    for (auto it = nodes_.begin(); it != nodes_.end();) {
      it = is_idle(it->second) ? nodes_.erase(it) : std::next(it);
    }
    if (nodes_.size() < static_cast<std::size_t>(tracked_node_cap_)) {
      return true;
    }
    collapse_departed(kMaxNamedDepartedEntries);
    if (nodes_.size() < static_cast<std::size_t>(tracked_node_cap_)) {
      return true;
    }
    collapse_departed(0);
    return nodes_.size() < static_cast<std::size_t>(tracked_node_cap_);
  }

  /// Collapse departed entries into per-code counts until at most `keep_named` of them are
  /// left in the map. Lexicographically LAST first, so the survivors are a stable prefix and
  /// the same graph always keeps the same names.
  void collapse_departed(std::size_t keep_named) {
    std::vector<std::string> departed;
    for (const auto & [fqn, node] : nodes_) {
      if (is_departed(node)) {
        departed.push_back(fqn);
      }
    }
    for (std::size_t i = departed.size(); i > keep_named; --i) {
      const auto it = nodes_.find(departed[i - 1]);
      count_collapsed(it->second);
      nodes_.erase(it);
    }
  }

  /// Fold one departed entry into the count for the code its clock was heading for. An
  /// unmeasured clock STILL CLIMBING counts under its cause rather than being dropped:
  /// absence advances it every tick and nothing can reset it any more, so it would have
  /// matured under that cause within a bounded number of ticks anyway, and dropping it
  /// instead would let freeing a slot heal a fault. A violation streak counts only once it
  /// has already matured (past `grace`): for a node the presence detector could also have
  /// reported (`ever_armed`), absence does not keep advancing a streak that has not yet
  /// matured (see the kInactive case in update()'s absence loop), so counting one that had
  /// not crossed `grace` would fabricate a violation the node never earned. The below-grace
  /// streak of a node the presence detector could never own is the one case where that is
  /// not true - absence keeps
  /// advancing it too, so left alone it would eventually mature the same way a climbing
  /// unmeasured clock does - but this tally still leaves it uncounted rather than folding
  /// it in: reaching this function with one still below `grace` needs `tracked_node_cap`
  /// genuinely saturated by departed identities, and losing that one entry's evidence is
  /// the safer direction to be incomplete in than risking a wrong count for every OTHER
  /// entry this function has to keep classifying correctly. An idle entry, or a
  /// live-but-unmatured streak, contributes nothing.
  void count_collapsed(const NodeState & node) {
    if (node.unmeasured_matured || node.unmeasured_clock > 0) {
      if (node.cause == LifecycleUnmeasuredCause::kUnreadable) {
        ++collapsed_unreadable_;
      } else {
        ++collapsed_not_managed_;
      }
    } else if (node.violation_streak > grace_) {
      ++collapsed_inactive_;
    }
  }

  /// Put a non-zero collapsed count into a code's content as one extra line. Keyed by a
  /// prefix that sorts below every node fqn (which always begins with '/') - see
  /// kCollapsedKeyPrefix for why that is the right end of the list.
  static void add_collapsed_content(std::map<std::string, std::string> & affected, int count, const char * left_as) {
    if (count <= 0) {
      return;
    }
    affected[std::string(kCollapsedKeyPrefix) + left_as] =
        trim_to("and " + std::to_string(count) + " more required node(s) left the graph " + left_as +
                    "; not named individually (tracked_node_cap is full)",
                kMaxLifecycleDetailChars);
  }

  static void append_all(std::vector<std::string> & target, const std::vector<std::string> & extra) {
    target.insert(target.end(), extra.begin(), extra.end());
  }

  /// One tick of the violation streak, recording the tick it CROSSES `grace_` on. Frozen
  /// one past the bound like every other clock here: past `grace_ + 1` the stored value
  /// only has to distinguish "confirmed" from "still climbing", never how far past, and a
  /// counter that only ever increments would eventually overflow on a node that is absent
  /// and violating for the life of a long-running process.
  void advance_violation_streak(NodeState & node, const std::string & fqn, std::set<std::string> & crossed) const {
    if (node.violation_streak > grace_) {
      return;
    }
    if (++node.violation_streak == grace_ + 1) {
      crossed.insert(fqn);
    }
  }

  /// One tick of the cause-blind unmeasured clock, taking ownership on the tick it matures.
  /// Taking it RELEASES the violation streak outright (see "Ownership is exclusive").
  void advance_unmeasured_clock(NodeState & node, const std::string & fqn, std::set<std::string> & crossed) const {
    if (node.unmeasured_matured) {
      return;  // frozen, and `cause` is STICKY from here - see the class doc
    }
    if (node.unmeasured_clock <= unmeasured_hold_ticks_) {
      ++node.unmeasured_clock;  // frozen one past the hold
    }
    if (node.unmeasured_clock == unmeasured_hold_ticks_ + 1) {
      node.unmeasured_matured = true;
      node.violation_streak = 0;  // RELEASED: GRAPH_NODE_INACTIVE's clear is now free
      crossed.insert(fqn);
    }
  }

  /// Detail phrase for a CONFIRMED violation. `departed` marks a node whose absence has
  /// outlived the blink tolerance: its streak is still climbing (absence continues a
  /// violation, it does not end one), but "is inactive" on its own would send an operator
  /// looking for a node that is no longer in the graph.
  static std::string inactive_detail(const std::string & fqn, const NodeState & node, bool departed) {
    std::string detail = "node " + fqn + " expected active but is " + node.last_label;
    if (departed) {
      detail += ", and has since left the graph";
    }
    return trim_to(detail + " (required by " + join_quoted(node.entries) + ")", kMaxLifecycleDetailChars);
  }

  /// Detail phrase for a matured unmeasured clock, under whichever cause froze at maturity.
  /// Carries no live label, so only the whole-detail backstop applies (R13's secondary
  /// guard) - the fqn and the "required by" list are the only unbounded parts.
  static std::string unmeasured_detail(const std::string & fqn, const NodeState & node, bool departed) {
    std::string detail = node.cause == LifecycleUnmeasuredCause::kUnreadable
                             ? "node " + fqn + " expected active but its lifecycle state could not be read"
                             : "node " + fqn + " expected active but is not a managed lifecycle node";
    if (departed) {
      detail += ", and has since left the graph";
    }
    return trim_to(detail + " (required by " + join_quoted(node.entries) + ")", kMaxLifecycleDetailChars);
  }

  /// Orders the four observed states by how much they tell the operator, so a duplicate
  /// match for one node keeps the MOST informative read: a confirmed violation is the
  /// strongest possible signal, an unmeasured read (either flavor) is a weaker "cannot
  /// tell" signal, and a healthy read is the least informative (nothing wrong to report).
  /// kUnreadable and kNotManaged rank equally - the unmeasured clock is deliberately blind
  /// to which of them it is seeing, so a tie between them is broken by iteration order,
  /// which never matters: duplicate matches for one node are the same physical app.id read
  /// twice, so in practice the two never disagree at all.
  static int observed_rank(LifecycleObservedState state) {
    switch (state) {
      case LifecycleObservedState::kInactive:
        return 3;
      case LifecycleObservedState::kUnreadable:
        return 2;
      case LifecycleObservedState::kNotManaged:
        return 1;
      case LifecycleObservedState::kActive:
        return 0;
    }
    return 0;  // unreachable: every enumerator is handled above
  }

  /// "'a', '/a'" - every entry that named the node, for a detail's context phrase.
  static std::string join_quoted(const std::vector<std::string> & entries) {
    std::string joined;
    for (const auto & entry : entries) {
      if (!joined.empty()) {
        joined += ", ";
      }
      joined += "'" + entry + "'";
    }
    return joined;
  }

  /// Key prefix for the collapsed-departed content line. '!' sorts below every character a
  /// node fqn can start with ('/'), so among the entries that are NOT crossing on this tick
  /// the count is the first thing the description spends its budget on. That is the right
  /// order: one line saying N nodes left the graph broken is worth more to an operator than
  /// the third name among them, and it cannot displace a node that just broke, because
  /// AggregatedFault::describe_ordered emits the caller's `order` list (the newly_* lists,
  /// present-first) ahead of any map order at all.
  static constexpr const char * kCollapsedKeyPrefix = "!departed: ";

  std::set<std::string> require_active_;
  int grace_;
  int absence_grace_;
  int no_match_warn_ticks_;
  int prune_ticks_;
  int unmeasured_hold_ticks_;
  int tracked_node_cap_;                     ///< clamped to at least 1 in the constructor
  std::map<std::string, NodeState> nodes_;   ///< fqn -> the whole per-node state machine
  std::map<std::string, int> no_match_;      ///< entry -> consecutive ticks matching nothing
  std::set<std::string> reported_no_match_;  ///< entries already warned about
  bool saturated_last_tick_ = false;         ///< for the saturation EDGE (see the report)
  // Departed entries folded into a count to free slots for present nodes. Monotone within
  // one tracker lifetime by design - see "What the operator sees when a count is non-zero".
  int collapsed_inactive_ = 0;
  int collapsed_unreadable_ = 0;
  int collapsed_not_managed_ = 0;
};

}  // namespace ros2_medkit_graph_watchdog
