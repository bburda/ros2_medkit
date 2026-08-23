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
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_graph_watchdog {

/// One sweep's verdict. `dead` maps every reported key to a human-readable detail phrase;
/// `keys_by_freshness` names the same keys ordered by MISS COUNT ascending - the most
/// recently departed key first, with the collapsed-count synthetic entry (see
/// NodeLivenessTracker::kCollapsedKey) always first of all when present.
///
/// The ordering matters because the description built from `dead` is capped
/// (AggregatedFault::kMaxDescriptionChars) while `dead` itself is a std::map and so walks
/// lexicographically. On a graph carrying several long-dead entries, a fresh death whose
/// key sorts late alphabetically would otherwise be cut from the text - and, being fresh,
/// it is also the one thing the operator has not already been told. Freshest-first is what
/// keeps a capped description still naming the thing that just happened.
struct NodeDeathReport {
  std::map<std::string, std::string> dead;
  std::vector<std::string> keys_by_freshness;
  /// LEVEL: true on every tick the DEPARTED (misses_ > 0) subset of the tracked map still
  /// exceeds `tracked_key_cap` after collapsing every entry it safely could - i.e. entries
  /// still mid-grace, which collapsing must never touch (see collapse_matured()'s own doc),
  /// are alone enough to keep it over the cap. Stays true for as long as the condition
  /// lasts. NOT the same condition LifecycleExpectationReport::tracking_saturated names for
  /// the sibling tracker: an ARMED/PRESENT key is never refused here (see the class doc), so
  /// this can no longer mean "a newly-armed key went unwatched."
  bool tracking_saturated = false;
  /// EDGE: the first tick of a saturation episode, for a caller's one-time warning. Re-arms
  /// when the episode ends.
  bool saturation_started = false;
};

/// Pure presence/absence state machine for "armed nodes that vanish."
///
/// Each call to update() is handed two sets over the SAME key space:
///   present - every key visible in this tick's graph snapshot (the liveness signal)
///   armed   - the subset whose departure the reliability gate says this detector OWNS: it
///             allows a raise for them AND their lifecycle state is either known not to be a
///             managed-non-active one or has been asked for as often as it ever will be
///             (ReliabilityGate::presence_ownership), so a managed node whose state is
///             unread but still being asked for is deliberately not in this set
///
/// A key becomes TRACKED (added to `known_`) the first time it is armed, and stays tracked
/// from then on regardless of its later arm state - so a node that is present but not
/// currently armed (still warming up, or lifecycle-inactive) is never mistaken for dead:
/// presence alone keeps a tracked key's miss counter at zero. A key is reported dead once
/// its consecutive-miss counter exceeds `miss_grace`.
///
/// That stickiness is the right default for a key admitted on knowledge, and the wrong one
/// for a key admitted only because nothing was known about it - see PresenceOwnership.
/// release() is how the caller undoes the second kind, and it is the caller's job to know
/// which kind it granted: this class is handed sets, not grounds.
///
/// update() never removes a key BY AGE. An unsuppressed, still-dead entry has to stay
/// reported for as long as it is actually dead - the alternative is a detector that quietly
/// stops saying so once enough time has passed, which is the one failure mode this exists
/// to rule out. prune() reclaims a key only once it has been DURABLY suppressed for long
/// enough (see its own doc below); ordinary, unsuppressed churn - unique identities that
/// arm once and are never seen again, which this detector's zero-config "every armed App is
/// a candidate" scope makes the common case on any fleet with per-run or per-namespace
/// names - has no age-based or suppression-based removal path at all. `tracked_key_cap` is
/// what bounds the map against exactly that growth.
///
/// **What the cap bounds, and why it differs from its own past shape.** The unbounded
/// growth is entirely in DEPARTED keys: a dead node's entry is never
/// reclaimed by anything but a durable suppressor, so unique dead identities accumulate for
/// the life of the process. A PRESENT key does not have that problem - its count at any
/// instant is bounded by the live graph, which is bounded by reality (a large single-robot
/// graph runs to roughly a hundred nodes, a ten-robot fleet to a few hundred - see
/// kDefaultTrackedKeyCap) - and it carries no state worth reclaiming anyway (misses_ == 0).
/// So the cap here bounds only the DEPARTED subset (misses_ > 0): admission of an
/// ARMED/PRESENT key is therefore NEVER refused and never evicts anything, at any map size.
/// A tracked_key_cap smaller than the live graph is consequently not saturation - the extra
/// present entries simply coexist with a full departed set - only a departed set that stays
/// oversized even after every eligible collapse is.
///
/// Collapsing (folding a departed identity into the one collapsed_dead_count_, see
/// collapse_matured()) may ONLY apply to an entry that has ACTUALLY crossed `miss_grace_` -
/// a confirmed death. Collapsing one that is merely mid-grace would report a death the node
/// has not earned, permanently (the identity is erased, so the node returning cannot un-ring
/// it) - the exact fabrication an earlier shape of this cap produced. An immature departed
/// entry is therefore never evicted by the cap either: it is kept, tracked exactly like any
/// other departed entry, until it either matures (and becomes a collapse candidate) or the
/// node returns (and the entry goes idle again). Under sustained pressure from still-maturing
/// churn the departed set may consequently exceed tracked_key_cap_ for a few ticks - bounded
/// by recent churn rate times miss_grace_, not by how long the process has been running,
/// which is the growth this cap exists to prevent in the first place.
///
/// Why this differs from LifecycleExpectationTracker's make_room(), despite looking
/// parallel: that tracker keeps TWO clocks per key (a violation streak and an unmeasured
/// clock), so one node's entry can be PRESENT and carrying evidence at the same time - that
/// third state is what makes its idle/collapsible/present-and-evidential three-way eviction
/// order meaningful, and is also why IT can evict a present entry without losing anything
/// (the evidence lives in the clocks, not in presence). This tracker keeps exactly one piece
/// of per-key state (`misses_`), so a key is always either present-and-empty (idle) or
/// absent-and-evidential (departed) - there is no third state, and evicting a present entry
/// here would only ever throw away a key that could simply be re-admitted next tick anyway,
/// at the cost of losing any death that happens to land in the eviction window (only ONLINE
/// nodes re-enter `armed`, so an evicted-while-present node that then dies is never
/// re-admitted at all). The sibling's shape does not transfer.
class NodeLivenessTracker {
 public:
  /// Sentinel `prune_ticks` meaning "never reclaim anything." The default for callers that
  /// only care about update()'s presence/absence bookkeeping (short-lived tests, mostly);
  /// real wiring (NodeDeathDetector::configure()) always passes an explicit, clamped value.
  static constexpr int kNoPrune = std::numeric_limits<int>::max();

  /// Default `tracked_key_cap` - same value, for the same reason, as
  /// LifecycleExpectationTracker::kDefaultTrackedNodeCap: a large single-robot ROS 2 graph
  /// runs to roughly a hundred nodes, a ten-robot fleet sharing one domain to a few hundred,
  /// so even an operator whose graph is entirely in scope (every App here, versus only
  /// require_active's named subset there) stays comfortably under it at a cost of a few
  /// hundred KB. Growth past it can only come from DEPARTED identity churn, which is exactly
  /// what the cap exists to bound - a present key never counts against it at all (see the
  /// class doc).
  static constexpr int kDefaultTrackedKeyCap = 512;

  /// Most DEPARTED, matured (confirmed dead) entries kept individually named when the
  /// departed set has to be shrunk; the rest are collapsed into the one count. Mirrors
  /// LifecycleExpectationTracker::kMaxNamedDepartedEntries and its reasoning: past three, a
  /// name is pure cost (AggregatedFault::kMaxDescriptionChars cannot fit a fourth alongside
  /// the freshest entries this tracker already prioritises).
  static constexpr int kMaxNamedDepartedEntries = 3;

  /// Key for the collapsed-departed synthetic report entry. '!' sorts below every character
  /// a real key can start with here ('/'), matching LifecycleExpectationTracker's identical
  /// convention - though this class additionally puts it FIRST in keys_by_freshness itself
  /// (see update()) rather than relying on describe_ordered()'s map-order fallback: unlike
  /// the sibling's `newly_*` lists (only entries crossing THIS tick), keys_by_freshness here
  /// already names every CURRENTLY dead key every tick, so leaving the collapsed entry out
  /// of it would place it last, not first, and a capped description could cut the one line
  /// that tells the operator identities are being lost to capacity pressure at all.
  static constexpr const char * kCollapsedKey = "!collapsed";

  explicit NodeLivenessTracker(int miss_grace, int prune_ticks = kNoPrune, int tracked_key_cap = kDefaultTrackedKeyCap)
    : miss_grace_(miss_grace), prune_ticks_(prune_ticks), tracked_key_cap_(tracked_key_cap < 1 ? 1 : tracked_key_cap) {
  }

  /// Undo the admission of a key that is still PRESENT and whose ground for being tracked
  /// has been withdrawn: a node admitted on PresenceOwnership::kProvisional whose lifecycle
  /// label finally arrived and answered kDisowned, so the graph has said the node belongs to
  /// another detector. Those are the only two grounds involved - kUnclaimed is not one of
  /// them, because it says nothing has been measured, and a node that is merely re-warming
  /// after a restart reads exactly like that.
  ///
  /// Only ever call this for a key the caller can see in this tick's snapshot. A present key
  /// carries `misses_ == 0`, so dropping it loses no evidence: it is exactly the state a key
  /// would be in if it had never been admitted at all. Calling it for an ABSENT key would be
  /// a different act entirely - erasing an in-flight or already-matured death - and this
  /// class deliberately offers no way to do that by age (see the class doc); prune() is the
  /// one path that reclaims a departed key, and it demands a durable suppressor's veto first.
  void release(const std::string & key) {
    known_.erase(key);
    misses_.erase(key);
    suppressed_streak_.erase(key);
  }

  NodeDeathReport update(const std::set<std::string> & present, const std::set<std::string> & armed) {
    NodeDeathReport report;
    // Unconditional: a PRESENT/armed key is never refused a slot and never costs an
    // existing entry its own - see the class doc for why this differs from the sibling.
    for (const auto & key : armed) {
      known_.insert(key);  // no-op if already tracked
    }

    for (const auto & key : known_) {
      if (present.count(key) > 0) {
        misses_[key] = 0;
      } else {
        ++misses_[key];
      }
    }

    // Bounds the DEPARTED subset only, and only by collapsing entries that have ACTUALLY
    // matured - never a present or still-immature one. Runs BEFORE the report below is
    // built, so a just-collapsed identity never simultaneously appears both individually and
    // inside the collapsed count.
    report.tracking_saturated = enforce_departed_cap();

    std::vector<std::pair<int, std::string>> by_miss_count;
    for (const auto & key : known_) {
      const int misses = misses_.at(key);
      if (misses > miss_grace_) {
        report.dead[key] = "node " + key + " disappeared (" + std::to_string(misses) + " missed cycles)";
        by_miss_count.emplace_back(misses, key);
      }
    }
    // Fewest misses first (freshest death first); the key itself breaks ties so the
    // ordering - and so the capped description - does not reshuffle tick to tick on its
    // own.
    std::sort(by_miss_count.begin(), by_miss_count.end());
    report.keys_by_freshness.reserve(by_miss_count.size() + 1);
    // See kCollapsedKey's own doc for why this goes first rather than through
    // describe_ordered()'s map-order fallback.
    if (collapsed_dead_count_ > 0) {
      report.dead[kCollapsedKey] = "and " + std::to_string(collapsed_dead_count_) +
                                   " more node(s) disappeared; not named individually (tracked_key_cap is full)";
      report.keys_by_freshness.emplace_back(kCollapsedKey);
    }
    for (auto & entry : by_miss_count) {
      report.keys_by_freshness.push_back(std::move(entry.second));
    }
    report.saturation_started = report.tracking_saturated && !saturated_last_tick_;
    saturated_last_tick_ = report.tracking_saturated;
    return report;
  }

  /// Reclaim bookkeeping for a key in `suppressed` once it has been suppressed on more than
  /// `prune_ticks_` CONSECUTIVE calls. Call this after update() each tick, with the set of
  /// keys a detector's DURABLE suppressors currently vote to suppress - never the raw
  /// "removed from this tick's report" set, and never a non-durable suppressor's verdict
  /// (see suppressor.hpp's own doc on why only a durable veto makes reclaiming sound).
  ///
  /// A key missing from `suppressed` has its streak reset to zero on this same call - not
  /// merely left alone - so a veto that lifts even once starts the count over rather than
  /// merely pausing it. A key that is never suppressed therefore has streak zero forever
  /// and can never be reclaimed no matter how long it stays dead: this asymmetry is what
  /// makes an unsuppressed death permanent-until-acknowledged while still letting a
  /// permanently-vetoed one stop costing memory.
  void prune(const std::set<std::string> & suppressed) {
    for (auto it = known_.begin(); it != known_.end();) {
      const auto & key = *it;
      int & streak = suppressed_streak_[key];
      streak = suppressed.count(key) > 0 ? streak + 1 : 0;
      if (streak > prune_ticks_) {
        suppressed_streak_.erase(key);
        misses_.erase(key);
        it = known_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /// How many keys are currently tracked (known_/misses_ size) - a test seam so a suite can
  /// assert the map stays bounded under churn without exposing the map itself. Never
  /// counts collapsed-departed identities: they no longer have one to count. Includes
  /// present entries, which are not bounded by tracked_key_cap_ at all (see the class doc).
  std::size_t tracked_count() const {
    return known_.size();
  }

  /// The keys currently tracked. A read-only view, not a copy of any mutable state a caller
  /// could corrupt - used by a detector that keeps its OWN per-key bookkeeping alongside
  /// this tracker's (e.g. a remembered App::id for allowlist matching after a key dies) and
  /// needs to reclaim it in step with prune() without this class exposing prune()'s own
  /// internal streak bookkeeping to do it.
  const std::set<std::string> & known_keys() const {
    return known_;
  }

 private:
  /// How many known_ keys currently carry evidence (misses_ > 0) - present (idle) entries
  /// never count. What tracked_key_cap_ actually bounds; see the class doc.
  std::size_t departed_count() const {
    std::size_t n = 0;
    for (const auto & [key, misses] : misses_) {
      (void)key;
      if (misses > 0) {
        ++n;
      }
    }
    return n;
  }

  /// Shrinks the DEPARTED subset of known_ to at most tracked_key_cap_ where it safely can,
  /// by collapsing MATURED entries (misses_ > miss_grace_) - never a present or immature
  /// one; see the class doc for why. Two stages, mirroring
  /// LifecycleExpectationTracker::make_room()'s identical shape for the same reason: first
  /// collapse down to kMaxNamedDepartedEntries (keeping a few individually named when that
  /// alone is enough to clear the cap), and only if the departed set is STILL over cap -
  /// meaning immature entries alone already account for the excess - collapse the rest of
  /// the matured ones too, since every one collapsed still reduces the pressure even when it
  /// cannot fully relieve it.
  ///
  /// Returns true when the departed set remains over tracked_key_cap_ even after collapsing
  /// every matured entry available - i.e. immature (still mid-grace) entries alone exceed
  /// the cap. That condition is left standing rather than forced down: an immature entry is
  /// never a collapse candidate, so there is nothing further this function may safely do
  /// about it.
  bool enforce_departed_cap() {
    if (departed_count() <= static_cast<std::size_t>(tracked_key_cap_)) {
      return false;
    }
    collapse_matured(kMaxNamedDepartedEntries);
    if (departed_count() <= static_cast<std::size_t>(tracked_key_cap_)) {
      return false;
    }
    collapse_matured(0);
    return departed_count() > static_cast<std::size_t>(tracked_key_cap_);
  }

  /// Fold MATURED departed entries (misses_ > miss_grace_) into collapsed_dead_count_ until
  /// at most `keep_named` remain individually tracked. NEVER touches an immature departed
  /// entry (0 < misses_ <= miss_grace_) or a present one - collapsing either would report
  /// (or lose) a death the node has not actually earned yet. Lexicographically LAST first,
  /// so the survivors are a stable prefix and the same graph always keeps the same names -
  /// mirrors LifecycleExpectationTracker::collapse_departed's identical choice.
  void collapse_matured(std::size_t keep_named) {
    std::vector<std::string> matured;
    for (const auto & [key, misses] : misses_) {
      if (misses > miss_grace_) {
        matured.push_back(key);
      }
    }
    for (std::size_t i = matured.size(); i > keep_named; --i) {
      const std::string & key = matured[i - 1];
      ++collapsed_dead_count_;
      known_.erase(key);
      misses_.erase(key);
      suppressed_streak_.erase(key);
    }
  }

  int miss_grace_;
  int prune_ticks_;
  int tracked_key_cap_;  ///< clamped to at least 1 in the constructor; bounds departed_count() only
  std::set<std::string> known_;
  std::map<std::string, int> misses_;
  std::map<std::string, int> suppressed_streak_;
  bool saturated_last_tick_ = false;  ///< for the saturation EDGE (see NodeDeathReport)
  /// Departed, MATURED entries folded into a count to shrink the departed set back toward
  /// tracked_key_cap_. Monotone within one tracker lifetime by design, mirroring the
  /// sibling: a gateway restart or a detector reconfigure both replace this object wholesale
  /// (see node_death_detector.cpp's own configure()), which is the only thing that ever
  /// resets it. A collapsed identity returning does not decrement it - the identity itself
  /// is gone, so there is no way to tell which of possibly-several collapsed departures came
  /// back - the aggregate fault this count keeps raised clears only via a reconfigure (which
  /// also re-baselines whatever is present at that point) or an operator's explicit
  /// acknowledgement.
  int collapsed_dead_count_ = 0;
};

}  // namespace ros2_medkit_graph_watchdog
