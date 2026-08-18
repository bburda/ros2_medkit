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
  /// LEVEL: true on every tick a newly-armed key had to be refused because
  /// `tracked_key_cap` is full of present keys all carrying evidence, and nothing could be
  /// reclaimed or collapsed. Stays true for as long as the condition lasts - a refused key
  /// is still armed on the following tick and refused again. Mirrors
  /// LifecycleExpectationReport::tracking_saturated.
  bool tracking_saturated = false;
  /// EDGE: the first tick of a saturation episode, for a caller's one-time warning. Re-arms
  /// when the episode ends.
  bool saturation_started = false;
};

/// Pure presence/absence state machine for "armed nodes that vanish."
///
/// Each call to update() is handed two sets over the SAME key space:
///   present - every key visible in this tick's graph snapshot (the liveness signal)
///   armed   - the subset the reliability gate currently allows a raise for
///
/// A key becomes TRACKED (added to `known_`) the first time it is armed, and stays tracked
/// from then on regardless of its later arm state - so a node that is present but not
/// currently armed (still warming up, or lifecycle-inactive) is never mistaken for dead:
/// presence alone keeps a tracked key's miss counter at zero. A key is reported dead once
/// its consecutive-miss counter exceeds `miss_grace`.
///
/// update() never removes a key BY AGE. An unsuppressed, still-dead entry has to stay
/// reported for as long as it is actually dead - the alternative is a detector that quietly
/// stops saying so once enough time has passed, which is the one failure mode this exists
/// to rule out. prune() reclaims a key only once it has been DURABLY suppressed for long
/// enough (see its own doc below); ordinary, unsuppressed churn - unique identities that
/// arm once and are never seen again, which this detector's zero-config "every armed App is
/// a candidate" scope makes the common case on any fleet with per-run or per-namespace
/// names - has no age-based or suppression-based removal path at all. `tracked_key_cap` is
/// what bounds the map against exactly that growth; see make_room()'s own doc for the
/// eviction order, deliberately the same one LifecycleExpectationTracker uses for its own
/// `tracked_node_cap`: idle entries first (nothing to lose), then departed entries collapsed
/// into a count that still keeps the fault raised, and a present key never evicted to make
/// room for another - refusing the newcomer instead, with `tracking_saturated` saying so, is
/// the safe direction, because a map full of the dead that refuses a genuinely broken
/// PRESENT node is the exact failure a presence detector exists to prevent. This detector
/// tracks every armed App - a strictly larger set than a require_active list - so the bound
/// matters even more here than it does for the sibling it is mirrored from.
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
  /// hundred KB. Growth past it can only come from identity CHURN, which is exactly what
  /// the cap exists to bound.
  static constexpr int kDefaultTrackedKeyCap = 512;

  /// Most DEPARTED entries kept individually named when the cap has to make room for a
  /// present key; the rest are collapsed into the one count. Mirrors
  /// LifecycleExpectationTracker::kMaxNamedDepartedEntries and its reasoning: past three, a
  /// name is pure cost (AggregatedFault::kMaxDescriptionChars cannot fit a fourth alongside
  /// the freshest entries this tracker already prioritises), and the cost is a slot a
  /// PRESENT key needs.
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

  NodeDeathReport update(const std::set<std::string> & present, const std::set<std::string> & armed) {
    NodeDeathReport report;
    for (const auto & key : armed) {
      if (known_.count(key) > 0) {
        continue;  // already tracked - re-affirms nothing, admission already happened
      }
      if (known_.size() >= static_cast<std::size_t>(tracked_key_cap_) && !make_room()) {
        report.tracking_saturated = true;
        continue;  // refused this tick; armed again next tick, so it is retried, not lost
      }
      known_.insert(key);
    }
    report.saturation_started = report.tracking_saturated && !saturated_last_tick_;
    saturated_last_tick_ = report.tracking_saturated;

    std::vector<std::pair<int, std::string>> by_miss_count;
    for (const auto & key : known_) {
      if (present.count(key) > 0) {
        misses_[key] = 0;
        continue;
      }
      const int misses = ++misses_[key];
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
  /// counts collapsed-departed identities: they no longer have one to count.
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
  /// An entry with nothing to lose: currently present, so its miss streak is zero. (A
  /// present key's suppressed_streak_ is always zero too - prune()'s own suppressed set is
  /// built by the caller only from keys that were candidates for report.dead, and presence
  /// resets misses_ before that candidacy could ever arise - so misses_ alone decides.)
  /// Evicting one loses nothing: if it is still armed on a later tick it is simply
  /// re-tracked, identical to never having been evicted at all.
  bool is_idle(const std::string & key) const {
    auto it = misses_.find(key);
    return it == misses_.end() || it->second == 0;
  }

  /// Frees a slot for a newly-armed key. Idle entries first (see is_idle()); then departed
  /// entries (misses_ > 0 - currently missing, carrying SOME evidence, confirmed past
  /// miss_grace_ or not) are collapsed into kCollapsedKey's one count, keeping at most
  /// kMaxNamedDepartedEntries of them individually named; if even that leaves no room every
  /// departed entry is collapsed.
  ///
  /// Returns bool, mirroring LifecycleExpectationTracker::make_room()'s identical contract
  /// ("false means refuse the newcomer, never evict a key still needed"), but for THIS
  /// tracker every tracked key is, at the instant this runs, either idle (present, so
  /// step 1 frees it - and if it is still armed it is simply re-tracked a moment later
  /// in the very same update(), indistinguishable from never having been evicted) or
  /// departed (step 2 collapses it). There is no third state - unlike the sibling, whose
  /// clocks let a node be simultaneously present and mid-violation, hence neither idle nor
  /// departed and so un-evictable - so collapsing every departed entry (keep_named=0)
  /// always empties known_ and this function cannot actually return false while
  /// tracked_key_cap_ >= 1 (guaranteed by the constructor). Kept returning bool anyway:
  /// the contract is what the caller (update()) relies on, a future key state that DOES
  /// carry present-and-unevictable evidence must not have to rediscover this shape, and
  /// NodeLivenessTrackerCap's own SaturationNeverFiresBecauseEveryKeyIsEitherIdleOrCollapsible
  /// test pins the current, narrower reality rather than leaving it undocumented.
  ///
  /// A departed entry collapsed here is not necessarily one this tick's caller will end up
  /// reporting unsuppressed: suppression is decided by the caller AFTER update() returns,
  /// so under simultaneous cap exhaustion and a suppressible departure among the collapse
  /// victims, kCollapsedKey's count can count an identity the caller would otherwise have
  /// silently suppressed. Deliberate, not overlooked: this tracker is pure presence/absence
  /// bookkeeping with no knowledge of suppression (see the class doc), the cap being full at
  /// all is itself the operator-visible condition to act on, and erring toward reporting one
  /// extra is the same direction every other tracker choice in this package already errs -
  /// silently losing a genuine death is the failure mode collapsing exists to prevent, and a
  /// narrow, cap-exhaustion-only overcount is a far smaller cost than that.
  bool make_room() {
    for (auto it = known_.begin(); it != known_.end();) {
      if (is_idle(*it)) {
        const std::string key = *it;
        it = known_.erase(it);
        misses_.erase(key);
        suppressed_streak_.erase(key);
      } else {
        ++it;
      }
    }
    if (known_.size() < static_cast<std::size_t>(tracked_key_cap_)) {
      return true;
    }
    collapse_departed(kMaxNamedDepartedEntries);
    if (known_.size() < static_cast<std::size_t>(tracked_key_cap_)) {
      return true;
    }
    collapse_departed(0);
    return known_.size() < static_cast<std::size_t>(tracked_key_cap_);
  }

  /// Collapse departed entries (misses_ > 0) into collapsed_dead_count_ until at most
  /// `keep_named` remain individually tracked. Lexicographically LAST first, so the
  /// survivors are a stable prefix and the same graph always keeps the same names - mirrors
  /// LifecycleExpectationTracker::collapse_departed's identical choice.
  void collapse_departed(std::size_t keep_named) {
    std::vector<std::string> departed;
    for (const auto & key : known_) {
      if (!is_idle(key)) {
        departed.push_back(key);
      }
    }
    for (std::size_t i = departed.size(); i > keep_named; --i) {
      const std::string & key = departed[i - 1];
      ++collapsed_dead_count_;
      known_.erase(key);
      misses_.erase(key);
      suppressed_streak_.erase(key);
    }
  }

  int miss_grace_;
  int prune_ticks_;
  int tracked_key_cap_;  ///< clamped to at least 1 in the constructor
  std::set<std::string> known_;
  std::map<std::string, int> misses_;
  std::map<std::string, int> suppressed_streak_;
  bool saturated_last_tick_ = false;  ///< for the saturation EDGE (see NodeDeathReport)
  /// Departed entries folded into a count to free slots for present keys. Monotone within
  /// one tracker lifetime by design, mirroring the sibling: a gateway restart or a detector
  /// reconfigure both replace this object wholesale (see node_death_detector.cpp's own
  /// configure()), which is the only thing that ever resets it.
  int collapsed_dead_count_ = 0;
};

}  // namespace ros2_medkit_graph_watchdog
