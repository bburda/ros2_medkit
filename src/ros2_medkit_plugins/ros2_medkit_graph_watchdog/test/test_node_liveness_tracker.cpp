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
//
// Pure logic: no rclcpp::Node, no rclcpp::init() anywhere in this file - both
// NodeLivenessTracker and AggregatedFault::describe_ordered() are ROS-free.
#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/node_liveness_tracker.hpp"

using ros2_medkit_graph_watchdog::AggregatedFault;
using ros2_medkit_graph_watchdog::NodeLivenessTracker;

// ---- update(): the presence/absence state machine ----------------------------------------

TEST(NodeLivenessTrackerUpdate, PresentButNeverArmedIsNeverTracked) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  auto report = tracker.update({"/a"}, /*armed=*/{});
  EXPECT_TRUE(report.dead.empty());
  EXPECT_EQ(tracker.tracked_count(), 0u);
}

TEST(NodeLivenessTrackerUpdate, OnceArmedPresenceAloneKeepsItAlive) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  tracker.update({"/a"}, {"/a"});            // arms it
  auto report = tracker.update({"/a"}, {});  // still present; arm state no longer matters
  EXPECT_TRUE(report.dead.empty());
  EXPECT_EQ(tracker.tracked_count(), 1u);
}

TEST(NodeLivenessTrackerUpdate, ArmedThenAbsentIsReportedOnlyPastMissGrace) {
  NodeLivenessTracker tracker(/*miss_grace=*/2);
  tracker.update({"/a"}, {"/a"});
  EXPECT_TRUE(tracker.update({}, {}).dead.empty()) << "miss 1";
  EXPECT_TRUE(tracker.update({}, {}).dead.empty()) << "miss 2 == miss_grace, not yet";
  auto report = tracker.update({}, {});  // miss 3 > miss_grace
  ASSERT_EQ(report.dead.count("/a"), 1u);
  EXPECT_NE(report.dead.at("/a").find("/a"), std::string::npos) << "the detail must name the key";
}

TEST(NodeLivenessTrackerUpdate, ReturningAfterBeingReportedDeadClearsIt) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  tracker.update({"/a"}, {"/a"});
  ASSERT_EQ(tracker.update({}, {}).dead.count("/a"), 1u);
  auto report = tracker.update({"/a"}, {});  // it came back
  EXPECT_TRUE(report.dead.empty());
}

TEST(NodeLivenessTrackerUpdate, AnUnsuppressedDeathIsNeverForgottenWithoutPrune) {
  // The anti-false-heal guarantee update() alone provides: a dead, never-pruned key stays
  // reported however many further ticks pass.
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  tracker.update({"/a"}, {"/a"});
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(tracker.update({}, {}).dead.count("/a"), 1u) << "tick " << i;
  }
  EXPECT_EQ(tracker.tracked_count(), 1u);
}

TEST(NodeLivenessTrackerUpdate, KeysByFreshnessOrdersTheMostRecentDeathFirst) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  tracker.update({"/old", "/new"}, {"/old", "/new"});
  tracker.update({"/new"}, {});          // /old dies now (miss 1); /new stays present
  auto report = tracker.update({}, {});  // /old miss 2; /new miss 1 (just died - freshest)
  ASSERT_EQ(report.dead.size(), 2u);
  ASSERT_EQ(report.keys_by_freshness.size(), 2u);
  EXPECT_EQ(report.keys_by_freshness.front(), "/new");
  EXPECT_EQ(report.keys_by_freshness.back(), "/old");
}

TEST(NodeLivenessTrackerUpdate, TwoIdsWithTheSameMissCountBreakTiesByKey) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  tracker.update({"/b", "/a"}, {"/b", "/a"});
  auto report = tracker.update({}, {});  // both die on the same tick, same miss count
  ASSERT_EQ(report.keys_by_freshness.size(), 2u);
  EXPECT_EQ(report.keys_by_freshness.front(), "/a") << "ties break lexicographically, deterministically";
}

// ---- prune(): S5 - only a veto that never lifts may safely reclaim -----------------------

TEST(NodeLivenessTrackerPrune, APermanentlySuppressedKeyIsReclaimedOnceItsStreakPassesPruneTicks) {
  NodeLivenessTracker tracker(/*miss_grace=*/0, /*prune_ticks=*/2);
  tracker.update({}, {"/durable"});
  ASSERT_EQ(tracker.update({}, {}).dead.count("/durable"), 1u);
  ASSERT_EQ(tracker.tracked_count(), 1u);

  tracker.prune({"/durable"});  // streak 1
  EXPECT_EQ(tracker.tracked_count(), 1u);
  tracker.prune({"/durable"});  // streak 2 == prune_ticks: not yet past it
  EXPECT_EQ(tracker.tracked_count(), 1u);
  tracker.prune({"/durable"});  // streak 3 > prune_ticks(2): reclaimed
  EXPECT_EQ(tracker.tracked_count(), 0u);
}

TEST(NodeLivenessTrackerPrune, AVetoThatLiftsEvenOnceResetsTheStreakAndDelaysReclaim) {
  // This is the property that makes it SAFE for a detector to feed prune() only its
  // durable suppressors' verdicts: a condition-based (non-durable) veto can stop matching
  // on any tick, and when it does, the streak must restart from zero rather than merely
  // pause - or a key could be reclaimed by ACCUMULATED, non-consecutive suppressed ticks
  // even though the condition was false in between, which is exactly the false-heal
  // suppressor.hpp's durable() doc warns against.
  NodeLivenessTracker tracker(/*miss_grace=*/0, /*prune_ticks=*/2);
  tracker.update({}, {"/lifts"});
  ASSERT_EQ(tracker.update({}, {}).dead.count("/lifts"), 1u);

  tracker.prune({"/lifts"});  // streak 1
  tracker.prune({});          // the veto lifts for one call -> streak resets to 0
  tracker.prune({"/lifts"});  // streak 1 again
  tracker.prune({"/lifts"});  // streak 2
  EXPECT_EQ(tracker.tracked_count(), 1u)
      << "two consecutive suppressed calls after a lift must not equal three from a permanent veto";
  tracker.prune({"/lifts"});  // streak 3 > prune_ticks(2): only now reclaimed
  EXPECT_EQ(tracker.tracked_count(), 0u);
}

TEST(NodeLivenessTrackerPrune, AnUnsuppressedDeathIsNeverReclaimedNoMatterHowManyPruneCalls) {
  NodeLivenessTracker tracker(/*miss_grace=*/0, /*prune_ticks=*/2);
  tracker.update({}, {"/never_suppressed"});
  ASSERT_EQ(tracker.update({}, {}).dead.count("/never_suppressed"), 1u);
  for (int i = 0; i < 20; ++i) {
    tracker.prune({});  // nothing is ever suppressed
  }
  EXPECT_EQ(tracker.tracked_count(), 1u);
}

TEST(NodeLivenessTrackerPrune, PruneNeverErasesAKeyFromAReportAlreadyHandedBack) {
  // Every real caller runs prune() strictly AFTER update() within the same tick - this
  // pins that pruning can never retroactively take back a key this tick's own report
  // already earned.
  NodeLivenessTracker tracker(/*miss_grace=*/0, /*prune_ticks=*/0);
  tracker.update({}, {"/a"});
  auto report = tracker.update({}, {});
  ASSERT_EQ(report.dead.count("/a"), 1u);
  tracker.prune({"/a"});  // reclaims it for the NEXT tick, not this one
  EXPECT_EQ(report.dead.count("/a"), 1u);
}

// ---- D1: the capped description survives a fresh entry among many stale ones -------------

TEST(NodeLivenessTrackerFreshness, AFreshDeathSurvivesTheDescriptionCapAmongManyStaleOnes) {
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  std::set<std::string> stale_ids;
  // Ten long, alphabetically-EARLY ids already exceed kMaxDescriptionChars by themselves -
  // proves the freshness ordering actually matters, not merely that a short text happens
  // to fit regardless of order.
  for (int i = 0; i < 10; ++i) {
    stale_ids.insert("/aaa_stale_node_padded_to_be_long_enough_" + std::to_string(1000 + i));
  }
  // Armed and immediately dead: absent from `present` on the very tick they are armed
  // already counts as one miss, which exceeds miss_grace(0).
  tracker.update({}, stale_ids);

  // A fresh id that sorts alphabetically LAST, armed a tick later while still present (so
  // it is not dead yet), then departs on the tick after - the genuinely most recent death.
  const std::string fresh_id = "/zzz_fresh_node";
  tracker.update({fresh_id}, {fresh_id});
  auto report = tracker.update({}, {});
  ASSERT_EQ(report.dead.count(fresh_id), 1u);
  ASSERT_FALSE(report.keys_by_freshness.empty());
  EXPECT_EQ(report.keys_by_freshness.front(), fresh_id);

  const std::string description = AggregatedFault::describe_ordered(report.dead, report.keys_by_freshness);
  EXPECT_LE(description.size(), AggregatedFault::kMaxDescriptionChars);
  EXPECT_NE(description.find(fresh_id), std::string::npos)
      << "a fresh death must survive the capped description even though it sorts last "
         "alphabetically among the stale entries";
}

TEST(NodeLivenessTrackerFreshness, PlainLexicographicOrderWouldHaveCutTheFreshEntry) {
  // The control case for the test above: build the SAME description from `dead` alone (the
  // uncapped, alphabetical helper's own default order), proving the fresh id really would
  // have been lost without keys_by_freshness - this is not a description that happens to
  // fit either way.
  NodeLivenessTracker tracker(/*miss_grace=*/0);
  std::set<std::string> stale_ids;
  for (int i = 0; i < 10; ++i) {
    stale_ids.insert("/aaa_stale_node_padded_to_be_long_enough_" + std::to_string(1000 + i));
  }
  tracker.update({}, stale_ids);
  const std::string fresh_id = "/zzz_fresh_node";
  tracker.update({fresh_id}, {fresh_id});
  auto report = tracker.update({}, {});

  const std::string lexicographic = AggregatedFault::describe(report.dead);
  EXPECT_EQ(lexicographic.find(fresh_id), std::string::npos)
      << "this pins that the freshness ordering is load-bearing: without it, the "
         "alphabetically-last fresh id is the one the cap would have cut";
}

// ---- tracked_key_cap: the bound that keeps the DEPARTED subset of known_/misses_/
// suppressed_streak_ from growing without limit under identity churn. A present entry never
// counts against it and is never evicted to make room - see node_liveness_tracker.hpp's own
// class doc for why this tracker's cap does not mirror LifecycleExpectationTracker's.

TEST(NodeLivenessTrackerCap, TrackedCountNeverExceedsTheConfiguredCapUnderChurn) {
  NodeLivenessTracker tracker(/*miss_grace=*/0, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/5);
  std::size_t max_seen = 0;
  // 200 distinct, never-repeating identities, never present again once armed - every one
  // instantly departed (miss_grace=0), so this exercises exactly the DEPARTED-subset cap at a
  // scale that pushes past it, with no present entries in the mix to blur the measurement.
  for (int i = 0; i < 200; ++i) {
    tracker.update({}, {"/churn_" + std::to_string(i)});
    max_seen = std::max(max_seen, tracker.tracked_count());
  }
  EXPECT_LE(max_seen, 5u) << "the cap must actually engage under scale past it, not merely "
                             "exist unexercised";
}

TEST(NodeLivenessTrackerCap, PresentEntriesExceedingTheCapAreAllStillIndividuallyReportableOnDeath) {
  // C1 regression, at the tracker level: the earlier shape of this cap evicted PRESENT
  // entries to admit a newcomer once the map (not merely the departed subset) reached
  // tracked_key_cap - at 513 armed nodes against the default 512, only one survived. A
  // present key must never be refused a slot or evicted for one, at any map size.
  NodeLivenessTracker tracker(/*miss_grace=*/0, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/5);
  std::set<std::string> ids;
  for (int i = 0; i < 50; ++i) {
    ids.insert("/n" + std::to_string(i));
  }
  auto report = tracker.update(ids, ids);  // 50 present, armed keys - ten times the cap
  EXPECT_TRUE(report.dead.empty());
  EXPECT_EQ(tracker.tracked_count(), 50u) << "a cap on the DEPARTED subset must never refuse "
                                             "or evict a present key, however many there are";

  ids.erase(ids.begin());  // kill exactly one of the fifty
  report = tracker.update(ids, ids);
  EXPECT_EQ(report.dead.size(), 1u) << "the one node that actually died must be reported - "
                                       "the other 49, still present, never competed for the "
                                       "departed cap at all";
}

TEST(NodeLivenessTrackerCap, ImmatureDepartedEntriesAreNeverCollapsedOrReportedUnderCapPressure) {
  // C2 regression, at the tracker level: the earlier shape of this cap collapsed every
  // NON-idle entry to make room, which included entries carrying only one below-grace miss -
  // fabricating a death the node had not actually earned, permanently (the identity is
  // erased, so the node returning cannot un-report it). An entry that has not crossed
  // miss_grace_ must never be a collapse candidate, however much departed-set pressure exists.
  NodeLivenessTracker tracker(/*miss_grace=*/2, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/1);
  tracker.update({"/a", "/b"}, {"/a", "/b"});  // both present, armed
  auto report = tracker.update({}, {});        // both miss 1 of 2 - departed count(2) > cap(1)
  EXPECT_TRUE(report.dead.empty()) << "neither has crossed miss_grace yet";
  EXPECT_EQ(report.dead.count(NodeLivenessTracker::kCollapsedKey), 0u)
      << "capacity pressure from two BELOW-GRACE entries must never manufacture a collapsed "
         "death - the earlier shape of this cap did exactly that";
  EXPECT_EQ(tracker.tracked_count(), 2u) << "both identities must survive the pressure "
                                            "untouched, so they can still mature genuinely";
  EXPECT_TRUE(report.tracking_saturated) << "the departed set (2) still exceeds the cap (1) "
                                            "even though nothing was eligible to collapse - an "
                                            "accurate, non-fabricating pressure signal";
}

TEST(NodeLivenessTrackerCap, MaturedDepartedEntriesAreCollapsedIntoACountUnderCapPressure) {
  NodeLivenessTracker tracker(/*miss_grace=*/0, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/1);
  tracker.update({}, {"/victim"});
  ASSERT_EQ(tracker.update({}, {}).dead.count("/victim"), 1u);
  ASSERT_EQ(tracker.tracked_count(), 1u);

  // "/newcomer" arms already absent - departed on its very first tick, same as "/victim" was.
  // Both are MATURED (miss_grace=0), and the departed set (2) now exceeds the cap (1): there
  // is no present entry to spare (there never is, for this tracker) and no immature entry to
  // leave alone either, so admitting the newcomer must collapse matured entries instead of
  // refusing it.
  auto report = tracker.update({}, {"/newcomer"});
  EXPECT_EQ(tracker.tracked_count(), 0u) << "cap(1) is below kMaxNamedDepartedEntries, so even "
                                            "the two-stage collapse cannot leave either one "
                                            "individually named";
  EXPECT_EQ(report.dead.count("/victim"), 0u);
  EXPECT_EQ(report.dead.count("/newcomer"), 0u);
  ASSERT_EQ(report.dead.count(NodeLivenessTracker::kCollapsedKey), 1u)
      << "both confirmed deaths must still count as content, or the cap forcing them out of "
         "individual tracking would silently heal the fault instead of collapsing it";
  EXPECT_NE(report.dead.at(NodeLivenessTracker::kCollapsedKey).find('2'), std::string::npos);
  ASSERT_FALSE(report.keys_by_freshness.empty());
  EXPECT_EQ(report.keys_by_freshness.front(), NodeLivenessTracker::kCollapsedKey)
      << "the collapsed count must never be the entry a capped description cuts, since it is "
         "the one line telling the operator identities are being lost at all";
}

TEST(NodeLivenessTrackerCap, CollapsedCountIsMonotoneAndKeepsAccumulating) {
  NodeLivenessTracker tracker(/*miss_grace=*/0, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/1);
  tracker.update({}, {"/first"});
  tracker.update({}, {});                               // "/first" confirmed dead, occupies the only slot
  auto after_second = tracker.update({}, {"/second"});  // "/second" arms already-departed: both collapse
  ASSERT_EQ(after_second.dead.count(NodeLivenessTracker::kCollapsedKey), 1u);
  EXPECT_NE(after_second.dead.at(NodeLivenessTracker::kCollapsedKey).find('2'), std::string::npos);
  ASSERT_EQ(tracker.tracked_count(), 0u);

  auto after_third = tracker.update({}, {"/third"});  // one free slot: "/third" stays named alone
  EXPECT_EQ(tracker.tracked_count(), 1u);
  EXPECT_NE(after_third.dead.at(NodeLivenessTracker::kCollapsedKey).find('2'), std::string::npos)
      << "the count from the first collapse must persist even on a tick that collapses nothing new";

  auto after_fourth = tracker.update({}, {"/fourth"});  // "/fourth" arms already-departed: collapses both
  ASSERT_EQ(after_fourth.dead.count(NodeLivenessTracker::kCollapsedKey), 1u);
  EXPECT_NE(after_fourth.dead.at(NodeLivenessTracker::kCollapsedKey).find('4'), std::string::npos)
      << "two more distinct identities have now been collapsed - the count must not have reset "
         "when the slot changed hands again";
}

TEST(NodeLivenessTrackerCap, AtZeroMissGraceEveryDepartedEntryIsInstantlyMaturedSoSaturationNeverFires) {
  // NOT a general property of this tracker - see
  // ImmatureDepartedEntriesAreNeverCollapsedOrReportedUnderCapPressure above for the case
  // that DOES saturate. At miss_grace=0 every departed entry matures on its very first
  // missed tick, so there is never an immature entry the cap has to leave alone: collapsing
  // every matured entry always succeeds in bringing the departed set back under the cap.
  NodeLivenessTracker tracker(/*miss_grace=*/0, NodeLivenessTracker::kNoPrune, /*tracked_key_cap=*/1);
  bool ever_saturated = false;
  for (int i = 0; i < 50; ++i) {
    // Two brand-new, simultaneously-arming keys every tick, cap=1: the tightest possible
    // squeeze this tracker can be put under.
    auto report = tracker.update({}, {"/a_" + std::to_string(i), "/b_" + std::to_string(i)});
    ever_saturated = ever_saturated || report.tracking_saturated;
  }
  EXPECT_FALSE(ever_saturated);
}
