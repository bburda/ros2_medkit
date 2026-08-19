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
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"

namespace {
using ros2_medkit_graph_watchdog::AggregatedFault;
using ros2_medkit_graph_watchdog::kDefaultAbsenceGrace;
using ros2_medkit_graph_watchdog::kDefaultNoMatchWarnTicks;
using ros2_medkit_graph_watchdog::kDefaultObservationSettleTicks;
using ros2_medkit_graph_watchdog::kDefaultTrackedNodeCap;
using ros2_medkit_graph_watchdog::kDefaultUnmeasuredHoldTicks;
using ros2_medkit_graph_watchdog::kMaxLifecycleDetailChars;
using ros2_medkit_graph_watchdog::kMaxLifecycleLabelChars;
using ros2_medkit_graph_watchdog::kMaxNamedDepartedEntries;
using ros2_medkit_graph_watchdog::LifecycleExpectationTracker;
using ros2_medkit_graph_watchdog::LifecycleMatch;
using M = std::vector<LifecycleMatch>;

// Violations are keyed by the NODE, so a match carries both the config entry that
// matched and the node's stable fqn. `armed` defaults true, matching LifecycleMatch's own
// default: a call that does not pass it exercises a node the presence detector could
// report, and only a test about the never-armed split passes `false` explicitly.
LifecycleMatch match(const std::string & entry, const std::string & fqn, std::optional<std::string> state,
                     bool armed = true) {
  return LifecycleMatch{entry, fqn, std::move(state), armed};
}

// ---- Violation streak: unchanged behaviour from before this slice's redesign ----

TEST(LifecycleExpectation, RequiredNodeStuckInactivePastGraceRaises) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2);
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty());  // 1 <= grace
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty());  // 2 == grace
  auto report = t.update(M{match("a", "/a", "inactive")});                  // 3 > grace
  ASSERT_EQ(report.affected.size(), 1u);
  EXPECT_TRUE(report.affected.count("/a")) << "the fault must be keyed by the node, not the config entry";
  EXPECT_NE(report.affected.at("/a").find("/a"), std::string::npos);
  EXPECT_NE(report.affected.at("/a").find("required by 'a'"), std::string::npos)
      << "the entry that demanded it is still worth naming, but as context";
}

TEST(LifecycleExpectation, RequiredNodeActiveNeverRaises) {
  LifecycleExpectationTracker t({"a"}, 0);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());
  }
}

TEST(LifecycleExpectation, ReachesActiveWithinGraceNeverRaises) {
  LifecycleExpectationTracker t({"a"}, 2);
  t.update(M{match("a", "/a", "unconfigured")});                          // 1
  t.update(M{match("a", "/a", "inactive")});                              // 2
  EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());  // reset
  EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());
}

TEST(LifecycleExpectation, AbsentNodeIsNotOurJob) {
  LifecycleExpectationTracker t({"a"}, 0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(t.update(M{}).affected.empty());  // absent -> GRAPH_NODE_DISAPPEARED owns it
  }
}

// The bare-name form is deliberately fleet-wide, so one entry covers N nodes. Keying the
// violation on the ENTRY meant the fault could not say which namesake broke, and with two
// offenders only one survived into the report at all.
TEST(LifecycleExpectation, BothNamesakesAreReportedSeparately) {
  LifecycleExpectationTracker t({"controller_server"}, /*grace=*/0);
  auto report = t.update(M{match("controller_server", "/left/controller_server", "inactive"),
                           match("controller_server", "/right/controller_server", "unconfigured")});
  ASSERT_EQ(report.affected.size(), 2u) << "one entry, two offenders, two reports";
  EXPECT_TRUE(report.affected.count("/left/controller_server"));
  EXPECT_TRUE(report.affected.count("/right/controller_server"));
}

TEST(LifecycleExpectation, HealthyNamesakeDoesNotMaskABrokenOne) {
  LifecycleExpectationTracker t({"controller_server"}, /*grace=*/0);
  auto report = t.update(M{match("controller_server", "/left/controller_server", "active"),
                           match("controller_server", "/right/controller_server", "inactive")});
  ASSERT_EQ(report.affected.size(), 1u);
  EXPECT_TRUE(report.affected.count("/right/controller_server"));
}

// A node that blinks out of one snapshot used to have its violation streak zeroed outright,
// so a genuinely stuck node on a churning graph could never accumulate grace+1 consecutive
// present-and-inactive ticks.
TEST(LifecycleExpectation, SingleSnapshotBlinkDoesNotResetTheViolationStreak) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/3);
  t.update(M{match("a", "/a", "inactive")});                // 1
  t.update(M{});                                            // blink: absent for one tick
  t.update(M{match("a", "/a", "inactive")});                // 2
  auto report = t.update(M{match("a", "/a", "inactive")});  // 3 > grace
  EXPECT_FALSE(report.affected.empty()) << "one absent snapshot must not restart the count";
}

// The bare-name form is fleet-wide and the full-FQN form pins one robot, so an operator
// combining them - both documented, both recommended - has ONE node named by TWO entries.
// The caller pushes one match per (entry, node) pair, so that node arrives twice in the
// same sweep; counting per match instead of per node advances its streak twice a tick and
// halves the grace the operator configured.
TEST(LifecycleExpectation, TwoEntriesMatchingOneNodeDoNotHalveItsGrace) {
  LifecycleExpectationTracker t({"a", "/a"}, /*grace=*/2);
  const M both{match("a", "/a", "inactive"), match("/a", "/a", "inactive")};
  EXPECT_TRUE(t.update(both).affected.empty()) << "tick 1: one node, one streak - 1 <= grace";
  EXPECT_TRUE(t.update(both).affected.empty()) << "tick 2: 2 == grace, not past it yet";
  auto report = t.update(both);  // tick 3: 3 > grace
  ASSERT_EQ(report.affected.size(), 1u) << "two entries naming one node must yield one report";
  ASSERT_TRUE(report.affected.count("/a"));
  EXPECT_NE(report.affected.at("/a").find("'a'"), std::string::npos) << "every matching entry is context";
  EXPECT_NE(report.affected.at("/a").find("'/a'"), std::string::npos) << "every matching entry is context";
}

// The same node, the same tick, one entry seeing it inactive: a violating read must never
// be tied-out by a benign one from the other entry.
TEST(LifecycleExpectation, ViolatingReadWinsOverABenignOneForTheSameNode) {
  LifecycleExpectationTracker t({"a", "/a"}, /*grace=*/0);
  auto report = t.update(M{match("a", "/a", "active"), match("/a", "/a", "inactive")});
  ASSERT_EQ(report.affected.size(), 1u) << "a measured violation must survive a disagreeing duplicate match";
  EXPECT_TRUE(report.affected.count("/a"));
}

// A duplicate match disagreeing between the two UNMEASURED causes - or between an unmeasured
// read and a healthy one - must pick the more informative reading too, mirroring the
// violating-wins rule above. This is edge-case hardening: two entries naming one node are
// the same physical app.id read twice, so in practice they never actually disagree.
TEST(LifecycleExpectation, DuplicateMatchPicksUnreadableOverNotManagedAndOverActive) {
  LifecycleExpectationTracker t({"a", "/a"}, /*grace=*/0);
  // "" (unreadable) must win over nullopt (not-managed): run both past the hold and check
  // which content map claims the node.
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks + 1; ++i) {
    auto report = t.update(M{match("a", "/a", std::string("")), match("/a", "/a", std::nullopt)});
    if (i == kDefaultUnmeasuredHoldTicks) {
      EXPECT_TRUE(report.unreadable_affected.count("/a")) << "unreadable did not win the tie-break over not-managed";
      EXPECT_TRUE(report.not_managed_affected.empty());
    }
  }
}

// ---- The unmeasured clock: UNREADABLE and NOT_MANAGED share one cause-blind clock ----

// Neither UNREADABLE nor NOT_MANAGED is ever a confirmed violation, however long the run:
// only a MEASURED not-active read ever advances the violation streak, and the unmeasured
// clock only ever feeds its OWN content maps, never `affected`.
TEST(LifecycleExpectation, UnreadableLabelNeverConfirmsInactiveWhileItsOwnClockClimbs) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    auto report = t.update(M{match("a", "/a", std::string(""))});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i;
    EXPECT_TRUE(report.unreadable_affected.empty()) << "iteration " << i << ": still below the hold";
    EXPECT_EQ(report.pending_unreadable.count("/a"), 1u) << "iteration " << i;
  }
}

TEST(LifecycleExpectation, NotManagedNeverConfirmsInactiveWhileItsOwnClockClimbs) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    auto report = t.update(M{match("a", "/a", std::nullopt)});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i;
    EXPECT_TRUE(report.not_managed_affected.empty()) << "iteration " << i << ": still below the hold";
    EXPECT_EQ(report.pending_not_managed.count("/a"), 1u) << "iteration " << i;
  }
}

// The instrument matters here (test-plan shape requirement 4): this asserts
// `state.has_value() && state->empty()` via the match builder's std::string("") overload,
// never merely "not active".
TEST(LifecycleExpectation, UnreadableMaturesIntoItsOwnFaultAtTheExactHoldBoundary) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    t.update(M{match("a", "/a", std::string(""))});
  }
  auto report = t.update(M{match("a", "/a", std::string(""))});  // hold + 1: crosses THIS tick
  ASSERT_EQ(report.unreadable_affected.size(), 1u);
  EXPECT_TRUE(report.not_managed_affected.empty());
  EXPECT_TRUE(report.affected.empty());
  EXPECT_NE(report.unreadable_affected.at("/a").find("/a"), std::string::npos);
  EXPECT_NE(report.unreadable_affected.at("/a").find("could not be read"), std::string::npos);
  ASSERT_EQ(report.newly_unreadable.size(), 1u) << "crossed the hold on exactly this tick";
  EXPECT_EQ(report.newly_unreadable[0], "/a");
  EXPECT_TRUE(report.pending_unreadable.empty()) << "matured: no longer pending, ownership was taken";
}

// The instrument matters here too: `!state.has_value()`, never an empty string.
TEST(LifecycleExpectation, NotManagedMaturesIntoItsOwnFaultAtTheExactHoldBoundary) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    t.update(M{match("a", "/a", std::nullopt)});
  }
  auto report = t.update(M{match("a", "/a", std::nullopt)});  // hold + 1: crosses THIS tick
  ASSERT_EQ(report.not_managed_affected.size(), 1u);
  EXPECT_TRUE(report.unreadable_affected.empty());
  EXPECT_TRUE(report.affected.empty());
  EXPECT_NE(report.not_managed_affected.at("/a").find("/a"), std::string::npos);
  EXPECT_NE(report.not_managed_affected.at("/a").find("not a managed lifecycle node"), std::string::npos);
  ASSERT_EQ(report.newly_not_managed.size(), 1u) << "crossed the hold on exactly this tick";
  EXPECT_EQ(report.newly_not_managed[0], "/a");
  EXPECT_TRUE(report.pending_not_managed.empty()) << "matured: no longer pending, ownership was taken";
}

// The redesign's whole point, proven directly: three review rounds each closed one
// alternation and revealed the next, because the design being replaced counted UNREADABLE
// and NOT_MANAGED on two SEPARATE counters, each reset by the other's tick. A node
// alternating between the two therefore never crossed either counter's hold, however long
// the run - invisible to both codes forever. Here the SAME alternation, against the new
// cause-blind clock, matures - proving the clock counts the ALTERNATION, not either cause
// alone. (This exact scenario was watched RED against the pre-redesign detector - see the
// integration test of the same shape for the end-to-end proof; this is the pure-logic proof
// at the level the fix actually lives.)
TEST(LifecycleExpectation, AlternatingUnreadableAndNotManagedStillMaturesTheClock) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5);
  bool matured = false;
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks + 5 && !matured; ++i) {
    const auto state = (i % 2 == 0) ? std::optional<std::string>("") : std::optional<std::string>(std::nullopt);
    auto report = t.update(M{match("a", "/a", state)});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i << ": never a confirmed violation";
    matured = !report.unreadable_affected.empty() || !report.not_managed_affected.empty();
  }
  EXPECT_TRUE(matured) << "a node alternating between unreadable and not-managed never matured under "
                          "either code - exactly the failure this redesign closes";
}

// A node whose cause flips EVERY SINGLE TICK (the fastest possible alternation, faster than
// the pairwise sweep above which flips at the same cadence) - the degenerate case that most
// directly defeats a per-cause counter design (each tick would have zeroed the OTHER
// counter under the design being replaced).
TEST(LifecycleExpectation, AlternatingEveryTickStillMaturesTheClockEventually) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    const auto state = (i % 2 == 0) ? std::optional<std::string>("") : std::optional<std::string>(std::nullopt);
    auto report = t.update(M{match("a", "/a", state)});
    EXPECT_TRUE(report.unreadable_affected.empty() && report.not_managed_affected.empty())
        << "iteration " << i << ": matured too early";
  }
  const auto last_state = (kDefaultUnmeasuredHoldTicks % 2 == 0) ? std::optional<std::string>("")
                                                                 : std::optional<std::string>(std::nullopt);
  auto report = t.update(M{match("a", "/a", last_state)});
  EXPECT_TRUE(!report.unreadable_affected.empty() || !report.not_managed_affected.empty())
      << "the clock never matured despite " << (kDefaultUnmeasuredHoldTicks + 1) << " total unmeasured ticks";
}

// The pair the two clocks have to divide between them: a node alternating between a REAL
// not-active read and a not-managed one, with runs of ABSENCE longer than the absence grace
// in between - so every tick belongs to one of the three cases. kNotManaged never touches the
// violation streak (only its own unmeasured clock), kInactive always resets the unmeasured
// clock, and absence below grace contributes to neither (see the kInactive case in update()'s
// absence loop) - so the ONLY thing that can ever advance the violation streak here is a
// genuine matched "inactive" read, and it takes exactly grace + 1 of them, however many
// not-managed legs and absence gaps sit in between.
TEST(LifecycleExpectation, InactiveAlternatingWithNotManagedAcrossAbsenceGapsCountsOnlyMatchedReads) {
  constexpr int kGap = kDefaultAbsenceGrace + 1;
  constexpr int kGrace = 3;
  LifecycleExpectationTracker t({"a"}, kGrace, kDefaultAbsenceGrace);
  for (int matched_tick = 1; matched_tick <= kGrace; ++matched_tick) {
    auto measured = t.update(M{match("a", "/a", "inactive")});
    EXPECT_TRUE(measured.affected.empty())
        << "matched tick " << matched_tick << " crossed grace(" << kGrace << ") early";
    for (int i = 0; i < kGap; ++i) {
      EXPECT_TRUE(t.update(M{}).affected.empty())
          << "matched tick " << matched_tick << ", absence gap " << i << ": crossed on an ABSENT tick";
    }
    auto unmanaged = t.update(M{match("a", "/a", std::nullopt)});
    EXPECT_TRUE(unmanaged.affected.empty())
        << "matched tick " << matched_tick << ": a not-managed read crossed grace on its own";
    EXPECT_EQ(unmanaged.pending_violation.count("/a"), 1u)
        << "matched tick " << matched_tick << ": the streak was RESET by a not-managed tick instead of held";
    for (int i = 0; i < kGap; ++i) {
      EXPECT_TRUE(t.update(M{}).affected.empty())
          << "matched tick " << matched_tick << ", trailing gap " << i << ": crossed on an ABSENT tick";
    }
  }
  auto crossed = t.update(M{match("a", "/a", "inactive")});  // the (grace + 1)-th matched inactive read
  EXPECT_EQ(crossed.affected.count("/a"), 1u)
      << "never crossed grace after exactly grace + 1 matched inactive reads - the not-managed legs and "
         "absence gaps in between must have contributed nothing to either side of the count";
  EXPECT_EQ(crossed.newly_affected, (std::vector<std::string>{"/a"}));
}

// The violation streak resets ONLY on kActive. A mutant that zeroed it on every unmeasured
// tick would leave every other test in this suite green, because during the climbing window
// the ONLY field that differs is `pending_violation` - which nothing else reads there. This
// reads it, on every unmeasured tick, and then proves the streak resumed rather than
// restarted by counting the remaining ticks EXACTLY.
TEST(LifecycleExpectation, ViolationStreakSurvivesNonMaturingUnmeasuredTicksAndIsNotRestarted) {
  constexpr int kGrace = 5;
  LifecycleExpectationTracker t({"a"}, kGrace);
  for (int i = 0; i < 3; ++i) {
    auto climbing = t.update(M{match("a", "/a", "inactive")});  // streak 1..3, all <= grace
    ASSERT_TRUE(climbing.affected.empty()) << "iteration " << i;
    ASSERT_EQ(climbing.pending_violation.count("/a"), 1u) << "iteration " << i;
  }
  // A run of unmeasured ticks far too short to mature the unmeasured clock. The streak is
  // HELD across all of them: not reported (nothing is measurable right now), not reset.
  for (int i = 0; i < 4; ++i) {
    auto unread = t.update(M{match("a", "/a", std::string(""))});
    EXPECT_EQ(unread.pending_violation.count("/a"), 1u)
        << "unmeasured tick " << i << ": the violation streak was dropped by a tick that measured nothing";
    EXPECT_TRUE(unread.affected.empty()) << "unmeasured tick " << i;
  }
  // Exactly two more measured not-active ticks reach 5 == grace, and the third crosses it.
  // Counted exactly: a streak that had restarted would need six here, not three.
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "streak 4 <= grace";
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "streak 5 == grace";
  auto crossed = t.update(M{match("a", "/a", "inactive")});
  EXPECT_EQ(crossed.affected.count("/a"), 1u)
      << "the streak restarted across the unmeasured run instead of resuming - a node that goes "
         "briefly unreadable every few ticks would never be confirmed";
  EXPECT_EQ(crossed.newly_affected, (std::vector<std::string>{"/a"}));
}

// The clock resets on EITHER real measurement - kActive as well as kInactive - never on
// anything else. Proven directly rather than assumed from the maturity tests above.
TEST(LifecycleExpectation, UnmeasuredClockResetsOnlyOnARealMeasurement) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks - 1; ++i) {
    t.update(M{match("a", "/a", std::string(""))});  // climbing, not yet matured
  }
  // A real measurement (active) resets it entirely.
  auto after_active = t.update(M{match("a", "/a", "active")});
  EXPECT_TRUE(after_active.pending_unreadable.empty()) << "an active read did not reset the unmeasured clock";

  // Climb again, then reset via the OTHER real measurement (inactive).
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks - 1; ++i) {
    t.update(M{match("a", "/a", std::nullopt)});
  }
  auto after_inactive = t.update(M{match("a", "/a", "inactive")});
  EXPECT_TRUE(after_inactive.pending_not_managed.empty()) << "an inactive read did not reset the unmeasured clock";
  EXPECT_EQ(after_inactive.pending_violation.count("/a"), 1u) << "the inactive read itself starts a fresh streak";
}

// R28: ownership is exclusive, and taking it is not merely a bookkeeping detail - the
// violation streak is RELEASED (reset to 0), not just excluded from `affected` THIS tick.
// A node CONFIRMED inactive that then goes unreadable long enough to mature must stop being
// GRAPH_NODE_INACTIVE's concern entirely, so a later return to inactive re-earns `grace`
// from zero rather than re-crossing on the very next tick.
TEST(LifecycleExpectation, UnmeasuredClockMaturityReleasesTheViolationStreakEntirely) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1);
  t.update(M{match("a", "/a", "inactive")});
  auto confirmed = t.update(M{match("a", "/a", "inactive")});  // 2 > grace: confirmed
  ASSERT_FALSE(confirmed.affected.empty()) << "sanity: must be confirmed before it can be released";

  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    auto report = t.update(M{match("a", "/a", std::string(""))});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i
                                         << ": the held streak must not re-confirm while "
                                            "unread - only maturity, or a real read, changes anything";
  }
  auto matured = t.update(M{match("a", "/a", std::string(""))});  // crosses the hold: releases
  ASSERT_FALSE(matured.unreadable_affected.empty()) << "sanity: must have matured";
  EXPECT_TRUE(matured.affected.empty());
  EXPECT_TRUE(matured.pending.count("/a") == 0) << "released: GRAPH_NODE_INACTIVE has nothing left to hold for it";

  // The node returns to inactive: it must re-earn `grace` (1) from zero, not re-cross
  // immediately on the strength of the old, released streak.
  auto first_back = t.update(M{match("a", "/a", "inactive")});
  EXPECT_TRUE(first_back.affected.empty()) << "1 <= grace: the streak restarted at zero, not resumed";
  EXPECT_EQ(first_back.pending_violation.count("/a"), 1u);
  auto second_back = t.update(M{match("a", "/a", "inactive")});
  EXPECT_FALSE(second_back.affected.empty()) << "2 > grace: re-earned normally from the fresh streak";
}

// The design decision this slice makes on its own, pinned directly: once matured, the fault
// code a node reports under is STICKY - it does not flip merely because a later tick's LIVE
// read shows the other cause, as long as the clock never resets via a real measurement. The
// rejected alternative (current-cause-wins) would flip `unreadable_affected` <->
// `not_managed_affected` on every alternating tick below; sticky keeps it fixed on whichever
// cause was live AT THE MOMENT of maturity.
TEST(LifecycleExpectation, StickyCauseSurvivesACauseChangeAfterMaturity) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    t.update(M{match("a", "/a", std::string(""))});  // climbs as UNREADABLE
  }
  auto matured = t.update(M{match("a", "/a", std::string(""))});  // matures AS unreadable
  ASSERT_EQ(matured.newly_unreadable.size(), 1u) << "sanity: must mature under kUnreadable first";

  // The cause now flips to NOT_MANAGED for a long stretch, without ever crossing a real
  // measurement. Sticky: the node must stay GRAPH_NODE_UNREADABLE's content throughout.
  for (int i = 0; i < 20; ++i) {
    auto report = t.update(M{match("a", "/a", std::nullopt)});
    EXPECT_TRUE(report.unreadable_affected.count("/a") == 1) << "iteration " << i << ": sticky cause must hold";
    EXPECT_TRUE(report.not_managed_affected.empty())
        << "iteration " << i << ": current-cause-wins would have flipped ownership here - rejected";
    EXPECT_TRUE(report.newly_not_managed.empty()) << "iteration " << i << ": no re-raise churn under a sticky cause";
  }

  // And the same the other way around: matured as NOT_MANAGED, flips to UNREADABLE.
  LifecycleExpectationTracker t2({"b"}, /*grace=*/5);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    t2.update(M{match("b", "/b", std::nullopt)});
  }
  auto matured2 = t2.update(M{match("b", "/b", std::nullopt)});
  ASSERT_EQ(matured2.newly_not_managed.size(), 1u) << "sanity: must mature under kNotManaged first";
  for (int i = 0; i < 20; ++i) {
    auto report = t2.update(M{match("b", "/b", std::string(""))});
    EXPECT_TRUE(report.not_managed_affected.count("/b") == 1) << "iteration " << i;
    EXPECT_TRUE(report.unreadable_affected.empty()) << "iteration " << i;
  }
}

// ---- Absence: held inside the blink tolerance, CONTINUED past it, never erased ----

// A blink well inside the absence grace must not disturb a still-climbing unmeasured clock
// at all - it neither advances it (a blink is tolerated, not counted) nor resets it (the
// same "held through a blink" contract every other clock in this tracker gets).
TEST(LifecycleExpectation, AbsentNodeWithAClimbingUnmeasuredClockIsHeldThroughABlink) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5, /*absence_grace=*/3);
  for (int i = 0; i < 10; ++i) {
    t.update(M{match("a", "/a", std::string(""))});
  }
  auto blink = t.update(M{});  // absent 1: well inside the absence grace
  EXPECT_EQ(blink.pending_unreadable.count("/a"), 1u) << "held through the blink, not reset nor advanced";

  // Resumes exactly where it left off: 10 climbing ticks before the blink, so exactly
  // (kDefaultUnmeasuredHoldTicks - 10) more real unreadable ticks are needed to reach the
  // hold itself (60 total) and one more past it matures - not kDefaultUnmeasuredHoldTicks
  // more from scratch, which would prove the blink reset it instead of merely holding it.
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks - 10; ++i) {
    auto report = t.update(M{match("a", "/a", std::string(""))});
    EXPECT_TRUE(report.unreadable_affected.empty()) << "iteration " << i;
  }
  auto matured = t.update(M{match("a", "/a", std::string(""))});
  EXPECT_FALSE(matured.unreadable_affected.empty())
      << "the blink must not have added extra ticks to the budget - the clock resumed, it did not reset";
}

// Past the absence grace the unmeasured clock KEEPS CLIMBING under the cause the node's
// last real observation set - it is neither reset nor frozen - so a node that vanishes
// mid-spell eventually matures on absence alone. The design this replaces discarded the
// whole thing here, which is what let a node that touches absence periodically evade every
// code forever.
TEST(LifecycleExpectation, AbsentPastGraceContinuesTheClimbingUnmeasuredClockAndEventuallyMaturesIt) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5, /*absence_grace=*/2);
  for (int i = 0; i < 10; ++i) {
    t.update(M{match("a", "/a", std::string(""))});  // optional(""): a MANAGED, unread node
  }
  t.update(M{});               // absent 1
  auto blink = t.update(M{});  // absent 2 == absence_grace: still inside the blink tolerance
  EXPECT_EQ(blink.pending_unreadable.count("/a"), 1u) << "absent 2 == absence_grace: held, unchanged";
  auto continued = t.update(M{});  // absent 3 > absence_grace: the clock resumes advancing
  EXPECT_EQ(continued.pending_unreadable.count("/a"), 1u)
      << "sustained absence discarded a climbing unmeasured clock instead of continuing it";
  EXPECT_TRUE(continued.unreadable_affected.empty()) << "sanity: nowhere near the hold yet";

  // 10 climbing ticks before the blink, plus the one absent tick above: exactly
  // (kDefaultUnmeasuredHoldTicks - 11) more absent ticks reach the hold, and one past it
  // matures. Counted exactly, so a clock that merely stopped resetting - without actually
  // advancing while gone - would fail here rather than passing on a generous budget.
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks - 11; ++i) {
    EXPECT_TRUE(t.update(M{}).unreadable_affected.empty()) << "iteration " << i << ": matured too early";
  }
  auto matured = t.update(M{});
  ASSERT_EQ(matured.unreadable_affected.size(), 1u)
      << "absence never matured the clock it had been advancing - the node is invisible for as long "
         "as it stays gone, which is exactly the evasion this closes";
  EXPECT_EQ(matured.newly_unreadable, (std::vector<std::string>{"/a"}));
  EXPECT_NE(matured.unreadable_affected.at("/a").find("has since left the graph"), std::string::npos)
      << "a fault raised about a node that is no longer in the graph must say so, or the operator is "
         "sent looking for it";
}

// A node that had ALREADY matured before it went absent KEEPS its fault: ownership is
// sticky and absence does not release it. The node was declared must-be-active, was never
// measured active, and is now gone - none of which is health.
TEST(LifecycleExpectation, AbsentPastGraceKeepsAnAlreadyMaturedUnmeasuredFault) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5, /*absence_grace=*/2);
  for (int i = 0; i <= kDefaultUnmeasuredHoldTicks; ++i) {
    t.update(M{match("a", "/a", std::nullopt)});  // nullopt: no tracked lifecycle at all
  }
  ASSERT_EQ(t.update(M{match("a", "/a", std::nullopt)}).not_managed_affected.size(), 1u) << "sanity: matured";

  t.update(M{});              // absent 1: content survives the blink
  auto held = t.update(M{});  // absent 2 == absence_grace: still held
  EXPECT_EQ(held.not_managed_affected.count("/a"), 1u);
  EXPECT_EQ(held.not_managed_affected.at("/a").find("has since left the graph"), std::string::npos)
      << "a blink inside the tolerance is not a departure and must not be described as one";
  for (int i = 0; i < 20; ++i) {
    auto still = t.update(M{});  // absent 3.. > absence_grace
    ASSERT_EQ(still.not_managed_affected.count("/a"), 1u)
        << "iteration " << i << ": absence released a matured fault, discarding evidence the node earned";
    EXPECT_TRUE(still.newly_not_managed.empty()) << "iteration " << i << ": no re-raise churn while merely absent";
  }
  EXPECT_NE(t.update(M{}).not_managed_affected.at("/a").find("has since left the graph"), std::string::npos);
}

// The one departure that starts nothing: a node measured ACTIVE and then shut down. Both
// clocks are at zero, absence has nothing to continue, and the entry is idle so it is
// reclaimed silently. Raising for a HEALTHY node that left is GRAPH_NODE_DISAPPEARED's job,
// which has no detector in this package - the single gap left here.
TEST(LifecycleExpectation, HealthyNodeThatVanishesRaisesNothingAndIsReclaimed) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, /*absence_grace=*/2, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/3);
  for (int i = 0; i < 5; ++i) {
    t.update(M{match("a", "/a", "active")});
  }
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks + 10; ++i) {
    auto report = t.update(M{});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i << ": a healthy node that left was reported inactive";
    EXPECT_TRUE(report.unreadable_affected.empty() && report.not_managed_affected.empty())
        << "iteration " << i << ": absence STARTED an unmeasured clock for a node that was measured active";
    EXPECT_TRUE(report.pending.empty()) << "iteration " << i << ": a healthy departure must not withhold anything";
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "an idle entry past the prune horizon must be reclaimed";
}

// The same for a node that had been measured active AFTER an earlier bad spell: what
// absence continues is the LAST real observation, not the worst one ever seen.
TEST(LifecycleExpectation, AbsenceContinuesTheLastObservationNotAnEarlierWorseOne) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/1);
  t.update(M{match("a", "/a", "inactive")});
  t.update(M{match("a", "/a", "inactive")});  // 2 > grace: confirmed
  ASSERT_FALSE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "sanity: confirmed first";
  t.update(M{match("a", "/a", "active")});  // healed: both clocks and the label cleared

  for (int i = 0; i < 10; ++i) {
    auto report = t.update(M{});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i
                                         << ": absence resumed a streak the node had already cleared by reading active";
    EXPECT_TRUE(report.pending.empty()) << "iteration " << i;
  }
}

// ---- Absence CONTINUES whatever the node was already doing ----

// The shape every one of the three tests below drives: `run_length` absent ticks between
// every real observation, forever. That is what a node in a restart loop looks like from
// the graph - start, crash, respawn delay, start - and it is the case this detector most
// exists to catch, so a clock that absence RESETS makes exactly that node invisible
// however long it runs. `run_length` is swept at the absence grace itself (a blink, which
// was already held) and past it (the run that used to wipe everything).
constexpr int kAbsenceRunLengths[] = {kDefaultAbsenceGrace, kDefaultAbsenceGrace + 2};

TEST(LifecycleExpectation, UnreadableInterleavedWithAbsenceRunsStillMaturesAndReports) {
  for (const int run_length : kAbsenceRunLengths) {
    LifecycleExpectationTracker t({"a"}, /*grace=*/5, kDefaultAbsenceGrace);
    bool matured = false;
    // Budget: every cycle contributes at least the one present tick, so this is far more
    // than enough for a clock that only counts present ticks, let alone one that counts
    // absent ones too.
    for (int cycle = 0; cycle < kDefaultUnmeasuredHoldTicks + 5 && !matured; ++cycle) {
      // The instrument is optional("") - a MANAGED node whose label never answered - not
      // merely "not active".
      matured = !t.update(M{match("a", "/a", std::string(""))}).unreadable_affected.empty();
      for (int i = 0; i < run_length && !matured; ++i) {
        matured = !t.update(M{}).unreadable_affected.empty();  // ABSENT: not in the snapshot at all
      }
    }
    EXPECT_TRUE(matured) << "run_length=" << run_length
                         << ": a node that is unreadable whenever it is present and absent the rest "
                            "of the time never matured - absence discarded the evidence, so the "
                            "restart loop this detector exists to catch evades it forever";
  }
}

TEST(LifecycleExpectation, NotManagedInterleavedWithAbsenceRunsStillMaturesAndReports) {
  for (const int run_length : kAbsenceRunLengths) {
    LifecycleExpectationTracker t({"a"}, /*grace=*/5, kDefaultAbsenceGrace);
    bool matured = false;
    for (int cycle = 0; cycle < kDefaultUnmeasuredHoldTicks + 5 && !matured; ++cycle) {
      // The instrument is nullopt - no tracked lifecycle at all - never an empty string.
      matured = !t.update(M{match("a", "/a", std::nullopt)}).not_managed_affected.empty();
      for (int i = 0; i < run_length && !matured; ++i) {
        matured = !t.update(M{}).not_managed_affected.empty();
      }
    }
    EXPECT_TRUE(matured) << "run_length=" << run_length
                         << ": a node that carries no tracked lifecycle whenever it is present and "
                            "is absent the rest of the time never matured";
  }
}

// Companion to UnreadableInterleavedWithAbsenceRunsStillMaturesAndReports and its
// not-managed sibling above, but for the VIOLATION streak the claim is the opposite: an
// absence run - however many, however long, however far past kDefaultAbsenceGrace - must
// contribute NOTHING to a streak that has not yet crossed `grace`. Crossing happens at
// exactly grace + 1 MATCHED "inactive" reads, never earlier and never later, whatever the
// interleaved absence pattern looks like - absence must never mature an unmatured streak,
// pinned directly against the exact shape that used to defeat that rule.
TEST(LifecycleExpectation, InactiveInterleavedWithAbsenceRunsNeverCrossesGraceFromAbsenceAlone) {
  for (const int run_length : kAbsenceRunLengths) {
    constexpr int kGrace = 3;
    LifecycleExpectationTracker t({"a"}, kGrace, kDefaultAbsenceGrace);
    for (int matched_tick = 1; matched_tick <= kGrace; ++matched_tick) {
      auto present = t.update(M{match("a", "/a", "inactive")});
      EXPECT_TRUE(present.affected.empty()) << "run_length=" << run_length << ": matched tick " << matched_tick
                                            << " crossed grace(" << kGrace << ") early";
      EXPECT_TRUE(present.unreadable_affected.empty() && present.not_managed_affected.empty())
          << "run_length=" << run_length << ": a MEASURED read must never feed an unmeasured code";
      for (int i = 0; i < run_length; ++i) {
        auto absent = t.update(M{});
        EXPECT_TRUE(absent.affected.empty()) << "run_length=" << run_length
                                             << ": crossed grace on an ABSENT tick - a below-grace violation "
                                                "must never mature while nobody can observe the node";
      }
    }
    // The (grace + 1)-th matched tick, after grace full interleaved cycles that must have
    // contributed nothing: this crosses if and only if every absence run above truly added
    // zero to the streak.
    auto crossed = t.update(M{match("a", "/a", "inactive")});
    EXPECT_EQ(crossed.affected.count("/a"), 1u)
        << "run_length=" << run_length
        << ": never crossed after exactly grace + 1 matched ticks - an "
           "absence run either stole progress from a match or silently added some of its own";
    EXPECT_EQ(crossed.newly_affected, (std::vector<std::string>{"/a"}));
  }
}

// ---- Never armed: nothing else can ever report the departure, so absence must mature it ----

// The node the row above cannot cover: one the reliability gate never armed at all. Compare
// directly against InactiveInterleavedWithAbsenceRunsNeverCrossesGraceFromAbsenceAlone above -
// same shape, same grace, same absence budget - with the single difference being `armed`.
// There the presence detector could report the departure instead, so absence merely holds a
// below-grace streak; here nothing else in the plugin ever will, so absence has to be the one
// that matures it, or the node's departure is reported by nothing at all.
TEST(LifecycleExpectation, NeverArmedNodeMaturesFromAbsenceAloneAndStaysGone) {
  constexpr int kGrace = 5;
  constexpr int kAbsenceGrace = 2;
  LifecycleExpectationTracker t({"a"}, kGrace, kAbsenceGrace);
  ASSERT_TRUE(t.update(M{match("a", "/a", "inactive", /*armed=*/false)}).affected.empty())
      << "sanity: one present tick, streak 1 <= grace";

  // Node vanishes forever. Inside the blink tolerance nothing moves - a never-armed node
  // gets the same blink tolerance as any other.
  for (int i = 0; i < kAbsenceGrace; ++i) {
    EXPECT_TRUE(t.update(M{}).affected.empty()) << "absence " << i << ": still inside the blink tolerance";
  }
  // Past the blink, absence itself keeps the streak climbing. It was 1 after the one present
  // tick above, so exactly (kGrace - 1) further absent ticks reach kGrace (still not past it)
  // and the next one crosses.
  for (int i = 0; i < kGrace - 1; ++i) {
    EXPECT_TRUE(t.update(M{}).affected.empty()) << "past the blink, iteration " << i;
  }
  auto matured = t.update(M{});
  ASSERT_EQ(matured.affected.count("/a"), 1u)
      << "a below-grace violation on a node the presence detector could never have reported "
         "must still mature from absence alone, or its departure is reported by nothing";
  EXPECT_NE(matured.affected.at("/a").find("has since left the graph"), std::string::npos);
  EXPECT_EQ(matured.newly_affected, (std::vector<std::string>{"/a"}));

  // And it stays matured while the node remains gone - absence never un-matures a violation,
  // never-armed or not.
  for (int i = 0; i < 5; ++i) {
    auto still = t.update(M{});
    EXPECT_EQ(still.affected.count("/a"), 1u) << "iteration " << i;
    EXPECT_TRUE(still.newly_affected.empty()) << "iteration " << i << ": no re-raise churn while merely absent";
  }
}

// The narrowing itself, restated for the case it must keep working for: a node armed once
// (however briefly) before it went non-active. `ever_armed` is sticky, so the CURRENT tick
// reading non-active - which, against a real gate, means NOT currently armed, since a managed
// node's node_ok() is false whenever it reads anything but "active" - must not un-arm it. This
// is the fact GRAPH_NODE_DISAPPEARED needs in order to be ABLE to report this exact node, so
// GRAPH_NODE_INACTIVE must stay out of the way, exactly as
// InactiveInterleavedWithAbsenceRunsNeverCrossesGraceFromAbsenceAlone already pins for the
// unqualified (default-armed) case.
TEST(LifecycleExpectation, ArmedNodeBelowGraceThenGoneStillDoesNotMatureFromAbsence) {
  constexpr int kGrace = 5;
  constexpr int kAbsenceGrace = 2;
  LifecycleExpectationTracker t({"a"}, kGrace, kAbsenceGrace);
  ASSERT_TRUE(t.update(M{match("a", "/a", "active", /*armed=*/true)}).affected.empty()) << "sanity: arms the node";
  ASSERT_TRUE(t.update(M{match("a", "/a", "inactive", /*armed=*/false)}).affected.empty())
      << "sanity: one inactive tick, streak 1 <= grace";

  for (int i = 0; i < kAbsenceGrace + 20; ++i) {
    auto report = t.update(M{});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i
                                         << ": a node the presence detector could report must never have "
                                            "its below-grace streak matured by absence alone";
    EXPECT_EQ(report.pending_violation.count("/a"), 1u)
        << "iteration " << i << ": the streak must stay HELD, not erased either";
  }
}

// A never-armed node that RESUMES rather than restarts: absence-driven progress survives the
// return exactly like a present-tick streak already does (see
// ViolationStreakSurvivesNonMaturingUnmeasuredTicksAndIsNotRestarted for the unmeasured-clock
// analogue), counted exactly enough that a mutant which restarted the streak at zero - either
// on absence or on return - would need more matched ticks than this test gives it.
TEST(LifecycleExpectation, NeverArmedNodeResumesRatherThanRestartingAfterReturning) {
  constexpr int kGrace = 5;
  constexpr int kAbsenceGrace = 2;
  LifecycleExpectationTracker t({"a"}, kGrace, kAbsenceGrace);
  t.update(M{match("a", "/a", "inactive", /*armed=*/false)});                                // streak 1
  ASSERT_TRUE(t.update(M{match("a", "/a", "inactive", /*armed=*/false)}).affected.empty());  // streak 2

  // Vanishes long enough to pass the blink and advance on absence alone a few times, but
  // stops short of grace: 2 held (absent_ticks 1..kAbsenceGrace) + 1 advancing tick
  // (absent_ticks kAbsenceGrace+1, streak 2 -> 3).
  for (int i = 0; i < kAbsenceGrace + 1; ++i) {
    EXPECT_TRUE(t.update(M{}).affected.empty()) << "absence " << i;
  }

  // Node returns, still inactive. If absence above had genuinely advanced the streak to 3
  // (not restarted it), exactly 2 more matched ticks reach grace(5) and the 3rd crosses.
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive", /*armed=*/false)}).affected.empty()) << "streak 3 -> 4";
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive", /*armed=*/false)}).affected.empty()) << "streak 4 -> 5 == grace";
  auto crossed = t.update(M{match("a", "/a", "inactive", /*armed=*/false)});  // streak 5 -> 6 > grace
  EXPECT_EQ(crossed.affected.count("/a"), 1u)
      << "the streak restarted at zero across the absence run instead of resuming what absence "
         "had already advanced - a node that departs and returns while never armed would then "
         "need far longer than grace + 1 present ticks to ever be reported";
}

// The tightest prune horizon the documented config space can produce (grace: 0 with
// prune_grace: 0). A node carrying a LIVE clock must survive it: reclaiming bookkeeping by
// age erases evidence, and at this endpoint the age is one tick.
TEST(LifecycleExpectation, NodeCarryingALiveClockSurvivesTheTightestPruneHorizon) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, kDefaultAbsenceGrace, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/0);
  auto confirmed = t.update(M{match("a", "/a", "inactive")});  // 1 > grace(0): confirmed at once
  ASSERT_FALSE(confirmed.affected.empty()) << "sanity: the node must carry evidence before it vanishes";

  for (int i = 0; i < 10; ++i) {
    t.update(M{});
    EXPECT_EQ(t.tracked_count(), 1u)
        << "iteration " << i
        << ": a node carrying a confirmed violation was reclaimed by the prune horizon, so its "
           "evidence is gone and it can never be reported again";
  }
}

// ---- R11: new-first order, for all three fault-shaped maps ----

// A streak still below grace has not crossed anything; the tick it finally passes grace is
// the ONE tick it belongs in newly_affected; every tick after that it is old news.
TEST(LifecycleExpectation, NewlyAffectedNamesOnlyTheNodeThatCrossedGraceThisTick) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1);
  auto below = t.update(M{match("a", "/a", "inactive")});  // 1 <= grace
  EXPECT_TRUE(below.newly_affected.empty()) << "below grace: nothing has crossed yet";

  auto crossed = t.update(M{match("a", "/a", "inactive")});  // 2 > grace: crosses THIS tick
  ASSERT_EQ(crossed.newly_affected.size(), 1u);
  EXPECT_EQ(crossed.newly_affected[0], "/a");
  ASSERT_TRUE(crossed.affected.count("/a"));

  auto still = t.update(M{match("a", "/a", "inactive")});  // 3 > grace: still affected, NOT new
  EXPECT_TRUE(still.newly_affected.empty()) << "a continuing violation must not re-announce as new";
  EXPECT_TRUE(still.affected.count("/a")) << "sanity: still reported, just not as new";
}

TEST(LifecycleExpectation, NewlyUnreadableNamesOnlyTheNodeThatMaturedThisTick) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    auto below = t.update(M{match("a", "/a", std::string(""))});
    EXPECT_TRUE(below.newly_unreadable.empty()) << "iteration " << i;
  }
  auto matured = t.update(M{match("a", "/a", std::string(""))});
  ASSERT_EQ(matured.newly_unreadable.size(), 1u);
  EXPECT_EQ(matured.newly_unreadable[0], "/a");
  auto still = t.update(M{match("a", "/a", std::string(""))});
  EXPECT_TRUE(still.newly_unreadable.empty()) << "a continuing unreadable node must not re-announce as new";
}

TEST(LifecycleExpectation, NewlyNotManagedNamesOnlyTheNodeThatMaturedThisTick) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/5);
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks; ++i) {
    auto below = t.update(M{match("a", "/a", std::nullopt)});
    EXPECT_TRUE(below.newly_not_managed.empty()) << "iteration " << i;
  }
  auto matured = t.update(M{match("a", "/a", std::nullopt)});
  ASSERT_EQ(matured.newly_not_managed.size(), 1u);
  EXPECT_EQ(matured.newly_not_managed[0], "/a");
  auto still = t.update(M{match("a", "/a", std::nullopt)});
  EXPECT_TRUE(still.newly_not_managed.empty()) << "a continuing not-managed node must not re-announce as new";
}

// The degenerate case R11 calls out by name: grace=0 makes EVERY node cross on its first
// tick, so "new first" has nothing to distinguish them by. The tie-break must still be
// deterministic - here, the tracker's own internal std::map iteration, i.e. lexicographic
// by fqn. This is not a design choice this slice adds on top; it falls out of `nodes` being
// a std::map in update() - pinning it here so a future change to that container is caught.
TEST(LifecycleExpectation, AllNodesCrossingOnTheSameTickAreAllNewInLexicographicOrder) {
  LifecycleExpectationTracker t({"z", "a", "m"}, /*grace=*/0);
  auto report = t.update(M{match("z", "/z", "inactive"), match("a", "/a", "inactive"), match("m", "/m", "inactive")});
  ASSERT_EQ(report.newly_affected.size(), 3u) << "grace=0: every one of them crosses on its first tick";
  EXPECT_EQ(report.newly_affected, (std::vector<std::string>{"/a", "/m", "/z"}))
      << "when everything is equally new, the tie-break is lexicographic";
}

// Config sweep: the SAME degenerate case again, but through the bare-name (fleet-wide) form
// instead of one-entry-per-node, so "many entries" and "one entry matching many nodes" are
// both covered by the ordering guarantee, not just the latter.
TEST(LifecycleExpectation, AllNodesCrossingViaOneBareNameEntryAreAllNewInLexicographicOrder) {
  LifecycleExpectationTracker t({"controller_server"}, /*grace=*/0);
  auto report = t.update(M{match("controller_server", "/right/controller_server", "inactive"),
                           match("controller_server", "/left/controller_server", "inactive")});
  ASSERT_EQ(report.newly_affected.size(), 2u);
  EXPECT_EQ(report.newly_affected, (std::vector<std::string>{"/left/controller_server", "/right/controller_server"}));
}

// Scale: many nodes already crossed and reported in an earlier tick, then one MORE crosses
// on a later tick. Only the fresh one is new - the earlier batch, however large, must not
// reappear.
TEST(LifecycleExpectation, ANodeCrossingLaterIsNamedNewEvenWithManyEarlierViolations) {
  constexpr int kEarlierCount = 24;
  std::set<std::string> entries;
  M first_batch;
  for (int i = 0; i < kEarlierCount; ++i) {
    const std::string fqn = "/a" + std::to_string(i);
    entries.insert(fqn);
    first_batch.push_back(match(fqn, fqn, "inactive"));
  }
  entries.insert("/z");
  LifecycleExpectationTracker t(entries, /*grace=*/0);
  auto batch = t.update(first_batch);  // all kEarlierCount cross together
  ASSERT_EQ(batch.newly_affected.size(), static_cast<std::size_t>(kEarlierCount));

  M second_batch = first_batch;
  second_batch.push_back(match("/z", "/z", "inactive"));  // "/z" joins on this tick
  auto later = t.update(second_batch);
  ASSERT_EQ(later.newly_affected.size(), 1u) << "only the fresh node crosses grace on this tick";
  EXPECT_EQ(later.newly_affected[0], "/z");
  EXPECT_EQ(later.affected.size(), static_cast<std::size_t>(kEarlierCount) + 1)
      << "sanity: the earlier batch is still reported, just not as new";
}

// Change over time: a node LEAVING affected and another ENTERING on the same tick must be
// handled independently.
TEST(LifecycleExpectation, ANodeLeavingAndAnotherEnteringOnTheSameTickAreIndependent) {
  LifecycleExpectationTracker t({"a", "b"}, /*grace=*/0);
  auto first = t.update(M{match("a", "/a", "inactive")});  // "a" crosses tick 1
  ASSERT_EQ(first.newly_affected.size(), 1u);

  auto mixed = t.update(M{match("a", "/a", "active"), match("b", "/b", "inactive")});
  EXPECT_FALSE(mixed.affected.count("/a")) << "\"a\" healed and must not be reported";
  ASSERT_TRUE(mixed.affected.count("/b"));
  ASSERT_EQ(mixed.newly_affected.size(), 1u) << "only the entering node is new";
  EXPECT_EQ(mixed.newly_affected[0], "/b");
}

// ---- R13: the remote-supplied label gets its own budget, before a whole-detail backstop ----

TEST(LifecycleExpectation, PathologicallyLongLabelIsTrimmedNotTheRestOfTheDetail) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  const std::string garbage_label(5000, 'x');  // no real lifecycle implementation ever sends this
  auto report = t.update(M{match("a", "/a", garbage_label)});
  ASSERT_EQ(report.affected.size(), 1u);
  const std::string & detail = report.affected.at("/a");
  EXPECT_LE(detail.size(), kMaxLifecycleDetailChars) << "the whole-detail backstop must bound it regardless";
  EXPECT_NE(detail.find("/a"), std::string::npos) << "the node name must survive the trim";
  EXPECT_NE(detail.find("required by 'a'"), std::string::npos) << "the naming entry must survive the trim";
  EXPECT_EQ(detail.find(garbage_label), std::string::npos) << "the raw untrimmed label must not appear whole";
}

TEST(LifecycleExpectation, LabelAloneIsTrimmedEvenWhenTheWholeDetailWouldFitTheBackstop) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  const std::string long_label(kMaxLifecycleLabelChars + 20, 'x');
  auto report = t.update(M{match("a", "/a", long_label)});
  ASSERT_EQ(report.affected.size(), 1u);
  const std::string & detail = report.affected.at("/a");
  EXPECT_LT(detail.size(), kMaxLifecycleDetailChars)
      << "sanity: fqn+entry are short here, so a total under the backstop proves the LABEL budget "
         "did the trimming, not the whole-detail one";
  EXPECT_NE(detail.find("..."), std::string::npos) << "the oversized label must be visibly truncated";
}

TEST(LifecycleExpectation, PathologicallyLongFqnIsCappedByTheDetailBackstop) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  const std::string huge_fqn = "/" + std::string(2000, 'n');
  auto report = t.update(M{match("a", huge_fqn, "inactive")});
  ASSERT_EQ(report.affected.size(), 1u);
  EXPECT_LE(report.affected.at(huge_fqn).size(), kMaxLifecycleDetailChars)
      << "a pathological fqn must not defeat the per-detail backstop";
}

// The unreadable/not-managed detail builders re-apply the SAME whole-detail backstop, for
// the same reason: the fqn and the "required by" list are graph- and config-controlled
// length, and neither detail carries a live label to add its own separate budget for.
TEST(LifecycleExpectation, PathologicalFqnOnAMaturedUnreadableNodeIsCappedByTheDetailBackstop) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  const std::string huge_fqn = "/" + std::string(2000, 'n');
  for (int i = 0; i <= kDefaultUnmeasuredHoldTicks; ++i) {
    t.update(M{match("a", huge_fqn, std::string(""))});
  }
  auto report = t.update(M{match("a", huge_fqn, std::string(""))});
  ASSERT_EQ(report.unreadable_affected.size(), 1u);
  EXPECT_LE(report.unreadable_affected.at(huge_fqn).size(), kMaxLifecycleDetailChars);
  EXPECT_EQ(report.unreadable_affected.at(huge_fqn).find("node "), 0u) << "the fixed prefix must survive at the head";
}

TEST(LifecycleExpectation, PathologicalLabelOnOneNodeDoesNotConsumeTheWholeBudget) {
  LifecycleExpectationTracker t({"a", "b"}, /*grace=*/0);
  const std::string garbage_label(5000, 'y');
  auto report = t.update(M{match("a", "/a", garbage_label), match("b", "/b", "inactive")});
  ASSERT_EQ(report.affected.size(), 2u);
  EXPECT_LE(report.affected.at("/a").size(), kMaxLifecycleDetailChars);
  EXPECT_LE(report.affected.at("/a").size() + report.affected.at("/b").size(), 2 * kMaxLifecycleDetailChars);
}

// C1/C3: the test the name above promises. Enough ORDINARY (short, capped-at-length-1)
// nodes to independently exceed AggregatedFault::kMaxDescriptionChars on their own, PLUS one
// pathological-label node, assembled through AggregatedFault::describe exactly as the real
// caller would.
TEST(LifecycleExpectation, PathologicalLabelAlongsideACapFillingBatchDoesNotCrowdOutTheOrdinaryNodes) {
  LifecycleExpectationTracker probe({"n0000"}, /*grace=*/0);
  auto probe_report = probe.update(M{match("n0000", "/n0000", std::string("inactive"))});
  const std::size_t entry_len = probe_report.affected.at("/n0000").size();
  constexpr std::size_t kJoinSep = 2;  // "; " - AggregatedFault::describe_ordered's join separator
  const std::size_t cap = AggregatedFault::kMaxDescriptionChars;
  std::size_t ordinary_count = 1;
  while (ordinary_count * entry_len + (ordinary_count - 1) * kJoinSep <= cap) {
    ++ordinary_count;
  }

  std::set<std::string> entries;
  M matches;
  std::vector<std::string> ordinary_fqns;
  for (std::size_t i = 0; i < ordinary_count; ++i) {
    std::string suffix = std::to_string(i);
    suffix.insert(0, 4 - std::min<std::size_t>(4, suffix.size()), '0');  // 4-digit zero pad
    const std::string id = "n" + suffix;
    entries.insert(id);
    ordinary_fqns.push_back("/" + id);
    matches.push_back(match(id, "/" + id, std::string("inactive")));
  }
  // 'A' (0x41) sorts before 'n' (0x6E), so this entry is always first in describe()'s
  // lexicographic join order - not by luck, by construction.
  entries.insert("AAA_pathological");
  matches.push_back(match("AAA_pathological", "/AAA_pathological", std::string(5000, 'z')));

  LifecycleExpectationTracker t(entries, /*grace=*/0);
  auto report = t.update(matches);
  ASSERT_EQ(report.affected.size(), ordinary_count + 1);
  const std::string & pathological_entry = report.affected.at("/AAA_pathological");
  ASSERT_LE(pathological_entry.size(), kMaxLifecycleDetailChars)
      << "sanity: the whole-detail backstop must bound the pathological entry regardless";
  EXPECT_LT(pathological_entry.size(), kMaxLifecycleDetailChars)
      << "the pathological entry landed exactly ON the whole-detail cap instead of comfortably "
         "under it - the label trim did not run";
  EXPECT_NE(pathological_entry.find("required by 'AAA_pathological'"), std::string::npos)
      << "the \"required by\" suffix was cut off the pathological entry";

  const std::string desc = AggregatedFault::describe(report.affected);
  std::size_t named = 0;
  for (const auto & fqn : ordinary_fqns) {
    if (desc.find(fqn) != std::string::npos) {
      ++named;
    }
  }
  EXPECT_GT(named, 0u) << "every ordinary node's name was crowded out of the final description "
                          "by the pathological node's entry";
}

// ---- Pruning and bounded bookkeeping ----

// An entry currently reported (stuck inactive past grace) must never be pruned - it is
// present, not absent, so it can never accumulate absence in the first place.
TEST(LifecycleExpectation, ReportedInactiveEntryIsNeverPruned) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  t.update(M{match("a", "/a", "inactive")});                // miss 1 <= grace
  auto report = t.update(M{match("a", "/a", "inactive")});  // miss 2 > grace -> reported
  ASSERT_FALSE(report.affected.empty());
  for (int i = 0; i < 10; ++i) {
    report = t.update(M{match("a", "/a", "inactive")});  // present + inactive the whole time
    EXPECT_FALSE(report.affected.empty()) << "iteration " << i;
  }
  EXPECT_EQ(t.tracked_count(), 1u) << "a currently-reported node must never be pruned away";
}

// The prune bound reclaims the WHOLE node atomically - one map, one entry, one horizon -
// but only an IDLE one. A node measured ACTIVE and then gone carries nothing, so reclaiming
// it loses nothing; reclaiming the entry reclaims everything it held, in the same tick.
TEST(LifecycleExpectation, PruneTicksReclaimsTheWholeIdleNodeAtomically) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  t.update(M{match("a", "/a", "inactive")});
  t.update(M{match("a", "/a", "active")});  // healed: the entry is now idle
  for (int i = 0; i < 6; ++i) {
    t.update(M{});  // absent far past prune_ticks(2)
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "the idle node must be fully reclaimed, not partially";
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty())
      << "nothing may have survived the reclaim - the returning node was reported on its first "
         "present-and-inactive tick instead of after grace + 1, which would mean a stale streak "
         "outlived the entry that was supposed to carry it";
}

// The other half of that rule, and the one that matters: a node carrying EVIDENCE is never
// reclaimed by age, at any horizon, however long it stays gone. prune_ticks SHORTER than
// the absence grace is reachable through the documented config (prune_grace: 2 is one under
// the default absence grace of 3), which is exactly where an age horizon used to erase a
// clock before the absence rules ever saw it.
TEST(LifecycleExpectation, ReportedNodeIsNeverPrunedEvenWhenThePruneBoundUndercutsTheAbsenceGrace) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, kDefaultAbsenceGrace, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  t.update(M{match("a", "/a", "inactive")});                  // 1 <= grace
  auto reported = t.update(M{match("a", "/a", "inactive")});  // 2 > grace: reported
  ASSERT_FALSE(reported.affected.empty());

  for (int i = 1; i <= 10; ++i) {
    auto absent = t.update(M{});  // well past prune_ticks(2) AND past the absence grace
    EXPECT_EQ(absent.affected.count("/a"), 1u)
        << "absent tick " << i << ": the node's confirmed violation was erased by the prune horizon";
    EXPECT_EQ(t.tracked_count(), 1u) << "absent tick " << i;
  }
}

// Scale: many reported nodes blinking on the same tick. Content follows the clocks, not the
// snapshot, so every one of them stays reported rather than dropping into a withheld limbo.
TEST(LifecycleExpectation, ManyReportedNodesBlinkingTogetherAllStayReported) {
  constexpr int kNodeCount = 20;
  std::set<std::string> entries;
  M matches;
  std::vector<std::string> fqns;
  for (int i = 0; i < kNodeCount; ++i) {
    const std::string fqn = "/n" + std::to_string(i);
    entries.insert(fqn);
    fqns.push_back(fqn);
    matches.push_back(match(fqn, fqn, "inactive"));
  }
  LifecycleExpectationTracker t(entries, /*grace=*/1, /*absence_grace=*/3);
  t.update(matches);                  // 1 <= grace, all of them
  auto reported = t.update(matches);  // 2 > grace, all reported
  ASSERT_EQ(reported.affected.size(), static_cast<std::size_t>(kNodeCount));

  auto blink = t.update(M{});  // every one of them vanishes from the snapshot on the same tick
  ASSERT_EQ(blink.affected.size(), static_cast<std::size_t>(kNodeCount))
      << "every one of them must keep its content, not just however many a description names";
  EXPECT_TRUE(blink.pending.empty()) << "a node still in `affected` has nothing left to withhold for";
  for (const auto & fqn : fqns) {
    EXPECT_EQ(blink.affected.count(fqn), 1u) << fqn;
    EXPECT_TRUE(blink.newly_affected.empty()) << "a blink is not a fresh crossing";
  }
}

// Change: one node crosses grace on the SAME tick another is deep inside its own absence.
TEST(LifecycleExpectation, NodeCrossingGraceDoesNotDisturbAnothersAbsenceBookkeeping) {
  LifecycleExpectationTracker t({"a", "b"}, /*grace=*/1, /*absence_grace=*/3);
  t.update(M{match("b", "/b", "inactive")});                    // b: 1 <= grace
  auto b_reported = t.update(M{match("b", "/b", "inactive")});  // b: 2 > grace -> reported
  ASSERT_TRUE(b_reported.affected.count("/b"));

  auto mixed = t.update(M{match("a", "/a", "inactive")});  // a: 1 <= grace; b: absent 1
  EXPECT_TRUE(mixed.affected.count("/a") == 0) << "a is only at 1 <= grace";
  EXPECT_EQ(mixed.affected.count("/b"), 1u) << "b, already reported, keeps its content through the blink";
  EXPECT_EQ(mixed.pending.count("/a"), 1u) << "a's own below-grace streak";
  EXPECT_TRUE(mixed.newly_affected.empty()) << "nothing crossed on this tick";

  auto crossed = t.update(M{match("a", "/a", "inactive")});  // a: 2 > grace -> reported; b: absent 2
  ASSERT_EQ(crossed.affected.size(), 2u) << "a joins b, which never left";
  ASSERT_EQ(crossed.newly_affected.size(), 1u) << "only a crossed grace on this tick";
  EXPECT_EQ(crossed.newly_affected[0], "/a");
}

// Sustained absence CONTINUES an ALREADY-MATURED streak instead of discarding it: the node
// was CONFIRMED not-active, nothing has said otherwise, and being gone is not an answer. It
// does not create one that was not already there - see
// BelowGraceStreakStaysPendingForeverWhileAbsentAndNeverBecomesContent for the below-grace
// half of the same absence branch.
TEST(LifecycleExpectation, MaturedStreakSurvivesAbsenceUnconditionally) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/1);
  t.update(M{match("a", "/a", "inactive")});                   // streak 1 <= grace
  auto confirmed = t.update(M{match("a", "/a", "inactive")});  // streak 2 > grace: confirmed
  ASSERT_EQ(confirmed.affected.count("/a"), 1u) << "sanity: must be confirmed before it can survive a departure";

  t.update(M{});  // absent 1 == absence_grace: held through the blink, unchanged
  for (int i = 0; i < 10; ++i) {
    auto still = t.update(M{});  // absent 2.. > absence_grace
    ASSERT_EQ(still.affected.count("/a"), 1u)
        << "iteration " << i
        << ": absence discarded an already-confirmed streak instead of continuing "
           "it, so a node that leaves while violating is not confirmed any more";
    EXPECT_TRUE(still.newly_affected.empty()) << "iteration " << i << ": no re-raise churn while merely absent";
  }
  auto report = t.update(M{});
  const std::string & detail = report.affected.at("/a");
  EXPECT_NE(detail.find("inactive"), std::string::npos)
      << "the detail must still name the state the node was last measured in";
  EXPECT_NE(detail.find("has since left the graph"), std::string::npos);
}

// The other half: a streak that had NOT yet crossed grace when the node left is held at
// whatever it already reached, resuming rather than restarting. Counted exactly - a streak
// that had restarted at zero would still read `grace` (not past it) after these same three
// return ticks, one short of the four a fresh climb needs.
TEST(LifecycleExpectation, BelowGraceStreakHeldByAbsenceResumesRatherThanRestartsOnReturn) {
  constexpr int kGrace = 3;
  LifecycleExpectationTracker t({"a"}, kGrace, /*absence_grace=*/1);
  auto first = t.update(M{match("a", "/a", "inactive")});  // streak 1 <= grace
  ASSERT_EQ(first.pending_violation.count("/a"), 1u);

  for (int i = 0; i < 30; ++i) {
    t.update(M{});  // absent, well past absence_grace: held at streak 1 the whole time
  }

  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "resumed streak 2 <= grace";
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "resumed streak 3 == grace";
  auto confirmed = t.update(M{match("a", "/a", "inactive")});  // resumed streak 4 > grace
  ASSERT_EQ(confirmed.affected.count("/a"), 1u)
      << "the streak restarted from zero on return instead of resuming where the absence had held it";
  EXPECT_EQ(confirmed.newly_affected, (std::vector<std::string>{"/a"}));
}

// `pending` NEVER becomes content merely by waiting: a below-grace streak that goes absent
// stays exactly where it was for as long as the node is gone, so the withheld-clear hold has
// no timeout of its own - it ends only when the tracker actually SETTLES the node's status (a
// fresh present tick reading active or inactive), never because enough absent ticks went by.
// Maturing a below-grace streak on absence alone would raise GRAPH_NODE_INACTIVE from ticks
// gathered while nobody could observe the node at all.
TEST(LifecycleExpectation, BelowGraceStreakStaysPendingForeverWhileAbsentAndNeverBecomesContent) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/3, /*absence_grace=*/2);
  EXPECT_EQ(t.update(M{match("a", "/a", "inactive")}).pending.count("/a"), 1u);  // streak 1 <= grace

  for (int i = 0; i < 50; ++i) {
    auto report = t.update(M{});  // absent, past absence_grace for every iteration but the first two
    EXPECT_EQ(report.pending.count("/a"), 1u) << "iteration " << i
                                              << ": the hold ended without the node ever being measured active or "
                                                 "confirmed inactive";
    EXPECT_EQ(report.pending_violation.count("/a"), 1u) << "iteration " << i;
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i
                                         << ": a below-grace streak matured purely from waiting while absent";
  }
}

// A node the detector already reported keeps its content through a blink AND past it - it
// is never silently cleared at any point, which is what a level-triggered emitter needs to
// avoid a raise/clear/raise churn on a node that is simply gone.
TEST(LifecycleExpectation, ReportedNodeKeepsItsContentThroughAndPastTheAbsenceGrace) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/2);
  t.update(M{match("a", "/a", "inactive")});                  // 1 <= grace
  auto reported = t.update(M{match("a", "/a", "inactive")});  // 2 > grace: reported
  ASSERT_FALSE(reported.affected.empty()) << "sanity: the node must already be reported before it blinks";

  for (int i = 1; i <= 8; ++i) {
    auto absent = t.update(M{});
    EXPECT_EQ(absent.affected.count("/a"), 1u) << "absent tick " << i << ": content was dropped";
    EXPECT_TRUE(absent.pending.empty()) << "absent tick " << i << ": content and a withhold are exclusive";
    EXPECT_TRUE(absent.newly_affected.empty()) << "absent tick " << i << ": no re-announcement as new";
  }
}

// R10: the re-raise on return is CORRECT and must be preserved, not "fixed".
TEST(LifecycleExpectation, ReportedNodeReRaisesOnReturnWithoutReEarningGrace) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3);
  t.update(M{match("a", "/a", "inactive")});                  // 1 <= grace
  auto reported = t.update(M{match("a", "/a", "inactive")});  // 2 > grace: reported
  ASSERT_FALSE(reported.affected.empty());

  t.update(M{});  // absent 1: held, not cleared

  auto back = t.update(M{match("a", "/a", "inactive")});  // still inactive on return
  EXPECT_FALSE(back.affected.empty()) << "the streak survived the blink - no fresh grace to re-earn";
  EXPECT_TRUE(back.pending.empty()) << "past grace again immediately, not a fresh below-grace streak";
}

// An IDLE node absent past prune_ticks is reclaimed and the map shrinks - the age horizon
// still does its job, on the only entries that have nothing to lose.
TEST(LifecycleExpectation, IdleNodeAbsentPastPruneTicksIsPrunedAndMapShrinks) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/3);
  t.update(M{match("a", "/a", "active")});  // measured healthy: idle from the first tick
  EXPECT_EQ(t.tracked_count(), 1u);
  t.update(M{});  // absent 1
  t.update(M{});  // absent 2
  t.update(M{});  // absent 3 (== prune_ticks, not past it yet)
  EXPECT_EQ(t.tracked_count(), 1u);
  t.update(M{});  // absent 4 (> prune_ticks) -> reclaimed
  EXPECT_EQ(t.tracked_count(), 0u) << "an idle node absent past prune_ticks must be reclaimed";
}

// ---- The tracked-node cap: what bounds the map now that evidence outlives absence ----

// Identity churn where every respawn is HEALTHY: each entry goes idle immediately, so the
// age horizon alone keeps the map small and the cap is never even approached.
TEST(LifecycleExpectation, ChurningHealthyIdentitiesAreReclaimedByTheAgeHorizon) {
  constexpr int kPruneTicks = 2;
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/1, kDefaultNoMatchWarnTicks, kPruneTicks);
  for (int i = 0; i < 50; ++i) {
    t.update(M{match("a", "/ns" + std::to_string(i) + "/a", "active")});
    EXPECT_LE(t.tracked_count(), static_cast<std::size_t>(kPruneTicks) + 1)
        << "iteration " << i << ": an idle entry is retained for every fqn ever seen";
  }
  for (int i = 0; i < 5; ++i) {
    t.update(M{});
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "every churned identity must be reclaimed once it is gone";
}

// Identity churn where every respawn is VIOLATING: nothing is ever idle, so the age horizon
// reclaims nothing (by design - that horizon is what a restart loop evaded) and the CAP is
// the only bound. The PRESENT node always wins a slot: the entries for the identities that
// are GONE are collapsed into a count instead, so the one node actually in the graph is
// always checked and nothing is refused. The count keeps the fault's content non-empty, so
// freeing those slots heals nothing.
TEST(LifecycleExpectation, ChurningViolatingIdentitiesAreCollapsedSoThePresentNodeIsAlwaysTracked) {
  constexpr int kCap = 8;
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2, kDefaultUnmeasuredHoldTicks, kCap);
  int saturations = 0;
  for (int i = 0; i < 50; ++i) {
    const std::string fqn = "/ns" + std::to_string(i) + "/a";
    auto report = t.update(M{match("a", fqn, "inactive")});
    saturations += report.tracking_saturated ? 1 : 0;
    ASSERT_LE(t.tracked_count(), static_cast<std::size_t>(kCap))
        << "iteration " << i << ": the map grew past the cap, so nothing bounds it at all";
    ASSERT_EQ(report.affected.count(fqn), 1u)
        << "iteration " << i
        << ": the PRESENT violating node was not reported - a cap full of entries for nodes that are "
           "GONE refused the only node actually in the graph, so the detector reports health it "
           "declined to check";
  }
  EXPECT_EQ(saturations, 0)
      << "a present node was refused while the cap held entries for departed nodes, which can never "
         "become idle again - the refusal would last for the life of the process";

  // Nothing was thrown away to make that room: the departed identities are still content,
  // as a count.
  auto steady = t.update(M{});
  ASSERT_FALSE(steady.affected.empty()) << "collapsing the departed entries healed the fault outright";
  const std::string description = AggregatedFault::describe(steady.affected);
  EXPECT_NE(description.find("more required node(s) left the graph"), std::string::npos)
      << "the collapsed identities left no trace in what the operator reads, so their evidence was "
         "silently discarded: "
      << description;
}

// The other side of the same cap: when every tracked node is PRESENT there is nothing to
// collapse, so the newcomer genuinely IS refused - and that must be reported on every such
// tick, not once, because the caller has to withhold GRAPH_NODE_INACTIVE's clear for as long
// as a required node is going unchecked.
TEST(LifecycleExpectation, AllPresentAtTheCapRefusesTheNewcomerAndSaysSoOnEveryTick) {
  constexpr int kCap = 2;
  LifecycleExpectationTracker t({"/a", "/b", "/c"}, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  const M all{match("/a", "/a", "inactive"), match("/b", "/b", "inactive"), match("/c", "/c", "inactive")};
  for (int i = 0; i < 5; ++i) {
    auto report = t.update(all);
    EXPECT_TRUE(report.tracking_saturated)
        << "tick " << i << ": a required node is present and going unchecked, and the report does not say so";
    EXPECT_EQ(report.saturation_started, i == 0)
        << "tick " << i << ": the saturation EDGE must be the first tick of the episode, and only that one";
    EXPECT_EQ(t.tracked_count(), static_cast<std::size_t>(kCap));
    EXPECT_EQ(report.affected.count("/c"), 0u) << "the refused node must not be reported as measured";
  }

  // The episode ends when the refused node leaves, and a LATER one is reported again rather
  // than silenced by the first.
  const M two{match("/a", "/a", "inactive"), match("/b", "/b", "inactive")};
  auto ended = t.update(two);
  EXPECT_FALSE(ended.tracking_saturated) << "nothing was refused, so nothing is saturated";
  auto recurred = t.update(all);
  EXPECT_TRUE(recurred.tracking_saturated);
  EXPECT_TRUE(recurred.saturation_started)
      << "a second, real saturation was not surfaced because the first episode spent the latch";
}

// At the cap, IDLE entries are reclaimed FIRST - they carry nothing, so freeing them costs
// nothing and the newcomer gets in without anything being refused.
TEST(LifecycleExpectation, IdleEntriesAreReclaimedFirstAtTheCapSoTheNewcomerIsAdmitted) {
  constexpr int kCap = 4;
  std::set<std::string> entries{"/keep", "/idle0", "/idle1", "/idle2", "/newcomer"};
  LifecycleExpectationTracker t(entries, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  // One entry carrying a confirmed violation, three measured healthy: the map is exactly full.
  t.update(M{match("/keep", "/keep", "inactive"), match("/idle0", "/idle0", "active"),
             match("/idle1", "/idle1", "active"), match("/idle2", "/idle2", "active")});
  ASSERT_EQ(t.tracked_count(), static_cast<std::size_t>(kCap));

  auto report = t.update(M{match("/newcomer", "/newcomer", "inactive")});
  EXPECT_FALSE(report.tracking_saturated) << "idle entries were available - nothing should have been refused";
  EXPECT_EQ(report.affected.count("/newcomer"), 1u) << "the newcomer was refused despite reclaimable idle entries";
  EXPECT_EQ(report.affected.count("/keep"), 1u) << "the entry carrying evidence must survive the reclaim";
  EXPECT_LE(t.tracked_count(), static_cast<std::size_t>(kCap));
}

// Scale, at the REAL shipped cap rather than a test-shrunk one: kDefaultTrackedNodeCap
// violating identities fit, and the very next one is refused.
TEST(LifecycleExpectation, TheShippedCapAdmitsExactlyItsOwnCountOfViolatingNodes) {
  std::set<std::string> entries;
  M matches;
  for (int i = 0; i < kDefaultTrackedNodeCap; ++i) {
    const std::string fqn = "/n" + std::to_string(i);
    entries.insert(fqn);
    matches.push_back(match(fqn, fqn, "inactive"));
  }
  entries.insert("/one_too_many");
  LifecycleExpectationTracker t(entries, /*grace=*/0);
  auto full = t.update(matches);
  ASSERT_EQ(full.affected.size(), static_cast<std::size_t>(kDefaultTrackedNodeCap));
  EXPECT_FALSE(full.tracking_saturated) << "exactly the cap must fit without saturating";

  matches.push_back(match("/one_too_many", "/one_too_many", "inactive"));
  auto over = t.update(matches);
  EXPECT_TRUE(over.tracking_saturated) << "one node past the cap must be refused, and said so";
  EXPECT_EQ(over.affected.count("/one_too_many"), 0u) << "the refused node must not be reported";
  EXPECT_EQ(t.tracked_count(), static_cast<std::size_t>(kDefaultTrackedNodeCap));
}

// ---- Settling: what absence is allowed to continue ----

// The transient the settling rule exists for: a present, HEALTHY, managed node whose
// get_state path is missing from one sweep reads "no tracked lifecycle" for a tick. If that
// is the last thing seen before a clean shutdown, continuing it literally matures a healthy
// departure into a permanent fault. Swept across every uncorroborated run length, so an
// off-by-one at the bound is caught rather than one arbitrary length being pinned.
TEST(LifecycleExpectation, UncorroboratedUnmeasuredRunBeforeAHealthyDepartureRaisesNothing) {
  for (int blink = 1; blink < kDefaultObservationSettleTicks; ++blink) {
    LifecycleExpectationTracker t({"a"}, /*grace=*/1);
    for (int i = 0; i < 10; ++i) {
      t.update(M{match("a", "/a", "active")});  // measured healthy, over and over
    }
    for (int i = 0; i < blink; ++i) {
      t.update(M{match("a", "/a", std::nullopt)});  // the missed sweep(s)
    }
    for (int i = 0; i < kDefaultUnmeasuredHoldTicks + kDefaultAbsenceGrace + 5; ++i) {
      auto report = t.update(M{});  // and then it is gone, cleanly
      ASSERT_TRUE(report.not_managed_affected.empty())
          << "blink=" << blink << ", absent tick " << i
          << ": a healthy managed node whose lifecycle services were missing from " << blink
          << " sweep(s) before a clean shutdown was reported as not managed";
      ASSERT_TRUE(report.unreadable_affected.empty()) << "blink=" << blink;
      ASSERT_TRUE(report.affected.empty()) << "blink=" << blink;
    }
    EXPECT_TRUE(t.update(M{}).pending.empty())
        << "blink=" << blink
        << ": the released node is still UNSETTLED, so it withholds GRAPH_NODE_INACTIVE's clear for "
           "every other node for the life of the process";
  }
}

// The other side of the same bound, and the reason it cannot simply be "ignore unmeasured
// readings before a departure": a node that is GENUINELY unmeasurable when it leaves must
// still be reported. Same shape as the test above, one tick longer.
TEST(LifecycleExpectation, CorroboratedUnmeasuredRunBeforeADepartureIsStillReported) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1);
  for (int i = 0; i < 10; ++i) {
    t.update(M{match("a", "/a", "active")});
  }
  for (int i = 0; i < kDefaultObservationSettleTicks; ++i) {
    t.update(M{match("a", "/a", std::nullopt)});
  }
  bool reported = false;
  for (int i = 0; i < kDefaultUnmeasuredHoldTicks + kDefaultAbsenceGrace + 5 && !reported; ++i) {
    reported = !t.update(M{}).not_managed_affected.empty();
  }
  EXPECT_TRUE(reported) << "a node observed not-managed for exactly the settling budget before it left "
                           "the graph was never reported - corroboration is not supposed to be a way "
                           "for a genuinely unmeasurable node to leave quietly";
}

// A real measurement needs no corroboration at all: one not-active read is a fact about the
// node, so it is content the instant it is read (grace: 0 makes that one tick the crossing
// tick), and a node that departs immediately after keeps that content unconditionally, the
// same as any other already-matured entry. Without instant settling for a real measurement,
// the corroboration rule built for the unmeasured clock would swallow the very case grace: 0
// exists for. grace: 0 also keeps this test meaningful now that absence alone may not mature
// a below-grace streak: at any grace > 0 a single read leaves the streak BELOW grace, and a
// below-grace streak is now held rather than
// confirmed by a departure - see BelowGraceStreakStaysPendingForeverWhileAbsentAndNever
// BecomesContent for that (deliberately different) claim.
TEST(LifecycleExpectation, OneMeasuredNotActiveReadBeforeADepartureStillConfirms) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0);
  auto confirmed_before_leaving = t.update(M{match("a", "/a", "inactive")});  // grace 0: confirmed on this tick
  ASSERT_EQ(confirmed_before_leaving.affected.count("/a"), 1u)
      << "sanity: the node must already be confirmed before it departs, or nothing below is tested";

  for (int i = 0; i < kDefaultAbsenceGrace + 10; ++i) {
    auto report = t.update(M{});  // gone, immediately
    ASSERT_EQ(report.affected.count("/a"), 1u)
        << "iteration " << i
        << ": a node measured not-active once, already confirmed, and then gone lost "
           "its fault - a lifecycle label is a measurement, not something a sweep can invent, so it needs "
           "no corroborating, and a departure must not un-confirm it either";
  }
}

// ---- The cap: a present node always wins a slot ----

// A cap held entirely by entries for DEPARTED nodes must not refuse a PRESENT one. Those
// entries can never become idle again (becoming idle needs a real measurement of a node that
// is gone), so without collapsing them the refusal lasts for the life of the process - and a
// refused node never enters `affected` or `pending`, which makes GRAPH_NODE_INACTIVE emit a
// level-triggered CLEAR every tick while that node reads not-active.
TEST(LifecycleExpectation, DepartedEntriesAreCollapsedSoAPresentBrokenNodeIsStillReported) {
  constexpr int kCap = 2;
  LifecycleExpectationTracker t({"/d0", "/d1", "/live"}, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  t.update(M{match("/d0", "/d0", "inactive"), match("/d1", "/d1", "inactive")});
  ASSERT_EQ(t.tracked_count(), 2u) << "the cap must be full of departed-to-be entries, or nothing is tested";
  for (int i = 0; i < 3; ++i) {
    t.update(M{});  // both leave for good, past the absence grace
  }

  auto report = t.update(M{match("/live", "/live", "inactive")});
  EXPECT_FALSE(report.tracking_saturated)
      << "a present, broken, required node was refused by entries for two nodes that are gone";
  EXPECT_EQ(report.affected.count("/live"), 1u)
      << "the present node was admitted but not reported - it is the one node actually in the graph";
  // And the departed evidence survived being collapsed, as content.
  EXPECT_NE(AggregatedFault::describe(report.affected).find("more required node(s) left the graph"), std::string::npos)
      << "collapsing the departed entries discarded their evidence: " << AggregatedFault::describe(report.affected);
}

// The bound on how many departed entries stay individually NAMED. Three, because three
// maximally-long details are all one description holds - a fourth name could never be shown
// however the ordering fell out, so keeping it only costs a slot a present node may need.
TEST(LifecycleExpectation, AtMostThreeDepartedEntriesStayNamedWhenTheCapMakesRoom) {
  constexpr int kCap = 8;
  std::set<std::string> entries;
  M departing;
  for (int i = 0; i < kCap; ++i) {
    const std::string fqn = "/d" + std::to_string(i);
    entries.insert(fqn);
    departing.push_back(match(fqn, fqn, "inactive"));
  }
  entries.insert("/live");
  LifecycleExpectationTracker t(entries, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  t.update(departing);
  ASSERT_EQ(t.tracked_count(), static_cast<std::size_t>(kCap));
  for (int i = 0; i < 3; ++i) {
    t.update(M{});  // every one of them leaves for good
  }

  t.update(M{match("/live", "/live", "inactive")});
  EXPECT_EQ(t.tracked_count(), static_cast<std::size_t>(kMaxNamedDepartedEntries) + 1)
      << "the departed entries were not collapsed down to the named bound, so they go on holding "
         "slots that can never appear in any description";
}

// Change in the other direction: a node whose entry was collapsed and which then COMES BACK.
// It is tracked and measured afresh - the tracker no longer knows which fqn it collapsed - and
// the count that stands in for the identity it lost keeps the fault raised. That is the
// documented behaviour, not an accident, so it is pinned rather than left to be discovered.
TEST(LifecycleExpectation, ANodeReturningAfterItsEntryWasCollapsedIsMeasuredAfresh) {
  constexpr int kCap = 1;
  LifecycleExpectationTracker t({"/gone", "/live"}, /*grace=*/0, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  t.update(M{match("/gone", "/gone", "inactive")});
  for (int i = 0; i < 3; ++i) {
    t.update(M{});  // "/gone" leaves for good, carrying a confirmed violation
  }
  auto collapsed = t.update(M{match("/live", "/live", "inactive")});  // its slot goes to the live node
  ASSERT_EQ(collapsed.affected.count("/live"), 1u);

  // "/gone" comes back HEALTHY. It cannot have the slot ("/live" is present and broken), and
  // even once it does it starts from zero - nothing remembers the entry that was collapsed.
  auto returned = t.update(M{match("/live", "/live", "active"), match("/gone", "/gone", "active")});
  EXPECT_EQ(returned.affected.count("/gone"), 0u) << "a returning node was reported without being measured";
  EXPECT_NE(AggregatedFault::describe(returned.affected).find("more required node(s) left the graph"),
            std::string::npos)
      << "the collapsed count was dropped when the node came back, so a fault that a departure must "
         "never heal was healed by one: "
      << AggregatedFault::describe(returned.affected);
}

// The one cap/collapse shape no OTHER test in this file can tell apart: every sibling above
// uses grace=0, where violation_streak > 0 and violation_streak > grace_ are the SAME
// condition, so they cannot distinguish count_collapsed() counting "any non-zero streak" from
// counting "only an already-matured one". Here grace is wide enough that the departed entry
// is genuinely BELOW it when the cap forces its collapse: under the design this replaces,
// absence kept advancing it regardless, so folding it into collapsed_inactive_ merely
// anticipated a maturity it would have reached anyway. That is no longer true now that
// absence alone never matures a below-grace streak - so collapsing it as content would
// fabricate a violation the node never earned.
TEST(LifecycleExpectation, BelowGraceDepartedEntryCollapsedAtTheCapContributesNothingToTheCount) {
  constexpr int kCap = 1;
  LifecycleExpectationTracker t({"/held", "/live"}, /*grace=*/5, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                LifecycleExpectationTracker::kNoPrune, kDefaultUnmeasuredHoldTicks, kCap);
  t.update(M{match("/held", "/held", "inactive")});  // streak 1, well below grace(5)
  ASSERT_EQ(t.tracked_count(), 1u);
  for (int i = 0; i < 3; ++i) {
    t.update(M{});  // "/held" leaves for good, past the absence grace - HELD, never matures
  }

  // "/live" needs the only slot. "/held" is departed, so it is collapsed rather than "/live"
  // being refused - but "/held" was never content, so nothing may be fabricated for it.
  auto report = t.update(M{match("/live", "/live", "inactive")});
  ASSERT_EQ(t.tracked_count(), 1u) << "the slot was never freed - a departed entry blocked a present node";
  EXPECT_FALSE(report.tracking_saturated) << "a departed, below-grace entry blocked a present node's slot";
  EXPECT_TRUE(report.affected.empty())
      << "the collapsed below-grace entry fabricated a violation it never earned ('/live' itself is "
         "only at streak 1, well below grace(5)): "
      << AggregatedFault::describe(report.affected);
}

// ---- Entries matching nothing: unrelated per-entry mechanism, unaffected by this slice ----

TEST(LifecycleExpectation, EntryMatchingNothingIsReportedOnce) {
  LifecycleExpectationTracker t({"typoed_name"}, /*grace=*/0, /*absence_grace=*/3,
                                /*no_match_warn_ticks=*/2);
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty());  // 1
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty());  // 2 == threshold
  auto report = t.update(M{});                                  // 3 > threshold
  ASSERT_EQ(report.entries_matching_nothing.size(), 1u);
  EXPECT_EQ(report.entries_matching_nothing[0], "typoed_name");
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty()) << "reported once, not every tick";
}

TEST(LifecycleExpectation, EntryThatMatchesIsNeverReportedAsMatchingNothing) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, /*absence_grace=*/3, /*no_match_warn_ticks=*/1);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).entries_matching_nothing.empty());
  }
}
}  // namespace
