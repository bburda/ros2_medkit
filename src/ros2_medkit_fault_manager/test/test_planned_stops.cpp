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

/// Planned-stop window storage, run against BOTH backends.
///
/// Every behavioural assertion is parameterised over the in-memory and the SQLite
/// backend, because the shipped default is SQLite and the in-memory one is what
/// the fast tests use: a rule enforced in only one of them is a rule that holds
/// on nobody's deployment. The SQLite-only fixture below adds what has no
/// in-memory equivalent - surviving a close and reopen.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_manager/time_utils.hpp"

using ros2_medkit_fault_manager::clamp_planned_stop_windows;
using ros2_medkit_fault_manager::DeclarePlannedStopOutcome;
using ros2_medkit_fault_manager::EndPlannedStopOutcome;
using ros2_medkit_fault_manager::FaultStorage;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::kMaxPlannedStopWindows;
using ros2_medkit_fault_manager::kMinPlannedStopWindows;
using ros2_medkit_fault_manager::PlannedStopWindow;
using ros2_medkit_fault_manager::SqliteFaultStorage;

namespace {

constexpr int64_t kSec = 1'000'000'000;

/// A window with sane defaults; the caller overrides what the case is about.
PlannedStopWindow make_window(const std::string & id, int64_t starts_at_ns, int64_t ends_at_ns,
                              int64_t declared_at_ns) {
  PlannedStopWindow w;
  w.id = id;
  w.starts_at_ns = starts_at_ns;
  w.ends_at_ns = ends_at_ns;
  w.reason = "changeover";
  w.declared_by = "operator_a";
  w.declared_at_ns = declared_at_ns;
  return w;
}

std::filesystem::path unique_db_path() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  return std::filesystem::temp_directory_path() / ("test_planned_stops_" + std::to_string(dist(gen)) + ".db");
}

enum class Backend : uint8_t { Memory, Sqlite };

}  // namespace

class PlannedStopStorageTest : public ::testing::TestWithParam<Backend> {
 protected:
  void SetUp() override {
    if (GetParam() == Backend::Sqlite) {
      db_path_ = unique_db_path();
      storage_ = std::make_unique<SqliteFaultStorage>(db_path_.string());
    } else {
      storage_ = std::make_unique<InMemoryFaultStorage>();
    }
  }

  void TearDown() override {
    storage_.reset();
    if (!db_path_.empty()) {
      std::filesystem::remove(db_path_);
      std::filesystem::remove(db_path_.string() + "-wal");
      std::filesystem::remove(db_path_.string() + "-shm");
    }
  }

  std::filesystem::path db_path_;
  std::unique_ptr<FaultStorage> storage_;
};

INSTANTIATE_TEST_SUITE_P(BothBackends, PlannedStopStorageTest, ::testing::Values(Backend::Memory, Backend::Sqlite),
                         [](const ::testing::TestParamInfo<Backend> & param_info) {
                           return param_info.param == Backend::Memory ? "Memory" : "Sqlite";
                         });

TEST_P(PlannedStopStorageTest, DeclareStoresEveryFieldVerbatim) {
  auto w = make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec);
  w.reason = "weekend maintenance";
  w.declared_by = "line_lead";
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored, storage_->declare_planned_stop(w));

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->id, "w1");
  EXPECT_EQ(stored->starts_at_ns, 10 * kSec);
  EXPECT_EQ(stored->ends_at_ns, 20 * kSec);
  EXPECT_EQ(stored->reason, "weekend maintenance");
  EXPECT_EQ(stored->declared_by, "line_lead");
  EXPECT_EQ(stored->declared_at_ns, 5 * kSec);
  EXPECT_FALSE(stored->ended_early);
}

TEST_P(PlannedStopStorageTest, DeclareRefusesADuplicateId) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec)));
  EXPECT_EQ(DeclarePlannedStopOutcome::DuplicateId,
            storage_->declare_planned_stop(make_window("w1", 30 * kSec, 40 * kSec, 25 * kSec)));

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->starts_at_ns, 10 * kSec) << "the second declaration must not overwrite the first";
}

TEST_P(PlannedStopStorageTest, GetUnknownIdIsEmpty) {
  EXPECT_FALSE(storage_->get_planned_stop("never_declared").has_value());
}

TEST_P(PlannedStopStorageTest, ListReturnsNewestDeclarationFirst) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("older", 10 * kSec, 20 * kSec, 1 * kSec)));
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("newer", 30 * kSec, 40 * kSec, 2 * kSec)));

  auto all = storage_->list_planned_stops();
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].id, "newer");
  EXPECT_EQ(all[1].id, "older");
}

// The closed interval is the contract the gateway derives `expected` from, so it
// is asserted at the boundary and one nanosecond outside it on both sides.
TEST_P(PlannedStopStorageTest, CoversIsClosedAtBothEnds) {
  const auto w = make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec);
  EXPECT_TRUE(w.covers(10 * kSec)) << "exactly at the start";
  EXPECT_TRUE(w.covers(20 * kSec)) << "exactly at the end";
  EXPECT_TRUE(w.covers(15 * kSec));
  EXPECT_FALSE(w.covers(10 * kSec - 1)) << "one nanosecond before the start";
  EXPECT_FALSE(w.covers(20 * kSec + 1)) << "one nanosecond after the end";
}

TEST_P(PlannedStopStorageTest, EndEarlyMovesEndsAtAndFlagsIt) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));

  auto result = storage_->end_planned_stop("w1", 30 * kSec, 50 * kSec);
  ASSERT_EQ(result.outcome, EndPlannedStopOutcome::Ended);
  EXPECT_EQ(result.window.ends_at_ns, 30 * kSec);
  EXPECT_TRUE(result.window.ended_early);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 30 * kSec) << "the stored window, not just the returned copy, must move";
  EXPECT_TRUE(stored->ended_early);
  EXPECT_EQ(stored->starts_at_ns, 10 * kSec) << "ending early must not move the start";
}

TEST_P(PlannedStopStorageTest, EndEarlyKeepsCoveringWhatCameBeforeTheEnd) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(storage_->end_planned_stop("w1", 30 * kSec, 50 * kSec).outcome, EndPlannedStopOutcome::Ended);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_TRUE(stored->covers(20 * kSec)) << "a fault raised before the early end stays expected";
  EXPECT_FALSE(stored->covers(40 * kSec)) << "a fault raised after the early end is not expected";
}

// R12/R15: which of the three situations a window is in is decided against NOW -
// the fault manager's wall clock - never against the `at` the caller supplied.
// Judging on `at` let a caller pick the answer: a backdated instant inside the
// original span re-ended a window that had already finished on its own, and one
// before the start deleted a finished window outright.

TEST_P(PlannedStopStorageTest, AWindowThatHasNotStartedIsCancelledNotEnded) {
  const int64_t now = 60 * kSec;
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("future", 100 * kSec, 200 * kSec, 50 * kSec)));

  auto result = storage_->end_planned_stop("future", now, now);
  ASSERT_EQ(result.outcome, EndPlannedStopOutcome::Cancelled);
  EXPECT_TRUE(result.window.cancelled);
  EXPECT_FALSE(result.window.ended_early) << "it never ran, so it did not end early";
  EXPECT_EQ(result.window.starts_at_ns, 100 * kSec) << "the window is returned as it was";
  EXPECT_EQ(result.window.ends_at_ns, 200 * kSec);

  EXPECT_FALSE(storage_->get_planned_stop("future").has_value()) << "a cancelled window is gone, not stored inverted";
}

TEST_P(PlannedStopStorageTest, ANotStartedWindowIsCancelledWhateverAtSays) {
  const int64_t now = 60 * kSec;
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("future", 100 * kSec, 200 * kSec, 50 * kSec)));

  // `at` inside the window's own span, which under the old rule would have made
  // it an "end". The window has not started at NOW, so it is a cancellation.
  auto result = storage_->end_planned_stop("future", 150 * kSec, now);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::Cancelled);
  EXPECT_FALSE(storage_->get_planned_stop("future").has_value());
}

TEST_P(PlannedStopStorageTest, NoStoredWindowEverHasAnEndAtOrBeforeItsStart) {
  // The message promises ends_at is strictly after starts_at. Every path that
  // can move ends_at is driven here, and the invariant is asserted over the
  // whole store afterwards.
  const int64_t now = 50 * kSec;
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("running", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("future", 100 * kSec, 200 * kSec, 5 * kSec)));
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("past", 1 * kSec, 2 * kSec, 1 * kSec)));

  storage_->end_planned_stop("running", 30 * kSec, now);  // ends early
  storage_->end_planned_stop("future", now, now);         // cancels
  storage_->end_planned_stop("past", 1 * kSec + 1, now);  // refused: already over
  storage_->end_planned_stop("running", 15 * kSec, now);  // refused: already ended

  // The input that used to store ends_at == starts_at: an `at` exactly on the
  // start of a window that is running.
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("at_its_start", 20 * kSec, 90 * kSec, 5 * kSec)));
  EXPECT_EQ(storage_->end_planned_stop("at_its_start", 20 * kSec, now).outcome, EndPlannedStopOutcome::InvalidAt);

  for (const auto & window : storage_->list_planned_stops()) {
    EXPECT_GT(window.ends_at_ns, window.starts_at_ns) << "window " << window.id;
  }
}

// The reported case: a window that finished on its own, re-ended with a
// backdated instant that still fell inside its original span.
TEST_P(PlannedStopStorageTest, ANaturallyFinishedWindowIsImmutable) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 20 * kSec, 5 * kSec)));
  const int64_t now = 40 * kSec;  // the window ran out ten seconds ago

  auto backdated = storage_->end_planned_stop("w", 15 * kSec, now);
  EXPECT_EQ(backdated.outcome, EndPlannedStopOutcome::AlreadyEnded);

  auto stored = storage_->get_planned_stop("w");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 20 * kSec) << "when a stop finished is not rewritable";
  EXPECT_FALSE(stored->ended_early) << "and it did not become an early end retroactively";
}

TEST_P(PlannedStopStorageTest, AFinishedWindowIsNotCancelledByAnAtBeforeItsStart) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 20 * kSec, 5 * kSec)));
  const int64_t now = 40 * kSec;

  // Under the old rule this took the cancel branch and DELETED a window that had
  // already run - erasing the reason a month of faults reads as expected.
  auto result = storage_->end_planned_stop("w", 1 * kSec, now);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::AlreadyEnded);
  EXPECT_TRUE(storage_->get_planned_stop("w").has_value()) << "the row must still be there";
}

TEST_P(PlannedStopStorageTest, ABackdatedEndCannotMoveAnEndThatAlreadyPassed) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(storage_->end_planned_stop("w", 50 * kSec, 50 * kSec).outcome, EndPlannedStopOutcome::Ended);

  auto backdated = storage_->end_planned_stop("w", 20 * kSec, 60 * kSec);
  EXPECT_EQ(backdated.outcome, EndPlannedStopOutcome::AlreadyEnded);

  auto stored = storage_->get_planned_stop("w");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 50 * kSec) << "the record of when the stop finished is not rewritable";
}

// `at` only REFINES the end instant of a window that is running now. Outside
// [starts_at, now] it is not an instant the window could have ended at.
TEST_P(PlannedStopStorageTest, AtBeforeTheStartOfARunningWindowIsRefused) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 100 * kSec, 5 * kSec)));

  auto result = storage_->end_planned_stop("w", 5 * kSec, 50 * kSec);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::InvalidAt);

  auto stored = storage_->get_planned_stop("w");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 100 * kSec);
  EXPECT_FALSE(stored->ended_early);
}

TEST_P(PlannedStopStorageTest, AtInTheFutureOfARunningWindowIsRefused) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 100 * kSec, 5 * kSec)));

  // 60 s has not happened yet at now = 50 s; a stop cannot end in the future.
  auto result = storage_->end_planned_stop("w", 60 * kSec, 50 * kSec);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::InvalidAt);
  EXPECT_EQ(storage_->get_planned_stop("w")->ends_at_ns, 100 * kSec);
}

TEST_P(PlannedStopStorageTest, AtWithinTheRunningSpanEndsTheWindowThere) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w", 10 * kSec, 100 * kSec, 5 * kSec)));

  auto result = storage_->end_planned_stop("w", 30 * kSec, 50 * kSec);
  ASSERT_EQ(result.outcome, EndPlannedStopOutcome::Ended);
  EXPECT_EQ(result.window.ends_at_ns, 30 * kSec);
  EXPECT_TRUE(result.window.ended_early);

  // `now` is a legal end. The START is not: ending there would store
  // ends_at == starts_at, which the message says can never happen.
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("at_now", 10 * kSec, 100 * kSec, 5 * kSec)));
  EXPECT_EQ(storage_->end_planned_stop("at_now", 50 * kSec, 50 * kSec).outcome, EndPlannedStopOutcome::Ended);

  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("at_start", 10 * kSec, 100 * kSec, 5 * kSec)));
  EXPECT_EQ(storage_->end_planned_stop("at_start", 10 * kSec, 50 * kSec).outcome, EndPlannedStopOutcome::InvalidAt)
      << "a window cannot end at the instant it started - that is a zero-length window";
  EXPECT_EQ(storage_->end_planned_stop("at_start", 10 * kSec + 1, 50 * kSec).outcome, EndPlannedStopOutcome::Ended)
      << "one nanosecond after the start is an interval, however short";
}

TEST_P(PlannedStopStorageTest, EndUnknownIdIsNotFound) {
  auto result = storage_->end_planned_stop("never_declared", 30 * kSec, 50 * kSec);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::NotFound);
}

TEST_P(PlannedStopStorageTest, EndingATwiceEndedWindowIsRefused) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(storage_->end_planned_stop("w1", 30 * kSec, 50 * kSec).outcome, EndPlannedStopOutcome::Ended);

  auto again = storage_->end_planned_stop("w1", 50 * kSec, 60 * kSec);
  EXPECT_EQ(again.outcome, EndPlannedStopOutcome::AlreadyEnded);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 30 * kSec) << "the refused second end must not move ends_at again";
}

TEST_P(PlannedStopStorageTest, EndingAWindowThatAlreadyExpiredIsRefused) {
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec)));

  // 40s is past the declared end: the window closed on its own.
  auto result = storage_->end_planned_stop("w1", 40 * kSec, 40 * kSec);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::AlreadyEnded);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 20 * kSec);
  EXPECT_FALSE(stored->ended_early);
}

// SCALE: the cap engages only past itself, so the case has to sit past it.
TEST_P(PlannedStopStorageTest, RetentionPrunesOldestEndedAndKeepsTheActiveOne) {
  const size_t cap = 5;
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(cap, now);

  // cap + 5 declarations: the first cap + 4 have already ended, the last is live.
  for (int i = 0; i < static_cast<int>(cap) + 4; ++i) {
    const int64_t declared = static_cast<int64_t>(i + 1) * kSec;
    ASSERT_EQ(
        DeclarePlannedStopOutcome::Stored,
        storage_->declare_planned_stop(make_window("ended_" + std::to_string(i), declared, declared + kSec, declared)));
  }
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("active", now - kSec, now + 100 * kSec, now)));

  auto all = storage_->list_planned_stops();
  EXPECT_LE(all.size(), cap) << "the cap is a hard bound on stored windows";

  bool has_active = false;
  for (const auto & w : all) {
    if (w.id == "active") {
      has_active = true;
    }
  }
  EXPECT_TRUE(has_active) << "a live window must never vanish under an operator";
  EXPECT_FALSE(storage_->get_planned_stop("ended_0").has_value()) << "the oldest declaration goes first";
  EXPECT_FALSE(storage_->get_planned_stop("ended_1").has_value());
}

// A window that has not ended is never pruned - not even to make room. Under the
// hard bound that means the DECLARATION is refused, rather than a live window
// vanishing under the operator who declared it.
TEST_P(PlannedStopStorageTest, AnActiveWindowIsNeverPrunedToMakeRoom) {
  const size_t cap = 2;
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(cap, now);

  for (size_t i = 0; i < cap; ++i) {
    const int64_t declared = now + static_cast<int64_t>(i);
    ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
              storage_->declare_planned_stop(
                  make_window("live_" + std::to_string(i), now - kSec, now + 500 * kSec, declared)));
  }

  EXPECT_EQ(storage_->declare_planned_stop(make_window("refused", now - kSec, now + 500 * kSec, now + 9)),
            DeclarePlannedStopOutcome::CapFull);
  auto all = storage_->list_planned_stops();
  EXPECT_EQ(all.size(), cap) << "the bound holds and no live window was sacrificed for it";
  for (const auto & w : all) {
    EXPECT_TRUE(w.active_at(now)) << "window " << w.id << " should still be running";
  }
}

TEST_P(PlannedStopStorageTest, ADeclarationIsRefusedWhenTheCapIsFullOfLiveWindows) {
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(1, now);
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("live", now - kSec, now + 500 * kSec, now - kSec)));

  EXPECT_EQ(storage_->declare_planned_stop(make_window("refused", 1 * kSec, 2 * kSec, now)),
            DeclarePlannedStopOutcome::CapFull);
  EXPECT_FALSE(storage_->get_planned_stop("refused").has_value()) << "a refused declaration must not be stored at all";
  EXPECT_EQ(storage_->list_planned_stops().size(), 1u) << "the bound is hard";

  ASSERT_EQ(storage_->end_planned_stop("live", now, now).outcome, EndPlannedStopOutcome::Ended);
  EXPECT_EQ(storage_->declare_planned_stop(make_window("accepted", 1 * kSec, 2 * kSec, now + kSec)),
            DeclarePlannedStopOutcome::Stored);
  EXPECT_EQ(storage_->list_planned_stops().size(), 1u) << "and the ended one made room by going";
}

TEST_P(PlannedStopStorageTest, FutureWindowsCannotGrowTheTablePastTheCap) {
  const int64_t now = 1000 * kSec;
  const size_t cap = 3;
  storage_->set_max_planned_stops(cap, now);

  for (size_t i = 0; i < cap; ++i) {
    ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
              storage_->declare_planned_stop(make_window("future_" + std::to_string(i), now + 100 * kSec,
                                                         now + 200 * kSec, now + static_cast<int64_t>(i))));
  }
  EXPECT_EQ(storage_->declare_planned_stop(make_window("one_too_many", now + 100 * kSec, now + 200 * kSec, now + 9)),
            DeclarePlannedStopOutcome::CapFull);
  EXPECT_EQ(storage_->list_planned_stops().size(), cap);
}

TEST_P(PlannedStopStorageTest, ADeclarationPrunesEndedWindowsToMakeRoomForItself) {
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(2, now);
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("old", 1 * kSec, 2 * kSec, 1 * kSec)));
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("newer", 3 * kSec, 4 * kSec, 3 * kSec)));

  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("newest", 5 * kSec, 6 * kSec, 5 * kSec)));
  EXPECT_TRUE(storage_->get_planned_stop("newest").has_value())
      << "a declaration reported as stored must be there afterwards";
  EXPECT_FALSE(storage_->get_planned_stop("old").has_value()) << "the oldest ended window made room";
  EXPECT_EQ(storage_->list_planned_stops().size(), 2u);
}

// CONFIG SWEEP at the storage boundary: a cap of one keeps exactly the newest.
TEST_P(PlannedStopStorageTest, RetentionOfOneKeepsOnlyTheNewestEndedWindow) {
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(1, now);

  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("first", 1 * kSec, 2 * kSec, 1 * kSec)));
  ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
            storage_->declare_planned_stop(make_window("second", 3 * kSec, 4 * kSec, 3 * kSec)));

  auto all = storage_->list_planned_stops();
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all[0].id, "second");
}

// CHANGE: raising the bound after the windows are stored evicts nothing; lowering
// it evicts on the spot, which is what a restart with a smaller cap must do.
TEST_P(PlannedStopStorageTest, LoweringTheBoundEvictsStoredWindowsImmediately) {
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(10, now);
  for (int i = 0; i < 6; ++i) {
    const int64_t declared = static_cast<int64_t>(i + 1) * kSec;
    ASSERT_EQ(DeclarePlannedStopOutcome::Stored, storage_->declare_planned_stop(make_window(
                                                     "w" + std::to_string(i), declared, declared + kSec, declared)));
  }
  ASSERT_EQ(storage_->list_planned_stops().size(), 6u);

  const size_t evicted = storage_->set_max_planned_stops(2, now);
  EXPECT_EQ(evicted, 4u);
  EXPECT_EQ(storage_->list_planned_stops().size(), 2u);
  EXPECT_TRUE(storage_->get_planned_stop("w5").has_value()) << "the newest declarations survive";
  EXPECT_TRUE(storage_->get_planned_stop("w4").has_value());
  EXPECT_FALSE(storage_->get_planned_stop("w0").has_value());
}

// --- SQLite only: the half that has no in-memory meaning -------------------

class SqlitePlannedStopTest : public ::testing::Test {
 protected:
  void SetUp() override {
    db_path_ = unique_db_path();
  }

  void TearDown() override {
    std::filesystem::remove(db_path_);
    std::filesystem::remove(db_path_.string() + "-wal");
    std::filesystem::remove(db_path_.string() + "-shm");
  }

  std::filesystem::path db_path_;
};

TEST_F(SqlitePlannedStopTest, WindowsSurviveCloseAndReopen) {
  {
    SqliteFaultStorage storage(db_path_.string());
    ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
              storage.declare_planned_stop(make_window("survivor", 10 * kSec, 900 * kSec, 5 * kSec)));
    ASSERT_EQ(storage.end_planned_stop("survivor", 500 * kSec, 500 * kSec).outcome, EndPlannedStopOutcome::Ended);
  }

  SqliteFaultStorage reopened(db_path_.string());
  auto stored = reopened.get_planned_stop("survivor");
  ASSERT_TRUE(stored.has_value()) << "a declared window must outlive the process that declared it";
  EXPECT_EQ(stored->starts_at_ns, 10 * kSec);
  EXPECT_EQ(stored->ends_at_ns, 500 * kSec) << "the early end must be persisted, not just held in memory";
  EXPECT_TRUE(stored->ended_early);
  EXPECT_EQ(stored->reason, "changeover");
  EXPECT_EQ(stored->declared_by, "operator_a");
}

// A database written before this feature existed has no planned_stops table.
// Opening it must add the table rather than fail, which is the whole point of
// running the schema step on every open.
TEST_F(SqlitePlannedStopTest, OpeningADatabaseWithoutThePlannedStopsTableAddsIt) {
  {
    SqliteFaultStorage storage(db_path_.string());
    ASSERT_EQ(DeclarePlannedStopOutcome::Stored,
              storage.declare_planned_stop(make_window("pre", 1 * kSec, 2 * kSec, 1 * kSec)));
  }
  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw, "DROP TABLE planned_stops", nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(raw);
  }

  SqliteFaultStorage reopened(db_path_.string());
  EXPECT_TRUE(reopened.list_planned_stops().empty());
  EXPECT_EQ(DeclarePlannedStopOutcome::Stored,
            reopened.declare_planned_stop(make_window("post", 3 * kSec, 4 * kSec, 3 * kSec)));
  EXPECT_TRUE(reopened.get_planned_stop("post").has_value());
}

// --- nanoseconds <-> builtin_interfaces/Time ---------------------------------

TEST(TimeMsgConversion, NegativeInstantsStayWellFormed) {
  using ros2_medkit_fault_manager::ns_to_time_msg;
  using ros2_medkit_fault_manager::time_msg_to_ns;

  // builtin_interfaces/Time carries `nanosec` as an UNSIGNED field, so the
  // conversion has to floor rather than truncate towards zero: -1 ns is one
  // nanosecond before the epoch, which is second -1 plus 999999999 ns. C's
  // remainder gives -1 there, and casting that to uint32 produced 4294967295 -
  // a message no reader can interpret.
  auto t = ns_to_time_msg(-1);
  EXPECT_EQ(t.sec, -1);
  EXPECT_EQ(t.nanosec, 999999999u);
  EXPECT_LT(t.nanosec, 1000000000u);
  EXPECT_EQ(time_msg_to_ns(t), -1) << "and the round trip must come back";

  t = ns_to_time_msg(-1500000000LL);
  EXPECT_EQ(t.sec, -2);
  EXPECT_EQ(t.nanosec, 500000000u);
  EXPECT_EQ(time_msg_to_ns(t), -1500000000LL);

  // INT64_MIN cannot be carried by an int32 second field at all. The conversion
  // must not wrap into a plausible-looking instant; it saturates, and the API
  // refuses such a value long before it gets here.
  t = ns_to_time_msg(std::numeric_limits<int64_t>::min());
  EXPECT_LT(t.nanosec, 1000000000u) << "whatever it saturates to must still be a legal Time";
}

TEST(TimeMsgConversion, PositiveInstantsRoundTrip) {
  using ros2_medkit_fault_manager::ns_to_time_msg;
  using ros2_medkit_fault_manager::time_msg_to_ns;

  const int64_t samples[] = {0, 1, kSec, kSec + 999999999LL, 1788698096LL * kSec};
  for (int64_t ns : samples) {
    const auto t = ns_to_time_msg(ns);
    EXPECT_LT(t.nanosec, 1000000000u) << "for " << ns;
    EXPECT_EQ(time_msg_to_ns(t), ns);
  }
}

// --- The configured bound, swept across its whole declared range ------------

TEST(PlannedStopWindowBoundTest, ClampSweepsTheDeclaredRange) {
  bool clamped = false;

  EXPECT_EQ(clamp_planned_stop_windows(100, clamped), 100);
  EXPECT_FALSE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(kMinPlannedStopWindows, clamped), kMinPlannedStopWindows);
  EXPECT_FALSE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(kMaxPlannedStopWindows, clamped), kMaxPlannedStopWindows);
  EXPECT_FALSE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(0, clamped), kMinPlannedStopWindows);
  EXPECT_TRUE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(-1, clamped), kMinPlannedStopWindows);
  EXPECT_TRUE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(kMaxPlannedStopWindows + 1, clamped), kMaxPlannedStopWindows);
  EXPECT_TRUE(clamped);

  // ROS parameters arrive as int64. A value above INT_MAX must be clamped
  // BEFORE it is narrowed, or it wraps back into the legal band and passes.
  EXPECT_EQ(clamp_planned_stop_windows(std::numeric_limits<int64_t>::max(), clamped), kMaxPlannedStopWindows);
  EXPECT_TRUE(clamped);
  EXPECT_EQ(clamp_planned_stop_windows(4'294'967'396, clamped), kMaxPlannedStopWindows);
  EXPECT_TRUE(clamped);

  EXPECT_EQ(clamp_planned_stop_windows(std::numeric_limits<int64_t>::min(), clamped), kMinPlannedStopWindows);
  EXPECT_TRUE(clamped);
}
