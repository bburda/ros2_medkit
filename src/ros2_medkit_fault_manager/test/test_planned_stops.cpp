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
  ASSERT_TRUE(storage_->declare_planned_stop(w));

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
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec)));
  EXPECT_FALSE(storage_->declare_planned_stop(make_window("w1", 30 * kSec, 40 * kSec, 25 * kSec)));

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->starts_at_ns, 10 * kSec) << "the second declaration must not overwrite the first";
}

TEST_P(PlannedStopStorageTest, GetUnknownIdIsEmpty) {
  EXPECT_FALSE(storage_->get_planned_stop("never_declared").has_value());
}

TEST_P(PlannedStopStorageTest, ListReturnsNewestDeclarationFirst) {
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("older", 10 * kSec, 20 * kSec, 1 * kSec)));
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("newer", 30 * kSec, 40 * kSec, 2 * kSec)));

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
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));

  auto result = storage_->end_planned_stop("w1", 30 * kSec);
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
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(storage_->end_planned_stop("w1", 30 * kSec).outcome, EndPlannedStopOutcome::Ended);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_TRUE(stored->covers(20 * kSec)) << "a fault raised before the early end stays expected";
  EXPECT_FALSE(stored->covers(40 * kSec)) << "a fault raised after the early end is not expected";
}

TEST_P(PlannedStopStorageTest, EndUnknownIdIsNotFound) {
  auto result = storage_->end_planned_stop("never_declared", 30 * kSec);
  EXPECT_EQ(result.outcome, EndPlannedStopOutcome::NotFound);
}

TEST_P(PlannedStopStorageTest, EndingATwiceEndedWindowIsRefused) {
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("w1", 10 * kSec, 100 * kSec, 5 * kSec)));
  ASSERT_EQ(storage_->end_planned_stop("w1", 30 * kSec).outcome, EndPlannedStopOutcome::Ended);

  auto again = storage_->end_planned_stop("w1", 50 * kSec);
  EXPECT_EQ(again.outcome, EndPlannedStopOutcome::AlreadyEnded);

  auto stored = storage_->get_planned_stop("w1");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->ends_at_ns, 30 * kSec) << "the refused second end must not move ends_at again";
}

TEST_P(PlannedStopStorageTest, EndingAWindowThatAlreadyExpiredIsRefused) {
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("w1", 10 * kSec, 20 * kSec, 5 * kSec)));

  // 40s is past the declared end: the window closed on its own.
  auto result = storage_->end_planned_stop("w1", 40 * kSec);
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
    ASSERT_TRUE(
        storage_->declare_planned_stop(make_window("ended_" + std::to_string(i), declared, declared + kSec, declared)));
  }
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("active", now - kSec, now + 100 * kSec, now)));

  auto all = storage_->list_planned_stops();
  EXPECT_LE(all.size(), cap) << "the stored count must never exceed the cap while ended windows remain";

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

TEST_P(PlannedStopStorageTest, RetentionNeverPrunesAnActiveWindowEvenPastTheCap) {
  const size_t cap = 2;
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(cap, now);

  for (int i = 0; i < 5; ++i) {
    const int64_t declared = now + static_cast<int64_t>(i) * kSec;
    ASSERT_TRUE(storage_->declare_planned_stop(
        make_window("live_" + std::to_string(i), now - kSec, now + 500 * kSec, declared)));
  }

  auto all = storage_->list_planned_stops();
  EXPECT_EQ(all.size(), 5u) << "five live windows must all survive a cap of two";
}

// CONFIG SWEEP at the storage boundary: a cap of one keeps exactly the newest.
TEST_P(PlannedStopStorageTest, RetentionOfOneKeepsOnlyTheNewestEndedWindow) {
  const int64_t now = 1000 * kSec;
  storage_->set_max_planned_stops(1, now);

  ASSERT_TRUE(storage_->declare_planned_stop(make_window("first", 1 * kSec, 2 * kSec, 1 * kSec)));
  ASSERT_TRUE(storage_->declare_planned_stop(make_window("second", 3 * kSec, 4 * kSec, 3 * kSec)));

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
    ASSERT_TRUE(
        storage_->declare_planned_stop(make_window("w" + std::to_string(i), declared, declared + kSec, declared)));
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
    ASSERT_TRUE(storage.declare_planned_stop(make_window("survivor", 10 * kSec, 900 * kSec, 5 * kSec)));
    ASSERT_EQ(storage.end_planned_stop("survivor", 500 * kSec).outcome, EndPlannedStopOutcome::Ended);
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
    ASSERT_TRUE(storage.declare_planned_stop(make_window("pre", 1 * kSec, 2 * kSec, 1 * kSec)));
  }
  {
    sqlite3 * raw = nullptr;
    ASSERT_EQ(sqlite3_open(db_path_.string().c_str(), &raw), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(raw, "DROP TABLE planned_stops", nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(raw);
  }

  SqliteFaultStorage reopened(db_path_.string());
  EXPECT_TRUE(reopened.list_planned_stops().empty());
  EXPECT_TRUE(reopened.declare_planned_stop(make_window("post", 3 * kSec, 4 * kSec, 3 * kSec)));
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
