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

/// The pure half of the planned-stop feature: which window covers a fault, and
/// how an ISO 8601 instant on the wire becomes a nanosecond count.
///
/// Both are total functions over their input, which is why they are tested here
/// at nanosecond resolution instead of through HTTP - a boundary one nanosecond
/// wide cannot be driven through a JSON body that carries milliseconds.

#include <gtest/gtest.h>

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/faults/planned_stop.hpp"

using ros2_medkit_gateway::faults::annotate_fault_with_planned_stop;
using ros2_medkit_gateway::faults::find_covering_window;
using ros2_medkit_gateway::faults::floor_to_ms_ns;
using ros2_medkit_gateway::faults::parse_iso8601_utc_ns;
using ros2_medkit_gateway::faults::PlannedStopCache;
using ros2_medkit_gateway::faults::PlannedStopWindow;
using ros2_medkit_gateway::faults::seconds_to_ns;
using ros2_medkit_gateway::faults::window_for_fault;

namespace {

constexpr int64_t kSec = 1'000'000'000;

PlannedStopWindow window(const std::string & id, int64_t from_ns, int64_t to_ns) {
  PlannedStopWindow w;
  w.id = id;
  w.from_ns = from_ns;
  w.to_ns = to_ns;
  w.reason = "changeover";
  w.declared_by = "operator";
  w.declared_at_ns = from_ns;
  return w;
}

}  // namespace

// --- the covering predicate -------------------------------------------------

TEST(PlannedStopCoverage, NoWindowsCoverNothing) {
  const std::vector<PlannedStopWindow> none;
  EXPECT_EQ(find_covering_window(none, 42 * kSec), nullptr);
}

TEST(PlannedStopCoverage, TheIntervalIsClosedAtBothEnds) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};

  ASSERT_NE(find_covering_window(windows, 10 * kSec), nullptr) << "exactly at from";
  ASSERT_NE(find_covering_window(windows, 20 * kSec), nullptr) << "exactly at to";
  ASSERT_NE(find_covering_window(windows, 15 * kSec), nullptr);
  // A millisecond, not a nanosecond: see PlannedStopCoverage.TheBoundaryIsAMillisecondWide.
  EXPECT_EQ(find_covering_window(windows, 10 * kSec - 1'000'000), nullptr) << "one millisecond before from";
  EXPECT_EQ(find_covering_window(windows, 20 * kSec + 1'000'000), nullptr) << "one millisecond after to";
}

TEST(PlannedStopCoverage, OverlappingWindowsPickTheEarliestStarting) {
  // Deliberately stored newest-first, which is the order the fault manager lists
  // them in: the choice must come from `from_ns`, not from position.
  const std::vector<PlannedStopWindow> windows{
      window("late", 12 * kSec, 40 * kSec),
      window("early", 8 * kSec, 30 * kSec),
      window("middle", 10 * kSec, 25 * kSec),
  };

  const auto * covering = find_covering_window(windows, 15 * kSec);
  ASSERT_NE(covering, nullptr);
  EXPECT_EQ(covering->id, "early");
}

TEST(PlannedStopCoverage, TwoWindowsStartingTogetherResolveDeterministically) {
  const std::vector<PlannedStopWindow> windows{
      window("bbb", 10 * kSec, 40 * kSec),
      window("aaa", 10 * kSec, 20 * kSec),
  };

  const auto * first = find_covering_window(windows, 15 * kSec);
  const auto * second = find_covering_window(windows, 15 * kSec);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->id, second->id) << "the same input must not answer two ways";
  EXPECT_EQ(first->id, "aaa") << "the tie is broken by id so two gateways agree";
}

TEST(PlannedStopCoverage, AWindowThatEndedEarlyStopsCoveringWhatCameAfter) {
  auto ended = window("w", 10 * kSec, 100 * kSec);
  ended.to_ns = 30 * kSec;
  ended.ended_early = true;
  const std::vector<PlannedStopWindow> windows{ended};

  EXPECT_NE(find_covering_window(windows, 20 * kSec), nullptr);
  EXPECT_EQ(find_covering_window(windows, 40 * kSec), nullptr);
}

TEST(PlannedStopCoverage, AWindowWhollyInThePastStillMarksWhatItCovers) {
  const std::vector<PlannedStopWindow> windows{window("historic", 1 * kSec, 2 * kSec)};
  EXPECT_NE(find_covering_window(windows, 1 * kSec + 500'000'000), nullptr)
      << "a stop declared after the fact must still mark the faults it covers";
}

// --- the wire representation of an instant ----------------------------------

TEST(PlannedStopTimeParsing, AcceptsIso8601Utc) {
  auto parsed = parse_iso8601_utc_ns("1970-01-01T00:00:00Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 0);

  parsed = parse_iso8601_utc_ns("1970-01-01T00:00:01Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, kSec);

  parsed = parse_iso8601_utc_ns("2026-09-06T12:34:56Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 1788698096LL * kSec);
}

TEST(PlannedStopTimeParsing, KeepsFractionalSecondsToNanosecondResolution) {
  auto parsed = parse_iso8601_utc_ns("1970-01-01T00:00:00.5Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 500'000'000);

  parsed = parse_iso8601_utc_ns("1970-01-01T00:00:00.123Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 123'000'000);

  parsed = parse_iso8601_utc_ns("1970-01-01T00:00:00.000000001Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 0) << "a fraction finer than a millisecond floors to the millisecond";

  // More digits than the API carries are truncated, not rejected: the instant is
  // still unambiguous.
  parsed = parse_iso8601_utc_ns("1970-01-01T00:00:00.1234567891Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 123'000'000);
}

TEST(PlannedStopTimeParsing, AcceptsAnExplicitZeroOffset) {
  const auto z = parse_iso8601_utc_ns("2026-09-06T12:34:56Z");
  const auto plus = parse_iso8601_utc_ns("2026-09-06T12:34:56+00:00");
  const auto minus = parse_iso8601_utc_ns("2026-09-06T12:34:56-00:00");
  ASSERT_TRUE(z.has_value());
  ASSERT_TRUE(plus.has_value());
  ASSERT_TRUE(minus.has_value());
  EXPECT_EQ(*plus, *z);
  EXPECT_EQ(*minus, *z);
}

TEST(PlannedStopTimeParsing, RefusesAYearWhoseNanosecondCountCannotBeCounted) {
  // int64 nanoseconds run out in April 2262. Multiplying first wrapped a year
  // past that back INTO the accepted range, so a stop declared for 2600 was
  // stored as one in 2015 and answered 201.
  for (const char * text :
       {"2262-04-12T00:00:00Z", "2555-01-01T00:00:00Z", "2600-01-01T00:00:00Z", "9999-12-31T23:59:59Z"}) {
    EXPECT_FALSE(parse_iso8601_utc_ns(text).has_value()) << "accepted: " << text;
  }

  // A year between the int32-second limit and the int64-nanosecond one parses
  // to a REAL value, which the representable-instant guard then refuses. That is
  // the split the fix depends on: the parser must not be the thing that hides it.
  const auto in_2100 = parse_iso8601_utc_ns("2100-01-01T00:00:00Z");
  ASSERT_TRUE(in_2100.has_value());
  EXPECT_GT(*in_2100, ros2_medkit_gateway::faults::kMaxRepresentableNs);
}

TEST(PlannedStopTimeParsing, RefusesAnInstantAtOrBeforeTheEpoch) {
  // The epoch is not a sentinel for "now" anywhere in this API, so it cannot be
  // a legal window bound either.
  const auto epoch = parse_iso8601_utc_ns("1970-01-01T00:00:00Z");
  ASSERT_TRUE(epoch.has_value()) << "the text is a valid instant; it is the value that is refused";
  EXPECT_EQ(*epoch, 0);
  EXPECT_FALSE(ros2_medkit_gateway::faults::is_representable_instant(*epoch));
  EXPECT_TRUE(ros2_medkit_gateway::faults::is_representable_instant(1'000'000))
      << "one millisecond after the epoch is a real instant";
}

TEST(PlannedStopTimeParsing, RefusesWhatItCannotReadAsAnInstantInUtc) {
  const char * unparsable[] = {
      "",
      "not a time",
      "2026-09-06",                 // date only: no instant
      "2026-09-06T12:34:56",        // no zone: the instant depends on the reader
      "2026-09-06T12:34:56+02:00",  // a real offset, which this route does not take
      "2026-09-06 12:34:56Z",       // space instead of T
      "2026-13-06T12:34:56Z",       // month 13
      "2026-09-32T12:34:56Z",       // day 32
      "2026-09-06T25:34:56Z",       // hour 25
      "2026-09-06T12:60:56Z",       // minute 60
      "2026-09-06T12:34:61Z",       // second 61
      "2026-09-06T12:34:56.Z",      // a dot with no fraction
      "2026-09-06T12:34:56Zjunk",   // trailing junk
      "  2026-09-06T12:34:56Z",     // leading space
  };
  for (const char * text : unparsable) {
    EXPECT_FALSE(parse_iso8601_utc_ns(text).has_value()) << "accepted: '" << text << "'";
  }
}

TEST(PlannedStopTimeParsing, SecondsWithAFractionBecomeNanoseconds) {
  EXPECT_EQ(seconds_to_ns(0.0), 0);
  EXPECT_EQ(seconds_to_ns(1.0), kSec);
  EXPECT_EQ(seconds_to_ns(1.5), kSec + 500'000'000);
  // The fault list carries first_occurred as a double; a value big enough to be
  // a real timestamp must not lose whole seconds on the way in.
  EXPECT_EQ(seconds_to_ns(1788698096.0), 1788698096LL * kSec);
}

// --- deriving the flag from a fault as it appears on the wire ----------------

TEST(PlannedStopAnnotation, MarksAFaultWhoseCycleStartedInsideAWindow) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};
  nlohmann::json fault{{"fault_code", "MOTOR_STALL"}, {"first_occurred", 15.0}, {"status", "CONFIRMED"}};

  EXPECT_TRUE(annotate_fault_with_planned_stop(fault, windows));
  EXPECT_TRUE(fault["x-medkit"]["expected"].get<bool>());
  EXPECT_EQ(fault["x-medkit"]["planned_stop_id"].get<std::string>(), "w");
}

TEST(PlannedStopAnnotation, LeavesAFaultOutsideEveryWindowUnexpectedAndUnattributed) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};
  nlohmann::json fault{{"fault_code", "MOTOR_STALL"}, {"first_occurred", 25.0}};

  EXPECT_FALSE(annotate_fault_with_planned_stop(fault, windows));
  EXPECT_FALSE(fault["x-medkit"]["expected"].get<bool>());
  EXPECT_FALSE(fault["x-medkit"].contains("planned_stop_id"))
      << "naming a window that does not cover the fault would be worse than naming none";
}

TEST(PlannedStopAnnotation, KeepsVendorKeysTheItemAlreadyCarried) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};
  nlohmann::json fault{{"first_occurred", 15.0}, {"x-medkit", {{"captured_at", "2026-09-06T00:00:00.000Z"}}}};

  ASSERT_TRUE(annotate_fault_with_planned_stop(fault, windows));
  EXPECT_EQ(fault["x-medkit"]["captured_at"].get<std::string>(), "2026-09-06T00:00:00.000Z");
  EXPECT_TRUE(fault["x-medkit"]["expected"].get<bool>());
}

TEST(PlannedStopAnnotation, ReAnnotatingClearsAStaleAttribution) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};
  nlohmann::json fault{{"first_occurred", 25.0}, {"x-medkit", {{"expected", true}, {"planned_stop_id", "gone"}}}};

  EXPECT_FALSE(annotate_fault_with_planned_stop(fault, windows));
  EXPECT_FALSE(fault["x-medkit"]["expected"].get<bool>());
  EXPECT_FALSE(fault["x-medkit"].contains("planned_stop_id"))
      << "a window that no longer covers the fault must not stay named on it";
}

TEST(PlannedStopAnnotation, AFaultThatCannotBeDatedIsNeverExpected) {
  const std::vector<PlannedStopWindow> windows{window("w", 0, 4'000'000'000LL * kSec)};

  nlohmann::json missing{{"fault_code", "NO_TIME"}};
  EXPECT_FALSE(annotate_fault_with_planned_stop(missing, windows));
  EXPECT_FALSE(missing["x-medkit"]["expected"].get<bool>());

  nlohmann::json wrong_type{{"first_occurred", "not a number"}};
  EXPECT_FALSE(annotate_fault_with_planned_stop(wrong_type, windows));

  nlohmann::json null_value{{"first_occurred", nullptr}};
  EXPECT_FALSE(annotate_fault_with_planned_stop(null_value, windows));
}

TEST(PlannedStopAnnotation, ANonObjectItemIsLeftAlone) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};
  nlohmann::json not_an_object = nlohmann::json::array({1, 2, 3});

  EXPECT_FALSE(annotate_fault_with_planned_stop(not_an_object, windows));
  EXPECT_TRUE(not_an_object.is_array()) << "a malformed peer item must pass through, not be rewritten";
}

TEST(PlannedStopAnnotation, WindowForFaultAgreesWithTheAnnotationAtTheBoundary) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};

  nlohmann::json at_start{{"first_occurred", 10.0}};
  nlohmann::json at_end{{"first_occurred", 20.0}};
  nlohmann::json just_after{{"first_occurred", 20.001}};

  EXPECT_NE(window_for_fault(windows, at_start), nullptr);
  EXPECT_NE(window_for_fault(windows, at_end), nullptr);
  EXPECT_EQ(window_for_fault(windows, just_after), nullptr);
}

// --- the resolution the API works at ----------------------------------------
//
// One millisecond, end to end (parse, storage, comparison, formatting). The
// reason is in the header: an operator declares a stop at minute or second
// precision, and the REST fault item carries first_occurred as a double whose
// resolution at today's epoch is about a quarter of a microsecond, so a boundary
// defined more finely than the data can express is a boundary nobody can rely
// on.

TEST(PlannedStopResolution, FloorToMillisecondIsATrueFloor) {
  EXPECT_EQ(floor_to_ms_ns(0), 0);
  EXPECT_EQ(floor_to_ms_ns(999'999), 0);
  EXPECT_EQ(floor_to_ms_ns(1'000'000), 1'000'000);
  EXPECT_EQ(floor_to_ms_ns(1'999'999), 1'000'000);
  EXPECT_EQ(floor_to_ms_ns(kSec + 123'456'789), kSec + 123'000'000);
  // Negative instants are refused by the API, but the helper is used on values
  // that arrive from outside it, so it must not round towards zero there.
  EXPECT_EQ(floor_to_ms_ns(-1), -1'000'000);
  EXPECT_EQ(floor_to_ms_ns(-1'000'000), -1'000'000);
  EXPECT_EQ(floor_to_ms_ns(-1'000'001), -2'000'000);
}

TEST(PlannedStopTimeParsing, RoundsAFinerFractionDownToTheMillisecond) {
  auto parsed = parse_iso8601_utc_ns("2026-09-06T12:34:56.123456789Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed % 1'000'000, 0) << "the parse must not keep what the API cannot carry";
  EXPECT_EQ(*parsed, 1788698096LL * kSec + 123'000'000);

  parsed = parse_iso8601_utc_ns("2026-09-06T12:34:56.999999Z");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, 1788698096LL * kSec + 999'000'000) << "down, never to the nearest";
}

TEST(PlannedStopCoverage, TheBoundaryIsAMillisecondWide) {
  const int64_t from_ns = 10 * kSec;
  const int64_t to_ns = 20 * kSec;
  const std::vector<PlannedStopWindow> windows{window("w", from_ns, to_ns)};

  // Exactly at the end, and anywhere inside that same millisecond, is covered.
  EXPECT_NE(find_covering_window(windows, to_ns), nullptr);
  EXPECT_NE(find_covering_window(windows, to_ns + 999'999), nullptr)
      << "a fault inside the closing millisecond is inside the window";
  EXPECT_EQ(find_covering_window(windows, to_ns + 1'000'000), nullptr) << "one millisecond after the end";

  EXPECT_NE(find_covering_window(windows, from_ns), nullptr);
  EXPECT_EQ(find_covering_window(windows, from_ns - 1'000'000), nullptr) << "one millisecond before the start";
  EXPECT_EQ(find_covering_window(windows, from_ns - 1), nullptr)
      << "the millisecond before the start is outside it, whole";
}

TEST(PlannedStopAnnotation, TheFaultSideOfTheComparisonIsFlooredTheSameWay) {
  const std::vector<PlannedStopWindow> windows{window("w", 10 * kSec, 20 * kSec)};

  // first_occurred arrives as a double. 20.0009 s lands inside the closing
  // millisecond of the window; 20.0011 s does not.
  nlohmann::json inside{{"first_occurred", 20.0009}};
  nlohmann::json outside{{"first_occurred", 20.0011}};
  EXPECT_TRUE(annotate_fault_with_planned_stop(inside, windows));
  EXPECT_FALSE(annotate_fault_with_planned_stop(outside, windows));
}

// --- what a builtin_interfaces/Time can carry --------------------------------

TEST(PlannedStopRepresentableInstant, AcceptsTheWholeInt32SecondRange) {
  using ros2_medkit_gateway::faults::is_representable_instant;
  using ros2_medkit_gateway::faults::kMaxRepresentableNs;
  using ros2_medkit_gateway::faults::kMinRepresentableNs;

  EXPECT_TRUE(is_representable_instant(kMinRepresentableNs));
  EXPECT_TRUE(is_representable_instant(kMaxRepresentableNs));
  EXPECT_TRUE(is_representable_instant(1788698096LL * kSec));

  EXPECT_FALSE(is_representable_instant(kMinRepresentableNs - 1)) << "the epoch itself is not a window bound";
  EXPECT_FALSE(is_representable_instant(0));
  EXPECT_FALSE(is_representable_instant(kMaxRepresentableNs + 1)) << "one nanosecond past the int32 second limit";

  // A window asked for in 2099. Its second count does not fit an int32, so the
  // narrowing would wrap it negative and hand back a window covering nothing.
  const auto in_2099 = parse_iso8601_utc_ns("2099-01-01T00:00:00Z");
  ASSERT_TRUE(in_2099.has_value()) << "the parse is fine; it is the storage that cannot hold it";
  EXPECT_FALSE(is_representable_instant(*in_2099));

  const auto before_the_epoch = parse_iso8601_utc_ns("1960-01-01T00:00:00Z");
  ASSERT_TRUE(before_the_epoch.has_value());
  EXPECT_FALSE(is_representable_instant(*before_the_epoch));
}

// --- the collection-level tally ---------------------------------------------

TEST(PlannedStopExpectedCount, CreatesTheExtensionWhenTheEmitterDidNot) {
  // A plugin's fault list need not carry an `x-medkit` object at all. Writing
  // through `collection["x-medkit"]["expected_count"]` without creating it left
  // the collection answering `"x-medkit": null`, which is neither the count nor
  // the absence of one.
  nlohmann::json collection{{"items", nlohmann::json::array()}};
  ros2_medkit_gateway::faults::set_expected_count(collection, 3);

  ASSERT_TRUE(collection["x-medkit"].is_object()) << "x-medkit must be an object, never null";
  EXPECT_EQ(collection["x-medkit"]["expected_count"].get<int64_t>(), 3);
}

TEST(PlannedStopExpectedCount, KeepsWhatTheExtensionAlreadyCarried) {
  nlohmann::json collection{{"items", nlohmann::json::array()}, {"x-medkit", {{"count", 7}}}};
  ros2_medkit_gateway::faults::set_expected_count(collection, 2);

  EXPECT_EQ(collection["x-medkit"]["count"].get<int64_t>(), 7);
  EXPECT_EQ(collection["x-medkit"]["expected_count"].get<int64_t>(), 2);
}

TEST(PlannedStopExpectedCount, ReplacesANullExtensionRatherThanWritingThroughIt) {
  nlohmann::json collection{{"items", nlohmann::json::array()}, {"x-medkit", nullptr}};
  ros2_medkit_gateway::faults::set_expected_count(collection, 0);

  ASSERT_TRUE(collection["x-medkit"].is_object());
  EXPECT_EQ(collection["x-medkit"]["expected_count"].get<int64_t>(), 0);
}

TEST(PlannedStopExpectedCount, LeavesANonObjectCollectionAlone) {
  nlohmann::json not_a_collection = nlohmann::json::array({1, 2});
  ros2_medkit_gateway::faults::set_expected_count(not_a_collection, 1);
  EXPECT_TRUE(not_a_collection.is_array());
}

// --- the cache the event path reads -----------------------------------------

TEST(PlannedStopCacheTest, StartsEmptyAndDueForARefresh) {
  PlannedStopCache cache;
  EXPECT_TRUE(cache.snapshot().empty());
  EXPECT_TRUE(cache.is_stale(std::chrono::seconds(2)));
  EXPECT_FALSE(cache.last_refresh_failed()) << "nothing has failed yet";
}

TEST(PlannedStopCacheTest, StoringWindowsMakesThemReadableAndFreshAgain) {
  PlannedStopCache cache;
  cache.store({window("w", 10 * kSec, 20 * kSec)});

  const auto snapshot = cache.snapshot();
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_EQ(snapshot[0].id, "w");
  EXPECT_FALSE(cache.is_stale(std::chrono::seconds(2))) << "a store that just happened cannot already be stale";
  EXPECT_TRUE(cache.is_stale(std::chrono::seconds(0))) << "a zero time-to-live means every read refreshes";
}

TEST(PlannedStopCacheTest, StoringAnEmptyListIsARealAnswerNotAMiss) {
  PlannedStopCache cache;
  cache.store({window("w", 10 * kSec, 20 * kSec)});
  cache.store({});

  EXPECT_TRUE(cache.snapshot().empty()) << "every window was ended; the cache must say so";
  EXPECT_FALSE(cache.is_stale(std::chrono::seconds(2)));
}

TEST(PlannedStopCacheTest, AFailedRefreshCountsAgainstTheTimeToLive) {
  PlannedStopCache cache;
  cache.store({window("w", 10 * kSec, 20 * kSec)});
  cache.note_failed_refresh();

  EXPECT_TRUE(cache.last_refresh_failed());
  EXPECT_FALSE(cache.is_stale(std::chrono::seconds(2)))
      << "an attempt that just failed must not invite an immediate retry - finding out costs a transport timeout";
  ASSERT_EQ(cache.snapshot().size(), 1u)
      << "a failed refresh must not blank the windows; a fault losing its flag reads as a surprise";
  EXPECT_EQ(cache.snapshot()[0].id, "w");
}

TEST(PlannedStopCacheTest, ASuccessfulRefreshClearsTheFailureFlag) {
  PlannedStopCache cache;
  cache.note_failed_refresh();
  ASSERT_TRUE(cache.last_refresh_failed());

  cache.store({window("w", 10 * kSec, 20 * kSec)});
  EXPECT_FALSE(cache.last_refresh_failed());
}

// R13: on the event stream, "unknown" is not "false". A consumer must not read
// an outage of the fault manager as "nothing is expected".
TEST(PlannedStopCacheTest, KnowsWhetherItHasEverSeenAWindowSet) {
  PlannedStopCache cache;
  EXPECT_FALSE(cache.has_knowledge()) << "nothing has been read yet";

  cache.note_failed_refresh();
  EXPECT_FALSE(cache.has_knowledge()) << "a read that got no answer taught it nothing";

  cache.store({});
  EXPECT_TRUE(cache.has_knowledge()) << "an empty answer IS an answer: no windows are declared";

  cache.note_failed_refresh();
  EXPECT_TRUE(cache.has_knowledge()) << "a later outage does not unlearn what was read";

  cache.invalidate();
  EXPECT_TRUE(cache.has_knowledge()) << "invalidate asks for a re-read; it does not forget";
}

TEST(PlannedStopCacheTest, InvalidateForcesTheNextReadToRefresh) {
  PlannedStopCache cache;
  cache.store({window("w", 10 * kSec, 20 * kSec)});
  ASSERT_FALSE(cache.is_stale(std::chrono::seconds(60)));

  cache.invalidate();
  EXPECT_TRUE(cache.is_stale(std::chrono::seconds(60)))
      << "a window declared through this gateway must not wait out the cache";
  EXPECT_EQ(cache.snapshot().size(), 1u) << "invalidating asks for a refresh; it does not throw away what is known";
}
