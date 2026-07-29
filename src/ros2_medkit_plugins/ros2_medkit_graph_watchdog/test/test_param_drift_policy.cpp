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
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_graph_watchdog/param_drift_policy.hpp"

using nlohmann::json;
using ros2_medkit_graph_watchdog::glob_match;
using ros2_medkit_graph_watchdog::is_unset_param;
using ros2_medkit_graph_watchdog::kUnsetParameterRendering;
using ros2_medkit_graph_watchdog::param_entry_value;
using ros2_medkit_graph_watchdog::param_values_equal;
using ros2_medkit_graph_watchdog::ParamDriftConfig;
using ros2_medkit_graph_watchdog::ParamDriftPolicy;
using ros2_medkit_graph_watchdog::read_gap;
using ros2_medkit_graph_watchdog::unset_param_value;
using Params = std::map<std::string, json>;

namespace {
/// The `got=` field of one drifted descriptor - the text an operator reads for the value the node
/// actually holds. Empty when the descriptor has no such field, which fails the assertion rather
/// than quietly comparing two empty strings.
std::string rendered_got(const std::string & descriptor) {
  const auto at = descriptor.find(" got=");
  return at == std::string::npos ? std::string{} : descriptor.substr(at + 5);
}
}  // namespace

TEST(GlobMatch, StarWildcard) {
  EXPECT_TRUE(glob_match("*_stamp", "header_stamp"));
  EXPECT_TRUE(glob_match("qos_overrides.*", "qos_overrides./scan.reliability"));
  EXPECT_TRUE(glob_match("*", "anything"));
  EXPECT_TRUE(glob_match("exact", "exact"));
  EXPECT_FALSE(glob_match("*_stamp", "stamp_header"));
  EXPECT_FALSE(glob_match("exact", "exacter"));
}

TEST(ParamValuesEqual, NumericNormalization) {
  EXPECT_TRUE(param_values_equal(json(20), json(20.0)));  // int vs double, same value
  EXPECT_TRUE(param_values_equal(json(1.5), json(1.5)));
  EXPECT_FALSE(param_values_equal(json(20.0), json(5.0)));
  EXPECT_TRUE(param_values_equal(json(false), json(false)));
  EXPECT_FALSE(param_values_equal(json(false), json(true)));
  EXPECT_TRUE(param_values_equal(json("a"), json("a")));
  EXPECT_FALSE(param_values_equal(json("a"), json("b")));
}

// A NaN-valued param (common "unset/disabled" sentinel) must NOT read as drifted against
// itself - IEEE NaN != NaN would make it a permanent false positive that never clears.
TEST(ParamValuesEqual, NaNParamIsStable) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(param_values_equal(json(nan), json(nan)));
  EXPECT_FALSE(param_values_equal(json(nan), json(1.0)));
  // Signed zero is not a drift. Written as an INTEGER parameter against a pin carrying a decimal
  // point, because that is the pairing our own comparison has to decide: two doubles of the same
  // kind never reach int_equals_float_exactly at all and only exercise nlohmann's operator==.
  // -0.0 is the value that makes the integrality check subtle - it is neither greater nor less
  // than zero, so a naive `frac != 0.0` test would have to get the sign of zero right.
  EXPECT_TRUE(param_values_equal(json(0), json(-0.0)));
  EXPECT_TRUE(param_values_equal(json(-0.0), json(0)));  // and the mirrored branch
}

// int64 values differing beyond 2^53 must be caught: a `double` coercion collapses them into a
// MISSED drift on 64-bit IDs, counters and epoch-ns params. Inside an ARRAY, because that is a
// path of our own - the recursion in param_values_equal - and because a 1-D array of large
// integers against a pin written with decimal points is exactly how the two get mixed. Two plain
// integers of the same kind would fall straight through to nlohmann's operator== and test nothing
// this package owns.
TEST(ParamValuesEqual, LargeInt64ExactCompareInsideAnArray) {
  EXPECT_TRUE(param_values_equal(json::array({9007199254740992LL}), json::array({9007199254740992.0})));
  EXPECT_FALSE(param_values_equal(json::array({9007199254740993LL}), json::array({9007199254740992.0})));
  // A late element is the one that gets lost when the recursion stops early.
  EXPECT_FALSE(
      param_values_equal(json::array({1LL, 2LL, 9007199254740993LL}), json::array({1.0, 2.0, 9007199254740992.0})));
}

TEST(ParamValuesEqual, NaNInsideArray) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(param_values_equal(json::array({nan, 1.0}), json::array({nan, 1.0})));  // never-changed NaN[] not drift
  EXPECT_FALSE(param_values_equal(json::array({nan, 1.0}), json::array({nan, 2.0})));
  EXPECT_FALSE(param_values_equal(json::array({1.0}), json::array({1.0, 2.0})));  // length differs
}

TEST(ParamDriftConfig, FlattensNestedExpectToDottedParamNames) {
  // The gateway un-nests dotted config keys, so a pin on a dotted param name
  // (expect.qos_overrides./scan.reliability) arrives NESTED. from_json must flatten it back
  // to the full dotted param name - the key the observed param map actually uses. `baseline`/
  // `ignore` are single-level keys and survive extract_detector_config's nested output as-is.
  json detector_cfg = {
      {"baseline", false},
      {"expect", {{"use_sim_time", false}, {"qos_overrides", {{"/scan", {{"reliability", "best_effort"}}}}}}},
      {"ignore", json::array({"*_stamp"})}};
  const auto cfg = ParamDriftConfig::from_json(detector_cfg);
  EXPECT_FALSE(cfg.baseline_capture);
  ASSERT_EQ(cfg.expect.count("use_sim_time"), 1u);
  EXPECT_EQ(cfg.expect.at("use_sim_time"), json(false));
  ASSERT_EQ(cfg.expect.count("qos_overrides./scan.reliability"), 1u);  // recovered from nesting
  EXPECT_EQ(cfg.expect.at("qos_overrides./scan.reliability"), json("best_effort"));
  ASSERT_EQ(cfg.ignore_globs.size(), 1u);
  EXPECT_EQ(cfg.ignore_globs[0], "*_stamp");
}

TEST(ParamDriftConfig, MalformedExpectWarnsAndKeepsEmpty) {
  // A present-but-non-object expect (operator error) must warn, like the other fields.
  const auto cfg = ParamDriftConfig::from_json({{"expect", "use_sim_time"}});  // scalar, not object
  EXPECT_TRUE(cfg.expect.empty());
  EXPECT_FALSE(cfg.warnings.empty());
}

TEST(ParamDriftConfig, MisspeltKeyWarnsInsteadOfBeingDropped) {
  // A wrong TYPE on a known key already warned; a wrong NAME did not, though it fails
  // the same way: `ignore_globs` (the struct's field name) leaves the globs empty and
  // param_drift keeps flagging exactly the params the operator meant to suppress.
  const auto cfg = ParamDriftConfig::from_json({{"ignore_globs", json::array({"*_stamp"})}});
  EXPECT_TRUE(cfg.ignore_globs.empty());
  ASSERT_EQ(cfg.warnings.size(), 1u);
  EXPECT_NE(cfg.warnings[0].find("ignore_globs"), std::string::npos);
}

TEST(ParamDriftConfig, EveryKnownKeyIsAcceptedWithoutWarning) {
  // Guards the known-key set against drifting away from the fields from_json reads:
  // a key dropped from the set here would start warning on a correct config.
  // baseline stays ENABLED here: `ignore` filters only the self-captured set, so pairing it with
  // `baseline: false` is a dead combination that now warns on purpose (see the test below).
  const auto cfg = ParamDriftConfig::from_json({{"baseline", true},
                                                {"ignore", json::array({"*_stamp"})},
                                                {"max_reads_per_tick", 4},
                                                {"expect", {{"use_sim_time", false}}}});
  EXPECT_TRUE(cfg.warnings.empty()) << (cfg.warnings.empty() ? "" : cfg.warnings.front());
}

TEST(ParamDriftConfig, DefaultsWhenEmpty) {
  const auto cfg = ParamDriftConfig::from_json(json::object());
  EXPECT_TRUE(cfg.baseline_capture);  // default true
  EXPECT_TRUE(cfg.expect.empty());
  EXPECT_TRUE(cfg.ignore_globs.empty());
  EXPECT_TRUE(cfg.warnings.empty());
}

// Malformed config must not be dropped silently - it must warn (a mistyped `ignore`
// would otherwise re-enable drift on suppressed params with no trail).
TEST(ParamDriftConfig, MalformedIgnoreWarnsAndKeepsEmpty) {
  const auto scalar = ParamDriftConfig::from_json({{"ignore", "*_stamp"}});  // scalar, not array
  EXPECT_TRUE(scalar.ignore_globs.empty());
  EXPECT_FALSE(scalar.warnings.empty());

  const auto wrong_elem = ParamDriftConfig::from_json({{"ignore", json::array({1, 2})}});  // int array
  EXPECT_TRUE(wrong_elem.ignore_globs.empty());
  EXPECT_FALSE(wrong_elem.warnings.empty());
}

TEST(ParamDriftConfig, MalformedBaselineWarnsAndKeepsDefault) {
  const auto cfg = ParamDriftConfig::from_json({{"baseline", "false"}});  // quoted string, not bool
  EXPECT_TRUE(cfg.baseline_capture);                                      // explicit override lost -> kept default true
  EXPECT_FALSE(cfg.warnings.empty());
}

TEST(ParamDriftConfig, MaxReadsPerTick) {
  EXPECT_EQ(ParamDriftConfig::from_json(json::object()).max_reads_per_tick, 8);  // default
  EXPECT_EQ(ParamDriftConfig::from_json({{"max_reads_per_tick", 20}}).max_reads_per_tick, 20);
  const auto bad = ParamDriftConfig::from_json({{"max_reads_per_tick", 0}});  // non-positive
  EXPECT_EQ(bad.max_reads_per_tick, 8);                                       // kept default
  EXPECT_FALSE(bad.warnings.empty());
}

TEST(ParamDriftPolicy, SelfCaptureFirstSeenNoDrift) {
  ParamDriftPolicy p{ParamDriftConfig{}};  // baseline true, no expect
  const auto out =
      p.evaluate("/n", Params{{"speed", json(1.0)}, {"use_sim_time", json(false)}}, /*capture_allowed=*/true);
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, RuntimeChangeDriftsThenClears) {
  ParamDriftPolicy p{ParamDriftConfig{}};
  p.evaluate("/n", Params{{"speed", json(1.0)}}, true);             // capture
  auto out = p.evaluate("/n", Params{{"speed", json(0.2)}}, true);  // changed
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(out[0].find("speed"), std::string::npos);
  out = p.evaluate("/n", Params{{"speed", json(1.0)}}, true);  // restored
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, NewParamAfterCaptureIsSilent) {
  ParamDriftPolicy p{ParamDriftConfig{}};
  p.evaluate("/n", Params{{"a", json(1)}}, true);
  const auto out = p.evaluate("/n", Params{{"a", json(1)}, {"b", json(2)}}, true);  // b is new
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, IgnoredParamNeverDrifts) {
  ParamDriftConfig cfg;
  cfg.ignore_globs = {"*_stamp"};
  ParamDriftPolicy p{cfg};
  p.evaluate("/n", Params{{"header_stamp", json(1)}}, true);
  const auto out = p.evaluate("/n", Params{{"header_stamp", json(999)}}, true);
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, ExpectRuleFiresRegardlessOfBaseline) {
  ParamDriftConfig cfg;
  cfg.baseline_capture = false;  // no self-capture
  cfg.expect = {{"use_sim_time", json(false)}};
  ParamDriftPolicy p{cfg};
  // First observation already violates - no prior "capture" needed.
  auto out = p.evaluate("/n", Params{{"use_sim_time", json(true)}}, true);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(out[0].find("use_sim_time"), std::string::npos);
  out = p.evaluate("/n", Params{{"use_sim_time", json(false)}}, true);
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, ExpectAbsentParamIsNotDrift) {
  ParamDriftConfig cfg;
  cfg.baseline_capture = false;
  cfg.expect = {{"use_sim_time", json(false)}};
  ParamDriftPolicy p{cfg};
  const auto out = p.evaluate("/n", Params{{"other", json(1)}}, true);  // node lacks use_sim_time
  EXPECT_TRUE(out.empty());
}

TEST(ParamDriftPolicy, BaselineFalseIgnoresUnlistedParams) {
  ParamDriftConfig cfg;
  cfg.baseline_capture = false;
  cfg.expect = {{"use_sim_time", json(false)}};
  ParamDriftPolicy p{cfg};
  p.evaluate("/n", Params{{"use_sim_time", json(false)}, {"speed", json(1.0)}}, true);
  const auto out = p.evaluate("/n", Params{{"use_sim_time", json(false)}, {"speed", json(9.0)}}, true);
  EXPECT_TRUE(out.empty());  // speed changed but unlisted + no baseline
}

TEST(ParamDriftPolicy, ExpectPlusBaselineBothProduceDescriptors) {
  ParamDriftConfig cfg;
  cfg.expect = {{"use_sim_time", json(false)}};  // baseline_capture stays true
  ParamDriftPolicy p{cfg};
  p.evaluate("/n", Params{{"use_sim_time", json(false)}, {"speed", json(1.0)}}, true);  // capture
  const auto out = p.evaluate("/n", Params{{"use_sim_time", json(true)}, {"speed", json(0.2)}}, true);
  ASSERT_EQ(out.size(), 2u);  // one expect drift + one baseline drift
  const std::string joined = out[0] + out[1];
  EXPECT_NE(joined.find("use_sim_time"), std::string::npos);
  EXPECT_NE(joined.find("speed"), std::string::npos);
}

TEST(ParamDriftPolicy, CaptureGatedByCaptureAllowed) {
  ParamDriftPolicy p{ParamDriftConfig{}};
  EXPECT_TRUE(
      p.evaluate("/n", {{"x", json(1.0)}}, /*capture_allowed=*/false).empty());  // not armed: no capture, no drift
  EXPECT_TRUE(p.evaluate("/n", {{"x", json(9.0)}}, /*capture_allowed=*/true)
                  .empty());                                  // first armed sweep: captures 9.0, no drift
  const auto d = p.evaluate("/n", {{"x", json(1.0)}}, true);  // now 1.0 != 9.0 -> drift
  ASSERT_EQ(d.size(), 1u);
  EXPECT_NE(d[0].find("/n"), std::string::npos);
  EXPECT_NE(d[0].find('x'), std::string::npos);
}

TEST(ParamDriftPolicy, ExpectFiresEvenWhenCaptureNotAllowed) {
  ParamDriftConfig cfg;
  cfg.baseline_capture = false;
  cfg.expect = {{"use_sim_time", json(false)}};
  ParamDriftPolicy p{cfg};
  const auto d = p.evaluate("/n", {{"use_sim_time", json(true)}}, /*capture_allowed=*/false);
  ASSERT_EQ(d.size(), 1u);  // expect is absolute; independent of capture gating
}

TEST(ParamDriftPolicy, ForgetDropsBaseline) {
  ParamDriftPolicy p{ParamDriftConfig{}};
  p.evaluate("/n", {{"x", json(1.0)}}, true);
  p.forget("/n");
  EXPECT_TRUE(p.evaluate("/n", {{"x", json(0.2)}}, true).empty());  // re-captures 0.2, not a drift
}

// read_gap is the only thing enforcing max_reads_per_tick: the reader takes one target per
// pass and waits out the rest of its slot, so a zero gap is no budget at all.
TEST(ParamDriftPolicy, ReadGapSpreadsTheBudgetOverTheTickPeriod) {
  EXPECT_EQ(read_gap(1000, 8), std::chrono::microseconds(125000));
  EXPECT_EQ(read_gap(200, 4), std::chrono::microseconds(50000));
}

// A budget larger than the tick period in milliseconds truncates to zero under integer
// millisecond division, which removes the bound instead of tightening it.
TEST(ParamDriftPolicy, ReadGapStaysNonZeroWhenTheBudgetExceedsTheTickInMilliseconds) {
  EXPECT_GT(read_gap(1000, 2000).count(), 0);
  EXPECT_EQ(read_gap(1000, 2000), std::chrono::microseconds(500));
}

// A non-positive budget must not divide by zero; it falls back to one read per period.
TEST(ParamDriftPolicy, ReadGapFallsBackToOneReadPerPeriod) {
  EXPECT_EQ(read_gap(1000, 0), std::chrono::microseconds(1000000));
  EXPECT_EQ(read_gap(1000, -5), std::chrono::microseconds(1000000));
}

// A fault description is cut at a byte offset, so a rendered value must never contain a
// multi-byte sequence a cut could split, and must never make the renderer throw.
TEST(ParamDriftPolicy, RenderedValuesAreAsciiOnly) {
  ParamDriftPolicy p(ParamDriftConfig{});
  p.evaluate("/n", Params{{"path", json("/maps/cafe/map.yaml")}}, true);
  const auto out = p.evaluate("/n", Params{{"path", json("/maps/caf\xc3\xa9/map.yaml")}}, true);
  ASSERT_EQ(out.size(), 1u);
  for (const char c : out[0]) {
    EXPECT_LT(static_cast<unsigned char>(c), 0x80u)
        << "non-ASCII byte in a description that is later cut at a byte offset";
  }
}

// A parameter whose bytes are not valid UTF-8 must not throw out of evaluate(): the plugin's
// per-detector guard would swallow it and the detector would then neither raise nor clear for as
// long as that value exists, so a confirmed fault could never heal.
TEST(ParamDriftPolicy, InvalidUtf8InAValueDoesNotThrow) {
  ParamDriftPolicy p(ParamDriftConfig{});
  // No explicit length: the literal is 10 bytes and hand-counting it got that wrong once, which
  // read past the end of the literal and only showed up under AddressSanitizer.
  const std::string bad("bad\xff\xfevalue");
  p.evaluate("/n", Params{{"s", json("ok")}}, true);
  EXPECT_NO_THROW({
    const auto out = p.evaluate("/n", Params{{"s", json(bad)}}, true);
    EXPECT_EQ(out.size(), 1u);
  });
}

// One array-valued parameter must not be able to consume the whole description budget.
//
// The 400-element array is the `got=` field, so that is what has to be measured. Measuring the text
// between `baseline=` and ` got=` measures the four-element BASELINE, which is a dozen characters
// long whatever elide() does - disabling elision entirely left that form of this test green.
//
// The budget is asserted against a LITERAL and not against the constant itself. Asserting
// `<= kMaxRenderedValueChars` says only that elide() honours whatever number it is handed, so
// widening the budget widens the assertion with it and the description cap this feeds silently
// grows: at 480 a single value fills the whole aggregated description on its own and every other
// drifted entry is cut. That is the same class of defect this test was repaired to catch.
TEST(ParamDriftPolicy, LongValuesAreElided) {
  constexpr std::size_t kBudget = 64;  // pinned; see TheRenderedValueBudgetIsSixtyFourCharacters
  const json big(std::vector<int>(400, 7));
  ParamDriftPolicy p(ParamDriftConfig{});
  p.evaluate("/n", Params{{"arr", json(std::vector<int>(4, 1))}}, true);
  const auto out = p.evaluate("/n", Params{{"arr", big}}, true);
  ASSERT_EQ(out.size(), 1u);
  const auto got_at = out[0].find(" got=");
  ASSERT_NE(got_at, std::string::npos);
  const std::string rendered_got = out[0].substr(got_at + 5);
  EXPECT_LE(rendered_got.size(), kBudget)
      << "the changed value rendered to " << rendered_got.size() << " chars, past the per-value budget";
  EXPECT_LT(rendered_got.size(), big.dump().size())
      << "the 400-element array was rendered in full, so elide() is a no-op on the field that "
         "actually carries a long value";
}

// The budget itself, pinned once with a literal so every other assertion may name the constant.
//
// Seven of these fit inside AggregatedFault's 480-character description and two fit inside the
// 150 characters one app may spend, which is what makes a multi-app, multi-parameter report
// readable at all. Raising it is not a free tuning choice: the description cap does not grow with
// it, so a wider value budget buys one entry room at the direct expense of every entry behind it.
TEST(ParamDriftPolicy, TheRenderedValueBudgetIsSixtyFourCharacters) {
  EXPECT_EQ(ParamDriftPolicy::kMaxRenderedValueChars, 64u)
      << "the per-value rendering budget moved. That is a change to what an operator can read in a "
         "capped description, not an implementation detail: update this pin deliberately, and check "
         "how many entries still fit under AggregatedFault::kMaxDescriptionChars when you do";
}

// The gap has a floor as well as microsecond resolution: a budget above a million reads per
// second truncates to zero again, and a zero gap is a reader that never waits.
TEST(ParamDriftPolicy, ReadGapNeverReachesZeroEvenForAnAbsurdBudget) {
  EXPECT_EQ(read_gap(1000, 2000000), std::chrono::microseconds(1));
  EXPECT_GT(read_gap(1, 100000).count(), 0);
}

// The reader spends one gap per round trip, so what the budget actually buys the watched node is
// `period / gap` round trips per tick period - and that is NOT `reads` unless the division comes
// out even. Flooring makes the gap shorter than asked and the rate correspondingly higher:
// `tick_interval_ms: 1` with `max_reads_per_tick: 600`, both accepted configurations, floors to a
// 1 us gap and 1000 round trips against a documented bound of 600.
//
// The whole accepted range is swept rather than one pair, because the overshoot only appears when
// the division is uneven and any single hand-picked pair is overwhelmingly likely to divide evenly.
// Both directions are pinned: the rate must never exceed the budget, and the gap must not be
// padded beyond the one microsecond the rounding can add, or the knob would silently throttle.
TEST(ParamDriftPolicy, ReadGapNeverLetsTheAchievableRateExceedTheBudget) {
  const std::vector<int> ticks{1, 2, 3, 5, 7, 10, 100, 125, 250, 333, 1000, 5000};
  const std::vector<int> reads{1, 2, 3, 6, 7, 8, 9, 11, 13, 64, 600, 999, 1024, 7777, 100000};
  for (const int tick_ms : ticks) {
    for (const int budget : reads) {
      const auto gap = read_gap(tick_ms, budget).count();
      const std::int64_t period_us = static_cast<std::int64_t>(tick_ms) * 1000;
      ASSERT_GT(gap, 0) << "tick_interval_ms=" << tick_ms << " max_reads_per_tick=" << budget;
      EXPECT_LE(period_us / gap, budget)
          << "tick_interval_ms=" << tick_ms << " with max_reads_per_tick=" << budget << " yields a " << gap
          << " us gap, i.e. " << period_us / gap
          << " round trips per tick period: the knob an operator lowered to protect a node is being "
             "overshot";
      // Tightness, so "hold the bound" cannot be met by simply pacing far slower than asked. One
      // slot per read plus at most the single microsecond rounding up can add.
      EXPECT_LT(gap * budget, period_us + budget)
          << "tick_interval_ms=" << tick_ms << " with max_reads_per_tick=" << budget << " spends " << gap
          << " us per round trip, well beyond the slot the budget asks for";
    }
  }
}

// The exact configuration the bound used to break on, spelled out so the regression is named.
TEST(ParamDriftPolicy, ReadGapHoldsTheBoundOnASubMillisecondSlot) {
  EXPECT_EQ(read_gap(1, 600), std::chrono::microseconds(2))
      << "a 1000 us tick period over 600 reads is 1.67 us per read; rounding down to 1 us buys the "
         "sweep 1000 round trips per tick period against a bound of 600";
}

// is_number_integer() spans the whole 64-bit range and get<int>() truncates, so the range check
// has to happen on the wide value. Otherwise 4294967297 arrives as 1, and -4294967295 arrives as
// 1 as well - a negative request passing a positive-only guard, both with no warning.
TEST(ParamDriftPolicy, OutOfRangeBudgetIsRejectedNotTruncated) {
  for (const auto & bad : {json(4294967297LL), json(-4294967295LL), json(-1), json(0)}) {
    const auto cfg = ParamDriftConfig::from_json(json{{"max_reads_per_tick", bad}});
    EXPECT_EQ(cfg.max_reads_per_tick, 8) << "accepted " << bad.dump();
    EXPECT_FALSE(cfg.warnings.empty()) << "accepted " << bad.dump() << " with no warning";
  }
  const auto ok = ParamDriftConfig::from_json(json{{"max_reads_per_tick", 32}});
  EXPECT_EQ(ok.max_reads_per_tick, 32);
  EXPECT_TRUE(ok.warnings.empty());

  // The ceiling itself. An off-by-one here either rejects a legal budget or lets the value above
  // it through, and neither shows up anywhere else - every other case in this test is orders of
  // magnitude away from the bound.
  const auto at_ceiling =
      ParamDriftConfig::from_json(json{{"max_reads_per_tick", ParamDriftConfig::kMaxReadsPerTickCeiling}});
  EXPECT_EQ(at_ceiling.max_reads_per_tick, ParamDriftConfig::kMaxReadsPerTickCeiling);
  EXPECT_TRUE(at_ceiling.warnings.empty()) << "the ceiling value itself must be accepted";

  const auto past_ceiling =
      ParamDriftConfig::from_json(json{{"max_reads_per_tick", ParamDriftConfig::kMaxReadsPerTickCeiling + 1}});
  EXPECT_EQ(past_ceiling.max_reads_per_tick, 8) << "a budget one past the ceiling was accepted";
  EXPECT_FALSE(past_ceiling.warnings.empty());
}

// nlohmann compares an integer against a float by casting the integer to double, which collapses
// values differing beyond 2^53 - exactly the missed drift the equality helper claims to avoid.
// Reachable from an expect pin written with a decimal point against an integer parameter.
TEST(ParamDriftPolicy, IntegerVersusFloatIsComparedExactly) {
  EXPECT_FALSE(param_values_equal(json(9007199254740993LL), json(9007199254740992.0)));
  EXPECT_TRUE(param_values_equal(json(9007199254740992LL), json(9007199254740992.0)));
  EXPECT_TRUE(param_values_equal(json(20), json(20.0)));  // the ordinary case must stay equal
  EXPECT_FALSE(param_values_equal(json(1), json(1.5)));
  EXPECT_FALSE(param_values_equal(json(1), json(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_FALSE(param_values_equal(json(1), json(std::numeric_limits<double>::infinity())));

  // The MIRRORED branch: a float on the left against an integer on the right swaps the operands
  // before calling int_equals_float_exactly, and only the int-left form was covered - so the swap
  // could have been the wrong way round in one direction and nothing here would have said so. A
  // float-left NaN is the ordinary shape of it: a NaN-valued parameter read from a node, compared
  // against an integer pin.
  EXPECT_FALSE(param_values_equal(json(std::numeric_limits<double>::quiet_NaN()), json(1)));
  EXPECT_FALSE(param_values_equal(json(std::numeric_limits<double>::infinity()), json(1)));
  EXPECT_FALSE(param_values_equal(json(1.5), json(1)));
  EXPECT_TRUE(param_values_equal(json(20.0), json(20)));
  EXPECT_FALSE(param_values_equal(json(9007199254740992.0), json(9007199254740993LL)));

  // A negative pin against an unsigned parameter, which is the ordinary shape: the transport
  // stores every positive integer unsigned. Converting the double to uint64_t before checking its
  // sign is undefined behaviour, and on x86-64 -1.0 lands on UINT64_MAX - so the one parameter
  // value that would then compare EQUAL to it is the largest one representable.
  EXPECT_FALSE(param_values_equal(json(std::uint64_t{18446744073709551615ULL}), json(-1.0)));
  EXPECT_FALSE(param_values_equal(json(std::uint64_t{5}), json(-1.0)));
  EXPECT_TRUE(param_values_equal(json(std::uint64_t{5}), json(5.0)));  // the branch still works

  // The 2^63 bounds. A double at or past +2^63 has no int64_t to compare against, so it is
  // reported as different even where the two are numerically the same - erring towards raising a
  // drift rather than missing one. -2^63 IS representable and must still compare equal.
  EXPECT_FALSE(param_values_equal(json(std::uint64_t{9223372036854775808ULL}), json(9223372036854775808.0)));
  EXPECT_FALSE(param_values_equal(json(1), json(-18446744073709551616.0)));  // -2^64
  EXPECT_TRUE(param_values_equal(json(std::numeric_limits<std::int64_t>::min()), json(-9223372036854775808.0)));
}

// Configuration that parses cleanly and can never do anything must say so. Silence is the worst
// outcome here: the detector looks configured and reports nothing, forever.
TEST(ParamDriftConfig, InertCombinationsAreReported) {
  const auto nothing_to_read = ParamDriftConfig::from_json(json{{"baseline", false}});
  EXPECT_FALSE(nothing_to_read.warnings.empty()) << "baseline off with no expect reads nothing at all";

  const auto dead_ignore = ParamDriftConfig::from_json(
      json{{"baseline", false}, {"expect", {{"use_sim_time", false}}}, {"ignore", {"*_x"}}});
  EXPECT_FALSE(dead_ignore.warnings.empty()) << "ignore cannot filter anything when baseline is off";

  // And the ordinary configurations stay quiet.
  EXPECT_TRUE(ParamDriftConfig::from_json(json::object()).warnings.empty());
  EXPECT_TRUE(ParamDriftConfig::from_json(json{{"baseline", false}, {"expect", {{"a", 1}}}}).warnings.empty());
}

// An ignore pattern that matches every parameter name suppresses the whole self-captured half.
// The detector then looks configured and reports nothing, which at runtime is indistinguishable
// from it working and finding nothing - the exact silent failure it is built to rule out.
TEST(ParamDriftConfig, ACatchAllIgnoreIsReported) {
  EXPECT_FALSE(ParamDriftConfig::from_json(json{{"ignore", {"*"}}}).warnings.empty());
  EXPECT_FALSE(ParamDriftConfig::from_json(json{{"ignore", {"**"}}}).warnings.empty())
      << "a glob of nothing but stars matches everything too";
  EXPECT_FALSE(ParamDriftConfig::from_json(json{{"ignore", {"debug_*", "*"}}}).warnings.empty())
      << "one catch-all among several patterns still swallows everything";

  // Pins still report, so this is a different message - but still not silence.
  EXPECT_FALSE(ParamDriftConfig::from_json(json{{"ignore", {"*"}}, {"expect", {{"a", 1}}}}).warnings.empty());

  // An ordinary pattern stays quiet.
  EXPECT_TRUE(ParamDriftConfig::from_json(json{{"ignore", {"debug_*"}}}).warnings.empty());
}

// A pin written two ways that flatten to the same parameter name loses one of them, and a pin
// with no value at all disappears. Both used to happen with no diagnostic.
TEST(ParamDriftConfig, LostExpectPinsAreReported) {
  const auto collide = ParamDriftConfig::from_json(json::parse(R"({"expect":{"a":{"b":1},"a.b":2}})"));
  EXPECT_FALSE(collide.warnings.empty());
  EXPECT_EQ(collide.expect.size(), 1u);

  const auto empty_pin = ParamDriftConfig::from_json(json::parse(R"({"expect":{"qos_overrides":{}}})"));
  EXPECT_FALSE(empty_pin.warnings.empty());
  EXPECT_TRUE(empty_pin.expect.empty());
}

// Parameter NAMES come verbatim from the watched node and are never validated, so they carry the
// same two hazards values do. An invalid UTF-8 byte in the name, or a valid multi-byte name split
// by the aggregated description's byte-offset cut, makes the gateway's JSON writer throw - which
// answers 500 on GET /faults and takes every OTHER fault down with it.
TEST(ParamDriftPolicy, ParameterNamesAreSanitisedToo) {
  ParamDriftPolicy p(ParamDriftConfig{});
  const std::string bad_name("na\xff\xfeme", 6);
  p.evaluate("/n", Params{{bad_name, json(1.0)}}, true);
  const auto out = p.evaluate("/n", Params{{bad_name, json(2.0)}}, true);
  ASSERT_EQ(out.size(), 1u);
  for (const char c : out[0]) {
    EXPECT_LT(static_cast<unsigned char>(c), 0x80u) << "non-ASCII byte from a parameter NAME";
  }
  EXPECT_NO_THROW(json(out[0]).dump());
}

// A parameter a node DECLARED and never SET must not read as `null` in a fault description.
//
// The transport answers such a parameter with a JSON null, and a NaN double DUMPS as `null` too, so
// one description said the same thing for "the node never set this parameter" and "the value is
// NaN" - two different faults with two different remedies, and nothing in the text to tell them
// apart. Both are reachable from an `expect` pin, so both really do end up in front of an operator.
//
// The equality half is asserted here as well, and not assumed: the marker is carried through the
// same map a real value is, so whatever it is has to keep the pin's verdict intact - unequal to the
// pinned value (or the raise this rendering describes would silently disappear) and equal to
// itself (or a parameter that is unset every sweep would drift against its own baseline for ever).
TEST(ParamDriftPolicy, AnUnsetParameterIsRenderedDistinctlyFromANaNValue) {
  ParamDriftConfig cfg;
  cfg.baseline_capture = false;
  cfg.expect = {{"threshold", json(1.0)}};
  ParamDriftPolicy p{cfg};

  const auto unset = p.evaluate("/unset", Params{{"threshold", unset_param_value()}}, /*capture_allowed=*/false);
  ASSERT_EQ(unset.size(), 1u) << "an expect pin on a parameter the node never set must still raise: a node with no "
                                 "value at all does not satisfy a pin that says the value must be 1.0";
  const auto nan_drift = p.evaluate("/nan", Params{{"threshold", json(std::numeric_limits<double>::quiet_NaN())}},
                                    /*capture_allowed=*/false);
  ASSERT_EQ(nan_drift.size(), 1u) << "a NaN value against a pin of 1.0 is a drift too";

  const std::string unset_got = rendered_got(unset[0]);
  const std::string nan_got = rendered_got(nan_drift[0]);
  ASSERT_FALSE(unset_got.empty()) << "no got= field in '" << unset[0] << "'";
  ASSERT_FALSE(nan_got.empty()) << "no got= field in '" << nan_drift[0] << "'";
  EXPECT_NE(unset_got, nan_got) << "an unset parameter and a NaN value both rendered as '" << unset_got
                                << "', so an operator cannot tell which fault they are looking at";
  EXPECT_EQ(unset_got, std::string(kUnsetParameterRendering)) << "an unset parameter rendered as '" << unset_got << "'";
  EXPECT_NE(unset_got, "null") << "the rendering of an unset parameter is exactly the text it must stop being";

  // The description is cut at a BYTE offset by the aggregated fault, so the marker's rendering
  // carries the same ASCII-only obligation every other rendered value does.
  for (const char c : unset[0]) {
    EXPECT_LT(static_cast<unsigned char>(c), 0x80u) << "non-ASCII byte in the unset rendering";
  }

  // The verdict the rendering describes, pinned directly.
  EXPECT_FALSE(param_values_equal(unset_param_value(), json(1.0))) << "an unset parameter would satisfy a pin";
  EXPECT_FALSE(param_values_equal(json(1.0), unset_param_value())) << "and the mirrored operand order";
  EXPECT_TRUE(param_values_equal(unset_param_value(), unset_param_value()))
      << "a parameter that is unset on every sweep would drift against its own captured baseline for ever";

  // A JSON OBJECT is the marker's whole safety argument: it is a shape no ROS parameter value and
  // no `expect` pin can take, so it cannot collide with something a node or an operator wrote. A
  // string sentinel could - a node is free to hold the string below.
  EXPECT_TRUE(unset_param_value().is_object());
  EXPECT_FALSE(is_unset_param(json(kUnsetParameterRendering)))
      << "a node holding this exact string would be reported as unset";
  EXPECT_FALSE(is_unset_param(json(nullptr)));
  EXPECT_FALSE(is_unset_param(json(std::numeric_limits<double>::quiet_NaN())));
}

// Which transport replies become the marker. Both signals are load-bearing and neither subsumes the
// other: the TYPE is what a live rclcpp node sends (`not_set` alongside a null value), and the null
// VALUE is the backstop for a transport that omits the type, since null is not a legal encoding of
// any ROS parameter value and so must never reach a description as one. A NaN double is caught by
// neither - it is a float that merely DUMPS as `null` - which is the distinction being kept.
TEST(ParamDriftPolicy, OnlyADeclaredButUnsetTransportEntryBecomesTheMarker) {
  EXPECT_TRUE(is_unset_param(param_entry_value(json{{"name", "a"}, {"type", "not_set"}, {"value", nullptr}})))
      << "the shape a live rclcpp node produces for a parameter it declared and never set";
  EXPECT_TRUE(is_unset_param(param_entry_value(json{{"name", "a"}, {"value", nullptr}})))
      << "a null value with no type is still not a legal ROS parameter value";

  EXPECT_FALSE(is_unset_param(param_entry_value(json{{"name", "a"}, {"type", "double"}, {"value", 1.0}})));
  EXPECT_EQ(param_entry_value(json{{"name", "a"}, {"type", "double"}, {"value", 1.0}}), json(1.0));
  EXPECT_EQ(param_entry_value(json{{"name", "a"}, {"type", "string"}, {"value", "not_set"}}), json("not_set"))
      << "a STRING parameter whose value is the word 'not_set' is an ordinary value, not an unset parameter";
  EXPECT_FALSE(is_unset_param(
      param_entry_value(json{{"name", "a"}, {"type", "double"}, {"value", std::numeric_limits<double>::quiet_NaN()}})))
      << "a NaN double is a value the node HAS; folding it into the marker would recreate the ambiguity";
  EXPECT_FALSE(is_unset_param(param_entry_value(json{{"name", "a"}, {"type", "bool"}, {"value", false}})))
      << "false is a value, not an absence";
}

// Head-only truncation renders two different values identically whenever they share a long prefix -
// the common case for robot_description and for nav2 footprint arrays where only a late element
// differs. The fault would fire correctly and then prove nothing about what changed.
TEST(ParamDriftPolicy, ElisionKeepsBothEndsSoValuesStayDistinguishable) {
  std::vector<double> before(40, 0.5);
  std::vector<double> after = before;
  after.back() = 9.5;  // only the LAST element differs
  ParamDriftPolicy p(ParamDriftConfig{});
  p.evaluate("/costmap", Params{{"footprint", json(before)}}, true);
  const auto out = p.evaluate("/costmap", Params{{"footprint", json(after)}}, true);
  ASSERT_EQ(out.size(), 1u);
  const auto baseline_at = out[0].find("baseline=");
  const auto got_at = out[0].find(" got=");
  ASSERT_NE(baseline_at, std::string::npos);
  ASSERT_NE(got_at, std::string::npos);
  const std::string rendered_baseline = out[0].substr(baseline_at + 9, got_at - baseline_at - 9);
  const std::string rendered_got = out[0].substr(got_at + 5);
  EXPECT_NE(rendered_baseline, rendered_got)
      << "both values rendered as '" << rendered_baseline << "', so the fault shows no difference";
}
