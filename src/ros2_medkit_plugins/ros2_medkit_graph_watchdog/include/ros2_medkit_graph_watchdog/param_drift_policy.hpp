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
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"  // collect_unknown_detector_keys

namespace ros2_medkit_graph_watchdog {

/// Minimal glob matcher supporting `*` (matches any run, including empty). No `?`,
/// no character classes - ROS param ignore patterns only need `*`.
inline bool glob_match(const std::string & pattern, const std::string & text) {
  std::size_t p = 0;
  std::size_t t = 0;
  std::size_t star = std::string::npos;
  std::size_t mark = 0;
  while (t < text.size()) {
    if (p < pattern.size() && pattern[p] == '*') {
      star = p++;
      mark = t;
    } else if (p < pattern.size() && pattern[p] == text[t]) {
      ++p;
      ++t;
    } else if (star != std::string::npos) {
      p = star + 1;
      t = ++mark;
    } else {
      return false;
    }
  }
  while (p < pattern.size() && pattern[p] == '*') {
    ++p;
  }
  return p == pattern.size();
}

/// The transport's own name for the type of a parameter a node DECLARED and never SET.
/// Ros2ParameterTransport::parameter_type_to_string maps rclcpp's PARAMETER_NOT_SET to this.
inline constexpr const char * kNotSetParameterType = "not_set";

/// Key of the out-of-band marker that stands in for such a parameter inside an observed param map.
inline constexpr const char * kUnsetParameterKey = "x-medkit-unset";

/// How that marker reads in a fault description. Unquoted on purpose: a STRING parameter whose
/// value happens to be `<unset>` renders WITH quotes, so no node-supplied value can produce this
/// exact text and an operator reading it is never looking at something a node wrote.
inline constexpr const char * kUnsetParameterRendering = "<unset>";

/// The stand-in carried through an observed param map for a parameter the node declared and
/// never set.
///
/// A JSON OBJECT, and that is the whole of the safety argument. ROS parameter values are bool,
/// int, double, string and the 1-D array forms of those, so an object is a shape no parameter
/// value can take; and ParamDriftConfig::flatten_expect recurses THROUGH objects and only ever
/// stores the scalar or array it bottoms out on, so an operator's `expect` pin cannot hold one
/// either. Neither side of a comparison can therefore be mistaken for this. A string sentinel
/// would have neither property - a node is free to hold the string `<unset>` - and the pin an
/// operator wrote on that node would silently stop reporting.
inline nlohmann::json unset_param_value() {
  return nlohmann::json{{kUnsetParameterKey, true}};
}

/// True for the marker above, and for nothing a node or an operator can write.
inline bool is_unset_param(const nlohmann::json & v) {
  return v.is_object() && v.contains(kUnsetParameterKey);
}

/// The value one transport parameter entry (`{"name","value","type"}`) contributes to an observed
/// param map, with a declared-but-never-set parameter mapped to the marker instead of a bare null.
///
/// Two signals, because either alone leaves a way for `null` to reach a fault description. The
/// TYPE is the transport's explicit statement and the one a live node produces: rclcpp answers a
/// declared-but-unset name with a PARAMETER_NOT_SET value rather than an absent one, so the read
/// SUCCEEDS carrying null and `type` is `not_set`. The VALUE is the backstop: null is not a legal
/// encoding of any ROS parameter value, so a transport that omits the type still cannot put a bare
/// `null` in front of an operator. A NaN double is caught by NEITHER - it arrives as a float that
/// merely DUMPS as `null` - and that is exactly the distinction this exists to keep: after this,
/// `got=null` in a description means the value is NaN (or infinite) and nothing else.
inline nlohmann::json param_entry_value(const nlohmann::json & entry) {
  const auto type = entry.find("type");
  if (type != entry.end() && type->is_string() && type->get<std::string>() == kNotSetParameterType) {
    return unset_param_value();
  }
  const auto value = entry.find("value");
  if (value == entry.end() || value->is_null()) {
    return unset_param_value();
  }
  return *value;
}

/// True when an integer and a float denote the SAME number, without routing the integer through
/// a double. nlohmann's own cross-type compare does exactly that cast, so beyond 2^53 two
/// different values compare equal and the drift is missed. Reachable from an `expect` pin written
/// with a decimal point against an integer parameter, or a parameter whose type flips between
/// INTEGER and DOUBLE.
inline bool int_equals_float_exactly(const nlohmann::json & i, const nlohmann::json & f) {
  const double d = f.get<double>();
  if (!std::isfinite(d)) {
    return false;  // NaN or infinity is never equal to an integer
  }
  // Integrality without an equality test on doubles: -Werror=float-equal forbids == and != here,
  // and it is right to - the same flag caught a real sentinel bug in this package before.
  double whole = 0.0;
  const double frac = std::modf(d, &whole);
  if (frac > 0.0 || frac < 0.0) {
    return false;  // a fractional value can never equal an integer
  }
  // 2^63 is not representable as int64_t; compare against the double bound instead.
  if (d < -9223372036854775808.0 || d >= 9223372036854775808.0) {
    return false;
  }
  if (i.is_number_unsigned()) {
    // The sign has to be checked BEFORE the cast. Converting a negative double to uint64_t is
    // undefined behaviour, and it does not merely produce a harmless mismatch: on x86-64 -1.0
    // lands on UINT64_MAX, so a parameter holding that value compares EQUAL to a pin of -1.0 and
    // the drift is missed. Every positive integer arrives from the transport stored unsigned, so
    // this branch is the ordinary path, not a corner of it.
    return d >= 0.0 && i.get<std::uint64_t>() == static_cast<std::uint64_t>(d);
  }
  return i.get<std::int64_t>() == static_cast<std::int64_t>(d);
}

/// Equality for drift detection. Three carve-outs sit in front of nlohmann's `operator==`:
///
/// - A mixed integer/float pair goes through int_equals_float_exactly rather than `operator==`,
///   whose cross-type compare casts the integer to a double and so collapses int64s differing
///   beyond 2^53 into a MISSED drift.
/// - Two NaN doubles are equal, because IEEE `NaN != NaN` would otherwise make any NaN-valued
///   param (a common "unset/disabled" sentinel) a permanent false positive that never clears.
/// - Arrays recurse element-wise, so both carve-outs apply inside an array param as well (ROS
///   params are at most 1-D, so no deeper recursion is needed).
///
/// Everything else defers to `operator==`, and the unset marker (see unset_param_value) is
/// deliberately one of them rather than a fourth carve-out: as a JSON object it compares unequal to
/// every value a node or an `expect` pin can hold, so a pin against a parameter the node never set
/// still reports - and equal to itself, so a parameter that is unset on every sweep does not drift
/// against its own captured baseline for ever. Both directions are pinned in the unit tests.
inline bool param_values_equal(const nlohmann::json & a, const nlohmann::json & b) {
  if (a.is_number_float() && b.is_number_float() && std::isnan(a.get<double>()) && std::isnan(b.get<double>())) {
    return true;
  }
  if (a.is_number_integer() && b.is_number_float()) {
    return int_equals_float_exactly(a, b);
  }
  if (a.is_number_float() && b.is_number_integer()) {
    return int_equals_float_exactly(b, a);
  }
  if (a.is_array() && b.is_array()) {
    if (a.size() != b.size()) {
      return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
      if (!param_values_equal(a[i], b[i])) {
        return false;  // recurse -> NaN[] handled element-wise (ROS params <= 1-D)
      }
    }
    return true;
  }
  return a == b;
}

/// Minimum spacing between two parameter reads so that no more than `max_reads_per_tick`
/// of them happen per tick period. This is the whole enforcement of that budget: the reader
/// takes one target per pass and waits out the rest of its slot, so a gap of zero means no
/// budget at all.
///
/// Computed in microseconds on purpose. A budget larger than the tick period expressed in
/// milliseconds (2000 reads against a 1000 ms tick) truncates to zero under integer
/// millisecond division, which silently removes the bound instead of tightening it.
/// Microseconds are not enough on their own either: a budget above a million reads per second
/// truncates to zero again, and a zero gap is a reader that never waits. The floor keeps the
/// loop paced no matter what an operator asks for, and it does so SILENTLY: a budget inside the
/// accepted range is taken as written, and whether the floor bites depends on the tick period,
/// which from_json never sees. `max_reads_per_tick: 100000` (the ceiling) against
/// `tick_interval_ms: 1` therefore buys 1000 round trips per tick period, not 100000, with
/// nothing logged - the floor caps the effective budget BELOW the request, which is the safe
/// direction for a load bound, so this is a limit and not a defect.
///
/// Rounded UP, because the achievable rate is `1 / gap` and not `reads / period`: flooring the
/// division makes the gap shorter than the budget allows and the rate correspondingly higher.
/// `tick_interval_ms: 1` with `max_reads_per_tick: 600` - both accepted configurations - floors
/// to a 1 us gap and 1000 round trips per tick period against a documented bound of 600. A knob
/// an operator lowers to protect a node must never be overshot; rounding up can only spend the
/// budget more slowly than asked, which is the safe direction for a bound.
inline std::chrono::microseconds read_gap(int tick_interval_ms, int max_reads_per_tick) {
  const auto reads = static_cast<std::int64_t>(max_reads_per_tick > 0 ? max_reads_per_tick : 1);
  const auto period_us = static_cast<std::int64_t>(tick_interval_ms) * 1000;
  return std::chrono::microseconds(std::max<std::int64_t>(1, (period_us + reads - 1) / reads));
}

/// Parsed param-drift detector configuration. See the design doc for the model.
struct ParamDriftConfig {
  bool baseline_capture = true;                  ///< self-capture drift on unlisted params
  std::map<std::string, nlohmann::json> expect;  ///< param_name -> absolute expected value
  std::vector<std::string> ignore_globs;         ///< param-name globs never flagged (self-capture set only)
  int max_reads_per_tick = 8;                    ///< bound on blocking param reads per tick (coverage latency lever)
  /// Upper bound on the configurable budget. A request beyond this is not a tuning choice, it is
  /// a typo or a unit mix-up: even at the floor gap of one microsecond the reader cannot issue a
  /// million service round-trips per tick, so accepting it would only mean pretending.
  static constexpr std::int64_t kMaxReadsPerTickCeiling = 100000;
  std::vector<std::string> warnings;  ///< non-fatal config problems, surfaced by the detector's logger

  /// Parse the per-detector config the plugin delivers (config["detectors"]["param_drift"],
  /// see detector_config.hpp): keys `baseline` (bool), `expect` (object of param_name ->
  /// expected value, flattened from the gateway's nested delivery), `ignore` (native JSON
  /// string array). A present-but-wrong-typed `baseline`/`ignore`/`expect` is NOT silently
  /// dropped: it records a warning (a mistyped `ignore` would otherwise re-enable drift alerts
  /// on suppressed params with no diagnostic trail).
  static ParamDriftConfig from_json(const nlohmann::json & detector_cfg) {
    ParamDriftConfig cfg;
    if (detector_cfg.contains("baseline")) {
      if (detector_cfg["baseline"].is_boolean()) {
        cfg.baseline_capture = detector_cfg["baseline"].get<bool>();
      } else {
        cfg.warnings.push_back("'baseline' must be a boolean; ignoring it and keeping the default (true)");
      }
    }
    if (detector_cfg.contains("ignore")) {
      if (detector_cfg["ignore"].is_array()) {
        for (const auto & g : detector_cfg["ignore"]) {
          if (g.is_string()) {
            cfg.ignore_globs.push_back(g.get<std::string>());
          } else {
            cfg.warnings.push_back("'ignore' entries must be strings; skipping a non-string entry");
          }
        }
      } else {
        cfg.warnings.push_back("'ignore' must be a string array (e.g. [\"*_stamp\"]); ignoring it");
      }
    }
    if (detector_cfg.contains("max_reads_per_tick")) {
      // Range-check BEFORE narrowing. is_number_integer() is true across the whole 64-bit range
      // and get<int>() truncates, so 4294967297 would arrive as 1 and -4294967295 would arrive as
      // 1 too - a NEGATIVE request silently passing a positive-only guard, and a budget quietly
      // eight times smaller than the default. Both were accepted without a warning.
      const auto & value = detector_cfg["max_reads_per_tick"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : 0;
      if (value.is_number_integer() && wide > 0 && wide <= kMaxReadsPerTickCeiling) {
        cfg.max_reads_per_tick = static_cast<int>(wide);
      } else {
        cfg.warnings.push_back("'max_reads_per_tick' must be an integer in 1.." +
                               std::to_string(kMaxReadsPerTickCeiling) + "; keeping the default (8)");
      }
    }
    if (detector_cfg.contains("expect")) {
      if (detector_cfg["expect"].is_object()) {
        // The gateway un-nests dotted config keys, so a pin on a dotted param name
        // (e.g. expect.qos_overrides./scan.reliability) arrives as a nested object.
        // Flatten it back to the full dotted param name so it matches the observed key.
        flatten_expect(detector_cfg["expect"], "", cfg.expect, &cfg.warnings);
      } else {
        cfg.warnings.push_back("'expect' must be an object of param_name -> expected value; ignoring it");
      }
    }
    // A wrong TYPE on the four keys above already warns; a wrong NAME used to not, even
    // though it fails the same way. `ignore_globs` (the struct's own field name) instead
    // of `ignore` leaves the globs empty and GRAPH_PARAM_DRIFT firing on exactly the
    // params the operator meant to suppress.
    collect_unknown_detector_keys(detector_cfg, known_keys(), cfg.warnings);
    // Configurations that parse cleanly and can never do anything. Silence here is the worst
    // outcome: the detector looks configured and reports nothing, forever.
    if (!cfg.baseline_capture && cfg.expect.empty()) {
      cfg.warnings.push_back(
          "'baseline' is false and 'expect' is empty, so this detector will never read a "
          "parameter or report anything; set 'expect' or leave 'baseline' enabled");
    }
    if (!cfg.baseline_capture && !cfg.ignore_globs.empty()) {
      cfg.warnings.push_back(
          "'ignore' only applies to self-captured parameters, and 'baseline' is false, so it has "
          "no effect here");
    }
    // A glob made of nothing but '*' matches every parameter name, so self-capture is suppressed
    // wholesale. At runtime that is indistinguishable from a detector that is working and finding
    // nothing - the silent failure this detector exists to rule out, reproduced in its own config.
    const bool catch_all_ignore =
        std::any_of(cfg.ignore_globs.begin(), cfg.ignore_globs.end(), [](const std::string & g) {
          return !g.empty() && g.find_first_not_of('*') == std::string::npos;
        });
    if (cfg.baseline_capture && catch_all_ignore) {
      if (cfg.expect.empty()) {
        cfg.warnings.push_back(
            "'ignore' matches every parameter name and 'expect' is empty, so this detector will "
            "never report anything; narrow the pattern");
      } else {
        cfg.warnings.push_back(
            "'ignore' matches every parameter name, so self-captured drift is fully suppressed "
            "and only the 'expect' pins can report");
      }
    }
    return cfg;
  }

  /// Config keys this detector reads. Anything else under detectors.param_drift is a typo.
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"baseline", "ignore", "max_reads_per_tick", "expect"};
    return keys;
  }

  /// Flatten a (possibly nested) `expect` object back to dotted param-name keys. The gateway
  /// splits dotted config keys into nested objects; ROS param values are scalars/arrays (never
  /// objects), so the recursion stops at the expected value and the joined key is the full
  /// param name (e.g. {"qos_overrides":{"/scan":{"reliability":v}}} -> "qos_overrides./scan.reliability").
  static void flatten_expect(const nlohmann::json & obj, const std::string & prefix,
                             std::map<std::string, nlohmann::json> & out,
                             std::vector<std::string> * warnings = nullptr) {
    for (const auto & item : obj.items()) {
      const std::string key = prefix.empty() ? item.key() : prefix + "." + item.key();
      if (item.value().is_object()) {
        if (item.value().empty() && warnings != nullptr) {
          // An empty object carries no expected value, so the pin the operator wrote silently
          // becomes nothing at all. Say so rather than dropping it.
          warnings->push_back("'expect." + key + "' has no value; ignoring that pin");
        }
        flatten_expect(item.value(), key, out, warnings);
      } else if (!out.emplace(key, item.value()).second && warnings != nullptr) {
        // Two config paths flattened to the same parameter name (e.g. `a: {b: 1}` next to
        // `a.b: 2`). emplace keeps the first, so the other pin is silently lost.
        warnings->push_back("'expect." + key + "' is pinned twice; keeping the first value");
      }
    }
  }
};

/// Pure, ROS-free drift core. Holds per-app captured baselines and computes this app's
/// drifted param descriptors from an observed param map. One instance per detector.
/// Aggregation (raise/clear across apps) moves to the detector.
class ParamDriftPolicy {
 public:
  explicit ParamDriftPolicy(ParamDriftConfig cfg) : cfg_(std::move(cfg)) {
  }

  const ParamDriftConfig & config() const {
    return cfg_;
  }

  /// Per-value budget inside the description. Small enough that several drifted entries fit under
  /// AggregatedFault's cap, large enough to still show a recognisable value. Public so a test can
  /// pin the budget rather than a loose descriptor length that would pass at twice the size.
  static constexpr std::size_t kMaxRenderedValueChars = 64;

  /// Returns this app's drifted descriptors (empty = clean). Captures the app's baseline
  /// the first time it is called with capture_allowed=true (so a not-yet-armed node is
  /// never baselined pre-settle). expect rules are absolute and fire regardless.
  /// `pinned_out`, when given, receives how many of the returned descriptors came from an
  /// `expect` pin. Those are always the leading entries, so a caller can tell an
  /// operator-declared violation from self-captured drift without parsing the text back. It
  /// must not parse: a parameter's own value can contain any substring, including the markers
  /// this function writes, so text sniffing would misclassify.
  std::vector<std::string> evaluate(const std::string & app_id, const std::map<std::string, nlohmann::json> & observed,
                                    bool capture_allowed, std::size_t * pinned_out = nullptr) {
    std::vector<std::string> drifted;
    for (const auto & [name, expected] : cfg_.expect) {
      const auto it = observed.find(name);
      if (it != observed.end() && !param_values_equal(it->second, expected)) {
        drifted.push_back(app_id + ":" + safe_name(name) + " expected=" + compact(expected) +
                          " got=" + compact(it->second));
      }
    }
    if (pinned_out != nullptr) {
      *pinned_out = drifted.size();
    }
    if (cfg_.baseline_capture && capture_allowed) {
      auto & base = baselines_[app_id];  // creates empty on first sight
      for (const auto & [name, value] : observed) {
        if (cfg_.expect.count(name) > 0 || is_ignored(name)) {
          continue;
        }
        const auto bit = base.find(name);
        if (bit == base.end()) {
          base.emplace(name, value);  // new param -> capture silently (not a drift)
          continue;
        }
        // No `first` check here: when this app had no baseline at all, every name takes the
        // capture branch above, so reaching this line already means the baseline pre-existed.
        if (!param_values_equal(value, bit->second)) {
          drifted.push_back(app_id + ":" + safe_name(name) + " baseline=" + compact(bit->second) +
                            " got=" + compact(value));
        }
      }
    }
    return drifted;
  }

  /// App vanished from the graph: drop its baseline so a later reappearance re-captures
  /// instead of drifting against a stale baseline.
  void forget(const std::string & app_id) {
    baselines_.erase(app_id);
  }

 private:
  bool is_ignored(const std::string & name) const {
    for (const auto & g : cfg_.ignore_globs) {
      if (glob_match(g, name)) {
        return true;
      }
    }
    return false;
  }

  /// Render a parameter value for a fault description. Node-supplied values are arbitrary, and
  /// this is the first detector to put them into fault text at all, so two things must hold.
  ///
  /// ASCII only. The aggregated description is capped by cutting at a BYTE offset, and a cut
  /// through a multi-byte UTF-8 sequence produces a description the gateway's own DTO writer
  /// cannot serialize - GET /faults then answers 500 instead of listing the fault. `ensure_ascii`
  /// escapes non-ASCII, so any byte offset is a safe cut point.
  ///
  /// Never throws. A value whose bytes are not valid UTF-8 makes a plain dump() throw, and that
  /// throw escapes into the plugin's per-detector guard, so the detector would neither raise nor
  /// clear for as long as the value is present - a CONFIRMED fault that can never heal. The
  /// replace handler substitutes U+FFFD instead.
  ///
  /// Long values are elided: one array-valued parameter can otherwise consume the whole
  /// description budget and hide every other drifted entry.
  ///
  /// A parameter the node DECLARED and never SET is rendered from the marker rather than dumped.
  /// Dumping it would print `null`, and a NaN double prints `null` too - so one description said
  /// the same thing about "the node never set this parameter" and "the value is NaN", which are
  /// different faults with different remedies. See unset_param_value().
  static std::string compact(const nlohmann::json & v) {
    if (is_unset_param(v)) {
      return kUnsetParameterRendering;
    }
    return elide(v.dump(-1, ' ', /*ensure_ascii=*/true, nlohmann::json::error_handler_t::replace));
  }

  /// Same treatment for a parameter NAME. Names come verbatim from the watched node's
  /// ListParameters reply and are never validated, so they carry exactly the two hazards values do:
  /// a name whose bytes are not valid UTF-8, or a valid multi-byte name split by the aggregated
  /// description's byte-offset cut, makes the gateway's JSON writer throw. That answers 500 on
  /// GET /faults and takes every OTHER fault down with it, since they share the response.
  static std::string safe_name(const std::string & name) {
    return elide(nlohmann::json(name).dump(-1, ' ', /*ensure_ascii=*/true, nlohmann::json::error_handler_t::replace));
  }

  /// Shorten to the budget while keeping BOTH ends. Head-only truncation renders two different
  /// values identically whenever they share a long prefix - the common case for robot_description
  /// and for nav2 footprint arrays, where only a late element differs - so the fault would fire
  /// correctly and then prove nothing about what changed.
  static std::string elide(std::string text) {
    if (text.size() <= kMaxRenderedValueChars) {
      return text;
    }
    const std::size_t keep = kMaxRenderedValueChars - 3;
    const std::size_t head = (keep + 1) / 2;
    return text.substr(0, head) + "..." + text.substr(text.size() - (keep - head));
  }

  ParamDriftConfig cfg_;
  std::map<std::string, std::map<std::string, nlohmann::json>> baselines_;  // app_id -> (param -> value)
};

}  // namespace ros2_medkit_graph_watchdog
