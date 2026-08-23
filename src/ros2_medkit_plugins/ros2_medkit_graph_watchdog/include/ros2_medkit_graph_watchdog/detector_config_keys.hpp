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
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// Deliberately ROS-free (no detector.hpp, no rclcpp): the pure policy headers that need
// this - param_drift_policy.hpp and friends - are unit-tested without a ROS build.
namespace ros2_medkit_graph_watchdog {

/// Keys the PLUGIN injects into every detector's config before calling configure()
/// (GraphWatchdogPlugin::set_context). They are plumbing, not a detector's own keys, so a
/// detector that ignores one must never report it as an operator typo.
inline const std::set<std::string> & plugin_injected_detector_keys() {
  static const std::set<std::string> keys{"prune_grace", "tick_interval_ms"};
  return keys;
}

/// Mirrors GraphWatchdogPlugin's own tick_interval_ms_ default, for the detectors that need
/// the tick period before the plugin has injected it (bare-context unit tests).
inline constexpr int kDefaultTickIntervalMs = 1000;

/// Floor for a clock-jump threshold, expressed in tick periods.
///
/// Every clock-jump check compares the delta between consecutive evaluate() calls, and that
/// delta is the whole tick period (sweep duration + tick_interval_ms), not a fixed quantity.
/// A threshold below the tick period therefore makes EVERY ordinary tick look like a
/// discontinuity: streaks are zeroed, entries re-baseline, and the detector can never raise -
/// silently, with nothing in the log. A tick has to overrun its period by this factor before
/// it counts as a real clock jump rather than a slow sweep.
inline constexpr double kJumpThreshTickPeriods = 3.0;

/// Smallest jump threshold that is safe at a given tick period.
inline constexpr double min_jump_thresh_sec(int tick_interval_ms) {
  return kJumpThreshTickPeriods * static_cast<double>(tick_interval_ms) / 1000.0;
}

/// Floor for node_death's own `miss_grace`, in milliseconds of wall clock.
///
/// The entity cache a detector reads (ctx.snapshot) is not rebuilt every tick: it is
/// rebuilt on a graph event debounced to about one refresh per second, so every tick
/// between two refreshes sees the SAME snapshot. A tick-counted miss_grace therefore does
/// not measure independent samples of the graph - it can re-count one stale cache
/// generation as several misses - and that collapses silently the moment the tick period
/// is shortened: at a 200ms tick, a miss_grace of 2 ticks is only 600ms of nominal grace,
/// well under a single refresh cycle, so one omitted refresh alone can already cross it.
/// 3000ms is one debounce cycle plus margin, chosen so the shipped default (miss_grace 2 at
/// the 1000ms default tick) is unaffected and only a faster-than-default tick ever raises
/// the effective grace.
inline constexpr int kMinNodeDeathWindowMs = 3000;

/// Ceiling node_death applies to BOTH `miss_grace` and `prune_grace` - ticks-before-
/// something-happens knobs that, at the 1s default tick, already mean an hour of silence
/// before either takes effect at 3600; a value past it is a typo or a unit mix-up, not a
/// real operator choice. Shared (not detector-local) so graph_watchdog_plugin.cpp's own
/// compute_departed_retention_ticks() - which has to predict node_death's eventual
/// prune_ticks_ before that detector's own configure() has run - clamps to the IDENTICAL
/// bound rather than risking a plugin-side window sized for a value the detector will
/// itself reject.
inline constexpr std::int64_t kMaxNodeDeathGraceTicks = 3600;

/// Smallest `miss_grace` (in ticks) that keeps kMinNodeDeathWindowMs of wall clock,
/// whatever `tick_interval_ms` is configured to. A death is reported once misses EXCEED
/// miss_grace, i.e. after miss_grace + 1 ticks, so the ceiling divides the window by the
/// tick period and subtracts the one tick that boundary already buys back.
///
/// Computed in int64_t throughout: `tick_interval_ms` is validated only against
/// `> 0 && <= INT_MAX` (node_death_detector.cpp's own configure()), and at that documented-
/// valid endpoint `kMinNodeDeathWindowMs + tick_interval_ms - 1` overflows a 32-bit int
/// before the division ever runs - undefined behaviour at a value the config contract
/// explicitly accepts. The final result is always small (it shrinks as tick_interval_ms
/// grows), so narrowing it back to int at the end never loses anything.
inline int min_node_death_miss_grace(int tick_interval_ms) {
  if (tick_interval_ms <= 0) {
    return 0;  // not a real cadence; nothing meaningful to floor against
  }
  const std::int64_t wide_tick = tick_interval_ms;
  const std::int64_t window = static_cast<std::int64_t>(kMinNodeDeathWindowMs) + wide_tick - 1;
  return static_cast<int>(std::max<std::int64_t>(0, window / wide_tick - 1));
}

/// Append one warning per key in `detector_cfg` that the detector does not read.
///
/// extract_detector_config() copies EVERY key under detectors.<id> verbatim, and a
/// detector's own reader then looks up only the handful of names it knows. A key matching
/// none of them is dropped with nothing in the log. That fails silently in the direction
/// that costs the operator most: write `ignore_globs:` where the detector reads `ignore:`
/// and the suppression never takes effect, so the faults you meant to silence keep coming
/// and nothing connects them to the typo. Detectors pass their own known-key set here.
inline void collect_unknown_detector_keys(const nlohmann::json & detector_cfg, const std::set<std::string> & known_keys,
                                          std::vector<std::string> & warnings) {
  if (!detector_cfg.is_object()) {
    return;
  }
  std::string known_list;
  for (const auto & key : known_keys) {
    if (!known_list.empty()) {
      known_list += ", ";
    }
    known_list += key;
  }
  for (const auto & item : detector_cfg.items()) {
    if (known_keys.count(item.key()) != 0 || plugin_injected_detector_keys().count(item.key()) != 0) {
      continue;
    }
    warnings.push_back("unknown config key '" + item.key() + "'; it has no effect (known keys: " + known_list + ")");
  }
}

}  // namespace ros2_medkit_graph_watchdog
