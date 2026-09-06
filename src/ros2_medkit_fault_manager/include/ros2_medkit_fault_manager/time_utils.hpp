// Copyright 2026 mfaferek93
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
#include <cstdint>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace ros2_medkit_fault_manager {

/// Get current wall clock time in nanoseconds.
/// This is not affected by use_sim_time parameter.
inline int64_t get_wall_clock_ns() {
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

/// Get current wall clock time as rclcpp::Time.
/// This is not affected by use_sim_time parameter.
inline rclcpp::Time get_wall_clock_time() {
  return rclcpp::Time(get_wall_clock_ns(), RCL_SYSTEM_TIME);
}

/// builtin_interfaces/Time -> nanoseconds since the Unix epoch.
inline int64_t time_msg_to_ns(const builtin_interfaces::msg::Time & t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

/// Nanoseconds since the Unix epoch -> builtin_interfaces/Time.
///
/// Floor division, not C's truncation towards zero. `nanosec` is UNSIGNED, so
/// for an instant before the epoch the remainder is negative and casting it
/// yields nanosec near 4e9 - a message that is not a time at all and that no
/// reader can interpret. -1 ns is second -1 plus 999999999 ns, and that is what
/// this returns.
///
/// The seconds are clamped to what the int32 field can carry. A caller must not
/// hand this an instant outside that range - the API refuses those long before
/// here - but saturating is the one behaviour that cannot turn an absurd input
/// into a plausible-looking output.
inline builtin_interfaces::msg::Time ns_to_time_msg(int64_t ns) {
  constexpr int64_t kNsPerSec = 1000000000LL;
  int64_t sec = ns / kNsPerSec;
  int64_t nsec = ns % kNsPerSec;
  if (nsec < 0) {
    nsec += kNsPerSec;
    --sec;
  }

  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(
      std::clamp<int64_t>(sec, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
  t.nanosec = static_cast<uint32_t>(nsec);
  return t;
}

}  // namespace ros2_medkit_fault_manager
