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
#include <cstdint>
#include <ctime>

namespace ros2_medkit_gateway {

// Resolve a ROS-parameter thread-count into a usable pool size (issue #440).
//
// Both the rclcpp executor and the cpp-httplib request pool are sized from an
// int64 ROS parameter that an operator may mis-set. This clamps the value to a
// closed [min_threads, max_threads] range so a typo can neither drop the pool to
// zero (which would queue requests forever / mean "all cores") nor spawn a
// pathological thread count. The range is two-sided to match the established
// clamp pattern used by the subscription_executor.* / data_provider.* knobs.
//
// This helper is deliberately silent: it is ROS-neutral (gateway_core links no
// rclcpp), so it cannot log. Reporting the coercion is the CALLER's job and is
// not optional - a clamp nobody hears leaves the config file and the running
// process disagreeing with nothing to show it. Compare the result with the
// requested value and RCLCPP_WARN when they differ, as main.cpp,
// http/rest_server.cpp and gateway_node.cpp do. The one exception is a re-read
// of a parameter another site has already reported (main.cpp compares the HTTP
// pool against the SSE + cold-wait budget), where a second warning would only
// teach operators to skim past the first.
//
// Pre-condition: 1 <= min_threads <= max_threads.
inline std::size_t clamp_thread_count(int64_t requested, int64_t min_threads, int64_t max_threads) {
  return static_cast<std::size_t>(std::clamp<int64_t>(requested, min_threads, max_threads));
}

// Resolve a ROS-parameter keep-alive timeout (seconds) into a usable value
// (issue #440). cpp-httplib pins one request-pool worker on an idle keep-alive
// connection for this long before freeing it. With a small bounded pool a long
// timeout lets a burst of short-lived client connections (e.g. a test polling
// /apps + /areas + /functions every cycle) hold every worker, so ordinary
// requests stall up to one timeout per cycle. Too small loses connection reuse
// for legitimate clients (extra TCP/TLS handshakes). This clamps a mis-set value
// to a closed [min_sec, max_sec] range so request serving can never break.
//
// Silent for the same reason as clamp_thread_count above, and with the same
// obligation on the caller: report the coercion when it fires.
//
// Pre-condition: 1 <= min_sec <= max_sec.
inline std::time_t clamp_keep_alive_timeout(int64_t requested, int64_t min_sec, int64_t max_sec) {
  return static_cast<std::time_t>(std::clamp<int64_t>(requested, min_sec, max_sec));
}

}  // namespace ros2_medkit_gateway
