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

#include <chrono>
#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace faults {

/// A window of wall-clock time during which faults are expected, as the gateway
/// holds it.
///
/// The fault manager owns the windows; the gateway only derives from them. This
/// is the gateway-side shape - nanoseconds since the Unix epoch rather than
/// builtin_interfaces/Time - so the neutral layer never sees a ROS type.
struct PlannedStopWindow {
  std::string id;
  int64_t from_ns{0};
  int64_t to_ns{0};
  std::string reason;
  std::string declared_by;
  int64_t declared_at_ns{0};
  bool ended_early{false};

  /// Whether @p when_ns falls inside the window. CLOSED at both ends, matching
  /// PlannedStopWindow::covers in the fault manager: a fault reported at the
  /// instant a window opens or closes must read the same on both sides.
  bool covers(int64_t when_ns) const {
    return when_ns >= from_ns && when_ns <= to_ns;
  }
};

/// The window that makes a fault at @p when_ns expected, or nullptr when none
/// does.
///
/// When several windows cover the instant, the EARLIEST-STARTING one wins, and a
/// tie on the start is broken by id. Both rules exist so that two readers of the
/// same data - the fault list and the event stream, or two gateways in a fleet -
/// name the same window; "whichever came first in the list" would not, because
/// the fault manager is free to order its answer as it likes.
///
/// The returned pointer aliases @p windows and is valid only while it lives.
const PlannedStopWindow * find_covering_window(const std::vector<PlannedStopWindow> & windows, int64_t when_ns);

/// Parse an ISO 8601 instant in UTC into nanoseconds since the Unix epoch, or
/// nothing when the text is not one.
///
/// Deliberately strict. `Z`, `+00:00` and `-00:00` are the only accepted zones:
/// a string without a zone means a different instant to every reader, and a real
/// offset is a shape this API does not take, so both are refused rather than
/// guessed at. Fractional seconds are kept to nanosecond resolution and any
/// further digits are truncated - the instant is still unambiguous.
std::optional<int64_t> parse_iso8601_utc_ns(const std::string & text);

/// Seconds-with-a-fraction (how a fault carries first_occurred on the wire) into
/// nanoseconds since the Unix epoch.
int64_t seconds_to_ns(double seconds);

/// The range of instants a window can actually be stored at.
///
/// `builtin_interfaces/Time` carries its seconds in an int32, so the
/// representable range runs from the Unix epoch to January 2038. An instant
/// outside it must be REFUSED rather than converted: the narrowing wraps, and a
/// window asked for in 2099 comes back with a negative end, covering nothing,
/// while the operator is told it was accepted.
constexpr int64_t kMinRepresentableNs = 0;
constexpr int64_t kMaxRepresentableNs = static_cast<int64_t>(2147483647) * 1000000000LL + 999999999LL;

/// Whether @p ns can be carried by a builtin_interfaces/Time. Written as a
/// positive range test on purpose.
inline bool is_representable_instant(int64_t ns) {
  return ns >= kMinRepresentableNs && ns <= kMaxRepresentableNs;
}

/// Which window covers the fault carried in @p fault_json, or nullptr.
///
/// Reads `first_occurred` - seconds since the Unix epoch, the shape
/// `fault_to_json` emits - because that is the start of the fault's CURRENT
/// cycle. A code that failed inside a window, was cleared, and failed again
/// afterwards is therefore expected for the first cycle and not for the second,
/// which is what an operator asking "did anything break outside the stop" means.
///
/// A fault carrying no usable `first_occurred` is never expected: the gateway
/// cannot place a cycle it cannot date, and marking it anyway would put the flag
/// on faults nobody planned for.
const PlannedStopWindow * window_for_fault(const std::vector<PlannedStopWindow> & windows,
                                           const nlohmann::json & fault_json);

/// Attach `expected` - and `planned_stop_id` when it is true - to the fault's own
/// `x-medkit` object, preserving anything already there. Returns whether the
/// fault was expected, so a caller can keep a count without deriving twice.
bool annotate_fault_with_planned_stop(nlohmann::json & fault_json, const std::vector<PlannedStopWindow> & windows);

/// The windows this gateway last read from the fault manager.
///
/// A window can be declared straight through the ROS service, so the event path
/// cannot rely on having seen every declaration itself; it re-reads on a time to
/// live instead. Writes through this gateway invalidate the cache so an operator
/// never watches their own declaration take effect late.
///
/// Thread-safe. `snapshot()` returns a copy on purpose: callers hold the result
/// while formatting an event, and handing out a reference into the cache would
/// tie that formatting to the refresh.
class PlannedStopCache {
 public:
  PlannedStopCache() = default;
  ~PlannedStopCache() = default;
  PlannedStopCache(const PlannedStopCache &) = delete;
  PlannedStopCache & operator=(const PlannedStopCache &) = delete;
  PlannedStopCache(PlannedStopCache &&) = delete;
  PlannedStopCache & operator=(PlannedStopCache &&) = delete;

  /// Replace what is known and mark it fresh. An empty list is an answer, not a
  /// miss: every window having ended is a state the stream must reflect.
  void store(std::vector<PlannedStopWindow> windows);

  /// A copy of what is known.
  std::vector<PlannedStopWindow> snapshot() const;

  /// Record that a refresh was attempted and did not produce windows, without
  /// touching what is known. This is what stops an unreachable fault manager
  /// from costing the transport's timeout on EVERY read: the attempt counts
  /// against the time to live exactly as a successful one does.
  void note_failed_refresh();

  /// Whether the most recent refresh attempt failed. A caller uses it to back
  /// off further than the normal time to live, because a fault manager that did
  /// not answer once will usually not answer the next event either, and the
  /// event stream must not stall waiting to find out.
  bool last_refresh_failed() const;

  /// Whether the last refresh attempt - successful or not - is older than
  /// @p ttl. True before the first one.
  bool is_stale(std::chrono::steady_clock::duration ttl) const;

  /// Ask for a refresh on the next read without discarding what is known - a
  /// gateway that just wrote a window should show it, but must not blank the
  /// flag for every other fault while the re-read is in flight.
  void invalidate();

 private:
  mutable std::mutex mutex_;
  std::vector<PlannedStopWindow> windows_;
  bool ever_attempted_{false};
  bool last_refresh_failed_{false};
  std::chrono::steady_clock::time_point attempted_at_{};
};

}  // namespace faults
}  // namespace ros2_medkit_gateway
