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
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace faults {

/// The resolution the planned-stop API works at, end to end: the parser rounds
/// down to it, storage keeps that value, responses return it unchanged (so a GET
/// equals the POST that made it), the covering test floors the window's bounds
/// to it, and a fault's own instant is rounded to the NEAREST one.
///
/// One millisecond rather than one nanosecond because an operator declares a
/// stop at minute or second precision, and the REST fault item carries
/// `first_occurred` as a double whose resolution at today's epoch is about a
/// quarter of a microsecond. A boundary defined more finely than the data can
/// express is a boundary nobody can rely on.
constexpr int64_t kNsPerMillisecond = 1000000;

/// Round @p ns DOWN to a whole millisecond. A true floor, not a truncation: it
/// must not round towards zero for a negative instant, or the boundary would sit
/// on the wrong side of it.
inline int64_t floor_to_ms_ns(int64_t ns) {
  const int64_t remainder = ns % kNsPerMillisecond;
  return remainder < 0 ? ns - remainder - kNsPerMillisecond : ns - remainder;
}

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

  /// True when an operator cut the window short, which moved to_ns to the moment
  /// of that request.
  bool ended_early{false};

  /// True on the window returned by a cancellation - one stopped before it ever
  /// started, and therefore removed rather than shortened. Never true on a
  /// window a list or a read returns.
  bool cancelled{false};

  /// Whether @p when_ns falls inside the window, at MILLISECOND resolution.
  ///
  /// Both bounds are floored to their millisecond and the interval is closed on
  /// those, so the covered set is `[floor_ms(from), floor_ms(to)]` counted in
  /// milliseconds. "Closed at both ends" is exact only at that resolution: a
  /// window declared with sub-millisecond bounds through the ROS service opens
  /// at the START of `from`'s millisecond, up to 999999 ns before the instant
  /// asked for.
  ///
  /// The fault's own instant is rounded to the NEAREST millisecond before it
  /// gets here (seconds_to_ns), so an instant within half a millisecond of a
  /// bound lands on that bound.
  bool covers(int64_t when_ns) const {
    const int64_t when_ms = floor_to_ms_ns(when_ns);
    return when_ms >= floor_to_ms_ns(from_ns) && when_ms <= floor_to_ms_ns(to_ns);
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
/// nanoseconds since the Unix epoch, ROUNDED TO THE NEAREST MILLISECOND.
///
/// Nearest, not floored: the double came from an exact nanosecond instant and
/// carries an error far below a millisecond, so rounding recovers the instant
/// that was meant. Flooring the nanoseconds first does not - at second
/// 1788724271, 375 of the 1000 exact-millisecond instants land just under their
/// own millisecond as a double and would floor to the previous one, which makes
/// the closed boundary this API promises false for a third of the instants a
/// fault can carry.
///
/// Nothing when @p seconds is not a usable instant: not finite, or so large that
/// the conversion would be undefined rather than merely wrong.
std::optional<int64_t> seconds_to_ns(double seconds);

/// The range of instants a window can actually be stored at.
///
/// The upper bound is `builtin_interfaces/Time`, whose seconds are an int32, so
/// the range ends in January 2038. An instant past it must be REFUSED rather
/// than converted: the narrowing wraps, and a window asked for in 2099 comes
/// back with a negative end, covering nothing, while the operator is told it was
/// accepted.
///
/// The lower bound is one millisecond after the Unix epoch, not the epoch
/// itself. Zero is what an omitted or unset `builtin_interfaces/Time` reads as,
/// and a sentinel that is also a legal value is two bugs waiting: a window
/// declared to start at the epoch would be indistinguishable from one whose
/// start nobody set.
constexpr int64_t kMinRepresentableNs = kNsPerMillisecond;
constexpr int64_t kMaxRepresentableNs = static_cast<int64_t>(2147483647) * 1000000000LL + 999999999LL;

/// Whether @p ns can be carried by a builtin_interfaces/Time. Written as a
/// positive range test on purpose.
inline bool is_representable_instant(int64_t ns) {
  return ns >= kMinRepresentableNs && ns <= kMaxRepresentableNs;
}

/// What this gateway knows about the declared windows at one instant.
///
/// `known` false is "this gateway has not been able to read them", which is NOT
/// "no window is declared". Every surface that reports the flag has to tell the
/// two apart: a consumer must not read an unreachable fault manager as "nothing
/// is expected", and that is as true of `GET /faults` as it is of the stream.
struct PlannedStopKnowledge {
  bool known{false};
  std::vector<PlannedStopWindow> windows;
};

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
/// on faults nobody planned for. "Usable" includes being in range - a plugin's
/// fault list is JSON this gateway did not write, and rounding an absurd double
/// into a nanosecond count is undefined behaviour, not a big number.
const PlannedStopWindow * window_for_fault(const std::vector<PlannedStopWindow> & windows,
                                           const nlohmann::json & fault_json);

/// How many DISTINCT fault codes in @p items say they were expected.
///
/// Distinct, because an aggregating gateway serves a code raised on two
/// gateways as two items and counting both would say two stops' worth of
/// expected faults where the plant had one. An item carrying no `expected` key
/// is counted by neither side: its flag is unknown, and unknown is not false.
int64_t count_expected_items(const nlohmann::json & items);

/// Write `expected_count` into a fault collection's `x-medkit` object, creating
/// that object when the emitter did not send one.
///
/// A plugin's fault list need not carry a vendor extension at all, and writing
/// through `collection["x-medkit"]["expected_count"]` while only guarding on
/// `is_object()` left the collection answering `"x-medkit": null` - neither the
/// count nor the absence of one. A non-object collection is left alone.
void set_expected_count(nlohmann::json & collection, int64_t count);

/// Attach `expected` - and `planned_stop_id` when it is true - to the fault's own
/// `x-medkit` object, preserving anything already there.
///
/// Returns whether the fault was expected, so a caller can keep a count without
/// deriving twice, or nothing when @p knowledge says the windows are unknown. In
/// that case the item is left EXACTLY as it was: no key is written, because the
/// honest answer is silence rather than `false`.
std::optional<bool> annotate_fault_with_planned_stop(nlohmann::json & fault_json,
                                                     const PlannedStopKnowledge & knowledge);

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

  /// Whether a window set was read successfully within @p max_age.
  ///
  /// False is "this gateway does not know", which is NOT the same as "no windows
  /// are declared" - an empty successful read is knowledge, an outage is not.
  /// Every surface that reports the flag omits it entirely in the first case.
  ///
  /// Knowledge EXPIRES. Without that, a gateway that read a window set once
  /// served it for the life of the process: replace the fault manager and
  /// `GET /x-medkit-planned-stops` answers 503 while `GET /faults` keeps naming
  /// a window that exists nowhere, indefinitely and with no way to tell. The
  /// bound is a small multiple of the refresh period, so a set that is merely
  /// being re-read never blinks, and one nothing can refresh goes quiet.
  bool has_knowledge(std::chrono::steady_clock::duration max_age) const;

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
  bool ever_stored_{false};
  bool last_refresh_failed_{false};
  std::chrono::steady_clock::time_point attempted_at_{};
  std::chrono::steady_clock::time_point stored_at_{};
};

}  // namespace faults
}  // namespace ros2_medkit_gateway
