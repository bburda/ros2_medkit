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

#include "ros2_medkit_gateway/core/faults/planned_stop.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <utility>

namespace ros2_medkit_gateway {
namespace faults {

namespace {

constexpr int64_t kNsPerSecond = 1'000'000'000;

/// Read exactly @p digits decimal digits starting at @p pos, advancing it.
/// Returns nothing when any of them is not a digit, which is what keeps
/// "2026-9-6T..." and "2026-09-06T1:2:3Z" out.
std::optional<int> read_fixed_digits(const std::string & text, size_t & pos, size_t digits) {
  if (pos + digits > text.size()) {
    return std::nullopt;
  }
  int value = 0;
  for (size_t i = 0; i < digits; ++i) {
    const char c = text[pos + i];
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    value = value * 10 + (c - '0');
  }
  pos += digits;
  return value;
}

bool expect_char(const std::string & text, size_t & pos, char expected) {
  if (pos >= text.size() || text[pos] != expected) {
    return false;
  }
  ++pos;
  return true;
}

}  // namespace

const PlannedStopWindow * find_covering_window(const std::vector<PlannedStopWindow> & windows, int64_t when_ns) {
  const PlannedStopWindow * best = nullptr;
  for (const auto & candidate : windows) {
    if (!candidate.covers(when_ns)) {
      continue;
    }
    if (best == nullptr || candidate.from_ns < best->from_ns ||
        (candidate.from_ns == best->from_ns && candidate.id < best->id)) {
      best = &candidate;
    }
  }
  return best;
}

std::optional<int64_t> parse_iso8601_utc_ns(const std::string & text) {
  size_t pos = 0;

  const auto year = read_fixed_digits(text, pos, 4);
  if (!year || !expect_char(text, pos, '-')) {
    return std::nullopt;
  }
  const auto month = read_fixed_digits(text, pos, 2);
  if (!month || !expect_char(text, pos, '-')) {
    return std::nullopt;
  }
  const auto day = read_fixed_digits(text, pos, 2);
  if (!day || !expect_char(text, pos, 'T')) {
    return std::nullopt;
  }
  const auto hour = read_fixed_digits(text, pos, 2);
  if (!hour || !expect_char(text, pos, ':')) {
    return std::nullopt;
  }
  const auto minute = read_fixed_digits(text, pos, 2);
  if (!minute || !expect_char(text, pos, ':')) {
    return std::nullopt;
  }
  const auto second = read_fixed_digits(text, pos, 2);
  if (!second) {
    return std::nullopt;
  }

  // Range check before timegm, which normalises out-of-range fields instead of
  // refusing them: month 13 would quietly become January of the next year.
  // Second 60 is refused with the rest - a leap second has no representation in
  // a Unix timestamp, so accepting it would move the instant by one second.
  if (*month < 1 || *month > 12 || *day < 1 || *day > 31 || *hour > 23 || *minute > 59 || *second > 59) {
    return std::nullopt;
  }

  int64_t fraction_ns = 0;
  if (pos < text.size() && text[pos] == '.') {
    ++pos;
    const size_t fraction_start = pos;
    int64_t scale = kNsPerSecond;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
      if (scale > 1) {
        scale /= 10;
        fraction_ns += static_cast<int64_t>(text[pos] - '0') * scale;
      }
      ++pos;
    }
    if (pos == fraction_start) {
      return std::nullopt;  // a dot with no digits behind it
    }
  }

  // The zone. Only UTC, spelled any of the three legal ways.
  if (pos < text.size() && (text[pos] == 'Z' || text[pos] == 'z')) {
    ++pos;
  } else if (pos + 6 <= text.size() && (text[pos] == '+' || text[pos] == '-')) {
    const std::string offset = text.substr(pos + 1, 5);
    if (offset != "00:00") {
      return std::nullopt;
    }
    pos += 6;
  } else {
    return std::nullopt;
  }

  if (pos != text.size()) {
    return std::nullopt;  // trailing junk
  }

  std::tm tm_value{};
  tm_value.tm_year = *year - 1900;
  tm_value.tm_mon = *month - 1;
  tm_value.tm_mday = *day;
  tm_value.tm_hour = *hour;
  tm_value.tm_min = *minute;
  tm_value.tm_sec = *second;
  tm_value.tm_isdst = 0;

  const std::time_t seconds = timegm(&tm_value);
  if (seconds == static_cast<std::time_t>(-1)) {
    return std::nullopt;
  }
  // timegm normalises, so a day that does not exist in that month (31 April)
  // comes back as the first of the next one. Refusing it here keeps the parse
  // total rather than creative.
  if (tm_value.tm_mday != *day || tm_value.tm_mon != *month - 1) {
    return std::nullopt;
  }

  return static_cast<int64_t>(seconds) * kNsPerSecond + fraction_ns;
}

int64_t seconds_to_ns(double seconds) {
  if (!std::isfinite(seconds)) {
    return 0;
  }
  return static_cast<int64_t>(std::llround(seconds * static_cast<double>(kNsPerSecond)));
}

const PlannedStopWindow * window_for_fault(const std::vector<PlannedStopWindow> & windows,
                                           const nlohmann::json & fault_json) {
  if (!fault_json.is_object()) {
    return nullptr;
  }
  const auto it = fault_json.find("first_occurred");
  if (it == fault_json.end() || !it->is_number()) {
    return nullptr;
  }
  const double first_occurred_sec = it->get<double>();
  if (!std::isfinite(first_occurred_sec)) {
    return nullptr;
  }
  return find_covering_window(windows, seconds_to_ns(first_occurred_sec));
}

bool annotate_fault_with_planned_stop(nlohmann::json & fault_json, const std::vector<PlannedStopWindow> & windows) {
  if (!fault_json.is_object()) {
    return false;
  }
  const auto * covering = window_for_fault(windows, fault_json);

  // Merge rather than assign: an item may already carry vendor keys, and the
  // flag is an addition to them, not a replacement.
  nlohmann::json & extension = fault_json["x-medkit"];
  if (!extension.is_object()) {
    extension = nlohmann::json::object();
  }
  extension["expected"] = covering != nullptr;
  if (covering != nullptr) {
    extension["planned_stop_id"] = covering->id;
  } else {
    extension.erase("planned_stop_id");
  }
  return covering != nullptr;
}

void PlannedStopCache::store(std::vector<PlannedStopWindow> windows) {
  const std::lock_guard<std::mutex> lock(mutex_);
  windows_ = std::move(windows);
  ever_attempted_ = true;
  last_refresh_failed_ = false;
  attempted_at_ = std::chrono::steady_clock::now();
}

void PlannedStopCache::note_failed_refresh() {
  const std::lock_guard<std::mutex> lock(mutex_);
  ever_attempted_ = true;
  last_refresh_failed_ = true;
  attempted_at_ = std::chrono::steady_clock::now();
}

bool PlannedStopCache::last_refresh_failed() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return last_refresh_failed_;
}

std::vector<PlannedStopWindow> PlannedStopCache::snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return windows_;
}

bool PlannedStopCache::is_stale(std::chrono::steady_clock::duration ttl) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!ever_attempted_) {
    return true;
  }
  return std::chrono::steady_clock::now() - attempted_at_ >= ttl;
}

void PlannedStopCache::invalidate() {
  const std::lock_guard<std::mutex> lock(mutex_);
  ever_attempted_ = false;
}

}  // namespace faults
}  // namespace ros2_medkit_gateway
