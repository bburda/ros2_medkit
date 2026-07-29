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

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <rmw/qos_profiles.h>
#include <rmw/types.h>

namespace ros2_medkit_graph_watchdog {

/// Whether a QoS duration policy value is finite: not RMW_DURATION_UNSPECIFIED ({0,0}, "not
/// set", the most permissive value for an offer) and not RMW_DURATION_INFINITE (also unbounded).
inline bool qos_duration_is_finite(const rmw_time_t & t) {
  constexpr rmw_time_t kUnspecified = RMW_DURATION_UNSPECIFIED;
  constexpr rmw_time_t kInfinite = RMW_DURATION_INFINITE;
  const bool is_unspecified = t.sec == kUnspecified.sec && t.nsec == kUnspecified.nsec;
  const bool is_infinite = t.sec == kInfinite.sec && t.nsec == kInfinite.nsec;
  return !is_unspecified && !is_infinite;
}

/// Nanosecond value of a QoS duration, for RxO magnitude comparison. Saturates instead of
/// wrapping: rmw_time_t::sec is uint64_t, so a malformed profile carrying a huge-but-finite
/// second count would overflow the multiply and come back as a SMALL nanosecond value -
/// which reads as a tighter deadline than it is and flips the RxO verdict. Saturating keeps
/// an absurd duration on the permissive side of every comparison, the same side
/// RMW_DURATION_INFINITE sits on.
inline uint64_t qos_duration_ns(const rmw_time_t & t) {
  constexpr uint64_t kNsPerSec = 1000000000ull;
  constexpr uint64_t kMax = std::numeric_limits<uint64_t>::max();
  if (t.sec > kMax / kNsPerSec) {
    return kMax;
  }
  const uint64_t sec_ns = t.sec * kNsPerSec;
  return sec_ns > kMax - t.nsec ? kMax : sec_ns + t.nsec;
}

/// RxO (Request <= Offered) compatibility for the QoS policies that silently starve a
/// subscriber. Publisher = offered, subscriber = requested. Returns a human reason when
/// incompatible, empty string when compatible. The reliability/durability/liveliness-kind
/// checks match only concrete incompatible enum pairs, so SYSTEM_DEFAULT/UNKNOWN (never
/// reported by a live endpoint, which carries the resolved profile) never raise. Deadline and
/// liveliness lease duration are RxO-compatibility dimensions too and are checked below; history
/// and depth are NOT RxO-compatibility dimensions (they affect what gets buffered/resent, not
/// whether the reader/writer match), so we deliberately do not check them.
inline std::string qos_incompatibility(const rmw_qos_profile_t & pub, const rmw_qos_profile_t & sub) {
  if (sub.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
      pub.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    return "reliability: publisher BEST_EFFORT vs subscriber RELIABLE";
  }
  if (sub.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL &&
      pub.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE) {
    return "durability: publisher VOLATILE vs subscriber TRANSIENT_LOCAL";
  }
  if (sub.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC &&
      pub.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC) {
    return "liveliness: publisher AUTOMATIC vs subscriber MANUAL_BY_TOPIC";
  }
  if (qos_duration_is_finite(sub.deadline) &&
      (!qos_duration_is_finite(pub.deadline) || qos_duration_ns(pub.deadline) > qos_duration_ns(sub.deadline))) {
    return "deadline: publisher deadline exceeds subscriber's requested deadline";
  }
  if (qos_duration_is_finite(sub.liveliness_lease_duration) &&
      (!qos_duration_is_finite(pub.liveliness_lease_duration) ||
       qos_duration_ns(pub.liveliness_lease_duration) > qos_duration_ns(sub.liveliness_lease_duration))) {
    return "liveliness lease: publisher lease duration exceeds subscriber's requested lease duration";
  }
  return "";
}

/// How a subscriber fares against the publishers on its topic.
enum class SubscriberQosVerdict {
  Ok,       ///< RxO-compatible with every publisher.
  Partial,  ///< Incompatible with at least one publisher, but at least one works.
  Starved,  ///< Incompatible with EVERY publisher - it receives nothing at all.
};

struct SubscriberQosResult {
  SubscriberQosVerdict verdict = SubscriberQosVerdict::Ok;
  std::string reason;  ///< Representative incompatibility; empty when Ok.
};

/// Classify one subscriber against the publishers on its topic.
///
/// Partial is reported, not swallowed. An RxO-incompatible pair is not a heuristic - it is
/// a match DDS has already refused, so the data on that edge is discarded and no error is
/// raised anywhere. On a topic with several publishers (a second /tf broadcaster, a
/// hand-rolled BEST_EFFORT publisher next to /diagnostics' aggregator) the subscriber keeps
/// receiving everyone else's traffic, so nothing looks wrong while one producer's output
/// silently goes nowhere. That is precisely the class of failure this plugin exists to
/// surface; the severity split is what keeps it from reading as loudly as total starvation.
///
/// An empty publisher set is Ok here: a subscriber with no publishers is an orphan, which
/// is the orphan detector's concern, not a QoS fault.
inline SubscriberQosResult classify_subscriber_qos(const std::vector<rmw_qos_profile_t> & pubs,
                                                   const rmw_qos_profile_t & sub) {
  SubscriberQosResult result;
  if (pubs.empty()) {
    return result;
  }
  std::size_t incompatible = 0;
  for (const auto & pub : pubs) {
    const auto reason = qos_incompatibility(pub, sub);
    if (reason.empty()) {
      continue;
    }
    ++incompatible;
    if (result.reason.empty()) {
      result.reason = reason;
    }
  }
  if (incompatible == 0) {
    return result;
  }
  result.verdict = incompatible == pubs.size() ? SubscriberQosVerdict::Starved : SubscriberQosVerdict::Partial;
  return result;
}

/// Whether a subscriber is STARVED on a topic: it has publishers but is RxO-incompatible with
/// ALL of them, so it receives nothing. Returns a representative reason when starved, empty
/// otherwise. Kept as the narrow "receives nothing" predicate; use classify_subscriber_qos()
/// to also see the partial case.
inline std::string subscriber_starvation(const std::vector<rmw_qos_profile_t> & pubs, const rmw_qos_profile_t & sub) {
  const auto result = classify_subscriber_qos(pubs, sub);
  return result.verdict == SubscriberQosVerdict::Starved ? result.reason : "";
}

}  // namespace ros2_medkit_graph_watchdog
