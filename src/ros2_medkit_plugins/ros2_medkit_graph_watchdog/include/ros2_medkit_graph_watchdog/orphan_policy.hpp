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
#include <set>
#include <string>
#include <vector>

namespace ros2_medkit_graph_watchdog {

struct TopicEndpointCounts {
  std::string name;
  std::string type;
  std::size_t publishers = 0;
  std::size_t subscribers = 0;
};

/// A near-miss pair of one-sided endpoints. We name both ends and deliberately do
/// NOT recommend which to rename - the canonical name is the operator's call.
struct OrphanFinding {
  std::string publisher_topic;   // the pub-only side of the pair
  std::string subscriber_topic;  // the sub-only side of the pair
  std::string reason;
};

inline int levenshtein(const std::string & a, const std::string & b) {
  std::vector<int> prev(b.size() + 1), cur(b.size() + 1);
  for (std::size_t j = 0; j <= b.size(); ++j) {
    prev[j] = static_cast<int>(j);
  }
  for (std::size_t i = 1; i <= a.size(); ++i) {
    cur[0] = static_cast<int>(i);
    for (std::size_t j = 1; j <= b.size(); ++j) {
      const int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
      cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost});
    }
    std::swap(prev, cur);
  }
  return prev[b.size()];
}

/// ROS 2 system topics, skipped because they have their own lifecycle and pairing them
/// would be noise. Both names are ABSOLUTE and global - a node in a namespace still
/// publishes to /rosout and /parameter_events - so an exact match is the whole set. A
/// prefix match would additionally swallow user topics that merely start with the same
/// text (/parameter_events_debug), silently exempting them from orphan detection. Same
/// exact-match rule as qos_mismatch_detector.cpp.
inline bool is_system_topic(const std::string & name) {
  return name == "/rosout" || name == "/parameter_events";
}

/// Everything up to and including the final '/'. "/ns/scan" -> "/ns/", "/scan" -> "/".
inline std::string parent_namespace(const std::string & topic) {
  const auto slash = topic.rfind('/');
  return slash == std::string::npos ? std::string() : topic.substr(0, slash + 1);
}

/// A name with every RUN of digits collapsed to a single '#'. "/lidar_1" and "/lidar_10"
/// both become "/lidar_#"; "/scan" is unchanged.
inline std::string collapse_digit_runs(const std::string & name) {
  std::string out;
  out.reserve(name.size());
  bool in_digits = false;
  for (const char c : name) {
    const bool digit = c >= '0' && c <= '9';
    if (digit) {
      if (!in_digits) {
        out.push_back('#');
      }
    } else {
      out.push_back(c);
    }
    in_digits = digit;
  }
  return out;
}

/// True when two names differ only in their numeric fields. `/lidar_1` vs `/lidar_2`,
/// `/image1` vs `/image2`, `/cameras/image1` vs `/cameras/image2`, and also `/lidar_9` vs
/// `/lidar_10`, where the index crosses a digit-count boundary. A difference confined to a
/// numeric field is a sibling index, not a typo: two independently one-sided members of such a
/// family is an ordinary state on a multi-sensor robot, where nothing is recording `/lidar_1`
/// while `/lidar_2`'s driver is down.
///
/// The same-namespace guard does not catch these, because it only helps when the differing
/// character sits in a namespace segment; here it sits in the leaf and both names share a
/// parent. Reporting them names two unrelated topics as a possible mismatch, at ERROR.
///
/// Comparing collapsed names rather than character positions is what covers the
/// digit-count boundary: `/lidar_9` and `/lidar_10` are not even the same length, yet they
/// are the ninth and tenth sensor of one family.
///
/// Known cost, accepted deliberately: a typo that IS a digit (a remap writing `/lidar_2`
/// where `/lidar_1` was meant) stops being reported. A false ERROR on every multi-sensor
/// robot is worse than that miss, and this detector's whole premise is staying silent on a
/// graph nobody configured. An appended digit is NOT this case: `/scan1` collapses to
/// `/scan#`, which is not `/scan`, so it is still reported.
inline bool differs_only_in_numeric_fields(const std::string & a, const std::string & b) {
  return a != b && collapse_digit_runs(a) == collapse_digit_runs(b);
}

/// Everything after the final '/'. "/ns/scan" -> "scan", "/scan" -> "scan".
inline std::string leaf_name(const std::string & topic) {
  const auto slash = topic.rfind('/');
  return slash == std::string::npos ? topic : topic.substr(slash + 1);
}

/// A one-sided endpoint (pub-only or sub-only) plus a near-miss topic of the SAME
/// message type carrying the complementary side is the signature of a remap / name
/// typo. Each such pair is reported ONCE (deduped by the unordered name pair). The
/// type + distance gate stops legitimate lone debug topics from false-positiving.
///
/// `max_edit_distance` budgets the LEAF, `namespace_edit_distance` budgets the parent
/// namespace, and the two are independent. The default namespace budget is 0, i.e. the
/// namespace must match to the character, which is the only safe default: on a fleet named
/// with letters (/amr_a, /amr_b) every robot's topic is one edit from its neighbour's, so any
/// namespace budget above 0 reports the whole fleet as typos. The numeric-field rule saves
/// a NUMBERED fleet (/robot1, /robot2) but nothing saves a lettered one, which is why this is
/// opt-in rather than a tuned default.
inline std::vector<OrphanFinding> find_orphans(const std::vector<TopicEndpointCounts> & topics, int max_edit_distance,
                                               int namespace_edit_distance = 0) {
  std::vector<OrphanFinding> out;
  std::set<std::string> paired;  // canonical "min\nmax" keys already reported
  for (const auto & t : topics) {
    if (is_system_topic(t.name)) {
      continue;
    }
    const bool pub_only = t.publishers > 0 && t.subscribers == 0;
    const bool sub_only = t.subscribers > 0 && t.publishers == 0;
    if (!pub_only && !sub_only) {
      continue;
    }
    // Admission is the two budgets above; this only picks the CLOSEST admitted candidate and
    // supplies the "differ by N char(s)" figure. The bound is the sum of both budgets because
    // a whole-name distance never exceeds the namespace distance plus the leaf distance.
    int best = max_edit_distance + namespace_edit_distance + 1;
    const TopicEndpointCounts * match = nullptr;
    for (const auto & u : topics) {
      if (&u == &t || is_system_topic(u.name) || u.type != t.type) {
        continue;
      }
      // Budget the namespace and the leaf separately. At the default namespace budget of 0
      // this is an exact-namespace guard: a leaf typo (/scann vs /scan) matches, while
      // distinct per-robot topics (/robot1/scan vs /robot2/scan, also distance 1) do not -
      // that is a fleet layout, not a typo. Raising the namespace budget is what lets a
      // misspelled namespace (/robto/scan vs /robot/scan) pair at all.
      if (levenshtein(parent_namespace(t.name), parent_namespace(u.name)) > namespace_edit_distance) {
        continue;
      }
      if (levenshtein(leaf_name(t.name), leaf_name(u.name)) > max_edit_distance) {
        continue;
      }
      // An enumeration of sensors, not a typo - see differs_only_in_numeric_fields().
      if (differs_only_in_numeric_fields(t.name, u.name)) {
        continue;
      }
      // The counterpart must be strictly one-sided in the COMPLEMENTARY direction: a typo means
      // the publisher meant the subscriber-only topic (which is starved) and vice-versa. A fully
      // connected sibling (a healthy /lidar_2 next to a one-sided /lidar_1) is NOT a typo, so it
      // must NOT pair - otherwise every digit-substitution sensor sibling would false-positive.
      const bool complements =
          pub_only ? (u.subscribers > 0 && u.publishers == 0) : (u.publishers > 0 && u.subscribers == 0);
      if (!complements) {
        continue;
      }
      const int d = levenshtein(t.name, u.name);
      if (d >= 1 && d < best) {
        best = d;
        match = &u;
      }
    }
    if (match == nullptr) {
      continue;
    }
    const std::string key = t.name < match->name ? t.name + "\n" + match->name : match->name + "\n" + t.name;
    if (!paired.insert(key).second) {
      continue;  // this unordered pair already reported (from the other side)
    }
    const std::string & pub_side = pub_only ? t.name : match->name;
    const std::string & sub_side = pub_only ? match->name : t.name;
    // Deliberately a CANDIDATE, not a conclusion. The pub-only/sub-only shape is not unique
    // to a typo: any node that publishes one name and subscribes a near-identical one of the
    // same type - a CAN/serial bridge on /can_rx + /can_tx, any relay that republishes a
    // one-character variant - leaves exactly this shape the moment its peer exits. Telling an
    // operator to "reconcile the two names" in that case hands them a wrong remediation on
    // top of whatever actually died.
    std::string detail = "possible topic-name mismatch: publisher-only '";
    detail += pub_side;
    detail += "' and subscriber-only '";
    detail += sub_side;
    detail += "' share type ";
    detail += t.type;
    detail += " and differ by ";
    detail += std::to_string(best);
    detail += " char(s) - either a remap typo or a departed counterpart; check both before renaming";
    out.push_back({pub_side, sub_side, std::move(detail)});
  }
  return out;
}

}  // namespace ros2_medkit_graph_watchdog
