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
#include <cstdint>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/orphan_policy.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// A pub-only/sub-only near-miss pair means data silently never arrives at its intended
// destination - as severe as QoS starvation.
constexpr std::uint8_t kOrphanSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
// Catches single-char typos, e.g. /scann vs /scan. Numeric siblings such as /lidar_1 vs
// /lidar_2 are within this distance but are spared by differs_only_in_numeric_fields().
// Config-overridable via "max_edit_distance".
constexpr int kDefaultMaxEditDistance = 1;
// Upper bound on the configurable edit distance: beyond a few edits a "near miss" matches
// unrelated topics, and an unbounded value would overflow `best` in find_orphans. A value
// outside (0, kMaxEditDistance] keeps the default.
constexpr int kMaxEditDistance = 8;
// The namespace must match to the character unless an operator opts out. A fleet named with
// letters (/amr_a, /amr_b) puts every robot one edit from its neighbour, so any budget above 0
// reports the entire fleet as typos - the numeric-field rule spares a NUMBERED fleet, but
// nothing spares a lettered one. Raising it is how a misspelled namespace (/robto/scan vs
// /robot/scan) becomes visible at all, which is a real remap mistake this detector otherwise
// cannot see. That trade belongs to whoever knows the naming scheme, so it is opt-in.
// A value outside [0, kMaxEditDistance] keeps the default.
constexpr int kDefaultNamespaceEditDistance = 0;

/// Consecutive ticks a candidate pair must hold before it is reported.
///
/// Without this one sweep raises, and the shipped fault_manager config carries
/// `confirmation_threshold: -1`, so that single tick becomes a CONFIRMED ERROR and - with
/// `snapshots.rosbag.enabled` - burns a black-box capture. The pub-only/sub-only shape this
/// detector looks for is exactly what a restart produces: a node that publishes one name and
/// subscribes a near-identical one of the same type (a CAN bridge on /can_rx + /can_tx) makes
/// its peer's topics look one-sided for as long as that peer is down, and staged bringup does
/// the same before the second half starts.
///
/// The reliability gate does not cover it: graph_source_id() is not an app id, so
/// allows_raise() takes the unknown-source branch and is permanently armed once the graph has
/// been non-empty for warmup_cycles ticks. Anything restarted after that gets no settle window
/// at all.
///
/// A real name typo is static - still there N ticks later - so this costs no meaningful
/// detection latency. The default outlives a supervisor restart at the 1s default tick; it
/// does NOT outlive a slow staged bringup, which is why the description stays a candidate.
constexpr int kDefaultGrace = 10;
}  // namespace

/// Watches every topic for a one-sided endpoint (pub-only or sub-only) with a near-miss,
/// same-type counterpart in the same namespace carrying the complementary side - the signature of
/// a remap / topic-name typo. See orphan_policy.hpp for the pure matching core (dedup,
/// system-topic skip, same-namespace guard) and design doc.
class OrphanDetector : public Detector {
 public:
  std::string id() const override {
    return "orphan";
  }

  void configure(const nlohmann::json & config) override {
    std::vector<std::string> warnings;
    if (config.contains("max_edit_distance")) {
      const auto & value = config["max_edit_distance"];
      if (value.is_number_integer() && value.get<int>() > 0 && value.get<int>() <= kMaxEditDistance) {
        max_edit_distance_ = value.get<int>();
      } else {
        warnings.push_back("'max_edit_distance' must be an integer in 1.." + std::to_string(kMaxEditDistance) +
                           "; keeping the default (" + std::to_string(kDefaultMaxEditDistance) + ")");
      }
    }
    if (config.contains("namespace_edit_distance")) {
      const auto & value = config["namespace_edit_distance"];
      if (value.is_number_integer() && value.get<int>() >= 0 && value.get<int>() <= kMaxEditDistance) {
        namespace_edit_distance_ = value.get<int>();
      } else {
        warnings.push_back("'namespace_edit_distance' must be an integer in 0.." + std::to_string(kMaxEditDistance) +
                           "; keeping the default (" + std::to_string(kDefaultNamespaceEditDistance) + ")");
      }
    }
    if (config.contains("grace")) {
      const auto & value = config["grace"];
      if (value.is_number_integer() && value.get<int>() >= 0) {
        grace_ = value.get<int>();
      } else {
        warnings.push_back("'grace' must be a non-negative integer; keeping the default (" +
                           std::to_string(kDefaultGrace) + ")");
      }
    }
    // Same escape hatch every other detector offers: a heuristic over topic NAMES cannot be
    // right for every graph, and without this an operator whose layout trips it has no option
    // but to turn the whole detector off. Exact match only, never a prefix, so allowlisting
    // /r1/scan cannot silently also silence /r2/scan.
    allowlist_.clear();
    if (config.contains("allowlist")) {
      const auto & value = config["allowlist"];
      if (value.is_array()) {
        for (const auto & entry : value) {
          if (entry.is_string() && !entry.get<std::string>().empty()) {
            allowlist_.insert(entry.get<std::string>());
          }
        }
      } else {
        warnings.push_back("'allowlist' must be an array of topic names; ignoring it");
      }
    }
    collect_unknown_detector_keys(config, known_keys(), warnings);
    warnings_ = std::move(warnings);
    streak_.clear();
  }

  void tick(DetectorContext & ctx) override {
    if (!ctx.gateway_node) {
      return;
    }
    log_warnings_once(ctx);

    std::vector<TopicEndpointCounts> topics;
    const auto names_and_types = ctx.gateway_node->get_topic_names_and_types();
    topics.reserve(names_and_types.size());
    for (const auto & [name, types] : names_and_types) {
      if (ctx.cancelled && ctx.cancelled->load()) {
        return;  // shutting down: do not emit a clear built from a partial sweep
      }
      TopicEndpointCounts t;
      t.name = name;
      t.type = types.empty() ? std::string() : types.front();
      t.publishers = ctx.gateway_node->count_publishers(name);
      t.subscribers = ctx.gateway_node->count_subscribers(name);
      topics.push_back(std::move(t));
    }

    std::map<std::string, std::string> affected;  // "pub <-> sub" -> reason
    std::vector<std::string> newly_affected;      // crossed the gate this tick -> named first
    std::set<std::string> seen_pairs;
    for (const auto & finding : find_orphans(topics, max_edit_distance_, namespace_edit_distance_)) {
      const std::string key = finding.publisher_topic + " <-> " + finding.subscriber_topic;
      seen_pairs.insert(key);
      // Persistence gate, per pair so one flapping pair never delays another.
      // A pair crossing the gate on THIS tick is the new information. Each orphan reason is
      // long, so two entries fill the description cap; without this a newly detected pair
      // sorting later changes nothing an operator can see.
      if (allowlist_.count(finding.publisher_topic) != 0 || allowlist_.count(finding.subscriber_topic) != 0) {
        streak_.erase(key);
        continue;
      }
      if (++streak_[key] == grace_ + 1) {
        newly_affected.push_back(key);
      }
      if (streak_[key] <= grace_) {
        continue;
      }
      affected[key] = finding.reason;
    }
    // Drop streaks for pairs that stopped matching, so a pair has to hold CONSECUTIVE ticks
    // and the map cannot grow without bound on a graph that churns topic names.
    for (auto it = streak_.begin(); it != streak_.end();) {
      it = seen_pairs.count(it->first) == 0 ? streak_.erase(it) : std::next(it);
    }

    aggregated_.emit_ordered(ctx, affected, newly_affected);
  }

 private:
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"allowlist", "grace", "max_edit_distance", "namespace_edit_distance"};
    return keys;
  }

  void log_warnings_once(const DetectorContext & ctx) {
    if (warnings_.empty() || warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    for (const auto & warning : warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "graph_watchdog orphan: %s", warning.c_str());
    }
    warnings_logged_ = true;
  }

  int max_edit_distance_ = kDefaultMaxEditDistance;
  int namespace_edit_distance_ = kDefaultNamespaceEditDistance;
  int grace_ = kDefaultGrace;
  std::set<std::string> allowlist_;    // topic names never reported
  std::map<std::string, int> streak_;  ///< pair key -> consecutive matching ticks
  std::vector<std::string> warnings_;
  bool warnings_logged_ = false;
  AggregatedFault aggregated_{graph_fault_codes::kOrphan, kOrphanSeverity};
};

REGISTER_DETECTOR(OrphanDetector, "orphan")

}  // namespace ros2_medkit_graph_watchdog
