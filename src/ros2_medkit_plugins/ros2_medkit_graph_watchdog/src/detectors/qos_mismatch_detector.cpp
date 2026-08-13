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

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/qos_policy.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// A starved subscriber never receives data at all - stronger than a drifted parameter.
constexpr std::uint8_t kStarvedSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
// A partially-incompatible subscriber still receives the other publishers, so the topic is
// degraded rather than dead. Reported, but not at the same level as total starvation.
constexpr std::uint8_t kPartialSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;

/// Consecutive affected ticks a topic needs before it is reported.
///
/// Without this a single sweep raises, and the shipped fault_manager config carries
/// `confirmation_threshold: -1`, so that one sweep is CONFIRMED ERROR immediately and -
/// with `snapshots.rosbag.enabled` - pulls a black-box capture with it. Endpoint discovery
/// is not atomic: during bringup a subscriber can be visible before a publisher's QoS is,
/// which reads exactly like starvation for a tick or two. A genuine QoS mismatch is static,
/// still there N ticks later, so persistence costs almost no detection latency and removes
/// the whole transient class.
constexpr int kDefaultGrace = 3;

/// Human-readable owner of an endpoint, for the fault description.
std::string endpoint_name(const rclcpp::TopicEndpointInfo & endpoint) {
  const std::string & ns = endpoint.node_namespace();
  const std::string & name = endpoint.node_name();
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  return ns + "/" + name;
}

std::string join_names(const std::set<std::string> & names) {
  std::string out;
  for (const auto & name : names) {
    if (!out.empty()) {
      out += ", ";
    }
    out += name;
  }
  return out;
}
}  // namespace

/// Watches every topic and raises GRAPH_QOS_MISMATCH when a subscriber is RxO-incompatible
/// with a publisher on its topic (see qos_policy.hpp). Two levels behind one fault code:
///
/// - STARVED: incompatible with EVERY publisher, so it receives nothing -> ERROR.
/// - PARTIAL: incompatible with some publisher but not all -> WARN. DDS refuses that one
///   pair, so that producer's data is discarded while the topic still looks alive; nothing
///   else in the system reports it.
///
/// A subscriber with zero publishers is an orphan (a different detector), not a QoS fault.
/// See design doc.
class QosMismatchDetector : public Detector {
 public:
  std::string id() const override {
    return "qos_mismatch";
  }

  void configure(const nlohmann::json & config) override {
    std::vector<std::string> warnings;
    if (config.contains("grace")) {
      if (config["grace"].is_number_integer() && config["grace"].get<int>() >= 0) {
        grace_ = config["grace"].get<int>();
      } else {
        warnings.push_back("'grace' must be a non-negative integer; keeping the default (" +
                           std::to_string(kDefaultGrace) + ")");
      }
    }
    if (config.contains("allowlist")) {
      if (config["allowlist"].is_array()) {
        for (const auto & entry : config["allowlist"]) {
          if (entry.is_string() && !entry.get<std::string>().empty()) {
            allowlist_.insert(entry.get<std::string>());
          } else {
            warnings.push_back("'allowlist' entries must be non-empty topic names; skipping one");
          }
        }
      } else {
        warnings.push_back("'allowlist' must be a string array of topic names; ignoring it");
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

    std::map<std::string, std::string> affected;  // topic -> "<topic>: <reasons>"
    std::vector<std::string> newly_affected;      // crossed the gate this tick -> named first
    bool any_starved = false;
    std::set<std::string> seen_topics;
    const auto topics = ctx.gateway_node->get_topic_names_and_types();
    for (const auto & [topic, types] : topics) {
      (void)types;
      if (ctx.cancelled && ctx.cancelled->load()) {
        return;  // shutting down: do not emit a clear built from a partial sweep
      }
      // /rosout and /parameter_events are ROS 2 system topics with their own
      // well-known QoS conventions - not useful signal for this detector.
      if (topic == "/rosout" || topic == "/parameter_events") {
        continue;
      }
      // An operator's own tool is not a robot fault. rviz2 builds every display's
      // subscription from rclcpp::QoS(5) - RELIABLE - and does not negotiate down to a
      // BEST_EFFORT sensor publisher, so opening an Image display on a driver topic reads
      // as starvation. Without this lever the only way to silence that is turning the
      // whole detector off.
      if (allowlist_.count(topic) != 0) {
        continue;
      }
      seen_topics.insert(topic);

      const auto pubs = ctx.gateway_node->get_publishers_info_by_topic(topic);
      if (pubs.empty()) {
        continue;  // no publishers -> not a QoS starvation (orphan detector's concern)
      }
      std::vector<rmw_qos_profile_t> pub_qos;
      pub_qos.reserve(pubs.size());
      for (const auto & pub : pubs) {
        pub_qos.push_back(pub.qos_profile().get_rmw_qos_profile());
      }
      // reason -> the subscribers hitting it. Keyed by reason so several subscribers
      // failing the same policy collapse into one phrase, but each still NAMES itself:
      // without that, "reliability: publisher BEST_EFFORT vs subscriber RELIABLE" on a
      // topic with a dozen subscribers still leaves the operator to ssh in and run
      // `ros2 topic info -v` to find the starved one - the step the fault exists to remove.
      std::map<std::string, std::set<std::string>> starved_by_reason;
      std::map<std::string, std::set<std::string>> partial_by_reason;
      for (const auto & sub : ctx.gateway_node->get_subscriptions_info_by_topic(topic)) {
        const auto result = classify_subscriber_qos(pub_qos, sub.qos_profile().get_rmw_qos_profile());
        if (result.verdict == SubscriberQosVerdict::Starved) {
          starved_by_reason[result.reason].insert(endpoint_name(sub));
        } else if (result.verdict == SubscriberQosVerdict::Partial) {
          partial_by_reason[result.reason].insert(endpoint_name(sub));
        }
      }
      if (starved_by_reason.empty() && partial_by_reason.empty()) {
        streak_.erase(topic);
        continue;
      }
      // Persistence gate, per topic so one flapping topic never delays another.
      if (++streak_[topic] <= grace_) {
        continue;
      }
      if (!starved_by_reason.empty()) {
        any_starved = true;
      }
      // A topic crossing the gate on THIS tick is the new information. The description is
      // capped, and affected topics are otherwise listed lexicographically, so without this
      // a newly starved subscriber on a late-sorting topic would produce no operator-visible
      // change at all: the code is already raised and the text is already full.
      if (streak_[topic] == grace_ + 1) {
        newly_affected.push_back(topic);
      }
      affected[topic] = describe_topic(topic, starved_by_reason, partial_by_reason);
    }
    // Drop streaks for topics that left the graph, so the map cannot grow without bound on
    // a graph that churns topic names.
    for (auto it = streak_.begin(); it != streak_.end();) {
      it = seen_topics.count(it->first) == 0 ? streak_.erase(it) : std::next(it);
    }

    // One fault code carries one severity, so it reflects the worst finding in the sweep:
    // ERROR as soon as any subscriber anywhere receives nothing at all, WARN when every
    // finding is a degraded-but-alive topic.
    AggregatedFault aggregated{graph_fault_codes::kQosMismatch, any_starved ? kStarvedSeverity : kPartialSeverity};
    aggregated.emit_ordered(ctx, affected, newly_affected);
  }

 private:
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"grace", "allowlist"};
    return keys;
  }

  static std::string describe_topic(const std::string & topic,
                                    const std::map<std::string, std::set<std::string>> & starved_by_reason,
                                    const std::map<std::string, std::set<std::string>> & partial_by_reason) {
    std::string detail = topic + ": ";
    bool first = true;
    for (const auto & [reason, names] : starved_by_reason) {
      if (!first) {
        detail += "; ";
      }
      detail += reason + " (receives nothing: " + join_names(names) + ")";
      first = false;
    }
    for (const auto & [reason, names] : partial_by_reason) {
      if (!first) {
        detail += "; ";
      }
      detail += reason + " (one publisher discarded, still receiving others: " + join_names(names) + ")";
      first = false;
    }
    return detail;
  }

  void log_warnings_once(const DetectorContext & ctx) {
    if (warnings_.empty() || warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    for (const auto & warning : warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "graph_watchdog qos_mismatch: %s", warning.c_str());
    }
    warnings_logged_ = true;
  }

  int grace_ = kDefaultGrace;
  std::set<std::string> allowlist_;
  std::map<std::string, int> streak_;  ///< topic -> consecutive affected ticks
  std::vector<std::string> warnings_;
  bool warnings_logged_ = false;
};

REGISTER_DETECTOR(QosMismatchDetector, "qos_mismatch")

}  // namespace ros2_medkit_graph_watchdog
