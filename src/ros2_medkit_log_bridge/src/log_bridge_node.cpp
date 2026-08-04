// Copyright 2026 mfaferek93, bburda
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

#include "ros2_medkit_log_bridge/log_bridge_node.hpp"

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <limits>
#include <utility>

#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_log_bridge {

namespace {

// rcl_interfaces/msg/Log severity levels.
constexpr uint8_t kLevelDebug = 10;
constexpr uint8_t kLevelInfo = 20;
constexpr uint8_t kLevelWarn = 30;
constexpr uint8_t kLevelError = 40;
constexpr uint8_t kLevelFatal = 50;

bool contains_substr(const std::vector<std::string> & list, const std::string & value) {
  for (const auto & item : list) {
    if (!item.empty() && value.find(item) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

LogBridgeNode::LogBridgeNode(const rclcpp::NodeOptions & options) : Node("log_bridge", options) {
  load_parameters();
  own_node_name_ = get_fully_qualified_name();

  // /rosout is published reliable+volatile; keep a deep queue so log bursts at
  // fault time are not dropped before the bridge processes them.
  log_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      rosout_topic_, rclcpp::QoS(rclcpp::KeepLast(1000)), [this](const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
        log_callback(msg);
      });

  RCLCPP_INFO(get_logger(), "LogBridge started (topic=%s, severity_floor=%u, prefix=%s)", rosout_topic_.c_str(),
              static_cast<unsigned>(severity_floor_), code_prefix_.c_str());
}

void LogBridgeNode::load_parameters() {
  rosout_topic_ = declare_parameter<std::string>("rosout_topic", "/rosout");
  // Default floor is WARN. WARN passes through each node's FaultReporter
  // LocalFilter (threshold/window debounce); ERROR/FATAL bypass it. Raise to 40
  // (ERROR) on chatty / constrained targets to cut volume.
  // ROS parameters are int64; read them as such and narrow only after the
  // clamp, so an out-of-range value cannot wrap on the way in.
  const int64_t floor = declare_parameter<int64_t>("severity_floor", static_cast<int64_t>(kLevelWarn));
  // int -> uint8_t silently wraps; clamp so a bad value cannot pass everything.
  if (floor < 0 || floor > kLevelFatal) {
    RCLCPP_WARN(get_logger(), "severity_floor=%" PRId64 " out of range [0,%u], clamping", floor,
                static_cast<unsigned>(kLevelFatal));
  }
  severity_floor_ = static_cast<uint8_t>(std::clamp(floor, INT64_C(0), static_cast<int64_t>(kLevelFatal)));
  // Normalize the prefix so a non-conforming value cannot yield a fault_code
  // violating medkit's [A-Z0-9_] charset.
  code_prefix_ = to_upper_snake(declare_parameter<std::string>("code_prefix", "LOG"), 32);
  if (code_prefix_.empty()) {
    code_prefix_ = "LOG";
  }
  exclude_nodes_ = declare_parameter<std::vector<std::string>>("exclude_nodes", std::vector<std::string>{});
  include_only_nodes_ = declare_parameter<std::vector<std::string>>("include_only_nodes", std::vector<std::string>{});
  const int64_t max_tracked_nodes = declare_parameter<int64_t>("max_tracked_nodes", INT64_C(512));
  max_tracked_nodes_ = static_cast<int>(
      std::clamp(max_tracked_nodes, INT64_C(1), static_cast<int64_t>(std::numeric_limits<int>::max())));
  report_cooldown_sec_ = declare_parameter<double>("report_cooldown_sec", 5.0);
  if (report_cooldown_sec_ < 0.0) {
    report_cooldown_sec_ = 0.0;
  }
  exclude_medkit_stack_ = declare_parameter<bool>("exclude_medkit_stack", true);
}

void LogBridgeNode::log_callback(const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
  // Resolve the FQN once and use it for code-gen, eligibility, self-exclusion,
  // and reporter source_id so they cannot disagree.
  const std::string source_id = node_source_id(msg->name);

  // Never promote our own logs (avoids any self-referential loop).
  if (source_id == own_node_name_ || source_id == "/log_bridge") {
    return;
  }
  // Skip the medkit stack's own infrastructure nodes (else fault_manager's own
  // /rosout lines feed back as faults about medkit). Matched on the raw logger
  // name, which keeps the namespace (node_source_id collapses it to the ns).
  if (skips_medkit_stack(msg->name)) {
    return;
  }
  if (!node_is_eligible(source_id)) {
    return;
  }

  uint8_t severity = 0;
  if (!map_level_to_severity(msg->level, severity_floor_, &severity)) {
    return;  // below floor / INFO / DEBUG -> dropped
  }

  const std::string fault_code = generate_fault_code(source_id, msg->msg);

  // ERROR/FATAL bypass each node's LocalFilter and forward immediately, so bound
  // a same-code flood here. WARN is left to the LocalFilter (cooling it would
  // starve its threshold counting). Keyed by severity so a WARN never suppresses
  // a same-message ERROR escalation.
  if (severity >= ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR && !cooldown_allows(fault_code, severity, now())) {
    return;
  }

  // Attribute to the node FQN so the fault associates with the gateway's
  // runtime-discovered entity (and its snapshots/rosbag become reachable).
  auto * reporter = reporter_for(source_id);
  if (reporter == nullptr) {
    return;
  }
  reporter->report(fault_code, severity, msg->msg);
  RCLCPP_DEBUG(get_logger(), "Log %s [lvl=%u] '%s' -> fault %s (severity=%u)", msg->name.c_str(),
               static_cast<unsigned>(msg->level), msg->msg.c_str(), fault_code.c_str(),
               static_cast<unsigned>(severity));
}

std::string LogBridgeNode::node_source_id(const std::string & log_name) {
  // The logger name is the node name, optionally with a sub-logger suffix after
  // '.' (node names themselves cannot contain '.'), so the first dotted segment
  // is the node. Prefix '/' to form the FQN the gateway's entity discovery uses.
  // Caveat: a namespaced node dots its namespace too (robot1.planner_server),
  // indistinguishable from a sub-logger, so this collapses it to '/robot1'.
  // See README (namespaced-node limitation).
  std::string node = log_name.substr(0, log_name.find('.'));
  if (node.empty()) {
    node = log_name;
  }
  if (node.empty() || node.front() != '/') {
    node = "/" + node;
  }
  return node;
}

bool LogBridgeNode::is_medkit_stack_logger(const std::string & logger_name) {
  // Match medkit's own infrastructure against the raw logger name so a
  // namespaced node (e.g. "robot1.fault_manager") is still caught, even though
  // node_source_id would collapse its FQN to the namespace.
  static const std::vector<std::string> kMedkitStack = {"fault_manager", "ros2_medkit_gateway", "diagnostic_bridge",
                                                        "action_status_bridge"};
  return contains_substr(kMedkitStack, logger_name);
}

bool LogBridgeNode::skips_medkit_stack(const std::string & logger_name) const {
  return exclude_medkit_stack_ && is_medkit_stack_logger(logger_name);
}

bool LogBridgeNode::node_is_eligible(const std::string & source_id) const {
  if (!include_only_nodes_.empty() && !contains_substr(include_only_nodes_, source_id)) {
    return false;
  }
  if (contains_substr(exclude_nodes_, source_id)) {
    return false;
  }
  return true;
}

bool LogBridgeNode::cooldown_allows(const std::string & fault_code, uint8_t severity, const rclcpp::Time & now) {
  if (report_cooldown_sec_ <= 0.0) {
    return true;
  }
  const std::string key = fault_code + ":" + std::to_string(static_cast<unsigned>(severity));
  const auto window = rclcpp::Duration::from_seconds(report_cooldown_sec_);
  std::lock_guard<std::mutex> lock(cooldown_mutex_);
  auto it = last_forward_.find(key);
  if (it != last_forward_.end() && (now - it->second) < window) {
    return false;
  }
  // Drop entries past their window so the map stays bounded by active codes.
  for (auto e = last_forward_.begin(); e != last_forward_.end();) {
    e = (e->first != key && (now - e->second) >= window) ? last_forward_.erase(e) : std::next(e);
  }
  last_forward_[key] = now;
  return true;
}

ros2_medkit_fault_reporter::FaultReporter * LogBridgeNode::reporter_for(const std::string & source_id) {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  auto it = reporters_.find(source_id);
  if (it != reporters_.end()) {
    // Touch: move to the back (most-recently-used).
    reporters_lru_.splice(reporters_lru_.end(), reporters_lru_, it->second);
    return it->second->reporter.get();
  }
  // source_id is the ORIGINATING node so faults are attributed correctly.
  // Multiple FaultReporters on one node are safe: the ctor guards parameter
  // declaration with has_parameter().
  auto reporter = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(this->shared_from_this(), source_id);
  auto * raw = reporter.get();
  reporters_lru_.push_back(ReporterEntry{source_id, std::move(reporter)});
  reporters_[source_id] = std::prev(reporters_lru_.end());

  // Cap the map: evict the least-recently-used node to bound growth under churn.
  if (static_cast<int>(reporters_lru_.size()) > max_tracked_nodes_) {
    RCLCPP_WARN_ONCE(get_logger(), "max_tracked_nodes (%d) reached; evicting least-recently-used reporters",
                     max_tracked_nodes_);
    reporters_.erase(reporters_lru_.front().source_id);
    reporters_lru_.pop_front();
  }
  return raw;
}

size_t LogBridgeNode::tracked_reporter_count() {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  return reporters_.size();
}

bool LogBridgeNode::map_level_to_severity(uint8_t log_level, uint8_t severity_floor, uint8_t * severity_out) {
  if (severity_out == nullptr) {
    return false;
  }
  using Fault = ros2_medkit_msgs::msg::Fault;
  if (log_level < severity_floor || log_level <= kLevelInfo) {
    return false;
  }
  switch (log_level) {
    case kLevelWarn:
      *severity_out = Fault::SEVERITY_WARN;
      return true;
    case kLevelError:
      *severity_out = Fault::SEVERITY_ERROR;
      return true;
    case kLevelFatal:
      *severity_out = Fault::SEVERITY_CRITICAL;
      return true;
    default:
      // Unknown level >= floor and above INFO: treat as ERROR rather than drop.
      *severity_out = Fault::SEVERITY_ERROR;
      return true;
  }
}

std::string LogBridgeNode::normalize_message(const std::string & message) {
  // Collapse anything that varies per-occurrence (numbers, hex, paths) so the
  // same logical message hashes to the same code. Keep only lowercase letters
  // and single spaces between word runs.
  std::string out;
  out.reserve(message.size());
  bool last_space = true;
  for (char c : message) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalpha(uc)) {
      out += static_cast<char>(std::tolower(uc));
      last_space = false;
    } else {
      // digits, punctuation, slashes, whitespace -> a single separator space
      if (!last_space) {
        out += ' ';
        last_space = true;
      }
    }
  }
  // trim trailing space
  if (!out.empty() && out.back() == ' ') {
    out.pop_back();
  }

  // Drop isolated single-letter tokens (e.g. "host A" / "host B" enumerations)
  // so an otherwise-identical message does not fragment into many codes.
  std::string filtered;
  filtered.reserve(out.size());
  size_t i = 0;
  while (i < out.size()) {
    size_t j = out.find(' ', i);
    if (j == std::string::npos) {
      j = out.size();
    }
    if (j - i > 1) {
      if (!filtered.empty()) {
        filtered += ' ';
      }
      filtered.append(out, i, j - i);
    }
    i = j + 1;
  }
  return filtered;
}

std::string LogBridgeNode::fnv1a_hex(const std::string & in) {
  // FNV-1a 32-bit, fixed spec: offset basis 2166136261, prime 16777619.
  uint32_t h = 2166136261u;
  for (char c : in) {
    h ^= static_cast<uint8_t>(c);
    h *= 16777619u;
  }
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08x", h);
  return std::string(buf);
}

std::string LogBridgeNode::to_upper_snake(const std::string & in, size_t max_len) {
  std::string result;
  result.reserve(in.size());
  bool last_sep = true;
  for (char c : in) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      result += static_cast<char>(std::toupper(uc));
      last_sep = false;
    } else if (!last_sep) {
      result += '_';
      last_sep = true;
    }
  }
  while (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  if (result.size() > max_len) {
    result.resize(max_len);
    while (!result.empty() && result.back() == '_') {
      result.pop_back();
    }
  }
  return result;
}

std::string LogBridgeNode::generate_fault_code(const std::string & source_id, const std::string & message) const {
  // Node part: fold the namespace into one token so /robot1/planner_server and
  // /robot2/planner_server do not collide. Leading '/' is dropped by
  // to_upper_snake (treated as a separator), '/'-separated segments join with '_'.
  std::string node_part = to_upper_snake(source_id, 48);

  // Stable 8-hex-char hash of the normalized message template. An all-punct /
  // all-digit / empty message normalizes to "" -> substitute a sentinel so such
  // entries do not all collapse onto one hash-of-"" code.
  std::string templ = normalize_message(message);
  if (templ.empty()) {
    templ = "_NOMSG";
  }
  const std::string hashpart = fnv1a_hex(templ);

  // Budget from the hash inward: the 8-hex hash and its '_' separators are
  // sacred (never truncated); trim the node part to fit 64 chars.
  // 64 - prefix - '_' - '_' - 8(hash)
  std::string code = code_prefix_;
  const size_t fixed = code.size() + 2 + hashpart.size();  // prefix + 2 separators + hash
  if (fixed >= 64) {
    // Prefix alone (plus hash) already at budget; keep prefix + hash only.
    return code + '_' + hashpart;
  }
  const size_t node_budget = 64 - fixed;
  if (node_part.size() > node_budget) {
    node_part.resize(node_budget);
    while (!node_part.empty() && node_part.back() == '_') {
      node_part.pop_back();
    }
  }

  if (!node_part.empty()) {
    code += '_';
    code += node_part;
  }
  code += '_';
  code += hashpart;
  return code;
}

}  // namespace ros2_medkit_log_bridge
