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

#include "ros2_medkit_fault_manager/snapshot_capture.hpp"
#include <atomic>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <set>
#include <thread>
#include <utility>

#include "ros2_medkit_fault_manager/time_utils.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// RAII guard that destroys a per-capture subscription and callback group
/// while holding the node-ops mutex. Creation of these entities is already
/// serialised by the same mutex; TSan confirmed a data race on the node's
/// internal rcutils_hash_map when two capture threads destroyed their
/// subscriptions concurrently after the create-side lock was released.
/// Locking the destroy-side too closes the window (same race class as
/// issue #375 in the gateway).
struct LockedSubscriptionGuard {
  std::mutex * mtx;
  rclcpp::GenericSubscription::SharedPtr subscription;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  // Explicit constructor instead of relying on aggregate initialization:
  // C++20 [dcl.init.aggr] disqualifies a class as an aggregate if it has
  // any user-declared constructors, including ``= delete`` ones (the rule
  // tightened from "user-provided" in C++17 to "user-declared" in C++20).
  // Lyrical's gcc 15 / libstdc++ enforces the C++20 wording even when
  // CMAKE_CXX_STANDARD is 17, so brace-init ``LockedSubscriptionGuard{
  // &mtx, sub, cg}`` would fail to find a matching constructor on Lyrical.
  // An explicit constructor sidesteps the aggregate-init rules entirely.
  LockedSubscriptionGuard(std::mutex * m, rclcpp::GenericSubscription::SharedPtr s, rclcpp::CallbackGroup::SharedPtr cg)
    : mtx(m), subscription(std::move(s)), callback_group(std::move(cg)) {
  }

  ~LockedSubscriptionGuard() {
    if (!subscription && !callback_group) {
      return;
    }
    std::lock_guard<std::mutex> lock(*mtx);
    // Drop subscription before callback group so any in-flight dispatch
    // finishes against a live group. Both destructors mutate the node
    // hash map.
    subscription.reset();
    callback_group.reset();
  }

  // Non-copyable, non-movable: the guard owns per-capture rcl state and
  // must destroy it exactly once, under the provided mutex.
  LockedSubscriptionGuard(const LockedSubscriptionGuard &) = delete;
  LockedSubscriptionGuard & operator=(const LockedSubscriptionGuard &) = delete;
  LockedSubscriptionGuard(LockedSubscriptionGuard &&) = delete;
  LockedSubscriptionGuard & operator=(LockedSubscriptionGuard &&) = delete;
};

}  // namespace

SnapshotCapture::SnapshotCapture(rclcpp::Node * node, FaultStorage * storage, const SnapshotConfig & config)
  : node_(node), storage_(storage), config_(config) {
  if (!node_) {
    throw std::invalid_argument("SnapshotCapture requires a valid node pointer");
  }
  if (!storage_) {
    throw std::invalid_argument("SnapshotCapture requires a valid storage pointer");
  }

  // Continue the sequence the store already holds; see capture_seq_.
  capture_seq_.store(storage_->get_max_capture_id());

  // Compile regex patterns for performance
  size_t failed_patterns = 0;
  for (const auto & [pattern, topics] : config_.patterns) {
    try {
      compiled_patterns_.emplace_back(std::regex(pattern), topics);
    } catch (const std::regex_error & e) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid regex pattern '%s' in snapshot config will be IGNORED: %s",
                   pattern.c_str(), e.what());
      ++failed_patterns;
    }
  }

  // Initialize background subscriptions if enabled
  if (config_.enabled && config_.background_capture) {
    init_background_subscriptions();
  }

  if (failed_patterns > 0) {
    RCLCPP_WARN(node_->get_logger(), "SnapshotCapture initialized with %zu/%zu patterns failed to compile",
                failed_patterns, config_.patterns.size());
  }

  RCLCPP_INFO(node_->get_logger(),
              "SnapshotCapture initialized (enabled=%s, background=%s, timeout=%.1fs, patterns=%zu)",
              config_.enabled ? "true" : "false", config_.background_capture ? "true" : "false", config_.timeout_sec,
              compiled_patterns_.size());
}

SnapshotCapture::~SnapshotCapture() {
  // Subscriptions are automatically cleaned up by shared_ptr
  background_subscriptions_.clear();
}

void SnapshotCapture::capture(const std::string & fault_code) {
  if (!config_.enabled) {
    RCLCPP_DEBUG(node_->get_logger(), "Snapshot capture disabled, skipping for fault '%s'", fault_code.c_str());
    return;
  }

  bool explicit_match = false;
  auto topics = resolve_topics(fault_code, explicit_match);
  // Zero-config fallback: nothing in the explicit config even mentioned this
  // code. A present-but-empty fault_specific/pattern entry is an explicit
  // per-fault opt-out (explicit_match == true) and must NOT fall through here.
  bool entity_scoped = false;
  if (topics.empty() && !explicit_match && config_.entity_default) {
    topics = resolve_entity_topics(fault_code);
    entity_scoped = !topics.empty();
  }
  if (topics.empty()) {
    // Unconfigured fault code: resolve_topics() yielded nothing, so capture returns early.
    // No freeze_frames row is written (we never persist an empty {} row) and no
    // subscriptions are created. A freeze-frame lookup for such a fault returns not-found
    // (get_freeze_frame() == nullopt); absence of a row means "no capture configured".
    RCLCPP_DEBUG(node_->get_logger(), "No topics configured for fault '%s'", fault_code.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Capturing snapshots for fault '%s' (%zu topics%s)", fault_code.c_str(),
              topics.size(), entity_scoped ? ", entity-default" : "");

  // Accumulate a compact dict of captured topic values alongside the per-topic snapshots.
  nlohmann::json freeze_frame = nlohmann::json::object();
  // One id for the whole capture, minted here so every row of this confirmation
  // shares it. Monotonic rather than a clock read: the rows are written seconds
  // apart under load and a wall clock is not guaranteed to move forward.
  const int64_t capture_id = ++capture_seq_;
  std::vector<SnapshotData> rows;
  size_t captured_count = 0;
  for (const auto & topic : topics) {
    bool success = false;
    // Entity-default topics are not known at construction, so the background
    // cache never holds them - always sample those on demand.
    if (config_.background_capture && !entity_scoped) {
      success = capture_topic_from_cache(fault_code, topic, freeze_frame, rows);
    } else {
      success = capture_topic_on_demand(fault_code, topic, freeze_frame, rows);
    }

    if (success) {
      ++captured_count;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Captured %zu/%zu snapshots for fault '%s'", captured_count, topics.size(),
              fault_code.c_str());

  // One write for the whole capture. Storing topic by topic let the per-fault cap
  // reject the tail of a capture, leaving a freeze frame with holes and nothing
  // saying which values were dropped.
  for (auto & row : rows) {
    row.capture_id = capture_id;
  }

  // capture() runs on a pool worker while clear_fault runs on the service thread,
  // and capture_topic_on_demand waits per silent topic, so an acknowledgement can
  // land mid-capture. Row-by-row writes used to be swept by that clear; one batch
  // written afterwards is not, and would resurrect readings the acknowledgement
  // had just promised were gone - on SQLite, across restarts. Where the operator
  // asked to retain snapshots there is no such promise, so the batch stands.
  // Absent is not cleared: a caller driving the capture directly still stores.
  // Mirrors rosbag_capture's wants_a_row.
  const auto fault = storage_->get_fault(fault_code);
  const bool acknowledged_mid_capture =
      fault.has_value() && fault->status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
  if (acknowledged_mid_capture && !storage_->retains_snapshots_on_clear()) {
    RCLCPP_DEBUG(node_->get_logger(), "Dropping capture for '%s' - acknowledged while it ran", fault_code.c_str());
  } else {
    storage_->store_snapshots(rows);
  }

  // Persist the compact freeze-frame keyed by fault_code (retained across clear_fault).
  // Only reached for faults with a configured capture set (unconfigured codes returned
  // early above and get no row). If nothing was publishing at confirmation time the row
  // holds an empty object, recording that a configured capture ran but sampled nothing.
  // An empty capture must never clobber an existing non-empty frame: a re-confirm while
  // the source publishers are down would otherwise replace the retained record with {}.
  if (captured_count == 0) {
    // Entity-default runs are best-effort: an empty one must not create a row
    // (absence still means "nothing configured captured anything") nor clobber
    // a retained frame from an earlier explicit-config capture.
    if (entity_scoped) {
      RCLCPP_DEBUG(node_->get_logger(), "Entity-default capture for fault '%s' sampled nothing; no row written",
                   fault_code.c_str());
      return;
    }
    auto existing = storage_->get_freeze_frame(fault_code);
    if (existing.has_value() && existing->data != "{}") {
      RCLCPP_WARN(node_->get_logger(), "Nothing captured for fault '%s'; keeping previously retained freeze-frame",
                  fault_code.c_str());
      return;
    }
  }

  FreezeFrameData frame;
  frame.fault_code = fault_code;
  frame.data = freeze_frame.dump();
  frame.captured_at_ns = get_wall_clock_ns();
  storage_->store_freeze_frame(frame);
}

std::vector<std::string> SnapshotCapture::resolve_topics(const std::string & fault_code, bool & explicit_match) const {
  explicit_match = true;

  // Priority 1: Exact match in fault_specific
  auto it = config_.fault_specific.find(fault_code);
  if (it != config_.fault_specific.end()) {
    RCLCPP_DEBUG(node_->get_logger(), "Using fault_specific topics for '%s'", fault_code.c_str());
    return it->second;
  }

  // Priority 2: First matching regex pattern
  for (const auto & [pattern_regex, topics] : compiled_patterns_) {
    if (std::regex_match(fault_code, pattern_regex)) {
      RCLCPP_DEBUG(node_->get_logger(), "Using pattern-matched topics for '%s'", fault_code.c_str());
      return topics;
    }
  }

  // Priority 3: Default topics (empty means "not configured", not an opt-out)
  if (!config_.default_topics.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Using default_topics for '%s'", fault_code.c_str());
    return config_.default_topics;
  }

  explicit_match = false;
  return {};
}

std::vector<std::string> SnapshotCapture::resolve_entity_topics(const std::string & fault_code) const {
  // Bound the on-demand sweep: each silent-but-published topic costs up to
  // timeout_sec, so a node with many declared topics must not stall a capture
  // pool worker indefinitely.
  static constexpr size_t kMaxEntityTopics = 16;

  std::set<std::string> topics;
  // Guarded like RosbagCapture::resolve_entity_topics: this runs on a capture
  // pool worker with no outer catch, and storage_->get_fault() can throw on
  // the sqlite backend. Any failure degrades to "no entity-default capture".
  try {
    auto fault = storage_->get_fault(fault_code);
    if (!fault || fault->reporting_sources.empty()) {
      return {};
    }

    // Only ROS node FQNs (e.g. "/ns/planner_server") participate. Plugin
    // entities report under bare SOVD ids (e.g. "tank_process"); treating one
    // as (name, ns="/") could match an unrelated root-namespace node of the
    // same name and freeze-frame its topics. Their zero-config frames come
    // from the gateway's EntityFreezeFrameCapture instead.
    std::set<std::pair<std::string, std::string>> wanted;
    for (const auto & source : fault->reporting_sources) {
      if (source.size() < 2 || source.front() != '/') {
        continue;
      }
      const auto slash = source.rfind('/');
      const std::string name = source.substr(slash + 1);
      const std::string ns = (slash == 0) ? "/" : source.substr(0, slash);
      if (!name.empty()) {
        wanted.emplace(name, ns);
      }
    }
    if (wanted.empty()) {
      return {};
    }

    // The entity's "own data" = topics it publishes. Every node publishes
    // /rosout and /parameter_events; those say nothing about the entity.
    static const std::set<std::string> kExcluded = {"/rosout", "/parameter_events"};

    auto published_by_wanted = [&wanted](const std::vector<rclcpp::TopicEndpointInfo> & eps) {
      for (const auto & ep : eps) {
        if (wanted.count({ep.node_name(), ep.node_namespace()})) {
          return true;
        }
      }
      return false;
    };
    for (const auto & [topic, types] : node_->get_topic_names_and_types()) {
      if (kExcluded.count(topic)) {
        continue;
      }
      if (published_by_wanted(node_->get_publishers_info_by_topic(topic))) {
        topics.insert(topic);
        if (topics.size() >= kMaxEntityTopics) {
          RCLCPP_WARN(node_->get_logger(),
                      "Entity-default capture for fault '%s' capped at %zu topics; remaining topics skipped",
                      fault_code.c_str(), kMaxEntityTopics);
          break;
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Entity-default topic resolution failed for fault '%s': %s", fault_code.c_str(),
                e.what());
    return {};
  }

  if (!topics.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Entity-default capture for fault '%s': %zu topic(s) from the reporting node(s)",
                fault_code.c_str(), topics.size());
  }
  return {topics.begin(), topics.end()};
}

bool SnapshotCapture::capture_topic_on_demand(const std::string & fault_code, const std::string & topic,
                                              nlohmann::json & freeze_frame, std::vector<SnapshotData> & rows) {
  // Get topic type
  std::string msg_type = get_topic_type(topic);
  if (msg_type.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Topic '%s' not found, skipping snapshot", topic.c_str());
    return false;
  }

  // Check if topic has publishers
  if (node_->count_publishers(topic) == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic '%s' has no publishers, skipping snapshot", topic.c_str());
    return false;
  }

  // Create one-shot subscription
  std::atomic<bool> received{false};
  rclcpp::SerializedMessage captured_msg;
  std::mutex msg_mutex;

  rclcpp::QoS qos = rclcpp::SensorDataQoS();  // Best effort for sensor data

  // Try to match publisher QoS
  auto pub_info = node_->get_publishers_info_by_topic(topic);
  if (!pub_info.empty()) {
    if (pub_info[0].qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      qos = rclcpp::QoS(10);  // Reliable
    }
  }

  // Create a local callback group for this capture operation (ensures clean executor lifecycle).
  // Pass false to prevent automatic association with the node's main executor -
  // we manually add this group to a local SingleThreadedExecutor below.
  //
  // node_ops_mutex_ serialises create AND destroy of these entities across
  // concurrent capture threads. rclcpp node internals (rcutils_hash_map) are
  // not thread-safe for concurrent entity creation OR destruction: TSan
  // initially caught the create race (fixed by locking the create side), then
  // caught an identical read-vs-write race on the destroy side (two capture
  // threads releasing their shared_ptr<GenericSubscription> concurrently, same
  // class as issue #375). LockedSubscriptionGuard above takes the same mutex
  // on destruction so both sides are now serialised.
  LockedSubscriptionGuard guard{&node_ops_mutex_, nullptr, nullptr};
  {
    std::lock_guard<std::mutex> lock(node_ops_mutex_);
    guard.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    try {
      // NOLINTNEXTLINE(performance-unnecessary-value-param)
      auto callback = [&received, &captured_msg, &msg_mutex](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        bool expected = false;
        if (received.compare_exchange_strong(expected, true)) {
          std::lock_guard<std::mutex> msg_lock(msg_mutex);
          captured_msg = *msg;
        }
      };

      // Use local callback group to avoid reentrancy with service callbacks
      rclcpp::SubscriptionOptions sub_options;
      sub_options.callback_group = guard.callback_group;

      guard.subscription = node_->create_generic_subscription(topic, msg_type, qos, callback, sub_options);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to create subscription for '%s': %s", topic.c_str(), e.what());
      // guard destructor unwinds callback_group under the lock before returning.
      return false;
    }
  }

  // Use a local executor with the local callback group (both destroyed together on exit).
  // The executor holds a raw pointer to the callback group; guard outlives it.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(guard.callback_group, node_->get_node_base_interface());

  // Wait for message with timeout
  const auto timeout = std::chrono::duration<double>(config_.timeout_sec);
  const auto start_time = std::chrono::steady_clock::now();

  while (!received.load()) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      RCLCPP_DEBUG(node_->get_logger(), "Timeout waiting for message on '%s'", topic.c_str());
      // Subscription and callback group destroyed on return, executor cleanup is automatic
      return false;
    }

    executor.spin_some(std::chrono::milliseconds(10));
  }

  // Deserialize to JSON
  try {
    std::lock_guard<std::mutex> lock(msg_mutex);

    // Check message size
    if (captured_msg.size() > config_.max_message_size) {
      RCLCPP_WARN(node_->get_logger(), "Message from '%s' too large (%zu > %zu), skipping", topic.c_str(),
                  captured_msg.size(), config_.max_message_size);
      return false;
    }

    ros2_medkit_serialization::JsonSerializer serializer;
    auto json_data = serializer.deserialize(msg_type, captured_msg);

    // Store snapshot (use wall clock time, not sim time, for proper timestamps)
    SnapshotData snapshot;
    snapshot.fault_code = fault_code;
    snapshot.topic = topic;
    snapshot.message_type = msg_type;
    snapshot.data = json_data.dump();
    snapshot.captured_at_ns = get_wall_clock_ns();

    rows.push_back(std::move(snapshot));

    // Record the value into the compact freeze-frame dict under the topic key.
    freeze_frame[topic] = std::move(json_data);

    RCLCPP_DEBUG(node_->get_logger(), "Captured snapshot from '%s' for fault '%s'", topic.c_str(), fault_code.c_str());
    return true;

  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_WARN(node_->get_logger(), "Unknown type '%s' for topic '%s': %s", msg_type.c_str(), topic.c_str(), e.what());
  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to deserialize message from '%s': %s", topic.c_str(), e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to process message from '%s': %s", topic.c_str(), e.what());
  }

  return false;
}

bool SnapshotCapture::capture_topic_from_cache(const std::string & fault_code, const std::string & topic,
                                               nlohmann::json & freeze_frame, std::vector<SnapshotData> & rows) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  auto it = message_cache_.find(topic);
  if (it == message_cache_.end() || it->second.data.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "No cached data for topic '%s'", topic.c_str());
    return false;
  }

  const auto & cached = it->second;

  // Store snapshot from cache
  SnapshotData snapshot;
  snapshot.fault_code = fault_code;
  snapshot.topic = topic;
  snapshot.message_type = cached.message_type;
  snapshot.data = cached.data;
  snapshot.captured_at_ns = cached.timestamp_ns;

  rows.push_back(std::move(snapshot));

  // Record the cached value into the compact freeze-frame dict. The cache holds a serialized
  // JSON string; parse it back so the frame nests structured values (fall back to the raw
  // string only if it somehow fails to parse).
  try {
    freeze_frame[topic] = nlohmann::json::parse(cached.data);
  } catch (const nlohmann::json::parse_error &) {
    freeze_frame[topic] = cached.data;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Captured snapshot from cache for '%s' (fault '%s')", topic.c_str(),
               fault_code.c_str());
  return true;
}

void SnapshotCapture::init_background_subscriptions() {
  auto topics = collect_all_configured_topics();
  if (topics.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No topics configured for background capture");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Initializing %zu background subscriptions for snapshot capture", topics.size());

  for (const auto & topic : topics) {
    std::string msg_type = get_topic_type(topic);
    if (msg_type.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Cannot determine type for topic '%s', skipping background subscription",
                  topic.c_str());
      continue;
    }

    try {
      rclcpp::QoS qos = rclcpp::SensorDataQoS();

      // NOLINTNEXTLINE(performance-unnecessary-value-param)
      auto callback = [this, topic, msg_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        try {
          // Check message size
          if (msg->size() > config_.max_message_size) {
            return;  // Skip oversized messages silently
          }

          ros2_medkit_serialization::JsonSerializer ser;
          auto json_data = ser.deserialize(msg_type, *msg);

          std::lock_guard<std::mutex> lock(cache_mutex_);
          auto & cached = message_cache_[topic];
          cached.topic = topic;
          cached.message_type = msg_type;
          cached.data = json_data.dump();
          // Use wall clock time, not sim time, for proper timestamps
          cached.timestamp_ns = get_wall_clock_ns();

        } catch (const std::exception & e) {
          RCLCPP_DEBUG(node_->get_logger(), "Failed to cache message from '%s': %s", topic.c_str(), e.what());
        }
      };

      auto subscription = node_->create_generic_subscription(topic, msg_type, qos, callback);
      background_subscriptions_.push_back(subscription);

      RCLCPP_DEBUG(node_->get_logger(), "Created background subscription for '%s'", topic.c_str());

    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to create background subscription for '%s': %s", topic.c_str(),
                  e.what());
    }
  }
}

std::vector<std::string> SnapshotCapture::collect_all_configured_topics() const {
  std::set<std::string> unique_topics;

  // Collect from fault_specific
  for (const auto & [fault_code, topics] : config_.fault_specific) {
    unique_topics.insert(topics.begin(), topics.end());
  }

  // Collect from patterns
  for (const auto & [pattern, topics] : config_.patterns) {
    unique_topics.insert(topics.begin(), topics.end());
  }

  // Collect from default_topics
  unique_topics.insert(config_.default_topics.begin(), config_.default_topics.end());

  return {unique_topics.begin(), unique_topics.end()};
}

std::string SnapshotCapture::get_topic_type(const std::string & topic) const {
  // node_->get_topic_names_and_types() throws if the rcl context is
  // invalidated mid-call (e.g. SIGINT fires between the check and the rcl
  // call). During shutdown this is expected and not actionable - treat it
  // as "no topic info available right now".
  try {
    auto topic_names_and_types = node_->get_topic_names_and_types();
    auto it = topic_names_and_types.find(topic);
    if (it != topic_names_and_types.end() && !it->second.empty()) {
      return it->second[0];
    }
  } catch (const std::runtime_error &) {
    // context invalid - caller handles empty return
  }
  return "";
}

}  // namespace ros2_medkit_fault_manager
