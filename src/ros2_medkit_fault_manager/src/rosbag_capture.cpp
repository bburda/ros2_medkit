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

#include "ros2_medkit_fault_manager/rosbag_capture.hpp"

#include <rcutils/types/uint8_array.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>

#include "ros2_medkit_fault_manager/time_utils.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Actionable install hint for an unavailable storage backend.
std::string storage_plugin_hint(const std::string & format) {
  if (format == "mcap") {
    const char * distro = std::getenv("ROS_DISTRO");
    const std::string d = (distro && *distro) ? distro : "$ROS_DISTRO";
    return "install ros-" + d + "-rosbag2-storage-mcap";
  }
  return "check the rosbag2 storage plugin installation";
}

/// Monotonic nanoseconds, for measuring how long something took.
///
/// Deliberately not the wall clock that timestamps messages: the wall clock can
/// step backwards (NTP, a hypervisor resyncing a VM), and a recording's duration
/// then comes out negative and is reported as zero. That is not hypothetical -
/// it was observed here as a 220 ms backwards step inside a 0.5 s post-fault
/// window, which made the finalise log a completion timestamp 220 ms earlier
/// than the confirmation that opened the recording.
int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// Seconds from @p started_at_steady_ns until now, never negative.
///
/// This is the span the RECORDING was open, not the span of its content. The two
/// differ for a bag that stayed quiet, and the recording span is the one reported:
/// "the black box covered these seconds and nothing was published" is the useful
/// statement for an operator, an empty content span would be indistinguishable from
/// a broken artifact, and only this definition is available to every path without
/// timestamping each message in the post-roll write path. A post-fault-only bag
/// therefore reports ~duration_after_sec whether or not anything arrived.
///
/// Zero is returned only when no start time was ever recorded - a defensive value,
/// not the way an empty bag reports itself.
double span_sec_since(int64_t started_at_steady_ns) {
  if (started_at_steady_ns <= 0) {
    return 0.0;
  }
  const double span = static_cast<double>(steady_now_ns() - started_at_steady_ns) / 1e9;
  return span > 0.0 ? span : 0.0;
}

/// Bound the probe reason so a verbose pluginlib error does not flood the log.
std::string truncate_reason(const std::string & reason, size_t max_len = 200) {
  if (reason.size() <= max_len) {
    return reason;
  }
  return reason.substr(0, max_len) + "...";
}

/// Custom deleter for rcutils_uint8_array_t that calls rcutils_uint8_array_fini
struct Uint8ArrayDeleter {
  void operator()(rcutils_uint8_array_t * array) const {
    if (array) {
      // Cleanup is best-effort - nothing meaningful to do if it fails during destruction
      [[maybe_unused]] rcutils_ret_t ret = rcutils_uint8_array_fini(array);
      delete array;
    }
  }
};

/// Create a properly-initialized SerializedBagMessage with RAII-safe memory management
/// @param topic Topic name for the message
/// @param timestamp_ns Timestamp in nanoseconds
/// @param src_msg Source serialized message to copy from
/// @param logger Logger for error reporting
/// @return Shared pointer to bag message, or nullptr on allocation failure
std::shared_ptr<rosbag2_storage::SerializedBagMessage> create_bag_message(const std::string & topic,
                                                                          int64_t timestamp_ns,
                                                                          const rcl_serialized_message_t & src_msg,
                                                                          const rclcpp::Logger & logger) {
  auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_msg->topic_name = topic;
  // rosbag2 API changed in Iron: Humble uses time_stamp, Iron+ uses recv_timestamp/send_timestamp
#ifdef ROSBAG2_USE_OLD_TIMESTAMP_FIELD
  bag_msg->time_stamp = timestamp_ns;
#else
  bag_msg->recv_timestamp = timestamp_ns;
  bag_msg->send_timestamp = timestamp_ns;
#endif

  // Create serialized_data with custom deleter that calls rcutils_uint8_array_fini
  auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t(), Uint8ArrayDeleter{});

  // Initialize with rcutils (RAII-safe allocation)
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t ret = rcutils_uint8_array_init(serialized_data.get(), src_msg.buffer_length, &allocator);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to allocate serialized message buffer (%zu bytes): %s", src_msg.buffer_length,
                 rcutils_get_error_string().str);
    rcutils_reset_error();
    return nullptr;
  }

  // Copy data
  memcpy(serialized_data->buffer, src_msg.buffer, src_msg.buffer_length);
  serialized_data->buffer_length = src_msg.buffer_length;

  bag_msg->serialized_data = serialized_data;
  return bag_msg;
}

}  // namespace

RosbagCapture::RosbagCapture(rclcpp::Node * node, FaultStorage * storage, const RosbagConfig & config,
                             const SnapshotConfig & snapshot_config, StorageProbeFn storage_probe)
  : node_(node), storage_(storage), config_(config), snapshot_config_(snapshot_config) {
  if (!node_) {
    throw std::invalid_argument("RosbagCapture requires a valid node pointer");
  }
  if (!storage_) {
    throw std::invalid_argument("RosbagCapture requires a valid storage pointer");
  }

  storage_probe_ = storage_probe ? std::move(storage_probe) : [this](const std::string & f) {
    return default_storage_probe(f);
  };

  if (!config_.enabled) {
    RCLCPP_INFO(node_->get_logger(), "RosbagCapture disabled");
    return;
  }

  // Resolve a usable storage backend without ever terminating the FaultManager:
  // an unavailable plugin (e.g. rosbag2_storage_mcap not installed) must degrade,
  // not crash. Unknown formats and a missing plugin fall back to sqlite3 (always
  // shipped with rosbag2); if no backend works, disable capture and keep running
  // (freeze-frame snapshots are independent of rosbag).
  if (config_.format != "sqlite3" && config_.format != "mcap") {
    RCLCPP_WARN(node_->get_logger(), "Unknown rosbag storage format '%s'; using 'sqlite3'", config_.format.c_str());
    config_.format = "sqlite3";
  }

  if (auto probe_err = storage_probe_(config_.format)) {
    const std::string reason = truncate_reason(*probe_err);
    if (config_.format == "sqlite3") {
      // The always-shipped baseline failed: an environment-level problem (broken
      // rosbag2 base install / disk / permissions), not a missing optional plugin.
      RCLCPP_WARN(node_->get_logger(),
                  "sqlite3 rosbag storage is unavailable (%s); the rosbag2 base install may be broken. "
                  "Black-box rosbag capture disabled (freeze-frame snapshots still work)",
                  reason.c_str());
      config_.enabled = false;
      return;
    }
    if (auto sqlite_err = storage_probe_("sqlite3")) {
      RCLCPP_WARN(node_->get_logger(),
                  "No usable rosbag storage backend: '%s' (%s) and sqlite3 (%s) both failed to load; black-box "
                  "rosbag capture disabled (freeze-frame snapshots still work)",
                  config_.format.c_str(), reason.c_str(), truncate_reason(*sqlite_err).c_str());
      config_.enabled = false;
      return;
    }
    // Explicitly-configured (non-default) format unavailable: name the fix so an
    // operator who chose e.g. mcap for Foxglove is not silently downgraded.
    RCLCPP_WARN(node_->get_logger(),
                "Rosbag storage format '%s' is unavailable (%s); %s. Falling back to 'sqlite3' for black-box "
                "capture (sqlite3 bags are not directly Foxglove-readable)",
                config_.format.c_str(), reason.c_str(), storage_plugin_hint(config_.format).c_str());
    config_.format = "sqlite3";
  }

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture initialized (duration=%.1fs, after=%.1fs, lazy_start=%s, format=%s)",
              config_.duration_sec, config_.duration_after_sec, config_.lazy_start ? "true" : "false",
              config_.format.c_str());

  // Start immediately if not lazy_start
  if (!config_.lazy_start) {
    start();
  }
}

RosbagCapture::~RosbagCapture() {
  stop();
}

void RosbagCapture::start() {
  if (!config_.enabled || running_.load()) {
    return;
  }

  init_subscriptions();
  running_.store(true);

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture started with %zu topic subscriptions", subscriptions_.size());
}

void RosbagCapture::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  // A post-roll in flight owns an open writer and a bag that has no metadata row
  // yet. Finalise it instead of just cancelling the timer: the faults of that
  // burst keep their black box, the writer is closed, and the recording state is
  // cleared so a confirmation after a restart opens its own bag rather than
  // attaching to a recording whose timer is gone.
  finalize_post_fault_recording();

  if (discovery_retry_timer_) {
    discovery_retry_timer_->cancel();
    {
      std::lock_guard<std::mutex> lock(node_ops_mutex_);
      discovery_retry_timer_.reset();
    }
  }

  // Clear subscriptions. Destroying them mutates the node just as creating them
  // does, so it takes the same lock.
  {
    std::lock_guard<std::mutex> lock(node_ops_mutex_);
    subscriptions_.clear();
  }
  subscribed_topics_.clear();

  // Clear buffer
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    message_buffer_.clear();
    buffer_bytes_ = 0;
  }

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture stopped");
}

bool RosbagCapture::is_running() const {
  return running_.load();
}

void RosbagCapture::on_fault_prefailed(const std::string & fault_code) {
  if (!config_.enabled) {
    return;
  }

  // Start buffer if lazy_start and not already running
  if (config_.lazy_start && !running_.load()) {
    RCLCPP_INFO(node_->get_logger(), "RosbagCapture starting on PREFAILED for fault '%s'", fault_code.c_str());
    start();
  }
}

bool RosbagCapture::attach_to_active_recording(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(post_fault_timer_mutex_);
  if (!recording_post_fault_.load()) {
    return false;
  }
  if (fault_code == current_fault_code_) {
    return true;
  }
  // Cap the attachment set: a flapping detector must not grow it without bound
  // between two post-roll windows. Past the cap the burst is already recorded,
  // only the per-fault lookup key is missing.
  if (attached_fault_codes_.size() >= kMaxAttachedFaults) {
    RCLCPP_WARN(node_->get_logger(), "Post-fault recording already covers %zu faults, not attaching '%s'",
                attached_fault_codes_.size(), fault_code.c_str());
    return true;
  }
  if (attached_fault_codes_.insert(fault_code).second) {
    RCLCPP_INFO(node_->get_logger(), "Fault '%s' confirmed during post-fault recording, attaching it to bag for '%s'",
                fault_code.c_str(), current_fault_code_.empty() ? "<cleared primary>" : current_fault_code_.c_str());
    widen_capture_filter_for(fault_code);
  }
  return true;
}

void RosbagCapture::widen_capture_filter_for(const std::string & fault_code) {
  // Entity mode scoped the in-flight writes to the first fault's topics; an
  // attached fault needs its own topics in the bag from the attach onwards, or
  // its row would serve a recording with none of its data. Union them in, and
  // degrade to writing everything when its scope cannot be resolved.
  if (config_.topics != "entity") {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(capture_topics_mutex_);
    if (active_capture_topics_.empty()) {
      return;  // already writing everything
    }
  }
  auto topics = compute_entity_topics(fault_code);
  std::lock_guard<std::mutex> lock(capture_topics_mutex_);
  if (active_capture_topics_.empty()) {
    return;
  }
  if (topics.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Entity scope unresolved for attached fault '%s'; widening the recording to all topics",
                fault_code.c_str());
    active_capture_topics_.clear();
    return;
  }
  active_capture_topics_.insert(topics.begin(), topics.end());
}

void RosbagCapture::on_fault_confirmed(const std::string & fault_code) {
  if (!config_.enabled || !running_.load()) {
    return;
  }

  // Faults arrive in bursts from one root cause, so a confirmation landing inside
  // an in-flight post-roll is exactly the correlated fault whose black box matters
  // most. It shares the recording window, so attach it to the running bag instead
  // of leaving it with no recording at all.
  if (attach_to_active_recording(fault_code)) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture: fault '%s' confirmed, flushing buffer to bag", fault_code.c_str());

  // In "entity" mode, narrow the broadly-buffered messages to the faulting node's
  // topics. Sets the filter used by flush_to_bag() and the post-fault write path.
  resolve_entity_topics(fault_code);

  // Flush buffer to bag
  auto flush = flush_to_bag(fault_code);
  std::string bag_path;
  switch (flush.status) {
    case FlushStatus::kOk:
      bag_path = std::move(flush.bag_path);
      break;

    case FlushStatus::kEmptyBuffer: {
      // The boundary case: this fault confirmed right after the previous post-fault
      // window finalised, so the ring buffer holds nothing - the earlier flush moved
      // the whole deque out and every message published during that window went
      // straight into its bag. The post-failure data is exactly what a burst's later
      // fault needs, so open a recording anyway and let the normal post-roll fill it.
      if (config_.duration_after_sec <= 0.0) {
        // No window to record into, and no history to write: nothing can be captured.
        RCLCPP_WARN(node_->get_logger(),
                    "No data buffered for fault '%s' and no post-fault window configured, no bag created",
                    fault_code.c_str());
        return;
      }
      auto opened = open_bag_writer(fault_code);
      if (!opened) {
        RCLCPP_WARN(node_->get_logger(), "Failed to create bag file for fault '%s'", fault_code.c_str());
        return;
      }
      bag_path = std::move(*opened);
      recording_started_at_ns_.store(steady_now_ns());
      RCLCPP_INFO(node_->get_logger(), "No pre-fault data buffered for fault '%s' - recording post-fault window only",
                  fault_code.c_str());
      break;
    }

    case FlushStatus::kIoError:
      // Never open a post-roll on top of an I/O failure: it would hold an unusable
      // writer open for the whole window and then store a row for a bag that the
      // failed flush already removed.
      RCLCPP_WARN(node_->get_logger(), "Failed to create bag file for fault '%s'", fault_code.c_str());
      return;
  }

  // If duration_after_sec > 0, continue recording
  if (config_.duration_after_sec > 0.0) {
    // Create timer for post-fault recording. The recording state is published
    // under the same lock attach_to_active_recording() takes, so a concurrent
    // confirmation either attaches to this bag or starts its own - never both.
    auto duration = std::chrono::duration<double>(config_.duration_after_sec);
    {
      std::lock_guard<std::mutex> lock(post_fault_timer_mutex_);
      current_fault_code_ = fault_code;
      current_bag_path_ = bag_path;
      attached_fault_codes_.clear();
      recording_post_fault_.store(true);
      arm_post_fault_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }

    RCLCPP_DEBUG(node_->get_logger(), "Recording %.1fs more after fault confirmation", config_.duration_after_sec);
  } else {
    // No post-fault recording, close writer and finalize immediately
    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      active_writer_.reset();
      created_topics_.clear();
    }

    size_t bag_size = calculate_bag_size(bag_path);

    RosbagFileInfo info;
    info.fault_code = fault_code;
    info.file_path = bag_path;
    info.format = config_.format;
    // The span the recording was open, not the configured window: a buffer that
    // held less than duration_sec of history would otherwise be advertised as a
    // full one. Exchanged without post_fault_timer_mutex_ here, unlike the finalize
    // path, and safe for a different reason: no post-roll state was published for
    // this recording, so no finalise can be running for it, and confirmations are
    // serialised against each other by the node-level rosbag mutex.
    info.duration_sec = span_sec_since(recording_started_at_ns_.exchange(0));
    info.size_bytes = bag_size;
    info.created_at_ns = get_wall_clock_ns();

    storage_->store_rosbag_file(info);
    enforce_storage_limits();

    RCLCPP_INFO(node_->get_logger(), "Bag file created: %s (%.2f MB)", bag_path.c_str(),
                static_cast<double>(bag_size) / (1024.0 * 1024.0));

    std::lock_guard<std::mutex> lock(capture_topics_mutex_);
    active_capture_topics_.clear();
  }
}

void RosbagCapture::on_fault_cleared(const std::string & fault_code) {
  if (!config_.enabled || !config_.auto_cleanup) {
    return;
  }

  // A fault of the in-flight burst has no row yet: drop it from the recording
  // state so the finalize never writes one. A leftover row for a cleared fault
  // would keep the shared bag referenced forever and break the burst's cleanup.
  {
    std::lock_guard<std::mutex> lock(post_fault_timer_mutex_);
    if (recording_post_fault_.load()) {
      attached_fault_codes_.erase(fault_code);
      if (fault_code == current_fault_code_) {
        current_fault_code_.clear();
      }
    }
  }

  // Delete the bag file for this fault
  if (storage_->delete_rosbag_file(fault_code)) {
    RCLCPP_INFO(node_->get_logger(), "Auto-cleanup: deleted bag file for fault '%s'", fault_code.c_str());
  }
}

void RosbagCapture::init_subscriptions() {
  // Broad modes ("all"/"auto"/"entity") capture whatever is on the graph, so the
  // subscribe set must keep growing as publishers appear after startup (dynamic
  // capture). Fixed modes (config/explicit/list) have a closed set known up front.
  dynamic_discovery_ = (config_.topics == "all" || config_.topics == "auto" || config_.topics == "entity");

  subscriptions_.clear();
  subscribed_topics_.clear();

  // Track topics that couldn't be subscribed yet (type not discoverable)
  std::vector<std::string> pending_topics;
  for (const auto & topic : resolve_topics()) {
    if (!try_subscribe_topic(topic)) {
      pending_topics.push_back(topic);
    }
  }

  // Run the discovery timer when there are type-pending topics OR we need to keep
  // picking up newly-appeared topics in a broad mode.
  if ((dynamic_discovery_ || !pending_topics.empty()) && !discovery_retry_timer_) {
    pending_topics_ = pending_topics;
    discovery_retry_count_ = 0;
    std::lock_guard<std::mutex> lock(node_ops_mutex_);
    discovery_retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      discovery_retry_callback();
    });
  }
}

bool RosbagCapture::try_subscribe_topic(const std::string & topic) {
  std::string msg_type = get_topic_type(topic);
  if (msg_type.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Cannot determine type for topic '%s', will retry", topic.c_str());
    return false;
  }

  // Cache the topic type
  {
    std::lock_guard<std::mutex> lock(topic_types_mutex_);
    topic_types_[topic] = msg_type;
  }

  try {
    rclcpp::QoS qos = config_.qos_match ? resolve_topic_qos(topic) : rclcpp::QoS(rclcpp::SensorDataQoS());

    auto callback = [this, topic, msg_type](const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
      message_callback(topic, msg_type, msg);
    };

    // create_generic_subscription mutates the node; this runs from start() and from
    // the executor's discovery timer, so it is serialised with the other node ops.
    rclcpp::GenericSubscription::SharedPtr subscription;
    {
      std::lock_guard<std::mutex> lock(node_ops_mutex_);
      subscription = node_->create_generic_subscription(topic, msg_type, qos, callback);
    }
    subscriptions_.push_back(subscription);
    subscribed_topics_.insert(topic);

    RCLCPP_INFO(node_->get_logger(), "Subscribed to '%s' (%s) for rosbag capture", topic.c_str(), msg_type.c_str());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create subscription for '%s': %s", topic.c_str(), e.what());
    return false;
  }
}

void RosbagCapture::discovery_retry_callback() {
  if (!running_.load()) {
    if (discovery_retry_timer_) {
      discovery_retry_timer_->cancel();
      std::lock_guard<std::mutex> lock(node_ops_mutex_);
      discovery_retry_timer_.reset();
    }
    return;
  }

  if (dynamic_discovery_) {
    // Re-resolve the broad set every tick and subscribe to anything new, so topics
    // whose publishers come up after startup are captured. Runs for the capture's
    // lifetime; already-subscribed topics are skipped cheaply.
    for (const auto & topic : resolve_topics()) {
      if (subscribed_topics_.count(topic) == 0) {
        try_subscribe_topic(topic);
      }
    }
    return;
  }

  // Fixed modes: bounded retry of the initially type-pending topics only.
  discovery_retry_count_++;
  constexpr int max_retries = 20;  // 10 seconds total (20 * 500ms)

  std::vector<std::string> still_pending;
  for (const auto & topic : pending_topics_) {
    if (!try_subscribe_topic(topic)) {
      still_pending.push_back(topic);
    }
  }
  pending_topics_ = still_pending;

  if (pending_topics_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "All topics subscribed after %d retries", discovery_retry_count_);
    discovery_retry_timer_->cancel();
    {
      std::lock_guard<std::mutex> lock(node_ops_mutex_);
      discovery_retry_timer_.reset();
    }
  } else if (discovery_retry_count_ >= max_retries) {
    RCLCPP_WARN(node_->get_logger(), "Giving up on %zu topics after %d retries: ", pending_topics_.size(), max_retries);
    for (const auto & topic : pending_topics_) {
      RCLCPP_WARN(node_->get_logger(), "  - %s", topic.c_str());
    }
    discovery_retry_timer_->cancel();
    {
      std::lock_guard<std::mutex> lock(node_ops_mutex_);
      discovery_retry_timer_.reset();
    }
    pending_topics_.clear();
  }
}

void RosbagCapture::message_callback(const std::string & topic, const std::string & msg_type,
                                     const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
  if (!running_.load()) {
    return;
  }

  // Use wall clock time, not sim time, for proper timestamps
  int64_t timestamp_ns = get_wall_clock_ns();

  // During post-fault recording, write directly to bag (no buffering)
  if (recording_post_fault_.load()) {
    // Entity mode: keep the post-fault stream scoped to the same topics as the flush
    if (!should_capture_topic(topic)) {
      return;
    }
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    if (active_writer_) {
      try {
        // Create topic if not already created
        if (created_topics_.find(topic) == created_topics_.end()) {
          rosbag2_storage::TopicMetadata topic_meta;
          topic_meta.name = topic;
          topic_meta.type = msg_type;
          topic_meta.serialization_format = "cdr";
          active_writer_->create_topic(topic_meta);
          created_topics_.insert(topic);
        }

        auto bag_msg = create_bag_message(topic, timestamp_ns, msg->get_rcl_serialized_message(), node_->get_logger());
        if (bag_msg) {
          active_writer_->write(bag_msg);
        }
        // Memory is automatically cleaned up by RAII when bag_msg goes out of scope
      } catch (const std::exception & e) {
        rclcpp::Clock clock(*node_->get_clock());
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000, "Failed to write post-fault message: %s", e.what());
      }
    }
    return;  // Don't buffer during post-fault recording
  }

  // Normal buffering mode
  BufferedMessage buffered;
  buffered.topic = topic;
  buffered.message_type = msg_type;
  buffered.serialized_data = std::make_shared<rclcpp::SerializedMessage>(*msg);
  buffered.timestamp_ns = timestamp_ns;
  const size_t msg_bytes = buffered.serialized_data->size();

  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_bytes_ += msg_bytes;
    message_buffer_.push_back(std::move(buffered));
  }

  // Prune old messages
  prune_buffer();
}

void RosbagCapture::prune_buffer() {
  // Don't prune during post-fault recording - we need all messages for the final flush
  if (recording_post_fault_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(buffer_mutex_);

  if (message_buffer_.empty()) {
    return;
  }

  // Use wall clock time, not sim time, for consistent buffer pruning
  int64_t now_ns = get_wall_clock_ns();
  int64_t duration_ns = static_cast<int64_t>(config_.duration_sec * 1e9);
  int64_t cutoff_ns = now_ns - duration_ns;

  // Remove messages older than cutoff
  while (!message_buffer_.empty() && message_buffer_.front().timestamp_ns < cutoff_ns) {
    buffer_bytes_ -= message_buffer_.front().serialized_data->size();
    message_buffer_.pop_front();
  }

  // RAM cap: drop the oldest messages once the buffer exceeds max_buffer_mb, so a
  // broad subscribe set on a busy robot cannot grow the ring buffer without bound.
  const size_t cap_bytes = config_.max_buffer_mb * 1024UL * 1024UL;
  while (message_buffer_.size() > 1 && buffer_bytes_ > cap_bytes) {
    buffer_bytes_ -= message_buffer_.front().serialized_data->size();
    message_buffer_.pop_front();
  }
}

bool RosbagCapture::is_high_bandwidth_topic(const std::string & topic) {
  // Segment-anchored match (leading '/') so '/camera/image' and '/points' are caught
  // but low-bandwidth lookalikes that merely contain the word - /waypoints,
  // /setpoints, /keypoints, /image_quality on another node - are not.
  static const std::array<const char *, 4> kSegments{"/image", "/points", "/depth", "/compressed"};
  for (const char * seg : kSegments) {
    if (topic.find(seg) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> RosbagCapture::resolve_topics() const {
  std::set<std::string> topics_set;

  if (config_.topics == "config") {
    // Reuse JSON snapshot config topics
    for (const auto & topic : snapshot_config_.default_topics) {
      topics_set.insert(topic);
    }
    for (const auto & [code, topics_vec] : snapshot_config_.fault_specific) {
      for (const auto & topic : topics_vec) {
        topics_set.insert(topic);
      }
    }
    for (const auto & [pattern, topics_vec] : snapshot_config_.patterns) {
      for (const auto & topic : topics_vec) {
        topics_set.insert(topic);
      }
    }
  } else if (config_.topics == "all" || config_.topics == "auto" || config_.topics == "entity") {
    // Broad discovery for the subscribe set ("auto" is an alias for "all"). "entity"
    // also subscribes broadly for pre-roll and narrows to the faulting node's topics
    // only at flush time. Skip gracefully if the context is invalidated mid-call.
    try {
      auto topic_names_and_types = node_->get_topic_names_and_types();
      for (const auto & [topic, types] : topic_names_and_types) {
        // Skip internal ROS topics
        if (topic.find("/parameter_events") != std::string::npos || topic.find("/rosout") != std::string::npos) {
          continue;
        }
        // Skip high-bandwidth sensor topics in broad modes to bound memory
        if (config_.exclude_sensor_topics && is_high_bandwidth_topic(topic)) {
          continue;
        }
        topics_set.insert(topic);
      }
    } catch (const std::runtime_error &) {
      // context invalid during shutdown - no topics to add
    }
  } else if (config_.topics == "explicit") {
    // Explicit mode: use only include_topics, no topic derivation
    // Topics will be populated from include_topics below
  } else {
    // Comma-separated list of topics
    std::istringstream iss(config_.topics);
    std::string topic;
    while (std::getline(iss, topic, ',')) {
      // Trim whitespace
      topic.erase(0, topic.find_first_not_of(" \t"));
      topic.erase(topic.find_last_not_of(" \t") + 1);
      if (!topic.empty()) {
        topics_set.insert(topic);
      }
    }
  }

  // Add include_topics
  for (const auto & topic : config_.include_topics) {
    topics_set.insert(topic);
  }

  // Remove exclude_topics
  for (const auto & topic : config_.exclude_topics) {
    topics_set.erase(topic);
  }

  return {topics_set.begin(), topics_set.end()};
}

rclcpp::QoS RosbagCapture::resolve_topic_qos(const std::string & topic) const {
  // QoS is resolved once, at subscribe time, from the publishers present then. It is
  // not re-resolved later: if a best-effort publisher appears afterwards on a topic
  // already subscribed as reliable, that publisher is QoS-incompatible and the
  // subscription receives nothing from it. Acceptable for black-box capture, where
  // the publisher set is stable by the time a fault occurs.
  std::vector<rclcpp::TopicEndpointInfo> pubs;
  try {
    pubs = node_->get_publishers_info_by_topic(topic);
  } catch (const std::exception &) {
    return rclcpp::SensorDataQoS();
  }
  if (pubs.empty()) {
    // No known publisher yet - best-effort default. Not upgraded once subscribed,
    // but a topic discovered via get_topic_names_and_types already has a publisher,
    // so this branch is a rare startup race rather than the steady state.
    return rclcpp::SensorDataQoS();
  }

  // Build a subscriber QoS compatible with every publisher while staying as faithful
  // as the offers allow: reliable only if ALL publishers offer reliable, transient-local
  // only if ALL offer it (otherwise the sub would be incompatible and never connect).
  bool all_reliable = true;
  bool all_transient_local = true;
  size_t depth = 10;
  for (const auto & p : pubs) {
    const auto & pq = p.qos_profile();
    if (pq.reliability() != rclcpp::ReliabilityPolicy::Reliable) {
      all_reliable = false;
    }
    if (pq.durability() != rclcpp::DurabilityPolicy::TransientLocal) {
      all_transient_local = false;
    }
    depth = std::max(depth, std::min<size_t>(pq.depth(), 100));
  }

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliability(all_reliable ? rclcpp::ReliabilityPolicy::Reliable : rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(all_transient_local ? rclcpp::DurabilityPolicy::TransientLocal : rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

std::set<std::string> RosbagCapture::compute_entity_topics(const std::string & fault_code) {
  std::set<std::string> topics;

  // The whole resolution is guarded: this runs on a detached capture thread with
  // no outer catch, and storage_->get_fault() can throw on the sqlite backend
  // (e.g. SQLITE_BUSY). A throw here would terminate the process - the exact crash
  // the crash-safety work removed - so any failure degrades to "write everything".
  try {
    auto fault = storage_->get_fault(fault_code);
    if (fault && !fault->reporting_sources.empty()) {
      // reporting_sources hold the reporting node's FQN (e.g. "/planner_server").
      // Split each into (name, namespace) to match against topic endpoints.
      std::set<std::pair<std::string, std::string>> wanted;
      for (const auto & source : fault->reporting_sources) {
        std::string ns = "/";
        std::string name = source;
        const auto slash = source.rfind('/');
        if (slash != std::string::npos) {
          name = source.substr(slash + 1);
          ns = (slash == 0) ? "/" : source.substr(0, slash);
        }
        if (!name.empty()) {
          wanted.emplace(name, ns);
        }
      }

      // rclcpp does not expose per-node topic listing, so scan each topic's
      // endpoints and keep the topics a wanted node publishes or subscribes to.
      auto owned_by_wanted = [&wanted](const std::vector<rclcpp::TopicEndpointInfo> & eps) {
        for (const auto & ep : eps) {
          if (wanted.count({ep.node_name(), ep.node_namespace()})) {
            return true;
          }
        }
        return false;
      };
      for (const auto & [topic, types] : node_->get_topic_names_and_types()) {
        if (owned_by_wanted(node_->get_publishers_info_by_topic(topic)) ||
            owned_by_wanted(node_->get_subscriptions_info_by_topic(topic))) {
          topics.insert(topic);
        }
      }

      if (!topics.empty()) {
        // Always-on context that makes a scoped bag useful for replay.
        topics.insert("/tf");
        topics.insert("/tf_static");
      }

      // Intersect with the actually-subscribed set so the filter never names a
      // topic that was never buffered (an excluded/sensor topic, or one with no
      // publisher) - which would otherwise be silently absent from the bag.
      const auto subscribed_vec = resolve_topics();
      const std::set<std::string> subscribed(subscribed_vec.begin(), subscribed_vec.end());
      std::set<std::string> scoped;
      for (const auto & t : topics) {
        if (subscribed.count(t)) {
          scoped.insert(t);
        }
      }
      topics = std::move(scoped);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Entity scope resolution failed for fault '%s' (%s); writing full buffer",
                fault_code.c_str(), e.what());
    topics.clear();
  }

  return topics;
}

void RosbagCapture::resolve_entity_topics(const std::string & fault_code) {
  std::set<std::string> topics;

  if (config_.topics == "entity") {
    topics = compute_entity_topics(fault_code);

    if (!topics.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Entity scope for fault '%s': %zu topic(s) from the faulting node(s) + /tf",
                  fault_code.c_str(), topics.size());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Entity scope unresolved for fault '%s' (source not a live node, or no buffered topics); "
                  "writing the full buffer",
                  fault_code.c_str());
    }
  }

  std::lock_guard<std::mutex> lock(capture_topics_mutex_);
  active_capture_topics_ = std::move(topics);
}

bool RosbagCapture::should_capture_topic(const std::string & topic) const {
  std::lock_guard<std::mutex> lock(capture_topics_mutex_);
  // Empty filter = manual modes, or entity resolution found nothing: write everything.
  if (active_capture_topics_.empty()) {
    return true;
  }
  return active_capture_topics_.count(topic) > 0;
}

std::string RosbagCapture::get_topic_type(const std::string & topic) const {
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

void RosbagCapture::discard_active_writer(const std::string & bag_path) {
  {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    active_writer_.reset();
    created_topics_.clear();
  }
  std::error_code ec;
  std::filesystem::remove_all(bag_path, ec);
}

std::optional<std::string> RosbagCapture::open_bag_writer(const std::string & fault_code) {
  std::string bag_path = generate_bag_path(fault_code);

  try {
    // Create parent directory if needed
    std::filesystem::path bag_dir(bag_path);
    if (!bag_dir.parent_path().empty()) {
      std::filesystem::create_directories(bag_dir.parent_path());
    }

    // Create writer and store as member for post-fault recording
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    active_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    created_topics_.clear();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = config_.format;
    storage_options.max_bagfile_size = config_.max_bag_size_mb * 1024 * 1024;

    active_writer_->open(storage_options);
    return bag_path;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open bag file '%s': %s", bag_path.c_str(), e.what());
    discard_active_writer(bag_path);
    return std::nullopt;
  }
}

RosbagCapture::FlushResult RosbagCapture::flush_to_bag(const std::string & fault_code) {
  // Copy buffer under lock, then release to avoid holding mutex during IO
  std::deque<BufferedMessage> messages_to_write;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (message_buffer_.empty()) {
      // Not a failure. Right after a post-fault window closes the buffer is empty
      // by construction, and the caller can still serve the fault with a
      // post-fault-only recording - so the distinction from an I/O error matters.
      return {FlushStatus::kEmptyBuffer, ""};
    }
    messages_to_write = std::move(message_buffer_);
    message_buffer_.clear();
    buffer_bytes_ = 0;
  }

  auto opened = open_bag_writer(fault_code);
  if (!opened) {
    return {FlushStatus::kIoError, ""};
  }
  const std::string bag_path = *opened;

  try {
    // Snapshot the entity filter once (empty = write everything) so the flush loop
    // does not take capture_topics_mutex_ for every buffered message.
    std::set<std::string> capture_filter;
    {
      std::lock_guard<std::mutex> lock(capture_topics_mutex_);
      capture_filter = active_capture_topics_;
    }
    const bool entity_filtered = !capture_filter.empty();

    // Write messages (no buffer_mutex_ held, writer_mutex_ only for brief access)
    size_t msg_count = 0;
    int64_t first_written_ns = 0;
    for (const auto & msg : messages_to_write) {
      // Entity mode: write only the faulting node's topics (+ /tf). No-op otherwise.
      if (entity_filtered && capture_filter.count(msg.topic) == 0) {
        continue;
      }

      std::lock_guard<std::mutex> wlock(writer_mutex_);
      if (!active_writer_) {
        break;
      }

      // Create topic if not already created
      if (created_topics_.find(msg.topic) == created_topics_.end()) {
        rosbag2_storage::TopicMetadata topic_meta;
        topic_meta.name = msg.topic;
        topic_meta.type = msg.message_type;
        topic_meta.serialization_format = "cdr";
        active_writer_->create_topic(topic_meta);
        created_topics_.insert(msg.topic);
      }

      auto bag_msg = create_bag_message(msg.topic, msg.timestamp_ns, msg.serialized_data->get_rcl_serialized_message(),
                                        node_->get_logger());
      if (bag_msg) {
        active_writer_->write(bag_msg);
        if (msg_count == 0) {
          first_written_ns = msg.timestamp_ns;
        }
        ++msg_count;
      }
      // Memory is automatically cleaned up by RAII when bag_msg goes out of scope
    }

    // The recording starts at its oldest written message, not at "now minus the
    // configured pre-fault window": a buffer that never filled, or an entity filter
    // that matched nothing, must not make the stored row claim history it lacks.
    // That start is a wall-clock instant (message timestamps are wall-clock), so
    // convert the history it represents into a monotonic start once, here, and
    // measure forward from that - a later clock step cannot corrupt an offset
    // already taken.
    const int64_t history_ns = msg_count > 0 ? std::max<int64_t>(0, get_wall_clock_ns() - first_written_ns) : 0;
    recording_started_at_ns_.store(steady_now_ns() - history_ns);

    RCLCPP_DEBUG(node_->get_logger(), "Flushed %zu messages to bag: %s", msg_count, bag_path.c_str());

    // Note: Writer is NOT closed here - it stays open for post-fault recording
    // It will be closed in post_fault_timer_callback()

    return {FlushStatus::kOk, bag_path};

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to write bag file '%s': %s", bag_path.c_str(), e.what());
    discard_active_writer(bag_path);
    return {FlushStatus::kIoError, ""};
  }
}

std::string RosbagCapture::generate_bag_path(const std::string & fault_code) const {
  std::string base_path;

  if (config_.storage_path.empty()) {
    // Use system temp directory
    base_path = std::filesystem::temp_directory_path().string();
  } else {
    base_path = config_.storage_path;
  }

  // Create unique name with timestamp
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  std::ostringstream oss;
  oss << base_path << "/fault_" << fault_code << "_" << timestamp;

  return oss.str();
}

size_t RosbagCapture::calculate_bag_size(const std::string & bag_path) const {
  size_t total_size = 0;

  try {
    if (std::filesystem::is_directory(bag_path)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(bag_path)) {
        if (entry.is_regular_file()) {
          total_size += entry.file_size();
        }
      }
    } else if (std::filesystem::is_regular_file(bag_path)) {
      total_size = std::filesystem::file_size(bag_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to calculate bag size for '%s': %s", bag_path.c_str(), e.what());
  }

  return total_size;
}

std::vector<std::string> RosbagCapture::evict_bags_over_quota(FaultStorage * storage, size_t max_bytes) {
  size_t current_bytes = storage->get_total_rosbag_storage_bytes();
  if (current_bytes <= max_bytes) {
    return {};
  }

  // get_all_rosbag_files() is one row per fault, and a burst of correlated faults
  // shares one recording. Group the rows back into bags first: deleting a single
  // row of a shared bag leaves the directory on disk (a sibling still points at
  // it), so freeing its bytes per row would drop the running total below the real
  // one and stop the eviction while the quota is still blown.
  std::vector<std::string> paths_oldest_first;
  std::unordered_map<std::string, std::vector<std::string>> codes_by_path;
  std::unordered_map<std::string, size_t> bytes_by_path;
  for (const auto & bag : storage->get_all_rosbag_files()) {
    auto & codes = codes_by_path[bag.file_path];
    if (codes.empty()) {
      paths_oldest_first.push_back(bag.file_path);
    }
    codes.push_back(bag.fault_code);
    // Mirror the storage accounting, which takes the largest row per path.
    auto & bytes = bytes_by_path[bag.file_path];
    bytes = std::max(bytes, bag.size_bytes);
  }

  std::vector<std::string> evicted;
  for (const auto & path : paths_oldest_first) {
    if (current_bytes <= max_bytes) {
      break;
    }

    // One call per bag: the SQLite backend removes the whole burst's rows in a
    // single transaction and unlinks the directory after the commit, so a crash
    // mid-eviction cannot leave rows pointing at a removed bag.
    storage->delete_rosbag_files(codes_by_path[path]);
    // Saturate rather than wrap: the quota must never be satisfied by underflow.
    current_bytes -= std::min(current_bytes, bytes_by_path[path]);
    evicted.push_back(path);
  }

  return evicted;
}

void RosbagCapture::enforce_storage_limits() {
  for (const auto & path : evict_bags_over_quota(storage_, config_.max_total_storage_mb * 1024 * 1024)) {
    RCLCPP_INFO(node_->get_logger(), "Deleted old bag '%s' to enforce storage limit", path.c_str());
  }
}

void RosbagCapture::arm_post_fault_timer(std::chrono::nanoseconds period) {
  // Re-arm rather than replace. rcl_timer_reset() restarts the period from now and
  // clears the cancelled flag, so one timer serves every recording and no timer is
  // ever destroyed while another is being created. See the header for why the
  // destruction side could not simply be locked instead.
  if (post_fault_timer_) {
    post_fault_timer_->reset();
    return;
  }

  // The one creation this class performs for this timer, serialised against the
  // other node-mutating calls (the discovery timer, the capture subscriptions).
  std::lock_guard<std::mutex> lock(node_ops_mutex_);
  post_fault_timer_ = node_->create_wall_timer(period, [this]() {
    post_fault_timer_callback();
  });
}

void RosbagCapture::post_fault_timer_callback() {
  finalize_post_fault_recording();
}

void RosbagCapture::finalize_post_fault_recording() {
  // Cancel timer (one-shot) and stop post-fault recording (no more direct writes
  // to bag). Clearing the guard under the lock is what makes the attach decision
  // race-free: a confirmation from now on opens its own bag rather than joining
  // a recording that is already being finalised. Testing the guard under the same
  // lock also settles a timer firing concurrently with stop() - the loser sees the
  // recording already claimed and returns.
  std::set<std::string> attached;
  std::string fault_code;
  std::string bag_path;
  int64_t started_at_ns = 0;
  std::unique_ptr<rosbag2_cpp::Writer> closing_writer;
  {
    std::lock_guard<std::mutex> lock(post_fault_timer_mutex_);
    if (!recording_post_fault_.load()) {
      return;
    }
    if (post_fault_timer_) {
      // Cancelled, not released: the timer outlives the recording and is re-armed
      // by the next one. See arm_post_fault_timer().
      post_fault_timer_->cancel();
    }
    recording_post_fault_.store(false);
    attached.swap(attached_fault_codes_);
    fault_code.swap(current_fault_code_);
    bag_path.swap(current_bag_path_);
    // Taken here, under the lock that clears the guard: a confirmation racing this
    // finalise opens its own recording the moment the guard drops, and would
    // otherwise overwrite the start time before this bag's row is built.
    started_at_ns = recording_started_at_ns_.exchange(0);
    // The writer has to change hands under that same lock, for the same reason.
    // Clearing the guard is precisely what invites a boundary confirmation in, and
    // it opens ITS writer into active_writer_; if this finalise were still to reach
    // for writer_mutex_ afterwards it would destroy that writer, leaving the new
    // recording to write through a null pointer - every message silently dropped -
    // and still store a row for the empty bag it produced. Which is #574 again,
    // now as a race and with a row that lies about it.
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    closing_writer = std::move(active_writer_);
    created_topics_.clear();
  }

  // Close outside both locks. Flushing the bag and writing metadata.yaml is real
  // I/O, and the writer is exclusively ours now, so nothing is gained by holding
  // the post-roll state or the write path hostage while it finishes. It must,
  // however, close BEFORE the size is measured below.
  closing_writer.reset();

  // Calculate final size and store metadata
  size_t bag_size = calculate_bag_size(bag_path);

  RosbagFileInfo info;
  info.file_path = bag_path;
  info.format = config_.format;
  // The real span of the recording, not the configured pre+post window. A
  // post-fault-only bag reports roughly duration_after_sec, and a bag flushed from
  // a partly-filled buffer reports what it holds.
  info.duration_sec = span_sec_since(started_at_ns);
  info.size_bytes = bag_size;
  info.created_at_ns = get_wall_clock_ns();

  // One recording, one row per fault it covers: the correlated faults that
  // confirmed inside this window each need their own lookup key to serve it.
  // A fault cleared during the post-roll was dropped from the recording state
  // (auto-cleanup) and gets no row.
  std::vector<RosbagFileInfo> rows;
  if (!fault_code.empty()) {
    info.fault_code = fault_code;
    rows.push_back(info);
  }
  for (const auto & code : attached) {
    info.fault_code = code;
    rows.push_back(info);
  }

  if (rows.empty()) {
    // Every fault of the burst was cleared while the post-roll ran; nothing
    // references the bag, so it goes the way auto-cleanup would have taken it.
    std::error_code ec;
    std::filesystem::remove_all(bag_path, ec);
    RCLCPP_INFO(node_->get_logger(), "Bag file discarded (all its faults cleared during post-roll): %s",
                bag_path.c_str());
  } else {
    // Reached from noexcept destructors (~RosbagCapture -> stop()): a busy/full
    // metadata store during shutdown must degrade to a warning, not terminate.
    try {
      storage_->store_rosbag_files(rows);
      enforce_storage_limits();
      RCLCPP_INFO(node_->get_logger(), "Bag file completed: %s (%.2f MB, %.1fs, %zu attached fault(s))",
                  bag_path.c_str(), static_cast<double>(bag_size) / (1024.0 * 1024.0), info.duration_sec,
                  attached.size());
    } catch (const std::exception & e) {
      // Without a row nothing can ever reach this bag: retrieval is keyed by fault
      // code, and quota accounting enumerates rows, so a kept directory would sit
      // on disk unreachable and uncounted, and the next full storage would not even
      // be able to evict it. Discard it for the same reason the all-cleared branch
      // above does.
      std::error_code ec;
      std::filesystem::remove_all(bag_path, ec);
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to store rosbag metadata for '%s' (%s); discarded the bag, nothing could reference it",
                  bag_path.c_str(), e.what());
    }
  }

  {
    std::lock_guard<std::mutex> lock(capture_topics_mutex_);
    active_capture_topics_.clear();
  }
}

std::optional<std::string> RosbagCapture::default_storage_probe(const std::string & format) const {
  // Probe the plugin by opening a throwaway bag. Returns the failure reason (never
  // throws) when the plugin is missing or unusable, so the caller can fall back or
  // self-disable instead of crashing.
  const std::string test_path =
      std::filesystem::temp_directory_path().string() + "/.rosbag_format_test_" + std::to_string(getpid());

  std::optional<std::string> error;
  try {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions opts;
    opts.uri = test_path;
    opts.storage_id = format;
    writer.open(opts);
  } catch (const std::exception & e) {
    // Keep the reason (plugin missing vs. I/O / permission) for the caller's log.
    error = e.what();
  }

  std::error_code ec;
  std::filesystem::remove_all(test_path, ec);
  return error;
}

}  // namespace ros2_medkit_fault_manager
