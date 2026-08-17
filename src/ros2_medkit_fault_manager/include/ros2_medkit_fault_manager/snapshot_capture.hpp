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

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// Configuration for rosbag recording (optional time-window capture)
struct RosbagConfig {
  /// Whether rosbag recording is enabled (opt-in)
  bool enabled{false};

  /// Duration in seconds to buffer before fault confirmation
  double duration_sec{5.0};

  /// Duration in seconds to continue recording after fault confirmation
  double duration_after_sec{1.0};

  /// Topic selection mode (default "entity"):
  ///   "entity"     - subscribe broadly for pre-roll, but on confirm write only the
  ///                  faulting entity's topics (resolved from the fault's reporting
  ///                  source node) plus always-on context (/tf, /tf_static)
  ///   "config"     - reuse JSON snapshot config topics
  ///   "all"/"auto" - every discovered topic
  ///   "explicit"   - only include_topics
  ///   "<a,b,c>"    - comma-separated topic list
  std::string topics{"entity"};

  /// Additional topics to include (added to resolved list)
  std::vector<std::string> include_topics;

  /// Topics to exclude from recording
  std::vector<std::string> exclude_topics;

  /// In broad modes ("all"/"auto"/"entity"), skip high-bandwidth sensor topics
  /// (image/points/depth/compressed) to bound memory; include_topics re-adds them.
  bool exclude_sensor_topics{true};

  /// If true, start ring buffer only when fault enters PREFAILED state
  /// If false (default), ring buffer runs continuously from startup
  bool lazy_start{false};

  /// Storage format: "mcap" (default; what Foxglove and Lichtblick open directly)
  /// or "sqlite3" (also what an unknown format string lands on). Neither is
  /// privileged over the other: whichever is configured, an unavailable plugin
  /// falls back automatically to the other one, and capture disables itself
  /// only if neither loads. Both plugins are runtime dependencies of this
  /// package.
  std::string format{"mcap"};

  /// Path to store bag files (empty = system temp directory)
  std::string storage_path;

  /// Maximum size of a single bag file in MB
  size_t max_bag_size_mb{50};

  /// Maximum total storage for all bag files in MB
  size_t max_total_storage_mb{500};

  /// Cap on the in-memory ring buffer in MB; oldest messages drop past it
  size_t max_buffer_mb{256};

  /// Subscribe with each topic's publisher-offered QoS for faithful capture
  /// (reliable/transient-local where offered) instead of forcing best-effort
  bool qos_match{true};

  /// Recordings retained per fault code (0 = unlimited, bounded only by
  /// max_total_storage_mb). Keep-newest: past the cap the fault's OLDEST recording
  /// loses its row, and the bag goes once no fault references it. 1 reproduces the
  /// pre-#620 behaviour, where a re-confirm replaced the previous recording.
  size_t max_bags_per_fault{1};

  /// If true, delete bag file when fault is cleared
  bool auto_cleanup{true};
};

/// Configuration for snapshot capture
struct SnapshotConfig {
  /// Whether snapshot capture is enabled
  bool enabled{true};

  /// Whether to use background subscriptions (always have latest data)
  /// If false, creates on-demand subscriptions when fault is confirmed
  bool background_capture{false};

  /// Timeout in seconds for on-demand topic sampling
  double timeout_sec{1.0};

  /// Maximum message size in bytes (messages larger than this are skipped)
  size_t max_message_size{65536};

  /// Topics to capture for specific fault codes (exact match)
  /// Key: fault_code, Value: list of topics
  std::map<std::string, std::vector<std::string>> fault_specific;

  /// Topics to capture for fault code patterns (regex)
  /// Key: regex pattern, Value: list of topics
  /// First matching pattern wins
  std::map<std::string, std::vector<std::string>> patterns;

  /// Default topics to capture if no specific or pattern match
  std::vector<std::string> default_topics;

  /// Zero-config fallback when NO explicit config matches a fault code: capture
  /// the topics published by the fault's reporting source node(s) instead of
  /// skipping the capture entirely. Explicit config (fault_specific / patterns /
  /// default_topics) always wins when present.
  bool entity_default{true};

  /// Rosbag recording configuration (optional)
  RosbagConfig rosbag;
};

/// Cached message from background subscription
struct CachedMessage {
  std::string topic;
  std::string message_type;
  std::string data;  ///< JSON-encoded message
  int64_t timestamp_ns{0};
};

/// Captures topic snapshots when faults are confirmed
///
/// Supports two modes:
/// - On-demand: Creates temporary subscriptions when fault is confirmed, waits for data
/// - Background: Maintains subscriptions to configured topics, caches latest messages
class SnapshotCapture {
 public:
  /// Create snapshot capture
  /// @param node ROS 2 node for creating subscriptions
  /// @param storage Fault storage for persisting snapshots
  /// @param config Snapshot configuration
  SnapshotCapture(rclcpp::Node * node, FaultStorage * storage, const SnapshotConfig & config);

  ~SnapshotCapture();

  // Non-copyable, non-movable
  SnapshotCapture(const SnapshotCapture &) = delete;
  SnapshotCapture & operator=(const SnapshotCapture &) = delete;
  SnapshotCapture(SnapshotCapture &&) = delete;
  SnapshotCapture & operator=(SnapshotCapture &&) = delete;

  /// Capture snapshots for a fault that was just confirmed
  ///
  /// If the fault code resolves to no capture set (not in fault_specific, no pattern
  /// match, no default_topics), the entity-default fallback (when enabled) captures
  /// the reporting source node's own published topics instead. A code explicitly
  /// listed in fault_specific or matched by a pattern never falls through - a
  /// present-but-empty topic list is a per-fault opt-out. Only when nothing
  /// resolves does capture return early: no freeze_frames row is written
  /// (no empty {} row) and FaultStorage::get_freeze_frame() returns nullopt for it.
  /// @param fault_code The fault code that was confirmed
  void capture(const std::string & fault_code);

  /// Get current configuration
  const SnapshotConfig & config() const {
    return config_;
  }

  /// Check if snapshot capture is enabled
  bool is_enabled() const {
    return config_.enabled;
  }

 private:
  /// Resolve which topics to capture for a given fault code
  /// Priority: fault_specific > patterns > default_topics
  /// @param[out] explicit_match true when the code hit fault_specific or a
  ///             pattern (even with an empty list - an explicit opt-out) or
  ///             default_topics applied; such codes never fall through to the
  ///             entity-default capture.
  std::vector<std::string> resolve_topics(const std::string & fault_code, bool & explicit_match) const;

  /// Entity-default fallback: topics published by the fault's reporting source
  /// node(s), excluding per-node noise (/rosout, /parameter_events). Non-FQN
  /// sources (bare plugin entity ids - the gateway covers those) are skipped,
  /// and empty is returned when no source resolves to a live node. Never
  /// throws; any failure degrades to empty.
  std::vector<std::string> resolve_entity_topics(const std::string & fault_code) const;

  /// Capture a single topic on-demand (creates temporary subscription)
  /// On success also records the captured value into @p freeze_frame under the topic key.
  /// @return true if capture was successful
  /// Appends to @p rows rather than storing: a capture is persisted as one set,
  /// so the per-fault cap can drop a whole old capture instead of truncating this
  /// one topic by topic.
  bool capture_topic_on_demand(const std::string & fault_code, const std::string & topic, nlohmann::json & freeze_frame,
                               std::vector<SnapshotData> & rows);

  /// Capture a topic from background cache
  /// On success also records the cached value into @p freeze_frame under the topic key.
  /// @return true if data was available in cache
  bool capture_topic_from_cache(const std::string & fault_code, const std::string & topic,
                                nlohmann::json & freeze_frame, std::vector<SnapshotData> & rows);

  /// Initialize background subscriptions for all configured topics
  void init_background_subscriptions();

  /// Collect all unique topics from configuration
  std::vector<std::string> collect_all_configured_topics() const;

  /// Get message type for a topic
  std::string get_topic_type(const std::string & topic) const;

  rclcpp::Node * node_;
  FaultStorage * storage_;
  SnapshotConfig config_;

  /// Mints one id per capture, shared by every row of that capture.
  ///
  /// Seeded from storage at construction rather than started at zero: the ids
  /// outlive the process on a persistent backend, and eviction protects the
  /// HIGHEST id, so a counter that restarts below what is already stored makes the
  /// freshly written capture the first one dropped.
  std::atomic<int64_t> capture_seq_{0};

  /// Compiled regex patterns (cached for performance)
  std::vector<std::pair<std::regex, std::vector<std::string>>> compiled_patterns_;

  /// Background subscription cache (topic -> latest message)
  mutable std::mutex cache_mutex_;
  std::map<std::string, CachedMessage> message_cache_;

  /// Serializes node_->create_callback_group() and node_->create_generic_subscription()
  /// across concurrent capture threads. rclcpp node internals (rcutils_hash_map) are
  /// not thread-safe for concurrent entity creation.
  std::mutex node_ops_mutex_;

  /// Background subscriptions (kept alive for continuous caching)
  std::vector<rclcpp::GenericSubscription::SharedPtr> background_subscriptions_;
};

}  // namespace ros2_medkit_fault_manager
