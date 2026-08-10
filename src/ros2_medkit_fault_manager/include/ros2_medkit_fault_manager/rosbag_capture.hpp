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
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

namespace ros2_medkit_fault_manager {

/// A buffered message stored in the ring buffer
struct BufferedMessage {
  std::string topic;
  std::string message_type;
  std::shared_ptr<rclcpp::SerializedMessage> serialized_data;
  int64_t timestamp_ns{0};
};

/// Manages rosbag recording with ring buffer for time-window capture
///
/// This class implements a "black box" style recording where messages are
/// continuously buffered in memory. When a fault is confirmed, the buffer
/// is flushed to a bag file along with continued recording for a short
/// period after the fault.
///
/// Lifecycle:
/// - start() begins buffering messages (or lazy_start waits for PREFAILED)
/// - on_fault_confirmed() flushes buffer to bag file
/// - on_fault_cleared() deletes bag file if auto_cleanup enabled
///
/// Only one recording is open at a time (one ring buffer, one writer, one post-roll
/// state machine). That makes the ring buffer empty by construction right after a
/// post-fault window closes: the flush that opened it moved the whole deque out, and
/// everything published during the window went straight to that bag instead of the
/// buffer. A fault confirming in that moment - typically the second fault of a burst -
/// therefore has no pre-fault history to write, and gets a post-fault-only recording
/// instead of nothing (a bag holding just its own duration_after_sec window). With
/// duration_after_sec == 0 no window exists and such a fault gets no bag.
class RosbagCapture {
 public:
  /// Probe a rosbag2 storage backend. Returns std::nullopt when the backend is
  /// usable, or the failure reason (never throws). Injectable so tests can force a
  /// backend unavailable without depending on which plugins CI happens to install.
  using StorageProbeFn = std::function<std::optional<std::string>(const std::string & format)>;

  /// Create rosbag capture
  /// @param node ROS 2 node for creating subscriptions and timers
  /// @param storage Fault storage for persisting bag file metadata
  /// @param config Rosbag configuration
  /// @param snapshot_config Snapshot configuration (for topic resolution when topics="config")
  /// @param storage_probe Optional storage-backend probe override (default: real probe)
  RosbagCapture(rclcpp::Node * node, FaultStorage * storage, const RosbagConfig & config,
                const SnapshotConfig & snapshot_config, StorageProbeFn storage_probe = {});

  ~RosbagCapture();

  // Non-copyable, non-movable
  RosbagCapture(const RosbagCapture &) = delete;
  RosbagCapture & operator=(const RosbagCapture &) = delete;
  RosbagCapture(RosbagCapture &&) = delete;
  RosbagCapture & operator=(RosbagCapture &&) = delete;

  /// Start the ring buffer recording
  /// If lazy_start is false, this is called automatically on construction
  void start();

  /// Stop recording and clean up subscriptions
  void stop();

  /// Check if ring buffer is currently running
  bool is_running() const;

  /// Called when a fault enters PREFAILED state (for lazy_start mode)
  /// @param fault_code The fault code that entered PREFAILED
  void on_fault_prefailed(const std::string & fault_code);

  /// Called when a fault is confirmed - flushes buffer to bag file. A fault that
  /// confirms while the previous fault's post-roll is still running is attached
  /// to that recording (same burst, same window) rather than losing its bag.
  /// @param fault_code The fault code that was confirmed
  void on_fault_confirmed(const std::string & fault_code);

  /// Called when a fault is cleared - deletes its bag record if auto_cleanup.
  /// A shared bag survives until its last referencing fault clears; a fault
  /// cleared during its burst's post-roll is dropped from the in-flight
  /// recording state and never gets a record.
  /// @param fault_code The fault code that was cleared
  void on_fault_cleared(const std::string & fault_code);

  /// Get current configuration
  const RosbagConfig & config() const {
    return config_;
  }

  /// Check if rosbag capture is enabled
  bool is_enabled() const {
    return config_.enabled;
  }

  /// Whether a topic is a high-bandwidth sensor stream (image/points/depth/compressed),
  /// auto-excluded from broad-mode capture. Static + public so it is directly testable.
  static bool is_high_bandwidth_topic(const std::string & topic);

  /// Delete oldest bags from @p storage until its accounted total fits @p max_bytes.
  /// A bag shared by a burst of correlated faults is evicted as one unit: every row
  /// referencing it goes together and its bytes are freed once, so the running total
  /// tracks what is really on disk instead of drifting below it.
  /// @return Paths of the evicted bags, oldest first.
  /// Static + public so the quota arithmetic is testable without a live recording.
  static std::vector<std::string> evict_bags_over_quota(FaultStorage * storage, size_t max_bytes);

 private:
  /// Outcome of a ring-buffer flush. "Nothing was buffered" and "the bag could not
  /// be written" used to share one empty-string return, but they call for opposite
  /// reactions: an empty buffer at a post-fault window boundary is the expected
  /// state for a burst's later fault and still deserves a recording, while an I/O
  /// failure must never have a post-roll opened on top of it.
  enum class FlushStatus {
    kOk,           ///< Buffered messages were written; active_writer_ stays open.
    kEmptyBuffer,  ///< Nothing was buffered; no writer was opened, nothing to clean up.
    kIoError,      ///< Path creation, writer open or write failed; the partial bag is gone.
  };

  struct FlushResult {
    FlushStatus status{FlushStatus::kEmptyBuffer};
    std::string bag_path;  ///< Set only when status == kOk.
  };

  /// Initialize subscriptions for configured topics
  void init_subscriptions();

  /// Message callback for all subscribed topics
  void message_callback(const std::string & topic, const std::string & msg_type,
                        const std::shared_ptr<const rclcpp::SerializedMessage> & msg);

  /// Prune old messages from buffer based on duration_sec
  void prune_buffer();

  /// Resolve which topics to record based on config
  std::vector<std::string> resolve_topics() const;

  /// Resolve the publisher-offered QoS for a topic so capture is faithful
  /// (falls back to SensorDataQoS when no publisher is known or qos_match is off)
  rclcpp::QoS resolve_topic_qos(const std::string & topic) const;

  /// Compute the entity topic set for a fault (the faulting source node's
  /// pub/sub topics + /tf, intersected with the subscribed set). Empty set =
  /// scope unresolved. Never throws; failures degrade to an empty set.
  std::set<std::string> compute_entity_topics(const std::string & fault_code);

  /// In "entity" mode, compute the set of topics to write for a confirmed fault
  /// (the faulting source node's pub/sub topics + /tf). Empty set = write all.
  void resolve_entity_topics(const std::string & fault_code);

  /// In "entity" mode, union an attached fault's entity topics into the active
  /// capture filter so its data reaches the shared bag from the attach onwards
  /// (empty resolution widens to all topics). Caller holds post_fault_timer_mutex_.
  void widen_capture_filter_for(const std::string & fault_code);

  /// Whether a topic should be written to the bag given the active entity filter
  bool should_capture_topic(const std::string & topic) const;

  /// Get message type for a topic
  std::string get_topic_type(const std::string & topic) const;

  /// Flush ring buffer to a bag file
  /// @param fault_code The fault code to associate with the bag
  /// @return The flush outcome. On kOk the bag path is set and active_writer_ is
  ///         left open for the post-fault window; on kEmptyBuffer no writer was
  ///         opened; on kIoError the partial bag has already been removed.
  FlushResult flush_to_bag(const std::string & fault_code);

  /// Open a fresh bag for @p fault_code: generate the path, create its parent
  /// directory and open active_writer_. Independent of the ring buffer, so a
  /// post-fault-only recording can use it with nothing buffered.
  /// @return The bag path, or std::nullopt when the open failed (never throws).
  std::optional<std::string> open_bag_writer(const std::string & fault_code);

  /// Drop active_writer_ and remove the partial bag at @p bag_path.
  void discard_active_writer(const std::string & bag_path);

  /// Generate bag file path for a fault
  std::string generate_bag_path(const std::string & fault_code) const;

  /// Calculate total size of a bag directory
  size_t calculate_bag_size(const std::string & bag_path) const;

  /// Enforce storage limits by deleting oldest bags
  void enforce_storage_limits();

  /// Start the post-fault window: re-arm the timer, creating it the first time.
  ///
  /// The timer is created once and re-armed, never replaced per recording. A timer
  /// per recording had the confirming capture-pool worker creating one while the
  /// executor thread destroyed the previous one, which mutates rcl's clock
  /// jump-callback list from two threads - and the two coincide exactly in the
  /// burst-at-the-boundary case this class exists to serve. The destruction side is
  /// not ours to serialise: the executor holds the last reference in
  /// `AnyExecutable::timer` and drops it after the callback returns, outside any
  /// mutex we could take. Re-arming removes the second party instead of trying to
  /// lock it. Caller must hold post_fault_timer_mutex_.
  void arm_post_fault_timer(std::chrono::nanoseconds period);

  /// Timer callback for post-fault recording
  void post_fault_timer_callback();

  /// Close the in-flight post-fault recording, store its metadata (for the
  /// triggering fault and every fault attached to it) and clear the recording
  /// state. Idempotent - a no-op when no post-roll is running, so the post-fault
  /// timer and stop() can both call it.
  void finalize_post_fault_recording();

  /// Attach @p fault_code to the in-flight post-fault recording, if there is one.
  /// Returns true when the fault was handled (attached, already recording, or the
  /// attachment cap was hit) and the caller must not open a second bag.
  bool attach_to_active_recording(const std::string & fault_code);

  /// Try to subscribe to a single topic
  /// @param topic The topic to subscribe to
  /// @return True if subscription was created, false if type couldn't be determined
  bool try_subscribe_topic(const std::string & topic);

  /// Timer callback for retrying topic discovery
  void discovery_retry_callback();

  /// Default storage probe: opens a throwaway bag for @p format. Returns
  /// std::nullopt when usable, or the failure reason (never throws), so the caller
  /// can degrade gracefully instead of terminating the node.
  std::optional<std::string> default_storage_probe(const std::string & format) const;

  /// Serialises the calls that MUTATE THE NODE - creating a timer or a
  /// subscription, and destroying the ones we own. rclcpp node internals are not
  /// thread-safe for concurrent entity creation or destruction (the rcutils_hash_map
  /// class of race, issue 375), and this class touches them from three threads: the
  /// capture-pool worker that confirms a fault, the executor thread that runs the
  /// discovery and post-fault timers, and whichever thread calls start()/stop().
  /// Same remedy as SnapshotCapture::node_ops_mutex_.
  ///
  /// Innermost in the lock order, and never held across bag I/O.
  std::mutex node_ops_mutex_;

  /// Storage-backend probe (the default real probe, or a test override).
  StorageProbeFn storage_probe_;

  rclcpp::Node * node_;
  FaultStorage * storage_;
  RosbagConfig config_;
  SnapshotConfig snapshot_config_;

  /// Ring buffer for messages
  mutable std::mutex buffer_mutex_;
  std::deque<BufferedMessage> message_buffer_;
  /// Running byte size of message_buffer_ (guarded by buffer_mutex_), for the RAM cap
  size_t buffer_bytes_{0};

  /// Topics to write for the in-flight capture in "entity" mode (guarded below).
  /// Empty = no entity filter, write everything buffered (manual modes / fallback).
  mutable std::mutex capture_topics_mutex_;
  std::set<std::string> active_capture_topics_;

  /// Subscriptions (kept alive for continuous recording)
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  /// Running state
  std::atomic<bool> running_{false};

  /// Upper bound on how many extra faults one recording is registered for.
  static constexpr size_t kMaxAttachedFaults = 32;

  /// Post-fault recording state
  std::string current_fault_code_;
  std::string current_bag_path_;
  /// Faults confirmed while the post-roll was already running. They share the
  /// recording window (one root cause, one burst), so each gets a metadata row
  /// pointing at the same bag when it finalises.
  std::set<std::string> attached_fault_codes_;
  /// Protects post_fault_timer_, the recording_post_fault_ transitions and the
  /// state above against concurrent access from on_fault_confirmed() (capture-pool
  /// thread) and post_fault_timer_callback() / stop() (executor thread). The
  /// node-level rosbag mutex serialises confirmations against each other but NOT
  /// against the timer, so this lock is the only thing ordering the two.
  ///
  /// Lock order (no cycle; every edge below is one-directional):
  ///   node rosbag mutex -> post_fault_timer_mutex_ -> capture_topics_mutex_
  ///   node rosbag mutex -> post_fault_timer_mutex_ -> writer_mutex_
  ///   node rosbag mutex -> post_fault_timer_mutex_ -> node_ops_mutex_
  /// buffer_mutex_ is never held across another lock. The paths that take
  /// capture_topics_mutex_ or writer_mutex_ on their own (the flush loop, the
  /// post-roll write path) release each before taking the next, so they add no
  /// reverse edge. Everything that hands the RECORDING over - the guard, the
  /// start time, the writer - must happen inside one post_fault_timer_mutex_
  /// critical section, or a confirmation racing a finalise ends up owning half of
  /// the previous recording's state.
  std::mutex post_fault_timer_mutex_;
  rclcpp::TimerBase::SharedPtr post_fault_timer_;
  std::atomic<bool> recording_post_fault_{false};

  /// When the open recording started, on the MONOTONIC clock: the moment the writer
  /// opened for a post-fault-only bag, or that moment less the age of the oldest
  /// flushed message for a full one. Read when the recording is finalised so
  /// duration_sec reports the span the bag covers instead of the configured window -
  /// a short buffer or a post-fault-only bag would otherwise claim history it does
  /// not hold. Monotonic and not the wall clock that timestamps messages, because a
  /// wall clock that steps backwards mid-window turns an elapsed time negative and
  /// the duration is then reported as zero. Atomic and exchanged inside the
  /// post_fault_timer_mutex_ critical section that clears the recording guard, so a
  /// confirmation racing the finalise cannot have its own start time attributed to
  /// the bag being closed.
  std::atomic<int64_t> recording_started_at_ns_{0};

  /// Active writer for current bag (kept open during post-fault recording)
  std::unique_ptr<rosbag2_cpp::Writer> active_writer_;
  std::mutex writer_mutex_;
  std::set<std::string> created_topics_;

  /// Topic types cache
  mutable std::mutex topic_types_mutex_;
  std::map<std::string, std::string> topic_types_;

  /// Discovery retry state
  rclcpp::TimerBase::SharedPtr discovery_retry_timer_;
  std::vector<std::string> pending_topics_;
  int discovery_retry_count_{0};

  /// Topics already subscribed (guards against duplicate subscriptions when the
  /// broad-mode discovery timer re-resolves the topic set).
  std::set<std::string> subscribed_topics_;

  /// True in broad modes ("all"/"auto"/"entity"): the discovery timer keeps
  /// re-resolving for the capture's lifetime so topics whose publishers appear
  /// after startup are still subscribed (dynamic capture). False in fixed modes
  /// (config/explicit/list), where only the initial set is retried.
  bool dynamic_discovery_{false};
};

}  // namespace ros2_medkit_fault_manager
