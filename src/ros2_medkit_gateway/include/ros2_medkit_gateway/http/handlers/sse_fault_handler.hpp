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
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handler for Server-Sent Events (SSE) fault streaming.
 *
 * Provides real-time fault event notifications via SSE at:
 * - GET /faults/stream
 *
 * Events streamed:
 * - fault_confirmed: When a fault transitions to CONFIRMED status
 * - fault_cleared: When a fault ends (acknowledged clear, or auto-heal)
 * - fault_updated: When fault data changes (occurrence_count, sources)
 *
 * Features:
 * - Multi-client support (multiple browsers can connect simultaneously)
 * - Keepalive every 30 seconds to prevent connection timeout
 * - Automatic reconnection support via Last-Event-ID header
 * - Replay buffer of up to 100 events. Eviction is state-convergent rather
 *   than plain FIFO: entries every live client has already been sent go
 *   first, then entries superseded by a newer event for the same fault code
 *   (a lagging client still converges on the correct current state of every
 *   fault). Only when neither applies is an event genuinely lost, and only
 *   that case is counted and logged as a drop.
 * - Clients that make no progress for 3 keepalive intervals are treated as
 *   dead, reaped, and excluded from the backpressure accounting.
 */
class SSEFaultHandler {
 public:
  /**
   * @brief Construct SSE fault handler with shared context.
   * @param ctx The shared handler context
   * @param client_tracker Shared SSE client counter (across all SSE handlers)
   */
  SSEFaultHandler(HandlerContext & ctx, std::shared_ptr<SSEClientTracker> client_tracker);
  /**
   * @brief Test-only constructor that overrides the keepalive interval.
   * @param ctx The shared handler context
   * @param client_tracker Shared SSE client counter (across all SSE handlers)
   * @param keepalive_interval Must be positive; non-positive values fall back to the 30s default.
   */
  SSEFaultHandler(HandlerContext & ctx, std::shared_ptr<SSEClientTracker> client_tracker,
                  std::chrono::milliseconds keepalive_interval);

  /// Destructor - cleanup subscription
  ~SSEFaultHandler();

  // Disable copy/move
  SSEFaultHandler(const SSEFaultHandler &) = delete;
  SSEFaultHandler & operator=(const SSEFaultHandler &) = delete;
  SSEFaultHandler(SSEFaultHandler &&) = delete;
  SSEFaultHandler & operator=(SSEFaultHandler &&) = delete;

  /**
   * @brief Handle GET /faults/stream - SSE stream endpoint (typed RouteRegistry).
   *
   * Returns a `SseStream` whose `next_event` callback the framework drives via
   * cpp-httplib's chunked content provider. On limit-exceeded the factory
   * returns `tl::unexpected(ErrorInfo)` with HTTP 503; the framework renders
   * a SOVD GenericError.
   *
   * Events streamed:
   * @code
   * event: fault_confirmed
   * data: {"event_type":"fault_confirmed","fault":{...},"timestamp":1234567890.123}
   *
   * event: fault_cleared
   * data: {"event_type":"fault_cleared","fault":{...},"timestamp":1234567890.456}
   * @endcode
   */
  http::Result<http::SseStream> sse_stream(const http::TypedRequest & req);

  /**
   * @brief Legacy SSE entry point - drives the chunked content provider on
   * `res` directly.
   *
   * Retained for the in-process unit test fixture (`test_sse_fault_handler`),
   * which exercises the streaming loop without spinning up the typed router.
   * The framework-registered route uses `sse_stream` via `reg.sse`; this
   * overload wraps the same logic and additionally sets the legacy headers
   * (Cache-Control / Connection / X-Accel-Buffering) that the framework wires
   * automatically for the typed path.
   */
  void handle_stream(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Get the number of currently connected SSE clients.
   */
  size_t connected_clients() const;

  /**
   * @brief Events genuinely lost: evicted while still owed to a live client and
   * not superseded by a newer event for the same fault code.
   *
   * Rotation of the replay buffer with no client attached is NOT a drop.
   */
  std::size_t dropped_events() const;

  /**
   * @brief Events evicted because a newer event for the same fault code was
   * already buffered. Lagging clients still converge on the correct state.
   */
  std::size_t coalesced_events() const;

  /**
   * @brief Clients declared dead (no delivery progress for 3 keepalive
   * intervals) and reaped while making room in the replay buffer.
   */
  std::size_t reaped_clients() const;

  /**
   * @brief Total number of fault events received and processed so far.
   *
   * Monotonically increasing (it never resets, even when the replay buffer
   * evicts old events). Lets tests wait deterministically until a published
   * event has actually been consumed by `on_fault_event` - which is where the
   * owning entity is snapshotted - instead of spinning for a fixed time.
   */
  uint64_t events_received() const;

  /**
   * @brief Signal shutdown so in-flight chunked-content-provider loops exit.
   *
   * Call this BEFORE stopping the HTTP server. The server thread's join
   * waits for active request lambdas to return; the SSE lambda sleeps on
   * queue_cv_ until keepalive (30s) so without an early signal the join
   * can exceed the launch_testing shutdown budget (5s SIGINT + 10s SIGTERM)
   * and the process ends up SIGKILLed (exit -9). Setting the flag and
   * notifying wakes the lambda, it returns false, and the http thread exits
   * promptly. Safe to call more than once.
   */
  void request_shutdown();

 private:
  /// Build the per-client streaming loop closure used by both `sse_stream`
  /// (typed RouteRegistry path) and `handle_stream` (legacy in-process test
  /// entry). The returned callable is invoked with a `DataSink` and returns
  /// `false` when the client disconnects or `shutdown_flag_` is set.
  std::function<bool(httplib::DataSink &)> make_stream_loop(uint64_t initial_last_event_id);

  /// Callback for fault events from ROS 2 topic
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  /// Resolved owning entity for a fault. Populates the ``x-medkit`` SOVD
  /// payload-extension object on outgoing SSE events.
  struct EntityContext {
    std::string type;
    std::string id;
  };

  /// Buffered queue entry. ``entity`` is resolved at enqueue time so a
  /// discovery refresh between enqueue and stream-out cannot retroactively
  /// flip the entity reported to consumers.
  struct QueuedEvent {
    uint64_t id;
    ros2_medkit_msgs::msg::FaultEvent event;
    std::optional<EntityContext> entity;
  };

  /// Per-connection delivery cursor. Lets the buffer tell "nobody is owed this
  /// event" from "a live client has not read it yet", so ring-buffer rotation
  /// is no longer reported as client backpressure.
  struct ClientCursor {
    uint64_t last_delivered{0};
    std::chrono::steady_clock::time_point last_progress{};
    bool stalled{false};
  };

  /// Outcome of one eviction pass, reported outside the queue lock.
  struct EvictionStats {
    std::size_t dropped{0};
    std::size_t coalesced{0};
    std::size_t reaped{0};
    std::size_t slow_clients{0};
  };

  /// Trim the buffer back to kMaxBufferedEvents. Caller holds queue_mutex_.
  EvictionStats evict_to_capacity_locked();

  /// Highest event id already delivered to every live client; kNothingOwed when
  /// no live client is attached. Caller holds queue_mutex_.
  uint64_t delivered_watermark_locked() const;

  /// Oldest buffered entry whose fault code appears again later in the buffer,
  /// i.e. one whose state a later entry already supersedes. Entries carrying
  /// auto_cleared_codes are never chosen: that correlation payload exists
  /// nowhere else. Caller holds queue_mutex_.
  std::deque<QueuedEvent>::iterator find_superseded_locked();

  /// Mark cursors with no delivery progress for kStallIntervals keepalives as
  /// dead so their stream loop exits. Caller holds queue_mutex_.
  std::size_t reap_stalled_clients_locked(std::chrono::steady_clock::time_point now);

  /// Record a successful write for this client. Caller holds queue_mutex_.
  void note_progress_locked(const std::shared_ptr<ClientCursor> & cursor, uint64_t delivered_id);

  /// Format a fault event as SSE message
  static std::string format_sse_event(const QueuedEvent & queued);

  /// Resolve the owning entity for a fault, snapshotting the cache. Manifest
  /// / hybrid mode uses the cache's node-to-app index; runtime mode falls
  /// back to the FQN's last segment, and for collision-disambiguated runtime
  /// apps also to ``<ns_prefix>_<name>`` per
  /// ros2_runtime_introspection.cpp's renaming rule. Only emits a value when
  /// an App with the resolved id actually exists in the cache - returns
  /// ``std::nullopt`` otherwise so the consumer falls back to discovery.
  std::optional<EntityContext> resolve_entity_context(const ros2_medkit_msgs::msg::Fault & fault) const;

  HandlerContext & ctx_;
  std::shared_ptr<SSEClientTracker> client_tracker_;

  /// Subscription to fault events topic
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr subscription_;

  /// Event queue for broadcasting to clients
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedEvent> event_queue_;

  /// Monotonically increasing event ID for Last-Event-ID support
  std::atomic<uint64_t> next_event_id_{1};

  /// Shutdown flag for clean termination
  std::atomic<bool> shutdown_flag_{false};

  /// Live delivery cursors, one per streaming connection. Guarded by queue_mutex_.
  std::vector<std::shared_ptr<ClientCursor>> clients_;

  /// Total events genuinely lost (owed to a live client, nothing newer for the
  /// same fault code). Surfaced via WARN logs at a fixed cadence
  /// (kDropLogEveryN) so operators notice client lag without flooding the
  /// logger.
  std::atomic<std::size_t> dropped_events_{0};

  /// Total events evicted because a newer event for the same fault code was
  /// buffered. Not a loss: the client still converges on the right state.
  std::atomic<std::size_t> coalesced_events_{0};

  /// Total clients reaped as dead while making room in the buffer.
  std::atomic<std::size_t> reaped_clients_{0};

  /// Keepalive interval used by the streaming loop
  std::chrono::milliseconds keepalive_interval_;

  /// Maximum events to buffer (for reconnecting clients)
  static constexpr size_t kMaxBufferedEvents = 100;

  /// Emit a WARN log every Nth dropped event to keep observability without
  /// log spam under sustained backpressure.
  static constexpr std::size_t kDropLogEveryN = 10;

  /// Keepalive interval in seconds
  static constexpr int kKeepaliveIntervalSec = 30;

  /// A client that has not completed a write (event or keepalive) for this many
  /// keepalive intervals is dead, not slow.
  static constexpr int kStallIntervals = 3;

  /// Watermark meaning "no live client is owed anything in the buffer".
  static constexpr uint64_t kNothingOwed = std::numeric_limits<uint64_t>::max();
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
