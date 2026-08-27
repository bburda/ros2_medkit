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

#include <httplib.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief A single event received from a streaming connection
 *
 * Represents an SSE event with type, data payload, optional ID, and
 * the name of the peer gateway that produced it.
 */
struct StreamEvent {
  std::string event_type;  ///< SSE event name (e.g., "data", "fault")
  std::string data;        ///< Event data (JSON string)
  std::string id;          ///< Event ID (optional)
  std::string peer_name;   ///< Which peer this event came from
};

/**
 * @brief Transport-agnostic interface for proxying streaming connections
 *
 * StreamProxy abstracts the transport layer for streaming event connections
 * to peer gateways. Currently implemented with SSE, but the interface allows
 * future implementations using WebSocket or gRPC streams.
 *
 * Usage:
 *   proxy->on_event([](const StreamEvent& e) { handle(e); });
 *   proxy->open();
 *   // ... events flow via callback ...
 *   proxy->close();
 */
class StreamProxy {
 public:
  virtual ~StreamProxy() = default;

  /// Start the streaming connection (non-blocking, spawns reader thread)
  virtual void open() = 0;

  /// Stop the streaming connection and join the reader thread
  virtual void close() = 0;

  /// Check if the streaming connection is currently active
  virtual bool is_connected() const = 0;

  /// Register a callback for incoming stream events
  virtual void on_event(std::function<void(const StreamEvent &)> cb) = 0;
};

/**
 * @brief SSE (Server-Sent Events) implementation of StreamProxy
 *
 * Connects to a peer gateway's SSE endpoint using cpp-httplib and parses
 * the SSE text/event-stream format into StreamEvent objects. Runs a
 * background reader thread that invokes the registered callback for each
 * parsed event.
 *
 * Thread safety: connected_ and should_stop_ are atomic. The callback
 * is set before open() and not modified afterwards.
 */
class SSEStreamProxy : public StreamProxy {
 public:
  /**
   * @brief Construct an SSEStreamProxy
   * @param peer_url Base URL of the peer gateway (e.g., "http://localhost:8081")
   * @param path SSE endpoint path (e.g., "/api/v1/components/abc/faults/sse")
   * @param peer_name Human-readable name for the peer (used in StreamEvent::peer_name)
   * @param headers Headers sent with every connect attempt, including after a
   *   reconnect. An aggregator relaying a peer's stream sends
   *   `X-Medkit-No-Fan-Out` here, which is what stops a chain of aggregating
   *   gateways relaying the same event round the loop.
   */
  SSEStreamProxy(const std::string & peer_url, const std::string & path, const std::string & peer_name = "",
                 httplib::Headers headers = {});

  ~SSEStreamProxy() override;

  SSEStreamProxy(const SSEStreamProxy &) = delete;
  SSEStreamProxy & operator=(const SSEStreamProxy &) = delete;
  SSEStreamProxy(SSEStreamProxy &&) = delete;
  SSEStreamProxy & operator=(SSEStreamProxy &&) = delete;

  void open() override;
  void close() override;
  bool is_connected() const override;
  void on_event(std::function<void(const StreamEvent &)> cb) override;

  /// The id of the last event this proxy received, or empty if none has been.
  ///
  /// Read by the owner when a stream is retired so a replacement for the same
  /// peer can resume where this one stopped. Without that hand-off a reopened
  /// stream asks for the peer's whole buffer and every event in it is
  /// delivered again, as new, to whoever is attached.
  std::string last_event_id() const;

  /// Resume from an id a previous proxy for this peer reached. Must be called
  /// before open().
  void set_last_event_id(std::string id);

  /**
   * @brief Parse raw SSE text/event-stream data into StreamEvent objects
   *
   * SSE format: events are separated by blank lines. Each event consists
   * of field lines: "event:", "data:", "id:". Multiple "data:" lines are
   * joined with newlines.
   *
   * This is a pure function suitable for unit testing without networking.
   *
   * @param raw Raw SSE text data
   * @param peer Peer name to set on each parsed event
   * @return Vector of parsed StreamEvent objects
   */
  static std::vector<StreamEvent> parse_sse_data(const std::string & raw, const std::string & peer);

 private:
  /// Background thread loop that connects and reads SSE events
  void reader_loop();

  std::string peer_url_;
  std::string path_;
  std::string peer_name_;
  httplib::Headers headers_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> should_stop_{false};
  std::atomic<bool> non_sse_content_type_{false};
  /// Status the peer answered with when it refused the stream, or 0. Recorded
  /// in the header callback and reported from the reader loop, where a logger
  /// is reachable. Without it a peer that answers 401, 429 or 503 is retried
  /// for ever and says nothing, which is the same open-and-silent stream this
  /// relay exists to remove, one hop up.
  std::atomic<int> rejected_status_{0};

  /// Newest `id:` this proxy has seen from the peer, sent back as
  /// `Last-Event-ID` when it reconnects. Without it every reconnect asks the
  /// peer for its whole replay buffer again, and the aggregator re-emits all of
  /// it under fresh ids - so a client sees the same faults repeated after every
  /// backoff cycle, which on a flapping link is continuous. Touched only by the
  /// reader thread.
  /// Written by the reader thread, read by the owner from another one.
  mutable std::mutex cursor_mutex_;
  std::string last_event_id_;
  std::function<void(const StreamEvent &)> callback_;
  std::thread reader_thread_;

  /// The client the reader thread is currently blocked in, so close() can
  /// interrupt it. Without this, close() only sets should_stop_, which the
  /// content callbacks read only when bytes arrive: on a connected but silent
  /// peer the join waits for the peer's next keepalive, or for the 300 s read
  /// timeout if the peer has stopped writing entirely. That wait sits on the
  /// live path through PeerFaultRelay, so an aggregator restarting while a
  /// peer is quiet can overrun its shutdown budget and be killed.
  ///
  /// Signals that reader_loop has returned. close() waits on this instead of
  /// joining straight away, so it can repeat the socket shutdown: one that
  /// lands after the socket exists but before connect() returns is a no-op on
  /// an unconnected socket, and the reader would then park with nothing left
  /// to wake it.
  std::mutex finished_mutex_;
  std::condition_variable finished_cv_;
  bool reader_finished_{false};

  /// Guarded because the reader thread publishes it and close() reads it.
  ///
  /// The socket, NOT the httplib::Client. Calling Client::stop() while another
  /// thread is inside Get() unlocks one of cpp-httplib's own mutexes from a
  /// thread that never locked it; TSan reports "unlock of an unlocked mutex
  /// (or by a wrong thread)" and glibc's pthread debug mode treats it as fatal.
  /// That is the same hazard PeerClient documents when it explains why it never
  /// stops its health-check client. Shutting the socket down instead wakes the
  /// blocked read through the kernel and touches none of the library's state.
  std::mutex socket_mutex_;
  int active_socket_{-1};
};

}  // namespace ros2_medkit_gateway
