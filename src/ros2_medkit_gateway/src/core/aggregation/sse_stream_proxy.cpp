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

#include "ros2_medkit_gateway/core/aggregation/stream_proxy.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

SSEStreamProxy::SSEStreamProxy(const std::string & peer_url, const std::string & path, const std::string & peer_name,
                               httplib::Headers headers)
  : peer_url_(peer_url), path_(path), peer_name_(peer_name), headers_(std::move(headers)) {
}

SSEStreamProxy::~SSEStreamProxy() {
  close();
}

void SSEStreamProxy::open() {
  // Guard against double-open: check if the reader thread is already running
  // (not just connected_, which may be false during reconnect backoff).
  if (reader_thread_.joinable()) {
    return;
  }
  should_stop_.store(false);
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    reader_finished_ = false;
  }
  reader_thread_ = std::thread(&SSEStreamProxy::reader_loop, this);
}

void SSEStreamProxy::close() {
  should_stop_.store(true);
  connected_.store(false);
  if (!reader_thread_.joinable()) {
    return;
  }
  // Interrupt the read the reader thread is parked in. should_stop_ alone is
  // only observed when bytes arrive, so a peer that is connected and quiet
  // would hold this join until its next keepalive.
  //
  // Repeated until the reader is actually out, because a single shutdown can
  // miss: on a socket that exists but has not finished connecting it returns
  // ENOTCONN and changes nothing, and the reader goes on to connect and park.
  // Retrying costs one syscall per 100 ms of a teardown that is normally over
  // in one pass.
  {
    std::unique_lock<std::mutex> lock(finished_mutex_);
    while (!reader_finished_) {
      {
        std::lock_guard<std::mutex> socket_lock(socket_mutex_);
        if (active_socket_ >= 0) {
          ::shutdown(active_socket_, SHUT_RDWR);
        }
      }
      finished_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
  }
  reader_thread_.join();
}

std::string SSEStreamProxy::last_event_id() const {
  std::lock_guard<std::mutex> lock(cursor_mutex_);
  return last_event_id_;
}

void SSEStreamProxy::set_last_event_id(std::string id) {
  std::lock_guard<std::mutex> lock(cursor_mutex_);
  last_event_id_ = std::move(id);
}

bool SSEStreamProxy::is_connected() const {
  return connected_.load();
}

void SSEStreamProxy::on_event(std::function<void(const StreamEvent &)> cb) {
  callback_ = std::move(cb);
}

void SSEStreamProxy::reader_loop() {
  // Reconnect loop with exponential backoff.
  // On connection failure or stream interruption, retry with increasing delays
  // (1s, 2s, 4s, ..., max 30s) until should_stop_ is set.
  constexpr int kInitialBackoffMs = 1000;
  constexpr int kMaxBackoffMs = 30000;
  int backoff_ms = kInitialBackoffMs;

  while (!should_stop_.load()) {
    httplib::Client client(peer_url_);
    // The socket is handed to this callback once cpp-httplib has created it,
    // which is where close() gets something it can interrupt. The library's own
    // defaults are applied first so overriding this hook does not drop them.
    //
    // A DUPLICATE, not the library's descriptor. cpp-httplib closes its own
    // inside Get(), and a closed number is free for any other thread in this
    // process to be handed for an unrelated file - after which close() would
    // shut down that one instead. Holding a dup keeps the number reserved until
    // this loop releases it, and shutdown() acts on the socket behind it, so it
    // still tears down the connection the reader is parked on.
    client.set_socket_options([this](socket_t sock) {
      httplib::default_socket_options(sock);
      // F_DUPFD_CLOEXEC, not dup(): a plain duplicate has FD_CLOEXEC clear, so
      // it would be inherited by every process this gateway execs - the script
      // provider runs arbitrary ones - handing them a live socket to a peer.
      const int held = ::fcntl(sock, F_DUPFD_CLOEXEC, 0);
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (active_socket_ >= 0) {
        // A second connection inside one Get(). Nothing is parked on the
        // previous socket any more, and leaving it open would leak a
        // descriptor per reconnect.
        ::close(active_socket_);
      }
      active_socket_ = held;
      if (should_stop_.load()) {
        // close() ran between this loop's stop check and this line, so it took
        // the mutex while there was still no socket to interrupt and is now
        // waiting on a join. It set should_stop_ before taking the mutex, so
        // one of the two orderings always sees the other: do its work here.
        ::shutdown(active_socket_, SHUT_RDWR);
      }
    });
    // Released on every exit from this iteration, including the early breaks
    // below. The dup is this class's to close and nothing else's.
    struct SocketHandleGuard {
      SSEStreamProxy * self;
      ~SocketHandleGuard() {
        std::lock_guard<std::mutex> lock(self->socket_mutex_);
        if (self->active_socket_ >= 0) {
          ::close(self->active_socket_);
          self->active_socket_ = -1;
        }
      }
    } socket_handle_guard{this};
    // SSE read timeout: 300s. If no data (including heartbeat comments) arrives
    // within this window, the connection is considered dead and we reconnect.
    // Peers should send periodic heartbeat comments (": keepalive\n\n") to
    // prevent timeout on idle streams.
    client.set_read_timeout(300, 0);
    client.set_connection_timeout(5, 0);

    // Track whether we received any data to set connected_ only after the
    // stream is actually delivering data (avoids race where is_connected()
    // returns true before the HTTP request even starts).
    bool received_first_data = false;
    non_sse_content_type_.store(false);

    // Use chunked content receiver to process SSE data as it arrives
    std::string buffer;
    // Resume where the last connection stopped. A reconnect without this asks
    // the peer to replay its whole buffer, and every replayed event is emitted
    // again under a new aggregator id.
    httplib::Headers attempt_headers = headers_;
    {
      std::lock_guard<std::mutex> lock(cursor_mutex_);
      if (!last_event_id_.empty()) {
        attempt_headers.emplace("Last-Event-ID", last_event_id_);
      }
    }

    auto result = client.Get(
        path_, attempt_headers,
        [this](const httplib::Response & response) {
          // Header callback - check status and content type before processing body
          if (response.status != 200) {
            rejected_status_.store(response.status);
            return false;
          }
          if (should_stop_.load()) {
            return false;
          }
          // Validate Content-Type is text/event-stream. A non-SSE 200 response
          // (e.g., application/json) would be silently misinterpreted as SSE data.
          auto ct_it = response.headers.find("Content-Type");
          if (ct_it == response.headers.end() || ct_it->second.find("text/event-stream") == std::string::npos) {
            // Not an SSE stream - abort connection. Log is not available here
            // (no rclcpp logger), so we set a flag and log in the outer scope.
            non_sse_content_type_.store(true);
            return false;
          }
          return true;
        },
        [this, &buffer, &received_first_data, &backoff_ms](const char * data, size_t data_length) {
          if (should_stop_.load()) {
            return false;  // Stop receiving
          }

          // Mark connected on first chunk of data from the stream.
          // This avoids the race condition where is_connected() returns true
          // before the HTTP connection is actually established.
          if (!received_first_data) {
            received_first_data = true;
            connected_.store(true);
            backoff_ms = 1000;  // Reset backoff on successful connection
          }

          constexpr size_t kMaxSSEBufferSize = 1 * 1024 * 1024;  // 1MB

          buffer.append(data, data_length);
          if (buffer.size() > kMaxSSEBufferSize) {
            return false;  // Disconnect - peer sending malformed stream
          }

          // Events end at a blank line, which SSE allows to be written either
          // way round: "\n\n" or "\r\n\r\n". Searching only for "\n\n" never
          // matches a CRLF stream, because "\r\n\r\n" holds "\n\r\n" and no
          // "\n\n" - such a peer would deliver bytes for ever and never yield
          // one event. parse_sse_data already handles CRLF within a block; it
          // is this boundary search that has to admit both.
          size_t pos = 0;
          while (true) {
            auto boundary = buffer.find("\n\n", pos);
            size_t boundary_len = 2;
            const auto crlf_boundary = buffer.find("\r\n\r\n", pos);
            if (crlf_boundary != std::string::npos && (boundary == std::string::npos || crlf_boundary < boundary)) {
              boundary = crlf_boundary;
              boundary_len = 4;
            }
            if (boundary == std::string::npos) {
              break;
            }

            std::string event_block = buffer.substr(pos, boundary - pos + 1);
            pos = boundary + boundary_len;

            auto events = parse_sse_data(event_block, peer_name_);
            for (const auto & event : events) {
              // Recorded before dispatch, so a callback that throws still
              // leaves the resume point at the last event actually received.
              if (!event.id.empty()) {
                std::lock_guard<std::mutex> lock(cursor_mutex_);
                last_event_id_ = event.id;
              }
              if (callback_) {
                // The callback is given a payload the peer wrote. One that
                // throws on a shape it did not expect would otherwise leave the
                // exception to escape this thread, and an exception leaving a
                // std::thread calls std::terminate - a peer could end the
                // gateway by sending one malformed field.
                try {
                  callback_(event);
                } catch (const std::exception & e) {
                  fprintf(stderr, "[SSEStreamProxy] dropping event from peer '%s': handler threw: %s\n",
                          peer_name_.c_str(), e.what());
                } catch (...) {
                  fprintf(stderr, "[SSEStreamProxy] dropping event from peer '%s': handler threw\n",
                          peer_name_.c_str());
                }
              }
            }
          }

          // Keep unprocessed data in buffer
          if (pos > 0) {
            buffer.erase(0, pos);
          }

          return true;  // Continue receiving
        });

    // Connection ended (server closed, timeout, or error) - mark disconnected
    connected_.store(false);

    // A peer that refuses the stream says so once per attempt rather than
    // never: 401 means this gateway is not authorised to relay, 503 means the
    // peer is at sse.max_clients, 429 means it is rate limiting. All three are
    // acted on differently and none of them is visible anywhere else.
    if (const int rejected = rejected_status_.exchange(0); rejected != 0) {
      fprintf(stderr, "[SSEStreamProxy] peer '%s' at %s%s refused the stream with status %d; retrying\n",
              peer_name_.c_str(), peer_url_.c_str(), path_.c_str(), rejected);
    }

    // Log if the peer returned a non-SSE Content-Type (e.g. application/json)
    if (non_sse_content_type_.load()) {
      fprintf(stderr,
              "[SSEStreamProxy] Warning: peer '%s' at %s%s returned non-SSE "
              "Content-Type. Expected text/event-stream. Skipping.\n",
              peer_name_.c_str(), peer_url_.c_str(), path_.c_str());
      non_sse_content_type_.store(false);
    }

    // If stop was requested, exit without retrying
    if (should_stop_.load()) {
      break;
    }

    // Exponential backoff before reconnecting
    // Sleep in small increments so we can check should_stop_ promptly
    int slept_ms = 0;
    while (slept_ms < backoff_ms && !should_stop_.load()) {
      constexpr int kSleepStepMs = 100;
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepStepMs));
      slept_ms += kSleepStepMs;
    }

    backoff_ms = std::min(backoff_ms * 2, kMaxBackoffMs);
  }

  // Announced so close() stops re-arming its shutdown and joins. It waits on
  // this rather than joining straight away, so it must always be set, on every
  // way out of the loop above.
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    reader_finished_ = true;
  }
  finished_cv_.notify_all();
}

std::vector<StreamEvent> SSEStreamProxy::parse_sse_data(const std::string & raw, const std::string & peer) {
  std::vector<StreamEvent> events;

  if (raw.empty()) {
    return events;
  }

  // Current event being built
  std::string current_event_type;
  std::string current_data;
  std::string current_id;
  bool has_data = false;

  std::istringstream stream(raw);
  std::string line;

  while (std::getline(stream, line)) {
    // Remove trailing \r if present (handles \r\n line endings)
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Blank line = event boundary
    if (line.empty()) {
      if (has_data) {
        StreamEvent event;
        event.event_type = current_event_type.empty() ? "message" : current_event_type;
        event.data = current_data;
        event.id = current_id;
        event.peer_name = peer;
        events.push_back(std::move(event));
      }
      // Reset for next event
      current_event_type.clear();
      current_data.clear();
      current_id.clear();
      has_data = false;
      continue;
    }

    // Skip comment lines (starting with ':')
    if (line[0] == ':') {
      continue;
    }

    // Parse field: value
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      // Field with no value - ignored per SSE spec
      continue;
    }

    std::string field = line.substr(0, colon_pos);
    std::string value;
    if (colon_pos + 1 < line.size()) {
      // Skip optional single space after colon
      size_t value_start = colon_pos + 1;
      if (value_start < line.size() && line[value_start] == ' ') {
        value_start++;
      }
      value = line.substr(value_start);
    }

    if (field == "event") {
      current_event_type = value;
    } else if (field == "data") {
      if (has_data) {
        current_data += "\n";
      }
      current_data += value;
      has_data = true;
    } else if (field == "id") {
      current_id = value;
    }
    // "retry" and unknown fields are ignored
  }

  // Handle trailing event without final blank line
  if (has_data) {
    StreamEvent event;
    event.event_type = current_event_type.empty() ? "message" : current_event_type;
    event.data = current_data;
    event.id = current_id;
    event.peer_name = peer;
    events.push_back(std::move(event));
  }

  return events;
}

}  // namespace ros2_medkit_gateway
