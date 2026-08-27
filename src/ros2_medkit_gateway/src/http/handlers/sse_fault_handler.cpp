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

#include "ros2_medkit_gateway/http/handlers/sse_fault_handler.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

std::string sanitize_sse_event_type(std::string event_type) {
  event_type.erase(std::remove(event_type.begin(), event_type.end(), '\r'), event_type.end());
  event_type.erase(std::remove(event_type.begin(), event_type.end(), '\n'), event_type.end());
  return event_type;
}

}  // namespace

SSEFaultHandler::SSEFaultHandler(HandlerContext & ctx, std::shared_ptr<SSEClientTracker> client_tracker)
  : SSEFaultHandler(ctx, std::move(client_tracker), std::chrono::seconds(kKeepaliveIntervalSec)) {
}

SSEFaultHandler::SSEFaultHandler(HandlerContext & ctx, std::shared_ptr<SSEClientTracker> client_tracker,
                                 std::chrono::milliseconds keepalive_interval)
  : ctx_(ctx)
  , client_tracker_(std::move(client_tracker))
  , keepalive_interval_(keepalive_interval > std::chrono::milliseconds::zero()
                            ? keepalive_interval
                            : std::chrono::seconds(kKeepaliveIntervalSec)) {
  if (keepalive_interval <= std::chrono::milliseconds::zero()) {
    RCLCPP_WARN(HandlerContext::logger(),
                "Non-positive SSE keepalive override %" PRId64 "ms rejected; using default %ds",
                keepalive_interval.count(), kKeepaliveIntervalSec);
  }

  const auto fault_events_topic = build_fault_manager_events_topic(ctx_.node());

  // Create subscription to fault events topic
  // Use fully qualified topic name since FaultManager publishes on ~/events
  subscription_ = ctx_.node()->create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
      fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
        on_fault_event(msg);
      });

  // The aggregation manager is looked up per call rather than captured here:
  // it is wired onto the context during gateway start-up and this handler is
  // built in the same pass, so a pointer taken now can be the null one.
  peer_relay_ = std::make_unique<PeerFaultRelay>(
      [this]() {
        std::vector<RelayTarget> targets;
        auto * agg = ctx_.aggregation_manager();
        if (agg == nullptr) {
          return targets;
        }
        for (const auto & endpoint : agg->healthy_peer_endpoints()) {
          // No Authorization. A relay is one connection shared by every local
          // client, so there is no single client whose token it could carry,
          // and sending the one that happened to open it would serve every
          // later client events fetched with somebody else's credentials.
          // A peer that requires authentication therefore refuses the relay.
          targets.push_back(RelayTarget{endpoint.name, endpoint.url, ""});
        }
        return targets;
      },
      // The peer's own route, not this gateway's configured prefix: what is
      // being addressed is a ros2_medkit gateway on the other side, which is
      // what PeerClient assumes everywhere else too.
      std::string(API_BASE_PATH) + "/faults/stream",
      [this](const StreamEvent & event) {
        on_peer_event(event);
      });

  RCLCPP_INFO(HandlerContext::logger(), "SSE fault handler initialized, subscribed to %s, max_clients=%zu",
              fault_events_topic.c_str(), client_tracker_->max_clients());
}

SSEFaultHandler::~SSEFaultHandler() {
  // Signal shutdown and wake up any waiting clients
  request_shutdown();
  subscription_.reset();
  // Before the queue-lock wait below: closing a peer stream joins its reader
  // thread, and that thread may be inside on_peer_event holding nothing but
  // wanting queue_mutex_.
  if (peer_relay_) {
    peer_relay_->shutdown();
  }

  // The per-stream unregister deleter dereferences this handler (it locks
  // queue_mutex_ and erases from clients_) and runs whenever the framework
  // destroys the content provider. Hold destruction until every closure is
  // gone so that deleter can never run on a dead handler.
  std::unique_lock<std::mutex> lock(queue_mutex_);
  queue_cv_.wait(lock, [this] {
    return clients_.empty();
  });
}

void SSEFaultHandler::request_shutdown() {
  if (peer_relay_) {
    peer_relay_->shutdown();
  }
  if (shutdown_flag_.exchange(true)) {
    return;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_cv_.notify_all();
}

std::optional<uint64_t> SSEFaultHandler::delivered_watermark_locked() const {
  std::optional<uint64_t> watermark;
  for (const auto & cursor : clients_) {
    watermark = watermark ? std::min(*watermark, cursor->last_delivered) : cursor->last_delivered;
  }
  return watermark;
}

std::deque<SSEFaultHandler::QueuedEvent>::iterator SSEFaultHandler::find_superseded_locked(bool updates_only) {
  // Keyed on the peer as well as the code: a fault code is unique on the
  // gateway that raised it and nowhere else, so two peers reporting the same
  // code are reporting two faults, and treating one as the newer state of the
  // other would delete a live fault from a client's view.
  auto supersede_key = [](const QueuedEvent & queued) {
    return queued.peer + '\0' + queued.event.fault.fault_code;
  };
  std::unordered_map<std::string, std::size_t> newest_index;
  for (std::size_t i = 0; i < event_queue_.size(); ++i) {
    newest_index[supersede_key(event_queue_[i])] = i;
  }
  for (std::size_t i = 0; i < event_queue_.size(); ++i) {
    const auto & entry = event_queue_[i].event;
    const auto key = supersede_key(event_queue_[i]);
    // An auto-clear cascade lists its symptoms only here; those codes get no
    // event of their own, so this entry is never redundant.
    if (!entry.auto_cleared_codes.empty()) {
      continue;
    }
    if (updates_only && entry.event_type != ros2_medkit_msgs::msg::FaultEvent::EVENT_UPDATED) {
      continue;
    }
    if (newest_index[key] != i) {
      return event_queue_.begin() + static_cast<std::ptrdiff_t>(i);
    }
  }
  return event_queue_.end();
}

bool SSEFaultHandler::has_pending_locked(uint64_t last_event_id, bool relay_peers) const {
  if (relay_peers) {
    return !event_queue_.empty() && event_queue_.back().id > last_event_id;
  }
  // A suppressed client is owed only what this gateway saw itself, so the
  // newest entry is not enough to answer: the tail may be entirely relayed.
  for (auto it = event_queue_.rbegin(); it != event_queue_.rend(); ++it) {
    if (it->id <= last_event_id) {
      break;
    }
    if (it->peer.empty()) {
      return true;
    }
  }
  return false;
}

SSEFaultHandler::EvictionStats SSEFaultHandler::evict_to_capacity_locked() {
  EvictionStats stats;
  if (event_queue_.size() <= kMaxBufferedEvents) {
    return stats;
  }

  const auto watermark = delivered_watermark_locked();
  uint64_t newest_lost = 0;

  while (event_queue_.size() > kMaxBufferedEvents) {
    if (!watermark || event_queue_.front().id <= *watermark) {
      event_queue_.pop_front();  // nobody attached, or every live client already has it
      continue;
    }
    auto superseded = find_superseded_locked(/*updates_only=*/true);
    if (superseded != event_queue_.end()) {
      event_queue_.erase(superseded);  // newer state survives, no transition erased
      ++stats.coalesced;
      continue;
    }
    // Something a live client is owed has to go. Prefer an entry a newer
    // same-code event supersedes: the current state still reaches the client
    // even though the transition history does not. Either way it is a real,
    // counted loss.
    auto victim = find_superseded_locked(/*updates_only=*/false);
    if (victim == event_queue_.end()) {
      victim = event_queue_.begin();
    }
    newest_lost = std::max(newest_lost, victim->id);
    event_queue_.erase(victim);
    ++stats.dropped;
  }

  if (stats.dropped > 0) {
    for (const auto & cursor : clients_) {
      if (cursor->last_delivered < newest_lost) {
        ++stats.slow_clients;  // this client was owed at least one lost event
      }
    }
  }
  return stats;
}

void SSEFaultHandler::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  // Snapshot entity context before acquiring the queue lock so cache state
  // is pinned to the fault arrival timestamp and the formatting path stays
  // lock-free with respect to the cache.
  auto entity = resolve_entity_context(msg->fault);
  QueuedEvent queued;
  queued.event = *msg;
  queued.entity = std::move(entity);
  enqueue(std::move(queued));
}

void SSEFaultHandler::on_peer_event(const StreamEvent & event) {
  auto payload = nlohmann::json::parse(event.data, nullptr, false);
  if (payload.is_discarded() || !payload.is_object()) {
    RCLCPP_WARN(HandlerContext::logger(), "Discarding unparseable fault event relayed from peer '%s'",
                event.peer_name.c_str());
    return;
  }

  // The peer names the fault and the transition; both are read back out so the
  // replay buffer's eviction and coalescing work on a relayed event exactly as
  // they do on a local one, without either of them learning about peers.
  // Every read below is type-checked before it is taken. nlohmann's value()
  // throws type_error when the stored value is of another type than the
  // default, and the peer chooses these types: a fault event carrying a
  // numeric event_type would throw out of the relay's reader thread.
  QueuedEvent queued;
  queued.peer = event.peer_name;
  const auto & type_field = payload.find("event_type");
  queued.event.event_type =
      (type_field != payload.end() && type_field->is_string()) ? type_field->get<std::string>() : event.event_type;
  if (payload.contains("fault") && payload["fault"].is_object()) {
    const auto & fault = payload["fault"];
    if (fault.contains("fault_code") && fault["fault_code"].is_string()) {
      queued.event.fault.fault_code = fault["fault_code"].get<std::string>();
    }
  }
  // The eviction pass never discards an entry that carries a cascade, because
  // those symptom codes get no event of their own and exist nowhere else.
  // Reading them back is what puts a relayed cascade under the same protection
  // as a local one.
  if (payload.contains("auto_cleared_codes") && payload["auto_cleared_codes"].is_array()) {
    for (const auto & code : payload["auto_cleared_codes"]) {
      if (code.is_string()) {
        queued.event.auto_cleared_codes.push_back(code.get<std::string>());
      }
    }
  }

  // Attribution goes next to whatever the peer already put under x-medkit,
  // which for a fault event is the entity it resolved. Overwriting the object
  // would throw that away.
  if (!payload.contains("x-medkit") || !payload["x-medkit"].is_object()) {
    payload["x-medkit"] = nlohmann::json::object();
  }
  payload["x-medkit"]["peer"] = event.peer_name;
  queued.relayed_payload = payload.dump();

  enqueue(std::move(queued));
}

void SSEFaultHandler::enqueue(QueuedEvent queued) {
  const std::string event_type = queued.event.event_type;
  const std::string fault_code = queued.event.fault.fault_code;

  uint64_t event_id = 0;
  EvictionStats stats;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // The id is claimed under the same lock that appends, so the buffer stays
    // ordered by id. Claiming it outside was safe while the ROS subscription
    // was the only producer; a relay adds one producer thread per peer, and two
    // of them interleaving between the claim and the append would leave a
    // lower id behind a higher one. Every consumer reads the deque in order and
    // compares against a single `last_event_id`, so out-of-order entries are
    // delivered twice to one client and never to another.
    event_id = next_event_id_.fetch_add(1);
    queued.id = event_id;

    event_queue_.push_back(std::move(queued));
    stats = evict_to_capacity_locked();
  }

  if (stats.coalesced > 0) {
    coalesced_events_.fetch_add(stats.coalesced);
  }

  // Surface real SSE backpressure without spamming the log: the first loss, then
  // one WARN per kDropLogEveryN. Buffer rotation with no client waiting on the
  // evicted event is not counted at all.
  if (stats.dropped > 0) {
    const auto previous = dropped_events_.fetch_add(stats.dropped);
    const auto total = previous + stats.dropped;
    if (previous == 0 || (total / kDropLogEveryN) > (previous / kDropLogEveryN)) {
      RCLCPP_WARN(HandlerContext::logger(),
                  "SSE fault event lost: %zu event(s) dropped total for %zu slow client(s) "
                  "(buffer cap=%zu, %zu coalesced so far)",
                  total, stats.slow_clients, kMaxBufferedEvents, coalesced_events_.load());
    }
  }

  // Notify all waiting clients
  queue_cv_.notify_all();

  RCLCPP_DEBUG(HandlerContext::logger(), "Received fault event: %s for %s (id=%" PRIu64 ")", event_type.c_str(),
               fault_code.c_str(), event_id);
}

namespace {

/// Parse the Last-Event-ID header; absent / malformed values map to 0 so the
/// client receives every buffered event on connect. Digits only: std::stoull
/// alone would accept "-1" and wrap it to UINT64_MAX.
uint64_t parse_last_event_id(std::string_view value) {
  if (value.empty() || value.find_first_not_of("0123456789") != std::string_view::npos) {
    return 0;
  }
  try {
    return std::stoull(std::string(value));
  } catch (...) {
    return 0;  // out of range
  }
}

}  // namespace

void SSEFaultHandler::note_progress_locked(const std::shared_ptr<ClientCursor> & cursor, uint64_t delivered_id) {
  cursor->last_delivered = std::max(cursor->last_delivered, delivered_id);
}

std::function<bool(httplib::DataSink &)> SSEFaultHandler::make_stream_loop(uint64_t initial_last_event_id,
                                                                           bool relay_peers) {
  // Clamp to the newest id issued so far: a Last-Event-ID above it (any bogus
  // value, e.g. "18446744073709551615") would otherwise leave collect_pending
  // permanently empty (blind stream, no keepalives under steady traffic) and
  // seed the delivery watermark with an id nobody was ever sent, evicting the
  // whole buffer as "already delivered".
  initial_last_event_id = std::min(initial_last_event_id, next_event_id_.load() - 1);

  auto cursor = std::make_shared<ClientCursor>();
  cursor->last_delivered = initial_last_event_id;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    clients_.push_back(cursor);
  }

  // Deregisters when the content provider (and with it this closure) is
  // destroyed, which is the only point at which the connection is certainly
  // gone for both the typed and the legacy entry.
  // The guard is built BEFORE acquire(), which opens threads and can throw:
  // a throw after the cursor is registered but before the guard exists would
  // leave that cursor in clients_ for ever, and ~SSEFaultHandler waits for
  // clients_ to empty.
  auto unregister = std::shared_ptr<void>(nullptr, [this, cursor, relay_peers](void *) {
    // The relay goes first, and outside the queue lock. Outside, because
    // closing a peer stream joins a reader thread that may be waiting for
    // exactly that lock inside on_peer_event. First, because the erase below
    // is what releases ~SSEFaultHandler from its wait, and the destructor is
    // free to destroy peer_relay_ the moment it returns - a release() running
    // after that erase would be reaching into a destroyed member.
    if (relay_peers && peer_relay_) {
      peer_relay_->release();
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    clients_.erase(std::remove(clients_.begin(), clients_.end(), cursor), clients_.end());
    queue_cv_.notify_all();  // ~SSEFaultHandler may be waiting for the last closure
  });

  if (relay_peers && peer_relay_) {
    peer_relay_->acquire();
  }

  return [this, cursor, unregister, relay_peers,
          last_event_id = initial_last_event_id](httplib::DataSink & sink) mutable -> bool {
    // Formatting happens under the lock, writing does not: the buffer now
    // erases from the middle to coalesce, so holding an iterator across a
    // write would be a use-after-free.
    auto collect_pending = [this, &last_event_id, relay_peers]() {
      std::vector<std::pair<uint64_t, std::string>> pending;
      for (const auto & queued : event_queue_) {
        if (queued.id <= last_event_id) {
          continue;
        }
        // Suppression has to bite HERE, not only where the relay is opened.
        // The buffer is shared by every client, so a relayed event put there
        // for one client would otherwise be delivered to a client that asked
        // for the local graph only - which is what an aggregating peer asks
        // for, and is what would carry an event round a chain of them.
        if (!relay_peers && !queued.peer.empty()) {
          // Still counts as delivered: the id is not owed to this client, and
          // leaving it behind would hold the watermark down for everyone.
          last_event_id = queued.id;
          continue;
        }
        pending.emplace_back(queued.id, format_sse_event(queued));
      }
      return pending;
    };

    auto flush = [this, &sink, &cursor, &last_event_id](const std::vector<std::pair<uint64_t, std::string>> & pending) {
      for (const auto & [id, sse_msg] : pending) {
        if (!sink.write(sse_msg.data(), sse_msg.size())) {
          return false;  // Client disconnected
        }
        last_event_id = id;
      }
      std::lock_guard<std::mutex> lock(queue_mutex_);
      note_progress_locked(cursor, last_event_id);
      return true;
    };

    // First, send any buffered events the client missed (for reconnection)
    {
      std::vector<std::pair<uint64_t, std::string>> pending;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pending = collect_pending();
      }
      if (!pending.empty() && !flush(pending)) {
        return false;
      }
    }

    // Wait for new events or keepalive timeout
    const auto timeout = keepalive_interval_;

    while (true) {
      // A peer that appeared or went away since the last wakeup. Cheap - a
      // comparison of two small name sets - and it removes the need for a
      // timer of its own, because the loop already wakes on every event and at
      // least once per keepalive interval.
      if (relay_peers && peer_relay_) {
        peer_relay_->reconcile();
      }

      std::vector<std::pair<uint64_t, std::string>> pending;
      bool keepalive_due = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        // Predicate form: an event enqueued between the last flush and this
        // wait already spent its notify_all while nobody was waiting; a plain
        // wait_for would sleep the full keepalive interval on top of it.
        const bool woke = queue_cv_.wait_for(lock, timeout, [this, &last_event_id, relay_peers] {
          return shutdown_flag_.load() || has_pending_locked(last_event_id, relay_peers);
        });

        if (shutdown_flag_.load()) {
          return false;  // Handler is shutting down
        }

        pending = collect_pending();
        keepalive_due = !woke && pending.empty();
      }

      if (keepalive_due) {
        const char * keepalive = ":keepalive\n\n";
        if (!sink.write(keepalive, strlen(keepalive))) {
          return false;  // Client disconnected
        }
        // A completed keepalive write is proof the connection is alive even
        // when there is nothing to deliver.
        std::lock_guard<std::mutex> lock(queue_mutex_);
        note_progress_locked(cursor, last_event_id);
        continue;
      }

      if (pending.empty()) {
        continue;  // Spurious wakeup
      }
      if (!flush(pending)) {
        return false;
      }
    }

    return true;
  };
}

http::Result<http::SseStream> SSEFaultHandler::sse_stream(const http::TypedRequest & req) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  if (!client_tracker_->try_connect()) {
    RCLCPP_WARN(HandlerContext::logger(), "SSE client limit reached (%zu), rejecting connection from %s",
                client_tracker_->max_clients(), raw_req.remote_addr.c_str());
    ErrorInfo err;
    err.code = ERR_SERVICE_UNAVAILABLE;
    err.message = "Maximum number of SSE clients reached. Please try again later.";
    err.http_status = 503;
    return tl::make_unexpected(std::move(err));
  }

  RCLCPP_INFO(HandlerContext::logger(), "SSE fault client connected from %s (%zu/%zu)", raw_req.remote_addr.c_str(),
              client_tracker_->connected_clients(), client_tracker_->max_clients());

  const uint64_t last_event_id = parse_last_event_id(req.header("Last-Event-ID").value_or(std::string{}));

  // The framework's `reg.sse` wrapper drives the chunked content provider and
  // calls `next_event` until it returns false. We pair the loop with a
  // tracker-release shared_ptr so the per-client counter decrements when the
  // stream terminates - the framework does not expose a disconnect callback
  // analogous to the legacy `handle_stream`'s 3-arg overload.
  auto release_guard = std::shared_ptr<void>(nullptr, [this, addr = raw_req.remote_addr](void *) {
    client_tracker_->disconnect();
    RCLCPP_INFO(HandlerContext::logger(), "SSE fault client disconnected from %s", addr.c_str());
  });

  // An aggregating gateway relaying from another sends X-Medkit-No-Fan-Out,
  // the same header the collection routes use to stop a request going round a
  // chained or bidirectional peering. Serve that request from the local graph
  // only.
  const bool relay_peers = !req.header("X-Medkit-No-Fan-Out").has_value();
  auto loop = make_stream_loop(last_event_id, relay_peers);
  http::SseStream stream;
  stream.next_event = [loop = std::move(loop), release_guard](httplib::DataSink & sink) mutable {
    return loop(sink);
  };
  return stream;
}

void SSEFaultHandler::handle_stream(const httplib::Request & req, httplib::Response & res) {
  // Check if we're at the combined SSE client limit before accepting connection
  if (!client_tracker_->try_connect()) {
    RCLCPP_WARN(HandlerContext::logger(), "SSE client limit reached (%zu), rejecting connection from %s",
                client_tracker_->max_clients(), req.remote_addr.c_str());
    // The legacy raw-route entry calls the framework primitive directly via
    // the friend gate; the HandlerContext public send_* surface has been
    // pruned and the typed `sse_stream` path is the production route.
    ErrorInfo err;
    err.code = ERR_SERVICE_UNAVAILABLE;
    err.message = "Maximum number of SSE clients reached. Please try again later.";
    err.http_status = 503;
    http::detail::write_generic_error(http::detail::FrameworkOrPluginAccess{}, res, err);
    return;
  }

  RCLCPP_INFO(HandlerContext::logger(), "SSE fault client connected from %s (%zu/%zu)", req.remote_addr.c_str(),
              client_tracker_->connected_clients(), client_tracker_->max_clients());

  const uint64_t last_event_id =
      req.has_header("Last-Event-ID") ? parse_last_event_id(req.get_header_value("Last-Event-ID")) : 0;

  // Set SSE headers (the typed `reg.sse` path wires Cache-Control and
  // X-Accel-Buffering automatically; this legacy entry preserves the historic
  // header set including Content-Type and Connection: keep-alive that the
  // in-process test fixture asserts on).
  res.set_header("Content-Type", "text/event-stream");
  res.set_header("Cache-Control", "no-cache");
  res.set_header("Connection", "keep-alive");
  res.set_header("X-Accel-Buffering", "no");  // Disable nginx buffering

  auto loop = make_stream_loop(last_event_id, !req.has_header("X-Medkit-No-Fan-Out"));

  // Use chunked content provider for streaming
  res.set_chunked_content_provider(
      "text/event-stream",
      [loop = std::move(loop)](size_t /*offset*/, httplib::DataSink & sink) mutable {
        return loop(sink);
      },
      [this, addr = req.remote_addr](bool success) {
        client_tracker_->disconnect();
        RCLCPP_INFO(HandlerContext::logger(), "SSE fault client disconnected from %s (success=%d)", addr.c_str(),
                    success);
      });
}

std::size_t SSEFaultHandler::relayed_peer_streams() const {
  return peer_relay_ ? peer_relay_->open_streams() : 0;
}

size_t SSEFaultHandler::connected_clients() const {
  return client_tracker_->connected_clients();
}

std::size_t SSEFaultHandler::dropped_events() const {
  return dropped_events_.load();
}

std::size_t SSEFaultHandler::coalesced_events() const {
  return coalesced_events_.load();
}

uint64_t SSEFaultHandler::events_received() const {
  // next_event_id_ starts at 1 and is incremented once per received event in
  // on_fault_event, so (next_event_id_ - 1) is the count consumed so far.
  return next_event_id_.load(std::memory_order_relaxed) - 1;
}

std::string SSEFaultHandler::format_sse_event(const QueuedEvent & queued) {
  const auto sanitized_event_type = sanitize_sse_event_type(queued.event.event_type);

  // A relayed event is the peer's own payload. It is emitted under THIS
  // gateway's id, because the id is what Last-Event-ID resumes from and two
  // peers number their events independently - a peer's id would address a
  // position in a stream this gateway does not keep.
  if (!queued.relayed_payload.empty()) {
    std::ostringstream relayed;
    relayed << "id: " << queued.id << "\n";
    relayed << "event: " << sanitized_event_type << "\n";
    relayed << "data: " << queued.relayed_payload << "\n\n";
    return relayed.str();
  }

  nlohmann::json json_event;
  json_event["event_type"] = sanitized_event_type;
  json_event["fault"] = ros2::conversions::fault_to_json(queued.event.fault);

  // Convert timestamp to seconds with nanosecond precision
  double timestamp_sec =
      static_cast<double>(queued.event.timestamp.sec) + static_cast<double>(queued.event.timestamp.nanosec) * 1e-9;
  json_event["timestamp"] = timestamp_sec;

  // Correlation payload: symptom codes auto-cleared with this event's root
  // cause. These codes get no event of their own, so this is the only place
  // a stream consumer learns about the cascade. Omitted when empty.
  if (!queued.event.auto_cleared_codes.empty()) {
    json_event["auto_cleared_codes"] = queued.event.auto_cleared_codes;
  }

  // SOVD payload extension: nest ``entity_type`` / ``entity_id`` under the
  // ``x-medkit`` response-extension object so global-stream consumers can
  // hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}``
  // directly instead of HEAD-probing every entity. That address is a
  // compatibility one since #620 - bags are keyed by recording id now - and
  // serves the fault's newest recording, which is what a stream consumer
  // reacting to the event it just received wants. Flat ``x-medkit-*``
  // names are reserved for endpoint paths (``/x-medkit-graph``) and error
  // codes, not payload fields.
  if (queued.entity) {
    json_event["x-medkit"] = {{"entity_type", queued.entity->type}, {"entity_id", queued.entity->id}};
  }

  std::ostringstream sse;
  sse << "id: " << queued.id << "\n";
  sse << "event: " << sanitized_event_type << "\n";
  sse << "data: " << json_event.dump() << "\n\n";

  return sse.str();
}

std::optional<SSEFaultHandler::EntityContext>
SSEFaultHandler::resolve_entity_context(const ros2_medkit_msgs::msg::Fault & fault) const {
  if (fault.reporting_sources.empty()) {
    return std::nullopt;
  }
  // reporting_sources is a set; debounced faults can carry several co-reporters
  // (e.g. node_a and node_b raising the same fault_code). .front() picks the
  // lexicographically-first FQN, not a defined owner - any co-reporter's
  // rosbag is fetchable, so this remains a valid hint, just not authoritative.
  const auto & raw_fqn = fault.reporting_sources.front();
  if (raw_fqn.empty()) {
    return std::nullopt;
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();

  // Manifest / hybrid mode: the linking step populated node_to_app with the
  // ROS FQN -> manifest app id mapping. Try both FQN forms (with and without
  // the leading '/'), mirroring gateway_node's node_resolver lambda.
  std::string entity_id = cache.resolve_node_to_app(raw_fqn);
  if (entity_id.empty() && raw_fqn.front() == '/') {
    entity_id = cache.resolve_node_to_app(raw_fqn.substr(1));
  }

  // Runtime fallback: synthetic apps are created with id = ROS node name
  // (the FQN's last segment) when there is no namespace collision, or
  // ``<ns_prefix>_<name>`` (slashes in the namespace replaced with '_') when
  // multiple nodes share the same name. See ros2_runtime_introspection.cpp.
  // Only accept a candidate that actually exists as an App in the cache so
  // we never point consumers at a 404.
  if (entity_id.empty()) {
    auto last_slash = raw_fqn.rfind('/');
    auto name = (last_slash != std::string::npos) ? raw_fqn.substr(last_slash + 1) : raw_fqn;
    if (!name.empty() && cache.has_app(name)) {
      entity_id = std::move(name);
    } else if (last_slash != std::string::npos && last_slash > 0) {
      // Try the collision-disambiguated form: ns prefix (sans leading '/'),
      // slashes replaced with '_', then '_' + name.
      auto ns_prefix = raw_fqn.substr(1, last_slash - 1);
      std::replace(ns_prefix.begin(), ns_prefix.end(), '/', '_');
      auto namespaced = ns_prefix + "_" + name;
      if (cache.has_app(namespaced)) {
        entity_id = std::move(namespaced);
      }
    }
  }

  if (entity_id.empty()) {
    RCLCPP_DEBUG(HandlerContext::logger(),
                 "SSE fault event: no entity match for reporting source '%s' (fault_code='%s'); "
                 "omitting x-medkit extension",
                 raw_fqn.c_str(), fault.fault_code.c_str());
    return std::nullopt;
  }

  // entity_type is hardcoded "apps" because apps are the leaf reporters in
  // SOVD - reporting_sources always carries ROS node FQNs which map to apps.
  // Components own faults transitively via their hosted apps; consumers can
  // walk up the hierarchy via /apps/<id> -> belongs_to if they need the
  // owning component. Manifest-only components without a bound node have no
  // FQN match here and fall back to plain discovery - by design.
  return EntityContext{"apps", std::move(entity_id)};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
