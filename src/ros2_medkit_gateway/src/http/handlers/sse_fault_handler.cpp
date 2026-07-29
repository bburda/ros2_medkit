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

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
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

  RCLCPP_INFO(HandlerContext::logger(), "SSE fault handler initialized, subscribed to %s, max_clients=%zu",
              fault_events_topic.c_str(), client_tracker_->max_clients());
}

SSEFaultHandler::~SSEFaultHandler() {
  // Signal shutdown and wake up any waiting clients
  request_shutdown();
  subscription_.reset();
}

void SSEFaultHandler::request_shutdown() {
  if (shutdown_flag_.exchange(true)) {
    return;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_cv_.notify_all();
}

uint64_t SSEFaultHandler::delivered_watermark_locked() const {
  uint64_t watermark = kNothingOwed;
  for (const auto & cursor : clients_) {
    if (cursor->stalled) {
      continue;  // dead client: its backlog is not backpressure
    }
    watermark = std::min(watermark, cursor->last_delivered);
  }
  return watermark;
}

std::deque<SSEFaultHandler::QueuedEvent>::iterator SSEFaultHandler::find_superseded_locked() {
  std::unordered_map<std::string, std::size_t> newest_index;
  for (std::size_t i = 0; i < event_queue_.size(); ++i) {
    newest_index[event_queue_[i].event.fault.fault_code] = i;
  }
  for (std::size_t i = 0; i < event_queue_.size(); ++i) {
    // An auto-clear cascade lists its symptoms only here; those codes get no
    // event of their own, so this entry is never redundant.
    if (!event_queue_[i].event.auto_cleared_codes.empty()) {
      continue;
    }
    if (newest_index[event_queue_[i].event.fault.fault_code] != i) {
      return event_queue_.begin() + static_cast<std::ptrdiff_t>(i);
    }
  }
  return event_queue_.end();
}

std::size_t SSEFaultHandler::reap_stalled_clients_locked(std::chrono::steady_clock::time_point now) {
  const auto stall_timeout = keepalive_interval_ * kStallIntervals;
  std::size_t reaped = 0;
  for (const auto & cursor : clients_) {
    if (cursor->stalled || (now - cursor->last_progress) <= stall_timeout) {
      continue;
    }
    cursor->stalled = true;  // the stream loop returns false on its next wakeup
    ++reaped;
  }
  return reaped;
}

SSEFaultHandler::EvictionStats SSEFaultHandler::evict_to_capacity_locked() {
  EvictionStats stats;
  if (event_queue_.size() <= kMaxBufferedEvents) {
    return stats;
  }

  stats.reaped = reap_stalled_clients_locked(std::chrono::steady_clock::now());
  const uint64_t watermark = delivered_watermark_locked();

  while (event_queue_.size() > kMaxBufferedEvents) {
    if (event_queue_.front().id <= watermark) {
      event_queue_.pop_front();  // every live client already has it
      continue;
    }
    auto superseded = find_superseded_locked();
    if (superseded != event_queue_.end()) {
      event_queue_.erase(superseded);  // newer state for that fault code survives
      ++stats.coalesced;
      continue;
    }
    event_queue_.pop_front();  // distinct fault codes only: this one is lost
    ++stats.dropped;
  }

  if (stats.dropped > 0) {
    for (const auto & cursor : clients_) {
      if (!cursor->stalled && cursor->last_delivered < event_queue_.back().id) {
        ++stats.slow_clients;
      }
    }
  }
  return stats;
}

void SSEFaultHandler::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  uint64_t event_id = next_event_id_.fetch_add(1);

  // Snapshot entity context before acquiring the queue lock so cache state
  // is pinned to the fault arrival timestamp and the formatting path stays
  // lock-free with respect to the cache.
  auto entity = resolve_entity_context(msg->fault);

  EvictionStats stats;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Add event to queue with resolved entity context
    event_queue_.push_back(QueuedEvent{event_id, *msg, std::move(entity)});
    stats = evict_to_capacity_locked();
  }

  if (stats.reaped > 0) {
    reaped_clients_.fetch_add(stats.reaped);
    RCLCPP_WARN(HandlerContext::logger(),
                "SSE fault stream: %zu client(s) made no progress for %" PRId64
                "ms; treating them as disconnected and reaping",
                stats.reaped, (keepalive_interval_ * kStallIntervals).count());
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
                  "(buffer cap=%zu, %zu coalesced so far, %zu client(s) reaped as dead)",
                  total, stats.slow_clients, kMaxBufferedEvents, coalesced_events_.load(), reaped_clients_.load());
    }
  }

  // Notify all waiting clients
  queue_cv_.notify_all();

  RCLCPP_DEBUG(HandlerContext::logger(), "Received fault event: %s for %s (id=%" PRIu64 ")", msg->event_type.c_str(),
               msg->fault.fault_code.c_str(), event_id);
}

namespace {

/// Parse the Last-Event-ID header; absent / malformed values map to 0 so the
/// client receives every buffered event on connect.
uint64_t parse_last_event_id(std::string_view value) {
  if (value.empty()) {
    return 0;
  }
  try {
    return std::stoull(std::string(value));
  } catch (...) {
    return 0;
  }
}

}  // namespace

void SSEFaultHandler::note_progress_locked(const std::shared_ptr<ClientCursor> & cursor, uint64_t delivered_id) {
  cursor->last_delivered = std::max(cursor->last_delivered, delivered_id);
  cursor->last_progress = std::chrono::steady_clock::now();
}

std::function<bool(httplib::DataSink &)> SSEFaultHandler::make_stream_loop(uint64_t initial_last_event_id) {
  auto cursor = std::make_shared<ClientCursor>();
  cursor->last_delivered = initial_last_event_id;
  cursor->last_progress = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    clients_.push_back(cursor);
  }

  // Deregisters when the content provider (and with it this closure) is
  // destroyed, which is the only point at which the connection is certainly
  // gone for both the typed and the legacy entry.
  auto unregister = std::shared_ptr<void>(nullptr, [this, cursor](void *) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    clients_.erase(std::remove(clients_.begin(), clients_.end(), cursor), clients_.end());
  });

  return [this, cursor, unregister, last_event_id = initial_last_event_id](httplib::DataSink & sink) mutable -> bool {
    // Formatting happens under the lock, writing does not: the buffer now
    // erases from the middle to coalesce, so holding an iterator across a
    // write would be a use-after-free.
    auto collect_pending = [this, &last_event_id]() {
      std::vector<std::pair<uint64_t, std::string>> pending;
      for (const auto & queued : event_queue_) {
        if (queued.id > last_event_id) {
          pending.emplace_back(queued.id, format_sse_event(queued));
        }
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
      std::vector<std::pair<uint64_t, std::string>> pending;
      bool keepalive_due = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (shutdown_flag_.load()) {
          return false;  // Handler is shutting down
        }

        const auto status = queue_cv_.wait_for(lock, timeout);

        if (shutdown_flag_.load()) {
          return false;  // Handler is shutting down
        }
        if (cursor->stalled) {
          return false;  // Reaped as dead while the buffer was under pressure
        }

        pending = collect_pending();
        keepalive_due = pending.empty() && status == std::cv_status::timeout;
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

  auto loop = make_stream_loop(last_event_id);
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

  auto loop = make_stream_loop(last_event_id);

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

size_t SSEFaultHandler::connected_clients() const {
  return client_tracker_->connected_clients();
}

std::size_t SSEFaultHandler::dropped_events() const {
  return dropped_events_.load();
}

std::size_t SSEFaultHandler::coalesced_events() const {
  return coalesced_events_.load();
}

std::size_t SSEFaultHandler::reaped_clients() const {
  return reaped_clients_.load();
}

uint64_t SSEFaultHandler::events_received() const {
  // next_event_id_ starts at 1 and is incremented once per received event in
  // on_fault_event, so (next_event_id_ - 1) is the count consumed so far.
  return next_event_id_.load(std::memory_order_relaxed) - 1;
}

std::string SSEFaultHandler::format_sse_event(const QueuedEvent & queued) {
  const auto sanitized_event_type = sanitize_sse_event_type(queued.event.event_type);

  nlohmann::json json_event;
  json_event["event_type"] = sanitized_event_type;
  json_event["fault"] = ros2::conversions::fault_to_json(queued.event.fault);

  // Convert timestamp to seconds with nanosecond precision
  double timestamp_sec =
      static_cast<double>(queued.event.timestamp.sec) + static_cast<double>(queued.event.timestamp.nanosec) * 1e-9;
  json_event["timestamp"] = timestamp_sec;

  // SOVD payload extension: nest ``entity_type`` / ``entity_id`` under the
  // ``x-medkit`` response-extension object so global-stream consumers can
  // hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}``
  // directly instead of HEAD-probing every entity. Flat ``x-medkit-*``
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
