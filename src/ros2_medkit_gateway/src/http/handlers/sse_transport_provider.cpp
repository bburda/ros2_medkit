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

#include "ros2_medkit_gateway/core/http/handlers/sse_transport_provider.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {

SseTransportProvider::SseTransportProvider(SubscriptionManager & sub_mgr,
                                           std::shared_ptr<SSEClientTracker> client_tracker)
  : sub_mgr_(sub_mgr), client_tracker_(std::move(client_tracker)) {
}

tl::expected<std::string, std::string> SseTransportProvider::start(const CyclicSubscriptionInfo & info,
                                                                   ResourceSamplerFn json_sampler,
                                                                   GatewayNode * /*node*/) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    streams_[info.id] = StreamState{std::move(json_sampler)};
  }

  // SSE event_source is the events endpoint URL
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/cyclic-subscriptions/" +
         info.id + "/events";
}

void SseTransportProvider::notify_update(const std::string & sub_id) {
  // SSE transport re-reads from sub_mgr_ each iteration, so no-op here.
  (void)sub_id;
}

void SseTransportProvider::stop(const std::string & sub_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  streams_.erase(sub_id);
  // The in-flight SSE streaming loop exits naturally because:
  // 1. stop() is called from on_removed callback, after state->active is set to false
  // 2. The loop checks sub_mgr_.is_active() each iteration, which returns false
  // 3. Erasing from streams_ prevents new client connections for this subscription
}

tl::expected<http::SseStream, ErrorInfo> SseTransportProvider::make_sse_stream(const std::string & sub_id) {
  ResourceSamplerFn sampler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = streams_.find(sub_id);
    if (it == streams_.end()) {
      ErrorInfo err;
      err.code = ERR_RESOURCE_NOT_FOUND;
      err.message = "Subscription stream not found";
      err.http_status = 404;
      err.params = json{{"subscription_id", sub_id}};
      return tl::make_unexpected(std::move(err));
    }
    sampler = it->second.sampler;
  }

  // Check combined SSE client limit. The tracker counts SSE clients across all
  // SSE-emitting routes (cyclic-subscriptions, triggers, faults, ...). Mirroring
  // the legacy handle_client_connect path, exhaustion produces a 503 SOVD
  // GenericError that the typed wrapper renders for the framework.
  if (!client_tracker_->try_connect()) {
    RCLCPP_WARN(handlers::HandlerContext::logger(),
                "SSE client limit reached (%zu), rejecting cyclic subscription stream", client_tracker_->max_clients());
    ErrorInfo err;
    err.code = ERR_SERVICE_UNAVAILABLE;
    err.message = "Maximum number of SSE clients reached. Please try again later.";
    err.http_status = 503;
    return tl::make_unexpected(std::move(err));
  }

  RCLCPP_INFO(handlers::HandlerContext::logger(), "SSE cyclic subscription client connected (%zu/%zu)",
              client_tracker_->connected_clients(), client_tracker_->max_clients());

  // The tracker_guard releases the SSE client slot on closure destruction. The
  // framework holds the SseStream via a shared_ptr, so the guard's deleter
  // fires when the chunked content provider stops (either client disconnect or
  // end-of-stream). This matches the disconnect callback the legacy
  // set_chunked_content_provider used.
  auto tracker = client_tracker_;
  // Intentional copy: both the tracker_guard deleter and the stream.next_event
  // lambda need their own owned copies because sub_id is a reference that does
  // not outlive this function.
  std::string captured_sub_id = sub_id;  // NOLINT(performance-unnecessary-copy-initialization)
  std::shared_ptr<void> tracker_guard(nullptr, [tracker, captured_sub_id](void *) {
    tracker->disconnect();
    RCLCPP_DEBUG(rclcpp::get_logger("rest_server"), "SSE cyclic subscription stream disconnected: %s",
                 captured_sub_id.c_str());
  });

  http::SseStream stream;
  stream.next_event = [this, captured_sub_id, sampler = std::move(sampler), tracker_guard,
                       last_write = std::chrono::steady_clock::now()](httplib::DataSink & sink) mutable -> bool {
    // Check if subscription is still active. Mirrors the legacy outer while-true
    // loop: the framework calls next_event in a loop until it returns false, so
    // a single pass per call is sufficient.
    if (!sub_mgr_.is_active(captured_sub_id)) {
      return false;
    }

    // Send keepalive if no data was written recently
    auto keepalive_timeout = std::chrono::seconds(kKeepaliveIntervalSec);
    auto since_last_write = std::chrono::steady_clock::now() - last_write;
    if (since_last_write >= keepalive_timeout) {
      const char * keepalive = ":keepalive\n\n";
      if (!sink.write(keepalive, std::strlen(keepalive))) {
        return false;
      }
      last_write = std::chrono::steady_clock::now();
    }

    // Get current subscription info (may have been updated via PUT)
    auto current_sub = sub_mgr_.get(captured_sub_id);
    if (!current_sub) {
      return false;
    }

    auto sample_interval = interval_to_duration(current_sub->interval);

    // Sample the resource via the registered sampler
    json envelope;

    // UTC timestamp in ISO 8601 format
    auto now = std::chrono::system_clock::now();
    auto time_t_val = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream ts;
    struct tm tm_buf;
    gmtime_r(&time_t_val, &tm_buf);
    ts << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    ts << "." << std::setw(3) << std::setfill('0') << ms << "Z";
    envelope["timestamp"] = ts.str();

    try {
      auto sample_result = sampler(current_sub->entity_id, current_sub->resource_path);
      if (sample_result.has_value()) {
        envelope["payload"] = *sample_result;
      } else {
        json error;
        // The vendor-error sentinel is only usable with the code it stands in
        // for: on its own it tells a client a vendor failure happened and not
        // which one. Every other emitter of the sentinel pairs the two
        // (primitives.cpp, peer_client.cpp, aggregation_manager.cpp); this one
        // did not, and `SubscriptionEventFrame.error` now publishes the pair as
        // its declared shape.
        error["error_code"] = ERR_VENDOR_ERROR;
        error["vendor_code"] = ERR_X_MEDKIT_RESOURCE_SAMPLE_FAILED;
        error["message"] = sample_result.error();
        envelope["error"] = error;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("sse_transport"), "Sampler exception for sub %s: %s", captured_sub_id.c_str(),
                  e.what());
      json error;
      error["error_code"] = ERR_INTERNAL_ERROR;
      error["message"] = "Internal error during resource sampling";
      envelope["error"] = error;
    }

    // Format SSE data frame
    std::string sse_msg = "data: " + envelope.dump() + "\n\n";
    if (!sink.write(sse_msg.data(), sse_msg.size())) {
      return false;
    }
    last_write = std::chrono::steady_clock::now();

    // Wait for interval or notification before yielding back to the framework.
    sub_mgr_.wait_for_update(captured_sub_id, sample_interval);
    return true;
  };

  return stream;
}

}  // namespace ros2_medkit_gateway
