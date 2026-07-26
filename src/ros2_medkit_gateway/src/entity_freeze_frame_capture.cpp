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

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"

#include <chrono>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Capture backlog bound: a confirm burst beyond this drops the oldest event.
constexpr size_t kMaxQueuedEvents = 64;

/// fallback_logged_ bound: past this the set resets (re-arming one warn per
/// code) instead of growing forever on churny/synthetic fault codes.
constexpr size_t kMaxLoggedFaultCodes = 1024;

/// Read a string field totally: json::value() throws type_error.302 when the
/// key is present but not a string, and plugin content is untrusted.
std::string string_field(const nlohmann::json & item, const char * field) {
  const auto it = item.find(field);
  return it != item.end() && it->is_string() ? it->get<std::string>() : std::string();
}

}  // namespace

EntityFreezeFrameCapture::EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                                                   DataProviderResolver resolver, RouteDataFetcher route_fetcher,
                                                   size_t max_faults, StandingFaultLister standing_lister)
  : resolver_(std::move(resolver))
  , route_fetcher_(std::move(route_fetcher))
  , logger_(node->get_logger())
  , max_faults_(max_faults > 0 ? max_faults : 1)
  , standing_lister_(std::move(standing_lister)) {
  // Resolve the topic from the gateway node (it owns fault_manager.namespace);
  // the subscription itself is created on the executor's dedicated _sub node so
  // it never races rcl's hash-map on the main node (issue #375).
  const auto fault_events_topic = build_fault_manager_events_topic(node);
  auto slot = ros2_common::Ros2SubscriptionSlot::create_typed<ros2_medkit_msgs::msg::FaultEvent>(
      exec, fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](std::shared_ptr<const ros2_medkit_msgs::msg::FaultEvent> msg) {
        on_fault_event(msg);
      });
  if (!slot) {
    RCLCPP_ERROR(logger_, "EntityFreezeFrameCapture: failed to subscribe to %s: %s", fault_events_topic.c_str(),
                 slot.error().c_str());
    return;
  }
  subscription_slot_ = std::move(*slot);
  capture_thread_ = std::thread([this] {
    capture_worker();
  });

  RCLCPP_INFO(logger_, "EntityFreezeFrameCapture initialized, subscribed to %s", fault_events_topic.c_str());
}

EntityFreezeFrameCapture::~EntityFreezeFrameCapture() {
  // Not a synchronous barrier: the slot posts an async destroy with a bounded
  // deadline, so use-after-free safety also relies on the owner tearing down
  // the subscription executor (joining its worker) before this object's node
  // dies - keep that ordering in main.cpp / gateway_node shutdown.
  subscription_slot_.reset();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  // Joins through any in-flight plugin call: a hung read delays shutdown
  // rather than leaving a capture racing plugin unload.
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

std::vector<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::frames_for(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = frames_.find(fault_code);
  return it != frames_.end() ? it->second : std::vector<Frame>{};
}

nlohmann::json EntityFreezeFrameCapture::values_from_list_content(const nlohmann::json & content) {
  if (!content.contains("items") || !content["items"].is_array()) {
    return content;
  }
  nlohmann::json values = nlohmann::json::object();
  for (const auto & item : content["items"]) {
    if (!item.is_object()) {
      continue;
    }
    std::string key = string_field(item, "id");
    if (key.empty()) {
      key = string_field(item, "name");
    }
    if (key.empty()) {
      continue;
    }
    values[key] = item.contains("value") ? item["value"] : nlohmann::json{};
  }
  return values;
}

bool EntityFreezeFrameCapture::content_has_live_data(const nlohmann::json & content) {
  if (!content.is_object()) {
    return false;
  }
  if (content.contains("connected") && content["connected"].is_boolean() && !content["connected"].get<bool>()) {
    return false;
  }
  return content.contains("items") && content["items"].is_array() && !content["items"].empty();
}

bool EntityFreezeFrameCapture::values_have_data(const nlohmann::json & values) {
  if (values.is_object()) {
    for (const auto & entry : values.items()) {
      if (!entry.value().is_null()) {
        return true;
      }
    }
    return false;
  }
  return !values.is_null();
}

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                             const nlohmann::json & content) {
  if (!content_has_live_data(content)) {
    log_fallback_failure_once(fault_code, "entity '" + entity_id + "' reported no live values");
    return std::nullopt;
  }
  Frame frame;
  frame.entity_id = entity_id;
  frame.values = values_from_list_content(content);
  if (!values_have_data(frame.values)) {
    // Items present but nothing usable in them (all-null values, or no usable
    // ids): still a dead/cold row - no frame, same invariant as above.
    log_fallback_failure_once(fault_code, "entity '" + entity_id + "' values are all null");
    return std::nullopt;
  }
  frame.captured_at_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  return frame;
}

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::capture_via_route(const std::string & entity_id, const std::string & fault_code) {
  std::optional<nlohmann::json> content;
  try {
    content = route_fetcher_(entity_id);
  } catch (const std::exception & e) {
    log_fallback_failure_once(fault_code, std::string("x-plc-data dispatch threw: ") + e.what());
    return std::nullopt;
  }
  if (!content) {
    return std::nullopt;  // not plugin-owned, no x-plc-data route, or handler error
  }
  return frame_from_content(entity_id, fault_code, *content);
}

void EntityFreezeFrameCapture::log_fallback_failure_once(const std::string & fault_code, const std::string & message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fallback_logged_.size() >= kMaxLoggedFaultCodes) {
      fallback_logged_.clear();
    }
    if (!fallback_logged_.insert(fault_code).second) {
      return;
    }
  }
  RCLCPP_WARN(logger_, "Entity freeze-frame for fault '%s': %s", fault_code.c_str(), message.c_str());
}

void EntityFreezeFrameCapture::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  // Mirror the fault_manager's own capture trigger: freeze-frames are taken
  // when a fault confirms, not on every update.
  if (msg->event_type != ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED) {
    return;
  }

  // This runs on the shared subscription worker (which also serves all /data
  // sampling and subscribe/unsubscribe), so only enqueue here: the plugin
  // calls run on capture_thread_, where a slow or live read cannot stall the
  // worker and a handler that re-enters the executor cannot self-deadlock.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    if (queue_.size() >= kMaxQueuedEvents) {
      RCLCPP_WARN(logger_, "Entity freeze-frame capture backlog full; dropping oldest confirm event");
      queue_.pop_front();
    }
    queue_.push_back(msg);
  }
  queue_cv_.notify_one();
}

void EntityFreezeFrameCapture::capture_standing_faults() {
  if (!standing_lister_) {
    return;
  }
  // The lister blocks until the fault services answer, so it runs here rather
  // than on a thread of its own: this thread is joined by the destructor, which
  // is what keeps that wait from outliving the node it waits on.
  const auto standing = standing_lister_();
  size_t framed = 0;
  for (const auto & fault : standing) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stop_) {
        return;
      }
    }
    if (fault.fault_code.empty() || fault.reporting_sources.empty()) {
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (frames_.count(fault.fault_code) != 0) {
        continue;  // the live event beat us to it
      }
    }
    ros2_medkit_msgs::msg::FaultEvent event;
    event.event_type = ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED;
    event.fault.fault_code = fault.fault_code;
    event.fault.reporting_sources = fault.reporting_sources;
    capture_for_event(event);
    ++framed;
  }
  if (framed > 0) {
    RCLCPP_INFO(logger_, "Entity freeze-frame: captured %zu fault(s) that were already confirmed at startup", framed);
  }
}

void EntityFreezeFrameCapture::capture_worker() {
  capture_standing_faults();
  for (;;) {
    ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr event;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return stop_ || !queue_.empty();
      });
      if (stop_) {
        // Drop the backlog: the owner destroys this right before plugin
        // shutdown, and a late capture would call into an unloading plugin.
        return;
      }
      event = queue_.front();
      queue_.pop_front();
    }
    capture_for_event(*event);
  }
}

void EntityFreezeFrameCapture::capture_for_event(const ros2_medkit_msgs::msg::FaultEvent & event) {
  const std::string & fault_code = event.fault.fault_code;

  std::vector<Frame> frames;
  for (const auto & source : event.fault.reporting_sources) {
    DataProvider * provider = resolver_ ? resolver_(source) : nullptr;
    if (provider == nullptr) {
      // No DataProvider: the owning plugin may still serve live values through
      // its own x-plc-data route (the commercial PLC bridges). For non-plugin
      // (ROS) sources the fetcher resolves no owner and returns nullopt.
      if (route_fetcher_) {
        if (auto frame = capture_via_route(source, fault_code)) {
          frames.push_back(std::move(*frame));
        }
      }
      continue;
    }

    // list_data is expected to serve from the plugin's latest polled values
    // (cheap); plugin code still gets exception-guarded like every other
    // provider call site.
    try {
      auto result = provider->list_data(source);
      if (!result) {
        log_fallback_failure_once(fault_code, "list_data('" + source + "') failed: " + result.error().message);
        continue;
      }
      if (auto frame = frame_from_content(source, fault_code, result->content)) {
        frames.push_back(std::move(*frame));
      }
    } catch (const std::exception & e) {
      log_fallback_failure_once(fault_code, "plugin threw for entity '" + source + "': " + e.what());
    }
  }

  if (frames.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (frames_.find(fault_code) == frames_.end()) {
    insertion_order_.push_back(fault_code);
    while (frames_.size() >= max_faults_ && !insertion_order_.empty()) {
      frames_.erase(insertion_order_.front());
      insertion_order_.pop_front();
    }
  }
  frames_[fault_code] = std::move(frames);

  RCLCPP_DEBUG(logger_, "Captured entity freeze-frame(s) for fault '%s'", fault_code.c_str());
}

}  // namespace ros2_medkit_gateway
