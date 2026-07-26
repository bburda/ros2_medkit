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

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Zero-config freeze-frames for plugin-backed entities.
 *
 * Plugin entities (PLC apps bridged by protocol plugins) report faults under
 * their bare SOVD entity id, and their live values are not ROS topics - the
 * fault_manager's snapshot capture can never freeze-frame them. This class
 * closes that gap on the gateway side: it subscribes to fault_manager events
 * and, when a fault confirms with a plugin-owned reporting source, snapshots
 * that entity's current data values (DataProvider::list_data, which serves
 * from the plugin's latest polled values) at fault time.
 *
 * The frames are merged into the fault detail's environment_data.snapshots
 * only when the fault_manager captured no freeze-frame itself - a configured
 * freeze-frame always wins, while a rosbag-only capture does not suppress the
 * entity frames (see FaultHandlers::merge_entity_freeze_frames).
 *
 * Retention mirrors the fault_manager's freeze-frame semantics: frames are
 * kept across EVENT_CLEARED (the confirmed-state record stays attached to
 * the cleared fault's detail) and overwritten on every EVENT_CONFIRMED, so
 * a re-occurrence re-samples the plugin at its own confirm time.
 */
class EntityFreezeFrameCapture {
 public:
  /// One captured frame: the entity's data values at fault-confirm time.
  struct Frame {
    std::string entity_id;
    nlohmann::json values;  ///< compact {resource_id: value} dict
    int64_t captured_at_ns{0};
  };

  /// Resolves an entity id to its owning plugin's DataProvider (nullptr when
  /// the entity is not plugin-owned). Called from the internal capture thread.
  using DataProviderResolver = std::function<DataProvider *(const std::string & entity_id)>;

  /// Fallback for plugins without a DataProvider: fetches the entity's current
  /// values by dispatching the owning plugin's own `x-plc-data` route
  /// in-process (typically wraps PluginManager::fetch_entity_data_via_route).
  /// Returns the parsed route response, nullopt when unavailable. Called from
  /// the internal capture thread, concurrently with HTTP threads serving the
  /// same route - dispatched handlers must be thread-safe.
  using RouteDataFetcher = std::function<std::optional<nlohmann::json>(const std::string & entity_id)>;

  /// One standing fault: its code and the entities that reported it.
  struct StandingFault {
    std::string fault_code;
    std::vector<std::string> reporting_sources;
  };

  /// Lists the faults that are already confirmed when this object starts, so
  /// their missed confirm edge can still be framed. Called once, on the capture
  /// thread (it may block on a service becoming available), never after the
  /// destructor has joined that thread - so it must not outlive its owner.
  using StandingFaultLister = std::function<std::vector<StandingFault>()>;

  /**
   * @param node ROS 2 node used to resolve the fault-events topic name and logger
   * @param exec shared subscription executor; the fault-events subscription is
   *        created and torn down on its serial worker (issue #375 invariant).
   *        The subscription callback only enqueues - plugin calls run on a
   *        dedicated capture thread so a slow read never stalls that worker.
   * @param resolver entity-to-DataProvider resolver (typically wraps PluginManager)
   * @param route_fetcher x-plc-data route fallback for entities whose plugin
   *        has no DataProvider (the commercial PLC bridges); may be null
   * @param max_faults retained-frame bound; oldest fault's frames evicted past it
   */
  EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                           DataProviderResolver resolver, RouteDataFetcher route_fetcher = nullptr,
                           size_t max_faults = 256, StandingFaultLister standing_lister = nullptr);

  ~EntityFreezeFrameCapture();

  // Non-copyable, non-movable
  EntityFreezeFrameCapture(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture & operator=(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture(EntityFreezeFrameCapture &&) = delete;
  EntityFreezeFrameCapture & operator=(EntityFreezeFrameCapture &&) = delete;

  /// Frames captured for a fault code (empty when none). Thread-safe.
  std::vector<Frame> frames_for(const std::string & fault_code) const;

  /// Build the compact {resource_id: value} dict from a DataProvider::list_data
  /// response. Items without a "value" field map to null; a response without an
  /// "items" array is kept verbatim (plugin-defined shape). Total: malformed
  /// items (non-string id/name) are skipped, never thrown on.
  static nlohmann::json values_from_list_content(const nlohmann::json & content);

  /// True when list-data-shaped content (DataProvider::list_data or x-plc-data
  /// route) carries real live values: not flagged disconnected, with a
  /// non-empty items array. A PLC that is down must not freeze-frame a row
  /// of nulls.
  static bool content_has_live_data(const nlohmann::json & content);

  /// True when a compact values dict holds at least one non-null value.
  /// Rejects the {} / all-null rows a cold poll cache or dead link yields.
  static bool values_have_data(const nlohmann::json & values);

 private:
  /// Subscription-worker side: filter for EVENT_CONFIRMED and enqueue only.
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  /// capture_thread_ main loop: drains queued confirm events.
  void capture_worker();

  /// Frame the faults that were already confirmed when this object came up:
  /// plugins start reporting while the gateway is still wiring the capture, so
  /// a device that is in fault at boot confirms to nobody. Frames are
  /// process-local, so this restores what the missed edge would have produced
  /// rather than duplicating anything. Runs on capture_thread_.
  void capture_standing_faults();

  /// Per-event capture (all plugin calls happen here, on capture_thread_).
  void capture_for_event(const ros2_medkit_msgs::msg::FaultEvent & event);

  /// Build a frame from list-data-shaped content, enforcing the shared
  /// no-row-of-nulls invariant on both capture paths.
  std::optional<Frame> frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                          const nlohmann::json & content);

  /// Capture via the plugin's own x-plc-data route (no DataProvider exported).
  /// Returns nullopt when the route yields nothing usable.
  std::optional<Frame> capture_via_route(const std::string & entity_id, const std::string & fault_code);

  /// Log a fallback failure once per fault code (faults re-confirm on every
  /// clear/re-report cycle; one line per code is enough for an operator).
  void log_fallback_failure_once(const std::string & fault_code, const std::string & message);

  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> subscription_slot_;
  DataProviderResolver resolver_;
  RouteDataFetcher route_fetcher_;
  rclcpp::Logger logger_;
  const size_t max_faults_;

  /// Guards frames_, insertion_order_ and fallback_logged_: capture thread
  /// writes, HTTP handler threads read.
  mutable std::mutex mutex_;
  /// Keyed by fault_code only: cross-entity isolation relies on the
  /// fault_manager keeping reporting_sources append-only for a code and on
  /// get_fault gating by source scope. A per-source clear upstream would need
  /// per-entity eviction here too.
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;          ///< eviction order (FIFO)
  std::unordered_set<std::string> fallback_logged_;  ///< fault codes already warned about (bounded)

  /// Confirm events pending capture: fed by the subscription worker, drained
  /// by capture_thread_. Bounded - oldest event dropped when full.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr> queue_;
  bool stop_{false};
  std::thread capture_thread_;

  /// Lists faults already confirmed at construction; run once on that thread.
  StandingFaultLister standing_lister_;
};

}  // namespace ros2_medkit_gateway
