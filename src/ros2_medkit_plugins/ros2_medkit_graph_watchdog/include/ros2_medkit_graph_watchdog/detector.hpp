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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_graph_watchdog/fault_request.hpp"
#include "ros2_medkit_graph_watchdog/presence_ownership.hpp"

namespace ros2_medkit_gateway {
struct IntrospectionInput;
// Only named in the test-only transport factory signature below, so a declaration is enough.
// Including the gateway header here would pull it into every consumer of this one, and the test
// targets that do not add GATEWAY_SRC_INCLUDE_DIR would then resolve it from install/ while the
// plugin resolves it from the source tree - the split this package's own CMakeLists warns about.
class ParameterTransport;
}  // namespace ros2_medkit_gateway

namespace ros2_medkit_graph_watchdog {

class WatchdogClock;    // defined in watchdog_clock.hpp
class ReliabilityGate;  // defined in reliability_gate.hpp; only a pointer is stored here.

// Out-of-line so detector.hpp does not need the full ReliabilityGate type (keeps the
// header free of lifecycle_msgs). Defined in reliability_gate.cpp. Returns true when
// gate is null (not yet wired) or the entity is armed + lifecycle-ok.
bool reliability_allows(const ReliabilityGate * gate, const std::string & source_id);

// The stricter sibling of reliability_allows(), same null-gate convention, also defined in
// reliability_gate.cpp: on what GROUNDS the presence detector owns this entity's departure -
// see ReliabilityGate::presence_ownership() and PresenceOwnership. Where reliability_allows()
// answers "may this entity raise" and is permissive about a label that has never been read,
// this answers "whose node is this" and distinguishes an answer earned from a measurement
// from one granted only because the asking stopped.
PresenceOwnership presence_ownership(const ReliabilityGate * gate, const std::string & source_id);

/// QoS for subscribing to /tf_static: publishers latch with transient-local
/// durability, so a late subscriber must match it to receive the static transforms.
inline rclcpp::QoS tf_static_qos() {
  return rclcpp::QoS(1).transient_local();
}

/// Raise = push faults; Advisory = observe but do not push; Off =
/// skipped by the tick loop (never constructed into the active set).
enum class DetectorMode { Raise, Advisory, Off };

/// Single decision point for whether a detector's findings become faults.
inline bool mode_emits(DetectorMode mode) {
  return mode == DetectorMode::Raise;
}

/// Handed to a detector on every tick. Detectors read the ROS graph and raise or
/// clear faults via raise_fault()/clear_fault() rather than the public fault_client member.
struct DetectorContext {
  rclcpp::Node * gateway_node = nullptr;   ///< Graph introspection + subscription host.
  std::mutex * node_mutex = nullptr;       ///< Hold when creating subscriptions on gateway_node.
  WatchdogClock * clock = nullptr;         ///< sim_time-aware now().
  const ReliabilityGate * gate = nullptr;  ///< Central reliability gate; null until wired by the plugin.
  DetectorMode mode = DetectorMode::Raise;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr fault_client;
  const ros2_medkit_gateway::IntrospectionInput * snapshot =
      nullptr;                                    ///< entities this tick (id + bound_fqn); null in bare-context tests
  const std::atomic<bool> * cancelled = nullptr;  ///< plugin shutdown flag; a long sweep polls it (null => never)

  /// Returns true only once `async_send_request` has actually been called - false for every
  /// suppression path (Advisory/Off, no client, empty source_id, reliability gate, service
  /// not ready). This proves the request was handed to rclcpp's client library for sending,
  /// nothing more: the client is deliberately fire-and-forget (see
  /// GraphWatchdogPlugin::set_context()'s own note on why - nothing consumes the future), so
  /// a true return does NOT prove fault_manager received or processed the request, only that
  /// this call was not one of the silent-decline paths above. A caller that needs to
  /// distinguish "genuinely attempted" from "merely warranted" (report non-empty) - not
  /// receipt - must use this return value rather than inferring it from its own inputs:
  /// node_death_detector.cpp's ever_raised_ guard is exactly that caller, and every one of
  /// these suppression paths is silent by design (no detector should have to duplicate them
  /// to know whether it may later trust its own silence as a clear).
  bool raise_fault(const std::string & code, uint8_t severity, const std::string & description,
                   const std::string & source_id) {
    if (!mode_emits(mode) || !fault_client) {
      return false;  // Advisory/Off suppressed, or client not yet wired.
    }
    if (source_id.empty()) {
      if (gateway_node) {
        RCLCPP_WARN_ONCE(gateway_node->get_logger(),
                         "graph_watchdog: dropping fault '%s' with empty source_id (detector contract violation)",
                         code.c_str());
      }
      return false;
    }
    if (!reliability_allows(gate, source_id)) {
      return false;  // entity warming up or lifecycle-inactive: suppressed by the reliability core.
    }
    if (!fault_client->service_is_ready()) {
      return false;  // fault_manager not reachable yet; avoid unbounded pending_requests_ growth.
    }
    fault_client->async_send_request(std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>(
        make_fault_report(source_id, code, severity, description)));
    return true;
  }

  /// See raise_fault()'s own doc on the return value - identical contract, minus the
  /// reliability-gate check clear_fault() has never applied.
  bool clear_fault(const std::string & code, const std::string & source_id) {
    if (!mode_emits(mode) || !fault_client) {
      return false;  // Advisory/Off suppressed, or client not yet wired.
    }
    if (source_id.empty()) {
      if (gateway_node) {
        RCLCPP_WARN_ONCE(gateway_node->get_logger(),
                         "graph_watchdog: dropping fault-clear '%s' with empty source_id (detector contract violation)",
                         code.c_str());
      }
      return false;
    }
    if (!fault_client->service_is_ready()) {
      return false;  // fault_manager not reachable yet; avoid unbounded pending_requests_ growth.
    }
    fault_client->async_send_request(
        std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>(make_fault_clear(source_id, code)));
    return true;
  }
};

/// A silent-fault detector. adds one subclass per file in src/detectors/.
class Detector {
 public:
  virtual ~Detector() = default;
  virtual std::string id() const = 0;  ///< e.g. "qos_mismatch".
  virtual void configure(const nlohmann::json & /*config*/) {
  }
  virtual void tick(DetectorContext & ctx) = 0;  ///< Observe; raise/clear via ctx.

  /// Test-only observability hook: how many identities this detector's internal
  /// tracker currently holds (0 = not applicable / no bounded tracker). Lets
  /// integration tests assert a tracker's map stays bounded under prune without
  /// exposing the tracker itself through the public interface. Default 0 for
  /// detectors with no such tracker.
  virtual std::size_t tracked_count_for_test() const {
    return 0;
  }

  /// Detector-scoped status, surfaced under `detectors.<id>` on GET /x-medkit-watchdog next
  /// to the reliability gate's own. For a condition that belongs to ONE detector and would
  /// otherwise exist only in the gateway log - which no HTTP client and no e2e assertion can
  /// read. Null (the default) means the detector has nothing to say and no key is added.
  ///
  /// Called on an HTTP handler thread while tick() runs on the plugin's tick thread, and the
  /// handler holds no lock a detector takes, so an implementation must build its payload from
  /// atomics (or other independently synchronised state) and must not block.
  virtual nlohmann::json status_json() const {
    return nullptr;
  }

  /// Test-only injection hook for detectors that read parameters over a transport. Returns false
  /// when the detector does not use one, so a test can assert it reached the right detector rather
  /// than silently doing nothing.
  ///
  /// It lives on the base because detectors are file-local and self-registering: a test only ever
  /// holds a `Detector*` from the registry, so there is no derived type to cast to. The alternative
  /// is leaving the behaviours that only a stand-in transport can produce - a read still in flight
  /// when its app is de-armed, a SUCCESS response carrying no parameters, a transport that fails to
  /// construct - untested, and none of them is reachable from a live ROS graph.
  using ParameterTransportFactory =
      std::function<std::unique_ptr<ros2_medkit_gateway::ParameterTransport>(rclcpp::Node *)>;
  virtual bool set_parameter_transport_factory_for_test(const ParameterTransportFactory & /*factory*/) {
    return false;
  }
};

}  // namespace ros2_medkit_graph_watchdog
