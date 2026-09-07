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

#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_types.hpp"
#include "ros2_medkit_gateway/core/faults/planned_stop.hpp"

namespace ros2_medkit_gateway {

/// Port for the seven services provided by the ros2_medkit_fault_manager
/// package. The transport wraps `rclcpp::Client<ros2_medkit_msgs::srv::*>`,
/// converts message types to JSON internally, and returns the neutral
/// FaultResult / FaultWithEnvJsonResult structs already filled.
/// Why the fault manager refused a planned-stop request, when it refused one.
///
/// Structural, not textual: the five refusals map onto three different HTTP
/// answers, and the fault manager is free to reword its messages. Reading the
/// prose to decide a status is how a rewording becomes an outage.
enum class PlannedStopRefusal : uint8_t {
  None,            ///< Not a refusal
  InvalidRequest,  ///< The interval or an instant was not one the API accepts
  DuplicateId,     ///< A window with that id already exists
  NotRetained,     ///< Accepted but not kept: the configured bound had no room
  NotFound,        ///< No window with that id
  AlreadyEnded     ///< The window had already ended; its end is not rewritable
};

/// Outcome of a planned-stop operation. `stops` carries the single stored window
/// for a declare or an end, and every window for a list; it stays empty on any
/// failure.
///
/// `failure` defaults to `Declined` for the same reason FaultResult's does: a
/// producer that reports a failure without classifying it must not be able to
/// manufacture a 503. Only a transport that got no answer at all asks for one.
struct PlannedStopResult {
  bool success{false};
  std::vector<faults::PlannedStopWindow> stops;
  std::string error_message;
  FaultFailure failure{FaultFailure::Declined};
  PlannedStopRefusal refusal{PlannedStopRefusal::None};
};

class FaultServiceTransport {
 public:
  FaultServiceTransport() = default;
  FaultServiceTransport(const FaultServiceTransport &) = delete;
  FaultServiceTransport & operator=(const FaultServiceTransport &) = delete;
  FaultServiceTransport(FaultServiceTransport &&) = delete;
  FaultServiceTransport & operator=(FaultServiceTransport &&) = delete;
  virtual ~FaultServiceTransport() = default;

  virtual FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                                   const std::string & source_id) = 0;

  virtual FaultResult list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                                  bool include_cleared, bool include_healed, bool include_muted,
                                  bool include_clusters) = 0;

  virtual FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code, const std::string & source_id) = 0;

  virtual FaultResult get_fault(const std::string & fault_code, const std::string & source_id) = 0;

  /// Clear a fault by its fault_code.
  /// `skip_correlation_auto_clear`, when true, asks the fault manager to NOT
  /// cascade-clear correlated symptom fault codes. Per-entity DELETE routes
  /// set this to true so the clear cannot reach faults reported by apps
  /// outside the addressed entity via the correlation graph.
  virtual FaultResult clear_fault(const std::string & fault_code, bool skip_correlation_auto_clear = false) = 0;

  virtual FaultResult get_snapshots(const std::string & fault_code, const std::string & topic) = 0;

  virtual FaultResult get_rosbag(const std::string & fault_code) = 0;

  virtual FaultResult list_rosbags(const std::string & entity_fqn) = 0;

  /// Declare a planned-stop window. Only `from_ns`, `to_ns`, `reason` and
  /// `declared_by` are read from @p request; the id and the declaration time are
  /// the fault manager's to assign, and come back on the result.
  virtual PlannedStopResult declare_planned_stop(const faults::PlannedStopWindow & request) = 0;

  /// Cut a window short at the fault manager's current wall clock.
  virtual PlannedStopResult end_planned_stop(const std::string & id) = 0;

  /// Every declared window, or only those covering the fault manager's current
  /// wall clock when @p active_only is set.
  virtual PlannedStopResult list_planned_stops(bool active_only) = 0;

  virtual bool wait_for_services(std::chrono::duration<double> timeout) = 0;

  virtual bool is_available() const = 0;

  /// Whether the planned-stop services are ready, specifically.
  ///
  /// Separate from is_available(), which gates only the four core fault
  /// services: a fault manager from before planned stops answers those and has
  /// no `~/list_planned_stops`, and asking it anyway cost a full service timeout
  /// on the fault-list path, once per back-off window, for nothing.
  virtual bool planned_stops_available() const = 0;
};

}  // namespace ros2_medkit_gateway
