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

#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"

#include <utility>

namespace ros2_medkit_gateway {

FaultManager::FaultManager(std::shared_ptr<FaultServiceTransport> transport) : transport_(std::move(transport)) {
}

FaultResult FaultManager::report_fault(const std::string & fault_code, uint8_t severity,
                                       const std::string & description, const std::string & source_id) {
  return transport_->report_fault(fault_code, severity, description, source_id);
}

FaultResult FaultManager::list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                                      bool include_cleared, bool include_healed, bool include_muted,
                                      bool include_clusters) {
  return transport_->list_faults(source_id, include_prefailed, include_confirmed, include_cleared, include_healed,
                                 include_muted, include_clusters);
}

FaultWithEnvJsonResult FaultManager::get_fault_with_env(const std::string & fault_code, const std::string & source_id) {
  return transport_->get_fault_with_env(fault_code, source_id);
}

FaultResult FaultManager::get_fault(const std::string & fault_code, const std::string & source_id) {
  return transport_->get_fault(fault_code, source_id);
}

FaultResult FaultManager::clear_fault(const std::string & fault_code, bool skip_correlation_auto_clear) {
  return transport_->clear_fault(fault_code, skip_correlation_auto_clear);
}

FaultResult FaultManager::get_snapshots(const std::string & fault_code, const std::string & topic) {
  return transport_->get_snapshots(fault_code, topic);
}

FaultResult FaultManager::get_rosbag(const std::string & fault_code) {
  return transport_->get_rosbag(fault_code);
}

FaultResult FaultManager::list_rosbags(const std::string & entity_fqn) {
  return transport_->list_rosbags(entity_fqn);
}

bool FaultManager::is_available() const {
  return transport_->is_available();
}

bool FaultManager::wait_for_services(std::chrono::duration<double> timeout) {
  return transport_->wait_for_services(timeout);
}

// ---------------------------------------------------------------------------
// Planned-stop windows
// ---------------------------------------------------------------------------

PlannedStopResult FaultManager::declare_planned_stop(int64_t from_ns, int64_t to_ns, const std::string & reason,
                                                     const std::string & declared_by) {
  faults::PlannedStopWindow request;
  request.from_ns = from_ns;
  request.to_ns = to_ns;
  request.reason = reason;
  request.declared_by = declared_by;

  auto result = transport_->declare_planned_stop(request);
  if (result.success) {
    planned_stop_cache_.invalidate();
  }
  return result;
}

PlannedStopResult FaultManager::end_planned_stop(const std::string & id) {
  auto result = transport_->end_planned_stop(id);
  if (result.success) {
    planned_stop_cache_.invalidate();
  }
  return result;
}

PlannedStopResult FaultManager::list_planned_stops(bool active_only) {
  return transport_->list_planned_stops(active_only);
}

std::vector<faults::PlannedStopWindow> FaultManager::planned_stop_windows(bool force_refresh) {
  const auto ttl = planned_stop_cache_.last_refresh_failed() ? kPlannedStopFailureBackoff : kPlannedStopCacheTtl;
  if (force_refresh || planned_stop_cache_.is_stale(ttl)) {
    // Checked before the call, not instead of it: list_planned_stops blocks for
    // the transport's whole timeout when the services are absent, and this read
    // sits on the fault-stream formatting path.
    auto fetched =
        transport_->is_available() ? transport_->list_planned_stops(/*active_only=*/false) : PlannedStopResult{};
    if (fetched.success) {
      planned_stop_cache_.store(std::move(fetched.stops));
    } else {
      // Keep what is known - a fault that briefly loses its flag reads as a
      // surprise, and a surprise is the one thing an expected fault must not
      // look like - but stop asking for a while.
      planned_stop_cache_.note_failed_refresh();
    }
  }
  return planned_stop_cache_.snapshot();
}

bool FaultManager::planned_stops_known() const {
  return planned_stop_cache_.has_knowledge();
}

void FaultManager::invalidate_planned_stop_cache() {
  planned_stop_cache_.invalidate();
}

}  // namespace ros2_medkit_gateway
