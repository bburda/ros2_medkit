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

#include <string_view>

namespace ros2_medkit_gateway {
namespace dto {

/// Entity types (used in entity detail responses).
inline constexpr std::string_view kEntityTypeValues[] = {"area", "component", "app", "function"};

/// Fault severity label (FaultListItem.severity_label). Must include "UNKNOWN":
/// fault_to_json emits it for any severity outside the four known levels
/// (fault_msg_conversions.cpp default branch).
inline constexpr std::string_view kFaultSeverityLabelValues[] = {"INFO", "WARN", "ERROR", "CRITICAL", "UNKNOWN"};

/// Fault aggregated status (fault_detail_schema - status.aggregatedStatus).
inline constexpr std::string_view kFaultAggregatedStatusValues[] = {"active", "passive", "cleared"};

/// Fault status query filter (FaultListQuery.status / FaultEntityListQuery.status
/// / FaultClearQuery.status).
/// Mirrors the values parse_fault_status_param() accepts in http_utils.hpp;
/// any other value yields ERR_INVALID_PARAMETER. The leading entry must be a
/// handler-accepted value (the OpenAPI callability test sends enum[0]).
inline constexpr std::string_view kFaultStatusFilterValues[] = {"pending", "confirmed", "cleared", "healed", "all"};

/// Log aggregation level (log_entry_list_schema - x-medkit.aggregation_level).
inline constexpr std::string_view kLogAggregationLevelValues[] = {"function", "area", "component"};

/// Operation/execution status (operation_execution_schema).
inline constexpr std::string_view kOperationExecutionStatusValues[] = {"pending", "running", "completed", "failed"};

/// Trigger status (trigger_schema).
inline constexpr std::string_view kTriggerStatusValues[] = {"active", "terminated"};

/// Cyclic subscription interval (cyclic_subscription_schema).
inline constexpr std::string_view kCyclicSubscriptionIntervalValues[] = {"fast", "normal", "slow"};

/// Update internal lifecycle phase (XMedkitUpdate.phase / UpdateStatus x-medkit).
inline constexpr std::string_view kUpdatePhaseValues[] = {"none",     "preparing", "prepared", "executing",
                                                          "executed", "failed",    "deleting"};

/// Update status (UpdateStatus.status).
inline constexpr std::string_view kUpdateStatusValues[] = {"pending", "inProgress", "completed", "failed"};

/// Log severity filter (log_configuration_schema).
inline constexpr std::string_view kLogSeverityFilterValues[] = {"debug", "info", "warning", "error", "fatal"};

/// Execution control capability (execution_update_request_schema).
inline constexpr std::string_view kExecutionCapabilityValues[] = {"stop", "execute", "freeze", "reset"};

/// Lifecycle status (LifecycleStatusResponse.status).
inline constexpr std::string_view kLifecycleStatusValues[] = {"ready", "notReady"};

/// Stable warning codes on GET /health. Keep in sync with
/// core/http/warning_codes.hpp, which is the canonical list.
inline constexpr std::string_view kHealthWarningCodeValues[] = {"leaf_id_collision", "unmanifested_nodes"};

/// Values of the manifest's `unmanifested_nodes` setting, as reported by
/// GET /health under discovery.linking. Lower-case; keep in sync with
/// ManifestConfig::policy_to_string.
inline constexpr std::string_view kUnmanifestedPolicyValues[] = {"ignore", "warn", "error", "include_as_orphan"};
/// Fault-trigger comparison operator (FaultTriggerEngine::valid_operator).
inline constexpr std::string_view kFaultTriggerOperatorValues[] = {">", "<", ">=", "<=", "=="};

/// Fault-trigger severity (FaultTriggerEngine::valid_severity). Deliberately
/// NOT kFaultSeverityLabelValues: the engine spells the middle level `WARNING`
/// where a reported fault's label spells it `WARN`, so one table for both would
/// document a value one of the two rejects.
inline constexpr std::string_view kFaultTriggerSeverityValues[] = {"INFO", "WARNING", "ERROR", "CRITICAL"};

}  // namespace dto
}  // namespace ros2_medkit_gateway
