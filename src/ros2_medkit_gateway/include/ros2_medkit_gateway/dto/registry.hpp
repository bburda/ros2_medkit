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

#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <utility>

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/auth.hpp"
#include "ros2_medkit_gateway/dto/bulkdata.hpp"
#include "ros2_medkit_gateway/dto/config.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/cyclic_subscriptions.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/entity_capability.hpp"
#include "ros2_medkit_gateway/dto/errors.hpp"
#include "ros2_medkit_gateway/dto/fault_triggers.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"
#include "ros2_medkit_gateway/dto/locks.hpp"
#include "ros2_medkit_gateway/dto/logs.hpp"
#include "ros2_medkit_gateway/dto/operations.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/scripts.hpp"
#include "ros2_medkit_gateway/dto/sse_frames.hpp"
#include "ros2_medkit_gateway/dto/triggers.hpp"
#include "ros2_medkit_gateway/dto/updates.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// The single compile-time list of every named DTO. Each domain header
/// (Phase 2/3) appends its types here. Order is irrelevant.
using AllDtos =
    std::tuple<GenericError, DroppedItem, PeerFailure, XMedkitRos2, XMedkitArea, XMedkitComponent, XMedkitApp,
               XMedkitFunction, XMedkitCollection, EntityCapability, AreaListItem, AreaDetail, ComponentListItem,
               ComponentDetail, AppListItem, AppDetail, FunctionListItem, FunctionDetail, Collection<AreaListItem>,
               Collection<ComponentListItem>, Collection<AppListItem>, Collection<FunctionListItem>, FaultListItem,
               Collection<FaultListItem, FaultListXMedkit>, Collection<FaultListItem, FaultListAggXMedkit>,
               FaultListXMedkit, FaultListAggXMedkit, FaultStatus, FaultItem, FaultEnvironmentData, FaultXMedkit,
               FaultDetail, FaultListResult, FaultDetailResult, FaultClearResult,
               // Fault-trigger (threshold-rule) domain DTOs
               FaultTriggerRule, FaultTriggerRuleCreateRequest, FaultTriggerRuleList, XMedkitOperationItem,
               XMedkitOperationExecution, OperationItem, Collection<OperationItem>, OperationDetail, OperationExecution,
               ExecutionId, Collection<ExecutionId>, ExecutionCreateRequest, ExecutionCreateAsync,
               ExecutionUpdateRequest, OperationExecutionResult,
               // Configuration domain DTOs
               ConfigXMedkitItem, ConfigurationMetaData, ConfigListXMedkit,
               Collection<ConfigurationMetaData, ConfigListXMedkit>, ConfigValueXMedkit, ConfigurationReadValue,
               ConfigurationWriteRequest, ConfigurationDeleteResultItem, ConfigurationDeleteMultiStatus,
               // Data domain DTOs. DataWriteResult is deliberately absent: it is the
               // return type of the plugin-facing DataProvider::write_data ABI, and no
               // route publishes it (PUT .../data/{id} answers with DataValue), so a
               // schema for it would be a type every generated client carries and none
               // can ever receive.
               XMedkitDataItem, DataItem, Collection<DataItem, DataListXMedkit>, DataListXMedkit, DataWriteRequest,
               DataListResult, DataValue,
               // Lock domain DTOs
               Lock, Collection<Lock>, AcquireLockRequest, ExtendLockRequest,
               // Trigger domain DTOs
               Trigger, Collection<Trigger>, TriggerCreateRequest, TriggerUpdateRequest,
               // Cyclic subscription domain DTOs
               CyclicSubscription, Collection<CyclicSubscription>, CyclicSubscriptionCreateRequest,
               CyclicSubscriptionUpdateRequest,
               // SSE frame DTOs - the JSON document inside a frame's `data:` field
               SubscriptionEventFrame, TriggerEventFrame, FaultStreamXMedkit, FaultStreamEvent,
               // Bulk-data domain DTOs
               BulkDataCategoryList, BulkDataDescriptor, Collection<BulkDataDescriptor>,
               // Log domain DTOs
               LogContext, LogEntry, LogListXMedkit, Collection<LogEntry, LogListXMedkit>, LogConfiguration,
               // Script domain DTOs
               ScriptMetadata, HateoasLinks, ScriptList, ScriptExecution, ScriptUploadResponse, ScriptControlRequest,
               ScriptExecutionRequest,
               // Software update domain DTOs
               UpdateList, UpdateDetail, UpdateSubProgress, XMedkitUpdate, UpdateStatus, UpdateRegisterRequest,
               UpdateRegisterResponse,
               // Auth domain DTOs
               AuthCredentials, AuthTokenResponse, AuthRevokeRequest, AuthRevokeResponse, OAuth2Error,
               // Health / Root domain DTOs
               HealthDiscoveryLinking, HealthDiscovery, HealthWarning, Health, VersionInfoVendor, VersionInfoEntry,
               XMedkitVersionInfo, VersionInfo, RootCapabilities, RootAuth, RootTls, RootOverview,
               // Lifecycle domain DTOs
               LifecycleStatusResponse>;

namespace detail {
template <class T>
void collect_one(nlohmann::json & schemas) {
  static_assert(!dto_name<T>.empty(),
                "collect_component_schemas: a DTO in AllDtos is missing a dto_name<T> specialization "
                "(would write its schema under an empty \"\" key and collide with others)");
  schemas[std::string(dto_name<T>)] = SchemaWriter<T>::schema();
}

template <class Tuple, std::size_t... I>
nlohmann::json collect_impl(std::index_sequence<I...> /*seq*/) {
  nlohmann::json schemas = nlohmann::json::object();
  (collect_one<std::tuple_element_t<I, Tuple>>(schemas), ...);
  return schemas;
}
}  // namespace detail

/// Build the components/schemas object from every DTO in AllDtos.
inline nlohmann::json collect_component_schemas() {
  return detail::collect_impl<AllDtos>(std::make_index_sequence<std::tuple_size_v<AllDtos>>{});
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
