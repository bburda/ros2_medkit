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
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"
#include "ros2_medkit_gateway/http/alternate_status.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// XMedkitOperationItem - x-medkit vendor extension on OperationItem responses
// (handle_list_operations / handle_get_operation).
//
//   ros2        - nested ROS 2 metadata sub-object (service|action path, type,
//                 kind)
//   entity_id   - SOVD entity the operation belongs to
//   source      - always "ros2_medkit_gateway"
//   type_info   - optional dynamic ROS IDL schema JSON (request/response for
//                 services; goal/result/feedback for actions); kept as
//                 nlohmann::json because the structure is runtime-determined
//                 by type introspection and cannot be statically typed
//   member_ids  - the members of an aggregating entity that provide this
//                 operation; absent on an entity that exposes its own
//   available   - present only as false, marking an item whose provider is not
//                 answering; absent means it can be served
// =============================================================================
struct XMedkitOperationItem {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> entity_id;
  std::optional<std::string> source;
  std::optional<nlohmann::json> type_info;  // free-form: dynamic ROS IDL schemas
  /// Members of the grouping that contribute this item.
  ///
  /// Absent on a leaf entity, where the entity is the only contributor.
  /// A list rather than one id because an item can genuinely have several:
  /// a topic published by one member and subscribed by another is merged
  /// into a single item, and two members can expose the same operation
  /// short name. More than one entry is what makes the bare item id
  /// ambiguous for addressing.
  std::optional<std::vector<std::string>> member_ids;
  /// Absent while the item can be served. False marks an item listed from a
  /// retained declaration because the member that owns it is not answering -
  /// the item is still part of the tree, and still counts towards ambiguity.
  std::optional<bool> available;
};

template <>
inline constexpr auto dto_fields<XMedkitOperationItem> = std::make_tuple(
    field("ros2", &XMedkitOperationItem::ros2), field("entity_id", &XMedkitOperationItem::entity_id),
    field("source", &XMedkitOperationItem::source), field("type_info", &XMedkitOperationItem::type_info),
    field("member_ids", &XMedkitOperationItem::member_ids), field("available", &XMedkitOperationItem::available));

template <>
inline constexpr std::string_view dto_name<XMedkitOperationItem> = "XMedkitOperationItem";

// =============================================================================
// XMedkitOperationExecution - x-medkit vendor extension on OperationExecution
// responses (handle_get_execution).
//
//   goal_id     - UUID of the tracked ROS 2 action goal
//   ros2_status - raw ROS 2 goal status string (accepted|executing|canceling|
//                 succeeded|canceled|aborted|unknown)
//   ros2        - nested ROS 2 metadata sub-object (action path + type)
// =============================================================================
struct XMedkitOperationExecution {
  std::string goal_id;
  std::optional<std::string> ros2_status;  // raw ROS 2 enum string
  std::optional<XMedkitRos2> ros2;
};

template <>
inline constexpr auto dto_fields<XMedkitOperationExecution> =
    std::make_tuple(field("goal_id", &XMedkitOperationExecution::goal_id),
                    field("ros2_status", &XMedkitOperationExecution::ros2_status),
                    field("ros2", &XMedkitOperationExecution::ros2));

template <>
inline constexpr std::string_view dto_name<XMedkitOperationExecution> = "XMedkitOperationExecution";

// =============================================================================
// OperationItem - single item emitted in handle_list_operations "items" array
// and wrapped inside OperationDetail.
//
// Wire keys (from handle_list_operations / handle_get_operation):
//   id                     - operation name (required)
//   name                   - operation name (required)
//   proximity_proof_required - bool (required, always false for ROS 2)
//   asynchronous_execution - bool (required; false for services, true for actions)
//   x-medkit               - typed vendor extension; carries ros2.{service|action,
//                            type,kind}, entity_id, source, optional type_info,
//                            and, on an aggregating entity, member_ids plus
//                            available
// =============================================================================
struct OperationItem {
  std::string id;
  std::string name;
  bool proximity_proof_required{false};
  bool asynchronous_execution{false};
  std::optional<XMedkitOperationItem> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<OperationItem> =
    std::make_tuple(field("id", &OperationItem::id), field("name", &OperationItem::name),
                    field("proximity_proof_required", &OperationItem::proximity_proof_required),
                    field("asynchronous_execution", &OperationItem::asynchronous_execution),
                    field("x-medkit", &OperationItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<OperationItem> = "OperationItem";

// =============================================================================
// OperationDetail - response shape for GET /{entity}/{id}/operations/{op_id}.
//
// Wire keys (from handle_get_operation):
//   item - required; the full OperationItem for this operation
// =============================================================================
struct OperationDetail {
  OperationItem item;
};

template <>
inline constexpr auto dto_fields<OperationDetail> = std::make_tuple(field("item", &OperationDetail::item));

template <>
inline constexpr std::string_view dto_name<OperationDetail> = "OperationDetail";

// =============================================================================
// OperationExecution - execution status response shape.
//
// Used by:
//   - GET  /{entity}/{id}/operations/{op_id}/executions/{exec_id}
//     (handle_get_execution): status + capability + optional parameters +
//     optional x-medkit (goal_id / ros2_status / ros2.action / ros2.type)
//   - POST /{entity}/{id}/operations/{op_id}/executions (202 for actions):
//     id + status
//   - PUT  /{entity}/{id}/operations/{op_id}/executions/{exec_id} (202 stop):
//     id + status
//
// Wire keys:
//   id          - execution / goal UUID (optional; set on 202 responses)
//   status      - execution status enum (required)
//   capability  - control capability in context (optional; "execute" on GET)
//   parameters  - dynamic ROS payload (optional; last_feedback on GET)
//   x-medkit    - vendor extension (optional; goal tracking data on GET)
// =============================================================================
struct OperationExecution {
  std::optional<std::string> id;
  std::string status;  // enum: pending|running|completed|failed
  std::optional<std::string> capability;
  std::optional<nlohmann::json> parameters;           // free-form: last_feedback / result
  std::optional<XMedkitOperationExecution> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<OperationExecution> =
    std::make_tuple(field("id", &OperationExecution::id),
                    field_enum("status", &OperationExecution::status, kOperationExecutionStatusValues),
                    field("capability", &OperationExecution::capability),
                    field("parameters", &OperationExecution::parameters),
                    field("x-medkit", &OperationExecution::x_medkit));

template <>
inline constexpr std::string_view dto_name<OperationExecution> = "OperationExecution";

// =============================================================================
// ExecutionUpdateRequest - PUT request body for execution control.
//
// Wire keys (from handle_update_execution / execution_update_request_schema):
//   capability - required; one of: stop | execute | freeze | reset
// =============================================================================
struct ExecutionUpdateRequest {
  std::string capability;  // enum: stop|execute|freeze|reset
};

template <>
inline constexpr auto dto_fields<ExecutionUpdateRequest> = std::make_tuple(
    field("capability", &ExecutionUpdateRequest::capability,
          "SOVD control capability to apply to the running execution: `stop`, `execute`, `freeze` or `reset`. A ROS 2 "
          "action implements only `stop`, which cancels the goal and answers 202; `execute` answers 409 (cancel first, "
          "then start a new execution) and `freeze` and `reset` answer 400. Described rather than declared as an "
          "`enum` on purpose - the handler answers an unknown value with a 400 that names `supported_capabilities` for "
          "the backend in front of the caller, and a schema-level enum would replace that with a generic "
          "body-validation error. Same reason `LogConfiguration.severity_filter` carries no enum."));

template <>
inline constexpr std::string_view dto_name<ExecutionUpdateRequest> = "ExecutionUpdateRequest";

// =============================================================================
// OperationExecutionResult - typed envelope around the plugin-defined response
// body emitted by OperationProvider::execute_operation.
//
// Wire shape: the bare JSON object the plugin returns. The wrapper is purely a
// C++ ABI affordance; JsonWriter / JsonReader / SchemaWriter are fully
// specialized so the wire bytes are byte-identical to the pre-typed ABI.
//
// The payload shape is runtime-dependent (depends on the operation name and
// the plugin's backend - ROS service / action result, OPC-UA method result,
// UDS service response, ...) and therefore cannot be statically modelled.
// Plugins fill `content` with whatever JSON object they want returned to the
// caller; the gateway emits it verbatim. The schema is opaque
// (`additionalProperties:true`, `x-medkit-opaque:true`) so downstream OpenAPI
// consumers know the shape is plugin-determined.
// =============================================================================
struct OperationExecutionResult {
  nlohmann::json content;  // wire shape: the bare result object
};

template <>
inline constexpr std::string_view dto_name<OperationExecutionResult> = "OperationExecutionResult";

// Mark as opaque DTO so the typed RouteRegistry / schema collector accept the
// type without requiring a `dto_fields` specialization (the wire shape is
// plugin-determined, see the JsonWriter / JsonReader / SchemaWriter
// specializations below).
template <>
inline constexpr bool is_opaque_dto_v<OperationExecutionResult> = true;

// JsonWriter specialization: emit the bare result object, not a wrapper.
template <>
struct JsonWriter<OperationExecutionResult> {
  static nlohmann::json write(const OperationExecutionResult & obj) {
    return obj.content;
  }
};

// JsonReader specialization: wrap any input object into
// OperationExecutionResult::content. Rejects non-object payloads so consumers
// get a structured FieldError. Round-trips JsonWriter byte-for-byte.
template <>
struct JsonReader<OperationExecutionResult> {
  static tl::expected<OperationExecutionResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    OperationExecutionResult out;
    out.content = j;
    return out;
  }
};

// SchemaWriter specialization: free-form opaque object schema. Mirrors the
// `opaque_object` field descriptor (see dto/contract.hpp) at the type level.
template <>
struct SchemaWriter<OperationExecutionResult> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"type", "object"},
        {"additionalProperties", true},
        {"x-medkit-opaque", true},
        {"description",
         "Result of a synchronous operation. The shape belongs to the operation, not to this endpoint: on "
         "the ROS 2 path it is the service response or action result converted field-for-field from its IDL "
         "type, and on a plugin-owned entity it is whatever the OperationProvider returns (an OPC-UA method "
         "result, a UDS service response). Discover the concrete shape from the operation's own document, "
         "`GET /{entity_type}/{entity_id}/operations/{operation_id}`."}};
  }
};

// dto_sample specialization: OperationExecutionResult bypasses the
// field-walking visitors, so the generic for_each_field path cannot produce a
// non-trivial sample. Hand-build one shaped like a typical plugin result
// (status / operation / entity_id) so EveryRegisteredDtoRoundTrips exercises
// the JsonWriter -> JsonReader round-trip on a representative payload.
template <>
struct dto_sample<OperationExecutionResult> {
  static OperationExecutionResult make() {
    OperationExecutionResult obj;
    obj.content = nlohmann::json{{"status", "ok"}, {"operation", "sample_op"}, {"entity_id", "sample_entity"}};
    return obj;
  }
};

// =============================================================================
// Collection<OperationItem> - named "OperationList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<OperationItem>> = "OperationList";

// =============================================================================
// ExecutionId - single item emitted in handle_list_executions "items" array.
//
// Wire shape (from handle_list_executions, Table 172): bare object with only
// the goal UUID. Distinct from `OperationExecution` because the list response
// does not carry status / parameters / x-medkit - those are returned by
// `GET /executions/{exec_id}` instead.
//
// Wire keys:
//   id - execution / goal UUID (required)
// =============================================================================
struct ExecutionId {
  std::string id;
};

template <>
inline constexpr auto dto_fields<ExecutionId> = std::make_tuple(field("id", &ExecutionId::id));

template <>
inline constexpr std::string_view dto_name<ExecutionId> = "ExecutionId";

// =============================================================================
// Collection<ExecutionId> - response shape for
// GET /{entity}/operations/{op-id}/executions. Renamed "OperationExecutionList"
// so the OpenAPI schema name stays stable across the migration (replaces the
// hand-written `items_wrapper_ref("OperationExecution")` in schema_builder).
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<ExecutionId>> = "OperationExecutionList";

// =============================================================================
// ExecutionCreateRequest - POST executions request body.
//
// SOVD treats the body of `POST /executions` as implementation-defined; in
// practice the gateway accepts either:
//   - `parameters` - the ROS service request / action goal payload (preferred,
//                    SOVD-conforming)
//   - `goal`       - legacy alias for actions
//   - `request`    - legacy alias for services
//   - `type`       - optional type override (allows callers to invoke a
//                    different service/action type at the same name)
// All fields are optional - an empty body `{}` is valid and means "no
// parameters, use the discovered type".
//
// Wire keys (all optional):
//   parameters - dynamic ROS payload (free-form JSON)
//   goal       - legacy alias for `parameters` (actions)
//   request    - legacy alias for `parameters` (services)
//   type       - optional ROS 2 service/action type override
// =============================================================================
struct ExecutionCreateRequest {
  std::optional<nlohmann::json> parameters;
  std::optional<nlohmann::json> goal;
  std::optional<nlohmann::json> request;
  std::optional<std::string> type;
};

template <>
inline constexpr auto dto_fields<ExecutionCreateRequest> =
    std::make_tuple(field("parameters", &ExecutionCreateRequest::parameters),
                    field("goal", &ExecutionCreateRequest::goal), field("request", &ExecutionCreateRequest::request),
                    field("type", &ExecutionCreateRequest::type));

template <>
inline constexpr std::string_view dto_name<ExecutionCreateRequest> = "ExecutionCreateRequest";

// =============================================================================
// ExecutionCreateAsync - 202 Accepted response body for POST executions on
// an asynchronous operation (ROS 2 action). The wire shape is intentionally
// the bare OperationExecution subset emitted on the 202 branch of
// handle_create_execution: `{id, status}`. Kept as a separate DTO so the
// `post_alternates` framework knob (`dto_alternate_status`) can pick 202 for
// this alternative and 200 for the synchronous `OperationExecutionResult`
// branch.
//
// Wire keys:
//   id     - execution / goal UUID (required)
//   status - execution status (required; always "running" on this branch)
// =============================================================================
struct ExecutionCreateAsync {
  std::string id;
  std::string status;  // always "running" for the 202 path
};

template <>
inline constexpr auto dto_fields<ExecutionCreateAsync> =
    std::make_tuple(field("id", &ExecutionCreateAsync::id),
                    field_enum("status", &ExecutionCreateAsync::status, kOperationExecutionStatusValues));

template <>
inline constexpr std::string_view dto_name<ExecutionCreateAsync> = "ExecutionCreateAsync";

}  // namespace dto

namespace http {

/// `ExecutionCreateAsync` is the 202 Accepted branch of `post_alternates` on
/// the POST executions route - the synchronous branch (`OperationExecutionResult`)
/// emits 200 by default, the async branch emits 202.
template <>
struct dto_alternate_status<dto::ExecutionCreateAsync> {
  static constexpr int value = 202;
};

}  // namespace http
}  // namespace ros2_medkit_gateway
