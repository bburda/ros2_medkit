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

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"
#include "ros2_medkit_gateway/http/alternate_status.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// ConfigXMedkitItem - x-medkit block on each item inside the configurations
// list ("items" array).  Emitted by handle_list_configurations per-parameter.
//
// Wire keys (from config_handlers.cpp per-item x-medkit construction):
//   source  - app_id that owns this parameter (aggregated entities only)
//   node    - node FQN providing this parameter (in all_parameters tracking list)
// =============================================================================
struct ConfigXMedkitItem {
  std::optional<std::string> source;  // app_id that owns this parameter
  std::optional<std::string> node;    // node FQN (only in parameters tracking list)
};

template <>
inline constexpr auto dto_fields<ConfigXMedkitItem> =
    std::make_tuple(field("source", &ConfigXMedkitItem::source), field("node", &ConfigXMedkitItem::node));

template <>
inline constexpr std::string_view dto_name<ConfigXMedkitItem> = "ConfigXMedkitItem";

// =============================================================================
// ConfigurationMetaData - single item in the configurations list response.
// Emitted by handle_list_configurations per discovered parameter.
//
// Wire keys (from config_handlers.cpp handle_list_configurations):
//   id       - unique param ID (app_id:param_name for aggregated, param_name otherwise)
//   name     - parameter name (without app_id prefix)
//   type     - always "parameter"
//   x-medkit - optional; present only for aggregated entities (source=app_id)
// =============================================================================
struct ConfigurationMetaData {
  std::string id;
  std::string name;
  std::string type;                           // always "parameter"
  std::optional<ConfigXMedkitItem> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ConfigurationMetaData> =
    std::make_tuple(field("id", &ConfigurationMetaData::id), field("name", &ConfigurationMetaData::name),
                    field("type", &ConfigurationMetaData::type), field("x-medkit", &ConfigurationMetaData::x_medkit));

template <>
inline constexpr std::string_view dto_name<ConfigurationMetaData> = "ConfigurationMetaData";

// =============================================================================
// ConfigListXMedkit - x-medkit vendor extension on configuration list responses.
// Emitted by handle_list_configurations (root of the response object).
//
//   entity_id         - SOVD entity ID being queried
//   source            - always "runtime"
//   aggregation_level - level string from EntityCache (e.g. "app", "component")
//   is_aggregated     - true when multiple nodes contribute parameters
//   parameters        - full parameter details including value/type/read_only
//                       (free-form: raw ConfigurationManager output + x-medkit)
//   source_ids        - namespace/FQN strings used for node lookup
//   queried_nodes      - list of node FQNs successfully queried
//   unavailable_nodes  - backing node FQNs that were down/unresponsive (503-class)
//   failed_nodes       - backing node FQNs that failed for a non-availability reason
//   partial            - true when a fan-out peer request failed OR a local backing
//                        node did not answer (see unavailable_nodes/failed_nodes)
//   failed_peers       - list of peer addresses that returned errors
//   peer_failures      - why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items - per-peer items dropped due to malformed JSON
//                        (observability for invisible drift)
// =============================================================================
struct ConfigListXMedkit {
  std::optional<std::string> entity_id;
  std::optional<std::string> source;
  std::optional<std::string> aggregation_level;
  std::optional<bool> is_aggregated;
  std::optional<nlohmann::json> parameters;  // free-form: array of raw param JSON
  std::optional<std::vector<std::string>> source_ids;
  std::optional<std::vector<std::string>> queried_nodes;
  std::optional<std::vector<std::string>> unavailable_nodes;
  std::optional<std::vector<std::string>> failed_nodes;
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
  std::optional<std::vector<DroppedItem>> peer_dropped_items;
};

template <>
inline constexpr auto dto_fields<ConfigListXMedkit> = std::make_tuple(
    field("entity_id", &ConfigListXMedkit::entity_id), field("source", &ConfigListXMedkit::source),
    field("aggregation_level", &ConfigListXMedkit::aggregation_level),
    field("is_aggregated", &ConfigListXMedkit::is_aggregated), field("parameters", &ConfigListXMedkit::parameters),
    field("source_ids", &ConfigListXMedkit::source_ids), field("queried_nodes", &ConfigListXMedkit::queried_nodes),
    field("unavailable_nodes", &ConfigListXMedkit::unavailable_nodes),
    field("failed_nodes", &ConfigListXMedkit::failed_nodes), field("partial", &ConfigListXMedkit::partial),
    field("failed_peers", &ConfigListXMedkit::failed_peers), field("peer_failures", &ConfigListXMedkit::peer_failures),
    field("peer_dropped_items", &ConfigListXMedkit::peer_dropped_items));

template <>
inline constexpr std::string_view dto_name<ConfigListXMedkit> = "ConfigListXMedkit";

// =============================================================================
// ConfigValueXMedkit - x-medkit vendor extension on configuration read/write
// value responses.  Emitted by handle_get_configuration and
// handle_set_configuration.
//
//   ros2        - nested ROS 2 metadata sub-object (node FQN)
//   entity_id   - SOVD entity ID
//   source      - always "runtime"
//   parameter   - full raw parameter detail JSON (value, type, read_only, etc.)
//   source_app  - app_id (present for aggregated entities only)
// =============================================================================
struct ConfigValueXMedkit {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> entity_id;
  std::optional<std::string> source;
  std::optional<nlohmann::json> parameter;  // free-form: raw ConfigurationManager output
  std::optional<std::string> source_app;
};

template <>
inline constexpr auto dto_fields<ConfigValueXMedkit> =
    std::make_tuple(field("ros2", &ConfigValueXMedkit::ros2), field("entity_id", &ConfigValueXMedkit::entity_id),
                    field("source", &ConfigValueXMedkit::source), field("parameter", &ConfigValueXMedkit::parameter),
                    field("source_app", &ConfigValueXMedkit::source_app));

template <>
inline constexpr std::string_view dto_name<ConfigValueXMedkit> = "ConfigValueXMedkit";

// =============================================================================
// ConfigurationReadValue - response shape for GET/PUT /{entity}/configurations/{id}.
// Emitted by handle_get_configuration and handle_set_configuration.
//
// Wire keys (from config_handlers.cpp):
//   id       - parameter ID as used in the request
//   data     - parameter value (free-form: any JSON scalar or object)
//   x-medkit - optional vendor extension
// =============================================================================
struct ConfigurationReadValue {
  std::string id;
  nlohmann::json data;                         // free-form: value from ConfigurationManager
  std::optional<ConfigValueXMedkit> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ConfigurationReadValue> =
    std::make_tuple(field("id", &ConfigurationReadValue::id), field("data", &ConfigurationReadValue::data),
                    field("x-medkit", &ConfigurationReadValue::x_medkit));

template <>
inline constexpr std::string_view dto_name<ConfigurationReadValue> = "ConfigurationReadValue";

// =============================================================================
// ConfigurationWriteRequest - PUT request body for /{entity}/configurations/{id}.
// Parsed by handle_set_configuration via parse_body<ConfigurationWriteRequest>.
//
// Wire keys (SOVD convention, from config_handlers.cpp):
//   data  - configuration value to set (free-form: any JSON scalar or object)
//   value - legacy alias accepted as fallback (used by older clients)
//
// At least one of "data" or "value" must be present; handler enforces this
// after parse and prefers "data" when both are supplied.
// =============================================================================
struct ConfigurationWriteRequest {
  std::optional<nlohmann::json> data;   // preferred
  std::optional<nlohmann::json> value;  // legacy alias, accepted as a fallback
};

template <>
inline constexpr auto dto_fields<ConfigurationWriteRequest> =
    std::make_tuple(field("data", &ConfigurationWriteRequest::data), field("value", &ConfigurationWriteRequest::value));

template <>
inline constexpr std::string_view dto_name<ConfigurationWriteRequest> = "ConfigurationWriteRequest";

// The "at least one of data or value" rule is a relation BETWEEN two members,
// and `dto_fields` describes members one at a time, so the derived schema
// cannot carry it: both are optional there and an empty object validates
// against a request `handle_write_configuration` answers 400. A generated
// client would compile that call. The properties still come from the
// descriptor - only the cross-field rule is added here, so a member renamed in
// `dto_fields` cannot leave this stale without also changing the key below.
template <>
struct SchemaWriter<ConfigurationWriteRequest> {
  static nlohmann::json schema() {
    auto derived = derived_object_schema<ConfigurationWriteRequest>();
    // Each branch carries `type: object` as well as its `required`: a DTO
    // schema describes an object body, and a branch that states only a
    // constraint leaves that unsaid.
    derived["anyOf"] =
        nlohmann::json::array({nlohmann::json{{"type", "object"}, {"required", nlohmann::json::array({"data"})}},
                               nlohmann::json{{"type", "object"}, {"required", nlohmann::json::array({"value"})}}});
    derived["description"] =
        "Configuration value to write. At least one of `data` or `value` must be present; `data` is preferred and "
        "wins when both are supplied. `value` is a legacy alias kept for older clients.";
    return derived;
  }
};

// =============================================================================
// ConfigurationDeleteResultItem - single entry in the 207 multi-status results
// array from handle_delete_all_configurations.
//
// Wire keys (from config_handlers.cpp multi_status construction):
//   node     - fully qualified node name
//   app_id   - SOVD app entity ID
//   success  - true if reset succeeded
//   error    - error message (present on failure)
//   details  - additional detail data (present on success if data non-empty)
// =============================================================================
struct ConfigurationDeleteResultItem {
  std::string node;
  std::string app_id;
  bool success{false};
  std::optional<std::string> error;
  std::optional<nlohmann::json> details;  // free-form: reset result data
};

template <>
inline constexpr auto dto_fields<ConfigurationDeleteResultItem> = std::make_tuple(
    field("node", &ConfigurationDeleteResultItem::node), field("app_id", &ConfigurationDeleteResultItem::app_id),
    field("success", &ConfigurationDeleteResultItem::success), field("error", &ConfigurationDeleteResultItem::error),
    field("details", &ConfigurationDeleteResultItem::details));

template <>
inline constexpr std::string_view dto_name<ConfigurationDeleteResultItem> = "ConfigurationDeleteResultItem";

// =============================================================================
// ConfigurationDeleteMultiStatus - 207 Multi-Status response body from
// handle_delete_all_configurations when partial failure occurs.
//
// Wire keys (from config_handlers.cpp):
//   entity_id - SOVD entity ID for which delete was attempted
//   results   - per-node reset outcome list
// =============================================================================
struct ConfigurationDeleteMultiStatus {
  std::string entity_id;
  std::vector<ConfigurationDeleteResultItem> results;
};

template <>
inline constexpr auto dto_fields<ConfigurationDeleteMultiStatus> =
    std::make_tuple(field("entity_id", &ConfigurationDeleteMultiStatus::entity_id),
                    field("results", &ConfigurationDeleteMultiStatus::results));

template <>
inline constexpr std::string_view dto_name<ConfigurationDeleteMultiStatus> = "ConfigurationDeleteMultiStatus";

// =============================================================================
// Collection<ConfigurationMetaData, ConfigListXMedkit> - typed list shape
// returned by handle_list_configurations (PR-403 commit 26).
//
// Same wire shape as the legacy `Collection<ConfigurationMetaData>` named
// `ConfigurationList`, but the `x-medkit` payload carries the rich
// `ConfigListXMedkit` fields (entity_id, source, parameters, aggregation_level,
// is_aggregated, source_ids, queried_nodes, unavailable_nodes, failed_nodes,
// partial, failed_peers,
// peer_dropped_items) instead of the generic `XMedkitCollection`. The schema
// name is kept identical to the legacy `ConfigurationList` $ref so existing
// OpenAPI clients are not affected; the difference is purely server-side
// typing.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<ConfigurationMetaData, ConfigListXMedkit>> = "ConfigurationList";

// =============================================================================
// dto_sample specializations for DTOs with non-optional nlohmann::json fields.
//
// The generic sample path produces nlohmann::json{} (null) for bare json fields,
// which the round-trip reader treats as missing (null == absent for required
// fields).  Provide explicit samples with a non-null value to ensure
// EveryRegisteredDtoRoundTrips passes.
// =============================================================================
template <>
struct dto_sample<ConfigurationReadValue> {
  static ConfigurationReadValue make() {
    ConfigurationReadValue obj;
    obj.id = "sample";
    obj.data = nlohmann::json{42};  // non-null: int scalar representative value
    return obj;
  }
};

template <>
struct dto_sample<ConfigurationWriteRequest> {
  static ConfigurationWriteRequest make() {
    ConfigurationWriteRequest obj;
    obj.data = nlohmann::json{42};  // non-null: int scalar representative value
    // value intentionally omitted; data is the preferred field
    return obj;
  }
};

}  // namespace dto

// =============================================================================
// Alternate-status specialization for the 207 Multi-Status response from
// handle_delete_all_configurations (PR-403 commit 26).
//
// The handler returns `Result<std::variant<NoContent, ConfigurationDeleteMultiStatus>>`
// via `RouteRegistry::del_alternates`. NoContent maps to 204 by the default
// specialization in `alternate_status.hpp`; ConfigurationDeleteMultiStatus
// maps to 207 here so the framework picks the right status from the active
// variant alternative without the handler ever touching `res.status`.
// =============================================================================
namespace http {
template <>
struct dto_alternate_status<dto::ConfigurationDeleteMultiStatus> {
  static constexpr int value = 207;
};
}  // namespace http

}  // namespace ros2_medkit_gateway
