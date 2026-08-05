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

#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// XMedkitDataItem - x-medkit vendor extension emitted on data collection list
// items (handle_list_data per-topic), on data write responses
// (handle_put_data_item), and on data read responses (handle_get_data_item).
//
// Wire keys for list items:
//   ros2.topic     - ROS 2 topic path
//   ros2.direction - topic direction: "publish" | "subscribe" | "both"
//                    (maps to XMedkitRos2::direction)
//   ros2.type      - ROS 2 message type string
//   type_info      - dynamic type schema + default_value (free-form JSON;
//                    only present when type introspection succeeds)
//
// Additional keys for write responses (handle_put_data_item):
//   entity_id         - SOVD entity ID
//   status            - publish result status
//   publisher_created - true when a new publisher was created
//
// Additional keys for read responses (handle_get_data_item):
//   timestamp        - sample timestamp in nanoseconds since epoch (int64)
//   publisher_count  - number of publishers on the topic at sample time (int64)
//   subscriber_count - number of subscribers on the topic at sample time (int64)
//
// Additional keys on an aggregating entity's list items:
//   member_ids - the members of the grouping that provide the topic
//   available  - present only as false, marking an item whose provider is not
//                answering; absent means it can be served
// =============================================================================
struct XMedkitDataItem {
  std::optional<XMedkitRos2> ros2;
  std::optional<nlohmann::json> type_info;       // free-form: dynamic ROS IDL schema
  std::optional<std::string> entity_id;          // SOVD entity ID (write + read responses)
  std::optional<nlohmann::json> status;          // publish result / sample status
  std::optional<bool> publisher_created;         // true when publisher created (write responses)
  std::optional<std::int64_t> timestamp;         // sample timestamp in ns (read responses)
  std::optional<std::int64_t> publisher_count;   // publisher count at sample time (read responses)
  std::optional<std::int64_t> subscriber_count;  // subscriber count at sample time (read responses)
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
  /// the item is still part of the tree, and still addressable.
  std::optional<bool> available;
};

template <>
inline constexpr auto dto_fields<XMedkitDataItem> =
    std::make_tuple(field("ros2", &XMedkitDataItem::ros2), field("type_info", &XMedkitDataItem::type_info),
                    field("entity_id", &XMedkitDataItem::entity_id), field("status", &XMedkitDataItem::status),
                    field("publisher_created", &XMedkitDataItem::publisher_created),
                    field("timestamp", &XMedkitDataItem::timestamp),
                    field("publisher_count", &XMedkitDataItem::publisher_count),
                    field("subscriber_count", &XMedkitDataItem::subscriber_count),
                    field("member_ids", &XMedkitDataItem::member_ids), field("available", &XMedkitDataItem::available));

template <>
inline constexpr std::string_view dto_name<XMedkitDataItem> = "XMedkitDataItem";

// =============================================================================
// DataItem - single item emitted in handle_list_data "items" array.
//
// Wire keys (from data_handlers.cpp handle_list_data per-topic construction):
//   id       - ROS 2 topic path (used as round-trip ID for GET/PUT)
//   name     - same as id (topic path)
//   category - always "currentData"
//   x-medkit - optional vendor extension with ros2 topology + type info
// =============================================================================
struct DataItem {
  std::string id;
  std::string name;
  std::string category;                     // always "currentData"
  std::optional<XMedkitDataItem> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<DataItem> =
    std::make_tuple(field("id", &DataItem::id), field("name", &DataItem::name), field("category", &DataItem::category),
                    field("x-medkit", &DataItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<DataItem> = "DataItem";

// =============================================================================
// DataListXMedkit - x-medkit vendor extension on the data collection list
// response (the top-level response object from handle_list_data).
//
// Wire keys (from data_handlers.cpp handle_list_data response-level XMedkit):
//   entity_id          - SOVD entity ID being queried
//   aggregated         - true when cache reports aggregated data
//   aggregation_sources - list of source IDs when aggregated
//   aggregation_level  - aggregation level string when aggregated
//   total_count        - total number of items in the response
//   partial            - true when a fan-out peer request failed
//   failed_peers       - list of peer addresses that returned errors
//   peer_failures      - why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items - per-peer items dropped due to malformed JSON
//                        (observability for invisible drift)
// =============================================================================
struct DataListXMedkit {
  std::optional<std::string> entity_id;
  std::optional<bool> aggregated;
  std::optional<std::vector<std::string>> aggregation_sources;
  std::optional<std::string> aggregation_level;
  std::optional<std::size_t> total_count;
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
  std::optional<std::vector<DroppedItem>> peer_dropped_items;
};

template <>
inline constexpr auto dto_fields<DataListXMedkit> =
    std::make_tuple(field("entity_id", &DataListXMedkit::entity_id), field("aggregated", &DataListXMedkit::aggregated),
                    field("aggregation_sources", &DataListXMedkit::aggregation_sources),
                    field("aggregation_level", &DataListXMedkit::aggregation_level),
                    field("total_count", &DataListXMedkit::total_count), field("partial", &DataListXMedkit::partial),
                    field("failed_peers", &DataListXMedkit::failed_peers),
                    field("peer_failures", &DataListXMedkit::peer_failures),
                    field("peer_dropped_items", &DataListXMedkit::peer_dropped_items));

template <>
inline constexpr std::string_view dto_name<DataListXMedkit> = "DataListXMedkit";

// =============================================================================
// DataWriteRequest - PUT request body for /{entity}/data/{id}.
// Parsed by handle_put_data_item via parse_body<DataWriteRequest>.
//
// Wire keys (SOVD convention, from data_handlers.cpp handle_put_data_item):
//   type  - ROS 2 message type string (e.g. "std_msgs/msg/Float32"), required
//   data  - message value to publish (free-form: any JSON object), required
// =============================================================================
struct DataWriteRequest {
  std::string type;
  nlohmann::json data;  // free-form: message payload
};

template <>
inline constexpr auto dto_fields<DataWriteRequest> =
    std::make_tuple(field("type", &DataWriteRequest::type), field("data", &DataWriteRequest::data));

template <>
inline constexpr std::string_view dto_name<DataWriteRequest> = "DataWriteRequest";

// =============================================================================
// Collection<DataItem, DataListXMedkit> - named "DataList", the list shape
// list_data returns for a ROS 2-backed entity.
//
// The `x-medkit` payload is `DataListXMedkit` (entity_id, aggregated,
// aggregation_sources, aggregation_level, total_count, partial, failed_peers,
// peer_dropped_items), which is what the handler writes. The single-argument
// `Collection<DataItem>` used to carry this name too, defaulting `x-medkit` to
// the generic `XMedkitCollection` no data route has ever emitted; naming only
// the emitted shape keeps the document from advertising the other one.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<DataItem, DataListXMedkit>> = "DataList";

// =============================================================================
// dto_sample specialization for DataWriteRequest.
//
// The generic sample path produces nlohmann::json{} (null) for bare json
// fields, which the round-trip reader treats as missing (null == absent for
// required fields).  Provide an explicit sample with a non-null value to
// ensure EveryRegisteredDtoRoundTrips passes.
// =============================================================================
template <>
struct dto_sample<DataWriteRequest> {
  static DataWriteRequest make() {
    DataWriteRequest obj;
    obj.type = "std_msgs/msg/Float32";
    obj.data = nlohmann::json{{"data", 0.0}};  // non-null: representative message value
    return obj;
  }
};

// =============================================================================
// DataListResult - typed envelope around the plugin-defined data collection
// response emitted by DataProvider::list_data.
//
// Wire shape: the bare JSON object the plugin returns (typically
// `{"items": [...], "x-medkit": {...}}` but plugin-determined). The wrapper is
// purely a C++ ABI affordance; JsonWriter / JsonReader / SchemaWriter are fully
// specialized so the wire bytes are byte-identical to the pre-typed ABI.
//
// Why opaque: per-item shape varies across plugins (the in-tree OPC-UA plugin
// adds per-item `value`, `unit`, `data_type`, `writable`). Forcing a
// `Collection<DataItem>` here would drop those fields. Plugins fill `content`
// with whatever JSON object they want returned to the caller; the gateway
// emits it verbatim.
//
// The dto_name is `DataListResult` to avoid colliding with the existing
// `Collection<DataItem>` registration named `DataList` (used by the
// gateway-internal ROS data path in handle_list_data).
// =============================================================================
struct DataListResult {
  nlohmann::json content;  // wire shape: the bare list response object
};

template <>
inline constexpr std::string_view dto_name<DataListResult> = "DataListResult";

template <>
inline constexpr bool is_opaque_dto_v<DataListResult> = true;

template <>
struct JsonWriter<DataListResult> {
  static nlohmann::json write(const DataListResult & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<DataListResult> {
  static tl::expected<DataListResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    DataListResult out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<DataListResult> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"description",
         "Data-resource list whose shape depends on who owns the entity. An entity backed by the ROS 2 "
         "graph answers with `DataList`. A plugin-owned entity answers with whatever its DataProvider "
         "returns and the gateway emits that verbatim. Which branch applies is discoverable ahead of the "
         "call: `x-medkit.source` on the entity's own document (`GET /{entity_type}/{entity_id}`) reads "
         "`plugin` for a plugin-owned entity."},
        {"anyOf", nlohmann::json::array(
                      {nlohmann::json{{"$ref", "#/components/schemas/DataList"}},
                       nlohmann::json{{"title", "PluginDataList"},
                                      {"type", "object"},
                                      {"additionalProperties", true},
                                      {"x-medkit-opaque", true},
                                      {"description",
                                       "Plugin-defined list. The in-tree OPC-UA plugin adds value / unit / "
                                       "data_type / writable per item. Read the plugin's own documentation."}}})}};
  }
};

template <>
struct dto_sample<DataListResult> {
  static DataListResult make() {
    DataListResult obj;
    obj.content = nlohmann::json{{"items", nlohmann::json::array()}};
    return obj;
  }
};

// =============================================================================
// DataValue - typed envelope around the plugin-defined data read response
// emitted by DataProvider::read_data.
//
// Wire shape: the bare JSON object the plugin returns (typically
// `{"id": ..., "value": <ros_msg>, ...}` for ROS plugins or
// `{"id", "value", "unit", "data_type", "writable", ...}` for OPC-UA). The
// wrapper is purely a C++ ABI affordance; JsonWriter / JsonReader /
// SchemaWriter are fully specialized so the wire bytes are byte-identical to
// the pre-typed ABI.
//
// Why opaque: the payload shape is runtime-dependent (depends on the resource
// and the plugin's backend - live ROS message, OPC-UA value, UDS DID, ...)
// and therefore cannot be statically modelled. The schema is opaque
// (`additionalProperties:true`, `x-medkit-opaque:true`) so downstream OpenAPI
// consumers know the shape is plugin-determined.
// =============================================================================
struct DataValue {
  nlohmann::json content;  // wire shape: the bare read response object
};

template <>
inline constexpr std::string_view dto_name<DataValue> = "DataValue";

template <>
inline constexpr bool is_opaque_dto_v<DataValue> = true;

template <>
struct JsonWriter<DataValue> {
  static nlohmann::json write(const DataValue & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<DataValue> {
  static tl::expected<DataValue, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    DataValue out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<DataValue> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"type", "object"},
        {"additionalProperties", true},
        {"x-medkit-opaque", true},
        {"description",
         "One data resource's value. No fixed shape exists: on the ROS 2 path the body carries the live "
         "message, whose fields come from the topic's IDL type, and on a plugin-owned entity it carries "
         "whatever the DataProvider returns (an OPC-UA node adds unit / data_type / writable, a UDS DID "
         "its own record layout). Discover the concrete shape from the item's entry in `GET "
         "/{entity_type}/{entity_id}/data`, whose `x-medkit` names the ROS 2 message type."}};
  }
};

template <>
struct dto_sample<DataValue> {
  static DataValue make() {
    DataValue obj;
    obj.content = nlohmann::json{{"id", "sample_resource"}, {"value", 0.0}};
    return obj;
  }
};

// =============================================================================
// DataWriteResult - typed envelope around the plugin-defined data write
// response emitted by DataProvider::write_data.
//
// Wire shape: the bare JSON object the plugin returns (typically
// `{"status": "ok", ...}` or `{"id", "status", "value_written"}` for OPC-UA).
// The wrapper is purely a C++ ABI affordance; JsonWriter / JsonReader /
// SchemaWriter are fully specialized so the wire bytes are byte-identical to
// the pre-typed ABI.
//
// Why opaque: see DataValue rationale - the payload shape is plugin-
// determined.
// =============================================================================
struct DataWriteResult {
  nlohmann::json content;  // wire shape: the bare write response object
};

template <>
inline constexpr std::string_view dto_name<DataWriteResult> = "DataWriteResult";

template <>
inline constexpr bool is_opaque_dto_v<DataWriteResult> = true;

template <>
struct JsonWriter<DataWriteResult> {
  static nlohmann::json write(const DataWriteResult & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<DataWriteResult> {
  static tl::expected<DataWriteResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    DataWriteResult out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<DataWriteResult> {
  static nlohmann::json schema() {
    return nlohmann::json{{"type", "object"}, {"additionalProperties", true}, {"x-medkit-opaque", true}};
  }
};

template <>
struct dto_sample<DataWriteResult> {
  static DataWriteResult make() {
    DataWriteResult obj;
    obj.content = nlohmann::json{{"status", "ok"}};
    return obj;
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
