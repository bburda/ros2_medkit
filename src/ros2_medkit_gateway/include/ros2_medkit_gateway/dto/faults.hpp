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
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// FaultXMedkit - x-medkit vendor extension carried both inside FaultDetail and
// on each item of a fault list. Declared ahead of FaultListItem because that
// struct holds one by value.
//
// Wire keys (from build_sovd_fault_response):
//   occurrence_count, reporting_sources, severity_label, status_raw
// =============================================================================
struct FaultXMedkit {
  std::optional<int64_t> occurrence_count;
  std::optional<std::vector<std::string>> reporting_sources;
  std::optional<std::string> severity_label;
  std::optional<std::string> status_raw;
  std::optional<bool> expected;
  std::optional<std::string> planned_stop_id;
};

template <>
inline constexpr auto dto_fields<FaultXMedkit> = std::make_tuple(
    field("occurrence_count", &FaultXMedkit::occurrence_count),
    field("reporting_sources", &FaultXMedkit::reporting_sources),
    field("severity_label", &FaultXMedkit::severity_label), field("status_raw", &FaultXMedkit::status_raw),
    field("expected", &FaultXMedkit::expected,
          "Whether this fault's current cycle started inside a declared planned-stop window. Derived from "
          "`first_occurred`, so a code that failed during a stop and failed again afterwards is expected for "
          "the first cycle and not for the second. The flag changes nothing about the fault: it was confirmed, "
          "healed, cleared, captured and audited exactly as any other."),
    field("planned_stop_id", &FaultXMedkit::planned_stop_id,
          "Which window made it expected - the earliest-starting one covering the fault when several overlap. "
          "Present only when `expected` is true. The window's `reason` is NOT carried on the fault: resolve "
          "the id with `GET /x-medkit-planned-stops/{id}`, which also answers who declared it and when."));

template <>
inline constexpr std::string_view dto_name<FaultXMedkit> = "FaultXMedkit";

// =============================================================================
// FaultListItem - flat fault list item emitted by fault_msg_conversions.cpp
// (fault_to_json shape), wrapped in Collection<FaultListItem> for list endpoints.
//
// Wire keys (exact, from fault_msg_conversions.cpp):
//   fault_code, severity, description, first_occurred, last_occurred,
//   last_passed (absent = never passed), occurrence_count, status,
//   reporting_sources, severity_label
// =============================================================================
struct FaultListItem {
  std::string fault_code;
  int64_t severity{0};
  std::optional<std::string> description;
  std::optional<double> first_occurred;
  std::optional<double> last_occurred;
  std::optional<double> last_passed;
  std::optional<int64_t> occurrence_count;
  std::string status;
  std::optional<std::vector<std::string>> reporting_sources;
  std::optional<std::string> severity_label;  // enum: INFO|WARN|ERROR|CRITICAL|UNKNOWN

  /// Vendor extension on the item itself, carrying the planned-stop flag. Absent
  /// on a fault the gateway could not derive a flag for - which is not the same
  /// as `expected: false`, and is why it is an object rather than a bare bool.
  std::optional<FaultXMedkit> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FaultListItem> = std::make_tuple(
    field("fault_code", &FaultListItem::fault_code), field("severity", &FaultListItem::severity),
    field("description", &FaultListItem::description), field("first_occurred", &FaultListItem::first_occurred),
    field("last_occurred", &FaultListItem::last_occurred), field("last_passed", &FaultListItem::last_passed),
    field("occurrence_count", &FaultListItem::occurrence_count), field("status", &FaultListItem::status),
    field("reporting_sources", &FaultListItem::reporting_sources),
    field_enum("severity_label", &FaultListItem::severity_label, kFaultSeverityLabelValues),
    field("x-medkit", &FaultListItem::x_medkit,
          "Vendor extension carrying `expected` and `planned_stop_id` for this fault. Absent when the gateway "
          "could not read the planned-stop windows, which is not the same as the fault being unexpected."));

template <>
inline constexpr std::string_view dto_name<FaultListItem> = "FaultListItem";

// =============================================================================
// FaultStatus - SOVD status sub-object inside FaultDetail.item
//
// Wire keys (from build_status_object in fault_handlers.cpp):
//   aggregatedStatus (required, enum), testFailed, confirmedDTC, pendingDTC
// =============================================================================
struct FaultStatus {
  std::string aggregated_status;             // wire key: "aggregatedStatus"
  std::optional<std::string> test_failed;    // wire key: "testFailed"
  std::optional<std::string> confirmed_dtc;  // wire key: "confirmedDTC"
  std::optional<std::string> pending_dtc;    // wire key: "pendingDTC"
};

template <>
inline constexpr auto dto_fields<FaultStatus> =
    std::make_tuple(field_enum("aggregatedStatus", &FaultStatus::aggregated_status, kFaultAggregatedStatusValues),
                    field("testFailed", &FaultStatus::test_failed), field("confirmedDTC", &FaultStatus::confirmed_dtc),
                    field("pendingDTC", &FaultStatus::pending_dtc));

template <>
inline constexpr std::string_view dto_name<FaultStatus> = "FaultStatus";

// =============================================================================
// FaultItem - SOVD "item" sub-object inside FaultDetail
//
// Wire keys (from build_sovd_fault_response):
//   code (required), fault_name (optional), severity (required), status (required)
// =============================================================================
struct FaultItem {
  std::string code;
  std::optional<std::string> fault_name;
  int64_t severity{0};
  FaultStatus status;
};

template <>
inline constexpr auto dto_fields<FaultItem> =
    std::make_tuple(field("code", &FaultItem::code), field("fault_name", &FaultItem::fault_name),
                    field("severity", &FaultItem::severity), field("status", &FaultItem::status));

template <>
inline constexpr std::string_view dto_name<FaultItem> = "FaultItem";

// =============================================================================
// FaultEnvironmentData - SOVD "environment_data" sub-object inside FaultDetail
//
// Wire keys (from build_sovd_fault_response):
//   extended_data_records (free-form JSON object, optional),
//   snapshots (free-form JSON array - discriminated freeze_frame|rosbag, optional)
//
// Both fields are genuinely free-form: snapshots carry a runtime type
// discriminator ("type"/"snapshot_type") with per-variant optional fields.
// =============================================================================
struct FaultEnvironmentData {
  std::optional<nlohmann::json> extended_data_records;
  std::optional<nlohmann::json> snapshots;
};

template <>
inline constexpr auto dto_fields<FaultEnvironmentData> =
    std::make_tuple(field("extended_data_records", &FaultEnvironmentData::extended_data_records),
                    field("snapshots", &FaultEnvironmentData::snapshots));

template <>
inline constexpr std::string_view dto_name<FaultEnvironmentData> = "FaultEnvironmentData";

// =============================================================================
// FaultListXMedkit - x-medkit vendor extension on fault list responses:
//   handle_list_all_faults (global, no entity_id) and
//   handle_list_faults APP branch (per-app, has entity_id + source_id).
//
// Wire keys:
//   count           - total items in the response (required after fan-out merge)
//   muted_count     - number of muted/correlated faults (from FaultManager)
//   cluster_count   - number of fault clusters (from FaultManager)
//   entity_id       - SOVD entity ID (optional; absent for global endpoint)
//   source_id       - namespace_path used for filtering (optional; App only)
//   muted_faults    - detailed muted fault list (optional; only if requested)
//   clusters        - detailed cluster list (optional; only if requested)
//   partial            - true when a fan-out peer request failed (optional)
//   failed_peers       - list of peer addresses that returned errors (optional)
//   peer_failures      - why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items - per-peer items dropped due to malformed JSON
//                        (observability for invisible drift; optional)
// =============================================================================
struct FaultListXMedkit {
  int64_t count{0};
  std::optional<int64_t> muted_count;
  std::optional<int64_t> cluster_count;
  std::optional<std::string> entity_id;
  std::optional<std::string> source_id;
  std::optional<nlohmann::json> muted_faults;  // free-form: FaultManager output
  std::optional<nlohmann::json> clusters;      // free-form: FaultManager output
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
  std::optional<std::vector<DroppedItem>> peer_dropped_items;

  /// How many of the items were expected. A sibling of `count`, not a
  /// replacement: "three faults, two of them during the changeover" is the
  /// sentence an operator wants, and it needs both numbers.
  std::optional<int64_t> expected_count;
};

template <>
inline constexpr auto dto_fields<FaultListXMedkit> = std::make_tuple(
    field("count", &FaultListXMedkit::count), field("muted_count", &FaultListXMedkit::muted_count),
    field("cluster_count", &FaultListXMedkit::cluster_count), field("entity_id", &FaultListXMedkit::entity_id),
    field("source_id", &FaultListXMedkit::source_id), field("muted_faults", &FaultListXMedkit::muted_faults),
    field("clusters", &FaultListXMedkit::clusters), field("partial", &FaultListXMedkit::partial),
    field("failed_peers", &FaultListXMedkit::failed_peers), field("peer_failures", &FaultListXMedkit::peer_failures),
    field("peer_dropped_items", &FaultListXMedkit::peer_dropped_items),
    field("expected_count", &FaultListXMedkit::expected_count,
          "How many of the returned items started their current cycle inside a planned-stop window. Counts the "
          "items this gateway derived a flag for; items merged from a peer carry the peer's own flag and are "
          "not recounted here."));

template <>
inline constexpr std::string_view dto_name<FaultListXMedkit> = "FaultListXMedkit";

// =============================================================================
// FaultListAggXMedkit - x-medkit vendor extension on aggregated fault list
// responses: handle_list_faults FUNCTION / COMPONENT / AREA branches.
//
// Wire keys:
//   entity_id           - SOVD entity ID being queried
//   aggregation_level   - one of: "function", "component", "area"
//   aggregated          - always true (signals multi-source aggregation)
//   host_count          - number of host apps (Function only)
//   app_count           - number of apps (Component / Area)
//   component_count     - number of components (Area only)
//   aggregation_sources - FQNs used for filtering (array of strings)
//   count               - total items after fan-out merge
//   partial             - true when a fan-out peer request failed (optional)
//   failed_peers        - list of peer addresses that returned errors (optional)
//   peer_failures       - why each of those peers failed (timeout, unreachable, ...)
//   peer_dropped_items  - per-peer items dropped due to malformed JSON
//                         (observability for invisible drift; optional)
// =============================================================================
struct FaultListAggXMedkit {
  std::optional<std::string> entity_id;
  std::optional<std::string> aggregation_level;
  std::optional<bool> aggregated;
  std::optional<int64_t> host_count;
  std::optional<int64_t> app_count;
  std::optional<int64_t> component_count;
  std::optional<std::vector<std::string>> aggregation_sources;
  int64_t count{0};
  std::optional<bool> partial;
  std::optional<std::vector<std::string>> failed_peers;
  /// Why each peer in `failed_peers` contributed nothing. Sibling rather than a
  /// widening of `failed_peers`, whose wire shape (a list of names) every
  /// existing client already reads.
  std::optional<std::vector<PeerFailure>> peer_failures;
  std::optional<std::vector<DroppedItem>> peer_dropped_items;

  /// How many of the items were expected; see FaultListXMedkit::expected_count.
  std::optional<int64_t> expected_count;
};

template <>
inline constexpr auto dto_fields<FaultListAggXMedkit> =
    std::make_tuple(field("entity_id", &FaultListAggXMedkit::entity_id),
                    field("aggregation_level", &FaultListAggXMedkit::aggregation_level),
                    field("aggregated", &FaultListAggXMedkit::aggregated),
                    field("host_count", &FaultListAggXMedkit::host_count),
                    field("app_count", &FaultListAggXMedkit::app_count),
                    field("component_count", &FaultListAggXMedkit::component_count),
                    field("aggregation_sources", &FaultListAggXMedkit::aggregation_sources),
                    field("count", &FaultListAggXMedkit::count), field("partial", &FaultListAggXMedkit::partial),
                    field("failed_peers", &FaultListAggXMedkit::failed_peers),
                    field("peer_failures", &FaultListAggXMedkit::peer_failures),
                    field("peer_dropped_items", &FaultListAggXMedkit::peer_dropped_items),
                    field("expected_count", &FaultListAggXMedkit::expected_count,
                          "How many of the returned items started their current cycle inside a planned-stop "
                          "window."));

template <>
inline constexpr std::string_view dto_name<FaultListAggXMedkit> = "FaultListAggXMedkit";

// =============================================================================
// FaultDetail - SOVD nested fault detail response
// Emitted by FaultHandlers::handle_get_fault via build_sovd_fault_response.
//
// Wire keys:
//   item (required), environment_data (required), x-medkit (optional)
// =============================================================================
struct FaultDetail {
  FaultItem item;
  FaultEnvironmentData environment_data;
  std::optional<FaultXMedkit> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FaultDetail> =
    std::make_tuple(field("item", &FaultDetail::item), field("environment_data", &FaultDetail::environment_data),
                    field("x-medkit", &FaultDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<FaultDetail> = "FaultDetail";

// =============================================================================
// Collection<FaultListItem, FaultListXMedkit> - named "FaultList"
//
// The x-medkit member is FaultListXMedkit, not the generic XMedkitCollection:
// every fault-list emitter writes a FaultListXMedkit (count / muted_count /
// cluster_count / ...) and none of them writes XMedkitCollection's
// total_count + contributors. Naming the generic one here published a
// vendor-extension object the gateway has never sent.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<FaultListItem, FaultListXMedkit>> = "FaultList";

// =============================================================================
// Collection<FaultListItem, FaultListAggXMedkit> - named "FaultListAggregated"
//
// The per-entity list emits this instead of FaultList when the entity spans
// several reporting sources (Function / Component / Area branches of
// handle_list_faults): same items, a different vendor extension carrying the
// aggregation level and the source FQNs.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<FaultListItem, FaultListAggXMedkit>> = "FaultListAggregated";

// =============================================================================
// FaultListResult - typed envelope around the plugin-defined fault list
// response emitted by FaultProvider::list_faults.
//
// Wire shape: the bare JSON object the plugin returns (typically
// `{"items": [...]}` but plugin-determined). The wrapper is purely a C++ ABI
// affordance; JsonWriter / JsonReader / SchemaWriter are fully specialized so
// the wire bytes are byte-identical to the pre-typed ABI.
//
// Why opaque: per-item shape varies across plugins (UDS plugins may add DTC
// status byte, snapshot record refs, extended data; OPC-UA plugins may add
// node references and severity metadata; the in-tree FaultListItem schema is
// the gateway-internal ROS path emitted by FaultManager, not what plugins
// produce). Forcing `Collection<FaultListItem>` would drop those fields.
// Plugins fill `content` with whatever JSON object they want returned; the
// gateway emits it verbatim.
// =============================================================================
struct FaultListResult {
  nlohmann::json content;  // wire shape: the bare list response object
};

template <>
inline constexpr std::string_view dto_name<FaultListResult> = "FaultListResult";

template <>
inline constexpr bool is_opaque_dto_v<FaultListResult> = true;

template <>
struct JsonWriter<FaultListResult> {
  static nlohmann::json write(const FaultListResult & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<FaultListResult> {
  static tl::expected<FaultListResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    FaultListResult out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<FaultListResult> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"description",
         "Fault list whose shape depends on who owns the entity. An entity backed by the ROS 2 graph "
         "answers with `FaultList`, or with `FaultListAggregated` when the entity spans several reporting "
         "sources. A plugin-owned entity answers with whatever its FaultProvider returns and the gateway "
         "emits that verbatim. Which branch applies is discoverable ahead of the call: `x-medkit.source` on "
         "the entity's own document (`GET /{entity_type}/{entity_id}`) reads `plugin` for a plugin-owned "
         "entity."},
        {"anyOf",
         nlohmann::json::array({nlohmann::json{{"$ref", "#/components/schemas/FaultList"}},
                                nlohmann::json{{"$ref", "#/components/schemas/FaultListAggregated"}},
                                nlohmann::json{{"title", "PluginFaultList"},
                                               {"type", "object"},
                                               {"additionalProperties", true},
                                               {"x-medkit-opaque", true},
                                               {"description",
                                                "Plugin-defined list. A UDS backend adds DTC status bytes and "
                                                "snapshot record references; an OPC-UA backend adds node references "
                                                "and severity metadata. Read the plugin's own documentation."}}})}};
  }
};

template <>
struct dto_sample<FaultListResult> {
  static FaultListResult make() {
    FaultListResult obj;
    obj.content = nlohmann::json{{"items", nlohmann::json::array()}};
    return obj;
  }
};

// =============================================================================
// FaultDetailResult - typed envelope around the plugin-defined fault detail
// response emitted by FaultProvider::get_fault.
//
// Wire shape: the bare JSON object the plugin returns. The wrapper is purely a
// C++ ABI affordance; JsonWriter / JsonReader / SchemaWriter are fully
// specialized so the wire bytes are byte-identical to the pre-typed ABI.
//
// Why opaque: the gateway-internal FaultDetail DTO models the ROS path's
// SOVD-compliant shape (item + environment_data + x-medkit). Plugins
// (UDS / OPC-UA / vendor) often return richer or different fields per
// backend - DTC environment records, snapshot blobs, vendor extended status.
// Forcing FaultDetail would drop those fields. The schema is opaque so
// downstream OpenAPI consumers know the shape is plugin-determined.
// =============================================================================
struct FaultDetailResult {
  nlohmann::json content;  // wire shape: the bare fault detail object
};

template <>
inline constexpr std::string_view dto_name<FaultDetailResult> = "FaultDetailResult";

template <>
inline constexpr bool is_opaque_dto_v<FaultDetailResult> = true;

template <>
struct JsonWriter<FaultDetailResult> {
  static nlohmann::json write(const FaultDetailResult & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<FaultDetailResult> {
  static tl::expected<FaultDetailResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    FaultDetailResult out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<FaultDetailResult> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"description",
         "One fault whose shape depends on who owns the entity. An entity backed by the ROS 2 graph answers "
         "with `FaultDetail` (SOVD `item` + `environment_data` + `x-medkit`). A plugin-owned entity answers "
         "with whatever its FaultProvider returns and the gateway emits that verbatim. Which branch applies "
         "is discoverable ahead of the call: `x-medkit.source` on the entity's own document "
         "(`GET /{entity_type}/{entity_id}`) reads `plugin` for a plugin-owned entity."},
        {"anyOf",
         nlohmann::json::array({nlohmann::json{{"$ref", "#/components/schemas/FaultDetail"}},
                                nlohmann::json{{"title", "PluginFaultDetail"},
                                               {"type", "object"},
                                               {"additionalProperties", true},
                                               {"x-medkit-opaque", true},
                                               {"description",
                                                "Plugin-defined fault. Backends carry DTC environment records, "
                                                "snapshot blobs or vendor extended status here. Read the plugin's "
                                                "own documentation."}}})}};
  }
};

template <>
struct dto_sample<FaultDetailResult> {
  static FaultDetailResult make() {
    FaultDetailResult obj;
    obj.content = nlohmann::json{{"code", "DTC_000001"}, {"status", "pending"}};
    return obj;
  }
};

// =============================================================================
// FaultClearResult - typed envelope around the plugin-defined fault clear
// response emitted by FaultProvider::clear_fault.
//
// Wire shape: the bare JSON object the plugin returns (typically
// `{"code": ..., "cleared": true}` or `{"status": "ok"}`). The wrapper is
// purely a C++ ABI affordance; JsonWriter / JsonReader / SchemaWriter are
// fully specialized so the wire bytes are byte-identical to the pre-typed
// ABI.
//
// Why opaque: plugins return arbitrary acknowledgement payloads (UDS clear
// response codes, vendor warnings, residual fault state) that cannot be
// statically modelled. Plugins fill `content` with whatever JSON object they
// want returned; the gateway emits it verbatim.
// =============================================================================
struct FaultClearResult {
  nlohmann::json content;  // wire shape: the bare clear response object
};

template <>
inline constexpr std::string_view dto_name<FaultClearResult> = "FaultClearResult";

template <>
inline constexpr bool is_opaque_dto_v<FaultClearResult> = true;

template <>
struct JsonWriter<FaultClearResult> {
  static nlohmann::json write(const FaultClearResult & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<FaultClearResult> {
  static tl::expected<FaultClearResult, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    FaultClearResult out;
    out.content = j;
    return out;
  }
};

template <>
struct SchemaWriter<FaultClearResult> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"type", "object"},
        {"additionalProperties", true},
        {"x-medkit-opaque", true},
        {"description",
         "Acknowledgement of a clear, shaped by whoever owns the entity. The ROS 2 path answers "
         "`{\"code\": <fault_code>, \"cleared\": true}`; a plugin answers with its backend's own "
         "acknowledgement - UDS clear response codes, vendor warnings, residual fault state - and the "
         "gateway emits it verbatim. Treat the 2xx status, not a body field, as the signal that the clear "
         "succeeded."}};
  }
};

template <>
struct dto_sample<FaultClearResult> {
  static FaultClearResult make() {
    FaultClearResult obj;
    obj.content = nlohmann::json{{"code", "DTC_000001"}, {"cleared", true}};
    return obj;
  }
};

// =============================================================================
// FaultListQuery - query parameters for the global GET /faults list only. Read
// by list_all_faults via TypedRequest::query<FaultListQuery>() and declared in
// the OpenAPI spec via the same descriptor, so the two cannot drift. The
// per-entity GET /{entity}/faults uses the narrower FaultEntityListQuery below.
// =============================================================================
struct FaultListQuery {
  std::optional<std::string> status;
  bool include_muted = false;
  bool include_clusters = false;
  std::optional<std::string> expected;
};

template <>
inline constexpr auto dto_fields<FaultListQuery> = std::make_tuple(
    field_enum("status", &FaultListQuery::status, kFaultStatusFilterValues,
               "Filter by fault status: pending, confirmed, cleared, healed, or all"),
    field("include_muted", &FaultListQuery::include_muted, Presence::kOptional, "Include muted faults in the response"),
    field("include_clusters", &FaultListQuery::include_clusters, Presence::kOptional,
          "Include fault clusters in the response"),
    field_enum("expected", &FaultListQuery::expected, kFaultExpectedFilterValues,
               "Keep only faults whose current cycle started inside a planned-stop window (`true`), only those "
               "that did not (`false`), or both, which is what omitting it means (`all`)."));

// FaultEntityListQuery - query parameters for the per-entity GET /{entity}/faults
// route. Only `status` applies: include_muted / include_clusters are honored
// solely by the global GET /faults, because their correlation metadata is
// computed across the whole fault manager and a scoped response would leak
// cross-entity data. Declaring a narrower DTO here keeps the route advertising
// exactly what the handler reads (no spec-vs-runtime drift on the declared axis).
struct FaultEntityListQuery {
  std::optional<std::string> status;
  std::optional<std::string> expected;
};

template <>
inline constexpr auto dto_fields<FaultEntityListQuery> = std::make_tuple(
    field_enum("status", &FaultEntityListQuery::status, kFaultStatusFilterValues,
               "Filter by fault status: pending, confirmed, cleared, healed, or all"),
    field_enum("expected", &FaultEntityListQuery::expected, kFaultExpectedFilterValues,
               "Keep only faults whose current cycle started inside a planned-stop window (`true`), only those "
               "that did not (`false`), or both, which is what omitting it means (`all`)."));

// FaultClearQuery - query parameters for DELETE /faults (clear all). Only the
// status filter applies; the correlation flags are list-only.
struct FaultClearQuery {
  std::optional<std::string> status;
};

template <>
inline constexpr auto dto_fields<FaultClearQuery> =
    std::make_tuple(field_enum("status", &FaultClearQuery::status, kFaultStatusFilterValues,
                               "Clear only faults in this status: pending, confirmed, cleared, healed, or all"));

}  // namespace dto
}  // namespace ros2_medkit_gateway
