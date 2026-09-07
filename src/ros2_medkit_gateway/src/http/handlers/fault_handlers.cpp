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

#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <set>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/faults/planned_stop.hpp"
#include "ros2_medkit_gateway/core/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

// =============================================================================
// Typed-handler helpers
// =============================================================================

/// Sanitize a plugin-supplied error into the standard `x-medkit-plugin-error`
/// shape: clamp HTTP status to [400, 599] and truncate message at 512 chars.
ErrorInfo make_plugin_error(int http_status, const std::string & message, json extra_params = {}) {
  static constexpr size_t kMaxMessageLength = 512;
  int status = http_status < 400 ? 400 : (http_status > 599 ? 599 : http_status);
  std::string msg = message.size() > kMaxMessageLength ? message.substr(0, kMaxMessageLength) + "..." : message;
  return make_error(status, ERR_PLUGIN_ERROR, std::move(msg), std::move(extra_params));
}

/// Read the first positional capture group (entity_id). cpp-httplib only invokes
/// the route when the regex matches, so the `nullopt` branch is effectively
/// unreachable; we surface it as 400 invalid-request to match the legacy
/// handlers' explicit `req.matches.size() < N` guard.
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the second positional capture group (fault_code).
tl::expected<std::string, ErrorInfo> read_fault_code(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Build a populated FaultStatusFilter from query params, surfacing an
/// ErrorInfo when the `status` value is unknown.
tl::expected<FaultStatusFilter, ErrorInfo> read_fault_status_filter(const std::optional<std::string> & status,
                                                                    const json & extra_params = {}) {
  auto filter = parse_fault_status_param(status);
  if (filter.is_valid) {
    return filter;
  }
  json params{{"allowed_values", "pending, confirmed, cleared, healed, all"},
              {"parameter", "status"},
              {"value", status.value_or("")}};
  if (extra_params.is_object()) {
    for (auto it = extra_params.begin(); it != extra_params.end(); ++it) {
      params[it.key()] = it.value();
    }
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid status parameter value", params));
}

// =============================================================================
// SOVD-compliant response helpers (legacy free functions kept verbatim)
// =============================================================================

/// Build SOVD status object from fault status string
/// Maps ROS 2 medkit status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED)
/// to SOVD aggregated status (active, passive, cleared)
json build_status_object(const std::string & status) {
  json status_obj;

  // SOVD aggregatedStatus mapping:
  // - "active": fault is confirmed and currently active
  // - "passive": fault detected but not yet confirmed (pending)
  // - "cleared": fault resolved or manually cleared
  std::string aggregated = "cleared";
  bool test_failed = false;
  bool confirmed_dtc = false;
  bool pending_dtc = false;

  if (status == "CONFIRMED") {
    aggregated = "active";
    test_failed = true;
    confirmed_dtc = true;
  } else if (status == "PREFAILED") {
    aggregated = "passive";
    test_failed = true;
    pending_dtc = true;
  } else if (status == "PREPASSED") {
    aggregated = "passive";
    pending_dtc = true;
  } else if (status == "HEALED" || status == "CLEARED") {
    aggregated = "cleared";
  }

  status_obj["aggregatedStatus"] = aggregated;
  status_obj["testFailed"] = test_failed ? "1" : "0";
  status_obj["confirmedDTC"] = confirmed_dtc ? "1" : "0";
  status_obj["pendingDTC"] = pending_dtc ? "1" : "0";

  return status_obj;
}

/// Convert nanoseconds since epoch to ISO 8601 string with milliseconds
/// Delegates to shared utility in http_utils.hpp
std::string to_iso8601_ns(int64_t ns) {
  return ros2_medkit_gateway::format_timestamp_ns(ns);
}

/// Map fault severity level to human-readable label.
/// Values mirror ros2_medkit_msgs/msg/Fault.msg SEVERITY_* constants:
///   SEVERITY_INFO = 0
///   SEVERITY_WARN = 1
///   SEVERITY_ERROR = 2
///   SEVERITY_CRITICAL = 3
std::string severity_to_label(uint8_t severity) {
  switch (severity) {
    case 0:
      return "INFO";
    case 1:
      return "WARN";
    case 2:
      return "ERROR";
    case 3:
      return "CRITICAL";
    default:
      return "UNKNOWN";
  }
}

/// Extract primary value from JSON data based on message type
/// For common ROS message types, extracts the main value field
json extract_primary_value(const std::string & message_type, const json & full_data) {
  // Map of message type to primary field
  static const std::unordered_map<std::string, std::string> primary_fields = {
      {"std_msgs/msg/Float64", "data"},
      {"std_msgs/msg/Float32", "data"},
      {"std_msgs/msg/Int32", "data"},
      {"std_msgs/msg/Int64", "data"},
      {"std_msgs/msg/Bool", "data"},
      {"std_msgs/msg/String", "data"},
      {"sensor_msgs/msg/Temperature", "temperature"},
      {"sensor_msgs/msg/BatteryState", "percentage"},
      {"sensor_msgs/msg/FluidPressure", "fluid_pressure"},
      {"sensor_msgs/msg/Range", "range"},
      {"geometry_msgs/msg/Twist", "linear"},  // Returns nested object
      {"nav_msgs/msg/Odometry", "pose"},      // Returns nested object
  };

  auto it = primary_fields.find(message_type);
  if (it != primary_fields.end() && full_data.contains(it->second)) {
    return full_data[it->second];
  }

  // Fallback: return full data
  return full_data;
}

/// How the `expected` query parameter narrows a fault list.
enum class ExpectedFilter : uint8_t { All, OnlyExpected, OnlyUnexpected };

tl::expected<ExpectedFilter, ErrorInfo> read_expected_filter(const std::optional<std::string> & value,
                                                             json params = json::object()) {
  if (!value.has_value() || value->empty() || *value == "all") {
    return ExpectedFilter::All;
  }
  if (*value == "true") {
    return ExpectedFilter::OnlyExpected;
  }
  if (*value == "false") {
    return ExpectedFilter::OnlyUnexpected;
  }
  params["parameter"] = "expected";
  params["value"] = *value;
  return tl::make_unexpected(
      make_error(400, ERR_INVALID_PARAMETER, "expected must be one of: true, false, all", std::move(params)));
}

/// Whether a peer's item says it was expected. Reads the flag the PEER derived;
/// an item that carries none says nothing either way.
std::optional<bool> peer_item_expected_flag(const json & item) {
  if (!item.is_object()) {
    return std::nullopt;
  }
  const auto ext = item.find("x-medkit");
  if (ext == item.end() || !ext->is_object()) {
    return std::nullopt;
  }
  const auto flag = ext->find("expected");
  if (flag == ext->end() || !flag->is_boolean()) {
    return std::nullopt;
  }
  return flag->get<bool>();
}

bool peer_item_is_expected(const json & item) {
  return peer_item_expected_flag(item).value_or(false);
}

/// Mark each item with the planned-stop flag, drop what the filter excludes, and
/// report how many of the survivors were expected.
///
/// Runs on the LOCAL items only, before any peer merge: a peer's gateway derives
/// its own flag from its own fault manager's windows, and this gateway has no
/// business re-deciding it from windows the peer never saw.
/// Whether a peer's item survives the `expected` filter. An item whose flag the
/// peer did not send is dropped by BOTH `true` and `false`: this gateway cannot
/// claim it was expected, and it cannot claim it was not either.
bool keep_peer_item(const json & item, ExpectedFilter filter) {
  if (filter == ExpectedFilter::All) {
    return true;
  }
  const auto flag = peer_item_expected_flag(item);
  if (!flag.has_value()) {
    return false;
  }
  return filter == ExpectedFilter::OnlyExpected ? *flag : !*flag;
}

int64_t apply_planned_stops(json & items, FaultManager & fault_mgr, ExpectedFilter filter) {
  if (!items.is_array()) {
    return 0;
  }
  const auto windows = fault_mgr.planned_stop_windows();

  json kept = json::array();
  int64_t expected_count = 0;
  for (auto & item : items) {
    const bool expected = faults::annotate_fault_with_planned_stop(item, windows);
    if (filter == ExpectedFilter::OnlyExpected && !expected) {
      continue;
    }
    if (filter == ExpectedFilter::OnlyUnexpected && expected) {
      continue;
    }
    if (expected) {
      ++expected_count;
    }
    kept.push_back(item);
  }
  items = std::move(kept);
  return expected_count;
}

/// Wrap a built JSON list payload in a typed FaultListResult envelope.
dto::FaultListResult wrap_list_result(json payload) {
  dto::FaultListResult out;
  out.content = std::move(payload);
  return out;
}

/// Wrap a built JSON detail payload in a typed FaultDetailResult envelope.
dto::FaultDetailResult wrap_detail_result(json payload) {
  dto::FaultDetailResult out;
  out.content = std::move(payload);
  return out;
}

}  // namespace

// Rationale and parameter contract are documented on the declaration in
// fault_handlers.hpp. In short: 503 is reserved for a call that never got an
// answer; anything the fault manager answered is a client error.
ErrorInfo FaultHandlers::classify_fault_failure(FaultFailure failure, const std::string & error_message,
                                                const std::string & unavailable_summary, const std::string & id_field,
                                                const std::string & entity_id, const std::string & fault_code) {
  const json params{{"details", error_message}, {id_field, entity_id}, {"fault_code", fault_code}};
  switch (failure) {
    case FaultFailure::Declined:
      return make_error(404, ERR_RESOURCE_NOT_FOUND, "Fault not found", params);
    case FaultFailure::Unavailable:
      break;
  }
  return make_error(503, ERR_SERVICE_UNAVAILABLE, unavailable_summary, params);
}

bool FaultHandlers::fault_in_source_scope(const json & fault, const std::set<std::string> & source_fqns) {
  // Thin wrapper preserving the public static API; the scope logic now lives in
  // the neutral core helper shared with the ROS 2 plugin-context fault path.
  return faults::fault_in_source_scope(fault, source_fqns);
}

// Static method: merge zero-config entity freeze-frames into the intermediate
// environment-data JSON (transport shape), before build_sovd_fault_response.
json FaultHandlers::merge_entity_freeze_frames(json env_data,
                                               const std::vector<EntityFreezeFrameCapture::Frame> & frames) {
  if (frames.empty()) {
    return env_data;
  }
  if (!env_data.contains("snapshots") || !env_data["snapshots"].is_array()) {
    env_data["snapshots"] = json::array();
  }
  // Explicit snapshot config wins: if the fault_manager captured any
  // freeze-frame for this fault, the zero-config entity frames stay out.
  for (const auto & s : env_data["snapshots"]) {
    if (s.value("snapshot_type", s.value("type", "")) == "freeze_frame") {
      return env_data;
    }
  }
  for (const auto & frame : frames) {
    // Intermediate transport shape (fault_msg_conversions): the freeze_frame
    // branch of build_sovd_fault_response turns this into the standard wire
    // entry (type/name/data + x-medkit.full_data/captured_at).
    json snap;
    snap["type"] = "freeze_frame";
    snap["snapshot_type"] = "freeze_frame";
    snap["name"] = frame.entity_id;
    snap["data"] = frame.values.dump();
    snap["topic"] = "";  // entity data values, not a ROS topic
    snap["message_type"] = "";
    snap["captured_at_ns"] = frame.captured_at_ns;
    if (frame.startup_catchup) {
      // Values were read at gateway start, not when the fault confirmed;
      // absent marker = captured on the confirm edge.
      snap["capture_origin"] = "startup";
    }
    // Value provenance, only when the plugin's payload reported it: a
    // disconnected entity serves last-known values whose age captured_at_ns
    // cannot convey.
    if (frame.connected.has_value()) {
      snap["connected"] = *frame.connected;
    }
    if (!frame.source_timestamp.is_null()) {
      snap["source_timestamp"] = frame.source_timestamp;
    }
    env_data["snapshots"].push_back(std::move(snap));
  }
  return env_data;
}

// Static method: Build SOVD-compliant fault response from transport-supplied JSON.
//
// The transport adapter performs ros2_medkit_msgs -> JSON translation; the
// handler post-processes the intermediate snapshot shape (freeze_frame parse,
// rosbag bulk_data_uri) using the entity_path it has at request time.
dto::FaultDetail
FaultHandlers::build_sovd_fault_response(const json & fault_json, const json & env_data_json,
                                         const std::string & entity_path,
                                         const std::vector<faults::PlannedStopWindow> & planned_stops) {
  const std::string fault_code = fault_json.value("fault_code", "");
  const std::string status = fault_json.value("status", "");
  const uint8_t severity = fault_json.value("severity", static_cast<uint8_t>(0));

  // === Build dto::FaultItem ===
  dto::FaultDetail detail;

  detail.item.code = fault_code;
  const std::string description = fault_json.value("description", "");
  if (!description.empty()) {
    detail.item.fault_name = description;
  }
  detail.item.severity = static_cast<int64_t>(severity);

  // Build FaultStatus from raw status string
  auto status_obj = build_status_object(status);
  detail.item.status.aggregated_status = status_obj.value("aggregatedStatus", "cleared");
  detail.item.status.test_failed = status_obj.value("testFailed", std::string{});
  detail.item.status.confirmed_dtc = status_obj.value("confirmedDTC", std::string{});
  detail.item.status.pending_dtc = status_obj.value("pendingDTC", std::string{});

  // === SOVD "environment_data" ===
  json snapshots = json::array();
  if (env_data_json.contains("snapshots") && env_data_json["snapshots"].is_array()) {
    for (const auto & s : env_data_json["snapshots"]) {
      json snap;
      const std::string type = s.value("type", "");
      // Prefer the explicit "snapshot_type" discriminator emitted by the
      // conversion layer; fall back to "type" for backward compatibility with
      // older transports that did not yet emit it.
      const std::string snapshot_type = s.value("snapshot_type", type);
      snap["type"] = type;
      snap["name"] = s.value("name", "");

      if (snapshot_type == "freeze_frame") {
        // Parse JSON data string and extract primary value.
        const std::string raw_data = s.value("data", "");
        const std::string topic = s.value("topic", "");
        const std::string message_type = s.value("message_type", "");
        const int64_t captured_at_ns = s.value("captured_at_ns", static_cast<int64_t>(0));
        try {
          json full_data = json::parse(raw_data);
          snap["data"] = extract_primary_value(message_type, full_data);
          snap["x-medkit"] = {{"topic", topic},
                              {"message_type", message_type},
                              {"full_data", full_data},
                              {"captured_at", to_iso8601_ns(captured_at_ns)}};
        } catch (const json::exception & e) {
          snap["data"] = raw_data;
          snap["x-medkit"] = {{"topic", topic}, {"message_type", message_type}, {"parse_error", e.what()}};
        }
        if (s.contains("capture_origin") && s["capture_origin"].is_string()) {
          snap["x-medkit"]["capture_origin"] = s["capture_origin"];
        }
        // Entity-frame provenance (merge_entity_freeze_frames), only when known.
        if (s.contains("connected") && s["connected"].is_boolean()) {
          snap["x-medkit"]["connected"] = s["connected"];
        }
        if (s.contains("source_timestamp")) {
          snap["x-medkit"]["source_timestamp"] = s["source_timestamp"];
        }
      } else if (snapshot_type == "rosbag") {
        // Build the absolute URI from the RECORDING id the transport supplied. It
        // must match list_descriptors, which uses the same value as the descriptor
        // id, and the download handler, which resolves it back to a recording.
        //
        // No fallback to the parent fault code any more. A fault can hold several
        // recordings, so substituting its code would advertise one URL for all of
        // them and serve whichever the compatibility path happened to pick.
        //
        // With no addressable id there is no snapshot to report: a rosbag entry
        // is a pointer to bytes, and one that points nowhere is not degraded
        // information, it is a download button that cannot work. Every rosbag
        // snapshot this endpoint has ever emitted carried a bulk_data_uri, and
        // consumers dereference it unguarded; dropping the entry keeps that
        // invariant rather than inventing a new shape for them to crash on.
        // Same rule as the bulk-data listing, which also drops an unaddressable
        // row instead of advertising it.
        if (!s.contains("bulk_data_id") || !s["bulk_data_id"].is_string() ||
            s["bulk_data_id"].get<std::string>().empty()) {
          RCLCPP_WARN(HandlerContext::logger(),
                      "Rosbag snapshot for fault '%s' on entity '%s' carries no 'bulk_data_id'; omitting it, as "
                      "nothing could address the recording. This is a transport-side bug.",
                      fault_code.c_str(), entity_path.c_str());
          continue;
        }
        std::string bulk_data_uri = entity_path;
        bulk_data_uri += "/bulk-data/rosbags/";
        bulk_data_uri += s["bulk_data_id"].get<std::string>();
        snap["bulk_data_uri"] = std::move(bulk_data_uri);
        if (s.contains("size_bytes")) {
          snap["size_bytes"] = s["size_bytes"];
        }
        if (s.contains("duration_sec")) {
          snap["duration_sec"] = s["duration_sec"];
        }
        if (s.contains("format")) {
          snap["format"] = s["format"];
        }
        // Same x-medkit.captured_at shape the freeze frame emits, because that
        // is the field the UIs already render. Zero means the recording predates
        // the timestamp being carried, and an epoch date on screen would be a
        // worse answer than none, so the key is left out.
        const int64_t captured_at_ns = s.value("captured_at_ns", static_cast<int64_t>(0));
        if (captured_at_ns > 0) {
          snap["x-medkit"] = {{"captured_at", to_iso8601_ns(captured_at_ns)}};
        }
      }

      snapshots.push_back(snap);
    }
  }

  if (env_data_json.contains("extended_data_records")) {
    detail.environment_data.extended_data_records = env_data_json["extended_data_records"];
  } else {
    detail.environment_data.extended_data_records = json{{"first_occurrence", ""}, {"last_occurrence", ""}};
  }
  detail.environment_data.snapshots = snapshots;

  // === x-medkit extensions ===
  std::vector<std::string> reporting_sources;
  if (fault_json.contains("reporting_sources") && fault_json["reporting_sources"].is_array()) {
    for (const auto & src : fault_json["reporting_sources"]) {
      reporting_sources.push_back(src.get<std::string>());
    }
  }

  dto::FaultXMedkit xm;
  xm.occurrence_count = static_cast<int64_t>(fault_json.value("occurrence_count", static_cast<uint64_t>(0)));
  if (!reporting_sources.empty()) {
    xm.reporting_sources = std::move(reporting_sources);
  }
  xm.severity_label = severity_to_label(severity);
  xm.status_raw = status;
  if (const auto * covering = faults::window_for_fault(planned_stops, fault_json)) {
    xm.expected = true;
    xm.planned_stop_id = covering->id;
  } else {
    xm.expected = false;
  }
  detail.x_medkit = std::move(xm);

  return detail;
}

// =============================================================================
// GET /faults - list all faults globally
// =============================================================================

http::Result<dto::FaultListResult> FaultHandlers::list_all_faults(const http::TypedRequest & req) {
  try {
    const auto q = req.query<dto::FaultListQuery>();
    auto filter_result = read_fault_status_filter(q.status);
    if (!filter_result) {
      return tl::make_unexpected(filter_result.error());
    }
    const auto filter = *filter_result;

    auto expected_result = read_expected_filter(q.expected);
    if (!expected_result) {
      return tl::make_unexpected(expected_result.error());
    }
    const auto expected_filter = *expected_result;

    auto fault_mgr = ctx_.node()->get_fault_manager();
    // Empty source_id = no filtering, return all faults
    auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                         filter.include_healed, q.include_muted, q.include_clusters);
    if (!result.success) {
      return tl::make_unexpected(
          make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults", json{{"details", result.error_message}}));
    }

    // Format: items array at top level
    json response = {{"items", result.data["faults"]}};
    int64_t expected_count_total = apply_planned_stops(response["items"], *fault_mgr, expected_filter);

    // x-medkit extension for ros2_medkit-specific fields (typed DTO)
    dto::FaultListXMedkit xm;
    xm.muted_count = result.data.value("muted_count", static_cast<int64_t>(0));
    xm.cluster_count = result.data.value("cluster_count", static_cast<int64_t>(0));

    // Include detailed correlation data if requested and present
    if (result.data.contains("muted_faults")) {
      xm.muted_faults = result.data["muted_faults"];
    }
    if (result.data.contains("clusters")) {
      xm.clusters = result.data["clusters"];
    }

    // Fan-out to peers: faults are managed by FaultManager, not cached, so we
    // need to query each peer's /faults endpoint and merge results. The legacy
    // path used the raw `fan_out_get` here (not the typed `fan_out_collection`)
    // because the per-item shape on the global list includes vendor extensions
    // that JsonReader<FaultListItem> would drop; preserve that behaviour.
    if (auto * agg = ctx_.aggregation_manager()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
      // build_fan_out_path, not raw path: `?expected=` and `?status=` are the
      // client's question, and a peer answering a different question is not an
      // answer. Forwarding `raw_req.path` dropped the query outright, so
      // `?expected=true` came back carrying the peer's unexpected faults.
      auto fan_result = agg->fan_out_get(build_fan_out_path(raw_req), raw_req.get_header_value("Authorization"));
      if (fan_result.merged_items.is_array()) {
        for (const auto & item : fan_result.merged_items) {
          // Filtered again here, on the flag the PEER derived from ITS OWN
          // windows - this gateway has no business re-deriving it, and an older
          // peer that ignored the parameter must not put items through. An item
          // with no flag is not known to satisfy either side of the filter, so
          // it is dropped by both.
          if (!keep_peer_item(item, expected_filter)) {
            continue;
          }
          if (peer_item_is_expected(item)) {
            ++expected_count_total;
          }
          response["items"].push_back(item);
        }
      }
      if (fan_result.is_partial) {
        xm.partial = true;
        xm.failed_peers = fan_result.failed_peers;
        xm.peer_failures = fan_result.peer_failures;
      }
    }

    // Counted after the merge, so the numbers describe the list that was
    // actually served rather than the local half of it.
    xm.count = static_cast<int64_t>(response["items"].size());
    xm.expected_count = expected_count_total;

    response["x-medkit"] = dto::JsonWriter<dto::FaultListXMedkit>::write(xm);
    return wrap_list_result(std::move(response));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in list_all_faults: %s", e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to list faults", json{{"details", e.what()}}));
  }
}

// =============================================================================
// GET /{entity-path}/faults - list faults for entity
// =============================================================================

http::Result<dto::FaultListResult> FaultHandlers::list_faults(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  try {
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::make_unexpected(flatten_validator_error(entity_result.error()));
    }
    const auto entity_info = *entity_result;

    // Delegate to plugin FaultProvider if entity is plugin-owned. Plugins
    // without a FaultProvider report via ReportFault only (the native path):
    // their faults live in the fault_manager, so fall through to the shared
    // scoped listing below like any other entity.
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov != nullptr) {
        try {
          auto result = fault_prov->list_faults(entity_id);
          if (!result) {
            return tl::make_unexpected(
                make_plugin_error(result.error().http_status, result.error().message, json{{"entity_id", entity_id}}));
          }
          // The plugin's records describe faults on the same plant, so they get
          // the same derivation. Reaching into the returned JSON rather than
          // asking the plugin keeps the flag a gateway concern: no FaultProvider
          // has to know that planned stops exist.
          auto plugin_query = req.query<dto::FaultEntityListQuery>();
          auto plugin_filter = read_expected_filter(plugin_query.expected, json{{"entity_id", entity_id}});
          if (!plugin_filter) {
            return tl::make_unexpected(plugin_filter.error());
          }
          if (result->content.is_object() && result->content.contains("items")) {
            const int64_t plugin_expected =
                apply_planned_stops(result->content["items"], *ctx_.node()->get_fault_manager(), *plugin_filter);
            // set_expected_count CREATES the extension. Writing through
            // `content["x-medkit"]["expected_count"]` behind an is_object()
            // guard left a plugin that sends no vendor extension answering
            // `"x-medkit": null` - neither the count nor the absence of one.
            faults::set_expected_count(result->content, plugin_expected);
          }
          return std::move(*result);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          return tl::make_unexpected(
              make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
        }
      }
    }

    // Validate entity type supports faults collection (SOVD Table 8)
    if (auto access = HandlerContext::validate_collection_access_typed(entity_info, ResourceCollection::FAULTS);
        !access) {
      ErrorInfo err = access.error();
      err.code = ERR_COLLECTION_NOT_SUPPORTED;
      return tl::make_unexpected(err);
    }

    const auto q = req.query<dto::FaultEntityListQuery>();
    auto filter_result = read_fault_status_filter(q.status, json{{entity_info.id_field, entity_id}});
    if (!filter_result) {
      return tl::make_unexpected(filter_result.error());
    }
    const auto filter = *filter_result;

    auto expected_result = read_expected_filter(q.expected, json{{entity_info.id_field, entity_id}});
    if (!expected_result) {
      return tl::make_unexpected(expected_result.error());
    }
    const auto expected_filter = *expected_result;

    // Note: the per-entity query DTO (FaultEntityListQuery) deliberately omits
    // include_muted / include_clusters - the underlying service returns
    // correlation metadata computed across the entire fault manager, so emitting
    // it on a scoped response would leak cross-entity data. The global
    // `GET /faults` route is the only place these flags are declared and honored.

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // For Functions, aggregate faults from all host apps.
    // Functions don't have a single namespace_path (it is always empty in EntityInfo)
    // because they host apps from potentially different namespaces.
    // Instead, we collect the FQNs of all host apps and filter by reporting_source.
    if (entity_info.type == EntityType::FUNCTION || entity_info.type == EntityType::COMPONENT) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                           filter.include_healed, /*include_muted=*/false, /*include_clusters=*/false);

      if (!result.success) {
        return tl::make_unexpected(
            make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                       json{{"details", result.error_message}, {entity_info.id_field, entity_id}}));
      }

      // Resolve FQN set via the shared helper so subareas (Areas) and
      // component-hosted Functions are handled the same way as the per-fault
      // routes.
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto source_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);

      json filtered_faults = faults::filter_faults_by_sources(result.data["faults"], source_fqns);
      json response = {{"items", filtered_faults}};
      const int64_t expected_count = apply_planned_stops(response["items"], *fault_mgr, expected_filter);

      // x-medkit extension (typed DTO)
      dto::FaultListAggXMedkit xm;
      xm.entity_id = entity_id;
      xm.expected_count = expected_count;
      const bool is_function = entity_info.type == EntityType::FUNCTION;
      xm.aggregation_level = is_function ? "function" : "component";
      xm.aggregated = true;
      if (is_function) {
        xm.host_count = static_cast<int64_t>(source_fqns.size());
      } else {
        xm.app_count = static_cast<int64_t>(source_fqns.size());
      }
      {
        std::vector<std::string> sources(source_fqns.begin(), source_fqns.end());
        if (!sources.empty()) {
          xm.aggregation_sources = std::move(sources);
        }
      }

      auto xm_json = dto::JsonWriter<dto::FaultListAggXMedkit>::write(xm);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
      merge_peer_items(ctx_.aggregation_manager(), raw_req, response, xm_json);
      xm_json["count"] = static_cast<int64_t>(response["items"].size());
      response["x-medkit"] = std::move(xm_json);
      return wrap_list_result(std::move(response));
    }

    // For Areas, aggregate faults from all apps in all components within the area
    // This is an x-medkit extension - SOVD spec does not define fault collections for Areas
    if (entity_info.type == EntityType::AREA) {
      auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                           filter.include_healed, /*include_muted=*/false, /*include_clusters=*/false);

      if (!result.success) {
        return tl::make_unexpected(
            make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                       json{{"details", result.error_message}, {entity_info.id_field, entity_id}}));
      }

      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto app_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);

      json filtered_faults = faults::filter_faults_by_sources(result.data["faults"], app_fqns);
      json response = {{"items", filtered_faults}};
      const int64_t expected_count = apply_planned_stops(response["items"], *fault_mgr, expected_filter);

      dto::FaultListAggXMedkit xm;
      xm.entity_id = entity_id;
      xm.expected_count = expected_count;
      xm.aggregation_level = "area";
      xm.aggregated = true;
      xm.component_count = static_cast<int64_t>(cache.get_components_for_area(entity_id).size());
      xm.app_count = static_cast<int64_t>(app_fqns.size());
      {
        std::vector<std::string> sources(app_fqns.begin(), app_fqns.end());
        if (!sources.empty()) {
          xm.aggregation_sources = std::move(sources);
        }
      }

      auto xm_json = dto::JsonWriter<dto::FaultListAggXMedkit>::write(xm);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
      merge_peer_items(ctx_.aggregation_manager(), raw_req, response, xm_json);
      xm_json["count"] = static_cast<int64_t>(response["items"].size());
      response["x-medkit"] = std::move(xm_json);
      return wrap_list_result(std::move(response));
    }

    // For Apps, scope by the app's effective FQN set.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto app_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
    auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                         filter.include_healed,
                                         /*include_muted=*/false, /*include_clusters=*/false);
    if (!result.success) {
      return tl::make_unexpected(
          make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                     json{{"details", result.error_message}, {entity_info.id_field, entity_id}}));
    }

    json filtered_faults = faults::filter_faults_by_sources(result.data["faults"], app_fqns);
    json response = {{"items", filtered_faults}};
    const int64_t expected_count = apply_planned_stops(response["items"], *fault_mgr, expected_filter);

    // x-medkit extension for ros2_medkit-specific fields (typed DTO)
    dto::FaultListXMedkit xm;
    xm.entity_id = entity_id;
    xm.expected_count = expected_count;
    xm.source_id = entity_info.namespace_path;

    if (result.data.contains("muted_faults")) {
      xm.muted_faults = result.data["muted_faults"];
    }
    if (result.data.contains("clusters")) {
      xm.clusters = result.data["clusters"];
    }

    auto xm_json = dto::JsonWriter<dto::FaultListXMedkit>::write(xm);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
    merge_peer_items(ctx_.aggregation_manager(), raw_req, response, xm_json);
    xm_json["count"] = static_cast<int64_t>(response["items"].size());
    xm_json["muted_count"] = result.data.value("muted_count", static_cast<int64_t>(0));
    xm_json["cluster_count"] = result.data.value("cluster_count", static_cast<int64_t>(0));
    response["x-medkit"] = std::move(xm_json);
    return wrap_list_result(std::move(response));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in list_faults for entity '%s': %s", entity_id.c_str(), e.what());
    return tl::make_unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to list faults",
                                          json{{"details", e.what()}, {"entity_id", entity_id}}));
  }
}

// =============================================================================
// GET /{entity-path}/faults/{fault_code} - get specific fault
// =============================================================================

http::Result<dto::FaultDetailResult> FaultHandlers::get_fault(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto code_result = read_fault_code(req);
  if (!code_result) {
    return tl::make_unexpected(code_result.error());
  }
  const std::string fault_code = *code_result;

  try {
    // Parse entity path from URL to get entity_path for bulk_data_uri
    auto entity_path_info = parse_entity_path(req.path());
    if (!entity_path_info) {
      return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid entity path"));
    }

    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::make_unexpected(flatten_validator_error(entity_result.error()));
    }
    const auto entity_info = *entity_result;

    // Fault codes may contain dots and underscores; validate basic constraints
    // before any plugin or fault_manager delegation so the plugin path rejects
    // empty/oversized codes the same as the non-plugin path.
    if (fault_code.empty() || fault_code.length() > 256) {
      return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                            json{{"details", "Fault code must be between 1 and 256 characters"}}));
    }

    // Delegate to plugin FaultProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      // Plugins without a FaultProvider report via ReportFault only (the
      // native path): skip the plugin delegation entirely and serve the
      // fault_manager's enriched record through the shared path below.
      if (fault_prov != nullptr) {
        // Prefer the fault_manager's environment-enriched record when this entity
        // OWNS the fault (it was reported to the fault_manager under an app the
        // entity owns, per the fault-scope resolution). This carries the
        // freeze-frame/rosbag snapshots, consistent with the non-plugin detail
        // path. Fall through to the plugin's own provider for faults the
        // fault_manager does not hold (e.g. on-demand UDS DTCs).
        if (auto * fault_mgr = ctx_.node()->get_fault_manager(); fault_mgr != nullptr) {
          auto mgr_result = fault_mgr->get_fault_with_env(fault_code, "");
          if (mgr_result.success) {
            const auto & owned_fault_json = mgr_result.data.value("fault", json::object());
            const auto & cache = ctx_.node()->get_thread_safe_cache();
            auto source_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
            if (FaultHandlers::fault_in_source_scope(owned_fault_json, source_fqns)) {
              json env_data_json = mgr_result.data.value("environment_data", json::object());
              if (auto * capture = ctx_.node()->get_entity_freeze_frame_capture()) {
                env_data_json = merge_entity_freeze_frames(std::move(env_data_json), capture->frames_for(fault_code));
              }
              auto detail = build_sovd_fault_response(owned_fault_json, env_data_json, entity_path_info->entity_path,
                                                      fault_mgr->planned_stop_windows());
              return wrap_detail_result(dto::JsonWriter<dto::FaultDetail>::write(detail));
            }
          }
        }

        try {
          auto result = fault_prov->get_fault(entity_id, fault_code);
          if (!result) {
            return tl::make_unexpected(
                make_plugin_error(result.error().http_status, result.error().message, json{{"entity_id", entity_id}}));
          }
          return std::move(*result);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          return tl::make_unexpected(
              make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
        }
      }
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();

    auto result = fault_mgr->get_fault_with_env(fault_code, "");
    if (!result.success) {
      return tl::make_unexpected(classify_fault_failure(result.failure, result.error_message, "Failed to get fault",
                                                        entity_info.id_field, entity_id, fault_code));
    }

    // Build SOVD-compliant response from the transport-supplied JSON shape.
    const auto & fault_json = result.data.value("fault", json::object());

    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto source_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
    if (!FaultHandlers::fault_in_source_scope(fault_json, source_fqns)) {
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
                     json{{"details",
                           "Fault is not in scope for this entity: every reporting source must be one of the entity's "
                           "owned apps, and a mixed-source fault that includes any out-of-entity reporter is rejected "
                           "to prevent cross-entity disclosure"},
                          {entity_info.id_field, entity_id},
                          {"fault_code", fault_code}}));
    }

    json env_data_json = result.data.value("environment_data", json::object());
    if (auto * capture = ctx_.node()->get_entity_freeze_frame_capture()) {
      env_data_json = merge_entity_freeze_frames(std::move(env_data_json), capture->frames_for(fault_code));
    }
    auto detail = build_sovd_fault_response(fault_json, env_data_json, entity_path_info->entity_path,
                                            fault_mgr->planned_stop_windows());

    return wrap_detail_result(dto::JsonWriter<dto::FaultDetail>::write(detail));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in get_fault for entity '%s', fault '%s': %s", entity_id.c_str(),
                 fault_code.c_str(), e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to get fault",
                   json{{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}}));
  }
}

// =============================================================================
// DELETE /{entity-path}/faults/{fault_code} - clear specific fault
// =============================================================================

http::Result<std::variant<http::NoContent, dto::FaultClearResult>>
FaultHandlers::clear_fault(const http::TypedRequest & req) {
  using Outcome = std::variant<http::NoContent, dto::FaultClearResult>;
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto code_result = read_fault_code(req);
  if (!code_result) {
    return tl::make_unexpected(code_result.error());
  }
  const std::string fault_code = *code_result;

  try {
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::make_unexpected(flatten_validator_error(entity_result.error()));
    }
    const auto entity_info = *entity_result;

    // Check lock access for faults (before plugin delegation - locks apply to all entities)
    if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "faults"); !lock_err) {
      return tl::make_unexpected(lock_err.error());
    }

    // Fault codes may contain dots and underscores; validate basic constraints
    // before any plugin or fault_manager delegation so the plugin path rejects
    // empty/oversized codes the same as the non-plugin path.
    if (fault_code.empty() || fault_code.length() > 256) {
      return tl::make_unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                            json{{"details", "Fault code must be between 1 and 256 characters"}}));
    }

    // Delegate to plugin FaultProvider if entity is plugin-owned. Plugin clear
    // succeeds with the plugin's acknowledgement payload at HTTP 200 (legacy
    // wire shape via `send_dto`); the typed alternates variant maps
    // `FaultClearResult` -> 200 via the default `dto_alternate_status<T>`.
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      // Plugins without a FaultProvider report via ReportFault only (the
      // native path): fall through to the scope-checked fault_manager clear
      // below like any other entity.
      if (fault_prov != nullptr) {
        // Cross-entity clear guard: the plugin clear_fault forwards the code to
        // the fault_manager with no scope check, so without this a
        // DELETE /{A}/faults/{code} could clear a fault owned by entity B. When
        // the fault_manager holds this fault, require it to be in this entity's
        // source scope before delegating; reject out-of-scope. Faults the
        // fault_manager does not hold (plugin-internal, e.g. on-demand UDS DTCs)
        // fall through to the plugin provider unchanged. Mirrors the ownership
        // check in the plugin get_fault branch and the non-plugin clear path.
        if (auto * fault_mgr = ctx_.node()->get_fault_manager(); fault_mgr != nullptr) {
          auto mgr_result = fault_mgr->get_fault_with_env(fault_code, "");
          if (mgr_result.success) {
            const auto & owned_fault_json = mgr_result.data.value("fault", json::object());
            const auto & cache = ctx_.node()->get_thread_safe_cache();
            auto source_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
            if (!FaultHandlers::fault_in_source_scope(owned_fault_json, source_fqns)) {
              return tl::make_unexpected(
                  make_error(404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
                             json{{"details",
                                   "Fault is not in scope for this entity: every reporting source must be one of the "
                                   "entity's owned apps, and a mixed-source fault that includes any out-of-entity "
                                   "reporter is rejected to prevent cross-entity clear"},
                                  {entity_info.id_field, entity_id},
                                  {"fault_code", fault_code}}));
            }
          }
        }

        try {
          auto result = fault_prov->clear_fault(entity_id, fault_code);
          if (!result) {
            return tl::make_unexpected(
                make_plugin_error(result.error().http_status, result.error().message, json{{"entity_id", entity_id}}));
          }
          return Outcome{std::move(*result)};
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          return tl::make_unexpected(
              make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
        }
      }
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // Verify the fault is in this entity's scope BEFORE clearing.
    auto get_result = fault_mgr->get_fault_with_env(fault_code, "");
    if (!get_result.success) {
      return tl::make_unexpected(classify_fault_failure(get_result.failure, get_result.error_message,
                                                        "Failed to clear fault", entity_info.id_field, entity_id,
                                                        fault_code));
    }

    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto source_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
    const auto & fault_json = get_result.data.value("fault", json::object());
    if (!FaultHandlers::fault_in_source_scope(fault_json, source_fqns)) {
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
                     json{{"details",
                           "Fault is not in scope for this entity: every reporting source must be one of the entity's "
                           "owned apps, and a mixed-source fault that includes any out-of-entity reporter is rejected "
                           "to prevent cross-entity disclosure"},
                          {entity_info.id_field, entity_id},
                          {"fault_code", fault_code}}));
    }

    auto result = fault_mgr->clear_fault(fault_code, /*skip_correlation_auto_clear=*/true);
    if (!result.success) {
      return tl::make_unexpected(classify_fault_failure(result.failure, result.error_message, "Failed to clear fault",
                                                        entity_info.id_field, entity_id, fault_code));
    }
    return Outcome{http::NoContent{}};
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in clear_fault for entity '%s', fault '%s': %s", entity_id.c_str(),
                 fault_code.c_str(), e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to clear fault",
                   json{{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}}));
  }
}

// =============================================================================
// DELETE /{entity-path}/faults - clear all faults for entity
// =============================================================================

http::Result<http::NoContent> FaultHandlers::clear_all_faults(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  try {
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::make_unexpected(flatten_validator_error(entity_result.error()));
    }
    const auto entity_info = *entity_result;

    // Check lock access for faults (before plugin delegation - locks apply to all entities)
    if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "faults"); !lock_err) {
      return tl::make_unexpected(lock_err.error());
    }

    // Delegate to plugin FaultProvider if entity is plugin-owned. Plugins
    // without a FaultProvider report via ReportFault only (the native path):
    // fall through to the scope-checked fault_manager clear below.
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov != nullptr) {
        try {
          auto list_result = fault_prov->list_faults(entity_id);
          if (!list_result) {
            return tl::make_unexpected(make_plugin_error(list_result.error().http_status, list_result.error().message,
                                                         json{{"entity_id", entity_id}}));
          }
          if (list_result->content.contains("items") && list_result->content["items"].is_array()) {
            std::vector<std::string> failed_codes;
            for (const auto & fault : list_result->content["items"]) {
              auto code = fault.value("code", "");
              if (code.empty()) {
                continue;
              }
              auto clear_result = fault_prov->clear_fault(entity_id, code);
              if (!clear_result) {
                failed_codes.push_back(code);
              }
            }
            if (!failed_codes.empty()) {
              return tl::make_unexpected(
                  make_plugin_error(500, "Failed to clear " + std::to_string(failed_codes.size()) + " fault(s)",
                                    json{{"entity_id", entity_id}, {"failed_codes", failed_codes}}));
            }
          }
          return http::NoContent{};
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          return tl::make_unexpected(
              make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
        }
      }
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // Same scope rules as `list_faults` and `get_fault`: every entity type
    // resolves through `HandlerContext::resolve_entity_source_fqns` so the
    // area BFS, function-hosting-component expansion, and wildcard-app
    // empty-set behavior stay consistent across all four fault routes.
    auto result = fault_mgr->list_faults("");
    if (!result.success) {
      return tl::make_unexpected(
          make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                     json{{"details", result.error_message}, {entity_info.id_field, entity_id}}));
    }
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto entity_fqns = HandlerContext::resolve_entity_source_fqns(cache, entity_info);
    json faults_to_clear = faults::filter_faults_by_sources(result.data["faults"], entity_fqns);

    // Clear each matching fault. Use `skip_correlation_auto_clear=true` for
    // the same reason as the single-fault DELETE: keep this entity's clear
    // from cascading into correlated symptoms reported by other entities.
    if (faults_to_clear.is_array()) {
      for (const auto & fault : faults_to_clear) {
        if (!fault.contains("fault_code")) {
          continue;
        }
        std::string code = fault["fault_code"].get<std::string>();
        auto clear_result = fault_mgr->clear_fault(code, /*skip_correlation_auto_clear=*/true);
        if (!clear_result.success) {
          RCLCPP_WARN(HandlerContext::logger(), "Failed to clear fault '%s' for entity '%s': %s", code.c_str(),
                      entity_id.c_str(), clear_result.error_message.c_str());
        }
      }
    }
    return http::NoContent{};
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in clear_all_faults for entity '%s': %s", entity_id.c_str(),
                 e.what());
    return tl::make_unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to clear faults",
                                          json{{"details", e.what()}, {"entity_id", entity_id}}));
  }
}

// =============================================================================
// DELETE /faults - clear all faults globally (extension, not SOVD)
// =============================================================================

http::Result<std::pair<http::NoContent, http::ResponseAttachments>>
FaultHandlers::clear_all_faults_global(const http::TypedRequest & req) {
  try {
    const auto q = req.query<dto::FaultClearQuery>();
    auto filter_result = read_fault_status_filter(q.status);
    if (!filter_result) {
      return tl::make_unexpected(filter_result.error());
    }
    const auto filter = *filter_result;

    auto fault_mgr = ctx_.node()->get_fault_manager();
    // Global clear is the "nuclear option" - always include muted (correlated)
    // faults, unlike per-entity clear which respects the default
    // include_muted=false.
    auto faults_result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed,
                                                filter.include_cleared, filter.include_healed, true);
    if (!faults_result.success) {
      return tl::make_unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                            json{{"details", faults_result.error_message}}));
    }

    // Build FQN-to-entity-ID map for lock checking
    auto * lock_mgr = ctx_.node() ? ctx_.node()->get_lock_manager() : nullptr;
    std::unordered_map<std::string, std::string> fqn_to_entity;
    if (lock_mgr) {
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      for (const auto & app : cache.get_apps()) {
        auto fqn = app.effective_fqn();
        if (!fqn.empty()) {
          fqn_to_entity[fqn] = app.id;
        }
      }
    }

    auto client_id = req.header("X-Client-Id").value_or(std::string{});

    // Clear each fault, skipping those on locked entities
    if (faults_result.data.contains("faults") && faults_result.data["faults"].is_array()) {
      for (const auto & fault : faults_result.data["faults"]) {
        if (!fault.contains("fault_code")) {
          continue;
        }

        // Check if any reporting source is on a locked entity
        bool blocked = false;
        if (lock_mgr && fault.contains("reporting_sources")) {
          for (const auto & src : fault["reporting_sources"]) {
            auto src_str = src.get<std::string>();
            auto it = fqn_to_entity.find(src_str);
            if (it != fqn_to_entity.end()) {
              auto access = lock_mgr->check_access(it->second, client_id, "faults");
              if (!access.allowed) {
                blocked = true;
                break;
              }
            }
          }
        }

        if (blocked) {
          continue;
        }

        std::string code = fault["fault_code"].get<std::string>();
        auto clear_result = fault_mgr->clear_fault(code);
        if (!clear_result.success) {
          RCLCPP_WARN(HandlerContext::logger(), "Failed to clear fault '%s': %s", code.c_str(),
                      clear_result.error_message.c_str());
        }
      }
    }

    // Design limitation: this only clears faults on the local FaultManager.
    // Peer faults visible via fan_out_get in `list_all_faults` are NOT cleared
    // because that would require fan-out DELETE requests to each peer, which
    // introduces distributed transaction semantics (partial failures, rollback)
    // that are out of scope. Clients should clear peer faults by calling each
    // peer's clear endpoint directly.
    //
    // The X-Medkit-Local-Only header signals this to clients. A 204 response
    // cannot carry a JSON body per HTTP spec, so we use a header instead.
    http::ResponseAttachments att;
    att.with_header("X-Medkit-Local-Only", "true");
    return std::pair{http::NoContent{}, std::move(att)};
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in clear_all_faults_global: %s", e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to clear faults", json{{"details", e.what()}}));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
