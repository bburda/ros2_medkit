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
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_types.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for fault management REST API endpoints.
 *
 * Provides handlers for:
 * - GET /faults - List all faults (extension)
 * - DELETE /faults - Clear all faults globally (extension)
 * - GET /{entity-path}/faults - List faults for entity
 * - GET /{entity-path}/faults/{code} - Get specific fault
 * - DELETE /{entity-path}/faults/{code} - Clear fault
 * - DELETE /{entity-path}/faults - Clear all faults for entity
 *
 * PR-403 commit 29: all 6 fault routes migrate to the typed RouteRegistry API.
 *
 * The list endpoints return the opaque `FaultListResult` envelope rather than
 * `Collection<FaultListItem, X>` because the per-entity-type x-medkit shape
 * varies between branches (FaultListXMedkit for Apps and global; the
 * aggregation variant FaultListAggXMedkit for Component / Area / Function).
 * `FaultListResult.content` lets the handler emit the wire payload it built
 * (including the typed x-medkit DTO serialised in place) verbatim. Plugin
 * pass-through (FaultProvider::list_faults) already returns FaultListResult so
 * both branches converge on a single typed return type.
 *
 * `get_fault` returns `FaultDetailResult` for the same reason: the ROS-path
 * wraps the SOVD-compliant FaultDetail DTO and the plugin path forwards a
 * vendor-defined payload; the opaque envelope keeps the typed signature
 * uniform while preserving wire bytes.
 *
 * `DELETE /faults` (global) uses the attachments variant
 * `Result<std::pair<NoContent, ResponseAttachments>>` so it can attach the
 * `X-Medkit-Local-Only: true` response header on top of the framework-default
 * 204 No Content.
 *
 * Note: Snapshot data is inline in fault responses (environment_data).
 * Rosbag downloads use the bulk-data endpoint pattern.
 */
class FaultHandlers {
 public:
  /**
   * @brief Construct fault handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit FaultHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// GET /faults - list all faults globally (extension, not SOVD).
  ///
  /// Returns `FaultListResult` whose `content` is the `{items, x-medkit}`
  /// object - opaque so the typed fan-out merge from the legacy path stays
  /// byte-identical to the pre-migration wire format.
  http::Result<dto::FaultListResult> list_all_faults(const http::TypedRequest & req);

  /// GET /{entity-path}/faults - list faults for entity.
  ///
  /// Returns `FaultListResult` for the same reason as `list_all_faults` plus
  /// the per-entity-type aggregation variants (Function / Component / Area
  /// emit FaultListAggXMedkit; App emits FaultListXMedkit). Plugin-owned
  /// entities delegate to `FaultProvider::list_faults` which already returns
  /// `FaultListResult`.
  http::Result<dto::FaultListResult> list_faults(const http::TypedRequest & req);

  /// GET /{entity-path}/faults/{fault_code} - get specific fault.
  ///
  /// Returns `FaultDetailResult` whose `content` is the SOVD-compliant detail
  /// payload (item / environment_data / x-medkit). Plugin pass-through forwards
  /// the vendor-defined detail JSON verbatim.
  http::Result<dto::FaultDetailResult> get_fault(const http::TypedRequest & req);

  /// DELETE /{entity-path}/faults/{fault_code} - clear specific fault.
  ///
  /// Returns one of two alternates: `NoContent` (ROS path -> 204) or
  /// `FaultClearResult` (plugin path -> 200 + plugin acknowledgement body).
  /// Two-status return is required to keep wire bytes byte-identical with the
  /// legacy code: plugin acknowledgement payloads (UDS clear response codes,
  /// vendor warnings) are forwarded verbatim with HTTP 200.
  http::Result<std::variant<http::NoContent, dto::FaultClearResult>> clear_fault(const http::TypedRequest & req);

  /// DELETE /{entity-path}/faults - clear all faults for entity.
  ///
  /// Always returns 204 on success - the bulk-clear path never carried a
  /// plugin acknowledgement payload on success in the legacy implementation
  /// (the plugin branch set `res.status = 204` directly after iterating the
  /// per-fault clear calls).
  http::Result<http::NoContent> clear_all_faults(const http::TypedRequest & req);

  /// DELETE /faults - clear all faults globally (extension, not SOVD).
  ///
  /// Accepts an optional `?status=` query parameter to filter which faults to
  /// clear. Returns 204 + `X-Medkit-Local-Only: true` header via the typed
  /// attachments variant; the framework-default 204 status is kept.
  http::Result<std::pair<http::NoContent, http::ResponseAttachments>>
  clear_all_faults_global(const http::TypedRequest & req);

  /**
   * @brief Build SOVD-compliant fault response from already-converted JSON.
   *
   * Consumes the JSON shape returned by FaultManager::get_fault_with_env -
   * `fault_json` is the per-fault flat representation (the FaultManager has
   * already translated severity_label, status string, timestamps, etc.) and
   * `env_data_json` carries `extended_data_records` plus a `snapshots` array
   * with intermediate-shape entries (freeze_frame: type/name/data/topic/
   * message_type/captured_at_ns; rosbag: type/name/fault_code/size_bytes/
   * duration_sec/format).
   *
   * Builds the final SOVD response containing:
   * - "item" with fault details and SOVD status object
   * - "environment_data" with extended_data_records and snapshots; for
   *   freeze_frame snapshots the handler parses "data", extracts the primary
   *   value and packs the full payload into x-medkit; for rosbag snapshots
   *   the handler appends `bulk_data_uri = entity_path + "/bulk-data/rosbags/" + fault_code`.
   * - "x-medkit" extensions with occurrence_count, severity_label, etc.
   *
   * @param fault_json Per-fault JSON (as produced by the transport adapter).
   * @param env_data_json Environment-data JSON (as produced by the transport).
   * @param entity_path Entity path used to construct rosbag bulk_data_uri.
   * @return SOVD-compliant FaultDetail DTO
   */
  static dto::FaultDetail build_sovd_fault_response(const nlohmann::json & fault_json,
                                                    const nlohmann::json & env_data_json,
                                                    const std::string & entity_path);

  /**
   * @brief Scope check used by per-entity fault routes.
   *
   * Returns true iff every entry in `fault["reporting_sources"]` is in scope
   * for `source_fqns`. A source matches a scope FQN when it equals the FQN
   * or is a strict path-child (i.e. `<fqn>/<...>`), so similarly named nodes
   * like `/ns/node` and `/ns/node_extra` are not conflated.
   *
   * The "all sources must match" semantic (rather than "any source") is
   * deliberate: it blocks two cross-entity escalation paths.
   *
   * 1. GET would otherwise return a response whose `reporting_sources` and
   *    environment data carry identities of nodes the caller has no
   *    business reading.
   * 2. DELETE would otherwise let a viewer of entity A clear the aggregated
   *    fault record for a fault that entity B also reports, because the
   *    underlying `ClearFault.srv` has no scope argument.
   *
   * An empty scope set, an empty `reporting_sources` array, a missing
   * `reporting_sources` field, or any non-string source entry all return
   * false - there is no vacuous "all match" case.
   *
   * Public for direct unit testing; called by `get_fault`, `clear_fault`,
   * and indirectly via the per-entity collection routes.
   */
  static bool fault_in_source_scope(const nlohmann::json & fault, const std::set<std::string> & source_fqns);

  /**
   * @brief Map a failed fault-manager call onto an HTTP error.
   *
   * Only a call that never obtained an answer is the server's fault. Anything
   * the fault manager itself answered - a code it does not hold, a code it
   * refuses to accept - is a condition of the request and is reported as 404,
   * which is what the fault routes document as well.
   *
   * The classification comes from `FaultFailure`, which the transport sets
   * from whether it received a response. It was previously derived from the
   * store's message text, and so any refusal worded in a way the matcher did
   * not anticipate - the fault manager's own `fault_code` validation among
   * them - was served to the client as 503.
   *
   * Public for direct unit testing; called by `get_fault` and `clear_fault`.
   *
   * @param failure Which layer failed, as reported by the transport.
   * @param error_message The store's message, passed through as `details`.
   * @param unavailable_summary Summary for the 503 case, per calling verb.
   * @param id_field Entity id field name for the error parameters.
   * @param entity_id Entity the request addressed.
   * @param fault_code Fault code the request addressed.
   */
  static ErrorInfo classify_fault_failure(FaultFailure failure, const std::string & error_message,
                                          const std::string & unavailable_summary, const std::string & id_field,
                                          const std::string & entity_id, const std::string & fault_code);

  /**
   * @brief Merge zero-config entity freeze-frames into environment data.
   *
   * Appends one intermediate-shape freeze_frame snapshot per captured frame
   * (same shape as the transport emits, so build_sovd_fault_response processes
   * them identically) UNLESS the fault_manager already captured a freeze-frame
   * for this fault - explicit snapshot config always wins over the zero-config
   * entity capture.
   *
   * Public static for direct unit testing; called by `get_fault`.
   */
  static nlohmann::json merge_entity_freeze_frames(nlohmann::json env_data,
                                                   const std::vector<EntityFreezeFrameCapture::Frame> & frames);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
