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

#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// UpdateList - response for GET /updates
//
// Wire shape: {"items": ["update_id_1", "update_id_2", ...]}
// The items array contains bare strings (update package IDs).
// =============================================================================
struct UpdateList {
  std::vector<std::string> items;
};

template <>
inline constexpr auto dto_fields<UpdateList> = std::make_tuple(field("items", &UpdateList::items));

template <>
inline constexpr std::string_view dto_name<UpdateList> = "UpdateList";

// =============================================================================
// UpdateDetail - response for GET /updates/{update_id}
//
// Wire shape: the full metadata object the plugin stored on register_update.
// Per the SOVD spec (ISO 17978-3, §7.18) the body MUST include id, update_name,
// automated, origins; many optional keys are defined plus arbitrary vendor
// extensions are permitted. Plugins (Uptane OTA, OTA, demo backends) retain the
// raw JSON metadata verbatim because keys outside the SOVD vocabulary (e.g.
// Uptane TUF metadata, vendor-specific component lists) must round-trip
// untouched between register/get.
//
// Typed envelope around the raw object: the C++ ABI is typed, the wire bytes
// are unchanged. JsonWriter/JsonReader/SchemaWriter are fully specialized so
// the wrapper is transparent at the wire layer.
// =============================================================================
struct UpdateDetail {
  nlohmann::json content;  // wire shape: the bare metadata object
};

// Empty dto_fields<> specialization: flips `is_dto_v<UpdateDetail>` to true so
// the typed RouteRegistry accepts UpdateDetail as a response type. The tuple
// is intentionally empty because UpdateDetail's three visitors
// (JsonWriter/JsonReader/SchemaWriter) are fully specialized below and do NOT
// fold over `dto_fields` - they just pass `content` through. `for_each_field`
// is never instantiated for UpdateDetail, so the empty tuple has no runtime
// effect; only the type-level "is this a DTO" check needs to flip.
template <>
inline constexpr auto dto_fields<UpdateDetail> = std::make_tuple();

template <>
inline constexpr std::string_view dto_name<UpdateDetail> = "UpdateDetail";

// JsonWriter specialization: emit the bare metadata object, not a wrapper.
template <>
struct JsonWriter<UpdateDetail> {
  static nlohmann::json write(const UpdateDetail & obj) {
    return obj.content;
  }
};

// JsonReader specialization: wrap any input object into UpdateDetail::content.
// Rejects non-object payloads so consumers get a structured FieldError instead
// of a silent type confusion. Round-trips JsonWriter byte-for-byte.
template <>
struct JsonReader<UpdateDetail> {
  static tl::expected<UpdateDetail, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    UpdateDetail out;
    out.content = j;
    return out;
  }
};

// SchemaWriter specialization. Unlike JsonWriter / JsonReader above, this is
// NOT a pass-through: SOVD (ISO 17978-3 section 7.18) defines the attribute
// table, so publishing `{type: object}` withheld a shape the standard already
// fixes. The keys below are that table; `additionalProperties: true` is what
// keeps the vendor extensions the reader passes through legal, so typing the
// schema costs a plugin nothing.
//
// Deliberately no `required` list. The four mandatory attributes are mandatory
// on the *SOVD server*, and the gateway is a pass-through here - it stores
// whatever `register_update` was given and returns it. Declaring them required
// would make the document promise something the gateway does not enforce.
template <>
struct SchemaWriter<UpdateDetail> {
  static nlohmann::json schema() {
    using nlohmann::json;
    const json string_array{{"type", "array"}, {"items", {{"type", "string"}}}};
    return json{
        {"type", "object"},
        {"additionalProperties", true},
        {"description",
         "Update package metadata, as stored by the UpdateProvider. The attributes below are SOVD's "
         "(ISO 17978-3 section 7.18); a backend may add its own - Uptane TUF metadata, vendor component "
         "lists - and the gateway returns those unchanged."},
        {"properties", json{{"id", {{"type", "string"}, {"description", "Update package identifier."}}},
                            {"update_name", {{"type", "string"}, {"description", "Display name."}}},
                            {"update_translation_id",
                             {{"type", "string"}, {"description", "Translation identifier for the display name."}}},
                            {"automated",
                             {{"type", "boolean"},
                              {"description",
                               "Whether the update can run unattended. `PUT /updates/{id}/automated` "
                               "answers 400 when this is false."}}},
                            {"origins",
                             {{"type", "array"},
                              {"items", {{"type", "string"}, {"enum", json::array({"remote", "proximity"})}}},
                              {"description", "Where the package can be fetched from."}}},
                            {"notes", {{"type", "string"}, {"description", "Additional descriptive notes."}}},
                            {"user_activity", {{"type", "string"}, {"description", "Actions the user has to take."}}},
                            {"preconditions", {{"type", "string"}, {"description", "Preconditions for the update."}}},
                            {"execution_conditions",
                             {{"type", "string"}, {"description", "Conditions that must hold during execution."}}},
                            {"duration", {{"type", "integer"}, {"description", "Estimated duration in seconds."}}},
                            {"size", {{"type", "integer"}, {"description", "Package size in kilobytes."}}},
                            {"added_components", string_array},
                            {"removed_components", string_array},
                            {"updated_components", string_array},
                            {"affected_components", string_array}}}};
  }
};

// dto_sample specialization: the generic path would call for_each_field, but
// UpdateDetail bypasses the field-walking visitors. Hand-build a sample with
// the SOVD-spec mandatory keys so EveryRegisteredDtoRoundTrips passes the
// JsonWriter -> JsonReader round-trip on a non-trivial payload.
template <>
struct dto_sample<UpdateDetail> {
  static UpdateDetail make() {
    UpdateDetail obj;
    obj.content = nlohmann::json{{"id", "sample"},
                                 {"update_name", "Sample Update"},
                                 {"automated", false},
                                 {"origins", nlohmann::json::array({"remote"})}};
    return obj;
  }
};

// =============================================================================
// UpdateSubProgress - a single sub-step progress entry.
//
// Wire shape (from update_status_to_json in update_types.hpp):
//   name     - sub-step name (required)
//   progress - sub-step progress percentage 0-100 (required)
//
// Nested inside UpdateStatus::sub_progress array.
// =============================================================================
struct UpdateSubProgress {
  std::string name;
  int progress{0};
};

template <>
inline constexpr auto dto_fields<UpdateSubProgress> =
    std::make_tuple(field("name", &UpdateSubProgress::name), field("progress", &UpdateSubProgress::progress));

template <>
inline constexpr std::string_view dto_name<UpdateSubProgress> = "UpdateSubProgress";

// =============================================================================
// XMedkitUpdate - typed x-medkit vendor extension on update status responses.
//
// Emitted by handle_get_status (and the SSE sampler via update_status_to_json).
// Wire keys (from update_status_to_json in update_types.hpp):
//
//   phase - internal lifecycle phase, distinguishes prepare-completed from
//           execute-completed (required; always emitted by update_status_to_json)
//           enum: none | preparing | prepared | executing | executed | failed | deleting
//
// Uses field_enum: phase is a RESPONSE-side field; the handler does NOT perform
// bespoke validation of the phase value in any request.
// =============================================================================
struct XMedkitUpdate {
  std::string phase;  // enum: kUpdatePhaseValues
};

template <>
inline constexpr auto dto_fields<XMedkitUpdate> =
    std::make_tuple(field_enum("phase", &XMedkitUpdate::phase, kUpdatePhaseValues));

template <>
inline constexpr std::string_view dto_name<XMedkitUpdate> = "XMedkitUpdate";

// =============================================================================
// UpdateStatus - response for GET /updates/{update_id}/status
//
// Wire shape (from update_status_to_json in update_types.hpp):
//   status       - SOVD update status enum (required)
//                  enum: pending | inProgress | completed | failed
//   progress     - optional overall progress percentage (0-100)
//   sub_progress - optional array of per-step progress entries
//   error        - optional error message string (set when status == failed)
//   x-medkit     - typed vendor extension (required; always emitted)
//
// status uses field_enum (response DTO, no bespoke handler-side range check).
// x-medkit is required: update_status_to_json always sets j["x-medkit"] unconditionally.
// =============================================================================
struct UpdateStatus {
  std::string status;           // enum: kUpdateStatusValues
  std::optional<int> progress;  // 0-100
  std::optional<std::vector<UpdateSubProgress>> sub_progress;
  std::optional<std::string> error;
  XMedkitUpdate x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<UpdateStatus> =
    std::make_tuple(field_enum("status", &UpdateStatus::status, kUpdateStatusValues),
                    field("progress", &UpdateStatus::progress), field("sub_progress", &UpdateStatus::sub_progress),
                    field("error", &UpdateStatus::error), field("x-medkit", &UpdateStatus::x_medkit));

template <>
inline constexpr std::string_view dto_name<UpdateStatus> = "UpdateStatus";

// =============================================================================
// UpdateRegisterRequest - request body for POST /updates.
//
// Wire shape: the bare metadata object the plugin will store. Per SOVD spec
// (ISO 17978-3, §7.18) the body MUST include `id`, `update_name`, `automated`,
// `origins`; arbitrary vendor extensions are permitted (Uptane TUF metadata,
// vendor component lists, etc.) and must round-trip untouched into
// `register_update(json)`. The opaque envelope mirrors `UpdateDetail` so the
// register/get pair shares the same shape on both directions of the wire.
//
// Why opaque: validating `id` in the handler is the only field-level check the
// gateway performs (CRLF-injection-safe Location header). All other fields are
// forwarded verbatim to the UpdateProvider; modelling them statically would
// drop vendor extensions.
// =============================================================================
struct UpdateRegisterRequest {
  nlohmann::json content;  // wire shape: the bare metadata object
};

// Empty dto_fields<> specialization: same rationale as UpdateDetail above -
// flips `is_dto_v<UpdateRegisterRequest>` to true without instantiating the
// field-fold visitor.
template <>
inline constexpr auto dto_fields<UpdateRegisterRequest> = std::make_tuple();

template <>
inline constexpr std::string_view dto_name<UpdateRegisterRequest> = "UpdateRegisterRequest";

template <>
struct JsonWriter<UpdateRegisterRequest> {
  static nlohmann::json write(const UpdateRegisterRequest & obj) {
    return obj.content;
  }
};

template <>
struct JsonReader<UpdateRegisterRequest> {
  static tl::expected<UpdateRegisterRequest, std::vector<FieldError>> read(const nlohmann::json & j) {
    if (!j.is_object()) {
      return tl::make_unexpected(std::vector<FieldError>{FieldError{"", "expected a JSON object"}});
    }
    UpdateRegisterRequest out;
    out.content = j;
    return out;
  }
};

// SchemaWriter specialization. Only `id` is declared, and only `id` is
// required, because that is the entire contract the gateway enforces:
// post_update checks its presence, type and format and forwards every other
// key verbatim to `register_update`. SOVD makes the register *request* body
// manufacturer-specific, so transplanting UpdateDetail's attribute table here
// would document a validation the gateway does not perform and a shape a
// backend need not accept.
template <>
struct SchemaWriter<UpdateRegisterRequest> {
  static nlohmann::json schema() {
    return nlohmann::json{
        {"type", "object"},
        {"additionalProperties", true},
        {"required", nlohmann::json::array({"id"})},
        {"description",
         "Update package metadata to store. Only `id` is validated by the gateway; every other key is "
         "forwarded to the UpdateProvider unchanged, so what else belongs here is the backend's contract. "
         "A backend that follows SOVD will want the `UpdateDetail` attributes, since that is what "
         "`GET /updates/{update_id}` returns."},
        {"properties",
         nlohmann::json{{"id",
                         {{"type", "string"},
                          {"description",
                           "Update package identifier, also the path segment of the `Location` header returned on "
                           "201. Must be non-empty and free of characters that would not survive a URI path."}}}}}};
  }
};

template <>
struct dto_sample<UpdateRegisterRequest> {
  static UpdateRegisterRequest make() {
    UpdateRegisterRequest obj;
    obj.content = nlohmann::json{{"id", "sample"},
                                 {"update_name", "Sample Update"},
                                 {"automated", false},
                                 {"origins", nlohmann::json::array({"remote"})}};
    return obj;
  }
};

// =============================================================================
// UpdateRegisterResponse - success body for POST /updates.
//
// Wire shape: `{"id": "<package_id>"}`. The handler emits a 201 status with a
// `Location: /updates/<id>` header (via ResponseAttachments) and this body.
// Integration tests (test_updates.test.py) assert on the Location header; the
// body shape is part of the existing contract.
// =============================================================================
struct UpdateRegisterResponse {
  std::string id;
};

template <>
inline constexpr auto dto_fields<UpdateRegisterResponse> = std::make_tuple(field("id", &UpdateRegisterResponse::id));

template <>
inline constexpr std::string_view dto_name<UpdateRegisterResponse> = "UpdateRegisterResponse";

// =============================================================================
// UpdateListQuery - query parameters for GET /updates. Read by the handler via
// TypedRequest::query<UpdateListQuery>() and declared in the OpenAPI spec via
// the same descriptor, so the two cannot drift.
// =============================================================================
struct UpdateListQuery {
  std::optional<std::string> origin;
  std::optional<std::string> target_version;
};

template <>
inline constexpr auto dto_fields<UpdateListQuery> =
    std::make_tuple(field("origin", &UpdateListQuery::origin, "Filter by update origin identifier"),
                    field("target-version", &UpdateListQuery::target_version, "Filter by target version"));

}  // namespace dto
}  // namespace ros2_medkit_gateway
