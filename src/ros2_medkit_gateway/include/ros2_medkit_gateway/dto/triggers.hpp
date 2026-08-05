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

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// Trigger - SOVD trigger CRUD response object.
//
// Emitted by handle_create (201), handle_list (items element),
// handle_get (200), handle_update (200).
//
// Wire keys (from trigger_info_to_dto):
//   id               - trigger UUID (required)
//   status           - enum: "active"|"terminated" (required)
//   observed_resource - resource URI being observed (required)
//   event_source     - server-generated SSE stream URI (required)
//   protocol         - transport protocol, e.g. "sse" (required)
//   trigger_condition - flat JSON object: condition_type + merged condition_params
//                       (required; free-form, additionalProperties: true)
//   multishot        - whether trigger fires multiple times (required)
//   persistent       - whether trigger survives server restarts (required)
//   lifetime         - trigger lifetime in seconds (optional)
//   path             - JSON Pointer notification delivery path (optional)
//   log_settings     - free-form log capture settings (optional)
// =============================================================================
struct Trigger {
  std::string id;
  std::string status;             // enum: "active"|"terminated"
  std::string observed_resource;  // wire key: "observed_resource"
  std::string event_source;       // wire key: "event_source"
  std::string protocol;
  nlohmann::json trigger_condition;  // flat merged object (free-form JSON)
  bool multishot{false};
  bool persistent{false};
  std::optional<int> lifetime;
  std::optional<std::string> path;
  std::optional<nlohmann::json> log_settings;
};

template <>
inline constexpr auto dto_fields<Trigger> =
    std::make_tuple(field("id", &Trigger::id), field_enum("status", &Trigger::status, kTriggerStatusValues),
                    field("observed_resource", &Trigger::observed_resource),
                    field("event_source", &Trigger::event_source), field("protocol", &Trigger::protocol),
                    field("trigger_condition", &Trigger::trigger_condition), field("multishot", &Trigger::multishot),
                    field("persistent", &Trigger::persistent), field("lifetime", &Trigger::lifetime),
                    field("path", &Trigger::path), field("log_settings", &Trigger::log_settings));

template <>
inline constexpr std::string_view dto_name<Trigger> = "Trigger";

// =============================================================================
// TriggerCreateRequest - POST /{entity}/triggers request body.
// Parsed by handle_create via parse_body<TriggerCreateRequest>.
//
// Wire keys (from handle_create body parsing + trigger_create_request_schema):
//   resource         - resource URI to observe (required)
//   trigger_condition - flat condition object with condition_type + extra fields
//                       (required; parsed as raw JSON then dissected by handler)
//   protocol         - transport protocol, default "sse" (optional)
//   multishot        - fire multiple times (optional)
//   persistent       - survive server restarts (optional)
//   lifetime         - lifetime in seconds, must be > 0 (optional)
//   path             - JSON Pointer delivery path (optional)
//   log_settings     - free-form log capture settings (optional)
// =============================================================================
struct TriggerCreateRequest {
  std::string resource;
  nlohmann::json trigger_condition;  // parsed raw; handler extracts condition_type
  std::optional<std::string> protocol;
  std::optional<bool> multishot;
  std::optional<bool> persistent;
  std::optional<int> lifetime;
  std::optional<std::string> path;
  std::optional<nlohmann::json> log_settings;
};

template <>
inline constexpr auto dto_fields<TriggerCreateRequest> = std::make_tuple(
    field("resource", &TriggerCreateRequest::resource),
    field("trigger_condition", &TriggerCreateRequest::trigger_condition),
    field("protocol", &TriggerCreateRequest::protocol), field("multishot", &TriggerCreateRequest::multishot),
    field("persistent", &TriggerCreateRequest::persistent), field("lifetime", &TriggerCreateRequest::lifetime),
    field("path", &TriggerCreateRequest::path), field("log_settings", &TriggerCreateRequest::log_settings));

template <>
inline constexpr std::string_view dto_name<TriggerCreateRequest> = "TriggerCreateRequest";

// =============================================================================
// TriggerUpdateRequest - PUT /{entity}/triggers/{trigger_id} request body.
// Parsed by handle_update via parse_body<TriggerUpdateRequest>.
//
// Wire keys (from handle_update body parsing + trigger_update_request_schema):
//   lifetime - new lifetime in seconds, must be > 0 (required)
// =============================================================================
struct TriggerUpdateRequest {
  int lifetime{0};  // additional validation: must be > 0
};

template <>
inline constexpr auto dto_fields<TriggerUpdateRequest> =
    std::make_tuple(field("lifetime", &TriggerUpdateRequest::lifetime));

template <>
inline constexpr std::string_view dto_name<TriggerUpdateRequest> = "TriggerUpdateRequest";

// =============================================================================
// Collection<Trigger> - named "TriggerList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<Trigger>> = "TriggerList";

// =============================================================================
// trigger_condition: opaque, and saying why.
//
// The member is a bare `nlohmann::json`, so the derived schema publishes `{}` -
// "any JSON at all" - on a *required* field. That is the same content-free
// declaration the fault and data envelopes were cured of, and it earns the same
// treatment: mark it opaque and name who decides the shape.
//
// Opaque rather than typed because the vocabulary is open. `ConditionRegistry`
// takes plugin-registered evaluators through `PluginContext`
// (`get_condition_registry()`), each with its own `condition_type` and its own
// operands, and the handler forwards every key except `condition_type` to the
// evaluator untouched. A closed schema would both misdescribe a plugin
// condition and, if the member were typed, silently drop its operands on read.
//
// Schema-only: `JsonWriter` / `JsonReader` keep walking the plain `field()`
// descriptor, so the reader stays as lenient as it is today and the handler
// keeps issuing its own 400 for a non-object. Typing the member properly is a
// separate, larger change - it needs a descriptor that publishes a $ref while
// leaving the member raw.
// =============================================================================
namespace detail {
inline nlohmann::json trigger_condition_schema() {
  return nlohmann::json{
      {"type", "object"},
      {"additionalProperties", true},
      {"x-medkit-opaque", true},
      {"description",
       "The condition that fires the trigger: a flat object of `condition_type` plus that type's own "
       "operands. The built-in types are `OnChange` (no operands), `OnChangeTo` (`target_value`), "
       "`EnterRange` and `LeaveRange` (`lower_bound`, `upper_bound`). The set is open - a plugin can "
       "register further evaluators through `ConditionRegistry`, conventionally under an `x-` prefixed "
       "name, with operands of its own - which is why no closed schema is published here."}};
}
}  // namespace detail

// SchemaWriter specializations: start from the derived schema so every other
// member still comes from `dto_fields`, and replace only `trigger_condition`.
template <>
struct SchemaWriter<Trigger> {
  static nlohmann::json schema() {
    nlohmann::json schema = derived_object_schema<Trigger>();
    schema["properties"]["trigger_condition"] = detail::trigger_condition_schema();
    return schema;
  }
};

template <>
struct SchemaWriter<TriggerCreateRequest> {
  static nlohmann::json schema() {
    nlohmann::json schema = derived_object_schema<TriggerCreateRequest>();
    schema["properties"]["trigger_condition"] = detail::trigger_condition_schema();
    return schema;
  }
};

// =============================================================================
// dto_sample specializations for DTOs with bare nlohmann::json members.
//
// The generic sample path produces nlohmann::json{} (null) for bare json
// fields, which the round-trip reader treats as missing (null == absent for
// required fields).  Provide explicit samples with non-null values so that
// EveryRegisteredDtoRoundTrips passes.
// =============================================================================
template <>
struct dto_sample<Trigger> {
  static Trigger make() {
    Trigger obj;
    obj.id = "sample";
    obj.status = "active";
    obj.observed_resource = "sample";
    obj.event_source = "sample";
    obj.protocol = "sample";
    obj.trigger_condition = nlohmann::json{{"condition_type", "OnChange"}};  // non-null required field
    obj.multishot = true;
    obj.persistent = true;
    return obj;
  }
};

template <>
struct dto_sample<TriggerCreateRequest> {
  static TriggerCreateRequest make() {
    TriggerCreateRequest obj;
    obj.resource = "sample";
    obj.trigger_condition = nlohmann::json{{"condition_type", "OnChange"}};  // non-null required field
    return obj;
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
