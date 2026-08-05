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
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// GenericError is the SOVD error body, and `SchemaBuilder::generic_error()`
// delegates here so the sub-page specs cannot describe a different shape.
//
//   required: error_code, message
//   optional: parameters, vendor_code
//
// `vendor_code` is a real fourth key, not a reserve: `write_generic_error`
// (core/http/detail/primitives.cpp) rewrites `error_code` to the `vendor-error`
// sentinel and moves the original into `vendor_code` whenever the code is an
// `x-medkit-*` one. A client matching on `error_code` alone therefore sees
// every vendor failure collapse into one value, and the schema said nothing
// about where the real one went.
struct GenericError {
  std::string error_code;
  std::string message;
  std::optional<nlohmann::json> parameters;
  /// NSDMI, not a bare member: the aggregate initialisations of this struct
  /// predate the field and must keep compiling under
  /// -Wmissing-field-initializers.
  std::optional<std::string> vendor_code{};
};

template <>
inline constexpr auto dto_fields<GenericError> =
    std::make_tuple(field("error_code", &GenericError::error_code,
                          "SOVD error code. The literal `vendor-error` means the specific code is in `vendor_code`."),
                    field("message", &GenericError::message, "Human-readable cause. Not stable enough to match on."),
                    // No description: `SchemaWriter<GenericError>` below replaces this whole
                    // property, so anything written here is dead text that never reaches the
                    // document. The published prose lives with the replacement.
                    field("parameters", &GenericError::parameters),
                    field("vendor_code", &GenericError::vendor_code,
                          "The `x-medkit-*` code the gateway actually raised, present exactly when `error_code` is "
                          "`vendor-error`."));

// SchemaWriter specialization: declare the `parameters` keys a client is
// expected to branch on.
//
// The generic walk types `parameters` from its C++ member, `optional<json>`,
// which publishes `anyOf: [{}, {"type": "null"}]` - not even constrained to an
// object. Prose in the field description cannot be checked by anything and is
// the weakest tier we have, so the two recovery-bearing keys get real
// properties. `additionalProperties: true` keeps every other key legal, which
// matters because the set is open: 30-odd diagnostic keys across the handlers,
// and a plugin can add its own.
//
// Starts from the derived schema rather than hand-writing all four properties,
// so adding a field to `dto_fields<GenericError>` still reaches the document.
template <>
struct SchemaWriter<GenericError> {
  static nlohmann::json schema() {
    nlohmann::json schema = derived_object_schema<GenericError>();
    schema["properties"]["parameters"] = nlohmann::json{
        {"type", "object"},
        {"additionalProperties", true},
        {"description",
         "Cause-specific detail. The declared keys are the ones a client can act on; the rest "
         "(`entity_id`, `parameter` - the field at fault on a 400 - `details`, `collection`, "
         "`invalid_scope`, `supported_capabilities`, ...) are diagnostic."},
        {"properties",
         nlohmann::json{{"existing_lock_id",
                         {{"type", "string"},
                          {"description",
                           "The lock already held on the entity, returned with the 409 from "
                           "`POST /{entity_type}/{entity_id}/locks`. Pass it to "
                           "`DELETE /{entity_type}/{entity_id}/locks/{lock_id}` to break the lock, or retry the "
                           "acquire with `break_lock`. Note this 409 carries `error_code: invalid-request`, not "
                           "`lock-broken` - see `lock_id` below."}}},
                        {"lock_id",
                         {{"type", "string"},
                          {"description",
                           "The lock that blocked a guarded write, returned with the `lock-broken` 409 from a data / "
                           "operations / configurations / faults / logs / bulk-data write. A different 409 from "
                           "`existing_lock_id` above: this one says an existing lock stopped this request, that one "
                           "says an acquire collided."}}}}}};
    return schema;
  }
};
template <>
inline constexpr std::string_view dto_name<GenericError> = "GenericError";

}  // namespace dto
}  // namespace ros2_medkit_gateway
