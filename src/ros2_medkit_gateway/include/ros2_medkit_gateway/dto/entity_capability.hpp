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

#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// -----------------------------------------------------------------------------
// EntityCapability - one element of an entity's "capabilities" array.
//
// Built by `CapabilityBuilder::build_capabilities`, then extended in place by
// `append_plugin_capabilities`. Both write the same two keys, which is why the
// array is typed rather than free-form: a client reads `name` to decide what
// the entity supports and follows `href` to reach it.
//
// It lives in its own header because both the entity detail DTOs and the
// `x-medkit` payloads that carry a copy of the same array need it, and
// `entities.hpp` already includes `x_medkit.hpp`.
//
// Wire keys: name, href
// -----------------------------------------------------------------------------
struct EntityCapability {
  std::string name;  ///< Collection or relationship name, e.g. "data", "belongs-to"
  std::string href;  ///< Absolute path, API prefix included ("/api/v1/...")
};

template <>
inline constexpr auto dto_fields<EntityCapability> =
    std::make_tuple(field("name", &EntityCapability::name, "Collection or relationship this entity supports."),
                    field("href", &EntityCapability::href, "Absolute path of the collection, API prefix included."));

template <>
inline constexpr std::string_view dto_name<EntityCapability> = "EntityCapability";

}  // namespace dto
}  // namespace ros2_medkit_gateway
