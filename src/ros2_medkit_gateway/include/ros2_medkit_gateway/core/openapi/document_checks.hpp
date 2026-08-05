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

#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <string>

namespace ros2_medkit_gateway {
namespace openapi {

/// Names in `components/schemas` that no operation can reach, transitively,
/// through `components/responses` (or through any other component section a
/// `$ref` chain passes on the way).
///
/// A non-empty result means the document ships schemas a generated client will
/// never use: the generator emits a type for each one, and nothing in the API
/// surface ever produces or consumes it.
///
/// Deliberately a free function over the finished document rather than a rule
/// inside `RouteRegistry::validate_completeness()`. The registry sees routes,
/// not components - `components/schemas` comes from `dto::AllDtos` and
/// `components/responses` from `OpenApiSpecBuilder`, neither of which the
/// registry can read. Only the assembled document knows both halves.
std::set<std::string> unreachable_schemas(const nlohmann::json & document);

/// The entries of `pool` that a `$ref` chain rooted in `subtree` can reach,
/// resolving each hop through `pool` itself.
///
/// The inverse of the walk above, and it exists for the sub-documents: a
/// `<entity-path>/docs` document publishes a slice of the API surface, and the
/// schemas its operations name have to travel with it - a `$ref` into a
/// `components/schemas` entry the document does not carry is a reference no
/// client can resolve. Shipping the whole pool instead would put every DTO the
/// gateway knows on every entity page.
///
/// Only `components/schemas` references are followed. References into other
/// component sections - `#/components/responses/GenericError` and the
/// middleware-owned responses beside it - are emitted unconditionally by
/// `OpenApiSpecBuilder::build()`, so a sub-document carries them whether or not
/// an operation names one.
std::map<std::string, nlohmann::json> referenced_schemas(const nlohmann::json & subtree,
                                                         const std::map<std::string, nlohmann::json> & pool);

}  // namespace openapi
}  // namespace ros2_medkit_gateway
