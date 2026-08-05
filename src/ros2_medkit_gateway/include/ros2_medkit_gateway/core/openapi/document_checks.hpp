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

}  // namespace openapi
}  // namespace ros2_medkit_gateway
