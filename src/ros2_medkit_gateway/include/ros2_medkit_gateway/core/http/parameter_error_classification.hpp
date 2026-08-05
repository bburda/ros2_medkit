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
#include <vector>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// HTTP status + SOVD error code for a failed parameter operation.
struct ParameterErrorClassification {
  int status_code = 500;
  std::string error_code;
};

/// Map a failed `ParameterResult` to an HTTP status + SOVD error code from its
/// structured `error_code`. Every failure path on the shipped
/// `Ros2ParameterTransport` sets a structured code, so a failure that still
/// carries `ParameterErrorCode::NONE` is a gateway-side defect and maps to 500
/// internal-error - it is never guessed from the free-form message text.
ParameterErrorClassification classify_parameter_error(const ParameterResult & result);

/// Every HTTP status `classify_parameter_error` can produce, ascending and
/// deduplicated.
///
/// Computed by running the classifier over every `ParameterErrorCode`, not by
/// listing statuses by hand: the routes that surface parameter failures declare
/// their error set from this, so a new enumerator mapping to a new status
/// widens those declarations with no edit at the registrations. Hand-listing is
/// what let three 503-emitting sites go undeclared earlier in this work.
const std::vector<int> & parameter_error_statuses();

}  // namespace handlers
}  // namespace ros2_medkit_gateway
