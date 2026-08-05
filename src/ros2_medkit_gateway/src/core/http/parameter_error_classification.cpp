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

#include "ros2_medkit_gateway/core/http/parameter_error_classification.hpp"

#include <algorithm>
#include <array>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Map a structured `ParameterErrorCode` to an HTTP status + SOVD error code.
ParameterErrorClassification classify_error_code(ParameterErrorCode error_code) {
  ParameterErrorClassification result;

  switch (error_code) {
    case ParameterErrorCode::NOT_FOUND:
    case ParameterErrorCode::NO_DEFAULTS_CACHED:
      result.status_code = 404;
      result.error_code = ERR_RESOURCE_NOT_FOUND;
      break;
    case ParameterErrorCode::READ_ONLY:
      result.status_code = 403;
      result.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
      break;
    case ParameterErrorCode::SERVICE_UNAVAILABLE:
    case ParameterErrorCode::TIMEOUT:
      result.status_code = 503;
      result.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
      break;
    case ParameterErrorCode::TYPE_MISMATCH:
    case ParameterErrorCode::INVALID_VALUE:
      result.status_code = 400;
      result.error_code = ERR_INVALID_PARAMETER;
      break;
    case ParameterErrorCode::NONE:
      // A failed ParameterResult that still carries NONE cannot occur on the
      // shipped transport (every failure path sets a structured code), so a
      // NONE failure is a gateway-side defect: surface it as 500 internal-error.
      // The removed legacy string-matching fallback was unreachable, and would
      // have misrouted messages it did not recognise (e.g. "did not respond",
      // "not currently set") to 400 invalid-request had it ever run.
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
    case ParameterErrorCode::SHUT_DOWN:
    case ParameterErrorCode::INTERNAL_ERROR:
    default:
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
  }

  return result;
}

/// Every `ParameterErrorCode`, so `parameter_error_statuses()` can run the
/// classifier over the whole enum instead of trusting a hand-written status
/// list.
constexpr std::array<ParameterErrorCode, 10> kAllParameterErrorCodes{
    ParameterErrorCode::NONE,          ParameterErrorCode::NOT_FOUND,
    ParameterErrorCode::READ_ONLY,     ParameterErrorCode::SERVICE_UNAVAILABLE,
    ParameterErrorCode::TIMEOUT,       ParameterErrorCode::TYPE_MISMATCH,
    ParameterErrorCode::INVALID_VALUE, ParameterErrorCode::NO_DEFAULTS_CACHED,
    ParameterErrorCode::SHUT_DOWN,     ParameterErrorCode::INTERNAL_ERROR};

/// Compile-time guard on the array above. The switch has a case per enumerator
/// and deliberately no `default`, so `-Werror=switch-enum` turns "somebody
/// added a ParameterErrorCode" into a build failure here - three lines from the
/// array that then has to list it - rather than into a status the routes
/// quietly stop declaring.
constexpr bool is_known_parameter_error_code(ParameterErrorCode code) {
  switch (code) {
    case ParameterErrorCode::NONE:
    case ParameterErrorCode::NOT_FOUND:
    case ParameterErrorCode::READ_ONLY:
    case ParameterErrorCode::SERVICE_UNAVAILABLE:
    case ParameterErrorCode::TIMEOUT:
    case ParameterErrorCode::TYPE_MISMATCH:
    case ParameterErrorCode::INVALID_VALUE:
    case ParameterErrorCode::NO_DEFAULTS_CACHED:
    case ParameterErrorCode::SHUT_DOWN:
    case ParameterErrorCode::INTERNAL_ERROR:
      return true;
  }
  return false;
}

static_assert(is_known_parameter_error_code(ParameterErrorCode::NONE), "guard must be constexpr-evaluable");

}  // namespace

ParameterErrorClassification classify_parameter_error(const ParameterResult & result) {
  return classify_error_code(result.error_code);
}

const std::vector<int> & parameter_error_statuses() {
  static const std::vector<int> statuses = [] {
    std::vector<int> out;
    for (ParameterErrorCode code : kAllParameterErrorCodes) {
      out.push_back(classify_error_code(code).status_code);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }();
  return statuses;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
