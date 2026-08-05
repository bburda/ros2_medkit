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
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Error codes for parameter operations.
enum class ParameterErrorCode {
  NONE = 0,             ///< No error (success)
  NOT_FOUND,            ///< Parameter does not exist
  READ_ONLY,            ///< Parameter is read-only and cannot be modified
  SERVICE_UNAVAILABLE,  ///< Parameter service not available (node unreachable)
  TIMEOUT,              ///< Operation timed out
  TYPE_MISMATCH,        ///< Value type doesn't match parameter type
  INVALID_VALUE,        ///< Invalid value for parameter
  NO_DEFAULTS_CACHED,   ///< No default values cached for reset operation
  SHUT_DOWN,            ///< ConfigurationManager has been shut down
  INTERNAL_ERROR,       ///< Internal/unexpected error
  /// Not an error code - the number of them. Keep last, and add new
  /// enumerators above it.
  ///
  /// `parameter_error_classification.cpp` runs the classifier over every
  /// enumerator to derive the statuses four route registrations declare, and
  /// it does that from a hand-written array. `-Werror=switch-enum` makes the
  /// compiler demand a `case` for a new enumerator, but nothing made it demand
  /// an array entry: adding the case alone left the array short and the
  /// registrations quietly declaring one status too few. This sentinel is what
  /// a `static_assert` on the array's size can compare against, which turns
  /// that omission into a build failure.
  COUNT
};

/// Result of a parameter operation.
struct ParameterResult {
  bool success;
  json data;
  std::string error_message;
  ParameterErrorCode error_code{ParameterErrorCode::NONE};  ///< Structured error code for programmatic handling
};

}  // namespace ros2_medkit_gateway
