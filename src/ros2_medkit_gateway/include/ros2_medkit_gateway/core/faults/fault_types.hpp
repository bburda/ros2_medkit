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

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Which layer failed a fault-management call. Meaningful only when the
/// outcome's `success` is false.
///
/// The distinction is structural, not textual. A transport knows for a fact
/// whether it obtained an answer from the fault manager, and that is the only
/// thing that separates a server-side failure from a client-side one: a fault
/// manager that answered and declined is healthy, and the request is what was
/// at fault, however the refusal happens to be worded. Reading the words
/// instead cannot make that call - the message is prose the fault manager is
/// free to change, and any wording a matcher did not anticipate is
/// indistinguishable from an outage.
enum class FaultFailure : uint8_t {
  /// The fault manager answered and declined: no such fault, a code it will
  /// not accept, or a fault outside the requested scope. A client error.
  Declined,
  /// No answer was obtained at all - the service was missing, uninitialised,
  /// or timed out. The only genuinely server-side case.
  Unavailable,
};

/// Outcome of a fault-management operation that returns JSON. `data` carries
/// the response body the handler will serve on success; remains empty on
/// errors.
///
/// `failure` defaults to `Declined` so that a producer which reports a failure
/// without classifying it cannot manufacture a server error: 503 has to be
/// asked for, and only a transport that failed to get an answer asks.
struct FaultResult {
  bool success;
  json data;
  std::string error_message;
  FaultFailure failure = FaultFailure::Declined;
};

/// Neutral outcome of `get_fault_with_env`. `data` carries
/// `{ "fault": {...}, "environment_data": {...} }` already converted to JSON
/// by the transport. The handler post-processes rosbag snapshots to add the
/// per-request `bulk_data_uri` (which depends on entity_path) and the
/// freeze_frame snapshots to extract their primary value.
struct FaultWithEnvJsonResult {
  bool success;
  std::string error_message;
  json data;
  FaultFailure failure = FaultFailure::Declined;
};

}  // namespace ros2_medkit_gateway
