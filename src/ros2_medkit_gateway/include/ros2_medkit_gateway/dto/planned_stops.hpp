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

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// PlannedStop - one declared window, as every planned-stop route emits it.
//
// The window's bounds are `from` and `to` here, where the ROS message calls them
// starts_at / ends_at: a message field named `from` is a Python keyword and
// rosidl cannot generate a binding for it, but nothing stops a JSON key.
//
// Wire keys: id, from, to, reason, declared_by, declared_at, ended_early
// =============================================================================
struct PlannedStop {
  std::string id;
  std::string from;
  std::string to;
  std::string reason;
  std::string declared_by;
  std::string declared_at;
  bool ended_early{false};
  bool cancelled{false};
};

template <>
inline constexpr auto dto_fields<PlannedStop> = std::make_tuple(
    field("id", &PlannedStop::id,
          "Identifier assigned by the fault manager. Unique within one fault manager, not across a fleet."),
    field("from", &PlannedStop::from,
          "When the window opens, ISO 8601 in UTC, to the millisecond. A fault whose current cycle started at "
          "or after this instant and at or before `to` is reported as expected, compared at millisecond "
          "resolution."),
    field("to", &PlannedStop::to,
          "When the window closes, ISO 8601 in UTC, to the millisecond. Always strictly after `from`."),
    field("reason", &PlannedStop::reason, "Why the plant is stopping, as the operator wrote it."),
    field("declared_by", &PlannedStop::declared_by,
          "The authenticated client that declared the window, or `anonymous` when the declaration arrived "
          "without authentication."),
    field("declared_at", &PlannedStop::declared_at,
          "When the declaration was recorded, ISO 8601 in UTC. Later than `to` for a window declared after the "
          "stop it describes, which is allowed."),
    field("ended_early", &PlannedStop::ended_early,
          "True when an operator cut the window short, which moved `to` to the moment of that request."),
    field("cancelled", &PlannedStop::cancelled,
          "True on the window returned by a DELETE that arrived before the window had STARTED: it marked "
          "nothing, so it was removed rather than shortened, and `ended_early` stays false. Never true on a "
          "window a list or a GET returns - a cancelled window is gone."));

template <>
inline constexpr std::string_view dto_name<PlannedStop> = "PlannedStop";

template <>
struct dto_sample<PlannedStop> {
  static PlannedStop make() {
    PlannedStop obj;
    obj.id = "1788716982473405798-1";
    obj.from = "2026-09-06T18:00:00.000Z";
    obj.to = "2026-09-06T22:00:00.000Z";
    obj.reason = "line changeover";
    obj.declared_by = "shift_lead";
    obj.declared_at = "2026-09-06T17:58:12.004Z";
    return obj;
  }
};

// =============================================================================
// PlannedStopCreateRequest - POST /x-medkit-planned-stops request body.
// =============================================================================
struct PlannedStopCreateRequest {
  std::optional<std::string> from;
  std::string to;
  std::string reason;
  std::optional<std::string> declared_by;
};

template <>
inline constexpr auto dto_fields<PlannedStopCreateRequest> = std::make_tuple(
    field("from", &PlannedStopCreateRequest::from,
          "When the window opens, ISO 8601 in UTC (`Z` or `+00:00`; a real offset is rejected). Omit it for a "
          "stop that starts now - the gateway fills in its own clock. A window wholly in the past is accepted "
          "and marks the faults it covers: a stop is a fact about the plant, not about when someone typed it "
          "in. The instant must be after the Unix epoch and no later than 2038-01-19T03:14:07Z, and it is "
          "recorded to the millisecond."),
    field("to", &PlannedStopCreateRequest::to,
          "When the window closes, ISO 8601 in UTC. Must be strictly after `from`; equal instants are rejected "
          "with 400, as is a `to` before `from`. There is no maximum duration, but the instant must lie in the "
          "same range as `from`, and it is recorded to the millisecond."),
    field("reason", &PlannedStopCreateRequest::reason,
          "Why the plant is stopping. Carried verbatim on every fault the window marks, so write what an "
          "engineer reading the fault next month needs.",
          FieldConstraints{{}, {}, /*max_length=*/512U, {}, {}}),
    field("declared_by", &PlannedStopCreateRequest::declared_by,
          "Who is declaring the stop. Omit it and the gateway fills in the authenticated client id, or "
          "`anonymous` when authentication is off.",
          FieldConstraints{{}, {}, /*max_length=*/256U, {}, {}}));

template <>
inline constexpr std::string_view dto_name<PlannedStopCreateRequest> = "PlannedStopCreateRequest";

template <>
struct dto_sample<PlannedStopCreateRequest> {
  static PlannedStopCreateRequest make() {
    PlannedStopCreateRequest obj;
    obj.to = "2026-09-06T22:00:00Z";
    obj.reason = "line changeover";
    return obj;
  }
};

// =============================================================================
// PlannedStopListQuery - query parameters for GET /x-medkit-planned-stops.
// =============================================================================
struct PlannedStopListQuery {
  bool active = false;
};

template <>
inline constexpr auto dto_fields<PlannedStopListQuery> = std::make_tuple(
    field("active", &PlannedStopListQuery::active, Presence::kOptional,
          "Return only the windows containing this instant. Omitted or false returns every declared window, "
          "ended ones included - they are the reason older faults read as expected."));

// =============================================================================
// Collection<PlannedStop> - named "PlannedStopList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<PlannedStop>> = "PlannedStopList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
