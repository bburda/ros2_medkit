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

#include "ros2_medkit_gateway/http/handlers/planned_stop_handlers.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

using json = nlohmann::json;

namespace {

/// Map a transport failure onto an HTTP answer. Only a transport that got no
/// answer at all yields 503; a fault manager that answered and declined is
/// healthy and the request was at fault.
///
/// Which refusal it was comes from the service's structured outcome, never from
/// its message: the refusals map onto three different statuses, and reading
/// prose to pick one turns a rewording into an outage.
ErrorInfo transport_error(const PlannedStopResult & result, const std::string & what, json params = json::object()) {
  params["operation"] = what;
  if (result.failure == FaultFailure::Unavailable) {
    params["details"] = result.error_message;
    return make_error(503, ERR_SERVICE_UNAVAILABLE, "Fault manager unavailable", std::move(params));
  }
  const std::string message = result.error_message.empty() ? what : result.error_message;
  switch (result.refusal) {
    case PlannedStopRefusal::NotFound:
      return make_error(404, ERR_RESOURCE_NOT_FOUND, "Planned stop not found", std::move(params));
    case PlannedStopRefusal::AlreadyEnded:
      return make_error(400, ERR_X_MEDKIT_PLANNED_STOP_ENDED,
                        "Planned stop has already ended and cannot be ended again", std::move(params));
    case PlannedStopRefusal::DuplicateId:
    case PlannedStopRefusal::NotRetained:
      // 409, not 400: the request was well formed and the store could not take
      // it. Raise planned_stop.max_windows, or end a window, and retry.
      return make_error(409, ERR_PRECONDITION_NOT_FULFILLED, message, std::move(params));
    case PlannedStopRefusal::InvalidRequest:
    case PlannedStopRefusal::None:
    default:
      return make_error(400, ERR_INVALID_REQUEST, message, std::move(params));
  }
}

}  // namespace

dto::PlannedStop PlannedStopHandlers::to_dto(const faults::PlannedStopWindow & window) {
  dto::PlannedStop out;
  out.id = window.id;
  out.from = format_timestamp_ns(window.from_ns);
  out.to = format_timestamp_ns(window.to_ns);
  out.reason = window.reason;
  out.declared_by = window.declared_by;
  out.declared_at = format_timestamp_ns(window.declared_at_ns);
  out.ended_early = window.ended_early;
  out.cancelled = window.cancelled;
  return out;
}

std::string PlannedStopHandlers::resolve_declared_by(const http::TypedRequest & req,
                                                     const dto::PlannedStopCreateRequest & body) const {
  if (body.declared_by.has_value() && !body.declared_by->empty()) {
    return *body.declared_by;
  }

  auto * auth = ctx_.auth_manager();
  if (auth != nullptr && auth->is_enabled()) {
    if (auto header = req.header("Authorization")) {
      if (auto token = AuthMiddleware::extract_bearer_token(*header)) {
        const auto validation = auth->validate_token(*token);
        if (validation.valid && validation.claims.has_value() && !validation.claims->sub.empty()) {
          return validation.claims->sub;
        }
      }
    }
  }
  // Authentication off, or a token this route never had to see: the honest
  // record is that nobody signed the declaration.
  return "anonymous";
}

http::Result<std::pair<http::Created<dto::PlannedStop>, http::ResponseAttachments>>
PlannedStopHandlers::declare_stop(const http::TypedRequest & req, dto::PlannedStopCreateRequest body) {
  try {
    // An omitted `from` is filled in HERE, with this gateway's own clock, and an
    // explicit instant always goes on the wire. Sending zero and letting the
    // service read it as "now" made the epoch - a legal instant an operator can
    // ask for - indistinguishable from a field nobody set.
    int64_t from_ns = faults::floor_to_ms_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
    if (body.from.has_value() && !body.from->empty()) {
      const auto parsed = faults::parse_iso8601_utc_ns(*body.from);
      if (!parsed) {
        return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "from is not an ISO 8601 instant in UTC",
                                         json{{"parameter", "from"}, {"value", *body.from}}));
      }
      from_ns = *parsed;
      if (!faults::is_representable_instant(from_ns)) {
        return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                         "from must be after 1970-01-01T00:00:00Z and no later than "
                                         "2038-01-19T03:14:07Z",
                                         json{{"parameter", "from"}, {"value", *body.from}}));
      }
    }

    const auto to_ns = faults::parse_iso8601_utc_ns(body.to);
    if (!to_ns) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "to is not an ISO 8601 instant in UTC",
                                       json{{"parameter", "to"}, {"value", body.to}}));
    }
    if (!faults::is_representable_instant(*to_ns)) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                       "to must be after 1970-01-01T00:00:00Z and no later than "
                                       "2038-01-19T03:14:07Z",
                                       json{{"parameter", "to"}, {"value", body.to}}));
    }
    // Checked here as well as in the fault manager so a caller who spelled the
    // window wrong is told which field, rather than being handed the service's
    // prose.
    if (*to_ns <= from_ns) {
      return tl::unexpected(
          make_error(400, ERR_INVALID_PARAMETER, "to must be strictly after from",
                     json{{"parameter", "to"}, {"from", format_timestamp_ns(from_ns)}, {"to", body.to}}));
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->declare_planned_stop(from_ns, *to_ns, body.reason, resolve_declared_by(req, body));
    if (!result.success || result.stops.empty()) {
      return tl::unexpected(transport_error(result, "declare planned stop"));
    }

    auto stop = to_dto(result.stops.front());
    http::ResponseAttachments att;
    att.with_location(api_path("/x-medkit-planned-stops/" + stop.id));
    return std::make_pair(http::Created<dto::PlannedStop>{std::move(stop)}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to declare planned stop", json{{"details", e.what()}}));
  }
}

http::Result<dto::Collection<dto::PlannedStop>> PlannedStopHandlers::list_stops(const http::TypedRequest & req) {
  try {
    const auto q = req.query<dto::PlannedStopListQuery>();

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->list_planned_stops(q.active);
    if (!result.success) {
      return tl::unexpected(transport_error(result, "list planned stops"));
    }

    dto::Collection<dto::PlannedStop> out;
    out.items.reserve(result.stops.size());
    for (const auto & window : result.stops) {
      out.items.push_back(to_dto(window));
    }
    return out;
  } catch (const std::exception & e) {
    return tl::unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to list planned stops", json{{"details", e.what()}}));
  }
}

http::Result<dto::PlannedStop> PlannedStopHandlers::get_stop(const http::TypedRequest & req) {
  try {
    auto id = req.path_param("1");
    if (!id) {
      return tl::unexpected(id.error());
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->list_planned_stops(/*active_only=*/false);
    if (!result.success) {
      return tl::unexpected(transport_error(result, "get planned stop"));
    }

    const auto found =
        std::find_if(result.stops.begin(), result.stops.end(), [&id](const faults::PlannedStopWindow & window) {
          return window.id == *id;
        });
    if (found == result.stops.end()) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Planned stop not found", json{{"planned_stop_id", *id}}));
    }
    return to_dto(*found);
  } catch (const std::exception & e) {
    return tl::unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to read planned stop", json{{"details", e.what()}}));
  }
}

http::Result<dto::PlannedStop> PlannedStopHandlers::end_stop(const http::TypedRequest & req) {
  try {
    auto id = req.path_param("1");
    if (!id) {
      return tl::unexpected(id.error());
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->end_planned_stop(*id);
    if (result.success && !result.stops.empty()) {
      // Cut short, or cancelled outright when the window had not started yet.
      // Both are 200 with the window; `cancelled` says which happened, and a
      // cancelled window is gone, so a GET on this id now answers 404.
      return to_dto(result.stops.front());
    }
    return tl::unexpected(transport_error(result, "end planned stop", json{{"planned_stop_id", *id}}));
  } catch (const std::exception & e) {
    return tl::unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to end planned stop", json{{"details", e.what()}}));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
