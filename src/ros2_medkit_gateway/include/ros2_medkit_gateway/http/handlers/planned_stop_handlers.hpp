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
#include <utility>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/faults/planned_stop.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/planned_stops.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for planned-stop windows.
 *
 * A planned stop is a window of wall-clock time during which faults are
 * expected. These four routes are the operator's side of it:
 *
 * - POST   /x-medkit-planned-stops        - declare a window (201)
 * - GET    /x-medkit-planned-stops        - list windows, `?active=true` filters
 * - GET    /x-medkit-planned-stops/{id}   - one window
 * - DELETE /x-medkit-planned-stops/{id}   - end a window early (200 + the window)
 *
 * The windows themselves live in the fault manager, next to the faults they
 * describe; this class only translates between the REST shape (ISO 8601 strings)
 * and the transport's nanosecond counts. Deriving `expected` for a fault is the
 * fault-list and event-stream code's job, not this one's.
 *
 * DELETE answers 200 with the ended window rather than 204: `to` moved to the
 * moment of the request, and the caller needs to be told where.
 */
class PlannedStopHandlers {
 public:
  explicit PlannedStopHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  ~PlannedStopHandlers() = default;
  PlannedStopHandlers(const PlannedStopHandlers &) = delete;
  PlannedStopHandlers & operator=(const PlannedStopHandlers &) = delete;
  PlannedStopHandlers(PlannedStopHandlers &&) = delete;
  PlannedStopHandlers & operator=(PlannedStopHandlers &&) = delete;

  /// POST /x-medkit-planned-stops
  http::Result<std::pair<http::Created<dto::PlannedStop>, http::ResponseAttachments>>
  declare_stop(const http::TypedRequest & req, dto::PlannedStopCreateRequest body);

  /// GET /x-medkit-planned-stops
  http::Result<dto::Collection<dto::PlannedStop>> list_stops(const http::TypedRequest & req);

  /// GET /x-medkit-planned-stops/{planned_stop_id}
  http::Result<dto::PlannedStop> get_stop(const http::TypedRequest & req);

  /// DELETE /x-medkit-planned-stops/{planned_stop_id}
  http::Result<dto::PlannedStop> end_stop(const http::TypedRequest & req);

  /// Convert a stored window into the wire shape. Public because the OpenAPI
  /// example recorder and the tests build one without going through a route.
  static dto::PlannedStop to_dto(const faults::PlannedStopWindow & window);

 private:
  /// Who to record as the declarer: the authenticated client id when the
  /// request carries a valid token, otherwise the literal `anonymous`. A
  /// caller-supplied `declared_by` wins - a maintenance system declaring on
  /// somebody's behalf is a real case, and the token identifies the machine.
  std::string resolve_declared_by(const http::TypedRequest & req, const dto::PlannedStopCreateRequest & body) const;

  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
