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

#include <httplib.h>

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/models/error_info.hpp"

namespace ros2_medkit_gateway {

// Forward declaration so we can friend HandlerContext without dragging the
// full handler_context.hpp into this header (it transitively pulls in rclcpp).
namespace handlers {
class HandlerContext;
class DocsHandlers;
class SSEFaultHandler;
}  // namespace handlers

// Forward declaration so we can friend openapi::RouteRegistry without dragging
// the full route_registry.hpp into this header (it transitively pulls in
// httplib + nlohmann/json which we already include here, but we also want to
// keep the friend list discoverable in this single source-of-truth file).
namespace openapi {
class RouteRegistry;
}  // namespace openapi

// Forward declaration so we can friend PluginResponse without including the
// plugin_http_types.hpp header (which would create a circular include because
// the plugin header pulls handler_context.hpp). PluginResponse lives in the
// top-level ros2_medkit_gateway namespace, not under ros2_medkit_gateway::plugins.
class PluginResponse;

namespace http {
namespace detail {

/**
 * @brief Access token gating the framework response-writing primitives.
 *
 * The primitives in this header are the only places that may call
 * `httplib::Response::set_content()` for JSON responses. To keep handler
 * code from reaching them directly, every primitive takes an instance of
 * this token as its first parameter, and the token is only constructible
 * by classes listed as `friend` below.
 *
 * Today friends are `openapi::RouteRegistry` (the typed router wrapper that
 * is the primary writer of JSON responses for migrated handlers),
 * `PluginResponse` (plugin shim that lets plugins emit responses without
 * going through HandlerContext), and the two remaining legacy raw-route
 * handlers that have not yet migrated to the typed router and so still
 * write `httplib::Response` directly: `DocsHandlers` (the
 * `<entity-path>/docs` scoped capability description, whose `(.+)/docs`
 * prefix is a whole entity or resource path rather than a `{param}` segment
 * the typed router can name; the plain `/docs` next to it is typed, and the
 * Swagger UI routes serve non-JSON bodies) and `SSEFaultHandler` (the
 * legacy `handle_stream` entry kept for the in-process unit test fixture).
 *
 * Handler code cannot default-construct the token, so cannot call the
 * primitives directly - it must go through the typed router.
 */
class FrameworkOrPluginAccess {
 private:
  // User-provided (not =default) so the class is NOT an aggregate in C++17.
  // If we used `= default`, `FrameworkOrPluginAccess{}` would fall through
  // to aggregate-initialization and bypass the private access check - which
  // would defeat the friend gate entirely.
  FrameworkOrPluginAccess() {
  }

  // Centrally controlled friend list - keep narrow. New entries require
  // an explicit justification (typically: another legacy raw-route handler
  // that has not yet migrated to the typed router).
  friend class ros2_medkit_gateway::openapi::RouteRegistry;     // typed router wrapper
  friend class ros2_medkit_gateway::PluginResponse;             // plugin response shim
  friend class ros2_medkit_gateway::handlers::DocsHandlers;     // legacy /docs raw routes
  friend class ros2_medkit_gateway::handlers::SSEFaultHandler;  // legacy in-process SSE test entry

  // Test-only access. The bridge is defined in test/test_primitives.cpp and
  // exists solely to construct tokens for direct primitive invocation in
  // unit tests. Production code must not depend on it.
  friend struct PrimitivesAccessForTesting;
};

/**
 * @brief Test-only bridge granting access to FrameworkOrPluginAccess.
 *
 * Defined in test/test_primitives.cpp. Production code must not use this -
 * the only legitimate consumers are the primitives unit tests, which need
 * to invoke the primitives directly without standing up a HandlerContext.
 */
struct PrimitivesAccessForTesting;

/**
 * @brief Write a JSON body with the given HTTP status.
 *
 * Indented with 2 spaces to match the gateway's existing send_json
 * convention. The caller is responsible for ensuring `body` is
 * well-formed (typically DTO-derived JSON).
 *
 * If `status` is `kKeepCurrentStatus` (sentinel = 0) the caller-provided
 * `res.status` is left untouched. This lets the remaining raw-route
 * callers (PluginResponse, DocsHandlers, SSEFaultHandler) pre-set the
 * status (e.g. 201 Created) before calling the writer. The typed router
 * always passes an explicit status.
 *
 * @param token Framework access token (constructible only by friends).
 * @param res HTTP response to mutate.
 * @param body JSON payload to serialize.
 * @param status HTTP status code; pass `kKeepCurrentStatus` to leave
 *               `res.status` unchanged. Defaults to 200.
 */
constexpr int kKeepCurrentStatus = 0;

void write_json_body(FrameworkOrPluginAccess token, httplib::Response & res, const nlohmann::json & body,
                     int status = 200);

/**
 * @brief Write an already-serialized JSON document.
 *
 * Same status sentinel and same `application/json` content type as
 * `write_json_body`; the difference is only that the caller has the JSON
 * text rather than a DOM. It exists for the `/docs` routes, which cache
 * capability documents serialized so that serving one does not have to
 * copy a parsed DOM (see `openapi::CapabilityGenerator`).
 *
 * `body` must be what `nlohmann::json::dump(2)` produces for the document,
 * which is what `write_json_body` would have written - that equality is what
 * lets the two writers be used interchangeably on the same route. Nothing
 * here validates it; the caller owns that.
 *
 * @param token Framework access token (constructible only by friends).
 * @param res HTTP response to mutate.
 * @param body Serialized JSON document.
 * @param status HTTP status code; pass `kKeepCurrentStatus` to leave
 *               `res.status` unchanged. Defaults to 200.
 */
void write_json_text(FrameworkOrPluginAccess token, httplib::Response & res, const std::string & body,
                     int status = 200);

/**
 * @brief Write a SOVD GenericError response.
 *
 * Body shape:
 * @code
 *   { "error_code": "<code>", "message": "<message>", "parameters": { ... } }
 * @endcode
 * The `parameters` key is omitted when `err.params` is null or empty. For
 * vendor-specific error codes (`x-medkit-*`), the wire `error_code` is
 * rewritten to `vendor-error` and the original code is reported as
 * `vendor_code`.
 *
 * @param token Framework access token (constructible only by friends).
 * @param res HTTP response to mutate.
 * @param err Transport-neutral error descriptor. `err.http_status` is clamped
 *            into the SOVD range 400-599.
 */
void write_generic_error(FrameworkOrPluginAccess token, httplib::Response & res, const ErrorInfo & err);

/**
 * @brief Write an OAuth 2.0 RFC 6749 error response.
 *
 * Body shape (per RFC 6749 §5.2):
 * @code
 *   { "error": "<code>", "error_description": "<message>" }
 * @endcode
 * Used only by the auth endpoints (`/auth/token`, `/auth/revoke`) which by
 * spec must speak the OAuth2 error shape, not the SOVD GenericError shape.
 * The `error` field carries the snake_case OAuth2 error code, not the SOVD
 * `error_code` key. There is no `parameters` wrapper.
 *
 * @param token Framework access token (constructible only by friends).
 * @param res HTTP response to mutate.
 * @param err Transport-neutral error descriptor. `err.http_status` is clamped
 *            into the SOVD range 400-599.
 */
void write_oauth2_error(FrameworkOrPluginAccess token, httplib::Response & res, const ErrorInfo & err);

}  // namespace detail
}  // namespace http
}  // namespace ros2_medkit_gateway
