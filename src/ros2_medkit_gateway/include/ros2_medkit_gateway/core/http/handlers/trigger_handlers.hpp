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

#include <memory>
#include <string>
#include <utility>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/dto/triggers.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// Result of parsing a SOVD resource URI for triggers (includes areas)
struct TriggerParsedResourceUri {
  std::string entity_type;  // "areas", "apps", "components", or "functions"
  std::string entity_id;
  std::string collection;     // "data", "faults", "x-medkit-metrics", etc.
  std::string resource_path;  // path within collection (may be empty)
};

/**
 * @brief HTTP handlers for trigger CRUD and SSE streaming.
 *
 * Implements SOVD trigger endpoints:
 * - POST /{entity}/triggers - create trigger
 * - GET /{entity}/triggers - list triggers
 * - GET /{entity}/triggers/{id} - get trigger
 * - PUT /{entity}/triggers/{id} - update trigger
 * - DELETE /{entity}/triggers/{id} - delete trigger
 * - GET /{entity}/triggers/{id}/events - SSE stream
 *
 * All 6 routes follow the PR-403 typed RouteRegistry convention:
 *
 *   http::Result<dto::TResponse> X(const http::TypedRequest & req [, dto::TBody body]);
 *
 * The SSE event-stream route uses the `reg.sse<>` escape hatch and returns a
 * `Result<SseStream>` factory; the framework drives the chunked content
 * provider. The forwarding-scope primitive (commit 17) is installed by the
 * framework so peer-forwarding still works for entities owned by a remote
 * gateway. CRUD POST returns `Created<Trigger>` so the 201 status lives in the
 * signature and the generated document cannot declare a status the handler
 * never emits.
 */
class TriggerHandlers {
 public:
  TriggerHandlers(HandlerContext & ctx, TriggerManager & trigger_mgr, std::shared_ptr<SSEClientTracker> client_tracker);

  /// POST /{entity}/triggers - create trigger.
  ///
  /// On success returns the new `Trigger` body as 201 Created.
  http::Result<http::Created<dto::Trigger>> post_trigger(const http::TypedRequest & req,
                                                         dto::TriggerCreateRequest body);

  /// GET /{entity}/triggers - list all triggers for entity.
  http::Result<dto::Collection<dto::Trigger>> get_triggers(const http::TypedRequest & req);

  /// GET /{entity}/triggers/{id} - get single trigger.
  http::Result<dto::Trigger> get_trigger(const http::TypedRequest & req);

  /// PUT /{entity}/triggers/{id} - update trigger.
  http::Result<dto::Trigger> put_trigger(const http::TypedRequest & req, dto::TriggerUpdateRequest body);

  /// DELETE /{entity}/triggers/{id} - delete trigger.
  http::Result<http::NoContent> del_trigger(const http::TypedRequest & req);

  /// GET /{entity}/triggers/{id}/events - SSE event stream.
  ///
  /// Returns a `SseStream` whose `next_event` callback the framework drives
  /// via cpp-httplib's chunked content provider. On validation failure the
  /// factory returns `tl::unexpected(ErrorInfo)` and the framework renders a
  /// SOVD GenericError.
  http::Result<http::SseStream> sse_trigger_events(const http::TypedRequest & req);

  /// Parse resource URI for triggers (includes areas in addition to apps/components/functions).
  static tl::expected<TriggerParsedResourceUri, std::string> parse_resource_uri(const std::string & resource);

 private:
  /// Build event_source URI from trigger info
  static std::string build_event_source(const TriggerInfo & info);

  /// Extract entity type string from request path
  static std::string extract_entity_type(const std::string & path);

  HandlerContext & ctx_;
  TriggerManager & trigger_mgr_;
  std::shared_ptr<SSEClientTracker> client_tracker_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
