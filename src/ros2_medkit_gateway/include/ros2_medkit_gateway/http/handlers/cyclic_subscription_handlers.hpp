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

#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"
#include "ros2_medkit_gateway/dto/cyclic_subscriptions.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// Result of parsing a SOVD resource URI for subscription
struct ParsedResourceUri {
  std::string entity_type;  // "apps" or "components"
  std::string entity_id;
  std::string collection;     // "data", "faults", "x-medkit-metrics", etc.
  std::string resource_path;  // path within collection (may be empty)
};

/**
 * @brief HTTP handlers for cyclic subscription CRUD and SSE streaming.
 *
 * Implements SOVD cyclic subscription endpoints:
 * - POST /{entity}/cyclic-subscriptions - create subscription
 * - GET /{entity}/cyclic-subscriptions - list subscriptions
 * - GET /{entity}/cyclic-subscriptions/{id} - get subscription
 * - PUT /{entity}/cyclic-subscriptions/{id} - update subscription
 * - DELETE /{entity}/cyclic-subscriptions/{id} - delete subscription
 * - GET /{entity}/cyclic-subscriptions/{id}/events - SSE stream
 *
 * All 6 routes follow the PR-403 typed RouteRegistry convention:
 *
 *   http::Result<dto::TResponse> X(const http::TypedRequest & req [, dto::TBody body]);
 *
 * The SSE event-stream route uses the `reg.sse<>` escape hatch and returns a
 * `Result<SseStream>` factory; the framework drives the chunked content
 * provider. The transport's `make_sse_stream` builds the `next_event` closure.
 * CRUD POST returns `Created<CyclicSubscription>` so the 201 status lives in
 * the signature and the generated document cannot declare a status the handler
 * never emits.
 */
class CyclicSubscriptionHandlers {
 public:
  CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr,
                             ResourceSamplerRegistry & sampler_registry, TransportRegistry & transport_registry,
                             int max_duration_sec);

  /// POST /{entity}/cyclic-subscriptions - create subscription.
  ///
  /// On success returns the new `CyclicSubscription` body as 201 Created.
  http::Result<http::Created<dto::CyclicSubscription>> post_subscription(const http::TypedRequest & req,
                                                                         dto::CyclicSubscriptionCreateRequest body);

  /// GET /{entity}/cyclic-subscriptions - list all subscriptions for entity.
  http::Result<dto::Collection<dto::CyclicSubscription>> get_subscriptions(const http::TypedRequest & req);

  /// GET /{entity}/cyclic-subscriptions/{id} - get single subscription.
  http::Result<dto::CyclicSubscription> get_subscription(const http::TypedRequest & req);

  /// PUT /{entity}/cyclic-subscriptions/{id} - update subscription.
  http::Result<dto::CyclicSubscription> put_subscription(const http::TypedRequest & req,
                                                         dto::CyclicSubscriptionUpdateRequest body);

  /// DELETE /{entity}/cyclic-subscriptions/{id} - delete subscription.
  http::Result<http::NoContent> del_subscription(const http::TypedRequest & req);

  /// GET /{entity}/cyclic-subscriptions/{id}/events - SSE event stream.
  ///
  /// Returns a `SseStream` whose `next_event` callback the framework drives via
  /// cpp-httplib's chunked content provider. On validation failure the factory
  /// returns `tl::unexpected(ErrorInfo)` and the framework renders a SOVD
  /// GenericError.
  http::Result<http::SseStream> sse_subscription_events(const http::TypedRequest & req);

  /// Parse resource URI to extract entity type, entity id, collection, and resource path.
  static tl::expected<ParsedResourceUri, std::string> parse_resource_uri(const std::string & resource);

  /// Refuse a resource path the collection's sampler cannot act on.
  ///
  /// A sampler registered without `honours_resource_path` answers with its
  /// whole collection on every tick, so a URI naming a single item of that
  /// collection cannot be served as written. The refusal keeps a narrow
  /// request from being answered with a wide stream.
  static tl::expected<void, ErrorInfo> validate_resource_path_support(const ResourceSamplerRegistry & registry,
                                                                      const ParsedResourceUri & parsed,
                                                                      const std::string & resource);

 private:
  /// Build event_source URI from subscription info
  static std::string build_event_source(const CyclicSubscriptionInfo & info);

  /// Extract entity type string ("apps" or "components") from request path
  static std::string extract_entity_type(const std::string & path);

  HandlerContext & ctx_;
  SubscriptionManager & sub_mgr_;
  ResourceSamplerRegistry & sampler_registry_;
  TransportRegistry & transport_registry_;
  int max_duration_sec_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
