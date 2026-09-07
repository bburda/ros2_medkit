// Copyright 2025-2026 bburda
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

#include <functional>
#include <memory>
#include <utility>

#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"

#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace openapi {
class RouteRegistry;
}  // namespace openapi

namespace handlers {

/**
 * @brief Health and system info endpoint handlers
 *
 * Handles:
 * - GET /health        -> dto::Health
 * - GET /              -> dto::RootOverview
 * - GET /version-info  -> dto::VersionInfo
 *
 * Migrated to the typed RouteRegistry API as part of PR-403 commit 16.
 * Handler signatures follow the project-wide convention established by
 * this commit:
 *
 *   http::Result<dto::TResponse> get_X(const http::TypedRequest & req);
 *
 * The framework owns the cpp-httplib response object - handlers never
 * touch it directly. Errors are returned as `tl::unexpected(ErrorInfo)`
 * and the framework renders them via the SOVD GenericError schema.
 */
class HealthHandlers {
 public:
  /// @param sse_tracker Shared SSE client counter, so /health can report how
  /// much of `sse.max_clients` is in use. Optional: a gateway built without
  /// one simply omits the field.
  explicit HealthHandlers(HandlerContext & ctx, const openapi::RouteRegistry * route_registry = nullptr,
                          std::shared_ptr<const SSEClientTracker> sse_tracker = nullptr)
    : ctx_(ctx), route_registry_(route_registry), sse_tracker_(std::move(sse_tracker)) {
  }

  /// Where /health reads the fault stream's received-event counter from.
  ///
  /// Set after construction because the SSE handler is built later than this
  /// one. It counts events the gateway ACCEPTED from the fault manager, which
  /// is the only thing that distinguishes "no faults are happening" from "the
  /// thread that takes them is stuck".
  void set_sse_event_counter(std::function<uint64_t()> counter) {
    sse_events_received_ = std::move(counter);
  }

  /// GET /health - Health check endpoint
  http::Result<dto::Health> get_health(const http::TypedRequest & req);

  /// GET / - Root endpoint with server capabilities
  http::Result<dto::RootOverview> get_root(const http::TypedRequest & req);

  /// GET /version-info - Version information
  http::Result<dto::VersionInfo> get_version_info(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
  const openapi::RouteRegistry * route_registry_{nullptr};
  std::shared_ptr<const SSEClientTracker> sse_tracker_;
  std::function<uint64_t()> sse_events_received_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
