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

#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {

class PluginManager;

namespace handlers {

/**
 * @brief Handlers for SOVD status/lifecycle endpoints.
 *
 * Handlers:
 * - GET /{entity}/status            - Read entity lifecycle status
 * - PUT /{entity}/status/{action}   - Request a lifecycle transition (async, 202)
 *
 * Routes are registered outside the 4-type entity loop (apps + components only).
 * Control is delegated to a LifecycleProvider when one is registered for the
 * entity; otherwise GET returns a default status from the entity cache and PUT
 * returns 501 Not Implemented.
 */
class LifecycleHandlers {
 public:
  /// Construct lifecycle handlers with shared context, optional plugin manager, and optional lifecycle reader.
  /// lifecycle_reader is nullptr by default so existing 2-argument call sites compile unchanged.
  LifecycleHandlers(HandlerContext & ctx, PluginManager * plugin_mgr,
                    std::shared_ptr<LifecycleStateReader> lifecycle_reader = nullptr);

  /// GET /{entity}/status - read lifecycle status.
  http::Result<dto::LifecycleStatusResponse> handle_get_status(const http::TypedRequest & req);

  /// PUT /{entity}/status/{action} - request a lifecycle transition.
  /// Returns 202 + Location on acceptance, or 501 when no provider is registered.
  http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
  handle_transition(const http::TypedRequest & req, std::string_view transition);

 private:
  HandlerContext & ctx_;
  PluginManager * plugin_mgr_;
  std::shared_ptr<LifecycleStateReader> lifecycle_reader_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
