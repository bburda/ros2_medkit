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

#include <utility>

#include "ros2_medkit_gateway/core/managers/update_manager.hpp"
#include "ros2_medkit_gateway/dto/updates.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for software update endpoints (/updates).
 *
 * All endpoints are server-level (no entity path). Without a loaded
 * backend plugin, every handler short-circuits with 501 Not Implemented.
 *
 * PR-403 commit 22: 8 routes migrated to typed `Result<TResponse>(TypedRequest
 * [, TBody])` signatures. The POST/PUT routes that emit 201/202 + Location use
 * the attachments variant so the framework still owns body serialization while
 * the handler controls status code and headers.
 */
class UpdateHandlers {
 public:
  UpdateHandlers(HandlerContext & ctx, UpdateManager * update_manager);

  /// GET /updates - returns a Collection-shaped list of registered update IDs.
  http::Result<dto::UpdateList> get_updates(const http::TypedRequest & req);

  /// GET /updates/{update_id} - returns the opaque metadata object the plugin
  /// stored at registration time.
  http::Result<dto::UpdateDetail> get_update(const http::TypedRequest & req);

  /// POST /updates - register a new update descriptor. On success returns the
  /// `UpdateRegisterResponse` body as 201 Created plus a
  /// `Location: /api/v1/updates/<id>` header.
  http::Result<std::pair<http::Created<dto::UpdateRegisterResponse>, http::ResponseAttachments>>
  post_update(const http::TypedRequest & req, dto::UpdateRegisterRequest body);

  /// DELETE /updates/{update_id} - 204 No Content on success.
  http::Result<http::NoContent> del_update(const http::TypedRequest & req);

  /// PUT /updates/{update_id}/prepare - 202 Accepted + `Location: .../status`
  /// header, kicks the background prepare task.
  http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
  put_prepare(const http::TypedRequest & req);

  /// PUT /updates/{update_id}/execute - 202 Accepted + `Location: .../status`
  /// header, kicks the background execute task.
  http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
  put_execute(const http::TypedRequest & req);

  /// PUT /updates/{update_id}/automated - 202 Accepted + `Location: .../status`
  /// header, kicks the background prepare+execute task.
  http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
  put_automated(const http::TypedRequest & req);

  /// GET /updates/{update_id}/status - returns the current async-task state.
  http::Result<dto::UpdateStatus> get_status(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
  UpdateManager * update_mgr_;

  /// Returns an ErrorInfo carrying the 501-not-implemented response when no
  /// backend is loaded. Empty optional means the backend is ready.
  std::optional<ErrorInfo> check_backend() const;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
