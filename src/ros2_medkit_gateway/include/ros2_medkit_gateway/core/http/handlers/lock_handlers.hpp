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

#include <chrono>
#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/dto/locks.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for SOVD entity locking REST API endpoints.
 *
 * Provides handlers for:
 * - POST   /{entity_type}/{entity_id}/locks                - Acquire lock
 * - GET    /{entity_type}/{entity_id}/locks                - List locks
 * - GET    /{entity_type}/{entity_id}/locks/{lock_id}      - Get lock
 * - PUT    /{entity_type}/{entity_id}/locks/{lock_id}      - Extend lock
 * - DELETE /{entity_type}/{entity_id}/locks/{lock_id}      - Release lock
 *
 * Locking is supported for components and apps only (per SOVD spec).
 *
 * All 5 routes follow the PR-403 typed RouteRegistry convention:
 *
 *   http::Result<dto::TResponse> X(const http::TypedRequest & req [, dto::TBody body]);
 *
 * The framework owns the cpp-httplib response object - handlers never touch
 * it. Errors are returned as `tl::unexpected(ErrorInfo)` and the framework
 * renders them via the SOVD GenericError schema. The acquire handler uses
 * the attachments variant so it can return 201 + `Location: <request-path>/<lock-id>`
 * without re-introducing a `httplib::Response &` parameter.
 */
class LockHandlers {
 public:
  /**
   * @brief Construct lock handlers with shared context and lock manager.
   * @param ctx The shared handler context
   * @param lock_manager Pointer to the LockManager (may be nullptr if locking disabled)
   */
  LockHandlers(HandlerContext & ctx, LockManager * lock_manager);

  /**
   * @brief POST /{entity_type}/{entity_id}/locks - acquire a lock.
   *
   * Request body: `AcquireLockRequest` (validated at framework level).
   * Requires X-Client-Id header.
   * On success returns the new `Lock` body as 201 Created plus a
   * `Location: <request-path>/<lock-id>` header.
   */
  http::Result<std::pair<http::Created<dto::Lock>, http::ResponseAttachments>> post_lock(const http::TypedRequest & req,
                                                                                         dto::AcquireLockRequest body);

  /**
   * @brief GET /{entity_type}/{entity_id}/locks - list locks on entity.
   *
   * X-Client-Id header is optional (used to determine the `owned` field).
   * Returns 200 with `Collection<Lock>` (single-item or empty).
   */
  http::Result<dto::Collection<dto::Lock>> get_locks(const http::TypedRequest & req);

  /**
   * @brief GET /{entity_type}/{entity_id}/locks/{lock_id} - get lock details.
   *
   * X-Client-Id header is optional (used to determine the `owned` field).
   * Returns 200 with `Lock`, or 404 if not found.
   */
  http::Result<dto::Lock> get_lock(const http::TypedRequest & req);

  /**
   * @brief PUT /{entity_type}/{entity_id}/locks/{lock_id} - extend lock.
   *
   * Request body: `ExtendLockRequest`.
   * Requires X-Client-Id header.
   * Returns 204 (NoContent) on success, 403 if not owner, 404 if not found.
   */
  http::Result<http::NoContent> put_lock(const http::TypedRequest & req, dto::ExtendLockRequest body);

  /**
   * @brief DELETE /{entity_type}/{entity_id}/locks/{lock_id} - release lock.
   *
   * Requires X-Client-Id header.
   * Returns 204 (NoContent) on success, 403 if not owner, 404 if not found.
   */
  http::Result<http::NoContent> del_lock(const http::TypedRequest & req);

  /// Format a time_point as ISO 8601 UTC string
  static std::string format_expiration(std::chrono::steady_clock::time_point expires_at);

 private:
  HandlerContext & ctx_;
  LockManager * lock_manager_;

  /// Check that locking is enabled; on failure returns the corresponding
  /// 501 ErrorInfo. Success carries no payload (monostate).
  tl::expected<void, ErrorInfo> check_locking_enabled() const;

  /// Extract and validate the X-Client-Id header. On failure returns the
  /// corresponding 400 ErrorInfo (missing / too long / control characters).
  tl::expected<std::string, ErrorInfo> require_client_id(const http::TypedRequest & req) const;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
