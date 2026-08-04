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
#include <utility>
#include <variant>

#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/operations/operation_types.hpp"
#include "ros2_medkit_gateway/dto/operations.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace detail {

/// Failure shape produced by `map_cancel_result`. `std::nullopt` from the
/// mapper means "the cancellation is in progress" and the entry point should
/// render its success shape (204 for DELETE, 202 for PUT-stop).
struct CancelFailure {
  int http_status;
  const char * error_code;
  std::string message;
};

/// Shared outcome mapping for the two cancel entry points (DELETE execution
/// and PUT-stop) - issue #576. Declared here rather than kept file-local so
/// the wire contract of every `CancelOutcome` is directly testable: the
/// manager-side guards (invalid uuid / goal evicted between the handler's
/// lookup and the manager's re-check) are only reachable over HTTP through a
/// race, so their mapping has to be pinned at this seam.
///
/// @param verb "Cancel" or "Stop" - keeps each entry point's message wording.
std::optional<CancelFailure> map_cancel_result(const ActionCancelResult & result, OperationManager & operation_mgr,
                                               const std::string & execution_id, const char * verb);

}  // namespace detail

/**
 * @brief Handlers for operation-related REST API endpoints (services and actions).
 *
 * Handlers:
 * - GET /{entity}/operations - List all operations (7.14.3)
 * - GET /{entity}/operations/{op-id} - Get operation details (7.14.4)
 * - GET /{entity}/operations/{op-id}/executions - List executions (7.14.5)
 * - POST /{entity}/operations/{op-id}/executions - Start execution (7.14.6)
 * - GET /{entity}/operations/{op-id}/executions/{exec-id} - Get execution status (7.14.7)
 * - PUT /{entity}/operations/{op-id}/executions/{exec-id} - Update execution (7.14.9)
 * - DELETE /{entity}/operations/{op-id}/executions/{exec-id} - Terminate execution (7.14.8)
 *
 * PR-403 commit 27: all 7 routes migrate to the typed RouteRegistry API. The
 * synchronous-vs-asynchronous execution dispatch on `POST executions` uses
 * `post_alternates<ExecutionCreateRequest, OperationExecutionResult,
 * ExecutionCreateAsync>` plus a `ResponseAttachments` channel so the framework
 * picks 200 for services / 202 for actions and applies a `Location` header on
 * the 202 branch. The `list_executions` endpoint now returns the typed
 * `Collection<ExecutionId>` (missed migration from earlier PR). The inline
 * service-error path in the legacy handler is replaced with the framework
 * error path (typed `ErrorInfo` return).
 */
class OperationHandlers {
 public:
  /// Construct operation handlers with shared context.
  explicit OperationHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// GET /{entity}/operations - list operations for an entity. Mixes runtime
  /// discovery (services + actions) with the per-entity plugin OperationProvider.
  http::Result<dto::Collection<dto::OperationItem>> list_operations(const http::TypedRequest & req);

  /// GET /{entity}/operations/{op_id} - get operation details.
  http::Result<dto::OperationDetail> get_operation(const http::TypedRequest & req);

  /// POST /{entity}/operations/{op_id}/executions - start an execution.
  ///
  /// Returns a `std::variant<OperationExecutionResult, ExecutionCreateAsync>`:
  /// - `OperationExecutionResult` -> 200 OK (synchronous service / plugin op)
  /// - `ExecutionCreateAsync`     -> 202 Accepted + Location header
  ///                                 (asynchronous ROS 2 action)
  /// The `ResponseAttachments` companion lets the async branch append the
  /// `Location` header without re-introducing a `httplib::Response &`.
  http::Result<
      std::pair<std::variant<dto::OperationExecutionResult, dto::ExecutionCreateAsync>, http::ResponseAttachments>>
  create_execution(const http::TypedRequest & req, dto::ExecutionCreateRequest body);

  /// GET /{entity}/operations/{op_id}/executions - list current executions.
  http::Result<dto::Collection<dto::ExecutionId>> list_executions(const http::TypedRequest & req);

  /// GET /{entity}/operations/{op_id}/executions/{exec_id} - execution status.
  http::Result<dto::OperationExecution> get_execution(const http::TypedRequest & req);

  /// DELETE /{entity}/operations/{op_id}/executions/{exec_id} - cancel.
  http::Result<http::NoContent> cancel_execution(const http::TypedRequest & req);

  /// PUT /{entity}/operations/{op_id}/executions/{exec_id} - update execution.
  ///
  /// Returns `OperationExecution` + attachments so the supported `stop`
  /// capability can emit 202 + `Location` (the SOVD async-update convention)
  /// while the success body stays 200 for any future synchronous capability
  /// that might land.
  http::Result<std::pair<dto::OperationExecution, http::ResponseAttachments>>
  update_execution(const http::TypedRequest & req, const dto::ExecutionUpdateRequest & body);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
