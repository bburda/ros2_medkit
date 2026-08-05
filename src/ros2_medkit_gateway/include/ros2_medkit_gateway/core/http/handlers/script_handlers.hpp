// Copyright 2026 Bartlomiej Burda
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

#include "ros2_medkit_gateway/core/managers/script_manager.hpp"
#include "ros2_medkit_gateway/dto/scripts.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// HTTP handlers for the per-entity Scripts collection.
///
/// PR-403 commit 24: 8 script routes migrated to the typed RouteRegistry API.
/// Every handler returns `http::Result<T>` (or a `pair<T, ResponseAttachments>`
/// for routes that need to emit 201 + Location). The HATEOAS `_links` envelope
/// on the list endpoint is now a typed `dto::HateoasLinks` sub-struct so the
/// JSON, schema, and reader paths all flow through the single descriptor.
/// Wire format is unchanged byte-for-byte.
class ScriptHandlers {
 public:
  ScriptHandlers(HandlerContext & ctx, ScriptManager * script_manager);

  /// POST /{entity}/scripts - multipart upload, returns 201 + Location.
  http::Result<std::pair<http::Created<dto::ScriptUploadResponse>, http::ResponseAttachments>>
  upload_script(const http::TypedRequest & req, const http::MultipartBody & body);

  /// GET /{entity}/scripts - list scripts, typed HATEOAS envelope.
  http::Result<dto::ScriptList> list_scripts(const http::TypedRequest & req);

  /// GET /{entity}/scripts/{script_id} - get a single script.
  http::Result<dto::ScriptMetadata> get_script(const http::TypedRequest & req);

  /// DELETE /{entity}/scripts/{script_id} - delete a script, 204.
  http::Result<http::NoContent> delete_script(const http::TypedRequest & req);

  /// POST /{entity}/scripts/{script_id}/executions - start, returns 202 + Location.
  http::Result<std::pair<http::Accepted<dto::ScriptExecution>, http::ResponseAttachments>>
  start_execution(const http::TypedRequest & req);

  /// GET /{entity}/scripts/{script_id}/executions/{execution_id} - get status.
  http::Result<dto::ScriptExecution> get_execution(const http::TypedRequest & req);

  /// PUT /{entity}/scripts/{script_id}/executions/{execution_id} - control execution.
  http::Result<dto::ScriptExecution> control_execution(const http::TypedRequest & req,
                                                       const dto::ScriptControlRequest & body);

  /// DELETE /{entity}/scripts/{script_id}/executions/{execution_id} - remove record, 204.
  http::Result<http::NoContent> delete_execution(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
  ScriptManager * script_mgr_;

  static bool is_valid_resource_id(const std::string & id);
  static std::string entity_type_from_path(const std::string & path);
  static dto::ScriptMetadata script_info_to_dto(const ScriptInfo & info, const std::string & base_path);
  static dto::ScriptExecution execution_info_to_dto(const ExecutionInfo & info);
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
