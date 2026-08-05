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

#include "ros2_medkit_gateway/core/http/handlers/script_handlers.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Read a positional capture group from the typed request. The legacy handlers
/// used `req.matches[N]` without bounds checking; the typed surface refuses
/// out-of-range captures with ERR_INVALID_REQUEST/400. cpp-httplib only routes
/// when the regex matches, so this branch is effectively unreachable in
/// production, but the helper keeps the typed flow explicit.
tl::expected<std::string, ErrorInfo> read_capture(const http::TypedRequest & req, std::string_view index) {
  auto raw = req.path_param(index);
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Map a ScriptBackendError code to the matching SOVD ErrorInfo. Centralised
/// so every typed handler routes backend errors through the same translation
/// table the legacy `send_script_error` produced; the wire body stays byte-
/// identical (status + error_code/vendor_code/message).
ErrorInfo script_backend_error(const ScriptBackendErrorInfo & err) {
  switch (err.code) {
    case ScriptBackendError::NotFound:
      return make_error(404, ERR_RESOURCE_NOT_FOUND, err.message);
    case ScriptBackendError::AlreadyExists:
      return make_error(409, ERR_SCRIPT_ALREADY_EXISTS, err.message);
    case ScriptBackendError::ManagedScript:
      return make_error(409, ERR_SCRIPT_MANAGED, err.message);
    case ScriptBackendError::AlreadyRunning:
      return make_error(409, ERR_SCRIPT_RUNNING, err.message);
    case ScriptBackendError::NotRunning:
      return make_error(409, ERR_SCRIPT_NOT_RUNNING, err.message);
    case ScriptBackendError::InvalidInput:
      return make_error(400, ERR_INVALID_REQUEST, err.message);
    case ScriptBackendError::UnsupportedType:
      return make_error(400, ERR_INVALID_PARAMETER, err.message);
    case ScriptBackendError::FileTooLarge:
      return make_error(413, ERR_SCRIPT_FILE_TOO_LARGE, err.message);
    case ScriptBackendError::ConcurrencyLimit:
      return make_error(429, ERR_SCRIPT_CONCURRENCY_LIMIT, err.message);
    case ScriptBackendError::Internal:
    default:
      return make_error(500, ERR_INTERNAL_ERROR, err.message);
  }
}

}  // namespace

ScriptHandlers::ScriptHandlers(HandlerContext & ctx, ScriptManager * script_manager)
  : ctx_(ctx), script_mgr_(script_manager) {
}

std::string ScriptHandlers::entity_type_from_path(const std::string & path) {
  // The collection the caller actually addressed, read back out of the routed
  // request path (`/api/v1/<collection>/<id>/scripts/...`).
  //
  // The scripts routes are registered for apps and components only
  // (`rest_server.cpp`, the `et_type_str == "apps" || == "components"` guard),
  // so the previous "components if the path says so, else apps" choice happened
  // to be right for every path that routes today. Reading the segment instead
  // of choosing it means it stays right if that list grows - and until then it
  // is one fewer place that has to be found and edited.
  //
  // The empty return below is a defensive fallback, not a supported answer: an
  // unprefixed path cannot reach a handler, because routes are mounted under
  // `API_BASE_PATH`. If it ever did, the caller would build `//<id>` from it.
  static const std::string kPrefix = std::string(API_BASE_PATH) + "/";
  if (path.compare(0, kPrefix.size(), kPrefix) != 0) {
    return {};
  }
  const size_t start = kPrefix.size();
  const size_t end = path.find('/', start);
  return end == std::string::npos ? path.substr(start) : path.substr(start, end - start);
}

bool ScriptHandlers::is_valid_resource_id(const std::string & id) {
  if (id.empty() || id.size() > 256) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

dto::ScriptMetadata ScriptHandlers::script_info_to_dto(const ScriptInfo & info, const std::string & base_path) {
  dto::ScriptMetadata meta;
  meta.id = info.id;
  meta.name = info.name;
  meta.description = info.description;
  meta.href = api_path(base_path + "/scripts/" + info.id);
  meta.managed = info.managed;
  meta.proximity_proof_required = info.proximity_proof_required;
  meta.parameters_schema = info.parameters_schema;
  return meta;
}

dto::ScriptExecution ScriptHandlers::execution_info_to_dto(const ExecutionInfo & info) {
  dto::ScriptExecution exec;
  exec.id = info.id;
  exec.status = info.status;
  exec.progress = info.progress;
  exec.started_at = info.started_at;
  exec.completed_at = info.completed_at;
  exec.parameters = info.output_parameters;
  exec.error = info.error;
  return exec;
}

// ---------------------------------------------------------------------------
// GET /{entity}/scripts - list scripts with typed HATEOAS envelope
// ---------------------------------------------------------------------------

http::Result<dto::ScriptList> ScriptHandlers::list_scripts(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto entity_type_segment = entity_type_from_path(req.path());
    auto base_path = "/" + entity_type_segment + "/" + entity_id;

    auto result = script_mgr_->list_scripts(entity_id);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }

    dto::ScriptList list;
    list.items.reserve(result->size());
    for (const auto & info : *result) {
      list.items.push_back(script_info_to_dto(info, base_path));
    }

    dto::HateoasLinks links;
    links.self = api_path(base_path + "/scripts");
    links.parent = api_path(base_path);
    list.links = std::move(links);
    return list;
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// POST /{entity}/scripts - multipart upload, 201 + Location
// ---------------------------------------------------------------------------

http::Result<std::pair<http::Created<dto::ScriptUploadResponse>, http::ResponseAttachments>>
ScriptHandlers::upload_script(const http::TypedRequest & req, const http::MultipartBody & body) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  // The framework's multipart_upload route accepts any Content-Type but the
  // legacy handler enforced `multipart/form-data` explicitly to match SOVD's
  // upload contract. Re-check via the typed header accessor so the validation
  // surface stays identical for clients sending JSON-with-attachments by
  // mistake.
  auto content_type = req.header("Content-Type").value_or(std::string{});
  if (content_type.find("multipart/form-data") == std::string::npos) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Expected Content-Type: multipart/form-data"));
  }

  // Locate the `file` part. cpp-httplib parses every named part into
  // MultipartBody.parts; we walk the vector instead of relying on `req.files`
  // map ordering so the typed surface does not leak through to the cpp-httplib
  // shape.
  // FIXME(#409): httplib::MultipartFormData is unavailable in cpp-httplib
  // >= 0.20; migrate to the req.form API (tracked with the vendored-header pin).
  const httplib::MultipartFormData * file_part = nullptr;
  const httplib::MultipartFormData * metadata_part = nullptr;
  for (const auto & part : body.parts) {
    if (part.name == "file" && !file_part) {
      file_part = &part;
    } else if (part.name == "metadata" && !metadata_part) {
      metadata_part = &part;
    }
  }

  if (!file_part) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing required multipart field: file"));
  }

  std::optional<json> metadata;
  if (metadata_part) {
    try {
      metadata = json::parse(metadata_part->content);
    } catch (const json::parse_error &) {
      return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid JSON in metadata field"));
    }
  }

  try {
    auto result = script_mgr_->upload_script(entity_id, file_part->filename, file_part->content, metadata);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }

    // The script is a child of the POST target, and `req.path()` already
    // carries the API prefix, so this is the same absolute form every `href`
    // uses - and it names the collection the caller addressed.
    const std::string script_path = child_resource_path(req.path(), result->id);

    dto::ScriptUploadResponse upload_resp;
    upload_resp.id = result->id;
    upload_resp.name = result->name;

    http::ResponseAttachments att;
    att.with_location(script_path);
    return std::make_pair(http::Created<dto::ScriptUploadResponse>{std::move(upload_resp)}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// GET /{entity}/scripts/{script_id}
// ---------------------------------------------------------------------------

http::Result<dto::ScriptMetadata> ScriptHandlers::get_script(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto entity_type_segment = entity_type_from_path(req.path());
    auto base_path = "/" + entity_type_segment + "/" + entity_id;

    auto result = script_mgr_->get_script(entity_id, script_id);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }
    return script_info_to_dto(*result, base_path);
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// DELETE /{entity}/scripts/{script_id} - 204
// ---------------------------------------------------------------------------

http::Result<http::NoContent> ScriptHandlers::delete_script(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto result = script_mgr_->delete_script(entity_id, script_id);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }
    return http::NoContent{};
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// POST /{entity}/scripts/{script_id}/executions - 202 + Location
// ---------------------------------------------------------------------------

http::Result<std::pair<http::Accepted<dto::ScriptExecution>, http::ResponseAttachments>>
ScriptHandlers::start_execution(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  // Body shape is free-form (parameters is provider-defined), so the typed
  // router's `request_body<TBody>` parsing is not the right fit here: we still
  // accept the raw bytes via the framework escape hatch and parse manually to
  // surface the same validation errors as the legacy handler.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop

  json body;
  try {
    body = json::parse(raw_req.body);
  } catch (const json::parse_error &) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid JSON body"));
  }

  if (!body.contains("execution_type") || !body["execution_type"].is_string() ||
      body["execution_type"].get<std::string>().empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing required field: execution_type"));
  }

  ExecutionRequest exec_req;
  exec_req.execution_type = body["execution_type"].get<std::string>();

  if (body.contains("parameters") && !body["parameters"].is_null()) {
    exec_req.parameters = body["parameters"];
  }

  if (body.contains("proximity_response") && body["proximity_response"].is_string()) {
    exec_req.proximity_response = body["proximity_response"].get<std::string>();
  }

  try {
    auto result = script_mgr_->start_execution(entity_id, script_id, exec_req);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }

    // Same as the upload above: the execution is a child of the POST target.
    const std::string exec_path = child_resource_path(req.path(), result->id);

    http::ResponseAttachments att;
    att.with_location(exec_path);
    return std::make_pair(http::Accepted<dto::ScriptExecution>{execution_info_to_dto(*result)}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// GET /{entity}/scripts/{script_id}/executions/{execution_id}
// ---------------------------------------------------------------------------

http::Result<dto::ScriptExecution> ScriptHandlers::get_execution(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  auto execution_id_result = read_capture(req, "3");
  if (!execution_id_result) {
    return tl::unexpected(execution_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;
  const std::string execution_id = *execution_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }
  if (!is_valid_resource_id(execution_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid execution ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto result = script_mgr_->get_execution(entity_id, script_id, execution_id);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }
    return execution_info_to_dto(*result);
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// PUT /{entity}/scripts/{script_id}/executions/{execution_id}
// ---------------------------------------------------------------------------

http::Result<dto::ScriptExecution> ScriptHandlers::control_execution(const http::TypedRequest & req,
                                                                     const dto::ScriptControlRequest & body) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  auto execution_id_result = read_capture(req, "3");
  if (!execution_id_result) {
    return tl::unexpected(execution_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;
  const std::string execution_id = *execution_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }
  if (!is_valid_resource_id(execution_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid execution ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto result = script_mgr_->control_execution(entity_id, script_id, execution_id, body.action);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }
    return execution_info_to_dto(*result);
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

// ---------------------------------------------------------------------------
// DELETE /{entity}/scripts/{script_id}/executions/{execution_id} - 204
// ---------------------------------------------------------------------------

http::Result<http::NoContent> ScriptHandlers::delete_execution(const http::TypedRequest & req) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured"));
  }

  auto id_result = read_capture(req, "1");
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  auto script_id_result = read_capture(req, "2");
  if (!script_id_result) {
    return tl::unexpected(script_id_result.error());
  }
  auto execution_id_result = read_capture(req, "3");
  if (!execution_id_result) {
    return tl::unexpected(execution_id_result.error());
  }
  const std::string entity_id = *id_result;
  const std::string script_id = *script_id_result;
  const std::string execution_id = *execution_id_result;

  if (!is_valid_resource_id(script_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid script ID format"));
  }
  if (!is_valid_resource_id(execution_id)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid execution ID format"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS); !access) {
    return tl::unexpected(access.error());
  }

  try {
    auto result = script_mgr_->delete_execution(entity_id, script_id, execution_id);
    if (!result) {
      return tl::unexpected(script_backend_error(result.error()));
    }
    return http::NoContent{};
  } catch (const std::exception & e) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, e.what()));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
