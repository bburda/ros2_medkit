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

#include "ros2_medkit_gateway/core/http/handlers/update_handlers.hpp"

#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Wrap a thrown std::exception as a 500 internal-error ErrorInfo, logging
/// the original `what()` via the shared handler logger.
ErrorInfo make_internal_error(const char * where, const std::exception & e) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  return make_error(500, ERR_INTERNAL_ERROR, e.what());
}

/// Convert an `UpdateStatusInfo` (manager-side struct) into the wire-typed
/// `dto::UpdateStatus`. Mirrors the legacy `to_update_status_dto` helper but
/// lives in an anonymous namespace inside the typed handler.
dto::UpdateStatus to_update_status_dto(const UpdateStatusInfo & info) {
  dto::UpdateStatus dto;
  dto.status = update_status_to_string(info.status);
  dto.x_medkit.phase = update_phase_to_string(info.phase);
  if (info.progress.has_value()) {
    dto.progress = *info.progress;
  }
  if (info.sub_progress.has_value()) {
    std::vector<dto::UpdateSubProgress> sub;
    sub.reserve(info.sub_progress->size());
    for (const auto & sp : *info.sub_progress) {
      sub.push_back({sp.name, sp.progress});
    }
    dto.sub_progress = std::move(sub);
  }
  if (info.error_message.has_value()) {
    dto.error = *info.error_message;
  }
  return dto;
}

/// Read the positional update-id capture group from the typed request. The
/// legacy handlers used `req.matches[1]` and did not test arity (cpp-httplib
/// guarantees the regex match before invoking the route); the typed wrapper
/// preserves the same shape and reports ERR_INVALID_REQUEST/400 only on the
/// unreachable "no capture group" path.
tl::expected<std::string, ErrorInfo> read_update_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Validate the update id with the shared entity-id rules (CRLF-injection-
/// safe; bounded length; alphanumeric + `_-`). Returns a 400 ErrorInfo on
/// rejection matching the legacy wire shape.
tl::expected<void, ErrorInfo> validate_update_id(HandlerContext & ctx, const std::string & id) {
  auto vr = ctx.validate_entity_id(id);
  if (!vr) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, vr.error()));
  }
  return {};
}

/// Map a manager-side `UpdateError` to a SOVD ErrorInfo for the
/// register_update path. Legacy mapping: AlreadyExists -> 400 update-already-
/// exists; everything else -> 400 invalid-request.
ErrorInfo map_register_error(const UpdateError & err) {
  if (err.code == UpdateErrorCode::AlreadyExists) {
    return make_error(400, ERR_X_MEDKIT_UPDATE_ALREADY_EXISTS, err.message);
  }
  return make_error(400, ERR_INVALID_REQUEST, err.message);
}

/// Map a manager-side `UpdateError` to a SOVD ErrorInfo for the delete_update
/// path. Legacy mapping: NotFound -> 404 update-not-found; InProgress -> 409
/// update-in-progress; otherwise 500 internal-error.
ErrorInfo map_delete_error(const UpdateError & err) {
  switch (err.code) {
    case UpdateErrorCode::InProgress:
      return make_error(409, ERR_X_MEDKIT_UPDATE_IN_PROGRESS, err.message);
    case UpdateErrorCode::NotFound:
      return make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, err.message);
    case UpdateErrorCode::AlreadyExists:
    case UpdateErrorCode::NotPrepared:
    case UpdateErrorCode::NotAutomated:
    case UpdateErrorCode::InvalidRequest:
    case UpdateErrorCode::Deleting:
    case UpdateErrorCode::NoBackend:
    case UpdateErrorCode::Internal:
    default:
      return make_error(500, ERR_INTERNAL_ERROR, err.message);
  }
}

/// Map a manager-side `UpdateError` to a SOVD ErrorInfo for the start_prepare
/// path. Legacy mapping: NotFound -> 404; InProgress/Deleting -> 409;
/// otherwise 400 invalid-request.
ErrorInfo map_prepare_error(const UpdateError & err) {
  switch (err.code) {
    case UpdateErrorCode::NotFound:
      return make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, err.message);
    case UpdateErrorCode::InProgress:
    case UpdateErrorCode::Deleting:
      return make_error(409, ERR_X_MEDKIT_UPDATE_IN_PROGRESS, err.message);
    case UpdateErrorCode::AlreadyExists:
    case UpdateErrorCode::NotPrepared:
    case UpdateErrorCode::NotAutomated:
    case UpdateErrorCode::InvalidRequest:
    case UpdateErrorCode::NoBackend:
    case UpdateErrorCode::Internal:
    default:
      return make_error(400, ERR_INVALID_REQUEST, err.message);
  }
}

/// Map a manager-side `UpdateError` to a SOVD ErrorInfo for the start_execute
/// path. Legacy mapping: NotFound -> 404; NotPrepared -> 400 update-not-
/// prepared; InProgress/Deleting -> 409; otherwise 400 invalid-request.
ErrorInfo map_execute_error(const UpdateError & err) {
  switch (err.code) {
    case UpdateErrorCode::NotFound:
      return make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, err.message);
    case UpdateErrorCode::NotPrepared:
      return make_error(400, ERR_X_MEDKIT_UPDATE_NOT_PREPARED, err.message);
    case UpdateErrorCode::InProgress:
    case UpdateErrorCode::Deleting:
      return make_error(409, ERR_X_MEDKIT_UPDATE_IN_PROGRESS, err.message);
    case UpdateErrorCode::AlreadyExists:
    case UpdateErrorCode::NotAutomated:
    case UpdateErrorCode::InvalidRequest:
    case UpdateErrorCode::NoBackend:
    case UpdateErrorCode::Internal:
    default:
      return make_error(400, ERR_INVALID_REQUEST, err.message);
  }
}

/// Map a manager-side `UpdateError` to a SOVD ErrorInfo for the start_automated
/// path. Legacy mapping: NotFound -> 404; NotAutomated -> 400 update-not-
/// automated; InProgress/Deleting -> 409; otherwise 400 invalid-request.
ErrorInfo map_automated_error(const UpdateError & err) {
  switch (err.code) {
    case UpdateErrorCode::NotFound:
      return make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, err.message);
    case UpdateErrorCode::NotAutomated:
      return make_error(400, ERR_X_MEDKIT_UPDATE_NOT_AUTOMATED, err.message);
    case UpdateErrorCode::InProgress:
    case UpdateErrorCode::Deleting:
      return make_error(409, ERR_X_MEDKIT_UPDATE_IN_PROGRESS, err.message);
    case UpdateErrorCode::AlreadyExists:
    case UpdateErrorCode::NotPrepared:
    case UpdateErrorCode::InvalidRequest:
    case UpdateErrorCode::NoBackend:
    case UpdateErrorCode::Internal:
    default:
      return make_error(400, ERR_INVALID_REQUEST, err.message);
  }
}

}  // namespace

UpdateHandlers::UpdateHandlers(HandlerContext & ctx, UpdateManager * update_manager)
  : ctx_(ctx), update_mgr_(update_manager) {
}

std::optional<ErrorInfo> UpdateHandlers::check_backend() const {
  if (!update_mgr_ || !update_mgr_->has_backend()) {
    return make_error(501, ERR_NOT_IMPLEMENTED, "Software updates backend not configured");
  }
  return std::nullopt;
}

http::Result<dto::UpdateList> UpdateHandlers::get_updates(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    const auto q = req.query<dto::UpdateListQuery>();
    UpdateFilter filter;
    if (q.origin) {
      filter.origin = *q.origin;
    }
    if (q.target_version) {
      filter.target_version = *q.target_version;
    }

    auto result = update_mgr_->list_updates(filter);
    if (!result) {
      return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, result.error().message));
    }
    return dto::UpdateList{std::move(*result)};
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_updates", e));
  }
}

http::Result<dto::UpdateDetail> UpdateHandlers::get_update(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->get_update(id);
    if (!result) {
      return tl::unexpected(make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error().message));
    }
    return *result;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_update", e));
  }
}

http::Result<std::pair<http::Created<dto::UpdateRegisterResponse>, http::ResponseAttachments>>
UpdateHandlers::post_update(const http::TypedRequest & /*req*/, dto::UpdateRegisterRequest body) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    // The framework's JsonReader<UpdateRegisterRequest> already rejected non-
    // object payloads with a 400 invalid-request. The remaining required
    // field-level check is the `id` presence + format, both preserved here
    // verbatim from the legacy handler to keep wire shapes byte-identical.
    if (!body.content.contains("id") || !body.content["id"].is_string() ||
        body.content["id"].get<std::string>().empty()) {
      return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing required field: id"));
    }
    auto id = body.content["id"].get<std::string>();

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->register_update(body.content);
    if (!result) {
      return tl::unexpected(map_register_error(result.error()));
    }

    dto::UpdateRegisterResponse resp;
    resp.id = id;
    http::ResponseAttachments att;
    att.with_location(api_path("/updates/" + id));
    return std::make_pair(http::Created<dto::UpdateRegisterResponse>{std::move(resp)}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("post_update", e));
  }
}

http::Result<http::NoContent> UpdateHandlers::del_update(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->delete_update(id);
    if (!result) {
      return tl::unexpected(map_delete_error(result.error()));
    }
    return http::NoContent{};
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("del_update", e));
  }
}

http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
UpdateHandlers::put_prepare(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->start_prepare(id);
    if (!result) {
      return tl::unexpected(map_prepare_error(result.error()));
    }
    http::ResponseAttachments att;
    att.with_location(api_path("/updates/" + id + "/status"));
    return std::make_pair(http::Accepted<http::NoContent>{http::NoContent{}}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("put_prepare", e));
  }
}

http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
UpdateHandlers::put_execute(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->start_execute(id);
    if (!result) {
      return tl::unexpected(map_execute_error(result.error()));
    }
    http::ResponseAttachments att;
    att.with_location(api_path("/updates/" + id + "/status"));
    return std::make_pair(http::Accepted<http::NoContent>{http::NoContent{}}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("put_execute", e));
  }
}

http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>>
UpdateHandlers::put_automated(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->start_automated(id);
    if (!result) {
      return tl::unexpected(map_automated_error(result.error()));
    }
    http::ResponseAttachments att;
    att.with_location(api_path("/updates/" + id + "/status"));
    return std::make_pair(http::Accepted<http::NoContent>{http::NoContent{}}, std::move(att));
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("put_automated", e));
  }
}

http::Result<dto::UpdateStatus> UpdateHandlers::get_status(const http::TypedRequest & req) {
  if (auto guard = check_backend()) {
    return tl::unexpected(*guard);
  }
  try {
    auto id_result = read_update_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const auto & id = *id_result;

    if (auto vr = validate_update_id(ctx_, id); !vr) {
      return tl::unexpected(vr.error());
    }

    auto result = update_mgr_->get_status(id);
    if (!result) {
      return tl::unexpected(make_error(404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error().message));
    }
    return to_update_status_dto(*result);
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_status", e));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
