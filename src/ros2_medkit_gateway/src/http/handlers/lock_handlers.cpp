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

#include "ros2_medkit_gateway/core/http/handlers/lock_handlers.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>
#include <variant>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Wrap a thrown std::exception as a 500 internal-error ErrorInfo, logging
/// the underlying exception via the shared handler logger so operators still
/// see the original `what()` even though the wire response drops `details`
/// for unrelated entity ids.
ErrorInfo make_internal_error(const char * where, const std::string & message, const std::exception & e,
                              json params = {}) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  if (params.is_null()) {
    params = json::object();
  }
  if (!params.is_object()) {
    params = json::object();
  }
  params["details"] = e.what();
  return make_error(500, ERR_INTERNAL_ERROR, message, std::move(params));
}

/// Map internal LockManager error codes to SOVD standard error codes.
///
/// The LockManager uses descriptive internal codes (lock-conflict,
/// lock-not-owner, etc.) but SOVD spec requires standard error codes
/// (invalid-request, forbidden, etc.).
std::string to_sovd_error_code(const std::string & lock_code) {
  if (lock_code == "lock-conflict" || lock_code == "lock-not-breakable") {
    return ERR_INVALID_REQUEST;
  }
  if (lock_code == "lock-not-owner") {
    return ERR_FORBIDDEN;
  }
  if (lock_code == "lock-not-found") {
    return ERR_RESOURCE_NOT_FOUND;
  }
  if (lock_code == "invalid-scope" || lock_code == "invalid-expiration") {
    return ERR_INVALID_PARAMETER;
  }
  return lock_code;  // Pass through if no mapping
}

/// Build a typed Lock DTO from a LockInfo.
dto::Lock lock_info_to_dto(const LockInfo & lock, const std::string & client_id) {
  dto::Lock dto;
  dto.id = lock.lock_id;
  dto.owned = !client_id.empty() && lock.client_id == client_id;
  if (!lock.scopes.empty()) {
    dto.scopes = lock.scopes;
  }
  dto.lock_expiration = LockHandlers::format_expiration(lock.expires_at);
  return dto;
}

/// Read the positional entity-id capture group from the typed request.
///
/// The legacy handlers tested `req.matches.size() < 2` and returned
/// ERR_INVALID_REQUEST/400. TypedRequest::path_param would otherwise emit
/// ERR_INVALID_PARAMETER on that path, which is a different wire shape - so
/// the helper preserves the legacy `invalid-request` code/message exactly.
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the positional lock-id capture group from the typed request. The
/// legacy handlers used `req.matches[2]` and reported ERR_INVALID_REQUEST/400
/// when fewer than 3 captures were present.
tl::expected<std::string, ErrorInfo> read_lock_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

}  // namespace

LockHandlers::LockHandlers(HandlerContext & ctx, LockManager * lock_manager) : ctx_(ctx), lock_manager_(lock_manager) {
}

// ============================================================================
// Private helpers
// ============================================================================

tl::expected<void, ErrorInfo> LockHandlers::check_locking_enabled() const {
  if (!lock_manager_) {
    return tl::unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Locking is not enabled on this gateway"));
  }
  return {};
}

tl::expected<std::string, ErrorInfo> LockHandlers::require_client_id(const http::TypedRequest & req) const {
  static constexpr size_t kMaxClientIdLen = 256;

  auto header = req.header("X-Client-Id");
  std::string client_id = header.value_or("");
  if (client_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Missing required X-Client-Id header",
                                     json{{"details", "X-Client-Id header is required for lock operations"}}));
  }
  if (client_id.size() > kMaxClientIdLen) {
    return tl::unexpected(
        make_error(400, ERR_INVALID_PARAMETER, "X-Client-Id exceeds maximum length",
                   json{{"details", "X-Client-Id must be at most 256 characters"}, {"max_length", kMaxClientIdLen}}));
  }
  // Reject control characters (defense-in-depth).
  for (char c : client_id) {
    if (static_cast<unsigned char>(c) < 0x20) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "X-Client-Id contains invalid characters",
                                       json{{"details", "X-Client-Id must not contain control characters"}}));
    }
  }
  return client_id;
}

std::string LockHandlers::format_expiration(std::chrono::steady_clock::time_point expires_at) {
  // Convert steady_clock to system_clock for wall-clock time formatting.
  // Compute the offset between the two clocks at the current instant,
  // then apply it to the steady_clock time_point.
  auto now_steady = std::chrono::steady_clock::now();
  auto now_system = std::chrono::system_clock::now();
  auto offset = expires_at - now_steady;
  auto system_time = now_system + std::chrono::duration_cast<std::chrono::system_clock::duration>(offset);

  auto time_t_val = std::chrono::system_clock::to_time_t(system_time);
  std::tm tm_val{};
  gmtime_r(&time_t_val, &tm_val);

  std::ostringstream oss;
  oss << std::put_time(&tm_val, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

// ============================================================================
// Handler implementations
// ============================================================================

http::Result<std::pair<http::Created<dto::Lock>, http::ResponseAttachments>>
LockHandlers::post_lock(const http::TypedRequest & req, dto::AcquireLockRequest body) {
  if (auto guard = check_locking_enabled(); !guard) {
    return tl::unexpected(guard.error());
  }

  std::string entity_id;
  try {
    auto id_result = read_entity_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    entity_id = *id_result;

    // Require X-Client-Id for acquire.
    auto client_id_result = require_client_id(req);
    if (!client_id_result) {
      return tl::unexpected(client_id_result.error());
    }
    const std::string client_id = *client_id_result;

    // Validate entity exists and matches route type (peer-forward aware).
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Validate lock_expiration is positive.
    if (body.lock_expiration <= 0) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid lock_expiration",
                                       json{{"details", "lock_expiration must be a positive integer (seconds)"}}));
    }

    // Validate individual scope strings.
    std::vector<std::string> scopes;
    if (body.scopes.has_value()) {
      for (const auto & scope_str : *body.scopes) {
        if (valid_lock_scopes().find(scope_str) == valid_lock_scopes().end()) {
          std::string scope_list;
          for (const auto & s : valid_lock_scopes()) {
            if (!scope_list.empty()) {
              scope_list += ", ";
            }
            scope_list += s;
          }
          return tl::unexpected(
              make_error(400, ERR_INVALID_PARAMETER, "Unknown lock scope: " + scope_str,
                         json{{"details", "Valid scopes: " + scope_list}, {"invalid_scope", scope_str}}));
        }
        scopes.push_back(scope_str);
      }
    }

    bool break_lock = body.break_lock.value_or(false);

    // Acquire the lock.
    auto result = lock_manager_->acquire(entity_id, client_id, scopes, body.lock_expiration, break_lock);

    if (!result.has_value()) {
      const auto & err = result.error();
      return tl::unexpected(
          make_error(err.status_code, to_sovd_error_code(err.code), err.message,
                     err.existing_lock_id ? json{{"existing_lock_id", *err.existing_lock_id}} : json{}));
    }

    auto lock_dto = lock_info_to_dto(*result, client_id);
    http::ResponseAttachments att;
    att.with_location(std::string(req.path()) + "/" + result->lock_id);
    return std::make_pair(http::Created<dto::Lock>{std::move(lock_dto)}, std::move(att));

  } catch (const std::exception & e) {
    return tl::unexpected(
        make_internal_error("handle_acquire_lock", "Failed to acquire lock", e, json{{"entity_id", entity_id}}));
  }
}

http::Result<dto::Collection<dto::Lock>> LockHandlers::get_locks(const http::TypedRequest & req) {
  if (auto guard = check_locking_enabled(); !guard) {
    return tl::unexpected(guard.error());
  }

  std::string entity_id;
  try {
    auto id_result = read_entity_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    entity_id = *id_result;

    // Validate entity exists and matches route type.
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // X-Client-Id is optional for list (used for `owned` field).
    auto client_id = req.header("X-Client-Id").value_or("");

    auto lock = lock_manager_->get_lock(entity_id);

    dto::Collection<dto::Lock> response;
    if (lock) {
      response.items.push_back(lock_info_to_dto(*lock, client_id));
    }
    return response;

  } catch (const std::exception & e) {
    return tl::unexpected(
        make_internal_error("handle_list_locks", "Failed to list locks", e, json{{"entity_id", entity_id}}));
  }
}

http::Result<dto::Lock> LockHandlers::get_lock(const http::TypedRequest & req) {
  if (auto guard = check_locking_enabled(); !guard) {
    return tl::unexpected(guard.error());
  }

  std::string entity_id;
  std::string lock_id;
  try {
    auto id_result = read_entity_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    entity_id = *id_result;
    auto lock_id_result = read_lock_id(req);
    if (!lock_id_result) {
      return tl::unexpected(lock_id_result.error());
    }
    lock_id = *lock_id_result;

    // Validate entity exists and matches route type.
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // X-Client-Id is optional for get (used for `owned` field).
    auto client_id = req.header("X-Client-Id").value_or("");

    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                       json{{"lock_id", lock_id}, {"entity_id", entity_id}}));
    }

    return lock_info_to_dto(*lock, client_id);

  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("handle_get_lock", "Failed to get lock", e,
                                              json{{"entity_id", entity_id}, {"lock_id", lock_id}}));
  }
}

http::Result<http::NoContent> LockHandlers::put_lock(const http::TypedRequest & req, dto::ExtendLockRequest body) {
  if (auto guard = check_locking_enabled(); !guard) {
    return tl::unexpected(guard.error());
  }

  std::string entity_id;
  std::string lock_id;
  try {
    auto id_result = read_entity_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    entity_id = *id_result;
    auto lock_id_result = read_lock_id(req);
    if (!lock_id_result) {
      return tl::unexpected(lock_id_result.error());
    }
    lock_id = *lock_id_result;

    // Require X-Client-Id for extend.
    auto client_id_result = require_client_id(req);
    if (!client_id_result) {
      return tl::unexpected(client_id_result.error());
    }
    const std::string client_id = *client_id_result;

    // Validate entity exists and matches route type.
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Verify lock exists and belongs to this entity.
    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                       json{{"lock_id", lock_id}, {"entity_id", entity_id}}));
    }

    if (body.lock_expiration <= 0) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid lock_expiration",
                                       json{{"details", "lock_expiration must be a positive integer (seconds)"}}));
    }

    // Extend the lock.
    auto result = lock_manager_->extend(entity_id, client_id, body.lock_expiration);
    if (!result.has_value()) {
      const auto & err = result.error();
      return tl::unexpected(make_error(err.status_code, to_sovd_error_code(err.code), err.message));
    }
    return http::NoContent{};

  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("handle_extend_lock", "Failed to extend lock", e,
                                              json{{"entity_id", entity_id}, {"lock_id", lock_id}}));
  }
}

http::Result<http::NoContent> LockHandlers::del_lock(const http::TypedRequest & req) {
  if (auto guard = check_locking_enabled(); !guard) {
    return tl::unexpected(guard.error());
  }

  std::string entity_id;
  std::string lock_id;
  try {
    auto id_result = read_entity_id(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    entity_id = *id_result;
    auto lock_id_result = read_lock_id(req);
    if (!lock_id_result) {
      return tl::unexpected(lock_id_result.error());
    }
    lock_id = *lock_id_result;

    // Require X-Client-Id for release.
    auto client_id_result = require_client_id(req);
    if (!client_id_result) {
      return tl::unexpected(client_id_result.error());
    }
    const std::string client_id = *client_id_result;

    // Validate entity exists and matches route type.
    auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Verify lock exists and belongs to this entity.
    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                       json{{"lock_id", lock_id}, {"entity_id", entity_id}}));
    }

    // Release the lock.
    auto result = lock_manager_->release(entity_id, client_id);
    if (!result.has_value()) {
      const auto & err = result.error();
      return tl::unexpected(make_error(err.status_code, to_sovd_error_code(err.code), err.message));
    }
    return http::NoContent{};

  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("handle_release_lock", "Failed to release lock", e,
                                              json{{"entity_id", entity_id}, {"lock_id", lock_id}}));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
