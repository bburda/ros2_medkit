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
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// Lock - SOVD lock response object.
//
// Emitted by all lock response handlers (acquire, list, get):
//   handle_acquire_lock (201 body), handle_list_locks (items array element),
//   handle_get_lock (200 body).
//
// Wire keys (from LockHandlers::lock_to_json):
//   id              - lock UUID (required)
//   owned           - true when X-Client-Id matches the lock owner (required)
//   scopes          - lock scope strings e.g. ["configurations", "operations"]
//                     (optional - absent when lock has no specific scopes)
//   lock_expiration - ISO 8601 UTC timestamp, e.g. "2026-01-01T00:05:00Z" (required)
// =============================================================================
struct Lock {
  std::string id;
  bool owned{false};
  std::optional<std::vector<std::string>> scopes;
  std::string lock_expiration;  // ISO 8601 date-time string
};

template <>
inline constexpr auto dto_fields<Lock> =
    std::make_tuple(field("id", &Lock::id), field("owned", &Lock::owned), field("scopes", &Lock::scopes),
                    field("lock_expiration", &Lock::lock_expiration));

template <>
inline constexpr std::string_view dto_name<Lock> = "Lock";

// =============================================================================
// AcquireLockRequest - POST /{entity}/locks request body.
// Parsed by handle_acquire_lock via parse_body<AcquireLockRequest>.
//
// Wire keys (from handle_acquire_lock body parsing + acquire_lock_request_schema):
//   lock_expiration - lock duration in seconds, must be > 0 (required, integer)
//   scopes          - optional list of scope strings (optional, array of strings)
//   break_lock      - force-acquire by breaking an existing lock (optional, bool)
// =============================================================================
struct AcquireLockRequest {
  int lock_expiration{0};  // seconds; additional validation: must be > 0
  std::optional<std::vector<std::string>> scopes;
  std::optional<bool> break_lock;
};

template <>
inline constexpr auto dto_fields<AcquireLockRequest> = std::make_tuple(
    field("lock_expiration", &AcquireLockRequest::lock_expiration,
          "How long the lock lives, in seconds from the moment the gateway grants it - a duration, unlike the "
          "`lock_expiration` on the Lock response, which is an ISO 8601 instant. Must be at least 1 and no more "
          "than the configured ceiling (`locking.default_max_expiration`, 3600 s unless a manifest overrides it "
          "for the entity); only the lower bound is a schema constraint, because the ceiling is deployment "
          "configuration. Extending a lock restarts the countdown from the new value - it does not add to what "
          "is left. Reaching the expiry is not the same as releasing the lock; the acquire operation says how "
          "they differ.",
          FieldConstraints{/*minimum=*/1.0, {}, {}, {}, {}}),
    field("scopes", &AcquireLockRequest::scopes,
          "Resource collections the lock covers, from `data`, `operations`, `configurations`, `faults`, `modes`, "
          "`scripts`, `bulk-data`, `logs` and `cyclic-subscriptions` (the set `valid_lock_scopes()` accepts; any "
          "other value is rejected with 400). Omitted or empty locks every collection."),
    field("break_lock", &AcquireLockRequest::break_lock,
          "Take the lock even though another client already holds one. Without it an existing lock answers 409 "
          "`lock-conflict`; with it, an existing lock the entity's configuration marks non-breakable answers 409 "
          "`lock-not-breakable` instead. Either way the 409 body carries the blocking lock's id in "
          "`parameters.existing_lock_id`."));

template <>
inline constexpr std::string_view dto_name<AcquireLockRequest> = "AcquireLockRequest";

// =============================================================================
// ExtendLockRequest - PUT /{entity}/locks/{lock_id} request body.
// Parsed by handle_extend_lock via parse_body<ExtendLockRequest>.
//
// Wire keys (from handle_extend_lock body parsing + extend_lock_request_schema):
//   lock_expiration - additional seconds to extend the lock, must be > 0 (required, integer)
// =============================================================================
struct ExtendLockRequest {
  int lock_expiration{0};  // additional seconds; additional validation: must be > 0
};

template <>
inline constexpr auto dto_fields<ExtendLockRequest> =
    std::make_tuple(field("lock_expiration", &ExtendLockRequest::lock_expiration));

template <>
inline constexpr std::string_view dto_name<ExtendLockRequest> = "ExtendLockRequest";

// =============================================================================
// Collection<Lock> - named "LockList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<Lock>> = "LockList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
