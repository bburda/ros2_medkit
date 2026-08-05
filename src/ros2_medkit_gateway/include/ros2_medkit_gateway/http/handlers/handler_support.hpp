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

#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/http/detail/status_recorder.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// The `peer:<name>` source of a member another gateway contributed, or empty
/// for a member this gateway runs or has never heard of.
///
/// One definition for every collection, because the reading decides three
/// things that have to agree: whether this gateway's own walk may report an
/// item, whether the fan-out copy is the one that counts, and which gateway a
/// request for it goes to. Two collections reading it apart would list an item
/// from one place and address it to another.
///
/// The declared source rather than the routing table: a member whose gateway has
/// gone quiet is retained under the same source, and a request for its items has
/// to keep resolving to it in order to be answered "not responding" rather than
/// sampled here and reported absent.
inline std::string peer_source_of_member(const ThreadSafeEntityCache & cache, const std::string & member_id) {
  static constexpr std::string_view kPeerPrefix = "peer:";
  const auto peer_source = [](const std::string & source) {
    return source.rfind(kPeerPrefix, 0) == 0 ? source : std::string{};
  };
  if (auto app = cache.get_app(member_id)) {
    return peer_source(app->source);
  }
  if (auto component = cache.get_component(member_id)) {
    return peer_source(component->source);
  }
  return {};
}

/// True for a member whose declaration reached this gateway from a peer, and so
/// whose items another gateway serves.
inline bool member_is_peer_contributed(const ThreadSafeEntityCache & cache, const std::string & member_id) {
  return !peer_source_of_member(cache, member_id).empty();
}

/// Build a SOVD-shaped ErrorInfo. Empty `params` are dropped so the wire body
/// matches the legacy `send_error` default and integration tests stay byte-
/// identical. Shared by every typed handler (was duplicated per handler TU).
///
/// In test builds (`MEDKIT_STATUS_RECORDER`, set from `BUILD_TESTING`) the
/// call site is recorded so a run can report which of the ~281 error sites it
/// actually reached - see `http/detail/status_recorder.hpp`. The two extra
/// parameters default to `__builtin_FILE()` / `__builtin_LINE()`, which
/// evaluate at the *caller*, so no call site changes. A shipped gateway
/// compiles neither the parameters nor the call and pays nothing.
///
/// The body is written once, under the conditional signature, rather than as
/// two overloads sharing a helper: the extra inlining frontier that a helper
/// introduces makes GCC 13's libstdc++ report `-Wnull-dereference` false
/// positives here (15 of them, verified by compiling this file both ways).
#ifdef MEDKIT_STATUS_RECORDER
inline ErrorInfo make_error(int status, const std::string & code, std::string message, nlohmann::json params = {},
                            const char * site_file = __builtin_FILE(), int site_line = __builtin_LINE()) {
  http::detail::record_error_site(status, site_file, site_line);
#else
inline ErrorInfo make_error(int status, const std::string & code, std::string message, nlohmann::json params = {}) {
#endif
  ErrorInfo err;
  err.code = code;
  err.message = std::move(message);
  err.http_status = status;
  if (!params.is_null() && !params.empty()) {
    err.params = std::move(params);
  }
  return err;
}

/// Collapse a `validate_entity_for_route` validator error into a single
/// `ErrorInfo`: the local-error branch passes through, while the `Forwarded`
/// branch becomes the framework-internal sentinel that the RouteRegistry
/// wrapper recognises and skips error rendering for. Shared by every typed
/// handler (was duplicated per handler TU).
inline ErrorInfo flatten_validator_error(const std::variant<ErrorInfo, http::Forwarded> & err) {
  return std::visit(
      [](auto && alt) -> ErrorInfo {
        using T = std::decay_t<decltype(alt)>;
        if constexpr (std::is_same_v<T, ErrorInfo>) {
          return alt;
        } else {
          return HandlerContext::forwarded_sentinel_error();
        }
      },
      err);
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
