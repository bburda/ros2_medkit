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

// httplib-free handler-result vocabulary. Split out of typed_router.hpp so that
// the DTO/provider header chain (operation_provider.hpp -> dto/operations.hpp ->
// alternate_status.hpp) does not transitively pull <httplib.h> into downstream
// plugins. Only TypedRequest (in typed_router.hpp) depends on cpp-httplib.

#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/models/error_info.hpp"

namespace ros2_medkit_gateway {
namespace http {

/// Handler return type. The success branch carries the DTO (or marker like
/// NoContent) the handler wants to send; the error branch carries a fully
/// formed ErrorInfo the framework can serialize as a SOVD GenericError.
template <class T>
using Result = tl::expected<T, ErrorInfo>;

/// Empty success marker used by handlers that complete with no body (typically
/// DELETE -> 204 No Content). Distinct from `Result<void>` so that the
/// framework can statically dispatch the response writer.
struct NoContent {};

/// Sentinel returned by validators when the entity belongs to a remote peer
/// and the validator has already committed the wire response by proxying the
/// request to that peer. Returning Forwarded tells the caller "the response
/// has been written, do not write anything else".
struct Forwarded {};

namespace detail {
/// Variant ordering invariant: ErrorInfo must be the first alternative so a
/// default-constructed `std::variant<ErrorInfo, Forwarded>` (the error state
/// callers may inadvertently produce) denotes an "unknown error", not the
/// misleading "request already proxied" Forwarded state. If a future refactor
/// reorders the alternatives, this static_assert breaks compilation.
template <class T>
inline constexpr bool kValidatorVariantOrderingOk =
    std::is_same_v<std::variant_alternative_t<0, std::variant<ErrorInfo, Forwarded>>, ErrorInfo>;
static_assert(kValidatorVariantOrderingOk<void>, "ErrorInfo must be the first variant alternative");
}  // namespace detail

/// Return type for HandlerContext::validate_entity_for_route and similar
/// validators that may either succeed locally, fail with an ErrorInfo, or
/// short-circuit by proxying the request to a remote peer (Forwarded).
template <class T>
using ValidatorResult = tl::expected<T, std::variant<ErrorInfo, Forwarded>>;

/// Carries a response's HTTP status in its type, so the route registry derives
/// the declared status from the handler's signature and the document cannot
/// drift from the wire.
///
/// Needed where the payload type does not imply the status on its own: a
/// `dto::Trigger` is a 201 body when created and a 200 body when read, so the
/// status cannot live on the DTO. Wrapping is what distinguishes the two.
///
/// The registry never introspects the wrapper - it asks
/// `dto_alternate_status<T>` for the status and `status_payload_t<T>` for the
/// schema, the serializer and the static assertions.
template <class T>
struct Created {
  T value;
};

/// See Created. Declares 202 Accepted. `Accepted<NoContent>` is the shape for
/// an asynchronous transition that returns no body.
template <class T>
struct Accepted {
  T value;
};

/// Side-channel a handler can attach to its successful response when the
/// default "200 OK + DTO body" is not enough. Examples:
///   - 201 Created with a `Location` header for POST creating a resource.
///   - 202 Accepted for asynchronously processed requests.
///   - 207 Multi-Status for aggregated fan-out responses.
///   - Vendor headers such as `X-Medkit-Local-Only`.
struct ResponseAttachments {
  /// HTTP status to use instead of the default 200. Leave empty for 200.
  std::optional<int> status_override;
  /// Additional headers to append. Order is preserved; duplicate names are
  /// allowed (cpp-httplib delivers them as separate Set-Cookie-style headers).
  std::vector<std::pair<std::string, std::string>> headers;

  /// Fluent setter for the status override.
  ResponseAttachments & with_status(int status) {
    status_override = status;
    return *this;
  }

  /// Fluent setter that appends a header entry.
  ResponseAttachments & with_header(std::string name, std::string value) {
    headers.emplace_back(std::move(name), std::move(value));
    return *this;
  }

  /// Fluent setter for the `Location` header of a 201 / 202 response.
  ///
  /// `uri` is the absolute, API-prefixed path form every `href` in the
  /// document already uses (`/api/v1/apps/x/triggers/7`). Build it with
  /// `api_path(...)` when the handler assembles the target from parts, or pass
  /// `req.path() + "/" + id` when the new resource is a child of the request
  /// path - `TypedRequest::path()` is already prefixed. Never hand-roll the
  /// `"/api/v1/"` literal: that is what let three different spellings of the
  /// same URI accumulate across the handlers.
  ///
  /// The route registry declares this header on every derived 201 / 202, so a
  /// handler that returns `Created<T>` / `Accepted<T>` without calling this
  /// publishes a header it does not send.
  ResponseAttachments & with_location(std::string uri) {
    return with_header("Location", std::move(uri));
  }
};

}  // namespace http
}  // namespace ros2_medkit_gateway
