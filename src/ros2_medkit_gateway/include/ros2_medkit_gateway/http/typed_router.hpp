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

#include <httplib.h>

#include <optional>
#include <string>
#include <string_view>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/dto/query.hpp"
// Re-export the httplib-free handler-result vocabulary so existing includers of
// typed_router.hpp keep seeing Result/NoContent/Forwarded/ValidatorResult/
// ResponseAttachments. The markers themselves live in this leaf header, which
// the DTO/provider chain includes WITHOUT pulling <httplib.h>.
#include "ros2_medkit_gateway/http/handler_result.hpp"

namespace ros2_medkit_gateway {
namespace http {

/// Thin, type-safe wrapper around `const httplib::Request &` so handlers do
/// not see raw cpp-httplib types. A future commit will extend the constructor
/// with a RouteEntry parameter to support named path-parameter lookup; for now
/// path_param() relies on regex group ordering through the placeholder path
/// below.
class TypedRequest {
 public:
  /// Wraps the framework-provided request reference. The TypedRequest does
  /// not own the request; the request must outlive the wrapper.
  explicit TypedRequest(const httplib::Request & req) : req_(req) {
  }

  /// Returns the value of the named path parameter, or an `ErrorInfo` if the
  /// lookup fails. This method never throws.
  ///
  /// PLACEHOLDER BEHAVIOR (until commit 4 wires named-lookup via `RouteEntry`):
  /// `name` must be a base-10 unsigned integer literal interpreted as a
  /// 0-based literal index into `req.matches` (matching `std::smatch` indexing
  /// where `matches[0]` is the full match, `matches[1]` is the first capture
  /// group, etc.). Empty `name`, non-numeric `name`, or out-of-range index all
  /// produce `tl::unexpected(ErrorInfo{ERR_INVALID_PARAMETER, ..., 400, {}})`.
  /// Handlers that need a real named lookup should wait for commit 4; this
  /// method is intentionally restrictive so misuse fails loudly rather than
  /// silently returning the wrong capture group.
  tl::expected<std::string, ErrorInfo> path_param(std::string_view name) const {
    auto make_err = [](std::string message) {
      ErrorInfo info;
      info.code = ERR_INVALID_PARAMETER;
      info.message = std::move(message);
      info.http_status = 400;
      return tl::unexpected(std::move(info));
    };
    if (name.empty()) {
      return make_err("path_param: empty name");
    }
    std::size_t idx = 0;
    for (char c : name) {
      if (c < '0' || c > '9') {
        return make_err("path_param: non-numeric name '" + std::string(name) + "' (named lookup not wired yet)");
      }
      idx = idx * 10 + static_cast<std::size_t>(c - '0');
    }
    if (idx >= req_.matches.size()) {
      return make_err("path_param: index " + std::to_string(idx) + " out of range");
    }
    return req_.matches[idx].str();
  }

  /// Returns the value of the named query parameter, or std::nullopt if the
  /// request does not carry that parameter at all.
  std::optional<std::string> query_param(std::string_view name) const {
    // Note: Humble's cpp-httplib is older and only accepts const char*; Jazzy
    // accepts std::string_view too. Keep the c_str() conversion for Humble
    // compatibility (clang-tidy on Jazzy flags it as redundant - the warning
    // is suppressed at the call site).
    const std::string name_str(name);
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    if (!req_.has_param(name_str.c_str())) {
      return std::nullopt;
    }
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    return req_.get_param_value(name_str.c_str());
  }

  /// Parses the request's query string into a typed query DTO `T`, using the
  /// same `dto_fields<T>` descriptor that declares the parameters in the OpenAPI
  /// spec (via `dto::QueryParamWriter<T>`). Absent parameters leave their member
  /// at its default. This is the sanctioned way for handlers to read query
  /// parameters: a handler cannot read a parameter the route did not declare,
  /// because it can only access members of `T`.
  template <class T>
  T query() const {
    T out{};
    dto::for_each_field<T>([&](const auto & f) {
      if (auto raw = query_param(f.key)) {
        dto::assign_query_field(out.*(f.ptr), *raw);
      }
    });
    return out;
  }

  /// Returns the value of the named header, or std::nullopt if absent.
  std::optional<std::string> header(std::string_view name) const {
    // Note: see query_param() comment - Humble compatibility requires c_str().
    const std::string name_str(name);
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    if (!req_.has_header(name_str.c_str())) {
      return std::nullopt;
    }
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    return req_.get_header_value(name_str.c_str());
  }

  /// True if the client explicitly opted out of fan-out (header
  /// `X-Medkit-No-Fan-Out` present, value irrelevant).
  bool fan_out_disabled() const {
    return req_.has_header("X-Medkit-No-Fan-Out");
  }

  /// Returns the request path exactly as the client sent it, **including the
  /// `/api/v1` prefix**: routes are registered with the prefix already
  /// concatenated (`RouteRegistry::register_all`) and nothing rewrites
  /// `req.path` on the way in, so this is the absolute path of the resource
  /// being addressed.
  ///
  /// That is what makes it the right basis for a `Location` header - either
  /// verbatim (PUT, where the target IS the resource) or with the new id
  /// appended (`Location: <request-path>/<new-id>` on POST). Do NOT wrap it in
  /// `api_path()`: that would double the prefix. Routes that need to inspect
  /// path segments should use `path_param` instead.
  const std::string & path() const {
    return req_.path;
  }

  /// Framework-only escape hatch back to the raw cpp-httplib request. Do not
  /// use this from handler bodies - it exists for the routing layer and for
  /// helpers that need access to fields not yet wrapped by TypedRequest. The
  /// `[[deprecated]]` attribute is load-bearing: it fires a warning at every
  /// caller, ensuring any premature handler use is surfaced at build time.
  /// The framework's own internal use sites (none yet; commit 4 will add the
  /// first) are expected to suppress the warning locally.
  [[deprecated("framework-internal; handlers must not call this")]]
  const httplib::Request & raw_for_framework() const {
    return req_;
  }

 private:
  const httplib::Request & req_;
};

}  // namespace http
}  // namespace ros2_medkit_gateway
