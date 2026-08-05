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

#include <deque>
#include <functional>
#include <httplib.h>
#include <initializer_list>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/query.hpp"
#include "ros2_medkit_gateway/http/alternate_status.hpp"
#include "ros2_medkit_gateway/http/detail/forward_response_scope.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

using HandlerFn = std::function<void(const httplib::Request &, httplib::Response &)>;

/// Which error wire shape a route renders when its typed handler returns the
/// error branch of `Result<T>`. Per-route knob on `RouteEntry`; default is
/// `kSovdGenericError` (the SOVD GenericError schema everywhere except the
/// `/auth/*` endpoints).
enum class ErrorRenderer {
  kSovdGenericError,  ///< `{"error_code","message","parameters"}` (default)
  kOAuth2Error,       ///< RFC 6749 §5.2 `{"error","error_description"}`
};

/// Availability guard for a route whose backing feature can be absent.
///
/// The framework evaluates `available` once per request from *inside* the
/// typed wrapper - after the request body has been parsed - so a malformed
/// body sent to a gated-off route still answers 400 rather than the gate's
/// status. `unavailable` is rendered through the route's `ErrorRenderer`, so
/// the wire shape is identical to the same `ErrorInfo` returned by the handler.
struct RouteGate {
  /// Re-evaluated per request: the manager backing a feature can appear after
  /// the route is registered, so a value snapshotted at registration would be
  /// permanently wrong.
  std::function<bool()> available;
  /// Returned when `available()` is false.
  ErrorInfo unavailable;
};

/// Shared handle to a route's gate. An empty optional means "no gate". Held by
/// shared_ptr for the same reason as `ErrorRenderer`: the typed wrapper closure
/// captures the handle when the route is registered and must observe a
/// `.gated_on(...)` applied to the returned `RouteEntry` afterwards.
using GateHandle = std::shared_ptr<std::optional<RouteGate>>;

/// One response header a route publishes, as an OpenAPI Header Object.
///
/// Response headers are optional by definition in OpenAPI (there is no
/// `required` on the emitted object here), which matches how the gateway sets
/// them: `Content-Disposition` only when the download names a file,
/// `Accept-Ranges` only when the provider is range-capable.
struct ResponseHeader {
  /// Header field name as it appears on the wire, e.g. `Location`.
  std::string name;
  /// Prose a generated client shows for the header.
  std::string description;
  /// OpenAPI schema for the value. Defaults to a plain string, which is what
  /// every header the gateway sets is.
  nlohmann::json schema{{"type", "string"}};
};

/// Fluent builder for a single route entry.
class RouteEntry {
 public:
  RouteEntry & tag(const std::string & t);
  RouteEntry & summary(const std::string & s);
  RouteEntry & description(const std::string & desc);
  RouteEntry & response(int status_code, const std::string & desc);
  RouteEntry & response(int status_code, const std::string & desc, const nlohmann::json & schema);

  /// Declare a response whose body is **not** JSON.
  ///
  /// Each media type becomes its own `content` entry with an empty Media Type
  /// Object - deliberately **no schema**. Two independent reasons:
  /// `{"type":"string","format":"binary"}` is an OpenAPI 3.0 idiom that 3.1
  /// dropped (3.1 aligned with JSON Schema 2020-12, where `format: binary` has
  /// no meaning and a `string` type actively misdescribes raw bytes); and the
  /// SSE routes emit three different frame shapes, so one schema here would be
  /// wrong for at least two of them.
  ///
  /// `schema` is accepted for signature symmetry with the JSON overloads and
  /// must be empty; a non-empty schema is recorded and reported by
  /// `validate_completeness()` rather than published against a media type it
  /// may not describe.
  RouteEntry & response(int status_code, const std::string & desc, const nlohmann::json & schema,
                        const std::vector<std::string> & content_types);

  RouteEntry & request_body(const std::string & desc, const nlohmann::json & schema,
                            const std::string & content_type = "application/json");

  /// Typed response: the schema is a $ref to the DTO's component schema.
  template <class T>
  RouteEntry & response(int status_code, const std::string & desc) {
    return response(status_code, desc,
                    nlohmann::json{{"$ref", "#/components/schemas/" + std::string(dto::dto_name<T>)}});
  }

  /// Typed request body: the schema is a $ref to the DTO's component schema.
  template <class T>
  RouteEntry & request_body(const std::string & desc) {
    return request_body(desc, nlohmann::json{{"$ref", "#/components/schemas/" + std::string(dto::dto_name<T>)}});
  }

  RouteEntry & path_param(const std::string & name, const std::string & desc);
  RouteEntry & query_param(const std::string & name, const std::string & desc, const std::string & type = "string");

  /// Appends a pre-built array of OpenAPI query parameter objects to this route.
  RouteEntry & add_query_parameters(const nlohmann::json & params);

  /// Typed query parameters: declares every member of the query DTO `T` as an
  /// `in: query` parameter, derived from the same `dto_fields<T>` descriptor a
  /// handler reads via `TypedRequest::query<T>()`. The declared parameters and
  /// the parsed object cannot drift - both come from one descriptor.
  template <class T>
  RouteEntry & query() {
    return add_query_parameters(dto::QueryParamWriter<T>::parameters());
  }
  RouteEntry & header_param(const std::string & name, const std::string & desc, bool required = true,
                            const nlohmann::json & schema = {{"type", "string"}});
  RouteEntry & deprecated();
  RouteEntry & operation_id(const std::string & id);

  /// Mark this route as returning one of several alternative success bodies,
  /// emitted as `x-medkit-alternates: true` on the operation. Set by the
  /// `post_alternates` / `del_alternates` helpers, so a route carries the
  /// marker exactly when its handler returns a `std::variant` - the document
  /// contract test uses it to tell a genuine multi-2xx operation from a route
  /// that declares a status it can never return.
  RouteEntry & mark_alternates();

  /// Mark this route as answering 206 to a `Range` request, emitted as
  /// `x-medkit-partial-content: true`. Set by the `binary_download` helper.
  ///
  /// This is the second - and only other - legitimate source of a multi-2xx
  /// operation, and it is deliberately a different marker from
  /// `mark_alternates()`: there the handler chooses between variant members,
  /// here the handler returns one thing and cpp-httplib turns it into 200 or
  /// 206 depending on the request. Collapsing the two would let the document
  /// contract test wave through a route that declares a status it can never
  /// return. Nothing outside `binary_download` may set this.
  RouteEntry & mark_partial_content();

  /// Author the prose published for this route's success response(s), leaving
  /// the status and the schema derived from the handler's return type. Use it
  /// instead of a hand-attached `response(201, "Trigger created", ref(...))`:
  /// restating the status at the call site is what let the document declare a
  /// status the handler could not return.
  ///
  /// Applies to every declared 2xx, which for a derived route is the single
  /// one `TResponse` produced.
  RouteEntry & success_description(const std::string & desc);

  /// Declare a response header this route sets on `status_code`.
  ///
  /// The status must already be declared - it comes from the handler's return
  /// type for every derived route - because a header is a property of a
  /// response, not a response of its own. Attaching one to an undeclared
  /// status would otherwise mint a description-less response object and put a
  /// status in the document the handler cannot return; instead the call is
  /// dropped and reported by `validate_completeness()`.
  ///
  /// Re-declaring the same header name on the same status replaces it, so the
  /// framework's automatic `Location` declaration can be overridden with
  /// route-specific prose.
  RouteEntry & response_header(int status_code, ResponseHeader header);

  /// Declare additional error statuses this route can emit. Statuses below 400
  /// are ignored and reported by validate_completeness() - use response() for
  /// success and redirect statuses.
  RouteEntry & errors(std::initializer_list<int> codes);

  /// Same, for a set computed at run time rather than written at the call site.
  /// Exists so a route can declare `handlers::parameter_error_statuses()` -
  /// derived by running the classifier over its whole enum - instead of a
  /// hand-copied list that goes stale when the enum grows.
  RouteEntry & errors(const std::vector<int> & codes);

  /// Declare that this route takes part in entity locking: it reads the
  /// caller's `X-Client-Id` and answers 409 when the entity's resource
  /// collection is locked by a different client.
  ///
  /// Three declarations in one call because they are one contract - the
  /// request header, the 409, and an `x-medkit-lock-guarded: true` operation
  /// extension a generated client or a contract test can select on. Splitting
  /// them is what lets a route publish two thirds of the contract.
  ///
  /// **Hand-applied, and only half-checked - do not read it as derived.** The
  /// header read that decides the 409 lives in
  /// `HandlerContext::validate_lock_access`, which 12 handlers call; a
  /// registration cannot see through that call, and the document is
  /// regenerated per `/docs` request rather than captured at registration
  /// time. Nothing therefore infers this marker from the handler: omitting the
  /// call on a new lock-checking route ships a route that can 409 without
  /// saying so, and no build or test failure follows.
  /// `test_openapi_contract.test.py::test_lock_guarded_set_matches_the_handlers`
  /// pins the marked set against a hand-maintained list, which catches the
  /// *document* drifting from that list - not the list drifting from the
  /// handlers.
  RouteEntry & lock_guarded();

  /// Declare the `X-Medkit-No-Fan-Out` request header this route reads.
  ///
  /// Presence-only: `TypedRequest::fan_out_disabled()` and the
  /// `fan_out_helpers` guards test `has_header` and never look at the value.
  /// The declared schema is therefore a bare string, not a boolean - a client
  /// sending `false` still suppresses fan-out, and a boolean schema would
  /// promise the opposite.
  RouteEntry & fan_out_aware();

  /// This route can only ever return `code`. Clears every other response and
  /// suppresses the blanket 400/404/500 injection.
  RouteEntry & only_status(int code, const std::string & desc);

  /// Guard this route with an availability predicate **and** declare the status
  /// the guard returns, in one call. A feature gate written as an inline
  /// `if (!handlers_) return tl::unexpected(...)` inside the handler lambda is
  /// invisible to the document generator; expressed here it is not.
  ///
  /// `available` is re-evaluated per request (see `RouteGate`). `unavailable`
  /// is rendered through this route's `ErrorRenderer` from inside the typed
  /// wrapper, after body parsing, so a malformed body still answers 400.
  ///
  /// The status is declared via `errors()`, which means a sub-400
  /// `unavailable.http_status` is rejected and surfaced by
  /// `validate_completeness()` rather than silently published.
  RouteEntry & gated_on(std::function<bool()> available, ErrorInfo unavailable);

  /// Hide this route from the OpenAPI spec output.
  /// The route is still registered with cpp-httplib and serves HTTP requests,
  /// but it won't appear in the generated spec or client code.
  /// Use for endpoints that always return errors (e.g., 405 Not Supported).
  RouteEntry & hidden();

  /// Override the error wire shape this route renders when a typed handler
  /// returns the error branch. Default is `kSovdGenericError`. The auth
  /// endpoints set this to `kOAuth2Error` so they emit the RFC 6749 shape.
  ///
  /// Backed by a shared_ptr so the change is visible to the typed handler
  /// wrapper closure even when called after the route is registered (the
  /// closure captures the shared_ptr by value).
  RouteEntry & error_renderer(ErrorRenderer renderer);

 private:
  friend class RouteRegistry;
  std::string method_;
  std::string path_;        // OpenAPI path (with {param} style)
  std::string regex_path_;  // cpp-httplib regex path (with capture groups)
  std::string tag_;
  std::string summary_;
  std::string description_;
  HandlerFn handler_;
  bool deprecated_{false};
  bool hidden_{false};
  /// Set by mark_alternates(); emitted as `x-medkit-alternates: true`.
  bool alternates_{false};
  /// Set by mark_partial_content(); emitted as `x-medkit-partial-content: true`.
  bool partial_content_{false};
  /// Set by lock_guarded(); emitted as `x-medkit-lock-guarded: true`.
  bool lock_guarded_{false};
  /// Set by only_status(); suppresses the blanket 400/404/500 injection.
  bool only_status_{false};
  /// Set by the *attachments* body-less typed `put<TResponse>` overload - the
  /// fire-and-forget state-machine kicks, which are the only registrations that
  /// take no payload at all. Without it validate_completeness() infers "PUT
  /// therefore a request body" from the method and reports 13 shipped routes
  /// that are correct as written - noise that would train readers to ignore the
  /// whole channel. Deliberately NOT set by the plain body-less `put` or by
  /// `post<TResponse>`: both of those parse a body by hand, so a missing
  /// declaration there is a genuine gap the check must keep reporting.
  bool takes_no_request_body_{false};
  std::string operation_id_;

  /// Heap-allocated so the typed wrapper closure can hold a stable handle to
  /// the renderer choice and observe later `.error_renderer(...)` updates.
  std::shared_ptr<ErrorRenderer> error_renderer_{std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError)};

  /// Heap-allocated for the same reason as `error_renderer_`: the typed wrapper
  /// closure captures this handle at registration time, and `.gated_on(...)` is
  /// called on the `RouteEntry` the registration returned - i.e. afterwards.
  GateHandle gate_{std::make_shared<std::optional<RouteGate>>()};

  struct ResponseInfo {
    std::string desc;
    nlohmann::json schema;
    /// NSDMI, not a bare member: the three brace-initialisations of this
    /// aggregate predate the field and must keep compiling warning-free under
    /// -Wmissing-field-initializers.
    std::vector<ResponseHeader> headers{};
    /// Media types this response's body can carry. Empty is the JSON default:
    /// the emitter writes `application/json` iff `schema` is non-empty, which
    /// is what every DTO-backed route wants and keeps a 204 content-free.
    /// Non-empty means a non-JSON body, and each entry is emitted with no
    /// schema - see the four-argument `response()` for why. NSDMI for the same
    /// aggregate-initialisation reason as `headers`.
    std::vector<std::string> content_types{};
  };
  std::map<int, ResponseInfo> responses_;

  /// Statuses whose non-JSON `response()` also carried a schema. The schema was
  /// dropped rather than published against a media type it may not describe;
  /// kept so validate_completeness() reports the miscall.
  std::vector<int> schema_on_non_json_statuses_;

  /// Statuses passed to response_header() that no response declares. Kept so
  /// validate_completeness() reports the miscall rather than the document
  /// silently losing a header the handler sets.
  std::vector<int> undeclared_header_statuses_;

  /// Error statuses declared via errors(), rendered as GenericError $refs
  /// alongside the blanket 400/404/500 set.
  std::vector<int> declared_errors_;
  /// Non-error statuses passed to errors() and therefore ignored. Kept so
  /// validate_completeness() can report the miscall instead of it passing
  /// silently.
  std::vector<int> rejected_error_codes_;

  struct RequestBodyInfo {
    std::string desc;
    nlohmann::json schema;
    std::string content_type;
  };
  std::optional<RequestBodyInfo> request_body_;

  std::vector<nlohmann::json> parameters_;
};

/// Validation issue found by validate_completeness().
struct ValidationIssue {
  enum class Severity { kError, kWarning };
  Severity severity;
  std::string route;    // e.g., "GET /apps/{app_id}/data"
  std::string message;  // e.g., "Missing response schema for 200"
};

/// Central registry: single source of truth for routes + OpenAPI metadata.
/// Routes registered here are both served via cpp-httplib AND documented in OpenAPI.
class RouteRegistry {
 public:
  // ---------------------------------------------------------------------------
  // Typed overloads.
  //
  // Each typed overload takes a handler with signature
  // `http::Result<TResponse>(http::TypedRequest)` (POST/PUT/PATCH variants also
  // take a parsed `TBody`). The framework:
  //   * generates the cpp-httplib HandlerFn (request parsing + response writing
  //     via the http::detail primitives),
  //   * auto-populates OpenAPI metadata (`response<T>(200,"")` and
  //     `request_body<TB>("")`) from the template parameters so handler call
  //     sites only need to set tag / summary / extra responses.
  //
  // Two flavours per method:
  //   * `T`-only:  handler returns `Result<T>` -> 200 + JSON body.
  //   * Pair:      handler returns `Result<std::pair<T, ResponseAttachments>>`
  //                so the handler can override status / append headers
  //                (201+Location, 204+X-Medkit-Local-Only, ...).
  // ---------------------------------------------------------------------------

  /// Typed GET that returns `Result<T>` -> 200 + JSON body.
  template <class TResponse>
  RouteEntry & get(const std::string & openapi_path,
                   std::function<http::Result<TResponse>(http::TypedRequest)> handler);

  /// Typed GET that returns `Result<std::pair<T, ResponseAttachments>>` so the
  /// handler can override status code / append headers per call.
  template <class TResponse>
  RouteEntry &
  get(const std::string & openapi_path,
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler);

  /// Typed POST: parses TBody, returns `Result<TResponse>`.
  template <class TBody, class TResponse>
  RouteEntry & post(const std::string & openapi_path,
                    std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler);

  /// Typed POST with attachments.
  template <class TBody, class TResponse>
  RouteEntry &
  post(const std::string & openapi_path,
       std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler);

  /// Body-less typed POST: returns `Result<TResponse>`. The framework neither
  /// parses nor declares a request body; the handler reads `req.body` directly
  /// via the framework escape hatch when it needs to handle non-JSON wire
  /// formats (e.g. RFC 6749 `application/x-www-form-urlencoded` on `/auth/*`).
  /// Call sites should attach an explicit `.request_body(...)` schema on the
  /// returned RouteEntry so the OpenAPI spec still documents the payload.
  template <class TResponse>
  RouteEntry & post(const std::string & openapi_path,
                    std::function<http::Result<TResponse>(http::TypedRequest)> handler);

  /// Body-less typed POST with attachments.
  template <class TResponse>
  RouteEntry &
  post(const std::string & openapi_path,
       std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler);

  /// Typed PUT.
  template <class TBody, class TResponse>
  RouteEntry & put(const std::string & openapi_path,
                   std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler);

  /// Typed PUT with attachments.
  template <class TBody, class TResponse>
  RouteEntry &
  put(const std::string & openapi_path,
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler);

  /// Body-less typed PUT: returns `Result<TResponse>`. The framework neither
  /// parses nor declares a request body; reserved for routes that legitimately
  /// take no payload (e.g. PUT /updates/{id}/prepare which is a fire-and-forget
  /// state-machine kick). Call sites should attach an explicit
  /// `.response(...)` for any non-default status the attachments variant emits.
  template <class TResponse>
  RouteEntry & put(const std::string & openapi_path,
                   std::function<http::Result<TResponse>(http::TypedRequest)> handler);

  /// Body-less typed PUT with attachments. Lets the handler emit 202 + Location
  /// (the async-job convention) without re-introducing a httplib::Response &.
  template <class TResponse>
  RouteEntry &
  put(const std::string & openapi_path,
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler);

  /// Typed PATCH.
  template <class TBody, class TResponse>
  RouteEntry & patch(const std::string & openapi_path,
                     std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler);

  /// Typed PATCH with attachments.
  template <class TBody, class TResponse>
  RouteEntry & patch(
      const std::string & openapi_path,
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler);

  /// Typed DELETE returning `Result<T>`.
  template <class TResponse>
  RouteEntry & del(const std::string & openapi_path,
                   std::function<http::Result<TResponse>(http::TypedRequest)> handler);

  /// Typed DELETE with attachments.
  template <class TResponse>
  RouteEntry &
  del(const std::string & openapi_path,
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler);

  /// Typed POST returning one of several alternates via `std::variant<TAlt...>`.
  /// The status per alternative is looked up via `http::dto_alternate_status<T>`
  /// (default 200; specialize per type, e.g. NoContent -> 204, Accepted -> 202).
  /// Schema: every alternate is registered under its `dto_name<T>` $ref at the
  /// corresponding status; the body is serialized via `JsonWriter<T>`.
  template <class TBody, class... TAlt>
  RouteEntry & post_alternates(const std::string & openapi_path,
                               std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest, TBody)> handler);

  /// Typed POST alternates with ResponseAttachments. Combines the per-alternative
  /// status dispatch of `post_alternates` with the headers/status-override
  /// channel of the pair-returning POST overloads. The framework picks the
  /// status from `dto_alternate_status<active_alternative>` first, then applies
  /// the attachments (which may further override the status and always append
  /// headers, e.g. a `Location` header for the 202 async branch).
  template <class TBody, class... TAlt>
  RouteEntry & post_alternates(const std::string & openapi_path,
                               std::function<http::Result<std::pair<std::variant<TAlt...>, http::ResponseAttachments>>(
                                   http::TypedRequest, TBody)>
                                   handler);

  /// Typed DELETE returning one of several alternates.
  template <class... TAlt>
  RouteEntry & del_alternates(const std::string & openapi_path,
                              std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest)> handler);

  // ---------------------------------------------------------------------------
  // Named escape hatches for non-DTO endpoints.
  //
  // These do not participate in DTO schema generation; they appear in the
  // OpenAPI spec via whatever metadata the caller attaches (the registry sets
  // no schema, only the route entry). Reserved for routes whose payload is
  // genuinely non-DTO (SSE, binary blobs, multipart, static assets, docs).
  // ---------------------------------------------------------------------------

  /// Register a Server-Sent Events stream route. The factory is invoked per
  /// request and returns an `SseStream` whose `next_event` callback the
  /// framework drives via cpp-httplib's chunked content provider.
  RouteEntry & sse(const std::string & openapi_path,
                   std::function<http::Result<http::SseStream>(http::TypedRequest)> stream_factory);

  /// Register a binary download (range-aware where the provider supports it).
  ///
  /// `media_types` is what the route publishes as the response body's possible
  /// media types, and it is a required argument rather than a default so a new
  /// download route cannot inherit somebody else's answer. It must cover every
  /// value the handler can put in `BinaryResponse::content_type`; where that
  /// set is open (a store that echoes a client-supplied type) the list carries
  /// `*/*` as its catch-all alongside the types that *are* derivable.
  ///
  /// The helper owns the whole Range contract - 200, 206, `Accept-Ranges`,
  /// `Content-Range`, the `Range` parameter, and the `x-medkit-partial-content`
  /// marker - because cpp-httplib produces the 206 from the request, not from
  /// the handler. Declaring any part of it at a call site would let a route
  /// publish some of the contract and not the rest.
  RouteEntry & binary_download(const std::string & openapi_path,
                               std::function<http::Result<http::BinaryResponse>(http::TypedRequest)> handler,
                               const std::vector<std::string> & media_types);

  /// Register a `multipart/form-data` upload endpoint. The handler receives
  /// the typed request plus the parsed multipart body, and returns a typed
  /// response with attachments (typically 201 Created + Location).
  template <class TResponse>
  RouteEntry &
  multipart_upload(const std::string & openapi_path,
                   std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest,
                                                                                               http::MultipartBody)>
                       handler);

  /// Register a static-asset endpoint (HTML / JS / CSS bundled into the binary).
  RouteEntry & static_asset(const std::string & openapi_path,
                            std::function<http::Result<http::StaticAsset>(http::TypedRequest)> handler);

  /// Register the OpenAPI JSON endpoint at the given path. The spec body is
  /// supplied by the caller (typically a closure over the gateway's
  /// `OpenApiSpecBuilder`).
  RouteEntry & docs_endpoint(const std::string & openapi_path,
                             std::function<http::Result<nlohmann::json>(http::TypedRequest)> handler);

  /// Register a catch-all docs route via cpp-httplib regex (used for Swagger
  /// UI subtree where the path arguments are not fixed). The route is hidden
  /// from the OpenAPI spec.
  RouteEntry & docs_subtree(const std::string & regex_pattern, HandlerFn handler);

  // ---------------------------------------------------------------------------
  // Registry-level operations.
  // ---------------------------------------------------------------------------

  /// Register all routes with cpp-httplib server.
  void register_all(httplib::Server & server, const std::string & api_prefix) const;

  /// Generate OpenAPI paths object from all registered routes.
  nlohmann::json to_openapi_paths() const;

  /// Get all unique tags (for OpenAPI tags section).
  std::vector<std::string> tags() const;

  /// Generate endpoint list for handle_root (e.g., "GET /api/v1/health").
  std::vector<std::string> to_endpoint_list(const std::string & api_prefix) const;

  /// Validate that all routes have required OpenAPI metadata.
  /// Returns issues found. Errors indicate missing required metadata,
  /// warnings indicate missing optional metadata.
  std::vector<ValidationIssue> validate_completeness() const;

  /// Number of registered routes.
  size_t size() const {
    return routes_.size();
  }

  /// Set whether authentication is enabled (controls 401/403 in OpenAPI output).
  void set_auth_enabled(bool enabled) {
    auth_enabled_ = enabled;
  }

  /// Set whether rate limiting is enabled (controls 429 in OpenAPI output).
  /// The limiter runs pre-routing on every non-OPTIONS request, so when it is
  /// on, 429 is reachable on every route and the document has to say so.
  void set_rate_limit_enabled(bool enabled) {
    rate_limit_enabled_ = enabled;
  }

  /// Set whether peer aggregation is enabled (controls 502/503 in OpenAPI
  /// output). `aggregation.enabled` defaults false and the AggregationManager
  /// is only constructed when it is set, so with aggregation off an entity can
  /// never be remote and neither status is reachable. When it is on, any route
  /// that resolves an entity may find that entity owned by a peer and answer
  /// with the gateway's own peer-failure statuses instead of the handler's -
  /// see `entity_scoped()` for which routes those are.
  void set_aggregation_enabled(bool enabled) {
    aggregation_enabled_ = enabled;
  }

  /// Escape hatch for JSON routes without typed DTOs (e.g. the fault-trigger
  /// CRUD): registers a raw cpp-httplib handler under an OpenAPI-style path so
  /// the route shows up in the generated spec, Swagger UI and the endpoint
  /// list. The caller documents request/response schemas manually via the
  /// fluent builder. Prefer the typed get<T>/post<T> API for new routes.
  RouteEntry & raw(const std::string & method, const std::string & openapi_path, HandlerFn handler) {
    return add_route(method, openapi_path, std::move(handler));
  }

 private:
  /// Convert OpenAPI path to cpp-httplib regex path.
  /// e.g., "/apps/{app_id}/data/{data_id}" -> "/apps/([^/]+)/data/(.+)"
  static std::string to_regex_path(const std::string & openapi_path, const std::string & method);

  RouteEntry & add_route(const std::string & method, const std::string & openapi_path, HandlerFn handler);

  /// Variant of add_route() for routes whose `regex_path_` is supplied
  /// directly (escape hatches whose URI cannot be derived from an
  /// OpenAPI-style path - e.g. `docs_subtree` catch-alls).
  RouteEntry & add_raw_route(const std::string & method, const std::string & openapi_path,
                             const std::string & regex_path, HandlerFn handler);

  std::deque<RouteEntry> routes_;
  bool auth_enabled_{false};
  bool rate_limit_enabled_{false};
  bool aggregation_enabled_{false};

  /// True when `openapi_path` carries one of the four entity-id path
  /// parameters, which is exactly the condition under which a handler can call
  /// `HandlerContext::validate_entity_for_route` and therefore hit the
  /// peer-forwarding branch. Derived from the path rather than from a list:
  /// the entity id has to come from the path for the lookup to happen at all,
  /// so there is no route that forwards without one.
  static bool entity_scoped(const std::string & openapi_path);

  // ---------------------------------------------------------------------------
  // Typed-handler wrapper helpers.
  //
  // These translate a typed `Result<T>(TypedRequest [, TBody])` lambda into a
  // raw cpp-httplib `HandlerFn` that:
  //   1. parses TBody (POST/PUT/PATCH) via `JsonReader<TBody>` -> 400 on failure,
  //   2. wraps the request in `http::TypedRequest` and invokes the user lambda,
  //   3. on success, serializes via `JsonWriter<TResponse>` and writes the body
  //      via `http::detail::write_json_body` (applying any attachments),
  //   4. on error, writes via `write_generic_error` or `write_oauth2_error`
  //      according to the route's current `error_renderer_` choice (read
  //      through the shared_ptr captured by the closure).
  //
  // Defined inline (templates) right below the class.
  // ---------------------------------------------------------------------------

  /// Append a parsed-body failure response (400 invalid-request).
  static ErrorInfo make_body_parse_error(const std::vector<dto::FieldError> & errs);

  /// Apply ResponseAttachments to a cpp-httplib response (status override,
  /// header append). Called by the pair-returning wrappers after the body has
  /// been written.
  static void apply_attachments(httplib::Response & res, const http::ResponseAttachments & att);

  /// Write the success body for a typed handler. NoContent specialization
  /// produces an empty body + 204 status.
  template <class TResponse>
  static void write_success_body(httplib::Response & res, const TResponse & value, int status);

  /// Write a typed error using the renderer pointed to by `renderer_ptr`.
  static void write_typed_error(httplib::Response & res, const ErrorInfo & err,
                                const std::shared_ptr<ErrorRenderer> & renderer_ptr);

  /// Evaluate a route's gate. When the route is gated off this renders the
  /// gate's `ErrorInfo` through `renderer` and returns true, meaning the
  /// handler must not run. Every typed wrapper calls it at the point the
  /// hand-written `if (!handlers_)` guard used to sit: after the request body
  /// has been parsed, so a malformed body still answers 400.
  static bool gate_blocked(httplib::Response & res, const GateHandle & gate,
                           const std::shared_ptr<ErrorRenderer> & renderer);

  /// Build the body-less typed HandlerFn (GET/DELETE/SSE-factory-style).
  template <class TResponse>
  static HandlerFn wrap_body_less(std::function<http::Result<TResponse>(http::TypedRequest)> handler,
                                  std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the body-less typed HandlerFn whose return type is
  /// `Result<pair<T, ResponseAttachments>>`.
  template <class TResponse>
  static HandlerFn wrap_body_less_with_attachments(
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler,
      std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the body-bearing typed HandlerFn.
  template <class TBody, class TResponse>
  static HandlerFn wrap_with_body(std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler,
                                  std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the body-bearing typed HandlerFn whose return type carries
  /// ResponseAttachments.
  template <class TBody, class TResponse>
  static HandlerFn wrap_with_body_attachments(
      std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler,
      std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the alternates-returning HandlerFn (POST flavour).
  template <class TBody, class... TAlt>
  static HandlerFn
  wrap_post_alternates(std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest, TBody)> handler,
                       std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the alternates+attachments HandlerFn (POST flavour). The active
  /// alternative drives the default status via `dto_alternate_status`; the
  /// `ResponseAttachments` companion appends headers and may further override
  /// the status (handlers attach `Location` here on the 202 async branch).
  template <class TBody, class... TAlt>
  static HandlerFn wrap_post_alternates_with_attachments(
      std::function<http::Result<std::pair<std::variant<TAlt...>, http::ResponseAttachments>>(http::TypedRequest,
                                                                                              TBody)>
          handler,
      std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);

  /// Build the alternates-returning HandlerFn (DELETE flavour).
  template <class... TAlt>
  static HandlerFn wrap_del_alternates(std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest)> handler,
                                       std::shared_ptr<ErrorRenderer> renderer, GateHandle gate);
};

// =============================================================================
// Template implementations
// =============================================================================

namespace detail {

/// Decode TBody from the request body JSON, returning either the parsed value
/// or a 400 ErrorInfo derived from the FieldError list.
template <class TBody>
inline tl::expected<TBody, ErrorInfo> parse_request_body(const httplib::Request & req) {
  static_assert(dto::is_dto_v<TBody>, "RouteRegistry typed body must be a DTO (dto_fields<T> specialization required)");
  nlohmann::json parsed;
  try {
    if (req.body.empty()) {
      parsed = nlohmann::json::object();
    } else {
      parsed = nlohmann::json::parse(req.body);
    }
  } catch (const nlohmann::json::parse_error & e) {
    ErrorInfo info;
    info.code = ERR_INVALID_REQUEST;
    info.message = std::string("Malformed JSON: ") + e.what();
    info.http_status = 400;
    return tl::make_unexpected(std::move(info));
  }
  auto out = dto::JsonReader<TBody>::read(parsed);
  if (out.has_value()) {
    return out.value();
  }
  // Collapse FieldErrors into a single GenericError with `parameters`
  // containing a `fields` array, matching the SOVD invalid-request convention.
  ErrorInfo info;
  info.code = ERR_INVALID_REQUEST;
  info.message = "Request body validation failed";
  info.http_status = 400;
  nlohmann::json fields = nlohmann::json::array();
  for (const auto & e : out.error()) {
    fields.push_back({{"field", e.field}, {"message", e.message}});
  }
  info.params = nlohmann::json::object();
  info.params["fields"] = std::move(fields);
  return tl::make_unexpected(std::move(info));
}

/// Default prose for a derived success status. Generated clients surface this
/// text, so it must read correctly on its own: the status now comes from the
/// handler's return type, and a route that returns `Accepted<NoContent>` must
/// not be published as "No content" just because it has no body. Call sites
/// that want something specific ("Trigger created") say so via
/// `RouteEntry::success_description`, which never restates the status.
inline const char * default_success_description(int status) {
  switch (status) {
    case 201:
      return "Created";
    case 202:
      return "Accepted";
    case 204:
      return "No content";
    default:
      return "Successful response";
  }
}

/// True when `TResponse` fixes a status whose declared response carries a
/// `Location` header - i.e. exactly the statuses `declare_location_header`
/// publishes one for.
///
/// The obligation and the means to meet it live in different places: the
/// status comes from the return type, but only the `ResponseAttachments`
/// overloads give a handler any way to set a header. This trait is what lets
/// the non-attachments overloads reject the combination at compile time
/// instead of shipping a route that advertises a header it cannot send.
template <class TResponse>
inline constexpr bool kStatusRequiresAttachments =
    http::dto_alternate_status<TResponse>::value == 201 || http::dto_alternate_status<TResponse>::value == 202;

/// Declare the `Location` header a 201 / 202 answer carries, deriving the fact
/// that it carries one from the status the return type already fixed.
///
/// RFC 9110 §15.3.2: a 201 identifies the resource it created with `Location`.
/// The gateway's 202 answers apply the same convention to the resource whose
/// status the client polls (`/updates/{id}/status`, `/{entity}/status`, an
/// execution). Every handler behind a `Created<T>` / `Accepted<T>` return type
/// sets the header, so declaring it here - rather than at each registration -
/// is what keeps the two from drifting apart one route at a time.
inline void declare_location_header(RouteEntry & entry, int status) {
  if (status != 201 && status != 202) {
    return;
  }
  entry.response_header(
      status,
      ResponseHeader{"Location",
                     status == 201
                         ? "Absolute path of the created resource, API prefix included (`/api/v1/...`)."
                         : "Absolute path of the resource whose status tracks this request, API prefix included.",
                     nlohmann::json{{"type", "string"}, {"format", "uri-reference"}}});
}

/// Declare the one success response a typed route derives from `TResponse`:
/// status from `dto_alternate_status`, schema from `status_payload_t`, prose
/// from the status, and the `Location` header when the status implies one.
///
/// Every typed registration entry point funnels through here so the derivation
/// exists once. A hand-rolled copy at a registration site is how a route ends
/// up with a status and a document that disagree.
template <class TResponse>
inline void declare_derived_response(RouteEntry & entry) {
  constexpr int status = http::dto_alternate_status<TResponse>::value;
  using Payload = http::status_payload_t<TResponse>;
  if constexpr (std::is_same_v<Payload, http::NoContent>) {
    entry.response(status, default_success_description(status));
  } else {
    entry.template response<Payload>(status, default_success_description(status));
  }
  declare_location_header(entry, status);
}

}  // namespace detail

template <class TResponse>
void RouteRegistry::write_success_body(httplib::Response & res, const TResponse & value, int status) {
  // TResponse may be a status wrapper (http::Created<T> / http::Accepted<T>).
  // The wire shape comes from the payload; the default status comes from the
  // wrapper. Callers that pass status == 0 rely on that default, so it must
  // not be a literal here or a wrapped response silently downgrades to 200.
  using Payload = http::status_payload_t<TResponse>;
  const int default_status = http::dto_alternate_status<TResponse>::value;
  if constexpr (std::is_same_v<Payload, http::NoContent>) {
    res.status = (status == 0) ? default_status : status;
    // No body: 204, and 202 for an accepted asynchronous transition.
    res.body.clear();
  } else if constexpr (std::is_same_v<Payload, nlohmann::json>) {
    // Raw JSON escape hatch (docs_endpoint).
    http::detail::write_json_body(http::detail::FrameworkOrPluginAccess{}, res, http::status_body(value),
                                  status == 0 ? default_status : status);
  } else {
    static_assert(dto::has_dto_shape_v<Payload>,
                  "RouteRegistry typed response must be a DTO (regular or opaque), NoContent, "
                  "or nlohmann::json (escape hatch), optionally wrapped in Created<>/Accepted<>");
    auto body = dto::JsonWriter<Payload>::write(http::status_body(value));
    http::detail::write_json_body(http::detail::FrameworkOrPluginAccess{}, res, body,
                                  status == 0 ? default_status : status);
  }
}

template <class TResponse>
HandlerFn RouteRegistry::wrap_body_less(std::function<http::Result<TResponse>(http::TypedRequest)> handler,
                                        std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // The forwarding scope makes the typed `validate_entity_for_route`
    // overload able to stream the proxied response body to `res` when an
    // entity is owned by a remote peer. Handlers never see the response, so
    // the framework installs the channel around the handler invocation.
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (outcome.has_value()) {
      write_success_body<TResponse>(res, outcome.value(), 0);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class TResponse>
HandlerFn RouteRegistry::wrap_body_less_with_attachments(
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler,
    std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (outcome.has_value()) {
      const auto & att = outcome.value().second;
      // Default status comes from the return type (Created<T> -> 201,
      // Accepted<T> -> 202, NoContent -> 204, plain DTO -> 200), never from a
      // literal, so a wrapped response cannot silently downgrade to 200.
      int status = att.status_override.value_or(http::dto_alternate_status<TResponse>::value);
      write_success_body<TResponse>(res, outcome.value().first, status);
      apply_attachments(res, att);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class TBody, class TResponse>
HandlerFn RouteRegistry::wrap_with_body(std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler,
                                        std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope: lets the typed validate_entity_for_route stream a
    // proxied response when the entity is owned by a remote peer (see
    // wrap_body_less). Without it, remote-entity writes return Forwarded with
    // no sink and the client gets an empty body instead of the peer response.
    http::detail::ForwardResponseScope forward_scope(&res);
    auto body = detail::parse_request_body<TBody>(req);
    if (!body.has_value()) {
      write_typed_error(res, body.error(), renderer);
      return;
    }
    // Deliberately after the body parse: a malformed payload sent to a route
    // whose feature is off is still a malformed payload, and answering 501
    // there would hide the client's own bug.
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req, std::move(body.value()));
    if (outcome.has_value()) {
      write_success_body<TResponse>(res, outcome.value(), 0);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class TBody, class TResponse>
HandlerFn RouteRegistry::wrap_with_body_attachments(
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler,
    std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope for remote-peer entities (see wrap_body_less / wrap_with_body).
    http::detail::ForwardResponseScope forward_scope(&res);
    auto body = detail::parse_request_body<TBody>(req);
    if (!body.has_value()) {
      write_typed_error(res, body.error(), renderer);
      return;
    }
    // Gate after the body parse (see wrap_with_body).
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req, std::move(body.value()));
    if (outcome.has_value()) {
      const auto & att = outcome.value().second;
      // Default status comes from the return type (Created<T> -> 201,
      // Accepted<T> -> 202, NoContent -> 204, plain DTO -> 200), never from a
      // literal, so a wrapped response cannot silently downgrade to 200.
      int status = att.status_override.value_or(http::dto_alternate_status<TResponse>::value);
      write_success_body<TResponse>(res, outcome.value().first, status);
      apply_attachments(res, att);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class TBody, class... TAlt>
HandlerFn RouteRegistry::wrap_post_alternates(
    std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest, TBody)> handler,
    std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope for remote-peer entities (see wrap_body_less / wrap_with_body).
    http::detail::ForwardResponseScope forward_scope(&res);
    auto body = detail::parse_request_body<TBody>(req);
    if (!body.has_value()) {
      write_typed_error(res, body.error(), renderer);
      return;
    }
    // Gate after the body parse (see wrap_with_body).
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req, std::move(body.value()));
    if (outcome.has_value()) {
      std::visit(
          [&res](const auto & alt) {
            using AltT = std::decay_t<decltype(alt)>;
            constexpr int status = http::dto_alternate_status<AltT>::value;
            write_success_body<AltT>(res, alt, status);
          },
          outcome.value());
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class TBody, class... TAlt>
HandlerFn RouteRegistry::wrap_post_alternates_with_attachments(
    std::function<http::Result<std::pair<std::variant<TAlt...>, http::ResponseAttachments>>(http::TypedRequest, TBody)>
        handler,
    std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope for remote-peer entities (see wrap_body_less / wrap_with_body).
    http::detail::ForwardResponseScope forward_scope(&res);
    auto body = detail::parse_request_body<TBody>(req);
    if (!body.has_value()) {
      write_typed_error(res, body.error(), renderer);
      return;
    }
    // Gate after the body parse (see wrap_with_body).
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req, std::move(body.value()));
    if (outcome.has_value()) {
      const auto & att = outcome.value().second;
      std::visit(
          [&res, &att](const auto & alt) {
            using AltT = std::decay_t<decltype(alt)>;
            constexpr int default_status = http::dto_alternate_status<AltT>::value;
            int status = att.status_override.value_or(default_status);
            write_success_body<AltT>(res, alt, status);
          },
          outcome.value().first);
      apply_attachments(res, att);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

template <class... TAlt>
HandlerFn
RouteRegistry::wrap_del_alternates(std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest)> handler,
                                   std::shared_ptr<ErrorRenderer> renderer, GateHandle gate) {
  return [handler = std::move(handler), renderer = std::move(renderer),
          gate = std::move(gate)](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope for remote-peer entities (see wrap_body_less / wrap_with_body).
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (outcome.has_value()) {
      std::visit(
          [&res](const auto & alt) {
            using AltT = std::decay_t<decltype(alt)>;
            constexpr int status = http::dto_alternate_status<AltT>::value;
            write_success_body<AltT>(res, alt, status);
          },
          outcome.value());
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
}

// -----------------------------------------------------------------------------
// Typed registration entry points.
// -----------------------------------------------------------------------------

template <class TResponse>
RouteEntry & RouteRegistry::get(const std::string & openapi_path,
                                std::function<http::Result<TResponse>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed get<T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("get", openapi_path, /*placeholder*/ HandlerFn{});
  entry.handler_ = wrap_body_less<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::get(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed get<T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("get", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less_with_attachments<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::post(const std::string & openapi_path,
                                 std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed post<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed post<TB,T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::post(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed post<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed post<TB,T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body_attachments<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::post(const std::string & openapi_path,
                                 std::function<http::Result<TResponse>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed post<T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  // No automatic request_body schema: body-less typed POST is reserved for
  // routes that parse the body manually (e.g. form-urlencoded auth endpoints).
  // Callers attach an explicit `.request_body(...)` to populate the OpenAPI
  // spec.
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::post(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed post<T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less_with_attachments<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::put(const std::string & openapi_path,
                                std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed put<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed put<TB,T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("put", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::put(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed put<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed put<TB,T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("put", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body_attachments<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::put(const std::string & openapi_path,
                                std::function<http::Result<TResponse>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed put<T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("put", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  // No automatic request_body schema, and deliberately NOT exempt from the
  // completeness check: this overload's caller reads the body itself
  // (`PUT /{entity}/data/{data_id}` parses free-form JSON by hand so
  // plugin-owned entities can accept shapes `DataWriteRequest` does not
  // describe), exactly like the body-less typed POST. A route here that omits
  // `.request_body(...)` has a real gap in its document, so it must still be
  // reported. The payload-free routes live on the attachments overload below.
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::put(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed put<T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("put", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less_with_attachments<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  // This is the payload-free shape: a fire-and-forget state-machine kick that
  // answers 202 with a `Location` (`/updates/{id}/prepare`, the lifecycle
  // transitions). Recording that on the route lets validate_completeness() read
  // it instead of inferring "PUT therefore a body" from the method. The plain
  // body-less overload above is NOT exempt - its caller parses a body by hand.
  entry.takes_no_request_body_ = true;
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::patch(const std::string & openapi_path,
                                  std::function<http::Result<TResponse>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed patch<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed patch<TB,T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("patch", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TBody, class TResponse>
RouteEntry & RouteRegistry::patch(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "typed patch<TB,T>: TB must be a DTO");
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed patch<TB,T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("patch", openapi_path, HandlerFn{});
  entry.handler_ = wrap_with_body_attachments<TBody, TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::del(const std::string & openapi_path,
                                std::function<http::Result<TResponse>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed del<T>: T must be a DTO (or NoContent)");
  static_assert(!detail::kStatusRequiresAttachments<TResponse>,
                "201/202 must use the ResponseAttachments overload: the document declares a Location "
                "header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("delete", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

template <class TResponse>
RouteEntry & RouteRegistry::del(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest)> handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "typed del<T>: T must be a DTO (or NoContent)");
  auto & entry = add_route("delete", openapi_path, HandlerFn{});
  entry.handler_ = wrap_body_less_with_attachments<TResponse>(std::move(handler), entry.error_renderer_, entry.gate_);
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

namespace detail {

/// Add one alternate's response slot to the entry by `(status, $ref)`.
template <class TAlt>
inline void add_alternate_response(RouteEntry & entry) {
  constexpr int status = http::dto_alternate_status<TAlt>::value;
  if constexpr (std::is_same_v<TAlt, http::NoContent>) {
    entry.response(status, default_success_description(status));
  } else {
    static_assert(dto::has_dto_shape_v<TAlt>,
                  "alternate variant member must be a DTO (regular or opaque) or NoContent");
    entry.template response<TAlt>(status, default_success_description(status));
  }
  declare_location_header(entry, status);
}

}  // namespace detail

template <class TBody, class... TAlt>
RouteEntry &
RouteRegistry::post_alternates(const std::string & openapi_path,
                               std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest, TBody)> handler) {
  static_assert(dto::is_dto_v<TBody>, "post_alternates: TBody must be a DTO");
  static_assert((!detail::kStatusRequiresAttachments<TAlt> && ...),
                "a 201/202 alternate must use the ResponseAttachments overload: the document declares a "
                "Location header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ = wrap_post_alternates<TBody, TAlt...>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  (detail::add_alternate_response<TAlt>(entry), ...);
  entry.mark_alternates();
  return entry;
}

template <class TBody, class... TAlt>
RouteEntry & RouteRegistry::post_alternates(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<std::variant<TAlt...>, http::ResponseAttachments>>(http::TypedRequest, TBody)>
        handler) {
  static_assert(dto::is_dto_v<TBody>, "post_alternates: TBody must be a DTO");
  auto & entry = add_route("post", openapi_path, HandlerFn{});
  entry.handler_ =
      wrap_post_alternates_with_attachments<TBody, TAlt...>(std::move(handler), entry.error_renderer_, entry.gate_);
  entry.template request_body<TBody>("");
  (detail::add_alternate_response<TAlt>(entry), ...);
  entry.mark_alternates();
  return entry;
}

template <class... TAlt>
RouteEntry &
RouteRegistry::del_alternates(const std::string & openapi_path,
                              std::function<http::Result<std::variant<TAlt...>>(http::TypedRequest)> handler) {
  static_assert((!detail::kStatusRequiresAttachments<TAlt> && ...),
                "a 201/202 alternate must use the ResponseAttachments overload: the document declares a "
                "Location header for those statuses, and only that overload lets the handler send one");
  auto & entry = add_route("delete", openapi_path, HandlerFn{});
  entry.handler_ = wrap_del_alternates<TAlt...>(std::move(handler), entry.error_renderer_, entry.gate_);
  (detail::add_alternate_response<TAlt>(entry), ...);
  entry.mark_alternates();
  return entry;
}

// -----------------------------------------------------------------------------
// Multipart upload: defined inline because TResponse is templated.
// -----------------------------------------------------------------------------

template <class TResponse>
RouteEntry & RouteRegistry::multipart_upload(
    const std::string & openapi_path,
    std::function<http::Result<std::pair<TResponse, http::ResponseAttachments>>(http::TypedRequest,
                                                                                http::MultipartBody)>
        handler) {
  static_assert(dto::has_dto_shape_v<http::status_payload_t<TResponse>> ||
                    std::is_same_v<http::status_payload_t<TResponse>, http::NoContent>,
                "multipart_upload<T>: T must be a DTO (or NoContent)");
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  auto gate = std::make_shared<std::optional<RouteGate>>();
  HandlerFn fn = [handler = std::move(handler), renderer, gate](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope for remote-peer entities (see wrap_body_less / wrap_with_body).
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::MultipartBody body;
    // body.parts default-constructs empty; the loop below populates it from req.files.
    // cpp-httplib exposes parsed multipart entries via `req.files`; surface
    // them through MultipartBody.parts as MultipartFormData entries.
    // FIXME(#409): `req.files` / httplib::MultipartFormData are unavailable in
    // cpp-httplib >= 0.20; migrate to the req.form API so the system package can
    // be used on Lyrical/Resolute instead of the vendored 0.14.3 header.
    for (const auto & [name, file] : req.files) {
      httplib::MultipartFormData mp;
      mp.name = name;
      mp.filename = file.filename;
      mp.content = file.content;
      mp.content_type = file.content_type;
      body.parts.push_back(std::move(mp));
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req, std::move(body));
    if (outcome.has_value()) {
      const auto & att = outcome.value().second;
      // Default status comes from the return type (Created<T> -> 201,
      // Accepted<T> -> 202, NoContent -> 204, plain DTO -> 200), never from a
      // literal, so a wrapped response cannot silently downgrade to 200.
      int status = att.status_override.value_or(http::dto_alternate_status<TResponse>::value);
      write_success_body<TResponse>(res, outcome.value().first, status);
      apply_attachments(res, att);
      return;
    }
    write_typed_error(res, outcome.error(), renderer);
  };
  auto & entry = add_route("post", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.gate_ = gate;
  entry.request_body("Multipart upload", nlohmann::json{{"type", "object"}, {"additionalProperties", true}},
                     "multipart/form-data");
  detail::declare_derived_response<TResponse>(entry);
  return entry;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
