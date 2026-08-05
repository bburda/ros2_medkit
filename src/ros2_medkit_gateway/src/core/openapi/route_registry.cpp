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

#include "route_registry.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/detail/forward_response_scope.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"
#include "ros2_medkit_gateway/http/detail/status_recorder.hpp"

#ifdef MEDKIT_STATUS_RECORDER
#include <mutex>
#include <tuple>
#endif

namespace ros2_medkit_gateway {

// Definition of the framework-internal thread-local forwarding sink declared
// in `forward_response_scope.hpp`. The typed router's `wrap_body_less`
// installs a ForwardResponseScope around every typed handler invocation so
// HandlerContext::validate_entity_for_route (typed overload) can write the
// proxied response body to the underlying cpp-httplib response without
// handlers ever touching it. Defined here in the core library so both
// gateway_core (where the typed wrappers are instantiated) and gateway_ros2
// (where HandlerContext lives) resolve the same storage. One definition per
// program is required by ODR even though the storage is thread-local.
namespace http {
namespace detail {
thread_local httplib::Response * tl_forward_response = nullptr;

#ifdef MEDKIT_STATUS_RECORDER
// Definitions for the test-build-only emitted-status recorder declared in
// `status_recorder.hpp`. Same placement and the same reason as the sink
// above: one definition per program, in the core library both static
// libraries resolve against. Unlike the sink, the recorder needs no
// thread-local state at all - `StatusRecordingScope` carries the route
// identity as its own member and `make_error()`'s hook is route-agnostic.
// See the header for what the recorder observes and what it structurally
// cannot.

namespace {

struct RecorderState {
  std::mutex mu;
  /// (method, OpenAPI templated path, status) actually put on the wire.
  std::set<std::tuple<std::string, std::string, int>> emitted;
  /// "file:line -> status" for every `make_error()` that ran.
  std::set<std::string> error_sites;
};

RecorderState & recorder_state() {
  // Function-local static, NOT thread_local: the state is process-wide and
  // guarded by its own mutex, so it carries none of the initial-exec TLS
  // hazard that rules out a `static thread_local` in `make_error()`.
  static RecorderState state;
  return state;
}

}  // namespace

void record_error_site(int status, const char * file, int line) {
  RecorderState & state = recorder_state();
  std::string site = (file != nullptr ? std::string(file) : std::string("<unknown>")) + ":" + std::to_string(line) +
                     " -> " + std::to_string(status);
  const std::lock_guard<std::mutex> lock(state.mu);
  state.error_sites.insert(std::move(site));
}

void record_emitted_status(const std::string & method, const std::string & path, int status) {
  RecorderState & state = recorder_state();
  const std::lock_guard<std::mutex> lock(state.mu);
  state.emitted.emplace(method, path, status);
}

nlohmann::json emitted_status_report() {
  RecorderState & state = recorder_state();
  const std::lock_guard<std::mutex> lock(state.mu);
  nlohmann::json emitted = nlohmann::json::array();
  for (const auto & [method, path, status] : state.emitted) {
    emitted.push_back({{"method", method}, {"path", path}, {"status", status}});
  }
  nlohmann::json sites = nlohmann::json::array();
  for (const auto & site : state.error_sites) {
    sites.push_back(site);
  }
  return nlohmann::json{{"emitted", std::move(emitted)}, {"error_sites", std::move(sites)}};
}
#endif  // MEDKIT_STATUS_RECORDER

}  // namespace detail
}  // namespace http

namespace openapi {

// -----------------------------------------------------------------------------
// RouteEntry fluent methods
// -----------------------------------------------------------------------------

RouteEntry & RouteEntry::tag(const std::string & t) {
  tag_ = t;
  return *this;
}

RouteEntry & RouteEntry::summary(const std::string & s) {
  summary_ = s;
  return *this;
}

RouteEntry & RouteEntry::description(const std::string & desc) {
  description_ = desc;
  return *this;
}

RouteEntry & RouteEntry::response(int status_code, const std::string & desc) {
  responses_[status_code] = {desc, {}};
  return *this;
}

RouteEntry & RouteEntry::response(int status_code, const std::string & desc, const nlohmann::json & schema) {
  responses_[status_code] = {desc, schema};
  return *this;
}

namespace {

/// Whether a JSON Schema can describe a body served under this media type.
bool media_type_carries_a_json_document(const std::string & media_type) {
  // An SSE frame's `data:` field is a JSON document, so a schema describes it
  // exactly - it just describes the frame rather than the whole response body.
  // `application/octet-stream` and `multipart/byteranges` carry bytes, where a
  // JSON Schema would describe nothing.
  return media_type == "text/event-stream";
}

}  // namespace

RouteEntry & RouteEntry::response(int status_code, const std::string & desc, const nlohmann::json & schema,
                                  const std::vector<std::string> & content_types) {
  const bool schema_is_publishable =
      !content_types.empty() && std::all_of(content_types.begin(), content_types.end(), [](const std::string & mt) {
        return media_type_carries_a_json_document(mt);
      });
  if (!schema.empty() && !schema_is_publishable) {
    // Reported, not published, and not asserted - this is a Release build. A
    // JSON Schema attached to `application/octet-stream` would describe a body
    // shape nothing validates.
    schema_on_non_json_statuses_.push_back(status_code);
  }
  responses_[status_code] = {desc, schema_is_publishable ? schema : nlohmann::json{}, {}, content_types};
  return *this;
}

RouteEntry & RouteEntry::request_body(const std::string & desc, const nlohmann::json & schema,
                                      const std::string & content_type) {
  request_body_ = RequestBodyInfo{desc, schema, content_type};
  return *this;
}

RouteEntry & RouteEntry::multipart_body(const std::string & desc, const std::vector<MultipartPart> & parts) {
  nlohmann::json properties = nlohmann::json::object();
  nlohmann::json required = nlohmann::json::array();
  nlohmann::json encoding = nlohmann::json::object();
  for (const auto & part : parts) {
    // A binary part is a schema-less property plus an encoding entry; anything
    // else carries the schema the call site gave it.
    properties[part.name] = part.schema.empty() ? nlohmann::json::object() : part.schema;
    if (!part.description.empty()) {
      properties[part.name]["description"] = part.description;
    }
    if (part.required) {
      required.push_back(part.name);
    }
    if (!part.content_type.empty()) {
      encoding[part.name]["contentType"] = part.content_type;
    }
  }
  nlohmann::json schema{{"type", "object"}, {"properties", properties}};
  if (!required.empty()) {
    schema["required"] = required;
  }
  request_body_ = RequestBodyInfo{desc, schema, "multipart/form-data"};
  multipart_encoding_ = std::move(encoding);
  return *this;
}

RouteEntry & RouteEntry::accepts(const std::string & content_type, const nlohmann::json & schema) {
  // No description: it is the primary body's, since this is the same payload in
  // another encoding. Read from request_body_ at emission time rather than
  // restated here, so the two cannot disagree.
  extra_bodies_.push_back(RequestBodyInfo{std::string{}, schema, content_type});
  return *this;
}

RouteEntry & RouteEntry::body_example(nlohmann::json example) {
  body_example_ = std::move(example);
  return *this;
}

RouteEntry & RouteEntry::path_param(const std::string & name, const std::string & desc) {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "path";
  param["required"] = true;
  param["description"] = desc;
  param["schema"] = {{"type", "string"}};
  parameters_.push_back(std::move(param));
  return *this;
}

RouteEntry & RouteEntry::query_param(const std::string & name, const std::string & desc, const std::string & type) {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "query";
  param["required"] = false;
  param["description"] = desc;
  param["schema"] = {{"type", type}};
  parameters_.push_back(std::move(param));
  return *this;
}

RouteEntry & RouteEntry::add_query_parameters(const nlohmann::json & params) {
  for (const auto & param : params) {
    parameters_.push_back(param);
  }
  return *this;
}

RouteEntry & RouteEntry::header_param(const std::string & name, const std::string & desc, bool required,
                                      const nlohmann::json & schema) {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "header";
  param["required"] = required;
  param["description"] = desc;
  param["schema"] = schema;
  parameters_.push_back(std::move(param));
  return *this;
}

RouteEntry & RouteEntry::deprecated() {
  deprecated_ = true;
  return *this;
}

RouteEntry & RouteEntry::operation_id(const std::string & id) {
  operation_id_ = id;
  return *this;
}

RouteEntry & RouteEntry::hidden() {
  hidden_ = true;
  return *this;
}

RouteEntry & RouteEntry::mark_alternates() {
  alternates_ = true;
  return *this;
}

RouteEntry & RouteEntry::mark_partial_content() {
  partial_content_ = true;
  return *this;
}

RouteEntry & RouteEntry::success_description(const std::string & desc) {
  for (auto & [code, info] : responses_) {
    if (code >= 200 && code < 300) {
      info.desc = desc;
    }
  }
  return *this;
}

RouteEntry & RouteEntry::success_schema(const nlohmann::json & schema) {
  ResponseInfo * target = nullptr;
  for (auto & [code, info] : responses_) {
    if (code >= 200 && code < 300) {
      if (target != nullptr) {
        // Two success responses and no way to know which one this describes.
        // No assert - Release build - so record it and let the startup check
        // report it rather than picking one at random.
        success_schema_without_single_2xx_ = true;
        return *this;
      }
      target = &info;
    }
  }
  if (target == nullptr) {
    success_schema_without_single_2xx_ = true;
    return *this;
  }
  // A schema can only describe a body a JSON Schema can describe. The JSON
  // default (no declared media types) always qualifies; a declared set does
  // only when every entry carries a JSON document, which is true of an SSE
  // frame and false of a binary download.
  const bool describable =
      target->content_types.empty() ||
      std::all_of(target->content_types.begin(), target->content_types.end(), [](const std::string & mt) {
        return media_type_carries_a_json_document(mt);
      });
  if (!describable) {
    success_schema_without_single_2xx_ = true;
    return *this;
  }
  target->schema = schema;
  return *this;
}

RouteEntry & RouteEntry::response_header(int status_code, ResponseHeader header) {
  auto it = responses_.find(status_code);
  if (it == responses_.end()) {
    // No assert: this is a release build, and an assert compiled out would let
    // the header vanish silently. Record the miscall so validate_completeness()
    // surfaces it at startup instead.
    undeclared_header_statuses_.push_back(status_code);
    return *this;
  }
  auto & headers = it->second.headers;
  auto existing = std::find_if(headers.begin(), headers.end(), [&header](const ResponseHeader & h) {
    return h.name == header.name;
  });
  if (existing != headers.end()) {
    *existing = std::move(header);
  } else {
    headers.push_back(std::move(header));
  }
  return *this;
}

RouteEntry & RouteEntry::errors(std::initializer_list<int> codes) {
  return errors(std::vector<int>(codes));
}

RouteEntry & RouteEntry::errors(const std::vector<int> & codes) {
  for (int code : codes) {
    if (code < 400) {
      // Not an error status. Recording it rather than silently dropping it is
      // what turns a miscall into a validate_completeness() issue.
      rejected_error_codes_.push_back(code);
      continue;
    }
    declared_errors_.push_back(code);
  }
  return *this;
}

RouteEntry & RouteEntry::lock_client_header(const std::string & desc) {
  lock_client_header_ = true;
  // Optional, not required: a caller that sends no `X-Client-Id` is treated as
  // an anonymous client, which succeeds while nothing is locked and is refused
  // once something is. Declaring it required would describe a gateway that
  // rejects the header-less request outright, which is not what happens.
  return header_param("X-Client-Id", desc, false, nlohmann::json{{"type", "string"}});
}

RouteEntry & RouteEntry::lock_guarded() {
  lock_guarded_ = true;
  lock_client_header(
      "Identifies the calling client for lock ownership. While a lock protects this "
      "entity's resource collection, only the client holding it may write; every other "
      "caller - including one that sends no `X-Client-Id` - is answered 409.");
  // Through errors(), not response(): a 409 here carries the SOVD GenericError
  // body, and errors() is what publishes it against the shared component
  // response instead of minting a bespoke bodyless one.
  return errors({409});
}

RouteEntry & RouteEntry::fan_out_aware() {
  // The prose deliberately does not promise loop-freedom in general: the
  // gateway sets this header on its own outbound peer requests, but that only
  // terminates a recursion on routes that read it - which is exactly the set
  // carrying this declaration. The global `GET /faults` fans out without
  // checking it and so is not declared here.
  return header_param("X-Medkit-No-Fan-Out",
                      "Present at any value: answer from this gateway alone and do not query "
                      "aggregated peers. The gateway sets it on its own outbound peer requests, so "
                      "bidirectional aggregation terminates at the first peer for this operation.",
                      false, nlohmann::json{{"type", "string"}});
}

RouteEntry & RouteEntry::only_status(int code, const std::string & desc) {
  responses_.clear();
  declared_errors_.clear();
  only_status_ = true;
  // An error status carries a GenericError body on the wire, so publishing it
  // with no `content` would describe a bodyless response a client then receives
  // JSON into. Attaching the schema here (rather than routing `code` through
  // declared_errors_, which would empty responses_ and make
  // validate_completeness inject a phantom 200) keeps `desc` AND keeps the
  // "only error responses" branch of both completeness gates satisfied.
  nlohmann::json schema;
  if (code >= 400) {
    schema = nlohmann::json{{"$ref", "#/components/schemas/GenericError"}};
  }
  responses_[code] = {desc, schema};
  // A live gate is a second outcome this route can produce, so "exactly one
  // status" is not true of a gated route. Re-declare the gate's status instead
  // of letting builder order decide whether the document mentions it: without
  // this, `.gated_on(...).only_status(...)` silently drops the very status the
  // gate exists to return.
  if (gate_ && gate_->has_value()) {
    errors({(*gate_)->unavailable.http_status});
  }
  return *this;
}

RouteEntry & RouteEntry::gated_on(std::function<bool()> available, ErrorInfo unavailable) {
  // Read the status before the move: declaring it is the half of this call the
  // document depends on, and a moved-from ErrorInfo has none.
  const int status = unavailable.http_status;
  // The shared handle is captured by the typed wrapper closure, so assigning
  // through it is what makes a `.gated_on(...)` applied AFTER `reg.get<...>()`
  // reach the already-built handler (same mechanism as `.error_renderer(...)`).
  *gate_ = RouteGate{std::move(available), std::move(unavailable)};
  // A gate is the only thing that can produce this status on this route, so the
  // route declares it here rather than leaving the document silent about it.
  // Routing it through errors() also means a sub-400 status is rejected and
  // reported by validate_completeness() instead of being published.
  return errors({status});
}

RouteEntry & RouteEntry::error_renderer(ErrorRenderer renderer) {
  // The shared_ptr is captured by the typed handler wrapper closure; mutating
  // through it is the mechanism by which `.error_renderer(...)` called AFTER
  // `reg.post<...>(...)` still influences the closure's behaviour.
  *error_renderer_ = renderer;
  return *this;
}

RouteEntry & RouteEntry::requires_role(UserRole role) {
  required_role_ = role;
  role_declared_ = true;
  return *this;
}

RouteEntry & RouteEntry::public_route() {
  required_role_.reset();
  role_declared_ = true;
  return *this;
}

// -----------------------------------------------------------------------------
// RouteRegistry route registration
// -----------------------------------------------------------------------------

RouteEntry & RouteRegistry::add_route(const std::string & method, const std::string & openapi_path, HandlerFn handler) {
  RouteEntry entry;
  entry.method_ = method;
  entry.path_ = openapi_path;
  entry.regex_path_ = to_regex_path(openapi_path, method);
  entry.handler_ = std::move(handler);
  routes_.push_back(std::move(entry));
  return routes_.back();
}

// -----------------------------------------------------------------------------
// add_raw_route - for escape hatches whose URI is a literal regex (docs_subtree)
// -----------------------------------------------------------------------------

RouteEntry & RouteRegistry::add_raw_route(const std::string & method, const std::string & openapi_path,
                                          const std::string & regex_path, HandlerFn handler) {
  RouteEntry entry;
  entry.method_ = method;
  entry.path_ = openapi_path;
  entry.regex_path_ = regex_path;
  entry.handler_ = std::move(handler);
  routes_.push_back(std::move(entry));
  return routes_.back();
}

// -----------------------------------------------------------------------------
// Typed-handler helpers
// -----------------------------------------------------------------------------

ErrorInfo RouteRegistry::make_body_parse_error(const std::vector<dto::FieldError> & errs) {
  ErrorInfo info;
  info.code = ERR_INVALID_REQUEST;
  info.message = "Request body validation failed";
  info.http_status = 400;
  nlohmann::json fields = nlohmann::json::array();
  for (const auto & e : errs) {
    fields.push_back({{"field", e.field}, {"message", e.message}});
  }
  info.params = nlohmann::json::object();
  info.params["fields"] = std::move(fields);
  return info;
}

void RouteRegistry::apply_attachments(httplib::Response & res, const http::ResponseAttachments & att) {
  if (att.status_override.has_value()) {
    res.status = *att.status_override;
  }
  for (const auto & [name, value] : att.headers) {
    res.set_header(name, value);
  }
}

void RouteRegistry::write_typed_error(httplib::Response & res, const ErrorInfo & err,
                                      const std::shared_ptr<ErrorRenderer> & renderer_ptr) {
  // Forwarded sentinel: the peer-forwarding path has already streamed the
  // proxied response (body, status, headers) to `res` via the framework's
  // forwarding sink. Rendering anything here would corrupt the wire response,
  // so this path is a strict no-op. The sentinel never escapes the framework
  // because typed handlers translate validator-returned Forwarded into it via
  // HandlerContext::forwarded_sentinel_error.
  if (err.code == ERR_X_INTERNAL_FORWARDED) {
    return;
  }
  ErrorRenderer renderer = renderer_ptr ? *renderer_ptr : ErrorRenderer::kSovdGenericError;
  if (renderer == ErrorRenderer::kOAuth2Error) {
    http::detail::write_oauth2_error(http::detail::FrameworkOrPluginAccess{}, res, err);
  } else {
    http::detail::write_generic_error(http::detail::FrameworkOrPluginAccess{}, res, err);
  }
}

bool RouteRegistry::gate_blocked(httplib::Response & res, const GateHandle & gate,
                                 const std::shared_ptr<ErrorRenderer> & renderer) {
  if (!gate || !gate->has_value()) {
    return false;
  }
  const RouteGate & g = **gate;
  // Fail closed. A gate whose predicate is empty is a half-built gate, and the
  // handlers behind a gate dereference their manager unconditionally - the gate
  // is the only thing keeping that safe. Answering the documented status beats
  // running the handler and segfaulting.
  if (g.available && g.available()) {
    return false;
  }
  write_typed_error(res, g.unavailable, renderer);
  return true;
}

// -----------------------------------------------------------------------------
// Escape-hatch routes (SSE / binary / static asset / docs)
// -----------------------------------------------------------------------------

RouteEntry & RouteRegistry::sse(const std::string & openapi_path,
                                std::function<http::Result<http::SseStream>(http::TypedRequest)> stream_factory) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  auto gate = std::make_shared<std::optional<RouteGate>>();
  HandlerFn fn = [factory = std::move(stream_factory), renderer, gate](const httplib::Request & req,
                                                                       httplib::Response & res) {
    // Install the forwarding scope so SSE factories that call
    // validate_entity_for_route can stream a proxied wire response for entities
    // owned by a remote peer. Without it the validator's Forwarded branch has
    // no response to write to. The scope ends before the chunked content
    // provider starts streaming - peer-forwarding is a synchronous decision
    // made up-front, never mid-stream.
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = factory(typed_req);
    if (!outcome.has_value()) {
      write_typed_error(res, outcome.error(), renderer);
      return;
    }
    auto stream = std::make_shared<http::SseStream>(std::move(outcome.value()));
    // SSE proxy-friendliness headers (no client wants buffered Server-Sent
    // Events). Set BEFORE the chunked content provider takes over; the
    // framework owns these so individual handlers stay free of httplib state.
    // Note: cpp-httplib's chunked content provider already sets the
    // Content-Type header; never set it on `res` first or it duplicates.
    res.set_header("Cache-Control", "no-cache");
    res.set_header("X-Accel-Buffering", "no");
    res.set_chunked_content_provider("text/event-stream",
                                     [stream](std::size_t /*offset*/, httplib::DataSink & sink) -> bool {
                                       if (!stream || !stream->next_event) {
                                         sink.done();
                                         return false;
                                       }
                                       const bool keep_going = stream->next_event(sink);
                                       if (!keep_going) {
                                         sink.done();
                                       }
                                       return keep_going;
                                     });
  };
  auto & entry = add_route("get", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.gate_ = gate;
  // Declared with the media type cpp-httplib is handed two lines above, so the
  // document and the wire come from one fact rather than from a summary string
  // containing the word "stream".
  //
  // No frame schema here, because the three SSE families (trigger events,
  // subscription data, fault notifications) put different shapes in `data:` and
  // one schema would be wrong for two of them. Each family attaches its own on
  // the returned RouteEntry with `.success_schema<T>()`, which replaces this
  // response's schema and leaves the status, description and the two headers
  // below untouched; this helper cannot know which family it is registering.
  entry.response(200, "Server-Sent Events stream", nlohmann::json{}, {"text/event-stream"});
  // Declared here, next to the `set_header` calls above, because that is what
  // stops the two from drifting: the framework owns these headers, so no SSE
  // route can be registered without them and none can document them wrongly.
  entry.response_header(200, ResponseHeader{"Cache-Control", "Always `no-cache`; event streams are never cached."});
  entry.response_header(
      200, ResponseHeader{"X-Accel-Buffering", "Always `no`; disables response buffering in nginx-style proxies."});
  return entry;
}

RouteEntry &
RouteRegistry::binary_download(const std::string & openapi_path,
                               std::function<http::Result<http::BinaryResponse>(http::TypedRequest)> handler,
                               const std::vector<std::string> & media_types) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  auto gate = std::make_shared<std::optional<RouteGate>>();
  HandlerFn fn = [handler = std::move(handler), renderer, gate](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope: entity-scoped binary downloads (bulk-data, scripts) on a
    // remote peer must proxy through validate_entity_for_route (see sse / wrap_body_less).
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (!outcome.has_value()) {
      write_typed_error(res, outcome.error(), renderer);
      return;
    }
    auto bin = std::make_shared<http::BinaryResponse>(std::move(outcome.value()));
    if (bin->filename.has_value()) {
      res.set_header("Content-Disposition", "attachment; filename=\"" + *bin->filename + "\"");
    }
    if (bin->supports_ranges) {
      // RFC 9110 §14.3: a range-capable resource advertises the unit it accepts.
      // cpp-httplib only fills this in for HEAD, so a GET would otherwise serve
      // partial content no client knew it could ask for.
      res.set_header("Accept-Ranges", "bytes");
      res.set_content_provider(static_cast<std::size_t>(bin->total_size), bin->content_type,
                               [bin](std::size_t offset, std::size_t length, httplib::DataSink & sink) -> bool {
                                 return bin->provider(static_cast<std::uint64_t>(offset),
                                                      static_cast<std::uint64_t>(length), sink);
                               });
    } else {
      res.set_chunked_content_provider(bin->content_type,
                                       [bin](std::size_t /*offset*/, httplib::DataSink & sink) -> bool {
                                         const bool keep = bin->provider(0, bin->total_size, sink);
                                         sink.done();
                                         return keep;
                                       });
    }
  };
  auto & entry = add_route("get", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.gate_ = gate;
  // The handler never assigns `res.status`, so cpp-httplib decides it: 200, or
  // 206 when the request carried a `Range` - and it fills in `Content-Range`
  // itself. Advertising `Accept-Ranges` while saying nothing about what a
  // `Range` request answers would invite clients into an undocumented response.
  entry.response(200, "Binary download", nlohmann::json{}, media_types);

  // A multi-range request is answered by wrapping the parts in
  // `multipart/byteranges` (cpp-httplib `apply_ranges`, the
  // `req.ranges.size() > 1` branch, which rewrites Content-Type and generates a
  // boundary). That is a media type the 200 can never carry, so it is declared
  // on the 206 only, and it is derived from the framework's own behaviour
  // rather than from what the caller passed in.
  std::vector<std::string> partial_media_types = media_types;
  partial_media_types.emplace_back("multipart/byteranges");
  entry.response(206, "Requested byte range of the file", nlohmann::json{}, partial_media_types);
  entry.mark_partial_content();

  // The request half of the same contract. Optional: without it the route
  // answers 200 with the whole body, which is the overwhelmingly common case.
  // ASCII on purpose: every other description the document emits is ASCII, and
  // the section sign appears in this file only inside comments.
  entry.header_param("Range",
                     "Byte range to fetch, per RFC 9110 section 14.2, e.g. `bytes=0-1023`. Several ranges "
                     "are answered as one `multipart/byteranges` body.",
                     false, nlohmann::json{{"type", "string"}});

  // Set on the response before the content provider takes over, so they ride on
  // whichever status cpp-httplib picks - hence declared on both. Each is
  // conditional on the BinaryResponse the handler returned (a filename, a
  // range-capable provider), which is why none is `required`: OpenAPI response
  // headers are optional by definition.
  const ResponseHeader content_disposition{
      "Content-Disposition", "`attachment; filename=\"...\"` when the download names a file. Absent otherwise."};
  const ResponseHeader accept_ranges{"Accept-Ranges",
                                     "`bytes` when the download supports range requests. Absent otherwise."};
  entry.response_header(200, content_disposition);
  entry.response_header(200, accept_ranges);
  entry.response_header(206, content_disposition);
  entry.response_header(206, accept_ranges);
  entry.response_header(
      206, ResponseHeader{"Content-Range", "`bytes <start>-<end>/<total>` for the range that was served."});
  return entry;
}

RouteEntry & RouteRegistry::static_asset(const std::string & openapi_path,
                                         std::function<http::Result<http::StaticAsset>(http::TypedRequest)> handler) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  auto gate = std::make_shared<std::optional<RouteGate>>();
  HandlerFn fn = [handler = std::move(handler), renderer, gate](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope kept uniform across wrappers (static assets are not
    // entity-scoped, so this never forwards; see the comment at the top of this file).
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (!outcome.has_value()) {
      write_typed_error(res, outcome.error(), renderer);
      return;
    }
    const auto & asset = outcome.value();
    for (const auto & [name, value] : asset.headers) {
      res.set_header(name, value);
    }
    res.status = 200;
    std::string body(asset.bytes.begin(), asset.bytes.end());
    res.set_content(body, asset.content_type);
  };
  auto & entry = add_route("get", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.gate_ = gate;
  entry.hidden();  // Static assets are not part of the documented JSON API.
  return entry;
}

RouteEntry & RouteRegistry::docs_endpoint(const std::string & openapi_path,
                                          std::function<http::Result<std::string>(http::TypedRequest)> handler) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  auto gate = std::make_shared<std::optional<RouteGate>>();
  HandlerFn fn = [handler = std::move(handler), renderer, gate](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope kept uniform across wrappers (see the comment at the top of this file).
    http::detail::ForwardResponseScope forward_scope(&res);
    if (gate_blocked(res, gate, renderer)) {
      return;
    }
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (!outcome.has_value()) {
      write_typed_error(res, outcome.error(), renderer);
      return;
    }
    http::detail::write_json_text(http::detail::FrameworkOrPluginAccess{}, res, outcome.value(), 200);
  };
  auto & entry = add_route("get", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.gate_ = gate;
  entry.response(200, "OpenAPI specification document",
                 nlohmann::json{{"type", "object"}, {"additionalProperties", true}});
  return entry;
}

RouteEntry & RouteRegistry::docs_subtree(const std::string & openapi_path, const std::string & regex_pattern,
                                         HandlerFn handler) {
  auto & entry = add_raw_route("get", openapi_path, regex_pattern, std::move(handler));
  // Attached here, not at the call site, for the same reason `docs_endpoint`
  // does it: a success status is a property of the registration, and a call
  // site that hand-attaches its own 2xx is the drift this design forbids
  // (design/dto_contract.rst). The body is the OpenAPI document itself, which
  // is why this helper exists rather than a typed `get<T>`.
  entry.response(200, "OpenAPI specification document scoped to the requested path",
                 nlohmann::json{{"type", "object"}, {"additionalProperties", true}});
  return entry;
}

// -----------------------------------------------------------------------------
// to_regex_path - convert OpenAPI {param} path to cpp-httplib regex
// -----------------------------------------------------------------------------

std::string RouteRegistry::to_regex_path(const std::string & openapi_path, const std::string & /*method*/) {
  // We need to convert {param} placeholders to regex capture groups.
  // Special cases:
  //   - {data_id} at the end of data paths -> (.+) (multi-segment, for slash-containing topic names)
  //   - {config_id} at the end of configuration paths -> (.+) (for slash-containing param names)
  //   - {operation_id} anywhere -> (.+) (an operation is addressed by ROS path where one
  //     provider carries its short name twice, and the executions sub-resources sit behind it)
  //   - All other {param} -> ([^/]+) (single segment)
  //
  // The "end of path" check ensures only the LAST param on data/config paths gets (.+).
  // {operation_id} carries no such check because the sub-resource routes put it mid-path;
  // the routes it can then swallow are registered ahead of it (see setup_routes).

  // Root path "/" -> just optional slash anchor (prefix already has the base path)
  if (openapi_path == "/") {
    return "/?$";
  }

  std::string result;
  size_t i = 0;
  while (i < openapi_path.size()) {
    if (openapi_path[i] == '{') {
      auto close = openapi_path.find('}', i);
      if (close == std::string::npos) {
        result += openapi_path[i];
        ++i;
        continue;
      }
      std::string param_name = openapi_path.substr(i + 1, close - i - 1);
      bool is_last = (close + 1 >= openapi_path.size());

      // Use (.+) for the final segment on data and configuration item paths
      if (param_name == "operation_id" || (is_last && (param_name == "data_id" || param_name == "config_id"))) {
        result += "(.+)";
      } else {
        result += "([^/]+)";
      }
      i = close + 1;
    } else {
      result += openapi_path[i];
      ++i;
    }
  }

  // Accept optional trailing slash, then anchor to ensure exact match
  result += "/?$";
  return result;
}

// -----------------------------------------------------------------------------
// entity_scoped - can this route resolve an entity, and therefore forward?
// -----------------------------------------------------------------------------

bool RouteRegistry::entity_scoped(const std::string & openapi_path) {
  static const std::array<const char *, 4> kEntityParams = {"{area_id}", "{component_id}", "{app_id}", "{function_id}"};
  return std::any_of(kEntityParams.begin(), kEntityParams.end(), [&openapi_path](const char * param) {
    return openapi_path.find(param) != std::string::npos;
  });
}

// -----------------------------------------------------------------------------
// register_all - register all routes with cpp-httplib server
// -----------------------------------------------------------------------------

void RouteRegistry::register_all(httplib::Server & server, const std::string & api_prefix) const {
  for (const auto & route : routes_) {
    std::string full_path = api_prefix + route.regex_path_;
#ifdef MEDKIT_STATUS_RECORDER
    // Test builds only. Mounting point is the one place that knows both the
    // route's OpenAPI templated path and every response the route produces,
    // so the recorder attaches here rather than inside the typed wrappers
    // (which see the handler but not its identity). The identity strings are
    // captured by value: cpp-httplib owns the resulting std::function and
    // must not outlive-dangle into the registry's deque.
    HandlerFn handler = [method = route.method_, path = route.path_,
                         inner = route.handler_](const httplib::Request & req, httplib::Response & res) {
      const http::detail::StatusRecordingScope scope(method, path, req, res);
      inner(req, res);
    };
#else
    const auto & handler = route.handler_;
#endif

    if (route.method_ == "get") {
      server.Get(full_path, handler);
    } else if (route.method_ == "post") {
      server.Post(full_path, handler);
    } else if (route.method_ == "put") {
      server.Put(full_path, handler);
    } else if (route.method_ == "patch") {
      server.Patch(full_path, handler);
    } else if (route.method_ == "delete") {
      server.Delete(full_path, handler);
    }
  }
}

// -----------------------------------------------------------------------------
// to_openapi_paths - generate OpenAPI paths object
// -----------------------------------------------------------------------------

bool RouteRegistry::auth_enforced_on(const RouteEntry & route) const {
  // Same two terms, same order, as `AuthManager::requires_authentication`.
  if (!auth_enabled_) {
    return false;
  }
  if (auth_policy_ == nullptr) {
    return true;
  }
  // The policy is written against the request line: `WriteOnlyAuth
  // RequirementPolicy` compares the method to upper-case literals and
  // `AllAuthRequirementPolicy` matches the path prefix the client sends. The
  // registry stores neither in that form, so both are converted back here
  // rather than the policies being loosened to accept the registry's.
  std::string method_upper = route.method_;
  for (char & c : method_upper) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return auth_policy_->requires_authentication(method_upper, auth_api_prefix_ + route.path_);
}

nlohmann::json RouteRegistry::to_openapi_paths() const {
  nlohmann::json paths = nlohmann::json::object();

  for (const auto & route : routes_) {
    // Hidden routes are served by cpp-httplib but excluded from OpenAPI spec
    if (route.hidden_) {
      continue;
    }

    nlohmann::json operation;

    if (!route.tag_.empty()) {
      operation["tags"] = nlohmann::json::array({route.tag_});
    }
    if (!route.summary_.empty()) {
      operation["summary"] = route.summary_;
    }
    if (!route.description_.empty()) {
      operation["description"] = route.description_;
    }
    if (route.deprecated_) {
      operation["deprecated"] = true;
    }
    if (route.alternates_) {
      // The handler returns a variant, so more than one 2xx code is genuine.
      operation["x-medkit-alternates"] = true;
    }
    if (route.partial_content_) {
      // cpp-httplib answers 206 instead of 200 whenever the request carries a
      // `Range` at all, so more than one 2xx code is genuine here too - for a
      // different reason than a variant-returning handler. Not "a satisfiable
      // Range": a parseable range past the end of the file still yields 206,
      // and an unparseable one is rejected with 416 before routing.
      operation["x-medkit-partial-content"] = true;
    }
    // `lock_guarded()` is a registration-time declaration; whether this gateway
    // has a LockManager at all is a deployment setting, and the marker, the
    // `X-Client-Id` parameter and the 409 are one contract that stands or falls
    // together. With `locking.enabled` off `GatewayNode` never builds the
    // manager, `HandlerContext::validate_lock_access` returns success without
    // reading the header, and no write can be refused - so all three come out,
    // and the document stops promising serialised writes on a gateway whose own
    // root reports `capabilities.locking: false`.
    //
    // The header is keyed separately from the marker because the two sets are
    // not the same: `DELETE /faults` declares the header through
    // `lock_client_header()` and deliberately carries no marker, since it skips
    // locked faults rather than refusing.
    const bool drop_lock_contract = route.lock_guarded_ && !locking_enabled_;
    const bool drop_lock_header = route.lock_client_header_ && !locking_enabled_;
    if (route.lock_guarded_ && locking_enabled_) {
      // Declared by the registration, never inferred from the handler - see
      // RouteEntry::lock_guarded() for why that derivation is not available.
      operation["x-medkit-lock-guarded"] = true;
    }

    // Parameters
    if (!route.parameters_.empty()) {
      operation["parameters"] = route.parameters_;
      if (drop_lock_header) {
        // Erases the first `X-Client-Id` and stops. That is the one
        // `lock_client_header()` pushed *provided no route both declares one of
        // its own and carries the locking flag* - true of every route today,
        // and not enforced anywhere. A route that did both would lose whichever
        // declaration came first. The lock CRUD routes are unaffected: they
        // declare theirs through plain `header_param` and never set the flag,
        // so this branch does not run for them at all.
        auto & params = operation["parameters"];
        for (auto it = params.begin(); it != params.end(); ++it) {
          const auto name = it->find("name");
          if (name != it->end() && *name == "X-Client-Id") {
            params.erase(it);
            break;
          }
        }
        if (params.empty()) {
          operation.erase("parameters");
        }
      }
    }

    // Also extract path parameters from the path template and add them
    // if they were not explicitly added via path_param()
    {
      std::set<std::string> explicit_params;
      for (const auto & p : route.parameters_) {
        // Looked up with find()/get_ref() rather than value(): the latter runs
        // nlohmann's from_json conversion machinery, which GCC inlines into a
        // -Wnull-dereference false positive here, and it silently inserted an
        // empty name for a parameter object missing "name".
        const auto in_it = p.find("in");
        if (in_it == p.end() || *in_it != "path") {
          continue;
        }
        const auto name_it = p.find("name");
        if (name_it != p.end() && name_it->is_string()) {
          explicit_params.insert(name_it->get_ref<const std::string &>());
        }
      }

      // Find all {param} in the path
      std::string::size_type pos = 0;
      while ((pos = route.path_.find('{', pos)) != std::string::npos) {
        auto close = route.path_.find('}', pos);
        if (close == std::string::npos) {
          break;
        }
        std::string pname = route.path_.substr(pos + 1, close - pos - 1);
        if (explicit_params.find(pname) == explicit_params.end()) {
          // Auto-generated path parameters. Every route carrying `{fault_code}`
          // or `{config_id}` is registered from a loop over the four entity
          // types, and none of them declares the parameter by hand, so this
          // table is the one place either is described - which is why the
          // length the handler enforces is a column here rather than a
          // per-registration call that a new route could forget.
          //
          // `max_length` 0 means "no length constraint published".
          //
          // **Precondition, and it is on you to keep it.** This table is keyed
          // by parameter *name*, so a bound written here is published on EVERY
          // route carrying that template - it cannot say "this route's handler
          // checks, that one's does not", and nothing verifies the mapping. So
          // only fill it where *every* handler behind the template rejects an
          // over-long value unconditionally. The first version of this table
          // broke that: `delete_configuration` was the one verb of the three
          // that never measured `config_id`, and the 512 was published on its
          // routes anyway. That check now exists.
          //
          // "Unconditionally" means no branch inside the handler skips the
          // measurement - not that it is the first thing the request meets.
          // The five handlers behind these two templates do not agree on where
          // it sits, so an over-long value on an *unknown* entity gets one of
          // two answers depending on the verb:
          //   `get_configuration`, `get_fault`, `clear_fault`  resolve the
          //       entity first -> 404
          //   `set_configuration`, `delete_configuration`      measure first
          //       -> 400
          // The entity-id check those last two run beforehand does not close
          // the gap: `validate_entity_id` is format-only and never consults the
          // cache. `clear_fault` also takes `validate_lock_access` before
          // measuring, so a competing lock answers 409.
          //
          // None of that weakens the precondition - every route publishing a
          // bound does reject an over-long value. Only the status a caller sees
          // when it is *also* wrong about something else differs, and that
          // split is an accident of handlers written at different times rather
          // than a decision anyone recorded. It is deliberately not pinned by a
          // test: pinning it would make an accident load-bearing.
          //
          // Both rows are covered on every verb they publish to, so the
          // precondition is tested rather than asserted:
          //   config_id  - test_configuration_api.test.py
          //                ::test_06b_every_verb_rejects_an_oversized_config_id
          //                (GET / PUT / DELETE)
          //   fault_code - test_faults_api.test.py
          //                ::test_both_verbs_reject_an_oversized_fault_code
          //                (GET / DELETE)
          // A new row needs its own, or the bound it publishes rests on a
          // reading of the handlers rather than on a run.
          struct PathParamInfo {
            const char * description;
            std::size_t max_length;
          };
          static const std::unordered_map<std::string, PathParamInfo> kParamDescriptions = {
              {"area_id", {"The area identifier", 0}},
              {"component_id", {"The component identifier", 0}},
              {"app_id", {"The app identifier", 0}},
              {"function_id", {"The function identifier", 0}},
              {"data_id",
               {"The data item identifier (ROS 2 topic name), or 'member_id:topic' when more than one member of "
                "the entity contributes an item under that name",
                0}},
              {"operation_id",
               {"The operation identifier (service or action short name), or 'member_id:operation' when more than "
                "one member of the entity exposes that name, or the ROS 2 path without its leading slash when one "
                "provider exposes that short name at more than one path",
                0}},
              {"execution_id", {"The execution identifier", 0}},
              // Not simply "the ROS 2 parameter name": on an entity backed by
              // more than one node a write of a bare name is rejected with 400
              // (`config_handlers.cpp`, "Aggregated configuration requires
              // app_id prefix"), and it is the prefixed form the list response
              // hands back as each item's `id`. 512 = 256 (entity id) + 1 (`:`)
              // + 256 (parameter name).
              {"config_id",
               {"The configuration parameter identifier. On an entity that aggregates several ROS 2 nodes this is "
                "the `app_id:param_name` form the configurations list returns as each item's `id`, and a write of "
                "a bare parameter name is rejected as ambiguous; on a single-node entity it is the bare parameter "
                "name and a colon in it is part of the name. Maximum 512 characters.",
                512}},
              {"fault_code", {"The fault code identifier. Maximum 256 characters.", 256}},
              {"subscription_id", {"The cyclic subscription identifier", 0}},
              {"category_id", {"The bulk data category identifier", 0}},
              {"file_id", {"The bulk data file identifier", 0}},
              {"update_id", {"The software update identifier", 0}},
              {"subarea_id", {"The subarea identifier", 0}},
              {"subcomponent_id", {"The subcomponent identifier", 0}},
              {"trigger_id", {"The trigger identifier", 0}},
              {"lock_id", {"The lock identifier", 0}},
              {"script_id", {"The script identifier", 0}},
          };
          nlohmann::json param;
          param["name"] = pname;
          param["in"] = "path";
          param["required"] = true;
          param["schema"] = {{"type", "string"}};
          auto desc_it = kParamDescriptions.find(pname);
          if (desc_it != kParamDescriptions.end()) {
            param["description"] = desc_it->second.description;
            if (desc_it->second.max_length > 0) {
              param["schema"]["maxLength"] = desc_it->second.max_length;
            }
          } else {
            param["description"] = "The " + pname + " value";
          }
          if (!operation.contains("parameters")) {
            operation["parameters"] = nlohmann::json::array();
          }
          operation["parameters"].push_back(std::move(param));
        }
        pos = close + 1;
      }
    }

    // Request body
    if (route.request_body_.has_value()) {
      operation["requestBody"]["description"] = route.request_body_->desc;
      const auto & ct = route.request_body_->content_type;
      if (!route.request_body_->schema.empty()) {
        operation["requestBody"]["content"][ct]["schema"] = route.request_body_->schema;
      }
      if (!route.multipart_encoding_.empty()) {
        operation["requestBody"]["content"][ct]["encoding"] = route.multipart_encoding_;
      }
      // Primary media type only: the extra encodings below are the same payload
      // in another wire format, and a JSON example would not parse as one.
      if (route.body_example_.has_value()) {
        operation["requestBody"]["content"][ct]["examples"]["default"]["value"] = *route.body_example_;
      }
      // Further encodings of the same payload (the auth endpoints' RFC 6749
      // form encoding). Merged into the same content object, because they are
      // alternative representations of one body rather than separate bodies.
      for (const auto & extra : route.extra_bodies_) {
        if (!extra.schema.empty()) {
          operation["requestBody"]["content"][extra.content_type]["schema"] = extra.schema;
        }
      }
      operation["requestBody"]["required"] = true;
    }

    // Responses
    if (!route.responses_.empty()) {
      for (const auto & [code, info] : route.responses_) {
        std::string code_str = std::to_string(code);
        operation["responses"][code_str]["description"] = info.desc;
        if (info.content_types.empty()) {
          // JSON default. Guard stays: a default-constructed nlohmann::json is
          // `null`, so dropping it writes `"schema": null` onto every bodyless
          // 204.
          if (!info.schema.empty()) {
            operation["responses"][code_str]["content"]["application/json"]["schema"] = info.schema;
          }
        } else {
          // Non-JSON body: one `content` entry per media type. Normally an
          // empty Media Type Object - the absent schema is the point, not an
          // omission, see RouteEntry::response(status, desc, schema,
          // content_types). The exception is a media type whose body IS a JSON
          // document: an SSE frame's `data:` field has a shape worth
          // publishing, and `response()` only kept a schema here after
          // checking that.
          for (const auto & media_type : info.content_types) {
            auto & media = operation["responses"][code_str]["content"][media_type];
            media = nlohmann::json::object();
            if (!info.schema.empty()) {
              media["schema"] = info.schema;
            }
          }
        }
        for (const auto & header : info.headers) {
          auto & header_obj = operation["responses"][code_str]["headers"][header.name];
          header_obj["description"] = header.description;
          header_obj["schema"] = header.schema;
        }
      }
    } else {
      // Default 200 response
      operation["responses"]["200"]["description"] = "Successful response";
    }

    // Add standard error responses as $ref to GenericError component.
    // Response-level $ref (not nested in content/schema) - the referenced
    // component is a complete response object with description and schema.
    auto add_response_ref = [&operation](const std::string & code, const std::string & component) {
      if (!operation["responses"].contains(code)) {
        operation["responses"][code] = {{"$ref", "#/components/responses/" + component}};
      }
    };
    // Which error body this route puts on the wire is already a per-route fact:
    // `error_renderer_` is what `write_typed_error` dispatches on. Reading it
    // here is what stops the document claiming a SOVD GenericError on the three
    // routes that answer RFC 6749 instead. Everything the middleware owns
    // (401/403/429) keeps its own component below - those never reach a handler,
    // so the route's renderer has no say in them.
    const bool oauth2 = route.error_renderer_ && *route.error_renderer_ == ErrorRenderer::kOAuth2Error;
    auto add_error_ref = [&add_response_ref, oauth2](const std::string & code) {
      add_response_ref(code, oauth2 ? "OAuth2Error" : "GenericError");
    };

    // Skips exactly one 409 - the one `lock_guarded()` pushed - rather than
    // every 409 on the route, so a lock-guarded route that also declared a 409
    // of its own would keep it. `declared_errors_` is a vector and keeps the
    // duplicate, which is what makes counting the right instrument here.
    // Nothing is removed from `declared_errors_` itself: `validate_completeness`
    // reads it to catch a later `only_status()` clearing the status this
    // marker promises, and that check is about the registration, not about the
    // deployment.
    int lock_409_to_drop = drop_lock_contract ? 1 : 0;
    for (int code : route.declared_errors_) {
      if (code == 409 && lock_409_to_drop > 0) {
        --lock_409_to_drop;
        continue;
      }
      add_error_ref(std::to_string(code));
    }

    // `only_status()` states the route has exactly one outcome, so the blanket
    // set would document statuses it can never emit. The auth refs below stay:
    // 401/403 come from the auth middleware ahead of the handler and are
    // reachable on every route regardless of what the handler can return.
    if (!route.only_status_) {
      add_error_ref("400");
      add_error_ref("404");
      add_error_ref("500");
    }

    // The middleware answers these ahead of routing, so they are declared
    // per-route but described once as shared components - that is the only
    // place their headers (`WWW-Authenticate`, `Retry-After`, `X-RateLimit-*`)
    // can live, since no handler return type produces them.
    //
    // "Ahead of routing" is not "on every route" for the auth pair, and
    // reading it that way is what this key corrects. The rate limiter really
    // does run on every non-OPTIONS request; the auth middleware runs
    // `AuthManager::requires_authentication`, which consults a policy, and
    // under `require_auth_for: write` or `none` that policy admits routes this
    // document used to promise a 401 on.
    //
    // These run AFTER the `errors()` loop above and `add_response_ref` is
    // first-wins, so a route that declares a status the middleware also owns
    // keeps its own: the script manager's concurrency 429 and a lifecycle
    // provider's AccessDenied 403 both shadow the middleware component on the
    // routes that declare them. That is deliberate - OpenAPI allows one
    // response object per status and the route-specific description is the more
    // useful of the two - but it costs the middleware's headers on those
    // operations. The body shape is unaffected: Unauthorized, Forbidden and
    // RateLimited all reference the same GenericError schema. Pinned by
    // RouteRegistryTest.RouteDeclaredStatusWinsOverTheMiddlewareComponent.
    if (auth_enforced_on(route)) {
      add_response_ref("401", "Unauthorized");
      add_response_ref("403", "Forbidden");
    }
    if (rate_limit_enabled_) {
      add_response_ref("429", "RateLimited");
    }

    // 416 is answered by cpp-httplib itself, before any handler and before
    // routing: `Server::process_request` rejects an unparseable `Range` header
    // and writes the response straight out (vendored httplib.h:6616-6622). It
    // therefore reaches every operation in this document, including paths that
    // do not exist, which is why it is declared here rather than on the six
    // download routes - those are where a `Range` is *useful*, not where the
    // status originates.
    //
    // It carries the GenericError body like any other error status, but by a
    // different route than the rest: cpp-httplib writes no body at all, and
    // `RESTServer::setup_global_error_handlers` (rest_server.cpp:265-286) fills every
    // body-less error response with a GenericError on the way out. So the
    // shape is the same and the $ref is correct - reading only the vendored
    // header suggests a body-less response, and the gateway's own handler is
    // what makes that wrong. Pinned on the wire by
    // test_openapi_contract::test_range_rejection_is_answered_on_a_route_that_declares_it.
    //
    // Unlike the limiter's 429 this is gated on nothing: rate limiting is
    // opt-in, whereas cpp-httplib's Range parsing has no configuration knob
    // and cannot be turned off.
    //
    // Outside only_status() for the same reason as the auth middleware's
    // 401/403: only_status says the *handler* has one outcome, and no handler
    // runs before this status is decided.
    //
    // The status recorder cannot observe it either - it wraps handlers - so
    // unlike the statuses derived from recorded runs, this declaration is a
    // framework-level constant, pinned by
    // RouteRegistryTest.EveryDocumentedRouteDeclaresTheFrameworkAnsweredRangeRejection.
    //
    // add_response_ref, not add_error_ref: this body does not come from the
    // route's own renderer. cpp-httplib writes 416 with no body, and
    // `RESTServer::setup_global_error_handlers` fills every body-less error
    // response with a GenericError - including on the `/auth/*` routes, whose
    // handler-returned errors are RFC 6749. Routing it through the renderer
    // would document the one status those routes answer in the SOVD shape as
    // an OAuth2Error.
    add_response_ref("416", "GenericError");

    // Peer aggregation. When an entity turns out to belong to a peer, the
    // request is proxied inside `validate_entity_for_route`, and the statuses
    // the *gateway itself* writes on that path are 502 (peer unknown,
    // unreachable, or its response over the size cap) and 503 (this gateway is
    // shutting down and refuses to forward) - see `peer_client.cpp` and
    // `aggregation_manager.cpp`. Both carry the SOVD GenericError body, so the
    // shared component describes them correctly.
    //
    // Gated the same way as the limiter's 429 and for the same reason:
    // `aggregation.enabled` defaults false and the AggregationManager is only
    // constructed when it is set, so with aggregation off no entity is remote
    // and declaring either status would document an unreachable outcome.
    //
    // NOT declared here: the status a healthy peer returns, which is copied
    // through verbatim (`peer_client.cpp` assigns the peer's own status). No
    // finite `errors({...})` describes "whatever the peer said", and deciding
    // what the document should promise there is an aggregation-contract
    // question, not a documentation one.
    //
    // Guarded by `only_status_` as well, for the same reason as the blanket
    // 400/404/500 above and unlike the middleware refs below: the forward
    // happens *inside* the handler, so a route that declares itself
    // single-outcome (the data-categories / data-groups 501 stubs ignore the
    // request entirely and never resolve an entity) genuinely cannot reach it.
    if (aggregation_enabled_ && !route.only_status_ && entity_scoped(route.path_)) {
      add_error_ref("502");
      add_error_ref("503");
    }

    // Use explicit operationId if set, otherwise auto-generate camelCase from path
    if (!route.operation_id_.empty()) {
      operation["operationId"] = route.operation_id_;
    } else {
      // Auto-generate: strip {param} segments, camelCase remaining path segments
      // e.g., GET /faults/stream -> getFaultsStream
      std::string op_id = route.method_;
      bool next_upper = false;
      bool in_param = false;
      for (char c : route.path_) {
        if (c == '{') {
          in_param = true;
          continue;
        }
        if (c == '}') {
          in_param = false;
          continue;
        }
        if (in_param) {
          continue;
        }
        if (c == '/' || c == '-') {
          next_upper = true;
        } else {
          if (next_upper) {
            op_id += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            next_upper = false;
          } else {
            op_id += c;
          }
        }
      }
      operation["operationId"] = op_id;
    }

    // The role declared at the registration, in the shape a plugin's
    // `OperationDesc::requires_role` already emits - one scheme, the role as
    // its scope. `public_route()` emits the empty requirement list, which is
    // how OpenAPI says "this operation overrides the document-level
    // requirement and needs no token"; without it the `/auth/*` endpoints
    // would inherit a token requirement that is precisely what a caller uses
    // them to obtain.
    //
    // Emitted unconditionally here, and removed again by
    // `CapabilityGenerator::generate_impl` when `auth.enabled` is off. That
    // split is deliberate: whether the running gateway honours a role is a
    // property of the deployment, not of the registration, and answering it
    // once over the finished document is what keeps every producer - registry,
    // plugin fold, sub-documents - on one rule.
    if (route.role_declared_) {
      if (route.required_role_.has_value()) {
        operation["security"] =
            nlohmann::json::array({{{"bearerAuth", nlohmann::json::array({role_to_string(*route.required_role_)})}}});
      } else {
        operation["security"] = nlohmann::json::array();
      }
    }

    paths[route.path_][route.method_] = std::move(operation);
  }

  return paths;
}

// -----------------------------------------------------------------------------
// Projections over an emitted `paths` object
// -----------------------------------------------------------------------------

nlohmann::json paths_under(const nlohmann::json & paths, const std::string & prefix) {
  nlohmann::json subtree = nlohmann::json::object();
  if (!paths.is_object()) {
    return subtree;
  }
  for (const auto & [key, item] : paths.items()) {
    const bool same = key == prefix;
    // The segment boundary: the next character after the prefix must be the
    // separator, or `/data` swallows `/data-groups`.
    const bool beneath =
        key.size() > prefix.size() && key.compare(0, prefix.size(), prefix) == 0 && key[prefix.size()] == '/';
    if (same || beneath) {
      subtree[key] = item;
    }
  }
  return subtree;
}

void strip_entity_path_parameter(nlohmann::json & path_item, const std::string & param_name) {
  if (!path_item.is_object()) {
    return;
  }
  for (auto & operation : path_item) {
    // A path item can carry non-operation members (a path-level `summary`, a
    // vendor extension). Only an object can hold `parameters`.
    if (!operation.is_object() || !operation.contains("parameters") || !operation["parameters"].is_array()) {
      continue;
    }
    // Rebuilt rather than erased in place: nlohmann's array `erase` takes an
    // iterator and invalidates the ones past it, so removing while iterating
    // needs an index dance this does not.
    //
    // Members are read through `contains()` + `operator[]` to match the
    // convention the parameter scan in `to_openapi_paths()` set, and for the
    // reason recorded there - GCC inlines nlohmann's iterator dereference into
    // a -Wnull-dereference this build treats as an error. Whether it would fire
    // on this particular loop was not tested; following the convention costs
    // nothing.
    nlohmann::json kept = nlohmann::json::array();
    for (const auto & param : operation["parameters"]) {
      const bool answered_by_the_substitution = param.is_object() && param.contains("in") && param["in"] == "path" &&
                                                param.contains("name") && param["name"] == param_name;
      if (!answered_by_the_substitution) {
        kept.push_back(param);
      }
    }
    if (kept.empty()) {
      // An empty `parameters` array is legal but says nothing; dropping it
      // keeps a concrete entity's operation looking like what it is.
      operation.erase("parameters");
    } else {
      operation["parameters"] = std::move(kept);
    }
  }
}

// -----------------------------------------------------------------------------
// tags - collect unique tags
// -----------------------------------------------------------------------------

namespace {

/// Rewrite a route's cpp-httplib regex as an `AuthManager::matches_path`
/// pattern.
///
/// The regexes `to_regex_path()` builds contain exactly two constructs beyond
/// literal text - `([^/]+)` for a segment and `(.+)` for a slash-spanning tail
/// - plus the `/?$` anchor it appends. `add_raw_route()` supplies its own
/// regex, and the one caller (`docs_subtree`) uses the same two constructs with
/// a bare `$`. Both anchors are handled; anything else left over is reported by
/// the caller rather than guessed at, because a pattern that silently drops a
/// metacharacter would grant a path nobody wrote down.
///
/// The optional trailing slash the `/?` anchor accepts is deliberately NOT
/// mirrored into a second pattern, and the reason is behaviour rather than
/// cost: `GET /api/v1/health/` was 403 for a viewer before this derivation and
/// still is, so mirroring the slash would be a widening nobody asked for.
///
/// It would also double a set that this change already grew. The old table
/// held seven entries for ADMIN - the four `**` wildcards plus three
/// `POST:/api/v1/auth/*` literals that `POST:/api/v1/**` already covered, and
/// that the middleware exempts by prefix before the table is consulted at all -
/// and a literal list for everyone else; the derived one carries an entry per
/// route per granted role, and `check_authorization` scans a role's whole set,
/// compiling a `std::regex` per pattern, on any request the exact-match lookup
/// misses. Doubling it again would be paid on every such request. Neither
/// reason on its own would settle it; together they do.
///
/// The one exception is the root route, whose regex IS the anchor: it gets both
/// forms, because `/api/v1` without the slash is otherwise unreachable - it was
/// refused for every role, ADMIN included, before this derivation.
std::optional<std::string> regex_to_permission_pattern(const std::string & regex_path) {
  std::string body = regex_path;
  if (body.size() >= 3 && body.compare(body.size() - 3, 3, "/?$") == 0) {
    body.erase(body.size() - 3);
  } else if (!body.empty() && body.back() == '$') {
    body.pop_back();
  }

  std::string pattern;
  pattern.reserve(body.size());
  for (size_t i = 0; i < body.size();) {
    if (body.compare(i, 7, "([^/]+)") == 0) {
      pattern += '*';
      i += 7;
      continue;
    }
    if (body.compare(i, 4, "(.+)") == 0) {
      pattern += "**";
      i += 4;
      continue;
    }
    // Any regex metacharacter that is not one of the two known captures means
    // the route's URI was written in a form this translation does not model.
    if (std::string("()[]{}.+*?^$|\\").find(body[i]) != std::string::npos) {
      return std::nullopt;
    }
    pattern += body[i];
    ++i;
  }
  return pattern;
}

/// Every role at or above `role`. `UserRole` is declared weakest-first, so the
/// tail of the enumerator list is exactly "this role and stronger" - the
/// expansion `AuthConfig`'s lack of inheritance forces.
std::vector<UserRole> roles_at_or_above(UserRole role) {
  static const std::array<UserRole, 4> kAscending = {UserRole::VIEWER, UserRole::OPERATOR, UserRole::CONFIGURATOR,
                                                     UserRole::ADMIN};
  std::vector<UserRole> out;
  bool reached = false;
  for (UserRole candidate : kAscending) {
    reached = reached || candidate == role;
    if (reached) {
      out.push_back(candidate);
    }
  }
  return out;
}

}  // namespace

RoutePermissions RouteRegistry::route_permissions(const std::string & api_prefix) const {
  RoutePermissions permissions;
  // Every route, `hidden()` included. Hidden keeps a route out of the document,
  // not out of the router: the request still arrives and still meets this
  // table. The two hidden 405 stubs (bulk-data writes on areas and functions)
  // are the case in point - without an entry the caller is told "forbidden"
  // where the truth is "this entity type cannot host uploads".
  for (const auto & route : routes_) {
    // Skipped: a `public_route()` (the middleware answers it before the table
    // is consulted, so an entry would be dead weight on a set that is scanned
    // per request) and a route that declared nothing at all (which
    // validate_completeness() reports as an error).
    if (!route.role_declared_ || !route.required_role_.has_value()) {
      continue;
    }
    auto pattern = regex_to_permission_pattern(route.regex_path_);
    if (!pattern.has_value()) {
      continue;  // Reported by validate_completeness().
    }
    std::string method_upper = route.method_;
    std::transform(method_upper.begin(), method_upper.end(), method_upper.begin(), [](unsigned char c) {
      return std::toupper(c);
    });

    // Built once per route rather than once per granted role: the entry is the
    // same string in every role's set, and a route declaring VIEWER lands in
    // four of them.
    std::string entry;
    entry.reserve(method_upper.size() + api_prefix.size() + pattern->size() + 2);
    entry += method_upper;
    entry += ':';
    entry += api_prefix;
    entry += *pattern;

    std::vector<std::string> entries;
    entries.reserve(2);
    if (pattern->empty()) {
      // The root route: its regex is the anchor alone, so the entry above ends
      // at the bare prefix. Both spellings are reachable URIs.
      std::string with_slash = entry;
      with_slash += '/';
      entries.push_back(std::move(with_slash));
    }
    entries.push_back(std::move(entry));
    for (UserRole granted : roles_at_or_above(*route.required_role_)) {
      permissions[granted].insert(entries.begin(), entries.end());
    }
  }
  return permissions;
}

std::vector<std::string> RouteRegistry::to_endpoint_list(const std::string & api_prefix) const {
  std::vector<std::string> endpoints;
  endpoints.reserve(routes_.size());
  for (const auto & route : routes_) {
    // Uppercase the method
    std::string method = route.method_;
    std::transform(method.begin(), method.end(), method.begin(), [](unsigned char c) {
      return std::toupper(c);
    });
    std::string endpoint = method;
    endpoint += " ";
    endpoint += api_prefix;
    endpoint += route.path_;
    endpoints.push_back(std::move(endpoint));
  }
  return endpoints;
}

std::vector<std::string> RouteRegistry::tags() const {
  std::set<std::string> tag_set;
  for (const auto & route : routes_) {
    if (!route.tag_.empty()) {
      tag_set.insert(route.tag_);
    }
  }
  return {tag_set.begin(), tag_set.end()};
}

// -----------------------------------------------------------------------------
// validate_completeness - check all routes have required OpenAPI metadata
// -----------------------------------------------------------------------------

std::vector<ValidationIssue> RouteRegistry::validate_completeness() const {
  std::vector<ValidationIssue> issues;

  for (const auto & route : routes_) {
    std::string method_upper = route.method_;
    std::transform(method_upper.begin(), method_upper.end(), method_upper.begin(), [](unsigned char c) {
      return std::toupper(c);
    });
    std::string route_id = method_upper + " " + route.path_;

    // Checked ahead of the hidden-route skip, and that ordering is the whole
    // value of the check. `hidden()` removes a route from the document, not
    // from the router: the request still arrives, the middleware still consults
    // the permission table, and a route with no declaration has no entry there.
    // Fail-closed enforcement turns that into a 403 nobody wrote down - which
    // no test of the document could ever see, because the route is not in it.
    if (!route.role_declared_) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "No requires_role() or public_route() on the registration; with fail-closed "
                        "authorization the route answers 403 for every role below ADMIN"});
    } else if (route.required_role_.has_value() && !regex_to_permission_pattern(route.regex_path_).has_value()) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "The route's URI pattern '" + route.regex_path_ +
                            "' cannot be expressed as a permission pattern, so the declared role grants "
                            "nothing and the route answers 403 for every role below ADMIN"});
    }

    // Hidden routes are excluded from OpenAPI - skip the document checks
    if (route.hidden_) {
      continue;
    }

    // Every route must have a tag
    if (route.tag_.empty()) {
      issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing tag"});
    }

    // errors() only accepts 4xx/5xx. A success or redirect status passed there
    // was dropped, so report it rather than let the route quietly lose it.
    for (int code : route.rejected_error_codes_) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "errors() ignored non-error status " + std::to_string(code) +
                            "; use response() for success and redirect statuses"});
    }

    // A lock-guarded route without its 409 publishes a marker a client cannot
    // act on. lock_guarded() declares both, so the only way to lose one is a
    // later only_status(), which clears declared_errors_ and leaves the marker
    // standing. Reported rather than asserted: this is a release build.
    const bool declares_409 = std::count(route.declared_errors_.begin(), route.declared_errors_.end(), 409) > 0 ||
                              route.responses_.count(409) > 0;
    if (route.lock_guarded_ && !declares_409) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "lock_guarded() marker without a declared 409; a later only_status() clears "
                        "the status the marker promises"});
    }

    // A schema handed to the non-JSON response() overload was dropped rather
    // than attached to a media type it may not describe.
    for (int code : route.schema_on_non_json_statuses_) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "response() dropped the schema given for non-JSON status " + std::to_string(code) +
                            "; a non-JSON body is declared by media type, without a schema"});
    }

    // response_header() attaches to an already-declared status. One aimed at a
    // status this route never declares was dropped, so the header the handler
    // sets would be missing from the document with nothing to show for it.
    for (int code : route.undeclared_header_statuses_) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "response_header() targeted undeclared status " + std::to_string(code) +
                            "; declare the status first (success statuses come from the return type)"});
    }

    // success_schema() replaces the body shape of the one declared 2xx. With no
    // 2xx, or with several, there is nothing unambiguous to replace, so the call
    // was dropped and the route still publishes the schema derived from its
    // return type - the opposite of what the call site asked for.
    if (route.success_schema_without_single_2xx_) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "success_schema() was dropped: the route declares no single 2xx to attach to, or "
                        "that 2xx carries a media type no JSON Schema can describe"});
    }

    // body_example() attaches to a declared request body. On a route with none
    // the example is dropped rather than minting a body the route does not
    // take, which would tell a client to send a payload the handler ignores.
    if (route.body_example_.has_value() && !route.request_body_.has_value()) {
      issues.push_back({ValidationIssue::Severity::kError, route_id,
                        "body_example() was dropped: the route declares no request body to attach it to"});
    }

    // Check response schemas for non-DELETE methods
    if (route.method_ != "delete") {
      bool has_success_response_with_schema = false;
      for (const auto & [code, info] : route.responses_) {
        if (code >= 200 && code < 300 && !info.schema.empty()) {
          has_success_response_with_schema = true;
          break;
        }
      }

      // A 2xx whose body is a non-JSON media type is complete without a schema:
      // the media type IS the description of the body, and attaching a JSON
      // Schema to `application/octet-stream` or `text/event-stream` would
      // describe a shape nothing validates. Covers both escape hatches - the
      // SSE streams and the range-aware binary downloads - and it reads the
      // declaration the helper made rather than sniffing the summary string for
      // the word "stream", so a route cannot acquire the exemption by being
      // named a certain way.
      //
      // The media type has to actually be non-JSON. Accepting any non-empty
      // `content_types` would exempt a route that declares
      // `application/json` and no schema, which is precisely the case this
      // check exists for - a JSON body still owes a shape. That is also the
      // rule stated in dto_contract.rst and the one
      // `test_health::test_docs_spec_completeness` enforces on the served
      // document, so the two gates agree by construction rather than by
      // coincidence. Pinned by
      // RouteRegistryTest.SchemaLessJsonSuccessIsStillReported.
      bool has_non_json_success = false;
      for (const auto & [code, info] : route.responses_) {
        if (code < 200 || code >= 300) {
          continue;
        }
        for (const auto & media_type : info.content_types) {
          if (media_type != "application/json") {
            has_non_json_success = true;
            break;
          }
        }
        if (has_non_json_success) {
          break;
        }
      }

      // Body-less success statuses need no schema: 204 never carries a body,
      // and a 202 declared without one is an accepted asynchronous transition
      // (`Accepted<NoContent>`). Mirrors the "202 without content is OK"
      // branch of the served-document completeness gate.
      bool has_bodyless_success = route.responses_.count(204) > 0;
      if (auto accepted = route.responses_.find(202);
          accepted != route.responses_.end() && accepted->second.schema.empty()) {
        has_bodyless_success = true;
      }

      // Endpoints that only return errors (e.g., 405) don't need success schemas
      bool has_only_error_responses = !route.responses_.empty();
      for (const auto & [code, info] : route.responses_) {
        if (code < 400) {
          has_only_error_responses = false;
          break;
        }
      }

      if (!has_success_response_with_schema && !has_non_json_success && !has_bodyless_success &&
          !has_only_error_responses) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing response schema for success (2xx)"});
      }
    } else {
      // DELETE must have an explicit response code
      if (route.responses_.empty()) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "DELETE missing explicit response code"});
      }
    }

    // POST/PUT must have request_body, unless the registration overload already
    // said the route takes none. The body-less typed `put<TResponse>` is
    // reserved for payload-free state-machine kicks (`/updates/{id}/prepare`,
    // the lifecycle transitions); demanding a body schema of those reports 13
    // shipped routes that are correct as written. The body-less typed
    // `post<TResponse>` is NOT exempt: its contract is that the handler parses
    // a non-JSON body itself, so a missing declaration there is a real gap.
    if ((route.method_ == "post" || route.method_ == "put") && !route.request_body_.has_value() &&
        !route.takes_no_request_body_) {
      // Exception: endpoints returning 405 (method not allowed) don't need request body
      bool is_405 = route.responses_.count(405) > 0;
      // Exception: PUT endpoints returning 204 (e.g., log config) don't need request body schema
      // if they also don't have a success schema - they may accept a body but it's optional
      if (!is_405) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing request body definition"});
      }
    }

    // Warnings for missing summary/description
    if (route.summary_.empty()) {
      issues.push_back({ValidationIssue::Severity::kWarning, route_id, "Missing summary"});
    }
  }

  return issues;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
