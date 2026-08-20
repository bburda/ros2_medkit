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
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/detail/forward_response_scope.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"

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

RouteEntry & RouteEntry::request_body(const std::string & desc, const nlohmann::json & schema,
                                      const std::string & content_type) {
  request_body_ = RequestBodyInfo{desc, schema, content_type};
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

RouteEntry & RouteEntry::error_renderer(ErrorRenderer renderer) {
  // The shared_ptr is captured by the typed handler wrapper closure; mutating
  // through it is the mechanism by which `.error_renderer(...)` called AFTER
  // `reg.post<...>(...)` still influences the closure's behaviour.
  *error_renderer_ = renderer;
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

// -----------------------------------------------------------------------------
// Escape-hatch routes (SSE / binary / static asset / docs)
// -----------------------------------------------------------------------------

RouteEntry & RouteRegistry::sse(const std::string & openapi_path,
                                std::function<http::Result<http::SseStream>(http::TypedRequest)> stream_factory) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  HandlerFn fn = [factory = std::move(stream_factory), renderer](const httplib::Request & req,
                                                                 httplib::Response & res) {
    // Install the forwarding scope so SSE factories that call
    // validate_entity_for_route can stream a proxied wire response for entities
    // owned by a remote peer. Without it the validator's Forwarded branch has
    // no response to write to. The scope ends before the chunked content
    // provider starts streaming - peer-forwarding is a synchronous decision
    // made up-front, never mid-stream.
    http::detail::ForwardResponseScope forward_scope(&res);
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
  // SSE has no JSON schema; mark it explicitly so validate_completeness skips
  // the success-schema check via its SSE-name heuristic.
  entry.response(200, "Server-Sent Events stream");
  return entry;
}

RouteEntry &
RouteRegistry::binary_download(const std::string & openapi_path,
                               std::function<http::Result<http::BinaryResponse>(http::TypedRequest)> handler) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  HandlerFn fn = [handler = std::move(handler), renderer](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope: entity-scoped binary downloads (bulk-data, scripts) on a
    // remote peer must proxy through validate_entity_for_route (see sse / wrap_body_less).
    http::detail::ForwardResponseScope forward_scope(&res);
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
  entry.response(200, "Binary download", nlohmann::json{{"type", "string"}, {"format", "binary"}});
  return entry;
}

RouteEntry & RouteRegistry::static_asset(const std::string & openapi_path,
                                         std::function<http::Result<http::StaticAsset>(http::TypedRequest)> handler) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  HandlerFn fn = [handler = std::move(handler), renderer](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope kept uniform across wrappers (static assets are not
    // entity-scoped, so this never forwards; see the comment at the top of this file).
    http::detail::ForwardResponseScope forward_scope(&res);
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
  entry.hidden();  // Static assets are not part of the documented JSON API.
  return entry;
}

RouteEntry & RouteRegistry::docs_endpoint(const std::string & openapi_path,
                                          std::function<http::Result<nlohmann::json>(http::TypedRequest)> handler) {
  auto renderer = std::make_shared<ErrorRenderer>(ErrorRenderer::kSovdGenericError);
  HandlerFn fn = [handler = std::move(handler), renderer](const httplib::Request & req, httplib::Response & res) {
    // Forwarding scope kept uniform across wrappers (see the comment at the top of this file).
    http::detail::ForwardResponseScope forward_scope(&res);
    http::TypedRequest typed_req(req);
    auto outcome = handler(typed_req);
    if (!outcome.has_value()) {
      write_typed_error(res, outcome.error(), renderer);
      return;
    }
    http::detail::write_json_body(http::detail::FrameworkOrPluginAccess{}, res, outcome.value(), 200);
  };
  auto & entry = add_route("get", openapi_path, std::move(fn));
  entry.error_renderer_ = renderer;
  entry.response(200, "OpenAPI specification document",
                 nlohmann::json{{"type", "object"}, {"additionalProperties", true}});
  entry.hidden();  // The docs spec endpoint describes itself externally.
  return entry;
}

RouteEntry & RouteRegistry::docs_subtree(const std::string & regex_pattern, HandlerFn handler) {
  auto & entry = add_raw_route("get", regex_pattern, regex_pattern, std::move(handler));
  entry.hidden();
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
// register_all - register all routes with cpp-httplib server
// -----------------------------------------------------------------------------

void RouteRegistry::register_all(httplib::Server & server, const std::string & api_prefix) const {
  for (const auto & route : routes_) {
    std::string full_path = api_prefix + route.regex_path_;
    const auto & handler = route.handler_;

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

    // Parameters
    if (!route.parameters_.empty()) {
      operation["parameters"] = route.parameters_;
    }

    // Also extract path parameters from the path template and add them
    // if they were not explicitly added via path_param()
    {
      std::set<std::string> explicit_params;
      for (const auto & p : route.parameters_) {
        if (p.value("in", "") == "path") {
          explicit_params.insert(p.value("name", ""));
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
          // Auto-generate path parameter with description
          static const std::unordered_map<std::string, std::string> kParamDescriptions = {
              {"area_id", "The area identifier"},
              {"component_id", "The component identifier"},
              {"app_id", "The app identifier"},
              {"function_id", "The function identifier"},
              {"data_id",
               "The data item identifier (ROS 2 topic name), or 'member_id:topic' when more than one member of "
               "the entity contributes an item under that name"},
              {"operation_id",
               "The operation identifier (service or action short name), or 'member_id:operation' when more than "
               "one member of the entity exposes that name"},
              {"execution_id", "The execution identifier"},
              {"config_id", "The configuration parameter identifier (ROS 2 parameter name)"},
              {"fault_code", "The fault code identifier"},
              {"subscription_id", "The cyclic subscription identifier"},
              {"category_id", "The bulk data category identifier"},
              {"file_id", "The bulk data file identifier"},
              {"update_id", "The software update identifier"},
              {"subarea_id", "The subarea identifier"},
              {"subcomponent_id", "The subcomponent identifier"},
              {"trigger_id", "The trigger identifier"},
              {"lock_id", "The lock identifier"},
              {"script_id", "The script identifier"},
          };
          nlohmann::json param;
          param["name"] = pname;
          param["in"] = "path";
          param["required"] = true;
          param["schema"] = {{"type", "string"}};
          auto desc_it = kParamDescriptions.find(pname);
          param["description"] = (desc_it != kParamDescriptions.end()) ? desc_it->second : "The " + pname + " value";
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
      operation["requestBody"]["required"] = true;
    }

    // Responses
    if (!route.responses_.empty()) {
      for (const auto & [code, info] : route.responses_) {
        std::string code_str = std::to_string(code);
        operation["responses"][code_str]["description"] = info.desc;
        if (!info.schema.empty()) {
          operation["responses"][code_str]["content"]["application/json"]["schema"] = info.schema;
        }
      }
    } else {
      // Default 200 response
      operation["responses"]["200"]["description"] = "Successful response";
    }

    // Add standard error responses as $ref to GenericError component.
    // Response-level $ref (not nested in content/schema) - the referenced
    // component is a complete response object with description and schema.
    auto add_error_ref = [&operation](const std::string & code) {
      if (!operation["responses"].contains(code)) {
        operation["responses"][code] = {{"$ref", "#/components/responses/GenericError"}};
      }
    };

    add_error_ref("400");
    add_error_ref("404");
    add_error_ref("500");

    if (auth_enabled_) {
      add_error_ref("401");
      add_error_ref("403");
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

    paths[route.path_][route.method_] = std::move(operation);
  }

  return paths;
}

// -----------------------------------------------------------------------------
// tags - collect unique tags
// -----------------------------------------------------------------------------

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
    // Hidden routes are excluded from OpenAPI - skip validation
    if (route.hidden_) {
      continue;
    }

    std::string method_upper = route.method_;
    std::transform(method_upper.begin(), method_upper.end(), method_upper.begin(), [](unsigned char c) {
      return std::toupper(c);
    });
    std::string route_id = method_upper + " " + route.path_;

    // Every route must have a tag
    if (route.tag_.empty()) {
      issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing tag"});
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

      // SSE endpoints use text/event-stream, not JSON schema - skip schema check
      // Convention: SSE endpoints have "SSE" or "stream" in summary
      bool is_sse = route.summary_.find("SSE") != std::string::npos ||
                    route.summary_.find("stream") != std::string::npos ||
                    route.summary_.find("Stream") != std::string::npos;

      // 204 No Content responses don't need a schema
      bool has_204 = route.responses_.count(204) > 0;

      // Endpoints that only return errors (e.g., 405) don't need success schemas
      bool has_only_error_responses = !route.responses_.empty();
      for (const auto & [code, info] : route.responses_) {
        if (code < 400) {
          has_only_error_responses = false;
          break;
        }
      }

      if (!has_success_response_with_schema && !is_sse && !has_204 && !has_only_error_responses) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing response schema for success (2xx)"});
      }
    } else {
      // DELETE must have an explicit response code
      if (route.responses_.empty()) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "DELETE missing explicit response code"});
      }
    }

    // POST/PUT must have request_body
    if ((route.method_ == "post" || route.method_ == "put") && !route.request_body_.has_value()) {
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
