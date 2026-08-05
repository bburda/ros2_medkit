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

#include <memory>

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/http/handler_result.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;
class PluginManager;

namespace openapi {
class CapabilityGenerator;
class RouteRegistry;
}  // namespace openapi

namespace handlers {

/**
 * @brief OpenAPI capability description endpoint handlers
 *
 * Handles:
 * - GET /docs - Root capability description (full OpenAPI spec)
 * - GET /{path}/docs - Context-scoped capability description
 */
class DocsHandlers {
 public:
  DocsHandlers(HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr,
               const openapi::RouteRegistry * route_registry = nullptr);
  ~DocsHandlers();

  /// GET /docs - Root capability description.
  ///
  /// Answers with the document already serialized. The generator caches
  /// documents in that form, so returning text is what lets a cache hit reach
  /// the response without a parsed DOM being copied on the way.
  http::Result<std::string> handle_docs_root(http::TypedRequest req);

  /// GET /{entity_path}/docs - Context-scoped capability description.
  ///
  /// Raw rather than typed: the route is mounted on a `(.+)/docs` regex, and
  /// the prefix it captures is a whole entity or resource path, which is not
  /// a path parameter the typed router's `{param}` grammar can express.
  void handle_docs_any_path(const httplib::Request & req, httplib::Response & res);

#ifdef ENABLE_SWAGGER_UI
  /// GET /swagger-ui - Serve Swagger UI HTML page
  void handle_swagger_ui(const httplib::Request & req, httplib::Response & res);

  /// GET /swagger-ui/{asset} - Serve embedded Swagger UI assets (JS/CSS)
  void handle_swagger_asset(const httplib::Request & req, httplib::Response & res);
#endif

 private:
  /// Write a 200 body from an already-serialized JSON document using the
  /// framework primitive. DocsHandlers is a friend of
  /// `FrameworkOrPluginAccess`; these helpers exist because
  /// `<entity-path>/docs` is a raw httplib handler (its `(.+)/docs$` regex
  /// shape does not map onto the typed router's OpenAPI path-template
  /// grammar) and, when Swagger UI is compiled in, so are its asset routes.
  static void write_json_text(httplib::Response & res, const std::string & body);

  /// Write a SOVD GenericError response using the framework primitive.
  static void write_error(httplib::Response & res, int status, const std::string & code, const std::string & message);

  HandlerContext & ctx_;
  std::unique_ptr<openapi::CapabilityGenerator> generator_;
  bool docs_enabled_{true};
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
