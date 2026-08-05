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

#include "ros2_medkit_gateway/core/http/handlers/docs_handlers.hpp"

#include <string>
#include <utility>

#include "../../openapi/capability_generator.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"

#ifdef ENABLE_SWAGGER_UI
#include "swagger_ui_assets.hpp"
#endif

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// The error a typed docs handler returns. Same three fields the raw
/// `write_error` below fills in, so the two `/docs` routes answer with the
/// same body whichever way they are mounted.
ErrorInfo make_docs_error(int status, const std::string & code, const std::string & message) {
  ErrorInfo err;
  err.code = code;
  err.message = message;
  err.http_status = status;
  return err;
}

}  // namespace

void DocsHandlers::write_json_text(httplib::Response & res, const std::string & body) {
  http::detail::write_json_text(http::detail::FrameworkOrPluginAccess{}, res, body);
}

void DocsHandlers::write_error(httplib::Response & res, int status, const std::string & code,
                               const std::string & message) {
  ErrorInfo err;
  err.code = code;
  err.message = message;
  err.http_status = status;
  http::detail::write_generic_error(http::detail::FrameworkOrPluginAccess{}, res, err);
}

DocsHandlers::DocsHandlers(HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr,
                           const openapi::RouteRegistry * route_registry)
  : ctx_(ctx)
  , generator_(std::make_unique<openapi::CapabilityGenerator>(ctx, node, plugin_mgr, route_registry))
  , docs_enabled_(true) {
  // Read docs.enabled parameter if it has been declared
  try {
    docs_enabled_ = node.get_parameter("docs.enabled").as_bool();
  } catch (const std::exception &) {
    docs_enabled_ = true;  // Parameter may not be declared - default to enabled
  } catch (...) {
    docs_enabled_ = true;
  }
}

DocsHandlers::~DocsHandlers() = default;

http::Result<std::string> DocsHandlers::handle_docs_root(http::TypedRequest /*req*/) {
  if (!docs_enabled_) {
    return tl::unexpected(make_docs_error(501, ERR_NOT_IMPLEMENTED, "Capability description is disabled"));
  }

  auto spec = generator_->generate_serialized("/");
  if (!spec) {
    return tl::unexpected(make_docs_error(500, ERR_INTERNAL_ERROR, "Failed to generate capability description"));
  }
  return std::move(*spec);
}

void DocsHandlers::handle_docs_any_path(const httplib::Request & req, httplib::Response & res) {
  if (!docs_enabled_) {
    DocsHandlers::write_error(res, 501, ERR_NOT_IMPLEMENTED, "Capability description is disabled");
    return;
  }

  auto base_path = req.matches[1].str();
  auto spec = generator_->generate_serialized(base_path);
  if (!spec) {
    DocsHandlers::write_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                              "No capability description available for the requested path");
    return;
  }
  DocsHandlers::write_json_text(res, *spec);
}

#ifdef ENABLE_SWAGGER_UI

namespace {

// HTML page that loads embedded Swagger UI assets and points to /api/v1/docs
const std::string & get_embedded_html() {
  static const std::string html =
      "<!DOCTYPE html>\n"
      "<html lang=\"en\">\n"
      "<head>\n"
      "  <meta charset=\"UTF-8\">\n"
      "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
      "  <title>ROS 2 Medkit Gateway - API Documentation</title>\n"
      "  <link rel=\"stylesheet\" href=\"/api/v1/swagger-ui/swagger-ui.css\">\n"
      "  <style>\n"
      "    html { box-sizing: border-box; overflow-y: scroll; }\n"
      "    *, *:before, *:after { box-sizing: inherit; }\n"
      "    body { margin: 0; background: #fafafa; }\n"
      "  </style>\n"
      "</head>\n"
      "<body>\n"
      "  <div id=\"swagger-ui\"></div>\n"
      "  <script src=\"/api/v1/swagger-ui/swagger-ui-bundle.js\"></script>\n"
      "  <script src=\"/api/v1/swagger-ui/swagger-ui-standalone-preset.js\"></script>\n"
      "  <script>\n"
      "    SwaggerUIBundle({\n"
      "      url: '/api/v1/docs',\n"
      "      dom_id: '#swagger-ui',\n"
      "      presets: [SwaggerUIBundle.presets.apis, SwaggerUIStandalonePreset],\n"
      "      layout: 'StandaloneLayout'\n"
      "    });\n"
      "  </script>\n"
      "</body>\n"
      "</html>\n";
  return html;
}

}  // namespace

void DocsHandlers::handle_swagger_ui(const httplib::Request & /*req*/, httplib::Response & res) {
  if (!docs_enabled_) {
    DocsHandlers::write_error(res, 501, ERR_NOT_IMPLEMENTED, "Capability description is disabled");
    return;
  }

  res.set_header("Content-Security-Policy",
                 "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'");
  res.set_content(get_embedded_html(), "text/html");
}

void DocsHandlers::handle_swagger_asset(const httplib::Request & req, httplib::Response & res) {
  if (!docs_enabled_) {
    DocsHandlers::write_error(res, 501, ERR_NOT_IMPLEMENTED, "Capability description is disabled");
    return;
  }

  auto asset_name = req.matches[1].str();

  const unsigned char * data = nullptr;
  size_t size = 0;
  std::string content_type;

  if (asset_name == "swagger-ui-bundle.js") {
    data = swagger_ui::swagger_ui_bundle_js;
    size = swagger_ui::swagger_ui_bundle_js_size;
    content_type = "application/javascript";
  } else if (asset_name == "swagger-ui.css") {
    data = swagger_ui::swagger_ui_css;
    size = swagger_ui::swagger_ui_css_size;
    content_type = "text/css";
  } else if (asset_name == "swagger-ui-standalone-preset.js") {
    data = swagger_ui::swagger_ui_standalone_preset_js;
    size = swagger_ui::swagger_ui_standalone_preset_js_size;
    content_type = "application/javascript";
  } else {
    DocsHandlers::write_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Asset not found");
    return;
  }

  // Set cache headers - assets are versioned, safe to cache
  res.set_header("Cache-Control", "public, max-age=31536000, immutable");
  res.set_content(reinterpret_cast<const char *>(data), size, content_type);
}

#endif  // ENABLE_SWAGGER_UI

}  // namespace handlers
}  // namespace ros2_medkit_gateway
