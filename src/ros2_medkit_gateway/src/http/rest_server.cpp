// Copyright 2025 mfaferek93
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

#include "ros2_medkit_gateway/core/http/rest_server.hpp"

#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <type_traits>

#include "ros2_medkit_gateway/core/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/thread_pool_config.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"

#include "../openapi/route_registry.hpp"
#include "../openapi/schema_builder.hpp"

namespace ros2_medkit_gateway {

namespace {

void set_internal_server_error(httplib::Response & res, const std::string & details) {
  nlohmann::json error;
  error["error_code"] = ERR_INTERNAL_ERROR;
  error["message"] = "Internal server error";
  error["parameters"] = nlohmann::json::object();
  error["parameters"]["details"] = details;

  res.status = 500;
  res.set_content(error.dump(2), "application/json");
}

struct HttplibExceptionHandlerAdapter {
  // Newer cpp-httplib versions pass std::exception_ptr.
  void operator()(const httplib::Request & /*req*/, httplib::Response & res, const std::exception_ptr & ep) const {
    try {
      if (ep) {
        std::rethrow_exception(ep);
      }
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled empty exception_ptr");
      set_internal_server_error(res, "Unknown exception");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled exception: %s", e.what());
      set_internal_server_error(res, "An internal error occurred");
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unknown exception caught");
      set_internal_server_error(res, "An internal error occurred");
    }
  }

  // Older cpp-httplib versions pass std::exception directly.
  void operator()(const httplib::Request & /*req*/, httplib::Response & res, const std::exception & e) const {
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled exception: %s", e.what());
    set_internal_server_error(res, "An internal error occurred");
  }
};

static_assert(std::is_constructible_v<httplib::Server::ExceptionHandler, HttplibExceptionHandlerAdapter>,
              "cpp-httplib exception handler signature changed; update HttplibExceptionHandlerAdapter");

}  // namespace

RESTServer::RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
                       const AuthConfig & auth_config, const RateLimitConfig & rate_limit_config,
                       const TlsConfig & tls_config)
  : node_(node)
  , host_(host)
  , port_(port)
  , cors_config_(cors_config)
  , auth_config_(auth_config)
  , tls_config_(tls_config) {
  // Create HTTP/HTTPS server manager with a bounded request thread pool and a
  // bounded keep-alive idle timeout (issue #440). clamp_thread_count keeps the
  // pool in [1, 1024]: a pool of 0 would queue every request forever, and a
  // typo'd huge value would spawn that many OS threads. Each active SSE stream
  // holds one worker for its lifetime and each cold-/data wait parks one, so the
  // default pool covers sse.max_clients + data_provider.cold_wait_cap (bulk-data
  // downloads also hold a worker, uncounted); main.cpp warns at startup if the
  // pool is set below that sum. clamp_keep_alive_timeout keeps the keep-alive
  // timeout in [1, 3600]s: with a small pool, the cpp-httplib default (5s) lets
  // a burst of short-lived client connections pin every worker, so we shorten it
  // (default 2s) to recover workers quickly while retaining reuse.
  const auto http_thread_pool_size =
      clamp_thread_count(node_->get_parameter("server.http_thread_pool_size").as_int(), 1, 1024);
  const auto keep_alive_timeout_sec =
      clamp_keep_alive_timeout(node_->get_parameter("server.keep_alive_timeout_sec").as_int(), 1, 3600);
  http_server_ = std::make_unique<HttpServerManager>(tls_config_, http_thread_pool_size, keep_alive_timeout_sec);
  RCLCPP_INFO(rclcpp::get_logger("rest_server"),
              "HTTP request thread pool bounded to %zu workers (each active SSE stream holds one), "
              "keep-alive timeout %lds",
              http_thread_pool_size, static_cast<long>(keep_alive_timeout_sec));

  // Set maximum payload size for uploads (cpp-httplib default is 8MB)
  auto * srv = http_server_->get_server();
  if (srv) {
    size_t max_payload = node_->get_bulk_data_store() ? node_->get_bulk_data_store()->max_upload_bytes() : 0;
    if (max_payload > 0) {
      srv->set_payload_max_length(max_payload);
      RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Max payload length set to %zu bytes", max_payload);
    }
  }

  // Initialize auth manager and middleware if auth is enabled
  if (auth_config_.enabled) {
    auth_manager_ = std::make_unique<AuthManager>(auth_config_);
    auth_middleware_ = std::make_unique<AuthMiddleware>(auth_config_, auth_manager_.get());
    RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Authentication enabled - algorithm: %s, require_auth_for: %s",
                algorithm_to_string(auth_config_.jwt_algorithm).c_str(),
                auth_config_.require_auth_for == AuthRequirement::NONE    ? "none"
                : auth_config_.require_auth_for == AuthRequirement::WRITE ? "write"
                                                                          : "all");
  }

  // Initialize rate limiter if enabled
  if (rate_limit_config.enabled) {
    rate_limiter_ = std::make_unique<RateLimiter>(rate_limit_config);
    RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Rate limiting enabled - global: %d rpm, per-client: %d rpm",
                rate_limit_config.global_requests_per_minute, rate_limit_config.client_requests_per_minute);
  }

  // Create handler context and domain-specific handlers
  handler_ctx_ = std::make_unique<handlers::HandlerContext>(node_, cors_config_, auth_config_, tls_config_,
                                                            auth_manager_.get(), node_->get_bulk_data_store());

  // Create route registry before handlers that need it (HealthHandlers, DocsHandlers).
  // Routes are populated later in setup_routes(), but both handlers only access
  // the registry lazily at request time, so the pointer is valid.
  route_registry_ = std::make_unique<openapi::RouteRegistry>();
  route_registry_->set_auth_enabled(auth_config_.enabled);

  health_handlers_ = std::make_unique<handlers::HealthHandlers>(*handler_ctx_, route_registry_.get());
  discovery_handlers_ = std::make_unique<handlers::DiscoveryHandlers>(*handler_ctx_);
  data_handlers_ = std::make_unique<handlers::DataHandlers>(*handler_ctx_);
  lifecycle_handlers_ = std::make_unique<handlers::LifecycleHandlers>(
      *handler_ctx_, node_->get_plugin_manager(), std::make_shared<Ros2LifecycleStateReader>(node_));
  operation_handlers_ = std::make_unique<handlers::OperationHandlers>(*handler_ctx_);
  config_handlers_ = std::make_unique<handlers::ConfigHandlers>(*handler_ctx_);
  fault_handlers_ = std::make_unique<handlers::FaultHandlers>(*handler_ctx_);
  log_handlers_ = std::make_unique<handlers::LogHandlers>(*handler_ctx_);
  auth_handlers_ = std::make_unique<handlers::AuthHandlers>(*handler_ctx_);
  sse_client_tracker_ = node_->get_sse_client_tracker();
  sse_fault_handler_ = std::make_unique<handlers::SSEFaultHandler>(*handler_ctx_, sse_client_tracker_);
  bulkdata_handlers_ = std::make_unique<handlers::BulkDataHandlers>(*handler_ctx_);
  auto max_duration_sec = static_cast<int>(node_->get_parameter("sse.max_duration_sec").as_int());
  if (max_duration_sec <= 0) {
    RCLCPP_WARN(node_->get_logger(), "sse.max_duration_sec must be > 0, using default 3600");
    max_duration_sec = 3600;
  }
  cyclic_sub_handlers_ = std::make_unique<handlers::CyclicSubscriptionHandlers>(
      *handler_ctx_, *node_->get_subscription_manager(), *node_->get_sampler_registry(),
      *node_->get_transport_registry(), max_duration_sec);

  if (node_->get_update_manager()) {
    update_handlers_ = std::make_unique<handlers::UpdateHandlers>(*handler_ctx_, node_->get_update_manager());
  }

  lock_handlers_ = std::make_unique<handlers::LockHandlers>(*handler_ctx_, node_->get_lock_manager());

  if (node_->get_script_manager()) {
    script_handlers_ = std::make_unique<handlers::ScriptHandlers>(*handler_ctx_, node_->get_script_manager());
  }

  docs_handlers_ = std::make_unique<handlers::DocsHandlers>(*handler_ctx_, *node_, node_->get_plugin_manager(),
                                                            route_registry_.get());

  // Set up global error handlers for SOVD GenericError compliance
  setup_global_error_handlers();
  // Set up pre-routing handler for CORS and Authentication
  setup_pre_routing_handler();
  setup_routes();

  // Register plugin custom routes
  if (node_->get_plugin_manager()) {
    auto * plugin_srv = http_server_->get_server();
    if (plugin_srv) {
      node_->get_plugin_manager()->register_routes(*plugin_srv, API_BASE_PATH);
    }
  }
}

void RESTServer::setup_pre_routing_handler() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    return;
  }

  // Set up pre-routing handler for CORS and Authentication
  // This handler runs before any route handler
  srv->set_pre_routing_handler([this](const httplib::Request & req, httplib::Response & res) {
    // 1. Handle CORS (existing logic)
    if (cors_config_.enabled) {
      std::string origin = req.get_header_value("Origin");
      bool origin_allowed = !origin.empty() && is_origin_allowed(origin);

      if (origin_allowed) {
        set_cors_headers(res, origin);
        // Add Authorization header to allowed headers for CORS
        if (auth_config_.enabled) {
          std::string current_headers = res.get_header_value("Access-Control-Allow-Headers");
          if (!current_headers.empty() && current_headers.find("Authorization") == std::string::npos) {
            res.set_header("Access-Control-Allow-Headers", current_headers + ", Authorization");
          }
        }
      }

      // 2. Handle preflight OPTIONS requests
      if (req.method == "OPTIONS") {
        if (origin_allowed) {
          res.set_header("Access-Control-Max-Age", std::to_string(cors_config_.max_age_seconds));
          res.status = 204;
        } else {
          res.status = 403;
        }
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    // 3. Rate limiting check. If rejected, return Handled (CORS headers already set)
    if (rate_limiter_ && rate_limiter_->is_enabled() && req.method != "OPTIONS") {
      auto rl_result = rate_limiter_->check(req.remote_addr, req.path);
      RateLimiter::apply_headers(rl_result, res);
      if (!rl_result.allowed) {
        RateLimiter::apply_rejection(rl_result, res);
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    // 1. Handle CORS (existing logic)

    // Handle Authentication if enabled
    if (auth_middleware_ && auth_middleware_->is_enabled()) {
      // Use AuthMiddleware to process the request
      auto auth_request = AuthMiddleware::from_httplib_request(req);
      auto result = auth_middleware_->process(auth_request);

      if (!result.allowed) {
        AuthMiddleware::apply_to_response(result, res);
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    return httplib::Server::HandlerResponse::Unhandled;
  });
}

void RESTServer::setup_global_error_handlers() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    return;
  }

  // Global error handler - catches HTTP errors like 404 Not Found
  // Only set error content if no content has been set by a handler
  srv->set_error_handler([](const httplib::Request & /*req*/, httplib::Response & res) {
    // If the response already has content (from a handler's send_error), don't overwrite it
    if (!res.body.empty()) {
      return;
    }

    nlohmann::json error;
    error["error_code"] = ERR_RESOURCE_NOT_FOUND;
    error["message"] = "Resource not found";
    error["parameters"] = nlohmann::json::object();
    error["parameters"]["status"] = res.status;

    res.set_content(error.dump(2), "application/json");
  });

  // Global exception handler - catches unhandled exceptions in route handlers
  srv->set_exception_handler(HttplibExceptionHandlerAdapter{});
}

void RESTServer::set_trigger_handlers(TriggerManager & trigger_mgr) {
  trigger_handlers_ = std::make_unique<handlers::TriggerHandlers>(*handler_ctx_, trigger_mgr, sse_client_tracker_);
}

void RESTServer::set_aggregation_manager(AggregationManager * mgr) {
  handler_ctx_->set_aggregation_manager(mgr);
}

RESTServer::~RESTServer() {
  stop();
}

void RESTServer::setup_routes() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    throw std::runtime_error("No server instance available for route setup");
  }

  // === Docs routes - MUST be before data/config item routes to avoid (.+) capture collision ===
  // These use special regex patterns that don't map cleanly to OpenAPI {param} style,
  // so they are registered directly with the server rather than through the route registry.
  srv->Get(api_path("/docs"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_docs_root(req, res);
  });
  srv->Get((api_path("") + R"((.+)/docs$)"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_docs_any_path(req, res);
  });

#ifdef ENABLE_SWAGGER_UI
  // Swagger UI - interactive API documentation browser
  srv->Get(api_path("/swagger-ui"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_swagger_ui(req, res);
  });
  srv->Get(api_path(R"(/swagger-ui/([^/]+))"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_swagger_asset(req, res);
  });
#endif

  // === Config-less fault-trigger (threshold-rule) routes (issue #235) ===
  // Registered through RouteRegistry's raw() escape hatch (no typed DTO), so
  // the CRUD is part of the generated OpenAPI spec / Swagger UI / endpoint
  // list instead of being invisible to /api/v1/docs. Mounted at
  // .../fault-triggers - a sibling of the SOVD notification `/triggers`
  // collection, never overloading it. GET is public; POST/DELETE follow the
  // gateway's write-auth policy (global pre-routing middleware).
  {
    auto ft_json_error = [](httplib::Response & res, int status, const std::string & message,
                            const char * error_code = nullptr) {
      nlohmann::json err;
      err["error_code"] = error_code ? error_code : (status == 404 ? ERR_RESOURCE_NOT_FOUND : ERR_INVALID_PARAMETER);
      err["message"] = message;
      res.status = status;
      res.set_content(err.dump(2), "application/json");
    };

    route_registry_
        ->raw("get", "/apps/{app_id}/fault-triggers",
              [this, ft_json_error](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_json_error(res, 404, "fault-trigger engine is not enabled");
                  return;
                }
                const std::string app_id = req.matches.size() > 1 ? req.matches[1].str() : std::string{};
                nlohmann::json items = nlohmann::json::array();
                for (const auto & r : engine->list(app_id)) {
                  items.push_back(FaultTriggerEngine::rule_to_json(r));
                }
                res.status = 200;
                res.set_content(nlohmann::json{{"items", items}}.dump(2), "application/json");
              })
        .tag("FaultTriggers")
        .summary("List fault-trigger rules")
        .description(
            "Threshold rules on the app's discovered data points; each fires a fault on cross "
            "and auto-clears on recovery.")
        .operation_id("listFaultTriggers")
        .path_param("app_id", "App (entity) the rules are scoped to")
        .response(200, "Rule list",
                  nlohmann::json{{"type", "object"},
                                 {"properties", {{"items", {{"type", "array"}, {"items", {{"type", "object"}}}}}}}});

    route_registry_
        ->raw("post", "/apps/{app_id}/fault-triggers",
              [this, ft_json_error](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_json_error(res, 404, "fault-trigger engine is not enabled");
                  return;
                }
                const std::string app_id = req.matches.size() > 1 ? req.matches[1].str() : std::string{};
                nlohmann::json body;
                try {
                  body = nlohmann::json::parse(req.body);
                } catch (const nlohmann::json::exception &) {
                  // Malformed request, not semantic validation - ERR_INVALID_REQUEST.
                  ft_json_error(res, 400, "request body is not valid JSON", ERR_INVALID_REQUEST);
                  return;
                }
                auto created = engine->create(app_id, body);
                if (!created) {
                  ft_json_error(res, created.error().first, created.error().second);
                  return;
                }
                res.status = 201;
                res.set_content(FaultTriggerEngine::rule_to_json(*created).dump(2), "application/json");
              })
        .tag("FaultTriggers")
        .summary("Create a fault-trigger rule")
        .description(
            "Body: data_name, operator (>, <, >=, <=, ==), threshold, fault_code, severity "
            "(INFO|WARNING|ERROR|CRITICAL), optional active. fault_code must be unique across "
            "all rules (409 on duplicates).")
        .operation_id("createFaultTrigger")
        .path_param("app_id", "App (entity) to scope the rule to")
        .request_body("Fault-trigger rule definition",
                      nlohmann::json{{"type", "object"}, {"additionalProperties", true}})
        .response(201, "Created rule", nlohmann::json{{"type", "object"}})
        // 400/404 come from the registry's automatic response-level
        // GenericError $ref; only 409 needs a manual declaration.
        .response(409, "fault_code already used by another rule",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}});

    route_registry_
        ->raw("delete", "/apps/{app_id}/fault-triggers/{trigger_id}",
              [this, ft_json_error](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_json_error(res, 404, "fault-trigger engine is not enabled");
                  return;
                }
                const std::string app_id = req.matches.size() > 1 ? req.matches[1].str() : std::string{};
                const std::string id = req.matches.size() > 2 ? req.matches[2].str() : std::string{};
                if (!engine->remove(app_id, id)) {
                  ft_json_error(res, 404, "no such fault-trigger '" + id + "'");
                  return;
                }
                res.status = 204;
              })
        .tag("FaultTriggers")
        .summary("Delete a fault-trigger rule")
        .description(
            "Removes the rule; a currently-asserted fault from it is cleared "
            "(correlation cascade skipped).")
        .operation_id("deleteFaultTrigger")
        .path_param("app_id", "App (entity) the rule is scoped to")
        .path_param("trigger_id", "Rule id as returned on create")
        .response(204, "Deleted");
  }

  auto & reg = *route_registry_;
  using SB = openapi::SchemaBuilder;

  auto capitalize = [](const std::string & s) -> std::string {
    if (s.empty()) {
      return s;
    }
    std::string result = s;
    result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
    return result;
  };

  // === Server endpoints ===
  // PR-403 commit 16: migrated to typed reg.get<T>. The framework auto-fills
  // .response<T>(200, "") from the template parameter, so no manual response
  // schema declaration is needed below.
  reg.get<dto::Health>("/health",
                       [this](http::TypedRequest req) -> http::Result<dto::Health> {
                         return health_handlers_->get_health(req);
                       })
      .tag("Server")
      .summary("Health check")
      .description("Returns gateway health status.")
      .operation_id("getHealth");

  reg.get<dto::RootOverview>("/",
                             [this](http::TypedRequest req) -> http::Result<dto::RootOverview> {
                               return health_handlers_->get_root(req);
                             })
      .tag("Server")
      .summary("API overview")
      .description("Returns gateway metadata, available endpoints, and capabilities.")
      .operation_id("getRoot");

  reg.get<dto::VersionInfo>("/version-info",
                            [this](http::TypedRequest req) -> http::Result<dto::VersionInfo> {
                              return health_handlers_->get_version_info(req);
                            })
      .tag("Server")
      .summary("SOVD version information")
      .description("Returns SOVD specification version and vendor info.")
      .operation_id("getVersionInfo");

  // === Discovery - entity collections ===
  // PR-403 commit 17: migrated discovery_handlers to the typed reg.get<T> API.
  // The framework auto-fills the response<T>(200,"") OpenAPI metadata from
  // each handler's TResponse, so the per-route response() lines drop here and
  // in every discovery route below.
  reg.get<dto::Collection<dto::AreaListItem>>(
         "/areas",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AreaListItem>> {
           return discovery_handlers_->get_areas(req);
         })
      .tag("Discovery")
      .summary("List areas")
      .description("Lists all discovered areas in the system.")
      .operation_id("listAreas");

  reg.get<dto::Collection<dto::AppListItem>>(
         "/apps",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AppListItem>> {
           return discovery_handlers_->get_apps(req);
         })
      .tag("Discovery")
      .summary("List apps")
      .description("Lists all discovered apps (ROS 2 nodes) in the system.")
      .operation_id("listApps");

  reg.get<dto::Collection<dto::ComponentListItem>>(
         "/components",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
           return discovery_handlers_->get_components(req);
         })
      .tag("Discovery")
      .summary("List components")
      .description("Lists all discovered components in the system.")
      .operation_id("listComponents");

  reg.get<dto::Collection<dto::FunctionListItem>>(
         "/functions",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::FunctionListItem>> {
           return discovery_handlers_->get_functions(req);
         })
      .tag("Discovery")
      .summary("List functions")
      .description("Lists all discovered functions in the system.")
      .operation_id("listFunctions");

  // === Per-entity-type resource routes ===
  // Entity types: areas, components, apps, functions
  // For each entity type, register data, operations, configurations, faults, logs, bulk-data,
  // and discovery relationship endpoints.

  // PR-403 commit 17: the per-entity detail handlers now return distinct
  // typed DTOs (AreaDetail / ComponentDetail / AppDetail / FunctionDetail),
  // so the detail registration moved into the loop body below as a small
  // if/else over et.type. The shared resource-collection routes (data,
  // operations, configurations, faults, ...) still iterate uniformly.
  struct EntityHandlers {
    const char * type;
    const char * singular;
  };
  std::vector<EntityHandlers> entity_types = {
      {"areas", "area"},
      {"components", "component"},
      {"apps", "app"},
      {"functions", "function"},
  };

  for (const auto & et : entity_types) {
    std::string base = std::string("/") + et.type;
    std::string entity_path = base + "/{" + et.singular + "_id}";

    // --- Data ---
    //
    // PR-403 commit 28: 5 data routes migrate to the typed RouteRegistry API.
    // The list endpoint uses the typed `fan_out_collection<DataItem>` from
    // commit 7 (per-item wire shape now enforced by `JsonReader<DataItem>`,
    // closing the issue #338 gap on this endpoint). Read returns
    // `DataValue` whose payload is an opaque object (live ROS message JSON).
    // Write uses body-less typed PUT and parses the body manually: ROS path
    // enforces the strict `DataWriteRequest` shape, plugin path accepts
    // free-form JSON (UDS sends a bare hex-encoded string, OPC-UA writes
    // vendor-specific objects) so a single framework-level body schema would
    // break plugin compatibility. The OpenAPI request-body schema is attached
    // manually below. The 501 stubs (data-categories / data-groups) ride on
    // the same typed-error renderer; their success type is a dummy DTO.
    //
    // Data item (specific topic) - MUST be before data collection to avoid (.+) capture
    reg.get<dto::DataValue>(entity_path + "/data/{data_id}",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->get_data_item(req);
                            })
        .tag("Data")
        .summary(std::string("Get data item for ") + et.singular)
        .description(std::string("Returns the latest value from a ROS 2 topic for this ") + et.singular + ".")
        .operation_id(std::string("get") + capitalize(et.singular) + "DataItem");

    reg.put<dto::DataValue>(entity_path + "/data/{data_id}",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->put_data_item(req);
                            })
        .tag("Data")
        .summary(std::string("Write data item for ") + et.singular)
        .description(std::string("Publishes a value to a ROS 2 topic on this ") + et.singular + ".")
        .request_body("Data value to write", SB::ref("DataWriteRequest"))
        .operation_id(std::string("put") + capitalize(et.singular) + "DataItem");

    // Data-categories (returns 501 - not yet implemented)
    reg.get<dto::DataValue>(entity_path + "/data-categories",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->data_categories(req);
                            })
        .tag("Data")
        .summary(std::string("List data categories for ") + et.singular)
        .description(std::string("Lists available data categories for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "DataCategories");

    // Data-groups (returns 501 - not yet implemented)
    reg.get<dto::DataValue>(entity_path + "/data-groups",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->data_groups(req);
                            })
        .tag("Data")
        .summary(std::string("List data groups for ") + et.singular)
        .description(std::string("Lists available data groups for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "DataGroups");

    // Data collection (all topics). Returns the opaque `DataListResult` envelope
    // (mirroring the fault list route): the runtime branch builds a typed
    // `Collection<DataItem, DataListXMedkit>` and serializes it into the envelope
    // (wire shape unchanged), while the plugin branch passes the provider's
    // free-form item shape through verbatim - so vendor per-item fields (OPC-UA
    // value/unit/data_type/writable) are no longer dropped by a typed re-parse.
    reg.get<dto::DataListResult>(entity_path + "/data",
                                 [this](http::TypedRequest req) -> http::Result<dto::DataListResult> {
                                   return data_handlers_->list_data(req);
                                 })
        .tag("Data")
        .summary(std::string("List data items for ") + et.singular)
        .description(std::string("Lists all data items (ROS 2 topics) available on this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Data");

    // --- Operations ---
    //
    // PR-403 commit 27: 7 operation routes migrate to the typed RouteRegistry
    // API. The POST executions route uses
    // `post_alternates<ExecutionCreateRequest, OperationExecutionResult,
    // ExecutionCreateAsync>` so the framework picks 200 for the synchronous
    // service branch (OperationExecutionResult) or 202 for the asynchronous
    // action branch (ExecutionCreateAsync); the ResponseAttachments channel
    // appends the Location header on the 202 path. The list_executions
    // endpoint returns the typed `Collection<ExecutionId>` (renamed
    // OperationExecutionList on the wire so the schema name stays stable).
    reg.get<dto::Collection<dto::OperationItem>>(
           entity_path + "/operations",
           [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::OperationItem>> {
             return operation_handlers_->list_operations(req);
           })
        .tag("Operations")
        .summary(std::string("List operations for ") + et.singular)
        .description(std::string("Lists all ROS 2 services and actions available on this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Operations");

    reg.get<dto::OperationDetail>(entity_path + "/operations/{operation_id}",
                                  [this](http::TypedRequest req) -> http::Result<dto::OperationDetail> {
                                    return operation_handlers_->get_operation(req);
                                  })
        .tag("Operations")
        .summary(std::string("Get operation details for ") + et.singular)
        .description(std::string("Returns operation details including request/response schema for this ") +
                     et.singular + ".")
        .operation_id(std::string("get") + capitalize(et.singular) + "Operation");

    // Execution endpoints
    reg.post_alternates<dto::ExecutionCreateRequest, dto::OperationExecutionResult, dto::ExecutionCreateAsync>(
           entity_path + "/operations/{operation_id}/executions",
           std::function<http::Result<std::pair<std::variant<dto::OperationExecutionResult, dto::ExecutionCreateAsync>,
                                                http::ResponseAttachments>>(http::TypedRequest,
                                                                            dto::ExecutionCreateRequest)>{
               [this](http::TypedRequest req, dto::ExecutionCreateRequest body)
                   -> http::Result<std::pair<std::variant<dto::OperationExecutionResult, dto::ExecutionCreateAsync>,
                                             http::ResponseAttachments>> {
                 return operation_handlers_->create_execution(req, std::move(body));
               }})
        .tag("Operations")
        .summary(std::string("Start operation execution for ") + et.singular)
        .description("Starts a new execution. Returns 200 for synchronous, 202 for asynchronous operations.")
        .operation_id(std::string("execute") + capitalize(et.singular) + "Operation");

    reg.get<dto::Collection<dto::ExecutionId>>(
           entity_path + "/operations/{operation_id}/executions",
           [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ExecutionId>> {
             return operation_handlers_->list_executions(req);
           })
        .tag("Operations")
        .summary(std::string("List operation executions for ") + et.singular)
        .description(std::string("Lists all executions of an operation on this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Executions");

    reg.get<dto::OperationExecution>(entity_path + "/operations/{operation_id}/executions/{execution_id}",
                                     [this](http::TypedRequest req) -> http::Result<dto::OperationExecution> {
                                       return operation_handlers_->get_execution(req);
                                     })
        .tag("Operations")
        .summary(std::string("Get execution status for ") + et.singular)
        .description("Returns the current status and result of a specific execution.")
        .operation_id(std::string("get") + capitalize(et.singular) + "Execution");

    reg.put<dto::ExecutionUpdateRequest, dto::OperationExecution>(
           entity_path + "/operations/{operation_id}/executions/{execution_id}",
           std::function<http::Result<std::pair<dto::OperationExecution, http::ResponseAttachments>>(
               http::TypedRequest, dto::ExecutionUpdateRequest)>{
               [this](http::TypedRequest req, const dto::ExecutionUpdateRequest & body)
                   -> http::Result<std::pair<dto::OperationExecution, http::ResponseAttachments>> {
                 return operation_handlers_->update_execution(req, body);
               }})
        .tag("Operations")
        .summary(std::string("Update execution for ") + et.singular)
        .description("Sends a control command to a running execution.")
        .response(202, "Accepted (asynchronous control)", SB::ref("OperationExecution"))
        // 400/404/500 come from the registry's automatic response-level
        // GenericError $ref; the remaining statuses this route can return
        // need manual declarations, or the generated SDK has no branch for
        // them (issue #576).
        .response(409, "Execution is still running (precondition-not-fulfilled)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .response(503, "Action server unavailable (x-medkit-ros2-action-unavailable)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .response(504, "No cancel response in time - outcome unknown (not-responding)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .operation_id(std::string("update") + capitalize(et.singular) + "Execution");

    reg.del<http::NoContent>(entity_path + "/operations/{operation_id}/executions/{execution_id}",
                             [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                               return operation_handlers_->cancel_execution(req);
                             })
        .tag("Operations")
        .summary(std::string("Cancel execution for ") + et.singular)
        .description("Cancels a running execution.")
        // Same cancel-outcome statuses as PUT-stop (issue #576);
        // 400/404/500 are auto-declared by the registry.
        .response(503, "Action server unavailable (x-medkit-ros2-action-unavailable)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .response(504, "No cancel response in time - outcome unknown (not-responding)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .operation_id(std::string("cancel") + capitalize(et.singular) + "Execution");

    // --- Configurations ---
    //
    // PR-403 commit 26: 5 config routes migrate to the typed RouteRegistry
    // API. The list endpoint uses the typed
    // `fan_out_collection<ConfigurationMetaData>` from commit 7 for peer
    // aggregation (per-item wire shape now enforced by
    // `JsonReader<ConfigurationMetaData>`, closing the issue #338 gap on this
    // endpoint). The delete-all endpoint uses
    // `del_alternates<NoContent, ConfigurationDeleteMultiStatus>` so the
    // framework picks 204 on full success or 207 on partial success based on
    // the active variant alternative. Wire format unchanged byte-for-byte.
    reg.get<dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>>(
           entity_path + "/configurations",
           [this](http::TypedRequest req)
               -> http::Result<dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>> {
             return config_handlers_->list_configurations(req);
           })
        .tag("Configuration")
        .summary(std::string("List configurations for ") + et.singular)
        .description(std::string("Lists all ROS 2 node parameters for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Configurations");

    reg.get<dto::ConfigurationReadValue>(entity_path + "/configurations/{config_id}",
                                         [this](http::TypedRequest req) -> http::Result<dto::ConfigurationReadValue> {
                                           return config_handlers_->get_configuration(req);
                                         })
        .tag("Configuration")
        .summary(std::string("Get specific configuration for ") + et.singular)
        .description(std::string("Returns a specific ROS 2 node parameter for this ") + et.singular + ".")
        .operation_id(std::string("get") + capitalize(et.singular) + "Configuration");

    reg.put<dto::ConfigurationWriteRequest, dto::ConfigurationReadValue>(
           entity_path + "/configurations/{config_id}",
           [this](http::TypedRequest req,
                  dto::ConfigurationWriteRequest body) -> http::Result<dto::ConfigurationReadValue> {
             return config_handlers_->set_configuration(req, std::move(body));
           })
        .tag("Configuration")
        .summary(std::string("Set configuration for ") + et.singular)
        .description(std::string("Sets a ROS 2 node parameter value for this ") + et.singular + ".")
        .operation_id(std::string("set") + capitalize(et.singular) + "Configuration");

    reg.del<http::NoContent>(entity_path + "/configurations/{config_id}",
                             [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                               return config_handlers_->delete_configuration(req);
                             })
        .tag("Configuration")
        .summary(std::string("Delete configuration for ") + et.singular)
        .description(std::string("Resets a configuration parameter to its default for this ") + et.singular + ".")
        .operation_id(std::string("delete") + capitalize(et.singular) + "Configuration");

    reg.del_alternates<http::NoContent, dto::ConfigurationDeleteMultiStatus>(
           entity_path + "/configurations",
           std::function<http::Result<std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>>(
               http::TypedRequest)>{
               [this](http::TypedRequest req)
                   -> http::Result<std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>> {
                 return config_handlers_->delete_all_configurations(req);
               }})
        .tag("Configuration")
        .summary(std::string("Delete all configurations for ") + et.singular)
        .description(std::string("Resets all configuration parameters for this ") + et.singular + ".")
        .operation_id(std::string("deleteAll") + capitalize(et.singular) + "Configurations");

    // --- Faults ---
    //
    // PR-403 commit 29: 4 per-entity fault routes migrate to the typed
    // RouteRegistry API. The list + detail endpoints emit `FaultListResult`
    // and `FaultDetailResult` opaque envelopes so the per-entity-type
    // x-medkit shape (FaultListXMedkit for App / global, FaultListAggXMedkit
    // for Function / Component / Area) stays byte-identical with the legacy
    // path while the typed router still owns wire framing. The single-fault
    // DELETE uses `del_alternates<NoContent, FaultClearResult>` so the ROS
    // path returns 204 while the plugin path keeps its 200 + ack-body shape.
    // The bulk-clear DELETE returns NoContent unconditionally - the legacy
    // plugin branch also emitted 204 after iterating the per-fault clears.
    reg.get<dto::FaultListResult>(entity_path + "/faults",
                                  [this](http::TypedRequest req) -> http::Result<dto::FaultListResult> {
                                    return fault_handlers_->list_faults(req);
                                  })
        .tag("Faults")
        .summary(std::string("List faults for ") + et.singular)
        .description(std::string("Returns all active faults reported by this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Faults")
        .query<dto::FaultEntityListQuery>();

    reg.get<dto::FaultDetailResult>(entity_path + "/faults/{fault_code}",
                                    [this](http::TypedRequest req) -> http::Result<dto::FaultDetailResult> {
                                      return fault_handlers_->get_fault(req);
                                    })
        .tag("Faults")
        .summary(std::string("Get specific fault for ") + et.singular)
        .description("Returns fault details including SOVD status, environment data, and rosbag snapshots.")
        .operation_id(std::string("get") + capitalize(et.singular) + "Fault");

    reg.del_alternates<http::NoContent, dto::FaultClearResult>(
           entity_path + "/faults/{fault_code}",
           std::function<http::Result<std::variant<http::NoContent, dto::FaultClearResult>>(http::TypedRequest)>{
               [this](http::TypedRequest req) -> http::Result<std::variant<http::NoContent, dto::FaultClearResult>> {
                 return fault_handlers_->clear_fault(req);
               }})
        .tag("Faults")
        .summary(std::string("Clear fault for ") + et.singular)
        .description(std::string("Clears a specific fault for this ") + et.singular + ".")
        .operation_id(std::string("clear") + capitalize(et.singular) + "Fault");

    reg.del<http::NoContent>(entity_path + "/faults",
                             [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                               return fault_handlers_->clear_all_faults(req);
                             })
        .tag("Faults")
        .summary(std::string("Clear all faults for ") + et.singular)
        .description(std::string("Clears all faults for this ") + et.singular + ".")
        .operation_id(std::string("clearAll") + capitalize(et.singular) + "Faults");

    // --- Logs ---
    // PR-403 commit 23: 3 log routes migrated to typed RouteRegistry API.
    // The list endpoint uses the typed `fan_out_collection<LogEntry>` peer
    // merge from commit 7. The framework auto-fills response<TResponse> and
    // request_body<TBody> from the template parameters so the per-route
    // builder calls drop here.
    reg.get<dto::Collection<dto::LogEntry, dto::LogListXMedkit>>(
           entity_path + "/logs",
           [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::LogEntry, dto::LogListXMedkit>> {
             return log_handlers_->get_logs(req);
           })
        .tag("Logs")
        .summary(std::string("Query log entries for ") + et.singular)
        .description(std::string("Queries application log entries for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "Logs")
        .query<dto::LogQuery>();

    reg.get<dto::LogConfiguration>(entity_path + "/logs/configuration",
                                   [this](http::TypedRequest req) -> http::Result<dto::LogConfiguration> {
                                     return log_handlers_->get_logs_configuration(req);
                                   })
        .tag("Logs")
        .summary(std::string("Get log configuration for ") + et.singular)
        .description(std::string("Returns the log filter configuration for this ") + et.singular + ".")
        .operation_id(std::string("get") + capitalize(et.singular) + "LogConfiguration");

    reg.put<dto::LogConfiguration, http::NoContent>(
           entity_path + "/logs/configuration",
           [this](http::TypedRequest req, dto::LogConfiguration body) -> http::Result<http::NoContent> {
             return log_handlers_->put_logs_configuration(req, std::move(body));
           })
        .tag("Logs")
        .summary(std::string("Update log configuration for ") + et.singular)
        .description(std::string("Updates the log severity filter and max entries for this ") + et.singular + ".")
        .operation_id(std::string("set") + capitalize(et.singular) + "LogConfiguration");

    // --- Bulk Data ---
    //
    // PR-403 commit 25: 11 bulk-data routes (5 per-entity + 3 nested subarea +
    // 3 nested subcomponent) migrated to the typed RouteRegistry API. The
    // download route uses the `reg.binary_download` escape hatch so the
    // chunked content provider, Content-Disposition filename, range support,
    // and content-type-by-format mapping all flow through the framework
    // instead of touching httplib::Response. The upload route uses
    // `reg.multipart_upload<BulkDataDescriptor>` which parses the multipart
    // body, validates the inferred response schema, and emits 201 + Location
    // via the typed attachments variant. Wire format unchanged byte-for-byte.
    reg.get<dto::BulkDataCategoryList>(entity_path + "/bulk-data",
                                       [this](http::TypedRequest req) -> http::Result<dto::BulkDataCategoryList> {
                                         return bulkdata_handlers_->list_categories(req);
                                       })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data categories for ") + et.singular)
        .description(std::string("Lists bulk-data categories (e.g., rosbag snapshots) for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "BulkDataCategories");

    reg.get<dto::Collection<dto::BulkDataDescriptor>>(
           entity_path + "/bulk-data/{category_id}",
           [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::BulkDataDescriptor>> {
             return bulkdata_handlers_->list_descriptors(req);
           })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data descriptors for ") + et.singular)
        .description(std::string("Lists downloadable files in a bulk-data category for this ") + et.singular + ".")
        .operation_id(std::string("list") + capitalize(et.singular) + "BulkDataDescriptors");

    reg.binary_download(entity_path + "/bulk-data/{category_id}/{file_id}",
                        [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
                          return bulkdata_handlers_->download(req);
                        })
        .tag("Bulk Data")
        .summary(std::string("Download bulk-data file for ") + et.singular)
        .description("Downloads a bulk-data file (binary content).")
        .operation_id(std::string("download") + capitalize(et.singular) + "BulkData");

    // Upload: only for apps and components (405 for areas and functions)
    std::string et_type_str = et.type;
    if (et_type_str == "apps" || et_type_str == "components") {
      reg.multipart_upload<dto::BulkDataDescriptor>(
             entity_path + "/bulk-data/{category_id}",
             [this](http::TypedRequest req, const http::MultipartBody & body)
                 -> http::Result<std::pair<dto::BulkDataDescriptor, http::ResponseAttachments>> {
               return bulkdata_handlers_->upload(req, body);
             })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular)
          .description(std::string("Uploads a file to a bulk-data category for this ") + et.singular + ".")
          .response(201, "File uploaded", SB::ref("BulkDataDescriptor"))
          .operation_id(std::string("upload") + capitalize(et.singular) + "BulkData");

      reg.del<http::NoContent>(entity_path + "/bulk-data/{category_id}/{file_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return bulkdata_handlers_->remove(req);
                               })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular)
          .description(std::string("Deletes a bulk-data file for this ") + et.singular + ".")
          .operation_id(std::string("delete") + capitalize(et.singular) + "BulkData");
    } else {
      // 405 stub routes for entity types that cannot host uploaded bulk-data
      // (areas, functions). Emit the legacy ERR_INVALID_REQUEST body via a
      // typed handler that returns an ErrorInfo with http_status=405; the
      // framework's error writer honours the status. Hidden from OpenAPI so
      // generated clients do not expose the no-op endpoints.
      reg.post<http::NoContent>(entity_path + "/bulk-data/{category_id}",
                                [](http::TypedRequest /*req*/) -> http::Result<http::NoContent> {
                                  ErrorInfo err;
                                  err.code = ERR_INVALID_REQUEST;
                                  err.message = "Bulk data upload is only supported for components and apps";
                                  err.http_status = 405;
                                  return tl::unexpected(std::move(err));
                                })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular + " (not supported)")
          .description("Bulk data upload is not supported for this entity type.")
          .response(405, "Method not allowed")
          .hidden();  // Always returns 405 - exclude from OpenAPI spec and generated clients

      reg.del<http::NoContent>(entity_path + "/bulk-data/{category_id}/{file_id}",
                               [](http::TypedRequest /*req*/) -> http::Result<http::NoContent> {
                                 ErrorInfo err;
                                 err.code = ERR_INVALID_REQUEST;
                                 err.message = "Bulk data deletion is only supported for components and apps";
                                 err.http_status = 405;
                                 return tl::unexpected(std::move(err));
                               })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular + " (not supported)")
          .description("Bulk data deletion is not supported for this entity type.")
          .response(405, "Method not allowed")
          .hidden();  // Always returns 405 - exclude from OpenAPI spec and generated clients
    }

    // --- Triggers (ALL entity types - x-medkit extension beyond SOVD) ---
    //
    // PR-403 commit 19: 6 trigger routes migrated to typed RouteRegistry API
    // (5 CRUD + 1 SSE). The framework auto-fills request_body<TBody> and
    // response<TResponse> from the template parameters, so the per-route
    // .request_body() / .response() builder calls drop here. POST uses the
    // attachments variant to emit 201 without re-introducing httplib::Response.
    // The SSE event-stream uses the `reg.sse<>` escape hatch.
    //
    // Triggers can be optional: if the manager is absent, the typed handler
    // wrappers below return a 501 ErrorInfo so the wire shape matches the
    // legacy "Triggers not available" SOVD GenericError exactly.
    {
      auto make_not_available_error = []() {
        ErrorInfo err;
        err.code = ERR_NOT_IMPLEMENTED;
        err.message = "Triggers not available";
        err.http_status = 501;
        return err;
      };

      // SSE events stream - registered before CRUD routes so the more specific
      // path takes precedence in cpp-httplib's first-match routing.
      reg.sse(entity_path + "/triggers/{trigger_id}/events",
              [this, make_not_available_error](http::TypedRequest req) -> http::Result<http::SseStream> {
                if (!trigger_handlers_) {
                  return tl::unexpected(make_not_available_error());
                }
                return trigger_handlers_->sse_trigger_events(req);
              })
          .tag("Triggers")
          .summary(std::string("SSE events stream for trigger on ") + et.singular)
          .description(std::string("Server-Sent Events stream for trigger notifications on this ") + et.singular + ".")
          .operation_id(std::string("stream") + capitalize(et.singular) + "TriggerEvents");

      reg.post<dto::TriggerCreateRequest, dto::Trigger>(
             entity_path + "/triggers",
             [this, make_not_available_error](http::TypedRequest req, dto::TriggerCreateRequest body)
                 -> http::Result<std::pair<dto::Trigger, http::ResponseAttachments>> {
               if (!trigger_handlers_) {
                 return tl::unexpected(make_not_available_error());
               }
               return trigger_handlers_->post_trigger(req, std::move(body));
             })
          .tag("Triggers")
          .summary(std::string("Create trigger for ") + et.singular)
          .description(std::string("Creates a new event trigger for this ") + et.singular + ".")
          .response(201, "Trigger created", SB::ref("Trigger"))
          .operation_id(std::string("create") + capitalize(et.singular) + "Trigger");

      reg.get<dto::Collection<dto::Trigger>>(
             entity_path + "/triggers",
             [this, make_not_available_error](http::TypedRequest req) -> http::Result<dto::Collection<dto::Trigger>> {
               if (!trigger_handlers_) {
                 return tl::unexpected(make_not_available_error());
               }
               return trigger_handlers_->get_triggers(req);
             })
          .tag("Triggers")
          .summary(std::string("List triggers for ") + et.singular)
          .description(std::string("Lists all triggers configured for this ") + et.singular + ".")
          .operation_id(std::string("list") + capitalize(et.singular) + "Triggers");

      reg.get<dto::Trigger>(entity_path + "/triggers/{trigger_id}",
                            [this, make_not_available_error](http::TypedRequest req) -> http::Result<dto::Trigger> {
                              if (!trigger_handlers_) {
                                return tl::unexpected(make_not_available_error());
                              }
                              return trigger_handlers_->get_trigger(req);
                            })
          .tag("Triggers")
          .summary(std::string("Get trigger for ") + et.singular)
          .description(std::string("Returns details of a specific trigger on this ") + et.singular + ".")
          .operation_id(std::string("get") + capitalize(et.singular) + "Trigger");

      reg.put<dto::TriggerUpdateRequest, dto::Trigger>(
             entity_path + "/triggers/{trigger_id}",
             [this, make_not_available_error](http::TypedRequest req,
                                              dto::TriggerUpdateRequest body) -> http::Result<dto::Trigger> {
               if (!trigger_handlers_) {
                 return tl::unexpected(make_not_available_error());
               }
               return trigger_handlers_->put_trigger(req, body);
             })
          .tag("Triggers")
          .summary(std::string("Update trigger for ") + et.singular)
          .description(std::string("Updates a trigger configuration on this ") + et.singular + ".")
          .operation_id(std::string("update") + capitalize(et.singular) + "Trigger");

      reg.del<http::NoContent>(
             entity_path + "/triggers/{trigger_id}",
             [this, make_not_available_error](http::TypedRequest req) -> http::Result<http::NoContent> {
               if (!trigger_handlers_) {
                 return tl::unexpected(make_not_available_error());
               }
               return trigger_handlers_->del_trigger(req);
             })
          .tag("Triggers")
          .summary(std::string("Delete trigger for ") + et.singular)
          .description(std::string("Deletes a trigger from this ") + et.singular + ".")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Trigger");
    }

    // --- Cyclic Subscriptions (apps, components, and functions) ---
    //
    // PR-403 commit 20: 6 cyclic-subscription routes migrated to typed
    // RouteRegistry API (5 CRUD + 1 SSE). The framework auto-fills
    // request_body<TBody> and response<TResponse> from the template parameters,
    // so the per-route .request_body() / .response() builder calls drop here.
    // POST uses the attachments variant to emit 201 without re-introducing
    // httplib::Response. The SSE event-stream uses the `reg.sse<>` escape
    // hatch and delegates the per-tick loop to the transport via
    // SubscriptionTransportProvider::make_sse_stream.
    if (et_type_str == "apps" || et_type_str == "components" || et_type_str == "functions") {
      // SSE events stream - registered before CRUD routes so the more specific
      // path takes precedence in cpp-httplib's first-match routing.
      reg.sse(entity_path + "/cyclic-subscriptions/{subscription_id}/events",
              [this](http::TypedRequest req) -> http::Result<http::SseStream> {
                return cyclic_sub_handlers_->sse_subscription_events(req);
              })
          .tag("Subscriptions")
          .summary(std::string("SSE events stream for cyclic subscription on ") + et.singular)
          .description(std::string("Server-Sent Events stream for subscription data on this ") + et.singular + ".")
          .operation_id(std::string("stream") + capitalize(et.singular) + "SubscriptionEvents");

      reg.post<dto::CyclicSubscriptionCreateRequest, dto::CyclicSubscription>(
             entity_path + "/cyclic-subscriptions",
             [this](http::TypedRequest req, dto::CyclicSubscriptionCreateRequest body)
                 -> http::Result<std::pair<dto::CyclicSubscription, http::ResponseAttachments>> {
               return cyclic_sub_handlers_->post_subscription(req, std::move(body));
             })
          .tag("Subscriptions")
          .summary(std::string("Create cyclic subscription for ") + et.singular)
          .description(std::string("Creates a new cyclic data subscription for this ") + et.singular + ".")
          .response(201, "Subscription created", SB::ref("CyclicSubscription"))
          .operation_id(std::string("create") + capitalize(et.singular) + "Subscription");

      reg.get<dto::Collection<dto::CyclicSubscription>>(
             entity_path + "/cyclic-subscriptions",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::CyclicSubscription>> {
               return cyclic_sub_handlers_->get_subscriptions(req);
             })
          .tag("Subscriptions")
          .summary(std::string("List cyclic subscriptions for ") + et.singular)
          .description(std::string("Lists all cyclic subscriptions for this ") + et.singular + ".")
          .operation_id(std::string("list") + capitalize(et.singular) + "Subscriptions");

      reg.get<dto::CyclicSubscription>(entity_path + "/cyclic-subscriptions/{subscription_id}",
                                       [this](http::TypedRequest req) -> http::Result<dto::CyclicSubscription> {
                                         return cyclic_sub_handlers_->get_subscription(req);
                                       })
          .tag("Subscriptions")
          .summary(std::string("Get cyclic subscription for ") + et.singular)
          .description(std::string("Returns details of a specific subscription on this ") + et.singular + ".")
          .operation_id(std::string("get") + capitalize(et.singular) + "Subscription");

      reg.put<dto::CyclicSubscriptionUpdateRequest, dto::CyclicSubscription>(
             entity_path + "/cyclic-subscriptions/{subscription_id}",
             [this](http::TypedRequest req,
                    dto::CyclicSubscriptionUpdateRequest body) -> http::Result<dto::CyclicSubscription> {
               return cyclic_sub_handlers_->put_subscription(req, std::move(body));
             })
          .tag("Subscriptions")
          .summary(std::string("Update cyclic subscription for ") + et.singular)
          .description(std::string("Updates a subscription configuration on this ") + et.singular + ".")
          .operation_id(std::string("update") + capitalize(et.singular) + "Subscription");

      reg.del<http::NoContent>(entity_path + "/cyclic-subscriptions/{subscription_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return cyclic_sub_handlers_->del_subscription(req);
                               })
          .tag("Subscriptions")
          .summary(std::string("Delete cyclic subscription for ") + et.singular)
          .description(std::string("Deletes a cyclic subscription from this ") + et.singular + ".")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Subscription");
    }

    // --- Locking (components and apps only, per SOVD spec) ---
    if (et_type_str == "components" || et_type_str == "apps") {
      // PR-403 commit 18: 5 lock routes migrated to typed RouteRegistry API.
      // The framework auto-fills request_body<TBody> and response<TResponse>
      // (200 / 201 / 204 per the handler return shape) from the template
      // parameters, so the per-route .request_body() / .response() builder
      // calls drop here. POST acquire-lock uses the attachments variant to
      // emit 201 + Location without re-introducing httplib::Response.
      static const nlohmann::json client_id_schema = {{"type", "string"}, {"minLength", 1}, {"maxLength", 256}};

      reg.post<dto::AcquireLockRequest, dto::Lock>(
             entity_path + "/locks",
             [this](http::TypedRequest req,
                    dto::AcquireLockRequest body) -> http::Result<std::pair<dto::Lock, http::ResponseAttachments>> {
               return lock_handlers_->post_lock(req, std::move(body));
             })
          .tag("Locking")
          .summary(std::string("Acquire lock on ") + et.singular)
          .description(std::string("Acquires an exclusive lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership", true, client_id_schema)
          .response(201, "Lock acquired", SB::ref("Lock"))
          .operation_id(std::string("acquire") + capitalize(et.singular) + "Lock");

      reg.get<dto::Collection<dto::Lock>>(entity_path + "/locks",
                                          [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::Lock>> {
                                            return lock_handlers_->get_locks(req);
                                          })
          .tag("Locking")
          .summary(std::string("List locks on ") + et.singular)
          .description(std::string("Lists all active locks on this ") + et.singular + ".")
          .header_param("X-Client-Id", "When provided, the 'owned' field indicates whether this client owns the lock",
                        false, client_id_schema)
          .operation_id(std::string("list") + capitalize(et.singular) + "Locks");

      reg.get<dto::Lock>(entity_path + "/locks/{lock_id}",
                         [this](http::TypedRequest req) -> http::Result<dto::Lock> {
                           return lock_handlers_->get_lock(req);
                         })
          .tag("Locking")
          .summary(std::string("Get lock details for ") + et.singular)
          .description(std::string("Returns details of a specific lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "When provided, the 'owned' field indicates whether this client owns the lock",
                        false, client_id_schema)
          .operation_id(std::string("get") + capitalize(et.singular) + "Lock");

      reg.put<dto::ExtendLockRequest, http::NoContent>(
             entity_path + "/locks/{lock_id}",
             [this](http::TypedRequest req, dto::ExtendLockRequest body) -> http::Result<http::NoContent> {
               return lock_handlers_->put_lock(req, body);
             })
          .tag("Locking")
          .summary(std::string("Extend lock on ") + et.singular)
          .description(std::string("Extends the expiration of a lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership", true, client_id_schema)
          .operation_id(std::string("extend") + capitalize(et.singular) + "Lock");

      reg.del<http::NoContent>(entity_path + "/locks/{lock_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return lock_handlers_->del_lock(req);
                               })
          .tag("Locking")
          .summary(std::string("Release lock on ") + et.singular)
          .description(std::string("Releases a lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership", true, client_id_schema)
          .operation_id(std::string("release") + capitalize(et.singular) + "Lock");
    }

    // --- Scripts (apps and components only) ---
    //
    // PR-403 commit 24: 8 script routes migrated to typed RouteRegistry API.
    // The list endpoint emits a domain-specific `ScriptList` wrapper so the
    // `_links` envelope is a typed `HateoasLinks` sub-struct instead of raw
    // JSON. POST upload and POST start-execution use the attachments variant
    // to emit 201/202 + Location without touching httplib::Response. The
    // framework auto-fills response<TResponse> / request_body<TBody> from the
    // template parameters; per-route .request_body() / .response() builder
    // calls stay only where the schema differs (multipart upload + free-form
    // start-execution body).
    if (script_handlers_ && (et_type_str == "apps" || et_type_str == "components")) {
      reg.multipart_upload<dto::ScriptUploadResponse>(
             entity_path + "/scripts",
             [this](http::TypedRequest req, const http::MultipartBody & body)
                 -> http::Result<std::pair<dto::ScriptUploadResponse, http::ResponseAttachments>> {
               return script_handlers_->upload_script(req, body);
             })
          .tag("Scripts")
          .summary(std::string("Upload diagnostic script for ") + et.singular)
          .description(std::string("Uploads a diagnostic script for this ") + et.singular + ".")
          .response(201, "Script uploaded", SB::ref("ScriptUploadResponse"))
          .operation_id(std::string("upload") + capitalize(et.singular) + "Script");

      reg.get<dto::ScriptList>(entity_path + "/scripts",
                               [this](http::TypedRequest req) -> http::Result<dto::ScriptList> {
                                 return script_handlers_->list_scripts(req);
                               })
          .tag("Scripts")
          .summary(std::string("List scripts for ") + et.singular)
          .description(std::string("Lists all diagnostic scripts for this ") + et.singular + ".")
          .operation_id(std::string("list") + capitalize(et.singular) + "Scripts");

      reg.get<dto::ScriptMetadata>(entity_path + "/scripts/{script_id}",
                                   [this](http::TypedRequest req) -> http::Result<dto::ScriptMetadata> {
                                     return script_handlers_->get_script(req);
                                   })
          .tag("Scripts")
          .summary(std::string("Get script metadata for ") + et.singular)
          .description(std::string("Returns metadata of a specific script for this ") + et.singular + ".")
          .operation_id(std::string("get") + capitalize(et.singular) + "Script");

      reg.del<http::NoContent>(entity_path + "/scripts/{script_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return script_handlers_->delete_script(req);
                               })
          .tag("Scripts")
          .summary(std::string("Delete script for ") + et.singular)
          .description(std::string("Deletes a diagnostic script from this ") + et.singular + ".")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Script");

      reg.post<dto::ScriptExecution>(entity_path + "/scripts/{script_id}/executions",
                                     [this](http::TypedRequest req)
                                         -> http::Result<std::pair<dto::ScriptExecution, http::ResponseAttachments>> {
                                       return script_handlers_->start_execution(req);
                                     })
          .tag("Scripts")
          .summary(std::string("Start script execution for ") + et.singular)
          .description(std::string("Starts execution of a diagnostic script on this ") + et.singular + ".")
          .request_body("Execution parameters", SB::generic_object_schema())
          .response(202, "Execution started", SB::ref("ScriptExecution"))
          .operation_id(std::string("start") + capitalize(et.singular) + "ScriptExecution");

      reg.get<dto::ScriptExecution>(entity_path + "/scripts/{script_id}/executions/{execution_id}",
                                    [this](http::TypedRequest req) -> http::Result<dto::ScriptExecution> {
                                      return script_handlers_->get_execution(req);
                                    })
          .tag("Scripts")
          .summary(std::string("Get execution status for ") + et.singular)
          .description("Returns the current status of a script execution.")
          .operation_id(std::string("get") + capitalize(et.singular) + "ScriptExecution");

      reg.put<dto::ScriptControlRequest, dto::ScriptExecution>(
             entity_path + "/scripts/{script_id}/executions/{execution_id}",
             [this](http::TypedRequest req,
                    const dto::ScriptControlRequest & body) -> http::Result<dto::ScriptExecution> {
               return script_handlers_->control_execution(req, body);
             })
          .tag("Scripts")
          .summary(std::string("Terminate script execution for ") + et.singular)
          .description("Sends a control command (e.g., terminate) to a running script execution.")
          .operation_id(std::string("control") + capitalize(et.singular) + "ScriptExecution");

      reg.del<http::NoContent>(entity_path + "/scripts/{script_id}/executions/{execution_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return script_handlers_->delete_execution(req);
                               })
          .tag("Scripts")
          .summary(std::string("Remove completed execution for ") + et.singular)
          .description("Removes a completed script execution record.")
          .operation_id(std::string("remove") + capitalize(et.singular) + "ScriptExecution");
    }

    // --- Discovery relationship endpoints (entity-type-specific) ---
    if (et_type_str == "areas") {
      reg.get<dto::Collection<dto::ComponentListItem>>(
             entity_path + "/components",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
               return discovery_handlers_->get_area_components(req);
             })
          .tag("Discovery")
          .summary("List components in area")
          .description("Lists components belonging to this area.")
          .operation_id("listAreaComponents");

      reg.get<dto::Collection<dto::AreaListItem>>(
             entity_path + "/subareas",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AreaListItem>> {
               return discovery_handlers_->get_subareas(req);
             })
          .tag("Discovery")
          .summary("List subareas")
          .description("Lists subareas within this area.")
          .operation_id("listSubareas");

      reg.get<dto::Collection<dto::ComponentListItem>>(
             entity_path + "/contains",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
               return discovery_handlers_->get_area_contains(req);
             })
          .tag("Discovery")
          .summary("List entities contained in area")
          .description("Lists all entities contained in this area.")
          .operation_id("listAreaContains");
    }

    if (et_type_str == "components") {
      reg.get<dto::Collection<dto::ComponentListItem>>(
             entity_path + "/subcomponents",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
               return discovery_handlers_->get_subcomponents(req);
             })
          .tag("Discovery")
          .summary("List subcomponents")
          .description("Lists subcomponents of this component.")
          .operation_id("listSubcomponents");

      reg.get<dto::Collection<dto::AppListItem>>(
             entity_path + "/hosts",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AppListItem>> {
               return discovery_handlers_->get_component_hosts(req);
             })
          .tag("Discovery")
          .summary("List component hosts")
          .description("Lists apps hosted by this component.")
          .operation_id("listComponentHosts");

      reg.get<dto::Collection<dto::ComponentListItem>>(
             entity_path + "/depends-on",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
               return discovery_handlers_->get_component_depends_on(req);
             })
          .tag("Discovery")
          .summary("List component dependencies")
          .description("Lists components this component depends on.")
          .operation_id("listComponentDependencies");
    }

    if (et_type_str == "apps") {
      reg.get<dto::Collection<dto::ComponentListItem>>(
             entity_path + "/is-located-on",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::ComponentListItem>> {
               return discovery_handlers_->get_app_is_located_on(req);
             })
          .tag("Discovery")
          .summary("Get app host component")
          .description("Returns the component hosting this app as a single-element collection.")
          .operation_id("getAppHost");

      reg.get<dto::Collection<dto::AreaListItem>>(
             entity_path + "/belongs-to",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AreaListItem>> {
               return discovery_handlers_->get_app_belongs_to(req);
             })
          .tag("Discovery")
          .summary("Get app parent area")
          .description(
              "Returns the area this app belongs to via its parent component, as a 0-or-1 element "
              "collection.")
          .operation_id("getAppArea");

      reg.get<dto::Collection<dto::AppListItem>>(
             entity_path + "/depends-on",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AppListItem>> {
               return discovery_handlers_->get_app_depends_on(req);
             })
          .tag("Discovery")
          .summary("List app dependencies")
          .description("Lists apps this app depends on.")
          .operation_id("listAppDependencies");
    }

    if (et_type_str == "functions") {
      reg.get<dto::Collection<dto::AppListItem>>(
             entity_path + "/hosts",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::AppListItem>> {
               return discovery_handlers_->get_function_hosts(req);
             })
          .tag("Discovery")
          .summary("List function hosts")
          .description("Lists components hosting this function.")
          .operation_id("listFunctionHosts");
    }

    // Single entity detail (capabilities) - must be LAST for this entity type.
    // Detail handlers return entity-type-specific DTOs so they cannot share a
    // single `HandlerFn` slot the way the loop's collection endpoints do; each
    // typed reg.get<T> is dispatched explicitly below.
    if (et_type_str == "areas") {
      reg.get<dto::AreaDetail>(entity_path,
                               [this](http::TypedRequest req) -> http::Result<dto::AreaDetail> {
                                 return discovery_handlers_->get_area(req);
                               })
          .tag("Discovery")
          .summary(std::string("Get ") + et.singular + " details")
          .description(std::string("Returns ") + et.singular +
                       " details with capabilities and resource collection URIs.")
          .operation_id(std::string("get") + capitalize(et.singular));
    } else if (et_type_str == "components") {
      reg.get<dto::ComponentDetail>(entity_path,
                                    [this](http::TypedRequest req) -> http::Result<dto::ComponentDetail> {
                                      return discovery_handlers_->get_component(req);
                                    })
          .tag("Discovery")
          .summary(std::string("Get ") + et.singular + " details")
          .description(std::string("Returns ") + et.singular +
                       " details with capabilities and resource collection URIs.")
          .operation_id(std::string("get") + capitalize(et.singular));
    } else if (et_type_str == "apps") {
      reg.get<dto::AppDetail>(entity_path,
                              [this](http::TypedRequest req) -> http::Result<dto::AppDetail> {
                                return discovery_handlers_->get_app(req);
                              })
          .tag("Discovery")
          .summary(std::string("Get ") + et.singular + " details")
          .description(std::string("Returns ") + et.singular +
                       " details with capabilities and resource collection URIs.")
          .operation_id(std::string("get") + capitalize(et.singular));
    } else if (et_type_str == "functions") {
      reg.get<dto::FunctionDetail>(entity_path,
                                   [this](http::TypedRequest req) -> http::Result<dto::FunctionDetail> {
                                     return discovery_handlers_->get_function(req);
                                   })
          .tag("Discovery")
          .summary(std::string("Get ") + et.singular + " details")
          .description(std::string("Returns ") + et.singular +
                       " details with capabilities and resource collection URIs.")
          .operation_id(std::string("get") + capitalize(et.singular));
    }
  }

  // === Nested entities - subareas bulk-data ===
  //
  // Typed wrappers for the nested entity bulk-data routes. The handler
  // dispatches on `parse_entity_path` so the same handler implementations
  // serve both top-level and nested entity URLs - the regex capture-group
  // shift between the two route templates is handled inside
  // `parse_entity_path` rather than in the handler signature.
  reg.get<dto::BulkDataCategoryList>("/areas/{area_id}/subareas/{subarea_id}/bulk-data",
                                     [this](http::TypedRequest req) -> http::Result<dto::BulkDataCategoryList> {
                                       return bulkdata_handlers_->list_categories(req);
                                     })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subarea")
      .description("Lists bulk-data categories for a subarea.")
      .operation_id("listSubareaBulkDataCategories");

  reg.get<dto::Collection<dto::BulkDataDescriptor>>(
         "/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::BulkDataDescriptor>> {
           return bulkdata_handlers_->list_descriptors(req);
         })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subarea")
      .description("Lists bulk-data descriptors for a subarea.")
      .operation_id("listSubareaBulkDataDescriptors");

  reg.binary_download("/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}/{file_id}",
                      [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
                        return bulkdata_handlers_->download(req);
                      })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subarea")
      .description("Downloads a bulk-data file for a subarea.")
      .operation_id("downloadSubareaBulkData");

  // === Nested entities - subcomponents bulk-data ===
  reg.get<dto::BulkDataCategoryList>("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data",
                                     [this](http::TypedRequest req) -> http::Result<dto::BulkDataCategoryList> {
                                       return bulkdata_handlers_->list_categories(req);
                                     })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subcomponent")
      .description("Lists bulk-data categories for a subcomponent.")
      .operation_id("listSubcomponentBulkDataCategories");

  reg.get<dto::Collection<dto::BulkDataDescriptor>>(
         "/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}",
         [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::BulkDataDescriptor>> {
           return bulkdata_handlers_->list_descriptors(req);
         })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subcomponent")
      .description("Lists bulk-data descriptors for a subcomponent.")
      .operation_id("listSubcomponentBulkDataDescriptors");

  reg.binary_download("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}/{file_id}",
                      [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
                        return bulkdata_handlers_->download(req);
                      })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subcomponent")
      .description("Downloads a bulk-data file for a subcomponent.")
      .operation_id("downloadSubcomponentBulkData");

  // === Global faults ===
  //
  // PR-403 commit 29: 3 global fault routes migrate to the typed RouteRegistry
  // API. The SSE stream uses the `reg.sse` escape hatch and returns
  // `Result<SseStream>`; the framework drives the chunked content provider
  // and renders limit-exceeded errors as SOVD GenericError. The list route
  // emits a `FaultListResult` opaque envelope (same as the per-entity list).
  // The global DELETE uses the attachments variant
  // `Result<pair<NoContent, ResponseAttachments>>` so the
  // `X-Medkit-Local-Only: true` header rides on top of the framework-default
  // 204 No Content.
  //
  // SSE stream must be registered before /faults to avoid regex conflict.
  reg.sse("/faults/stream",
          [this](http::TypedRequest req) -> http::Result<http::SseStream> {
            return sse_fault_handler_->sse_stream(req);
          })
      .tag("Faults")
      .summary("Stream fault events (SSE)")
      .description("Server-Sent Events stream for real-time fault notifications.")
      .operation_id("streamFaults");

  reg.get<dto::FaultListResult>("/faults",
                                [this](http::TypedRequest req) -> http::Result<dto::FaultListResult> {
                                  return fault_handlers_->list_all_faults(req);
                                })
      .tag("Faults")
      .summary("List all faults globally")
      .description("Retrieve all faults across the system.")
      .operation_id("listAllFaults")
      .query<dto::FaultListQuery>();

  reg.del<http::NoContent>(
         "/faults",
         [this](http::TypedRequest req) -> http::Result<std::pair<http::NoContent, http::ResponseAttachments>> {
           return fault_handlers_->clear_all_faults_global(req);
         })
      .tag("Faults")
      .summary("Clear all faults globally")
      .description("Clears all faults across the entire system.")
      .operation_id("clearAllFaults")
      .query<dto::FaultClearQuery>();

  // === Software Updates ===
  //
  // PR-403 commit 22: 8 update routes migrated to typed RouteRegistry API.
  // The handler instance may be null when no backend plugin is loaded; each
  // typed lambda short-circuits with a 501 ErrorInfo in that case so the
  // routes remain in the OpenAPI spec.
  //
  // - GET    /updates                          -> Result<UpdateList>
  // - POST   /updates                          -> attachments (201 + Location)
  // - GET    /updates/{update_id}              -> Result<UpdateDetail>
  // - DELETE /updates/{update_id}              -> Result<NoContent> (204)
  // - PUT    /updates/{update_id}/prepare      -> attachments (202 + Location)
  // - PUT    /updates/{update_id}/execute      -> attachments (202 + Location)
  // - PUT    /updates/{update_id}/automated    -> attachments (202 + Location)
  // - GET    /updates/{update_id}/status       -> Result<UpdateStatus>
  static const ErrorInfo kUpdate501 = [] {
    ErrorInfo err;
    err.code = ERR_NOT_IMPLEMENTED;
    err.message = "Software updates not available";
    err.http_status = 501;
    return err;
  }();

  reg.get<dto::UpdateList>("/updates",
                           [this](http::TypedRequest req) -> http::Result<dto::UpdateList> {
                             if (!update_handlers_) {
                               return tl::unexpected(kUpdate501);
                             }
                             return update_handlers_->get_updates(req);
                           })
      .tag("Updates")
      .summary("List software updates")
      .description("Lists all registered software updates.")
      .operation_id("listUpdates")
      .query<dto::UpdateListQuery>();

  reg.post<dto::UpdateRegisterRequest, dto::UpdateRegisterResponse>(
         "/updates",
         [this](http::TypedRequest req, dto::UpdateRegisterRequest body)
             -> http::Result<std::pair<dto::UpdateRegisterResponse, http::ResponseAttachments>> {
           if (!update_handlers_) {
             return tl::unexpected(kUpdate501);
           }
           return update_handlers_->post_update(req, std::move(body));
         })
      .tag("Updates")
      .summary("Register a software update")
      .description("Registers a new software update descriptor.")
      .response(201, "Update registered", SB::ref("UpdateRegisterResponse"))
      .operation_id("registerUpdate");

  reg.get<dto::UpdateStatus>("/updates/{update_id}/status",
                             [this](http::TypedRequest req) -> http::Result<dto::UpdateStatus> {
                               if (!update_handlers_) {
                                 return tl::unexpected(kUpdate501);
                               }
                               return update_handlers_->get_status(req);
                             })
      .tag("Updates")
      .summary("Get update status")
      .description("Returns the current status and progress of an update.")
      .operation_id("getUpdateStatus");

  reg.put<http::NoContent>(
         "/updates/{update_id}/prepare",
         [this](http::TypedRequest req) -> http::Result<std::pair<http::NoContent, http::ResponseAttachments>> {
           if (!update_handlers_) {
             return tl::unexpected(kUpdate501);
           }
           return update_handlers_->put_prepare(req);
         })
      .tag("Updates")
      .summary("Prepare update for execution")
      .description("Prepares an update for execution (downloads, validates).")
      .response(202, "Update preparation started")
      .operation_id("prepareUpdate");

  reg.put<http::NoContent>(
         "/updates/{update_id}/execute",
         [this](http::TypedRequest req) -> http::Result<std::pair<http::NoContent, http::ResponseAttachments>> {
           if (!update_handlers_) {
             return tl::unexpected(kUpdate501);
           }
           return update_handlers_->put_execute(req);
         })
      .tag("Updates")
      .summary("Execute update")
      .description("Starts executing a prepared update.")
      .response(202, "Update execution started")
      .operation_id("executeUpdate");

  reg.put<http::NoContent>(
         "/updates/{update_id}/automated",
         [this](http::TypedRequest req) -> http::Result<std::pair<http::NoContent, http::ResponseAttachments>> {
           if (!update_handlers_) {
             return tl::unexpected(kUpdate501);
           }
           return update_handlers_->put_automated(req);
         })
      .tag("Updates")
      .summary("Run automated update")
      .description("Runs a fully automated update (prepare + execute).")
      .response(202, "Automated update started")
      .operation_id("automateUpdate");

  reg.get<dto::UpdateDetail>("/updates/{update_id}",
                             [this](http::TypedRequest req) -> http::Result<dto::UpdateDetail> {
                               if (!update_handlers_) {
                                 return tl::unexpected(kUpdate501);
                               }
                               return update_handlers_->get_update(req);
                             })
      .tag("Updates")
      .summary("Get update details")
      .description("Returns details of a specific update.")
      .operation_id("getUpdate");

  reg.del<http::NoContent>("/updates/{update_id}",
                           [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                             if (!update_handlers_) {
                               return tl::unexpected(kUpdate501);
                             }
                             return update_handlers_->del_update(req);
                           })
      .tag("Updates")
      .summary("Delete update")
      .description("Removes an update registration.")
      .operation_id("deleteUpdate");

  // === Authentication ===
  // OAuth2 endpoints render errors per RFC 6749 §5.2 instead of SOVD
  // GenericError - the framework swaps the renderer via
  // `.error_renderer(kOAuth2Error)` so any `tl::unexpected(ErrorInfo)`
  // returned by the typed handler becomes `{"error","error_description"}`.
  // The bodies are parsed manually by the handlers because the auth endpoints
  // accept both `application/json` and `application/x-www-form-urlencoded`
  // (RFC 6749 §4.1.3); the body-less typed POST overload is used here.
  reg.post<dto::AuthTokenResponse>("/auth/authorize",
                                   [this](http::TypedRequest req) -> http::Result<dto::AuthTokenResponse> {
                                     return auth_handlers_->post_authorize(req);
                                   })
      .tag("Authentication")
      .summary("Authorize client")
      .description("Authenticate and obtain authorization tokens.")
      .request_body("Client credentials", SB::ref("AuthCredentials"))
      .operation_id("authorize")
      .error_renderer(openapi::ErrorRenderer::kOAuth2Error);

  reg.post<dto::AuthTokenResponse>("/auth/token",
                                   [this](http::TypedRequest req) -> http::Result<dto::AuthTokenResponse> {
                                     return auth_handlers_->post_token(req);
                                   })
      .tag("Authentication")
      .summary("Obtain access token")
      .description("Exchange credentials or refresh token for a JWT access token.")
      .request_body("Token request credentials", SB::ref("AuthCredentials"))
      .operation_id("getToken")
      .error_renderer(openapi::ErrorRenderer::kOAuth2Error);

  reg.post<dto::AuthRevokeResponse>("/auth/revoke",
                                    [this](http::TypedRequest req) -> http::Result<dto::AuthRevokeResponse> {
                                      return auth_handlers_->post_revoke(req);
                                    })
      .tag("Authentication")
      .summary("Revoke token")
      .description("Revoke an access or refresh token.")
      .request_body("Token to revoke", SB::ref("AuthRevokeRequest"))
      .operation_id("revokeToken")
      .error_renderer(openapi::ErrorRenderer::kOAuth2Error);

  // === Lifecycle (status) - apps and components only, outside the 4-type loop ===
  // GET and PUT use different HTTP methods so neither can shadow the other.
  // Registration order within the loop is arbitrary from the router's perspective.
  for (const auto & et_lc :
       std::vector<std::pair<const char *, const char *>>{{"apps", "app"}, {"components", "component"}}) {
    const std::string base_lc = std::string("/") + et_lc.first + "/{" + et_lc.second + "_id}";
    // e.g. "App" / "Component" for operation ID construction
    const std::string entity_cap = capitalize(std::string(et_lc.second));

    for (const auto & action : {"start", "restart", "force-restart", "shutdown", "force-shutdown"}) {
      std::string action_str = action;
      // Capitalise action for operation ID: "force-restart" -> "ForceRestart"
      std::string action_cap;
      bool cap_next = true;
      for (char c : action_str) {
        if (c == '-') {
          cap_next = true;
        } else {
          action_cap += cap_next ? static_cast<char>(std::toupper(static_cast<unsigned char>(c))) : c;
          cap_next = false;
        }
      }
      reg.put<http::NoContent>(base_lc + "/status/" + action,
                               [this, action_str](http::TypedRequest req)
                                   -> http::Result<std::pair<http::NoContent, http::ResponseAttachments>> {
                                 return lifecycle_handlers_->handle_transition(req, action_str);
                               })
          .tag("Lifecycle")
          .summary(std::string("Request lifecycle transition '") + action + "'")
          .response(202, "Lifecycle transition accepted")
          .operation_id(std::string("put").append(entity_cap).append("Status").append(action_cap));
    }

    reg.get<dto::LifecycleStatusResponse>(base_lc + "/status",
                                          [this](http::TypedRequest req) -> http::Result<dto::LifecycleStatusResponse> {
                                            return lifecycle_handlers_->handle_get_status(req);
                                          })
        .tag("Lifecycle")
        .summary(std::string("Get ") + et_lc.second + " lifecycle status")
        .operation_id(std::string("get") + entity_cap + "Status");
  }

  // Register all routes with cpp-httplib
  route_registry_->register_all(*srv, API_BASE_PATH);
}

void RESTServer::start() {
  http_server_->listen(host_, port_);
}

void RESTServer::stop() {
  // Wake any blocked SSE chunked-content-provider loops first. The server
  // thread's join (in GatewayNode::stop_rest_server) waits for active request
  // lambdas to return; without this signal the SSE lambda sleeps on its
  // keepalive CV (30 s) and the join can outlast launch_testing's shutdown
  // budget, ending up as SIGKILL.
  if (sse_fault_handler_) {
    sse_fault_handler_->request_shutdown();
  }
  if (http_server_) {
    http_server_->stop();
  }
}

void RESTServer::set_cors_headers(httplib::Response & res, const std::string & origin) const {
  res.set_header("Access-Control-Allow-Origin", origin);

  // Use pre-built header strings from CorsConfig
  if (!cors_config_.methods_header.empty()) {
    res.set_header("Access-Control-Allow-Methods", cors_config_.methods_header);
  }
  if (!cors_config_.headers_header.empty()) {
    res.set_header("Access-Control-Allow-Headers", cors_config_.headers_header);
  }

  // Expose headers that JavaScript needs access to (e.g., for file downloads)
  res.set_header("Access-Control-Expose-Headers",
                 "Content-Disposition, Content-Length, X-RateLimit-Limit, X-RateLimit-Remaining, "
                 "X-RateLimit-Reset, Retry-After");

  // Set credentials header if enabled
  if (cors_config_.allow_credentials) {
    res.set_header("Access-Control-Allow-Credentials", "true");
  }
}

bool RESTServer::is_origin_allowed(const std::string & origin) const {
  // Check if origin matches any allowed origin
  // Note: Wildcard "*" is allowed here but credentials+wildcard is blocked at startup
  // (see gateway_node.cpp validation). When wildcard is used, we echo back the actual
  // origin for security, as browsers require exact origin match with credentials.
  for (const auto & allowed : cors_config_.allowed_origins) {
    if (allowed == "*" || allowed == origin) {
      return true;
    }
  }
  return false;
}

}  // namespace ros2_medkit_gateway
