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

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <exception>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <type_traits>

#include "ros2_medkit_gateway/core/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/parameter_error_classification.hpp"
#include "ros2_medkit_gateway/core/thread_pool_config.hpp"
#include "ros2_medkit_gateway/dto/fault_triggers.hpp"
#include "ros2_medkit_gateway/dto/sse_frames.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/detail/status_recorder.hpp"
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
  // Both clamps report when they fire: the pool that ends up serving requests
  // must be traceable to the config file that asked for it.
  const auto requested_pool_size = node_->get_parameter("server.http_thread_pool_size").as_int();
  const auto http_thread_pool_size = clamp_thread_count(requested_pool_size, 1, 1024);
  if (static_cast<int64_t>(http_thread_pool_size) != requested_pool_size) {
    RCLCPP_WARN(rclcpp::get_logger("rest_server"), "server.http_thread_pool_size %" PRId64 " clamped to %zu",
                requested_pool_size, http_thread_pool_size);
  }
  const auto requested_keep_alive = node_->get_parameter("server.keep_alive_timeout_sec").as_int();
  const auto keep_alive_timeout_sec = clamp_keep_alive_timeout(requested_keep_alive, 1, 3600);
  if (static_cast<int64_t>(keep_alive_timeout_sec) != requested_keep_alive) {
    RCLCPP_WARN(rclcpp::get_logger("rest_server"), "server.keep_alive_timeout_sec %" PRId64 " clamped to %" PRId64,
                requested_keep_alive, static_cast<int64_t>(keep_alive_timeout_sec));
  }
  http_server_ = std::make_unique<HttpServerManager>(tls_config_, http_thread_pool_size, keep_alive_timeout_sec);
  RCLCPP_INFO(rclcpp::get_logger("rest_server"),
              "HTTP request thread pool bounded to %zu workers (each active SSE stream holds one), "
              "keep-alive timeout %" PRId64 "s",
              http_thread_pool_size, static_cast<int64_t>(keep_alive_timeout_sec));

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
  // Read the same flag the middleware branch above reads, so the document
  // declares 429 exactly when the limiter is live.
  route_registry_->set_rate_limit_enabled(rate_limit_config.enabled);

  // Taken before the handlers that read it: /health reports how much of the
  // SSE client cap is in use, and a tracker fetched afterwards would reach
  // HealthHandlers as a null pointer and silently omit the field.
  sse_client_tracker_ = node_->get_sse_client_tracker();

  health_handlers_ =
      std::make_unique<handlers::HealthHandlers>(*handler_ctx_, route_registry_.get(), sse_client_tracker_);
  discovery_handlers_ = std::make_unique<handlers::DiscoveryHandlers>(*handler_ctx_);
  data_handlers_ = std::make_unique<handlers::DataHandlers>(*handler_ctx_);
  lifecycle_handlers_ = std::make_unique<handlers::LifecycleHandlers>(
      *handler_ctx_, node_->get_plugin_manager(), std::make_shared<Ros2LifecycleStateReader>(node_));
  operation_handlers_ = std::make_unique<handlers::OperationHandlers>(*handler_ctx_);
  config_handlers_ = std::make_unique<handlers::ConfigHandlers>(*handler_ctx_);
  fault_handlers_ = std::make_unique<handlers::FaultHandlers>(*handler_ctx_);
  log_handlers_ = std::make_unique<handlers::LogHandlers>(*handler_ctx_);
  auth_handlers_ = std::make_unique<handlers::AuthHandlers>(*handler_ctx_);
  // Documented range 1-3600 s. Clamped rather than refused, and the clamp is
  // reported, because a silently corrected value and the config file then
  // disagree with no way to see it from outside.
  constexpr int64_t kMinKeepaliveSec = 1;
  constexpr int64_t kMaxKeepaliveSec = 3600;
  const int64_t keepalive_raw = node_->get_parameter("sse.keepalive_interval_sec").as_int();
  const int64_t keepalive_used = std::clamp<int64_t>(keepalive_raw, kMinKeepaliveSec, kMaxKeepaliveSec);
  if (keepalive_used != keepalive_raw) {
    RCLCPP_WARN(node_->get_logger(), "sse.keepalive_interval_sec %" PRId64 " clamped to %" PRId64, keepalive_raw,
                keepalive_used);
  }
  sse_fault_handler_ = std::make_unique<handlers::SSEFaultHandler>(*handler_ctx_, sse_client_tracker_,
                                                                   std::chrono::seconds(keepalive_used));
  bulkdata_handlers_ = std::make_unique<handlers::BulkDataHandlers>(*handler_ctx_);
  // Validated as int64 before narrowing: narrowing first wraps a value above
  // INT_MAX into a small positive one that passes the check, so 4294967300
  // would silently cap every subscription at 4 seconds. The warning names the
  // requested value, otherwise the operator cannot tell which value was refused.
  const int64_t max_duration_raw = node_->get_parameter("sse.max_duration_sec").as_int();
  int max_duration_sec = 3600;
  if (max_duration_raw < 1 || max_duration_raw > std::numeric_limits<int>::max()) {
    RCLCPP_WARN(rclcpp::get_logger("rest_server"),
                "sse.max_duration_sec %" PRId64 " must be in [1, 2147483647]. Using default 3600.", max_duration_raw);
  } else {
    max_duration_sec = static_cast<int>(max_duration_raw);
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
  // Read the same pointer the forwarding branch reads
  // (`HandlerContext::validate_entity_for_route` forwards only when
  // `aggregation_mgr_` is non-null), so the document's 502/503 cannot drift
  // from the condition that makes them reachable. Called before the first
  // `/docs` request, and the document is generated per request, so setting it
  // after `setup_routes()` is fine.
  if (route_registry_) {
    route_registry_->set_aggregation_enabled(mgr != nullptr);
  }
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

#ifdef MEDKIT_STATUS_RECORDER
  // Test builds only (BUILD_TESTING; see CMakeLists.txt). Serves what the
  // emitted-status recorder has observed so far so an integration test can
  // assert the served document declares every status the gateway actually
  // put on the wire. Registered straight onto the server rather than through
  // the RouteRegistry: it must not appear in the document it is used to
  // check, and it must not be recorded by the recorder it reads.
  srv->Get(api_path("/x-medkit-status-coverage"), [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
    res.set_content(http::detail::emitted_status_report().dump(), "application/json");
  });
#endif

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

    // The engine only exists when the feature is on and at least one plugin is
    // loaded. That is the same "route mounted, backend absent" shape as the
    // `/updates` and `/triggers` gates, so it answers the same way they do:
    // 501 not-implemented. The 404 it used to answer said the *rule collection*
    // did not exist, which is indistinguishable from an unknown app and told a
    // client to go looking for an id instead of enabling a feature.
    auto ft_engine_absent = [ft_json_error](httplib::Response & res) {
      ft_json_error(res, 501, "fault-trigger engine is not enabled", ERR_NOT_IMPLEMENTED);
    };

    route_registry_
        ->raw("get", "/apps/{app_id}/fault-triggers",
              [this, ft_engine_absent](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_engine_absent(res);
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
        // 501 when the engine is not running (feature off, or no plugin loaded).
        .errors({501})
        // `array of object` said nothing about a rule. The schema now comes
        // from the same descriptor the engine's rule_to_json() field names are
        // documented against.
        .response<dto::FaultTriggerRuleList>(200, "Rule list");

    route_registry_
        ->raw("post", "/apps/{app_id}/fault-triggers",
              [this, ft_json_error, ft_engine_absent](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_engine_absent(res);
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
                  // The engine reports three statuses and they are three
                  // different failures. Letting them all fall through to the
                  // status-shaped default told a client that an unknown app was
                  // a missing sub-resource and that a duplicate fault_code was a
                  // malformed parameter it could fix by editing the body.
                  const int status = created.error().first;
                  const char * code = ERR_INVALID_PARAMETER;
                  if (status == 404) {
                    // The rule names an app the entity registry does not know.
                    code = ERR_ENTITY_NOT_FOUND;
                  } else if (status == 409) {
                    // fault_code is the fault store's primary key, so uniqueness
                    // is a precondition on the request, not a field format.
                    code = ERR_PRECONDITION_NOT_FULFILLED;
                  }
                  ft_json_error(res, status, created.error().second, code);
                  return;
                }
                // Raw route: the typed registry's automatic 201 `Location`
                // declaration cannot reach here, so the header is set - and
                // declared below - by hand. `req.path` already carries the API
                // prefix, matching the form every other 201 uses.
                res.set_header("Location", child_resource_path(req.path, created->id));
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
        .request_body<dto::FaultTriggerRuleCreateRequest>("Fault-trigger rule definition")
        // 501 when the engine is not running (feature off, or no plugin loaded).
        .errors({501})
        .response<dto::FaultTriggerRule>(201, "Created rule")
        .response_header(
            201, openapi::ResponseHeader{"Location",
                                         "Absolute path of the created rule, API prefix included (`/api/v1/...`).",
                                         nlohmann::json{{"type", "string"}, {"format", "uri-reference"}}})
        // 400/404 come from the registry's automatic response-level
        // GenericError $ref; only 409 needs a manual declaration.
        .response(409, "fault_code already used by another rule",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}});

    route_registry_
        ->raw("delete", "/apps/{app_id}/fault-triggers/{trigger_id}",
              [this, ft_json_error, ft_engine_absent](const httplib::Request & req, httplib::Response & res) {
                auto * engine = node_->get_fault_trigger_engine();
                if (!engine) {
                  ft_engine_absent(res);
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
        // 501 when the engine is not running (feature off, or no plugin loaded).
        .errors({501})
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
      // HealthHandlers::get_version_info -> merge_peer_items (peer vendor blocks).
      .fan_out_aware()
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
        // DataHandlers::get_data_item answers 503 when topic sampling is not
        // configured. The emitted-status recorder cannot reach it: the test
        // fixture configures sampling, so no run drives that branch.
        .errors({503})
        .operation_id(std::string("get") + capitalize(et.singular) + "DataItem");

    reg.put<dto::DataValue>(entity_path + "/data/{data_id}",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->put_data_item(req);
                            })
        .tag("Data")
        .summary(std::string("Write data item for ") + et.singular)
        .description(std::string("Publishes a value to a ROS 2 topic on this ") + et.singular + ".")
        .request_body("Data value to write", SB::ref("DataWriteRequest"))
        // DataHandlers::put_data_item -> HandlerContext::validate_lock_access("data").
        .lock_guarded()
        .operation_id(std::string("put") + capitalize(et.singular) + "DataItem");

    // Data-categories. Unconditionally 501: `only_status` drops both the
    // fabricated 200 (the DataValue return type is a placeholder the handler
    // never produces) and the blanket 400/404/500 the route cannot emit either.
    reg.get<dto::DataValue>(entity_path + "/data-categories",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->data_categories(req);
                            })
        .tag("Data")
        .summary(std::string("List data categories for ") + et.singular)
        .description(std::string("Lists available data categories for this ") + et.singular + ".")
        .only_status(501, "Data categories are not implemented for ROS 2")
        .operation_id(std::string("list") + capitalize(et.singular) + "DataCategories");

    // Data-groups. Unconditionally 501 (see data-categories above).
    reg.get<dto::DataValue>(entity_path + "/data-groups",
                            [this](http::TypedRequest req) -> http::Result<dto::DataValue> {
                              return data_handlers_->data_groups(req);
                            })
        .tag("Data")
        .summary(std::string("List data groups for ") + et.singular)
        .description(std::string("Lists available data groups for this ") + et.singular + ".")
        .only_status(501, "Data groups are not implemented for ROS 2")
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
        // DataHandlers::list_data -> fan_out_collection<DataItem>.
        .fan_out_aware()
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
        // OperationHandlers::list_operations -> fan_out_collection<OperationItem>.
        .fan_out_aware()
        .operation_id(std::string("list") + capitalize(et.singular) + "Operations");

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
        // 400/404/500 come from the registry's automatic response-level
        // GenericError $ref; these two are new answers this route can give and
        // need their own declarations, or a generated SDK has no branch for a
        // timeout and treats it as an unknown failure.
        .response(503,
                  "The backing ROS 2 service or action is not on the graph "
                  "(x-medkit-ros2-service-unavailable / x-medkit-ros2-action-unavailable)",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        .response(504,
                  "The backing ROS 2 endpoint did not answer inside service_call_timeout_sec "
                  "(not-responding). The work may still be running.",
                  nlohmann::json{{"$ref", "#/components/schemas/GenericError"}})
        // OperationHandlers::create_execution -> validate_lock_access("operations").
        .lock_guarded()
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

    reg.put<dto::ExecutionUpdateRequest, http::Accepted<dto::OperationExecution>>(
           entity_path + "/operations/{operation_id}/executions/{execution_id}",
           std::function<http::Result<std::pair<http::Accepted<dto::OperationExecution>, http::ResponseAttachments>>(
               http::TypedRequest, dto::ExecutionUpdateRequest)>{
               [this](http::TypedRequest req, const dto::ExecutionUpdateRequest & body)
                   -> http::Result<std::pair<http::Accepted<dto::OperationExecution>, http::ResponseAttachments>> {
                 return operation_handlers_->update_execution(req, body);
               }})
        .tag("Operations")
        .summary(std::string("Update execution for ") + et.singular)
        .description("Sends a control command to a running execution.")
        .success_description("Accepted (asynchronous control)")
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
        // OperationHandlers::update_execution -> validate_lock_access("operations").
        .lock_guarded()
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
        // OperationHandlers::cancel_execution -> validate_lock_access("operations").
        .lock_guarded()
        .operation_id(std::string("cancel") + capitalize(et.singular) + "Execution");

    // Operation item, registered after its own sub-resources. {operation_id}
    // spans segments, so this pattern also matches an executions URI; the
    // router takes the first route that matches, and the specific ones have to
    // be reachable.
    reg.get<dto::OperationDetail>(entity_path + "/operations/{operation_id}",
                                  [this](http::TypedRequest req) -> http::Result<dto::OperationDetail> {
                                    return operation_handlers_->get_operation(req);
                                  })
        .tag("Operations")
        .summary(std::string("Get operation details for ") + et.singular)
        .description(std::string("Returns operation details including request/response schema for this ") +
                     et.singular + ".")
        .operation_id(std::string("get") + capitalize(et.singular) + "Operation");

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
        // ConfigHandlers::list_configurations -> fan_out_collection<ConfigurationMetaData>.
        .fan_out_aware()
        // Parameter failures reach the wire through `classify_parameter_error`,
        // whose whole range is declared here rather than copied: 403 (the
        // parameter is read-only) and 503 (the backing node is unreachable or
        // timed out) are first-party statuses `Ros2ParameterTransport` produces
        // and no blanket rule adds. Out of the emitted-status recorder's reach -
        // the fixture's nodes answer and none of their parameters is read-only.
        .errors(handlers::parameter_error_statuses())
        .operation_id(std::string("list") + capitalize(et.singular) + "Configurations");

    reg.get<dto::ConfigurationReadValue>(entity_path + "/configurations/{config_id}",
                                         [this](http::TypedRequest req) -> http::Result<dto::ConfigurationReadValue> {
                                           return config_handlers_->get_configuration(req);
                                         })
        .tag("Configuration")
        .summary(std::string("Get specific configuration for ") + et.singular)
        .description(std::string("Returns a specific ROS 2 node parameter for this ") + et.singular + ".")
        // Parameter failures reach the wire through `classify_parameter_error`,
        // whose whole range is declared here rather than copied: 403 (the
        // parameter is read-only) and 503 (the backing node is unreachable or
        // timed out) are first-party statuses `Ros2ParameterTransport` produces
        // and no blanket rule adds. Out of the emitted-status recorder's reach -
        // the fixture's nodes answer and none of their parameters is read-only.
        .errors(handlers::parameter_error_statuses())
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
        // ConfigHandlers::set_configuration -> validate_lock_access("configurations").
        .lock_guarded()
        // Parameter failures reach the wire through `classify_parameter_error`,
        // whose whole range is declared here rather than copied: 403 (the
        // parameter is read-only) and 503 (the backing node is unreachable or
        // timed out) are first-party statuses `Ros2ParameterTransport` produces
        // and no blanket rule adds. Out of the emitted-status recorder's reach -
        // the fixture's nodes answer and none of their parameters is read-only.
        .errors(handlers::parameter_error_statuses())
        .operation_id(std::string("set") + capitalize(et.singular) + "Configuration");

    reg.del<http::NoContent>(entity_path + "/configurations/{config_id}",
                             [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                               return config_handlers_->delete_configuration(req);
                             })
        .tag("Configuration")
        .summary(std::string("Delete configuration for ") + et.singular)
        .description(std::string("Resets a configuration parameter to its default for this ") + et.singular + ".")
        // ConfigHandlers::delete_configuration -> validate_lock_access("configurations").
        .lock_guarded()
        // Parameter failures reach the wire through `classify_parameter_error`,
        // whose whole range is declared here rather than copied: 403 (the
        // parameter is read-only) and 503 (the backing node is unreachable or
        // timed out) are first-party statuses `Ros2ParameterTransport` produces
        // and no blanket rule adds. Out of the emitted-status recorder's reach -
        // the fixture's nodes answer and none of their parameters is read-only.
        .errors(handlers::parameter_error_statuses())
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
        // ConfigHandlers::delete_all_configurations -> validate_lock_access("configurations").
        .lock_guarded()
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
        // FaultHandlers::list_faults -> merge_peer_items.
        .fan_out_aware()
        // FaultHandlers::list_faults answers 503 when the fault store cannot
        // be read. The recorder cannot reach it: the fixture's store is
        // healthy, so no run drives that branch.
        .errors({503})
        .operation_id(std::string("list") + capitalize(et.singular) + "Faults")
        .query<dto::FaultEntityListQuery>();

    reg.get<dto::FaultDetailResult>(entity_path + "/faults/{fault_code}",
                                    [this](http::TypedRequest req) -> http::Result<dto::FaultDetailResult> {
                                      return fault_handlers_->get_fault(req);
                                    })
        .tag("Faults")
        .summary(std::string("Get specific fault for ") + et.singular)
        .description("Returns fault details including SOVD status, environment data, and rosbag snapshots.")
        // 503 when the fault store cannot be read - same branch as the list
        // routes, and equally out of the recorder's reach.
        .errors({503})
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
        // FaultHandlers::clear_fault -> validate_lock_access("faults").
        .lock_guarded()
        // 503 when the fault store cannot be read - clear_fault reads the fault
        // before clearing it, so it answers the same status the read does.
        .errors({503})
        .operation_id(std::string("clear") + capitalize(et.singular) + "Fault");

    reg.del<http::NoContent>(entity_path + "/faults",
                             [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                               return fault_handlers_->clear_all_faults(req);
                             })
        .tag("Faults")
        .summary(std::string("Clear all faults for ") + et.singular)
        .description(std::string("Clears all faults for this ") + et.singular + ".")
        // FaultHandlers::clear_all_faults -> validate_lock_access("faults").
        .lock_guarded()
        // 503 when the fault store cannot be read - same unreachable-in-test
        // branch as the list route above.
        .errors({503})
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
        // LogHandlers::get_logs -> fan_out_collection<LogEntry>.
        .fan_out_aware()
        // All three log routes answer 503 when no LogManager is attached, or
        // when it refuses the read. The recorder cannot reach it: the fixture
        // always has one, so no run drives that branch.
        .errors({503})
        .operation_id(std::string("list") + capitalize(et.singular) + "Logs")
        .query<dto::LogQuery>();

    reg.get<dto::LogConfiguration>(entity_path + "/logs/configuration",
                                   [this](http::TypedRequest req) -> http::Result<dto::LogConfiguration> {
                                     return log_handlers_->get_logs_configuration(req);
                                   })
        .tag("Logs")
        .summary(std::string("Get log configuration for ") + et.singular)
        .description(std::string("Returns the log filter configuration for this ") + et.singular + ".")
        .errors({503})  // No LogManager attached - see the list route above.
        .operation_id(std::string("get") + capitalize(et.singular) + "LogConfiguration");

    reg.put<dto::LogConfiguration, http::NoContent>(
           entity_path + "/logs/configuration",
           [this](http::TypedRequest req, dto::LogConfiguration body) -> http::Result<http::NoContent> {
             return log_handlers_->put_logs_configuration(req, std::move(body));
           })
        .tag("Logs")
        .summary(std::string("Update log configuration for ") + et.singular)
        .description(std::string("Updates the log severity filter and max entries for this ") + et.singular + ".")
        // LogHandlers::put_logs_configuration -> validate_lock_access("logs").
        .lock_guarded()
        .errors({503})  // No LogManager attached - see the list route above.
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

    reg.binary_download(
           entity_path + "/bulk-data/{category_id}/{file_id}",
           [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
             return bulkdata_handlers_->download(req);
           },
           handlers::BulkDataHandlers::download_media_types())
        .tag("Bulk Data")
        .summary(std::string("Download bulk-data file for ") + et.singular)
        .description("Downloads a bulk-data file (binary content).")
        .operation_id(std::string("download") + capitalize(et.singular) + "BulkData");

    // Upload: only for apps and components (405 for areas and functions)
    std::string et_type_str = et.type;
    if (et_type_str == "apps" || et_type_str == "components") {
      reg.multipart_upload<http::Created<dto::BulkDataDescriptor>>(
             entity_path + "/bulk-data/{category_id}",
             [this](http::TypedRequest req, const http::MultipartBody & body)
                 -> http::Result<std::pair<http::Created<dto::BulkDataDescriptor>, http::ResponseAttachments>> {
               return bulkdata_handlers_->upload(req, body);
             })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular)
          .description(std::string("Uploads a file to a bulk-data category for this ") + et.singular + ".")
          // Part names read off BulkDataHandlers::upload. `file` is the only
          // one whose absence is a 400; `description` is stored as-is and
          // `metadata` must parse as a JSON object or the request is rejected.
          .multipart_body("File to store in the category, with optional description and metadata",
                          {openapi::MultipartPart{"file", "File content. The part's filename becomes the stored name.",
                                                  nlohmann::json{}, true, "application/octet-stream"},
                           openapi::MultipartPart{"description", "Free-text description stored alongside the file.",
                                                  nlohmann::json{{"type", "string"}}, false, ""},
                           openapi::MultipartPart{"metadata",
                                                  "JSON object stored alongside the file. Must be an object; anything "
                                                  "else is rejected with 400.",
                                                  nlohmann::json{{"type", "object"}, {"additionalProperties", true}},
                                                  false, "application/json"}})
          .success_description("File uploaded")
          // BulkDataHandlers::upload -> validate_lock_access("bulk-data").
          .lock_guarded()
          // BulkDataHandlers::upload answers 413 when the part exceeds
          // `bulk_data.max_upload_bytes`. The recorder cannot reach it: the
          // fixture leaves the limit unbounded, so no run drives it.
          .errors({413})
          .operation_id(std::string("upload") + capitalize(et.singular) + "BulkData");

      reg.del<http::NoContent>(entity_path + "/bulk-data/{category_id}/{file_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return bulkdata_handlers_->remove(req);
                               })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular)
          .description(std::string("Deletes a bulk-data file for this ") + et.singular + ".")
          // BulkDataHandlers::remove -> validate_lock_access("bulk-data").
          .lock_guarded()
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
    // Triggers can be optional: if the manager is absent, `.gated_on(...)`
    // short-circuits the route with the "Triggers not available" SOVD
    // GenericError. Expressing the gate on the registration instead of inside
    // each lambda is what puts the 501 into the generated document.
    {
      auto triggers_available = [this] {
        return trigger_handlers_ != nullptr;
      };
      ErrorInfo triggers_unavailable;
      triggers_unavailable.code = ERR_NOT_IMPLEMENTED;
      triggers_unavailable.message = "Triggers not available";
      triggers_unavailable.http_status = 501;

      // SSE events stream - registered before CRUD routes so the more specific
      // path takes precedence in cpp-httplib's first-match routing.
      reg.sse(entity_path + "/triggers/{trigger_id}/events",
              [this](http::TypedRequest req) -> http::Result<http::SseStream> {
                return trigger_handlers_->sse_trigger_events(req);
              })
          .tag("Triggers")
          .summary(std::string("SSE events stream for trigger on ") + et.singular)
          .description(std::string("Server-Sent Events stream for trigger notifications on this ") + et.singular +
                       ". Each frame's `data:` field is a TriggerEventFrame. An idle stream sends "
                       "`:keepalive` comment lines every 15s, which carry no JSON.")
          // The frame TriggerManager builds - {timestamp, payload}, no error
          // branch, unlike the subscription stream beside it.
          .success_schema<dto::TriggerEventFrame>()
          .gated_on(triggers_available, triggers_unavailable)
          // TriggerHandlers::sse_trigger_events answers 503 once the SSE
          // client limit is reached. The recorder cannot reach it: the fixture
          // never opens enough concurrent streams.
          .errors({503})
          .operation_id(std::string("stream") + capitalize(et.singular) + "TriggerEvents");

      reg.post<dto::TriggerCreateRequest, http::Created<dto::Trigger>>(
             entity_path + "/triggers",
             [this](http::TypedRequest req, dto::TriggerCreateRequest body)
                 -> http::Result<std::pair<http::Created<dto::Trigger>, http::ResponseAttachments>> {
               return trigger_handlers_->post_trigger(req, std::move(body));
             })
          .tag("Triggers")
          .summary(std::string("Create trigger for ") + et.singular)
          .description(std::string("Creates a new event trigger for this ") + et.singular + ".")
          .success_description("Trigger created")
          .gated_on(triggers_available, triggers_unavailable)
          // TriggerHandlers::post_trigger answers 503 when the trigger engine
          // refuses the rule or the resource subscription fails. The recorder
          // cannot reach it: the fixture's engine accepts every rule.
          .errors({503})
          .operation_id(std::string("create") + capitalize(et.singular) + "Trigger");

      reg.get<dto::Collection<dto::Trigger>>(
             entity_path + "/triggers",
             [this](http::TypedRequest req) -> http::Result<dto::Collection<dto::Trigger>> {
               return trigger_handlers_->get_triggers(req);
             })
          .tag("Triggers")
          .summary(std::string("List triggers for ") + et.singular)
          .description(std::string("Lists all triggers configured for this ") + et.singular + ".")
          .gated_on(triggers_available, triggers_unavailable)
          .operation_id(std::string("list") + capitalize(et.singular) + "Triggers");

      reg.get<dto::Trigger>(entity_path + "/triggers/{trigger_id}",
                            [this](http::TypedRequest req) -> http::Result<dto::Trigger> {
                              return trigger_handlers_->get_trigger(req);
                            })
          .tag("Triggers")
          .summary(std::string("Get trigger for ") + et.singular)
          .description(std::string("Returns details of a specific trigger on this ") + et.singular + ".")
          .gated_on(triggers_available, triggers_unavailable)
          .operation_id(std::string("get") + capitalize(et.singular) + "Trigger");

      reg.put<dto::TriggerUpdateRequest, dto::Trigger>(
             entity_path + "/triggers/{trigger_id}",
             [this](http::TypedRequest req, dto::TriggerUpdateRequest body) -> http::Result<dto::Trigger> {
               return trigger_handlers_->put_trigger(req, body);
             })
          .tag("Triggers")
          .summary(std::string("Update trigger for ") + et.singular)
          .description(std::string("Updates a trigger configuration on this ") + et.singular + ".")
          .gated_on(triggers_available, triggers_unavailable)
          .operation_id(std::string("update") + capitalize(et.singular) + "Trigger");

      reg.del<http::NoContent>(entity_path + "/triggers/{trigger_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return trigger_handlers_->del_trigger(req);
                               })
          .tag("Triggers")
          .summary(std::string("Delete trigger for ") + et.singular)
          .description(std::string("Deletes a trigger from this ") + et.singular + ".")
          .gated_on(triggers_available, triggers_unavailable)
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
          .description(std::string("Server-Sent Events stream for subscription data on this ") + et.singular +
                       ". Each frame's `data:` field is a SubscriptionEventFrame carrying either the sample "
                       "or the reason there was none; a failed sample does not close the stream.")
          // SubscriptionTransportProvider::make_sse_stream's envelope -
          // {timestamp, payload | error}.
          .success_schema<dto::SubscriptionEventFrame>()
          // Non-HTTP transports (MQTT, WebSocket, Zenoh) cannot produce an HTTP
          // stream: SubscriptionTransportProvider::make_sse_stream answers 501.
          .errors({501})
          .operation_id(std::string("stream") + capitalize(et.singular) + "SubscriptionEvents");

      reg.post<dto::CyclicSubscriptionCreateRequest, http::Created<dto::CyclicSubscription>>(
             entity_path + "/cyclic-subscriptions",
             [this](http::TypedRequest req, dto::CyclicSubscriptionCreateRequest body)
                 -> http::Result<std::pair<http::Created<dto::CyclicSubscription>, http::ResponseAttachments>> {
               return cyclic_sub_handlers_->post_subscription(req, std::move(body));
             })
          .tag("Subscriptions")
          .summary(std::string("Create cyclic subscription for ") + et.singular)
          .description(std::string("Creates a new cyclic data subscription for this ") + et.singular + ".")
          .success_description("Subscription created")
          // CyclicSubscriptionHandlers::post_subscription answers 503 when the
          // subscription manager or the event source refuses. The recorder
          // cannot reach it: both accept in the fixture.
          .errors({503})
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

      reg.post<dto::AcquireLockRequest, http::Created<dto::Lock>>(
             entity_path + "/locks",
             [this](http::TypedRequest req, dto::AcquireLockRequest body)
                 -> http::Result<std::pair<http::Created<dto::Lock>, http::ResponseAttachments>> {
               return lock_handlers_->post_lock(req, std::move(body));
             })
          .tag("Locking")
          .summary(std::string("Acquire lock on ") + et.singular)
          .description(std::string("Acquires an exclusive lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership", true, client_id_schema)
          .success_description("Lock acquired")
          // 409 from LockManager::acquire, passed through verbatim by
          // post_lock: `lock-conflict` when the entity is already locked and
          // `break_lock` was not requested, `lock-not-breakable` when it was
          // but the existing lock forbids it. Acquire is the only lock verb
          // that can 409 - extend and release answer 400/403/404.
          .errors({409, 501})  // 501: locking disabled, LockHandlers::check_locking_enabled
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
          .errors({501})  // Locking disabled: LockHandlers::check_locking_enabled
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
          .errors({501})  // Locking disabled: LockHandlers::check_locking_enabled
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
          // 403 `lock-not-owner` from LockManager, passed through verbatim:
          // the lock exists but belongs to a different client. Nothing to do
          // with the auth middleware's 403, so it is declared here and stands
          // whether or not authentication is on. 400 and 404 (invalid
          // expiration, no such lock) are in the blanket set already, and 409
          // is deliberately absent - acquire is the only verb that conflicts.
          .errors({403, 501})  // 501: locking disabled, LockHandlers::check_locking_enabled
          .operation_id(std::string("extend") + capitalize(et.singular) + "Lock");

      reg.del<http::NoContent>(entity_path + "/locks/{lock_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return lock_handlers_->del_lock(req);
                               })
          .tag("Locking")
          .summary(std::string("Release lock on ") + et.singular)
          .description(std::string("Releases a lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership", true, client_id_schema)
          // 403 `lock-not-owner` from LockManager, passed through verbatim:
          // the lock exists but belongs to a different client. Nothing to do
          // with the auth middleware's 403, so it is declared here and stands
          // whether or not authentication is on. 400 and 404 (invalid
          // expiration, no such lock) are in the blanket set already, and 409
          // is deliberately absent - acquire is the only verb that conflicts.
          .errors({403, 501})  // 501: locking disabled, LockHandlers::check_locking_enabled
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
    //
    // All eight routes funnel backend failures through one total mapper,
    // `script_backend_error` (script_handlers.cpp), but the mapper's totality
    // is NOT a licence to declare its whole range everywhere: what each route
    // can answer is what its own `ScriptProvider` method returns. Declared per
    // route against the shipped `DefaultScriptProvider`, so
    // `GET .../scripts` does not tell a client it might answer 413:
    //
    //   list_scripts      - no backend error at all
    //   get_script        - NotFound / Internal
    //   upload_script     - FileTooLarge (413), InvalidInput, Internal
    //   delete_script     - ManagedScript / AlreadyRunning (409), NotFound, Internal
    //   start_execution   - ConcurrencyLimit (429), UnsupportedType, InvalidInput, NotFound
    //   get_execution     - NotFound
    //   control_execution - NotRunning (409), InvalidInput, NotFound
    //   delete_execution  - AlreadyRunning (409), NotFound
    //
    // The 429 is the script manager's concurrency limit, not the HTTP rate
    // limiter's: it is reachable whether or not `rate_limiting.enabled` is set,
    // which is why it is declared on the one route that answers it rather than
    // by the registry's limiter gate. 409 / 413 / 429 are all out of the
    // emitted-status recorder's reach - they need a backend in a state no
    // integration fixture puts it in.
    //
    // A `ScriptProvider` plugin may return codes its `DefaultScriptProvider`
    // counterpart never does (`AlreadyExists` exists for exactly that, see
    // `script_provider.hpp`). These declarations describe the shipped backend;
    // widening them to the mapper's full range for every route would make the
    // document describe no backend at all.
    if (script_handlers_ && (et_type_str == "apps" || et_type_str == "components")) {
      reg.multipart_upload<http::Created<dto::ScriptUploadResponse>>(
             entity_path + "/scripts",
             [this](http::TypedRequest req, const http::MultipartBody & body)
                 -> http::Result<std::pair<http::Created<dto::ScriptUploadResponse>, http::ResponseAttachments>> {
               return script_handlers_->upload_script(req, body);
             })
          .tag("Scripts")
          .summary(std::string("Upload diagnostic script for ") + et.singular)
          .description(std::string("Uploads a diagnostic script for this ") + et.singular + ".")
          // Part names read off ScriptHandlers::upload_script. `file` is the
          // only one whose absence is a 400; `metadata` must parse as JSON or
          // the request is rejected.
          .multipart_body("Script file, with optional metadata",
                          {openapi::MultipartPart{"file",
                                                  "Script content. The part's filename becomes the script name and "
                                                  "decides the interpreter.",
                                                  nlohmann::json{}, true, "application/octet-stream"},
                           openapi::MultipartPart{"metadata",
                                                  "JSON document stored with the script. Malformed JSON is rejected "
                                                  "with 400.",
                                                  nlohmann::json{{"type", "object"}, {"additionalProperties", true}},
                                                  false, "application/json"}})
          .success_description("Script uploaded")
          // DefaultScriptProvider::upload_script -> FileTooLarge
          .errors({413, 501})
          .operation_id(std::string("upload") + capitalize(et.singular) + "Script");

      reg.get<dto::ScriptList>(entity_path + "/scripts",
                               [this](http::TypedRequest req) -> http::Result<dto::ScriptList> {
                                 return script_handlers_->list_scripts(req);
                               })
          .tag("Scripts")
          .summary(std::string("List scripts for ") + et.singular)
          .description(std::string("Lists all diagnostic scripts for this ") + et.singular + ".")
          // DefaultScriptProvider::list_scripts returns no backend error
          .errors({501})
          .operation_id(std::string("list") + capitalize(et.singular) + "Scripts");

      reg.get<dto::ScriptMetadata>(entity_path + "/scripts/{script_id}",
                                   [this](http::TypedRequest req) -> http::Result<dto::ScriptMetadata> {
                                     return script_handlers_->get_script(req);
                                   })
          .tag("Scripts")
          .summary(std::string("Get script metadata for ") + et.singular)
          .description(std::string("Returns metadata of a specific script for this ") + et.singular + ".")
          // DefaultScriptProvider::get_script -> NotFound / Internal only
          .errors({501})
          .operation_id(std::string("get") + capitalize(et.singular) + "Script");

      reg.del<http::NoContent>(entity_path + "/scripts/{script_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return script_handlers_->delete_script(req);
                               })
          .tag("Scripts")
          .summary(std::string("Delete script for ") + et.singular)
          .description(std::string("Deletes a diagnostic script from this ") + et.singular + ".")
          // DefaultScriptProvider::delete_script -> ManagedScript / AlreadyRunning
          .errors({409, 501})
          .operation_id(std::string("delete") + capitalize(et.singular) + "Script");

      reg.post<http::Accepted<dto::ScriptExecution>>(
             entity_path + "/scripts/{script_id}/executions",
             [this](http::TypedRequest req)
                 -> http::Result<std::pair<http::Accepted<dto::ScriptExecution>, http::ResponseAttachments>> {
               return script_handlers_->start_execution(req);
             })
          .tag("Scripts")
          .summary(std::string("Start script execution for ") + et.singular)
          .description(std::string("Starts execution of a diagnostic script on this ") + et.singular + ".")
          // The handler parses this body by hand (framework escape hatch) so it
          // can keep its own 400 messages, which is why the schema is declared
          // here rather than derived from a TBody template parameter. It still
          // comes from a DTO descriptor, so the declaration and the fields the
          // handler reads are one edit apart, not two files apart.
          .request_body<dto::ScriptExecutionRequest>("Execution parameters")
          .success_description("Execution started")
          // DefaultScriptProvider::start_execution -> ConcurrencyLimit
          .errors({429, 501})
          .operation_id(std::string("start") + capitalize(et.singular) + "ScriptExecution");

      reg.get<dto::ScriptExecution>(entity_path + "/scripts/{script_id}/executions/{execution_id}",
                                    [this](http::TypedRequest req) -> http::Result<dto::ScriptExecution> {
                                      return script_handlers_->get_execution(req);
                                    })
          .tag("Scripts")
          .summary(std::string("Get execution status for ") + et.singular)
          .description("Returns the current status of a script execution.")
          // DefaultScriptProvider::get_script -> NotFound / Internal only
          .errors({501})
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
          // DefaultScriptProvider::control_execution -> NotRunning
          .errors({409, 501})
          .operation_id(std::string("control") + capitalize(et.singular) + "ScriptExecution");

      reg.del<http::NoContent>(entity_path + "/scripts/{script_id}/executions/{execution_id}",
                               [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                                 return script_handlers_->delete_execution(req);
                               })
          .tag("Scripts")
          .summary(std::string("Remove completed execution for ") + et.singular)
          .description("Removes a completed script execution record.")
          // DefaultScriptProvider::delete_execution -> AlreadyRunning
          .errors({409, 501})
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

  reg.binary_download(
         "/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}/{file_id}",
         [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
           return bulkdata_handlers_->download(req);
         },
         handlers::BulkDataHandlers::download_media_types())
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

  reg.binary_download(
         "/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}/{file_id}",
         [this](http::TypedRequest req) -> http::Result<http::BinaryResponse> {
           return bulkdata_handlers_->download(req);
         },
         handlers::BulkDataHandlers::download_media_types())
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
      .description(
          "Server-Sent Events stream for real-time fault notifications. Each frame's `data:` field is a "
          "FaultStreamEvent, and the frame also carries `id:` (a monotonic event id) and `event:` (the "
          "event type), so a client can resume after a drop rather than restart.")
      // SSEFaultHandler::format_sse_event's shape - a third one, with no
      // `payload` key at all.
      .success_schema<dto::FaultStreamEvent>()
      // The other half of the replay protocol. The handler parses this header
      // (sse_fault_handler.cpp) and replays only events newer than the id, so
      // without it in the document a client has no way to reconnect without
      // gaps or duplicates - it is the only reason the `id:` field is there.
      .header_param("Last-Event-ID",
                    "Id of the last frame the client received. The stream resumes after it, replaying "
                    "whatever is still buffered. Omit to start from the next new event.",
                    /*required=*/false)
      .operation_id("streamFaults");

  reg.get<dto::FaultListResult>("/faults",
                                [this](http::TypedRequest req) -> http::Result<dto::FaultListResult> {
                                  return fault_handlers_->list_all_faults(req);
                                })
      .tag("Faults")
      .summary("List all faults globally")
      .description("Retrieve all faults across the system.")
      // The handler's return type is the opaque `FaultListResult` so the fault
      // store's items and the peers' merged items pass through byte-for-byte
      // (`JsonReader<FaultListItem>` would drop any vendor key a newer peer
      // adds). The wire shape here is nonetheless fixed: unlike the per-entity
      // list this route has no plugin-delegation branch, so it is always the
      // FaultManager's own list plus other gateways' merged items - never a
      // plugin's own JSON. Publishing the concrete schema is what lets a
      // generated client type this body at all.
      .success_schema<dto::Collection<dto::FaultListItem, dto::FaultListXMedkit>>()
      // 503 when the fault store cannot be read - same branch as the
      // per-entity list, and equally out of the recorder's reach.
      .errors({503})
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
      // A 204 cannot carry a body, so the "peers were not cleared" caveat this
      // route ships travels as a header - which makes declaring it the only way
      // a generated client can see it at all.
      .response_header(204, openapi::ResponseHeader{"X-Medkit-Local-Only",
                                                    "`true` when only the local FaultManager was cleared; faults held "
                                                    "by aggregated peers are untouched and must be cleared per peer."})
      // Reads `X-Client-Id` like every lock-guarded write, but deliberately not
      // `.lock_guarded()`: FaultHandlers::clear_all_faults_global never answers
      // 409. It consults the lock manager per fault and *skips* the ones on
      // entities locked by somebody else, then answers 204 anyway. Marking it
      // lock-guarded would publish a 409 this route cannot return - the exact
      // defect the marker exists to make visible.
      //
      // Nothing on the response reports which faults were skipped. The
      // `X-Medkit-Local-Only` header declared above is set unconditionally and
      // is about aggregated peers, not locks - do not read it as the skip
      // signal. A client that needs to know re-reads the entity's faults.
      .header_param("X-Client-Id",
                    "Identifies the calling client for lock ownership. Faults on entities locked by "
                    "a different client are silently skipped rather than cleared, and the request "
                    "still answers 204 - re-read the entity's faults to see what survived.",
                    false, nlohmann::json{{"type", "string"}})
      // 503 when the fault store cannot be read - see the list route above.
      .errors({503})
      .operation_id("clearAllFaults")
      .query<dto::FaultClearQuery>();

  // === Software Updates ===
  //
  // PR-403 commit 22: 8 update routes migrated to typed RouteRegistry API.
  // The handler instance may be null when no backend plugin is loaded;
  // `.gated_on(...)` short-circuits every route with a 501 in that case, so the
  // routes stay in the OpenAPI spec AND the spec declares the 501.
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
  auto updates_available = [this] {
    return update_handlers_ != nullptr;
  };

  reg.get<dto::UpdateList>("/updates",
                           [this](http::TypedRequest req) -> http::Result<dto::UpdateList> {
                             return update_handlers_->get_updates(req);
                           })
      .tag("Updates")
      .summary("List software updates")
      .description("Lists all registered software updates.")
      .gated_on(updates_available, kUpdate501)
      .operation_id("listUpdates")
      .query<dto::UpdateListQuery>();

  reg.post<dto::UpdateRegisterRequest, http::Created<dto::UpdateRegisterResponse>>(
         "/updates",
         [this](http::TypedRequest req, dto::UpdateRegisterRequest body)
             -> http::Result<std::pair<http::Created<dto::UpdateRegisterResponse>, http::ResponseAttachments>> {
           return update_handlers_->post_update(req, std::move(body));
         })
      .tag("Updates")
      .summary("Register a software update")
      .description("Registers a new software update descriptor.")
      .success_description("Update registered")
      .gated_on(updates_available, kUpdate501)
      .operation_id("registerUpdate");

  reg.get<dto::UpdateStatus>("/updates/{update_id}/status",
                             [this](http::TypedRequest req) -> http::Result<dto::UpdateStatus> {
                               return update_handlers_->get_status(req);
                             })
      .tag("Updates")
      .summary("Get update status")
      .description("Returns the current status and progress of an update.")
      .gated_on(updates_available, kUpdate501)
      .operation_id("getUpdateStatus");

  reg.put<http::Accepted<http::NoContent>>(
         "/updates/{update_id}/prepare",
         [this](http::TypedRequest req)
             -> http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>> {
           return update_handlers_->put_prepare(req);
         })
      .tag("Updates")
      .summary("Prepare update for execution")
      .description("Prepares an update for execution (downloads, validates).")
      .success_description("Update preparation started")
      .gated_on(updates_available, kUpdate501)
      // 409 when another update is already in progress or this one is being
      // deleted (update_handlers.cpp maps UpdateErrorCode::InProgress and
      // ::Deleting). The recorder cannot reach it: no fixture drives two
      // overlapping updates.
      .errors({409})
      .operation_id("prepareUpdate");

  reg.put<http::Accepted<http::NoContent>>(
         "/updates/{update_id}/execute",
         [this](http::TypedRequest req)
             -> http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>> {
           return update_handlers_->put_execute(req);
         })
      .tag("Updates")
      .summary("Execute update")
      .description("Starts executing a prepared update.")
      .success_description("Update execution started")
      .gated_on(updates_available, kUpdate501)
      // 409 when another update is already in progress or this one is being
      // deleted (update_handlers.cpp maps UpdateErrorCode::InProgress and
      // ::Deleting). The recorder cannot reach it: no fixture drives two
      // overlapping updates.
      .errors({409})
      .operation_id("executeUpdate");

  reg.put<http::Accepted<http::NoContent>>(
         "/updates/{update_id}/automated",
         [this](http::TypedRequest req)
             -> http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>> {
           return update_handlers_->put_automated(req);
         })
      .tag("Updates")
      .summary("Run automated update")
      .description("Runs a fully automated update (prepare + execute).")
      .success_description("Automated update started")
      .gated_on(updates_available, kUpdate501)
      // 409 when another update is already in progress or this one is being
      // deleted (update_handlers.cpp maps UpdateErrorCode::InProgress and
      // ::Deleting). The recorder cannot reach it: no fixture drives two
      // overlapping updates.
      .errors({409})
      .operation_id("automateUpdate");

  reg.get<dto::UpdateDetail>("/updates/{update_id}",
                             [this](http::TypedRequest req) -> http::Result<dto::UpdateDetail> {
                               return update_handlers_->get_update(req);
                             })
      .tag("Updates")
      .summary("Get update details")
      .description("Returns details of a specific update.")
      .gated_on(updates_available, kUpdate501)
      .operation_id("getUpdate");

  reg.del<http::NoContent>("/updates/{update_id}",
                           [this](http::TypedRequest req) -> http::Result<http::NoContent> {
                             return update_handlers_->del_update(req);
                           })
      .tag("Updates")
      .summary("Delete update")
      .description("Removes an update registration.")
      .gated_on(updates_available, kUpdate501)
      // 409 when another update is already in progress or this one is being
      // deleted (update_handlers.cpp maps UpdateErrorCode::InProgress and
      // ::Deleting). The recorder cannot reach it: no fixture drives two
      // overlapping updates.
      .errors({409})
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
      // AuthorizeRequest::parse_request accepts either encoding of the same
      // payload; declaring only JSON hid the one RFC 6749 clients default to.
      .accepts("application/x-www-form-urlencoded", SB::ref("AuthCredentials"))
      .operation_id("authorize")
      // Bad client credentials answer 401 independently of `auth.enabled`
      // (auth_handlers.cpp: authenticate() failure). The middleware's own 401 is
      // gated on the auth flag, so without this the document declares no 401 at
      // all on the one route whose whole purpose is to reject bad credentials.
      .errors({401})
      .error_renderer(openapi::ErrorRenderer::kOAuth2Error);

  reg.post<dto::AuthTokenResponse>("/auth/token",
                                   [this](http::TypedRequest req) -> http::Result<dto::AuthTokenResponse> {
                                     return auth_handlers_->post_token(req);
                                   })
      .tag("Authentication")
      .summary("Obtain access token")
      .description("Exchange credentials or refresh token for a JWT access token.")
      .request_body("Token request credentials", SB::ref("AuthCredentials"))
      .accepts("application/x-www-form-urlencoded", SB::ref("AuthCredentials"))
      .operation_id("getToken")
      // A refresh token that is expired, revoked or unknown answers 401, again
      // regardless of `auth.enabled`.
      .errors({401})
      .error_renderer(openapi::ErrorRenderer::kOAuth2Error);

  reg.post<dto::AuthRevokeResponse>("/auth/revoke",
                                    [this](http::TypedRequest req) -> http::Result<dto::AuthRevokeResponse> {
                                      return auth_handlers_->post_revoke(req);
                                    })
      .tag("Authentication")
      .summary("Revoke token")
      .description("Revoke an access or refresh token.")
      // No `.accepts(...)` and no `.errors({401})`, unlike the two above:
      // post_revoke parses JSON only, and per RFC 7009 §2.2 it must not reveal
      // whether the submitted token was valid, so it answers 200 either way.
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
      reg.put<http::Accepted<http::NoContent>>(
             base_lc + "/status/" + action,
             [this, action_str](http::TypedRequest req)
                 -> http::Result<std::pair<http::Accepted<http::NoContent>, http::ResponseAttachments>> {
               return lifecycle_handlers_->handle_transition(req, action_str);
             })
          .tag("Lifecycle")
          .summary(std::string("Request lifecycle transition '") + action + "'")
          .success_description("Lifecycle transition accepted")
          // 501: no LifecycleProvider, or the provider reports the transition
          // unsupported. 403 and 409 come from the same total mapper
          // (`to_error_info`, lifecycle_handlers.cpp): the provider can refuse
          // the transition outright (AccessDenied) or report the entity is not
          // in a state that allows it (PreconditionFailed). Neither is the auth
          // middleware's 403 or a lock 409 - they are the provider's own
          // answers, so they are declared here whether or not auth is on. The
          // recorder cannot reach them: the fixture has no provider at all, so
          // every run stops at the 501.
          .errors({403, 409, 501})
          .operation_id(std::string("put").append(entity_cap).append("Status").append(action_cap));
    }

    reg.get<dto::LifecycleStatusResponse>(base_lc + "/status",
                                          [this](http::TypedRequest req) -> http::Result<dto::LifecycleStatusResponse> {
                                            return lifecycle_handlers_->handle_get_status(req);
                                          })
        .tag("Lifecycle")
        .summary(std::string("Get ") + et_lc.second + " lifecycle status")
        // 501 when the provider reports the entity unsupported; 403/409 from
        // the same mapper - see the transition routes above.
        .errors({403, 409, 501})
        .operation_id(std::string("get") + entity_cap + "Status");
  }

  // Register all routes with cpp-httplib
  route_registry_->register_all(*srv, API_BASE_PATH);

  report_route_metadata_issues();
}

void RESTServer::report_route_metadata_issues() const {
  // The registry's self-checks (`errors()` handed a sub-400 status,
  // `response_header()` aimed at a status no response declares, a route with no
  // tag or no success schema) are only worth recording if something reads them.
  // Until this call existed they were collected and thrown away outside the unit
  // tests, so the comments promising they would be "surfaced" were promising
  // nothing on a shipped gateway.
  //
  // Logged, never fatal: every issue here is a defect in the *document*, and a
  // gateway that refused to serve traffic because one route is missing a summary
  // would trade a documentation bug for an outage.
  //
  // The summary line below is emitted unconditionally, including for a clean
  // route set, and that is deliberate: a log line that only appears on failure
  // cannot be asserted on, so the check itself would be guarded by nothing.
  // `test_openapi_contract.test.py::test_shipped_route_set_declares_complete_metadata`
  // waits for it and requires the error count to be zero, which is what makes
  // this a gate rather than a diagnostic nobody reads.
  std::size_t errors = 0;
  std::size_t warnings = 0;
  for (const auto & issue : route_registry_->validate_completeness()) {
    if (issue.severity == openapi::ValidationIssue::Severity::kError) {
      ++errors;
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "OpenAPI metadata error on %s: %s", issue.route.c_str(),
                   issue.message.c_str());
    } else {
      ++warnings;
      RCLCPP_WARN(rclcpp::get_logger("rest_server"), "OpenAPI metadata warning on %s: %s", issue.route.c_str(),
                  issue.message.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("rest_server"),
              "OpenAPI metadata check: %zu error(s), %zu warning(s) across %zu route(s)", errors, warnings,
              route_registry_->size());
  if (errors > 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Routes above publish incomplete OpenAPI metadata; generated clients will be wrong for them");
  }
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
