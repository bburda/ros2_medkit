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

#pragma once

#include <httplib.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <vector>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/core/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/http/handlers/handlers.hpp"
#include "ros2_medkit_gateway/core/http/http_server.hpp"
#include "ros2_medkit_gateway/core/http/rate_limiter.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"

// Forward declare RouteRegistry to avoid pulling in full header
namespace ros2_medkit_gateway {
namespace openapi {
class RouteRegistry;
}  // namespace openapi
}  // namespace ros2_medkit_gateway

namespace ros2_medkit_gateway {

class AggregationManager;
class GatewayNode;
class TriggerManager;

/**
 * @brief REST API server for ROS 2 Medkit Gateway.
 *
 * Provides a RESTful interface to ROS 2 functionality including:
 * - Discovery (areas, components)
 * - Data access (topic read/write)
 * - Operations (service calls, action goals)
 * - Configuration (parameter management)
 * - Fault management
 * - Authentication (OAuth2-like flow)
 *
 * The server delegates request handling to specialized handler classes
 * organized by domain (health, areas, components, operations, config, faults, auth).
 */
class RESTServer {
 public:
  RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
             const AuthConfig & auth_config, const RateLimitConfig & rate_limit_config,
             const TlsConfig & tls_config = TlsConfig{});
  ~RESTServer();

  RESTServer(const RESTServer &) = delete;
  RESTServer & operator=(const RESTServer &) = delete;
  RESTServer(RESTServer &&) = delete;
  RESTServer & operator=(RESTServer &&) = delete;

  void start();
  void stop();

  /// Set trigger handlers (called by GatewayNode after TriggerManager is created).
  /// Creates TriggerHandlers using the existing handler context and SSE client tracker.
  void set_trigger_handlers(TriggerManager & trigger_mgr);

  /// Set aggregation manager on the handler context (called by GatewayNode when aggregation is enabled).
  /// Must be called before start() so handlers can forward requests to peers.
  void set_aggregation_manager(AggregationManager * mgr);

  /// Check if TLS/HTTPS is enabled
  bool is_tls_enabled() const {
    return http_server_ && http_server_->is_tls_enabled();
  }

  /// True once cpp-httplib's listen() has reached its accept loop.
  /// GatewayNode uses this as the start-up readiness signal so shutdown
  /// cannot race a listen() that has not yet started.
  bool is_running() const {
    return http_server_ && http_server_->is_running();
  }

 private:
  void setup_routes();

  /// Log every issue `RouteRegistry::validate_completeness()` finds for the
  /// route set this configuration assembled, plus an unconditional summary line
  /// carrying the error and warning counts. Called once, after registration.
  ///
  /// Never fatal - an incomplete OpenAPI declaration is a documentation defect,
  /// not a reason to refuse traffic. The summary is what the integration suite
  /// asserts on, so the check is gated rather than merely logged.
  void report_route_metadata_issues() const;

  void setup_pre_routing_handler();
  void setup_global_error_handlers();

  // CORS helper methods
  void set_cors_headers(httplib::Response & res, const std::string & origin) const;
  bool is_origin_allowed(const std::string & origin) const;

  GatewayNode * node_;
  std::string host_;
  int port_;
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  TlsConfig tls_config_;
  std::unique_ptr<AuthManager> auth_manager_;
  std::unique_ptr<AuthMiddleware> auth_middleware_;
  std::unique_ptr<RateLimiter> rate_limiter_;

  // HTTP/HTTPS server manager
  std::unique_ptr<HttpServerManager> http_server_;

  // Handler context and domain-specific handlers
  std::unique_ptr<handlers::HandlerContext> handler_ctx_;
  std::unique_ptr<handlers::HealthHandlers> health_handlers_;
  std::unique_ptr<handlers::DiscoveryHandlers> discovery_handlers_;
  std::unique_ptr<handlers::DataHandlers> data_handlers_;
  std::unique_ptr<handlers::LifecycleHandlers> lifecycle_handlers_;
  std::unique_ptr<handlers::OperationHandlers> operation_handlers_;
  std::unique_ptr<handlers::ConfigHandlers> config_handlers_;
  std::unique_ptr<handlers::FaultHandlers> fault_handlers_;
  std::unique_ptr<handlers::AuthHandlers> auth_handlers_;
  std::shared_ptr<SSEClientTracker> sse_client_tracker_;
  std::unique_ptr<handlers::SSEFaultHandler> sse_fault_handler_;
  std::unique_ptr<handlers::BulkDataHandlers> bulkdata_handlers_;
  std::unique_ptr<handlers::CyclicSubscriptionHandlers> cyclic_sub_handlers_;
  std::unique_ptr<handlers::UpdateHandlers> update_handlers_;
  std::unique_ptr<handlers::LogHandlers> log_handlers_;
  std::unique_ptr<handlers::LockHandlers> lock_handlers_;
  std::unique_ptr<handlers::ScriptHandlers> script_handlers_;
  std::unique_ptr<handlers::DocsHandlers> docs_handlers_;
  std::unique_ptr<handlers::TriggerHandlers> trigger_handlers_;

  // Route registry: single source of truth for routes + OpenAPI metadata
  std::unique_ptr<openapi::RouteRegistry> route_registry_;
};

}  // namespace ros2_medkit_gateway
