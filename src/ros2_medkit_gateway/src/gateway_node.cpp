// Copyright 2025 bburda, mfaferek93
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

#include "ros2_medkit_gateway/gateway_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/network_utils.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/refresh_debounce.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
#include "ros2_medkit_gateway/core/thread_pool_config.hpp"
#include "ros2_medkit_gateway/param_utils.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

#include "ros2_medkit_gateway/core/http/handlers/sse_transport_provider.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

GatewayNode::GatewayNode(const rclcpp::NodeOptions & options) : Node("ros2_medkit_gateway", options) {
  RCLCPP_INFO(get_logger(), "Initializing ROS 2 Medkit Gateway...");

  // Declare parameters with defaults
  declare_parameter("server.host", "127.0.0.1");
  declare_parameter("server.port", 8080);

  // Thread-pool bounds (issue #440). Both pools default to a small fixed size
  // instead of scaling with the host core count, so the footprint stays the
  // same on a 4-core SBC and a 64-core server.
  //
  // executor_threads: the main executor delivers the node's own callbacks
  // (timers, graph events, subscriptions) and the service-response callbacks
  // that complete operation/action RPC futures. The RPC response callbacks
  // live in a shared Reentrant callback group (issue #575; see
  // ros2_common/callback_groups.hpp), so a second executor thread can
  // dispatch a response while a default-group callback (e.g. a discovery
  // refresh pass) is running - the default of 2 buys real RPC-response
  // parallelism. Timers and the SSE-fault / trigger-fault / rosout
  // subscriptions stay in the default MutuallyExclusive group on purpose:
  // refresh passes must stay serialized and those subscriptions rely on
  // in-order delivery. A single thread remains safe (a Reentrant group does
  // not require a second thread), and the blocking wait for an RPC runs on
  // the cpp-httplib pool thread (off the executor), never on an executor
  // thread, so it cannot deadlock the executor.
  //
  // http_thread_pool_size: each active SSE stream pins one worker for its
  // lifetime and each cold-/data wait parks one for up to
  // topic_sample_timeout_sec, so the pool defaults at or above
  // sse.max_clients + data_provider.cold_wait_cap (2 + 4 = 6) to avoid
  // starvation; main.cpp warns at startup if it is set below that sum. Both
  // values are clamped to a bounded range on read (see clamp_thread_count).
  declare_parameter("server.executor_threads", 2);
  declare_parameter("server.http_thread_pool_size", 6);

  // cpp-httplib keep-alive idle timeout (seconds). The library default (5s)
  // parks a request-pool worker on an idle keep-alive connection for that long;
  // with the small bounded pool above, a burst of short-lived client connections
  // (e.g. a client polling several endpoints per cycle) can pin every worker and
  // stall ordinary requests for up to one timeout per cycle. Defaulting to 2s
  // recovers workers quickly while retaining connection reuse for legitimate
  // clients. Clamped to [1, 3600] on read (see clamp_keep_alive_timeout).
  declare_parameter("server.keep_alive_timeout_sec", 2);
  // Safety-backstop refresh interval. Primary discovery refresh is driven
  // by rclcpp graph events; this only controls the periodic forced
  // refresh used when a graph event would otherwise be missed.
  declare_parameter("refresh_interval_ms", 30000);
  declare_parameter("discovery.refresh_debounce_ms", 1000);
  declare_parameter("fault_manager.namespace", "");
  declare_parameter("fault_manager.service_timeout_sec", 5.0);
  declare_parameter("cors.allowed_origins", std::vector<std::string>{});
  declare_parameter("cors.allowed_methods", std::vector<std::string>{"GET", "PUT", "POST", "DELETE", "OPTIONS"});
  declare_parameter("cors.allowed_headers", std::vector<std::string>{"Content-Type", "Accept"});
  declare_parameter("cors.allow_credentials", false);
  declare_parameter("cors.max_age_seconds", 86400);

  // SSE (Server-Sent Events) parameters
  // Concurrent SSE connections. Each one pins an HTTP pool worker for its
  // lifetime, so this is kept within the pool's budget: the default (2) plus
  // data_provider.cold_wait_cap (4) equals server.http_thread_pool_size (6),
  // leaving the pool able to cover the documented worst case. Raise the pool to
  // match if you raise this (main.cpp warns at startup otherwise). Clamped on
  // read.
  declare_parameter("sse.max_clients", 2);
  declare_parameter("sse.max_subscriptions", 100);  // Maximum active cyclic subscriptions across all entities
  declare_parameter("sse.max_duration_sec", 3600);  // Maximum subscription duration in seconds (1 hour default)

  // Log management parameters
  declare_parameter("logs.buffer_size",
                    200);  // Ring buffer capacity per node; entries exceeding this are dropped (oldest first)

  // Entity cache capacity. Pre-reserves the cache backing stores so steady-state
  // refreshes below this entity count cause no structural reallocation.
  // Valid range: 16-1000000 (values outside this range are clamped with a warning).
  // Default: 256 (covers most production graphs with headroom).
  declare_parameter("entity_cache.capacity", 256);

  // Topic-sample default timeout used by the data-access path.
  declare_parameter("topic_sample_timeout_sec", 1.0);

  // Parameter-service tuning consumed by Ros2ParameterTransport. Declared
  // here alongside the rest of the gateway parameters; the transport reads
  // the resolved values via get_parameter() at construction time.
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Timeout for ROS 2 parameter service calls (configurations endpoint)";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    desc.floating_point_range.push_back(range);
    declare_parameter("parameter_service_timeout_sec", 2.0, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Negative cache TTL for unavailable parameter services (0 = disabled)";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 3600.0;
    desc.floating_point_range.push_back(range);
    declare_parameter("parameter_service_negative_cache_sec", 60.0, desc);
  }

  // TLS/HTTPS parameters
  declare_parameter("server.tls.enabled", false);
  declare_parameter("server.tls.cert_file", "");
  declare_parameter("server.tls.key_file", "");
  declare_parameter("server.tls.ca_file", "");
  declare_parameter("server.tls.min_version", "1.2");
  // TODO(future): Add mutual_tls parameter when implemented
  // declare_parameter("server.tls.mutual_tls", false);

  // Authentication parameters (REQ_INTEROP_086, REQ_INTEROP_087)
  declare_parameter("auth.enabled", false);
  declare_parameter("auth.jwt_secret", "");
  declare_parameter("auth.jwt_public_key", "");
  declare_parameter("auth.jwt_algorithm", "HS256");
  declare_parameter("auth.token_expiry_seconds", 3600);
  declare_parameter("auth.refresh_token_expiry_seconds", 86400);
  declare_parameter("auth.require_auth_for", "write");
  declare_parameter("auth.issuer", "ros2_medkit_gateway");
  declare_parameter("auth.clients", std::vector<std::string>{});

  // OpenAPI documentation endpoints
  declare_parameter("docs.enabled", true);

  // Rate limiting parameters
  declare_parameter("rate_limiting.enabled", false);
  declare_parameter("rate_limiting.global_requests_per_minute", 600);
  declare_parameter("rate_limiting.client_requests_per_minute", 60);
  declare_parameter("rate_limiting.endpoint_limits", std::vector<std::string>{});
  declare_parameter("rate_limiting.client_cleanup_interval_seconds", 300);
  declare_parameter("rate_limiting.client_max_idle_seconds", 600);

  // Discovery mode parameters
  declare_parameter("discovery.mode", "runtime_only");  // runtime_only, manifest_only, hybrid
  declare_parameter("discovery.manifest_path", "");
  declare_parameter("discovery.manifest_strict_validation", true);
  declare_parameter("discovery.manifest.enabled", true);
  // Directory containing manifest fragment yaml files. When non-empty, the
  // gateway scans this directory on every manifest load / reload and merges
  // apps / components / functions on top of the base manifest. Empty =
  // disabled (default). See ManifestManager::set_fragments_dir for the
  // merge semantics.
  declare_parameter("discovery.manifest.fragments_dir", "");
  // CSV file listing manually inventoried assets (columns: id, manufacturer,
  // model, serial, hardware_rev, firmware, endpoint, role, area, plus extras).
  // Each row is appended to the manifest as a Component and merged into the
  // tree by id. Requires a manifest-backed discovery mode (manifest_only /
  // hybrid with discovery.manifest_path set). Empty = disabled (default).
  declare_parameter("discovery.inventory.csv_path", "");
  declare_parameter("discovery.runtime.enabled", true);

  // Software updates parameters
  declare_parameter("updates.enabled", false);

  // Scripts parameters
  declare_parameter("scripts.scripts_dir", "");
  declare_parameter("scripts.max_file_size_mb", 10);
  declare_parameter("scripts.max_concurrent_executions", 5);
  declare_parameter("scripts.default_timeout_sec", 300);
  declare_parameter("scripts.max_execution_history", 100);
  declare_parameter("scripts.allow_uploads", true);

  // Trigger subsystem parameters
  declare_parameter("triggers.enabled", true);
  declare_parameter("triggers.max_triggers", 1000);
  declare_parameter("triggers.on_restart_behavior", "reset");
  declare_parameter("triggers.storage.path", "");

  // Config-less threshold-rule (fault-trigger) engine parameters (issue #235).
  declare_parameter("fault_triggers.enabled", true);
  declare_parameter("fault_triggers.poll_interval_ms", 1000);
  declare_parameter("fault_triggers.storage.path", "");

  // Plugin framework parameters
  declare_parameter("plugins", std::vector<std::string>{});

  // Zero-config freeze-frames for plugin-backed entities (opt-out)
  declare_parameter("entity_freeze_frame.enabled", true);

  // Locking parameters
  declare_parameter("locking.enabled", true);
  declare_parameter("locking.default_max_expiration", 3600);
  declare_parameter("locking.cleanup_interval", 30);
  declare_parameter("locking.defaults.components.lock_required_scopes", std::vector<std::string>{""});
  declare_parameter("locking.defaults.components.breakable", true);
  declare_parameter("locking.defaults.apps.lock_required_scopes", std::vector<std::string>{""});
  declare_parameter("locking.defaults.apps.breakable", true);

  // Aggregation parameters (peer gateway federation)
  declare_parameter("aggregation.enabled", false);
  declare_parameter("aggregation.timeout_ms", 2000);
  declare_parameter("aggregation.announce", false);
  declare_parameter("aggregation.discover", false);
  declare_parameter("aggregation.mdns_service", std::string("_medkit._tcp.local"));
  declare_parameter("aggregation.mdns_name", std::string(""));  // defaults to hostname
  // Security: forward Authorization header to peers (default: false to prevent token leakage)
  declare_parameter("aggregation.forward_auth", false);
  // Security: require TLS for all peer URLs (default: false)
  declare_parameter("aggregation.require_tls", false);
  // URL scheme for mDNS-discovered peer URLs (default: "http")
  declare_parameter("aggregation.peer_scheme", std::string("http"));
  // Maximum number of mDNS-discovered peers (prevents unbounded growth from rogue announcements)
  declare_parameter("aggregation.max_discovered_peers", 50);
  // Static peers: parallel arrays of URLs and names.
  // Example: peer_urls=["http://localhost:8081"], peer_names=["subsystem_b"]
  declare_parameter("aggregation.peer_urls", std::vector<std::string>{""});
  declare_parameter("aggregation.peer_names", std::vector<std::string>{""});

  // Bulk data storage parameters
  declare_parameter("bulk_data.storage_dir", "/tmp/ros2_medkit_bulk_data");
  declare_parameter("bulk_data.max_upload_size", 104857600);  // 100MB
  declare_parameter("bulk_data.categories", std::vector<std::string>{});

  // Runtime (heuristic) discovery options
  // These control how nodes are mapped to SOVD entities in runtime mode
  declare_parameter("discovery.runtime.default_component.enabled", true);
  declare_parameter("discovery.runtime.create_functions_from_namespaces", true);
  declare_parameter("discovery.runtime.filter_internal_nodes", true);

  // Merge pipeline configuration (hybrid mode only)
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_apps", true);
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_functions", false);
  declare_parameter("discovery.merge_pipeline.gap_fill.namespace_whitelist", std::vector<std::string>{});
  declare_parameter("discovery.merge_pipeline.gap_fill.namespace_blacklist", std::vector<std::string>{});

  // Per-layer merge policy overrides (optional, empty string = use layer default)
  for (const auto & layer : {"manifest", "runtime"}) {
    for (const auto & fg : {"identity", "hierarchy", "live_data", "status", "metadata"}) {
      declare_parameter(std::string("discovery.merge_pipeline.layers.") + layer + "." + fg, std::string(""));
    }
  }

  // Check for removed parameters and warn users with stale YAML configs.
  // ROS 2 silently ignores undeclared parameters from YAML, so we must
  // explicitly check the parameter overrides to detect stale configs.
  {
    const auto & overrides = this->get_node_options().parameter_overrides();
    // Use fully-qualified parameter names to match YAML override paths.
    // E.g., YAML key "discovery.runtime.create_synthetic_areas" becomes
    // override name "discovery.runtime.create_synthetic_areas".
    static const std::vector<std::pair<std::string, std::string>> removed_params = {
        {"discovery.runtime.create_synthetic_areas",
         "Areas now come from manifest only. Use 'discovery.mode: hybrid' with a manifest file to define Areas."},
        {"discovery.runtime.create_synthetic_components",
         "Components are now host-derived in runtime mode. A single Component is auto-created from the hostname. "
         "Use 'discovery.mode: hybrid' with a manifest file to define custom Components."},
        {"discovery.runtime.grouping_strategy",
         "Replaced by 'discovery.runtime.create_functions_from_namespaces'. "
         "Set 'discovery.runtime.create_functions_from_namespaces: true' to group nodes by namespace into Functions."},
        {"discovery.runtime.synthetic_component_name_pattern",
         "Components are now host-derived in runtime mode. A single Component is auto-created from the hostname. "
         "Use 'discovery.mode: hybrid' with a manifest file to define custom Components."},
        {"discovery.runtime.topic_only_policy",
         "Topic-only discovery has been removed. All nodes are now discovered via the ROS 2 graph API. "
         "Remove this parameter from your configuration."},
        {"discovery.runtime.min_topics_for_component",
         "Topic-only discovery has been removed. All nodes are now discovered via the ROS 2 graph API. "
         "Remove this parameter from your configuration."},
        {"discovery.merge_pipeline.gap_fill.allow_heuristic_areas",
         "Areas now come from manifest only. Use 'discovery.mode: hybrid' with a manifest file to define Areas."},
        {"discovery.merge_pipeline.gap_fill.allow_heuristic_components",
         "Components are now host-derived in runtime mode. A single Component is auto-created from the hostname. "
         "Use 'discovery.mode: hybrid' with a manifest file to define custom Components."},
    };
    for (const auto & [name, message] : removed_params) {
      for (const auto & override : overrides) {
        if (override.get_name() == name) {
          RCLCPP_WARN(get_logger(), "Parameter '%s' has been removed. %s", name.c_str(), message.c_str());
          break;
        }
      }
    }
  }

  // Get parameter values
  server_host_ = get_parameter("server.host").as_string();
  server_port_ = static_cast<int>(get_parameter("server.port").as_int());
  refresh_interval_ms_ = static_cast<int>(get_parameter("refresh_interval_ms").as_int());
  // Read as int64 and validate BEFORE narrowing to int: a value above INT_MAX
  // would otherwise wrap into the valid range and silently bypass the range
  // check (e.g. 4294967296 -> 0, disabling debounce). 0 disables debouncing
  // (refresh on every graph event, the pre-#442 behaviour); otherwise coalesce
  // graph events into at most one refresh per this interval. An out-of-range
  // value is rejected with a warning and the default (1000 ms) is used.
  const int64_t refresh_debounce_raw = get_parameter("discovery.refresh_debounce_ms").as_int();
  if (refresh_debounce_raw < 0 || refresh_debounce_raw > 60000) {
    RCLCPP_WARN(get_logger(),
                "Invalid discovery.refresh_debounce_ms %" PRId64 "ms. Must be between 0-60000ms. Using default 1000ms.",
                refresh_debounce_raw);
    refresh_debounce_ms_ = 1000;
  } else {
    refresh_debounce_ms_ = static_cast<int>(refresh_debounce_raw);
  }

  // Build CORS configuration using builder pattern
  // Throws std::invalid_argument if configuration is invalid
  cors_config_ = CorsConfigBuilder()
                     .with_origins(get_parameter("cors.allowed_origins").as_string_array())
                     .with_methods(get_parameter("cors.allowed_methods").as_string_array())
                     .with_headers(get_parameter("cors.allowed_headers").as_string_array())
                     .with_credentials(get_parameter("cors.allow_credentials").as_bool())
                     .with_max_age(static_cast<int>(get_parameter("cors.max_age_seconds").as_int()))
                     .build();

  // Validate port range
  if (server_port_ < 1024 || server_port_ > 65535) {
    RCLCPP_ERROR(get_logger(), "Invalid port %d. Must be between 1024-65535. Using default 8080.", server_port_);
    server_port_ = 8080;
  }

  // Validate host
  if (server_host_.empty()) {
    RCLCPP_WARN(get_logger(), "Empty host specified. Using default 127.0.0.1");
    server_host_ = "127.0.0.1";
  }

  // Warn if binding to all interfaces
  if (server_host_ == "0.0.0.0") {
    RCLCPP_WARN(get_logger(), "Binding to 0.0.0.0 - REST API accessible from ALL network interfaces!");
  }

  // Validate backstop interval. Discovery is primarily driven by rclcpp
  // graph events polled every 100 ms; this value only governs the
  // safety-backstop timer that periodically forces a refresh in case a
  // graph event is missed.
  if (refresh_interval_ms_ < 100 || refresh_interval_ms_ > 60000) {
    RCLCPP_WARN(get_logger(),
                "Invalid backstop refresh interval %dms. Must be between 100-60000ms. Using default 30000ms.",
                refresh_interval_ms_);
    refresh_interval_ms_ = 30000;
  }

  // Log configuration
  RCLCPP_INFO(get_logger(), "Configuration: REST API at %s:%d, backstop refresh interval: %dms", server_host_.c_str(),
              server_port_, refresh_interval_ms_);

  if (cors_config_.enabled) {
    std::string origins_str;
    for (const auto & origin : cors_config_.allowed_origins) {
      if (!origins_str.empty()) {
        origins_str += ", ";
      }
      origins_str += origin;
    }

    std::string methods_str;
    for (const auto & method : cors_config_.allowed_methods) {
      if (!methods_str.empty()) {
        methods_str += ", ";
      }
      methods_str += method;
    }

    RCLCPP_INFO(get_logger(), "CORS enabled - origins: [%s], methods: [%s], credentials: %s, max_age: %ds",
                origins_str.c_str(), methods_str.c_str(), cors_config_.allow_credentials ? "true" : "false",
                cors_config_.max_age_seconds);
  } else {
    RCLCPP_INFO(get_logger(), "CORS: disabled (no configuration provided)");
  }

  // Build TLS/HTTPS configuration
  bool tls_enabled = get_parameter("server.tls.enabled").as_bool();
  if (tls_enabled) {
    try {
      tls_config_ = TlsConfigBuilder()
                        .with_enabled(true)
                        .with_cert_file(get_parameter("server.tls.cert_file").as_string())
                        .with_key_file(get_parameter("server.tls.key_file").as_string())
                        .with_ca_file(get_parameter("server.tls.ca_file").as_string())
                        .with_min_version(get_parameter("server.tls.min_version").as_string())
                        // TODO(future): Add .with_mutual_tls() when implemented
                        .build();
      // Note: HttpServerManager will log TLS configuration details
    } catch (const std::exception & e) {
      // Fail closed: TLS was explicitly requested but could not be built (e.g.
      // missing/invalid cert or key). Refuse to start rather than silently
      // serving plaintext HTTP. Disabling here would expose the API in the
      // clear under a configuration that asked for encryption.
      RCLCPP_FATAL(get_logger(),
                   "TLS is enabled (server.tls.enabled=true) but the TLS configuration is invalid: %s. "
                   "Refusing to start in plaintext - fix the certificate/key configuration or disable TLS.",
                   e.what());
      throw std::runtime_error(std::string("Invalid TLS configuration: ") + e.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "TLS/HTTPS: disabled");
    tls_config_ = TlsConfig{};
  }

  // Build Authentication configuration (REQ_INTEROP_086, REQ_INTEROP_087)
  bool auth_enabled = get_parameter("auth.enabled").as_bool();
  if (auth_enabled) {
    try {
      AuthConfigBuilder auth_builder;
      auth_builder.with_enabled(true)
          .with_jwt_secret(get_parameter("auth.jwt_secret").as_string())
          .with_jwt_public_key(get_parameter("auth.jwt_public_key").as_string())
          .with_algorithm(string_to_algorithm(get_parameter("auth.jwt_algorithm").as_string()))
          .with_token_expiry(static_cast<int>(get_parameter("auth.token_expiry_seconds").as_int()))
          .with_refresh_token_expiry(static_cast<int>(get_parameter("auth.refresh_token_expiry_seconds").as_int()))
          .with_require_auth_for(string_to_auth_requirement(get_parameter("auth.require_auth_for").as_string()))
          .with_issuer(get_parameter("auth.issuer").as_string());

      // Parse clients from configuration
      // Format: "client_id:client_secret:role" (e.g., "admin:secret123:admin")
      auto clients = get_parameter("auth.clients").as_string_array();
      for (const auto & client_str : clients) {
        if (client_str.empty()) {
          continue;
        }
        // Parse "client_id:client_secret:role"
        size_t first_colon = client_str.find(':');
        size_t last_colon = client_str.rfind(':');
        if (first_colon != std::string::npos && last_colon != std::string::npos && first_colon != last_colon) {
          std::string client_id = client_str.substr(0, first_colon);
          std::string client_secret = client_str.substr(first_colon + 1, last_colon - first_colon - 1);
          std::string role_str = client_str.substr(last_colon + 1);
          try {
            UserRole role = string_to_role(role_str);
            auth_builder.add_client(client_id, client_secret, role);
            RCLCPP_INFO(get_logger(), "Registered client '%s' with role '%s'", client_id.c_str(), role_str.c_str());
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Invalid role '%s' for client '%s': %s", role_str.c_str(), client_id.c_str(),
                        e.what());
          }
        } else {
          RCLCPP_WARN(get_logger(), "Invalid client format: '%s'. Expected 'client_id:client_secret:role'",
                      client_str.c_str());
        }
      }

      auth_config_ = auth_builder.build();
      RCLCPP_INFO(get_logger(), "Authentication enabled - algorithm: %s, require_auth_for: %s",
                  algorithm_to_string(auth_config_.jwt_algorithm).c_str(),
                  get_parameter("auth.require_auth_for").as_string().c_str());
    } catch (const std::exception & e) {
      // Fail closed: authentication was explicitly requested but could not be
      // built (e.g. empty jwt_secret). Refuse to start rather than silently
      // serving an unauthenticated API under a configuration that asked for
      // auth.
      RCLCPP_FATAL(get_logger(),
                   "Authentication is enabled (auth.enabled=true) but the auth configuration is invalid: %s. "
                   "Refusing to start unauthenticated - fix the auth configuration or disable auth.",
                   e.what());
      throw std::runtime_error(std::string("Invalid authentication configuration: ") + e.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "Authentication: disabled");
    auth_config_ = AuthConfig{};
  }

  // Build Rate Limiting configuration
  bool rate_limit_enabled = get_parameter("rate_limiting.enabled").as_bool();
  if (rate_limit_enabled) {
    try {
      RateLimitConfigBuilder rl_builder;
      rl_builder.with_enabled(true)
          .with_global_rpm(static_cast<int>(get_parameter("rate_limiting.global_requests_per_minute").as_int()))
          .with_client_rpm(static_cast<int>(get_parameter("rate_limiting.client_requests_per_minute").as_int()))
          .with_cleanup_interval(
              static_cast<int>(get_parameter("rate_limiting.client_cleanup_interval_seconds").as_int()))
          .with_max_idle(static_cast<int>(get_parameter("rate_limiting.client_max_idle_seconds").as_int()));

      // Parse endpoint limits from "pattern:rpm" format
      auto endpoint_strs = get_parameter("rate_limiting.endpoint_limits").as_string_array();
      for (const auto & entry : endpoint_strs) {
        if (entry.empty()) {
          continue;
        }
        size_t colon = entry.rfind(':');
        if (colon != std::string::npos && colon > 0) {
          std::string pattern = entry.substr(0, colon);
          try {
            int rpm = std::stoi(entry.substr(colon + 1));
            rl_builder.add_endpoint_limit(pattern, rpm);
          } catch (...) {
            RCLCPP_WARN(get_logger(), "Non-numeric RPM in endpoint_limit: '%s'", entry.c_str());
            continue;
          }
        } else {
          RCLCPP_WARN(get_logger(), "Invalid endpoint_limit format: '%s'. Expected 'pattern:rpm'", entry.c_str());
        }
      }

      rate_limit_config_ = rl_builder.build();
      RCLCPP_INFO(get_logger(), "Rate limiting enabled - global: %d rpm, per-client: %d rpm",
                  rate_limit_config_.global_requests_per_minute, rate_limit_config_.client_requests_per_minute);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid rate limiting configuration: %s. Rate limiting disabled.", e.what());
      rate_limit_config_ = RateLimitConfig{};
    }
  } else {
    RCLCPP_INFO(get_logger(), "Rate limiting: disabled");
    rate_limit_config_ = RateLimitConfig{};
  }

  // Initialize managers
  discovery_mgr_ = std::make_unique<DiscoveryManager>(this);

  // Configure and initialize discovery manager
  DiscoveryConfig discovery_config;

  auto mode_str = get_parameter("discovery.mode").as_string();
  discovery_config.mode = parse_discovery_mode(mode_str);
  if (mode_str != "runtime_only" && mode_str != "manifest_only" && mode_str != "hybrid") {
    RCLCPP_WARN(get_logger(), "Unknown discovery.mode '%s', defaulting to 'runtime_only'", mode_str.c_str());
  }

  discovery_config.manifest_path = get_parameter("discovery.manifest_path").as_string();
  discovery_config.manifest_strict_validation = get_parameter("discovery.manifest_strict_validation").as_bool();
  discovery_config.manifest_enabled = get_parameter("discovery.manifest.enabled").as_bool();
  discovery_config.manifest_fragments_dir = get_parameter("discovery.manifest.fragments_dir").as_string();
  discovery_config.inventory_csv_path = get_parameter("discovery.inventory.csv_path").as_string();
  discovery_config.runtime_enabled = get_parameter("discovery.runtime.enabled").as_bool();

  // Runtime discovery options
  discovery_config.runtime.default_component_enabled =
      get_parameter("discovery.runtime.default_component.enabled").as_bool();
  discovery_config.runtime.create_functions_from_namespaces =
      get_parameter("discovery.runtime.create_functions_from_namespaces").as_bool();
  discovery_config.runtime.filter_internal_nodes = get_parameter("discovery.runtime.filter_internal_nodes").as_bool();
  filter_internal_nodes_ = discovery_config.runtime.filter_internal_nodes;

  // Merge pipeline gap-fill configuration (hybrid mode)
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_apps =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_apps").as_bool();
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_functions =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_functions").as_bool();
  discovery_config.merge_pipeline.gap_fill.namespace_whitelist =
      get_parameter("discovery.merge_pipeline.gap_fill.namespace_whitelist").as_string_array();
  discovery_config.merge_pipeline.gap_fill.namespace_blacklist =
      get_parameter("discovery.merge_pipeline.gap_fill.namespace_blacklist").as_string_array();

  // Read per-layer merge policy overrides
  for (const auto & layer : {"manifest", "runtime"}) {
    for (const auto & fg : {"identity", "hierarchy", "live_data", "status", "metadata"}) {
      auto val = get_parameter(std::string("discovery.merge_pipeline.layers.") + layer + "." + fg).as_string();
      if (!val.empty()) {
        discovery_config.merge_pipeline.layer_policies[layer][fg] = val;
      }
    }
  }

  if (!discovery_mgr_->initialize(discovery_config)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize discovery manager");
    throw std::runtime_error("Discovery initialization failed");
  }

  const auto topic_sample_timeout_sec = get_parameter("topic_sample_timeout_sec").as_double();
  topic_transport_ = std::make_shared<ros2::Ros2TopicTransport>(this, topic_sample_timeout_sec);
  data_access_mgr_ = std::make_unique<DataAccessManager>(topic_transport_, topic_sample_timeout_sec);
  // Shared callback groups for blocking-RPC response dispatch and action
  // status tracking (issue #575). Created once, here on the startup thread,
  // before any executor spins this node and before RESTServer threads exist
  // (see ros2_common/callback_groups.hpp for the thread-safety contract).
  callback_groups_ = ros2_common::create_gateway_callback_groups(*this);
  service_transport_ = std::make_shared<ros2::Ros2ServiceTransport>(this, callback_groups_.rpc_reentrant);
  action_transport_ =
      std::make_shared<ros2::Ros2ActionTransport>(this, callback_groups_.rpc_reentrant, callback_groups_.action_status);
  // Budget for every service call and every action RPC, including the action
  // cancel (issue #576 removed the special 15s cancel floor, making this the
  // sole cancel budget). Degenerate values are not cosmetic here: 0 or a
  // negative makes the response wait expire immediately, so every cancel
  // answers 504 unless the status stream wins the race, and an unbounded
  // value pins an HTTP worker for as long as it says. Clamp to the documented
  // range with a warning, like the sibling timeouts.
  static constexpr int64_t kMinServiceCallTimeoutSec = 1;
  static constexpr int64_t kMaxServiceCallTimeoutSec = 3600;
  const auto raw_service_call_timeout =
      declare_parameter<int64_t>("service_call_timeout_sec", static_cast<int64_t>(10));
  const auto clamped_service_call_timeout =
      std::clamp(raw_service_call_timeout, kMinServiceCallTimeoutSec, kMaxServiceCallTimeoutSec);
  if (clamped_service_call_timeout != raw_service_call_timeout) {
    RCLCPP_WARN(get_logger(), "service_call_timeout_sec %" PRId64 " clamped to %" PRId64, raw_service_call_timeout,
                clamped_service_call_timeout);
  }
  const auto service_call_timeout_sec = static_cast<int>(clamped_service_call_timeout);
  operation_mgr_ = std::make_unique<OperationManager>(service_transport_, action_transport_, discovery_mgr_.get(),
                                                      service_call_timeout_sec);

  // Parameter-service tuning parameters were declared at construction-time
  // alongside the rest of the gateway parameters; resolve them here.
  const double parameter_service_timeout_sec = get_parameter("parameter_service_timeout_sec").as_double();
  const double parameter_service_negative_cache_sec = get_parameter("parameter_service_negative_cache_sec").as_double();
  parameter_transport_ = std::make_shared<ros2::Ros2ParameterTransport>(this, parameter_service_timeout_sec,
                                                                        parameter_service_negative_cache_sec);
  config_mgr_ = std::make_unique<ConfigurationManager>(parameter_transport_);
  fault_service_transport_ = std::make_shared<ros2::Ros2FaultServiceTransport>(this);
  fault_mgr_ = std::make_unique<FaultManager>(fault_service_transport_);

  // Initialize bulk data store
  auto bd_storage_dir = get_parameter("bulk_data.storage_dir").as_string();
  auto bd_max_upload = static_cast<size_t>(get_parameter("bulk_data.max_upload_size").as_int());
  auto bd_categories = get_parameter("bulk_data.categories").as_string_array();
  bd_categories.erase(std::remove_if(bd_categories.begin(), bd_categories.end(),
                                     [](const auto & item) {
                                       return item.empty();
                                     }),
                      bd_categories.end());

  bulk_data_store_ = std::make_unique<BulkDataStore>(bd_storage_dir, bd_max_upload, bd_categories);
  RCLCPP_INFO(get_logger(), "Bulk data store: dir=%s, max_upload=%zuB, categories=%zu", bd_storage_dir.c_str(),
              bd_max_upload, bd_categories.size());

  // Initialize subscription manager for cyclic subscriptions
  auto max_subscriptions = static_cast<size_t>(get_parameter("sse.max_subscriptions").as_int());
  subscription_mgr_ = std::make_unique<SubscriptionManager>(max_subscriptions);
  RCLCPP_INFO(get_logger(), "Subscription manager: max_subscriptions=%zu", max_subscriptions);

  // Create SSE client tracker (shared between SseTransportProvider and SSEFaultHandler).
  // Clamp like the pool knobs (issue #440): read raw, a negative value would wrap
  // to a huge size_t. Bounded to [1, 1024]; main.cpp warns at startup if this
  // exceeds the HTTP pool budget (pool < sse.max_clients + cold_wait_cap).
  auto max_sse_clients = clamp_thread_count(get_parameter("sse.max_clients").as_int(), 1, 1024);
  sse_client_tracker_ = std::make_shared<SSEClientTracker>(max_sse_clients);

  // Initialize resource sampler and transport registries
  sampler_registry_ = std::make_unique<ResourceSamplerRegistry>();
  transport_registry_ = std::make_unique<TransportRegistry>();

  // Initialize plugin manager
  plugin_mgr_ = std::make_unique<PluginManager>();
  plugin_mgr_->set_registries(*sampler_registry_, *transport_registry_);
  auto plugin_names = get_parameter("plugins").as_string_array();
  plugin_names.erase(std::remove_if(plugin_names.begin(), plugin_names.end(),
                                    [](const auto & item) {
                                      return item.empty();
                                    }),
                     plugin_names.end());
  std::vector<PluginConfig> configs;
  if (!plugin_names.empty()) {
    // Plugin name validation: alphanumeric, underscore, hyphen only (max 256 chars)
    auto is_valid_plugin_name = [](const std::string & name) -> bool {
      if (name.empty() || name.size() > 256) {
        return false;
      }
      return std::all_of(name.begin(), name.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
      });
    };
    std::unordered_set<std::string> seen_names;
    for (const auto & pname : plugin_names) {
      if (!seen_names.insert(pname).second) {
        RCLCPP_WARN(get_logger(), "Duplicate plugin name '%s' - skipping", pname.c_str());
        continue;
      }
      if (!is_valid_plugin_name(pname)) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid plugin name '%s': must be alphanumeric, underscore, or hyphen (max 256 chars)",
                     pname.c_str());
        continue;
      }
      auto path_param = "plugins." + pname + ".path";
      rcl_interfaces::msg::ParameterDescriptor path_desc;
      path_desc.read_only = true;
      path_desc.description = "Absolute path to plugin .so file";
      declare_parameter(path_param, std::string(""), path_desc);
      auto path = get_parameter(path_param).as_string();
      if (path.empty()) {
        RCLCPP_ERROR(get_logger(), "Plugin '%s' has no path configured", pname.c_str());
        continue;
      }
      auto plugin_config = extract_plugin_config(this, pname);
      if (!plugin_config.empty()) {
        RCLCPP_INFO(get_logger(), "Plugin '%s' config: %zu key(s)", pname.c_str(), plugin_config.size());
      }
      configs.push_back({pname, path, std::move(plugin_config)});
    }
  }

  auto loaded = plugin_mgr_->load_plugins(configs);
  plugin_mgr_->configure_plugins();
  plugin_ctx_ = make_gateway_plugin_context(this, fault_mgr_.get(), sampler_registry_.get());
  plugin_mgr_->set_context(*plugin_ctx_);
  RCLCPP_INFO(get_logger(), "Loaded %zu plugin(s)", loaded);

  // Register IntrospectionProvider plugins
  auto introspection_providers = plugin_mgr_->get_named_introspection_providers();
  if (!introspection_providers.empty()) {
    if (discovery_mgr_->get_mode() == DiscoveryMode::HYBRID) {
      // Hybrid: add as pipeline layers for merge
      for (auto & [name, provider] : introspection_providers) {
        discovery_mgr_->add_plugin_layer(name, provider);
      }
      discovery_mgr_->refresh_pipeline();
    }
    // Non-hybrid modes: plugin entities are injected during refresh_cache().
    // Entity ownership is also registered during refresh_cache() (single introspect() call
    // per plugin handles both injection and ownership). No separate init-time registration
    // needed because refresh_cache() runs before the HTTP server accepts requests.
  }

  // Zero-config freeze-frames need the shared subscription executor, which is
  // built after the node (main.cpp). Its wiring is deferred to
  // init_entity_freeze_frame_capture(), called once the executor exists.

  // Initialize log manager (subscribes to /rosout, delegates to plugin if available)
  static constexpr int kMinBufferSize = 1;
  static constexpr int kMaxBufferSize = 100000;
  auto raw_buffer_size = get_parameter("logs.buffer_size").as_int();
  auto clamped =
      std::clamp(raw_buffer_size, static_cast<int64_t>(kMinBufferSize), static_cast<int64_t>(kMaxBufferSize));
  if (clamped != raw_buffer_size) {
    RCLCPP_WARN(get_logger(), "logs.buffer_size %" PRId64 " clamped to %" PRId64, raw_buffer_size, clamped);
  }
  auto log_buffer_size = static_cast<size_t>(clamped);

  // Apply entity_cache.capacity: clamp to [16, 1000000] and pre-reserve the cache.
  // Reserve is called here (in the ctor) so the backing stores are sized before
  // the first refresh_cache() call. thread_safe_cache_ is a value member so it
  // is already default-constructed at this point; reserve() only grows it.
  static constexpr int kMinCacheCapacity = 16;
  static constexpr int kMaxCacheCapacity = 1000000;
  auto raw_cache_capacity = get_parameter("entity_cache.capacity").as_int();
  auto clamped_cache_capacity =
      std::clamp(raw_cache_capacity, static_cast<int64_t>(kMinCacheCapacity), static_cast<int64_t>(kMaxCacheCapacity));
  if (clamped_cache_capacity != raw_cache_capacity) {
    RCLCPP_WARN(get_logger(), "entity_cache.capacity %" PRId64 " clamped to %" PRId64, raw_cache_capacity,
                clamped_cache_capacity);
  }
  thread_safe_cache_.reserve(static_cast<size_t>(clamped_cache_capacity));

  log_source_ = std::make_shared<ros2::Ros2LogSource>(this);
  // Inject a sink that forwards LogManager's neutral diagnostics back to the
  // gateway's rclcpp logger so plugin-provider exceptions remain visible to
  // /rosout, ros2 bag record --all, and any other standard observability path.
  auto log_sink = [this](int level, std::string_view msg) {
    if (level >= LogManager::kLogLevelError) {
      RCLCPP_ERROR(get_logger(), "%.*s", static_cast<int>(msg.size()), msg.data());
    } else {
      RCLCPP_WARN(get_logger(), "%.*s", static_cast<int>(msg.size()), msg.data());
    }
  };
  log_mgr_ = std::make_unique<LogManager>(log_source_, plugin_mgr_.get(), log_buffer_size, std::move(log_sink));

  // Initialize update manager
  auto updates_enabled = get_parameter("updates.enabled").as_bool();
  if (updates_enabled) {
    update_mgr_ = std::make_unique<UpdateManager>();
    auto * update_provider = plugin_mgr_->get_update_provider();
    if (update_provider) {
      update_mgr_->set_backend(update_provider);
      RCLCPP_INFO(get_logger(), "Update backend provided by plugin");
    } else {
      RCLCPP_WARN(get_logger(), "Updates enabled but no UpdateProvider plugin loaded");
    }
  }

  // Initialize lock manager
  auto locking_enabled = get_parameter("locking.enabled").as_bool();
  if (locking_enabled) {
    LockConfig lock_config;
    lock_config.enabled = true;
    lock_config.default_max_expiration = static_cast<int>(get_parameter("locking.default_max_expiration").as_int());
    lock_config.cleanup_interval = static_cast<int>(get_parameter("locking.cleanup_interval").as_int());

    // Helper: parse lock_required_scopes, filtering out empty-string sentinel
    auto parse_scopes = [&](const std::string & param_name) {
      auto raw = get_parameter(param_name).as_string_array();
      std::vector<std::string> scopes;
      for (const auto & s : raw) {
        if (!s.empty()) {
          scopes.push_back(s);
        }
      }
      return scopes;
    };

    EntityLockConfig comp_config;
    comp_config.required_scopes = parse_scopes("locking.defaults.components.lock_required_scopes");
    comp_config.breakable = get_parameter("locking.defaults.components.breakable").as_bool();
    lock_config.type_defaults["component"] = comp_config;

    EntityLockConfig app_config;
    app_config.required_scopes = parse_scopes("locking.defaults.apps.lock_required_scopes");
    app_config.breakable = get_parameter("locking.defaults.apps.breakable").as_bool();
    lock_config.type_defaults["app"] = app_config;

    // Apply per-entity lock overrides from manifest
    auto * manifest_mgr = discovery_mgr_->get_manifest_manager();
    if (manifest_mgr) {
      auto overrides = manifest_mgr->get_lock_overrides();
      for (auto & [entity_id, manifest_lock] : overrides) {
        EntityLockConfig entity_config;
        entity_config.required_scopes = manifest_lock.required_scopes;
        entity_config.breakable = manifest_lock.breakable;
        entity_config.max_expiration = manifest_lock.max_expiration;
        lock_config.entity_overrides[entity_id] = entity_config;
      }
      if (!overrides.empty()) {
        RCLCPP_INFO(get_logger(), "Lock manager: %zu entity overrides from manifest", overrides.size());
      }
    }

    lock_manager_ = std::make_unique<LockManager>(get_thread_safe_cache(), lock_config);

    auto cleanup_interval = lock_config.cleanup_interval;
    lock_cleanup_timer_ = this->create_wall_timer(std::chrono::seconds(cleanup_interval), [this]() {
      auto expired = lock_manager_->cleanup_expired();
      for (const auto & info : expired) {
        RCLCPP_INFO(get_logger(), "Lock %s expired on entity %s", info.lock_id.c_str(), info.entity_id.c_str());

        // Clean up temporary resources within expired lock scope
        auto scope_includes = [&](const std::string & collection) {
          return info.scopes.empty() ||
                 std::find(info.scopes.begin(), info.scopes.end(), collection) != info.scopes.end();
        };

        // Remove cyclic subscriptions for the entity
        if (scope_includes("cyclic-subscriptions") && subscription_mgr_) {
          auto subs = subscription_mgr_->list(info.entity_id);
          for (const auto & sub : subs) {
            if (subscription_mgr_->remove(sub.id)) {
              RCLCPP_INFO(get_logger(), "Removed subscription %s on lock expiry", sub.id.c_str());
            }
          }
        }
      }
    });

    RCLCPP_INFO(get_logger(), "Lock manager: enabled, max_expiration=%ds, cleanup_interval=%ds",
                lock_config.default_max_expiration, lock_config.cleanup_interval);
  } else {
    RCLCPP_INFO(get_logger(), "Lock manager: disabled");
  }

  // Scripts
  script_mgr_ = std::make_unique<ScriptManager>();
  auto * plugin_script_provider = plugin_mgr_->get_script_provider();
  if (plugin_script_provider) {
    script_mgr_->set_backend(plugin_script_provider);
    RCLCPP_INFO(get_logger(), "Using plugin ScriptProvider");
  } else {
    auto scripts_dir = get_parameter("scripts.scripts_dir").as_string();
    if (!scripts_dir.empty()) {
      ScriptsConfig scripts_config;
      scripts_config.scripts_dir = scripts_dir;
      scripts_config.max_file_size_mb = static_cast<int>(get_parameter("scripts.max_file_size_mb").as_int());
      scripts_config.max_concurrent_executions =
          static_cast<int>(get_parameter("scripts.max_concurrent_executions").as_int());
      scripts_config.default_timeout_sec = static_cast<int>(get_parameter("scripts.default_timeout_sec").as_int());
      scripts_config.max_execution_history = static_cast<int>(get_parameter("scripts.max_execution_history").as_int());
      scripts_config.allow_uploads = get_parameter("scripts.allow_uploads").as_bool();

      // Load manifest-defined scripts
      auto * manifest_mgr = discovery_mgr_->get_manifest_manager();
      if (manifest_mgr) {
        scripts_config.entries = manifest_mgr->get_scripts();
        if (!scripts_config.entries.empty()) {
          RCLCPP_INFO(get_logger(), "Loaded %zu manifest-defined scripts", scripts_config.entries.size());
        }
      }

      default_script_provider_ = std::make_unique<DefaultScriptProvider>(scripts_config);
      script_mgr_->set_backend(default_script_provider_.get());
      RCLCPP_INFO(get_logger(), "Scripts enabled with directory: %s", scripts_dir.c_str());
    } else {
      auto * manifest_mgr = discovery_mgr_->get_manifest_manager();
      if (manifest_mgr) {
        auto manifest_scripts = manifest_mgr->get_scripts();
        if (!manifest_scripts.empty()) {
          RCLCPP_WARN(get_logger(),
                      "Manifest defines %zu script(s) but scripts.scripts_dir is not configured - "
                      "manifest scripts will not be loaded. Set 'scripts.scripts_dir' in gateway parameters to enable.",
                      manifest_scripts.size());
        }
      }
    }
  }

  // --- Trigger infrastructure ---
  resource_change_notifier_ = std::make_unique<ResourceChangeNotifier>([this](const std::string & msg) {
    RCLCPP_WARN(get_logger(), "ResourceChangeNotifier: %s", msg.c_str());
  });

  condition_registry_ = std::make_unique<ConditionRegistry>();
  condition_registry_->register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  condition_registry_->register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
  condition_registry_->register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
  condition_registry_->register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());

  auto triggers_enabled = get_parameter("triggers.enabled").as_bool();
  if (triggers_enabled) {
    auto storage_path = get_parameter("triggers.storage.path").as_string();
    if (storage_path.empty()) {
      trigger_store_ = std::make_unique<SqliteTriggerStore>(":memory:");
    } else {
      trigger_store_ = std::make_unique<SqliteTriggerStore>(storage_path);
    }

    TriggerConfig trigger_config;
    trigger_config.max_triggers = static_cast<int>(get_parameter("triggers.max_triggers").as_int());
    trigger_config.on_restart_behavior = get_parameter("triggers.on_restart_behavior").as_string();

    // Subscribe to data topics for data triggers. The adapter is created
    // before the trigger manager because the manager takes the transport
    // by shared_ptr at construction time.
    trigger_topic_subscriber_ = std::make_unique<TriggerTopicSubscriber>(this);
    trigger_topic_transport_ = std::make_shared<ros2::Ros2TopicSubscriptionTransport>(trigger_topic_subscriber_.get());

    trigger_mgr_ = std::make_unique<TriggerManager>(*resource_change_notifier_, *condition_registry_, *trigger_store_,
                                                    trigger_config, trigger_topic_transport_);

    // Set entity hierarchy resolver using thread-safe cache
    trigger_mgr_->set_entity_children_fn(
        [this](const std::string & entity_id, const std::string & entity_type) -> std::vector<std::string> {
          const auto & cache = get_thread_safe_cache();
          if (entity_type == "components") {
            return cache.get_apps_for_component(entity_id);
          } else if (entity_type == "areas") {
            auto comps = cache.get_components_for_area(entity_id);
            std::vector<std::string> all_apps;
            for (const auto & comp_id : comps) {
              auto apps = cache.get_apps_for_component(comp_id);
              all_apps.insert(all_apps.end(), apps.begin(), apps.end());
            }
            return all_apps;
          } else if (entity_type == "functions") {
            return cache.get_apps_for_function(entity_id);
          }
          return {};
        });

    // Set entity existence checker for orphan sweep
    trigger_mgr_->set_entity_exists_fn(
        [this](const std::string & entity_id, const std::string & /*entity_type*/) -> bool {
          const auto & cache = get_thread_safe_cache();
          return cache.find_entity(entity_id).has_value();
        });

    // Load persistent triggers
    trigger_mgr_->load_persistent_triggers();

    // Wire notifier to managers so they emit events for trigger evaluation
    if (update_mgr_) {
      update_mgr_->set_notifier(resource_change_notifier_.get());
    }
    operation_mgr_->set_notifier(resource_change_notifier_.get());

    // Wire LogManager <-> TriggerManager for log_settings integration
    if (log_mgr_) {
      trigger_mgr_->set_log_manager(log_mgr_.get());
      log_mgr_->set_notifier(resource_change_notifier_.get());
    }

    // Subscribe to fault events and forward to notifier
    trigger_fault_subscriber_ = std::make_unique<TriggerFaultSubscriber>(this, *resource_change_notifier_);

    // Wire deferred topic resolution: when a trigger's resource_path couldn't
    // be resolved to a ROS 2 topic at creation time, retry periodically using
    // the entity cache to find matching topics.
    trigger_mgr_->set_resolve_topic_fn([this](const std::string & entity_id,
                                              const std::string & resource_path) -> std::string {
      const auto & cache = get_thread_safe_cache();
      auto aggregated = cache.get_entity_data(entity_id);
      for (const auto & topic : aggregated.topics) {
        if (topic.name == resource_path ||
            (topic.name.size() > resource_path.size() &&
             topic.name.compare(topic.name.size() - resource_path.size(), resource_path.size(), resource_path) == 0)) {
          return topic.name;
        }
      }
      return "";
    });
    trigger_mgr_->set_warn_log_fn([this](const std::string & message) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    });
    // The deferred-resolution budget must outlive at least one full discovery
    // refresh: plugin-declared topics only reach the entity cache on a refresh
    // pass, and refresh_interval_ms is configurable up to 60 s - equal to the
    // old fixed budget, which made "resolves on a later tick" false at the
    // extreme. Two refresh cycles keeps the reasoning true for every allowed
    // configuration.
    trigger_mgr_->set_unresolved_timeout(
        std::max(std::chrono::seconds(60), std::chrono::seconds(2 * refresh_interval_ms_ / 1000)));
    trigger_topic_subscriber_->set_retry_callback([this]() {
      trigger_mgr_->retry_unresolved_triggers();
    });

    // Wire node-to-entity resolver for trigger notifications (#305).
    // The resolver looks up node FQNs in the ThreadSafeEntityCache's node_to_app
    // mapping (populated from linking result). Captures 'this' - the lambda is
    // called from ROS subscription callbacks on the same node's executor.
    auto node_resolver = [this](const std::string & ros_fqn) -> std::string {
      auto result = thread_safe_cache_.resolve_node_to_app(ros_fqn);
      if (!result.empty()) {
        return result;
      }
      // Try with normalized FQN (strip leading /)
      if (!ros_fqn.empty() && ros_fqn[0] == '/') {
        result = thread_safe_cache_.resolve_node_to_app(ros_fqn.substr(1));
        if (!result.empty()) {
          return result;
        }
      }
      return "";
    };

    if (trigger_fault_subscriber_) {
      trigger_fault_subscriber_->set_node_to_entity_resolver(node_resolver);
    }
    if (log_mgr_) {
      log_mgr_->set_node_to_entity_resolver(node_resolver);
    }

    RCLCPP_INFO(get_logger(), "Trigger subsystem: enabled (max=%d, storage=%s)", trigger_config.max_triggers,
                storage_path.empty() ? ":memory:" : storage_path.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Trigger subsystem: disabled");
  }

  // --- Aggregation (peer gateway federation) ---
  if (get_parameter("aggregation.enabled").as_bool()) {
    AggregationConfig agg_config;
    agg_config.enabled = true;
    agg_config.timeout_ms = static_cast<int>(get_parameter("aggregation.timeout_ms").as_int());
    agg_config.announce = get_parameter("aggregation.announce").as_bool();
    agg_config.discover = get_parameter("aggregation.discover").as_bool();
    agg_config.mdns_service = get_parameter("aggregation.mdns_service").as_string();
    agg_config.forward_auth = get_parameter("aggregation.forward_auth").as_bool();
    agg_config.require_tls = get_parameter("aggregation.require_tls").as_bool();
    agg_config.peer_scheme = get_parameter("aggregation.peer_scheme").as_string();
    if (agg_config.peer_scheme != "http" && agg_config.peer_scheme != "https") {
      RCLCPP_ERROR(get_logger(),
                   "Aggregation: peer_scheme '%s' is invalid (must be 'http' or 'https'). "
                   "Falling back to 'http'.",
                   agg_config.peer_scheme.c_str());
      agg_config.peer_scheme = "http";
    }
    agg_config.max_discovered_peers = static_cast<size_t>(get_parameter("aggregation.max_discovered_peers").as_int());

    // Parse static peers from parallel arrays
    auto peer_urls = get_parameter("aggregation.peer_urls").as_string_array();
    auto peer_names = get_parameter("aggregation.peer_names").as_string_array();
    if (peer_urls.size() != peer_names.size()) {
      RCLCPP_ERROR(get_logger(),
                   "Aggregation: peer_urls has %zu entries but peer_names has %zu. "
                   "These parallel arrays must have the same length. "
                   "All static peers will be ignored until the configuration is fixed.",
                   peer_urls.size(), peer_names.size());
    } else {
      for (size_t i = 0; i < peer_urls.size(); ++i) {
        if (!peer_urls[i].empty() && !peer_names[i].empty()) {
          agg_config.peers.push_back({peer_urls[i], peer_names[i]});
          RCLCPP_INFO(get_logger(), "Aggregation: static peer '%s' at %s", peer_names[i].c_str(), peer_urls[i].c_str());
        }
      }
    }

    if (agg_config.peers.empty() && !agg_config.announce && !agg_config.discover) {
      RCLCPP_WARN(get_logger(),
                  "Aggregation enabled but no static peers and mDNS disabled. "
                  "No peer communication will occur.");
    }

    // Security warning: forwarding auth tokens over cleartext is dangerous
    if (agg_config.forward_auth && !agg_config.require_tls) {
      RCLCPP_WARN(get_logger(),
                  "Aggregation: forward_auth is enabled but require_tls is false. "
                  "Authorization tokens may be sent to peers over cleartext HTTP. "
                  "Set aggregation.require_tls=true for production deployments.");
    }

    auto logger = get_logger();
    aggregation_mgr_ = std::make_unique<AggregationManager>(agg_config, &logger);

    // mDNS discovery/announcement
    if (agg_config.announce || agg_config.discover) {
      MdnsDiscovery::Config mdns_config;
      mdns_config.announce = agg_config.announce;
      mdns_config.discover = agg_config.discover;
      mdns_config.service = agg_config.mdns_service;
      mdns_config.port = server_port_;
      mdns_config.peer_scheme = agg_config.peer_scheme;
      // mDNS instance name must be unique per gateway instance.
      // Defaults to hostname via MdnsDiscovery constructor when empty.
      // Operators should set aggregation.mdns_name for multi-gateway-per-host deployments.
      mdns_config.name = get_parameter("aggregation.mdns_name").as_string();
      mdns_config.on_error = [this](const std::string & msg) {
        RCLCPP_ERROR(get_logger(), "mDNS: %s", msg.c_str());
      };
      mdns_config.on_log = [this](const std::string & msg) {
        RCLCPP_DEBUG(get_logger(), "mDNS: %s", msg.c_str());
      };
      mdns_discovery_ = std::make_unique<MdnsDiscovery>(mdns_config);
      // Get the actual mDNS instance name for self-discovery filtering.
      // MdnsDiscovery constructor resolves empty name to gethostname().
      // Sanitize it the same way browse_callback sanitizes discovered peer names.
      const std::string self_mdns_name = HostInfoProvider::sanitize_entity_id(mdns_discovery_->instance_name());

      // Collect local interface addresses for IP-based self-discovery filtering.
      // Name-only checks are insufficient: an attacker can send mDNS responses with
      // a different name but our own IP:port, creating forwarding loops.
      // The address set is refreshed periodically (every 60s) to pick up network
      // changes (new interfaces, DHCP renewals) without stale data.
      auto local_addrs = std::make_shared<std::unordered_set<std::string>>(collect_local_addresses());
      auto last_addr_refresh =
          std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
      auto addr_mutex = std::make_shared<std::mutex>();
      const int self_port = server_port_;

      mdns_discovery_->start(
          [this, self_mdns_name, local_addrs, last_addr_refresh, addr_mutex, self_port](const std::string & url,
                                                                                        const std::string & name) {
            if (name == self_mdns_name) {
              return;  // Skip self-discovery (name match)
            }

            // Refresh local addresses periodically (every 60s) to handle network changes.
            // collect_local_addresses() is cheap (getifaddrs syscall), safe to call often.
            {
              std::lock_guard<std::mutex> lock(*addr_mutex);
              auto now = std::chrono::steady_clock::now();
              if (std::chrono::duration_cast<std::chrono::seconds>(now - *last_addr_refresh).count() >= 60) {
                *local_addrs = collect_local_addresses();
                *last_addr_refresh = now;
              }
            }

            // Also reject peers whose resolved IP:port matches our own listen address.
            // Prevents forwarding loops from spoofed mDNS responses.
            auto [peer_host, peer_port] = parse_url_host_port(url);
            if (peer_port == self_port && local_addrs->count(peer_host) > 0) {
              RCLCPP_WARN(get_logger(),
                          "mDNS: rejecting peer '%s' at %s - resolves to local address with our port "
                          "(possible spoofed mDNS response)",
                          name.c_str(), url.c_str());
              return;
            }
            aggregation_mgr_->add_discovered_peer(url, name);
          },
          [this](const std::string & name) {
            aggregation_mgr_->remove_discovered_peer(name);
          });
    }

    RCLCPP_INFO(get_logger(),
                "Aggregation: enabled (timeout=%dms, announce=%s, discover=%s, "
                "forward_auth=%s, require_tls=%s, peer_scheme=%s, max_discovered_peers=%zu)",
                agg_config.timeout_ms, agg_config.announce ? "true" : "false", agg_config.discover ? "true" : "false",
                agg_config.forward_auth ? "true" : "false", agg_config.require_tls ? "true" : "false",
                agg_config.peer_scheme.c_str(), agg_config.max_discovered_peers);
  } else {
    RCLCPP_INFO(get_logger(), "Aggregation: disabled");
  }

  // Register built-in resource samplers
  sampler_registry_->register_sampler(
      "data",
      [this](const std::string & /*entity_id*/,
             const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
        auto * dam = get_data_access_manager();
        if (!dam) {
          return tl::make_unexpected(std::string("DataAccessManager not available"));
        }
        auto * provider = dam->get_topic_data_provider();
        if (!provider) {
          return tl::make_unexpected(std::string("TopicDataProvider not configured"));
        }
        const auto timeout_ms = std::chrono::milliseconds{
            static_cast<std::int64_t>(std::max(dam->get_topic_sample_timeout(), 0.0) * 1000.0)};
        auto r = provider->sample(resource_path, timeout_ms);
        if (!r) {
          return tl::make_unexpected(std::string{"Topic sample failed: "} + r.error().message);
        }
        TopicSampleResult sample = *r;
        if (sample.has_data && sample.data.has_value()) {
          nlohmann::json payload;
          payload["id"] = resource_path;
          payload["data"] = *sample.data;
          return payload;
        }
        return tl::make_unexpected("Topic data not available: " + resource_path);
      },
      true);

  sampler_registry_->register_sampler(
      "faults",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * fault_mgr = get_fault_manager();
        if (!fault_mgr) {
          return tl::make_unexpected(std::string("FaultManager not available"));
        }
        const auto & cache = get_thread_safe_cache();
        auto entity_ref = cache.find_entity(entity_id);
        if (!entity_ref) {
          return tl::make_unexpected("Entity not found: " + entity_id);
        }
        // Same scoping as GET /{entity}/faults: shared source resolution
        // (external app -> bare id, external component owns its own id) and
        // the shared boundary-aware filter. A private variant here answered
        // "Entity no longer available" for external apps whose effective_fqn
        // is empty while REST returned the fault.
        auto source_fqns = faults::resolve_entity_source_fqns(cache, entity_ref->type, entity_id);
        auto result = fault_mgr->list_faults("");
        if (!result.success) {
          return tl::make_unexpected(result.error_message);
        }
        auto filtered = faults::filter_faults_by_sources(result.data["faults"], source_fqns);
        result.data["count"] = filtered.size();
        result.data["faults"] = std::move(filtered);
        return result.data;
      },
      true);

  sampler_registry_->register_sampler(
      "configurations",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * config_mgr = get_configuration_manager();
        if (!config_mgr) {
          return tl::make_unexpected(std::string("ConfigurationManager not available"));
        }
        const auto & cache = get_thread_safe_cache();
        auto entity_configs = cache.get_entity_configurations(entity_id);
        nlohmann::json items = nlohmann::json::array();
        for (const auto & node : entity_configs.nodes) {
          auto result = config_mgr->list_parameters(node.node_fqn);
          if (result.success && result.data.is_array()) {
            for (auto & param : result.data) {
              items.push_back(std::move(param));
            }
          }
        }
        nlohmann::json payload;
        payload["items"] = std::move(items);
        return payload;
      },
      true);

  sampler_registry_->register_sampler(
      "logs",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * log_mgr = get_log_manager();
        if (!log_mgr) {
          return tl::make_unexpected(std::string("LogManager not available"));
        }
        const auto & cache = get_thread_safe_cache();
        auto entity_ref = cache.find_entity(entity_id);
        if (!entity_ref) {
          return tl::make_unexpected("Entity not found: " + entity_id);
        }
        // Same scoping as GET /{entity}/logs via the shared helper (external
        // app -> bare id, external component owns its own id, plus the
        // namespace-prefix merge for COMPONENT / AREA).
        std::string entity_fqn;
        if (entity_ref->type == SovdEntityType::COMPONENT) {
          if (auto comp = cache.get_component(entity_id)) {
            entity_fqn = comp->fqn;
          }
        } else if (entity_ref->type == SovdEntityType::AREA) {
          if (auto area = cache.get_area(entity_id)) {
            entity_fqn = area->namespace_path;
          }
        } else if (entity_ref->type == SovdEntityType::APP) {
          if (auto app = cache.get_app(entity_id)) {
            entity_fqn = app->effective_fqn();
          }
        }
        auto result = logs::query_entity_logs(*log_mgr, cache, entity_ref->type, entity_id, entity_fqn, "", "");
        if (!result.has_value()) {
          return tl::make_unexpected(result.error());
        }
        nlohmann::json payload;
        payload["items"] = std::move(*result);
        return payload;
      },
      true);

  // Register update status sampler (server-level, uses UpdateManager)
  if (update_mgr_) {
    sampler_registry_->register_sampler(
        "updates",
        [this](const std::string & /*entity_id*/,
               const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
          auto * mgr = get_update_manager();
          if (!mgr || !mgr->has_backend()) {
            return tl::make_unexpected(std::string("Update backend not available"));
          }
          auto result = mgr->get_status(resource_path);
          if (!result) {
            return tl::make_unexpected(result.error().message);
          }
          return update_status_to_json(*result);
        },
        true);
  }

  RCLCPP_INFO(get_logger(), "Registered built-in resource samplers: data, faults, configurations, logs%s",
              update_mgr_ ? ", updates" : "");

  // Register built-in SSE transport
  transport_registry_->register_transport(
      std::make_unique<SseTransportProvider>(*subscription_mgr_, sse_client_tracker_));

  // Wire subscription lifecycle -> transport cleanup
  subscription_mgr_->set_on_removed([this](const CyclicSubscriptionInfo & info) {
    if (auto * transport = transport_registry_->get_transport(info.protocol)) {
      transport->stop(info.id);
    }
  });

  // Connect topic sampler to discovery manager for component-topic mapping
  discovery_mgr_->set_topic_data_provider(data_access_mgr_->get_topic_data_provider());

  // Connect type introspection for operation schema enrichment
  discovery_mgr_->set_type_introspection(data_access_mgr_->get_type_introspection());

  // Initial discovery
  refresh_cache();

  // Primary refresh trigger: ROS 2 graph events.
  //
  // `get_graph_event()` returns a shared rclcpp::Event the executor signals
  // whenever the graph changes (node up/down, topic/service/action add or
  // remove). We poll `check_and_clear()` every 100 ms and only run the
  // (relatively expensive) `refresh_cache()` pipeline when the event has
  // fired. On a stable graph this keeps idle CPU near zero.
  //
  // 100 ms is a deliberate balance: spawn-detection latency stays low
  // (graph-event arrival + at most one tick), while the polling overhead is
  // a single atomic check per tick.
  graph_event_ = this->get_graph_event();
  graph_check_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    if (!graph_event_) {
      return;
    }
    // Coalesce graph events: under graph churn (nodes/topics created and
    // destroyed repeatedly) the event fires many times per second, and each
    // refresh_cache() runs the full discovery pipeline. Debounce so we run at
    // most one refresh per `refresh_debounce_ms` (default 1000 ms) - the ROS
    // graph rarely changes faster than that in practice, and coalescing
    // bounds the per-event allocation churn (issue #442). A pending event is
    // always serviced within refresh_debounce_ms + one 100 ms tick.
    const bool event_fired = graph_event_->check_and_clear();
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_graph_refresh_.load(std::memory_order_relaxed))
            .count();
    const auto decision = decide_graph_refresh(event_fired, graph_dirty_.load(std::memory_order_relaxed), elapsed_ms,
                                               refresh_debounce_ms_);
    graph_dirty_.store(decision.dirty, std::memory_order_relaxed);
    if (!decision.refresh) {
      return;  // nothing pending, or still within the debounce window
    }
    last_graph_refresh_.store(now, std::memory_order_relaxed);
    refresh_cache();
    // Note: orphan-trigger sweep is deliberately NOT run from the graph-event
    // path. Graph events fire on every node up/down at high frequency,
    // including the cold-start window where DDS discovery has only seen a
    // partial node set. Running the sweep there would treat
    // not-yet-discovered entities as orphans and incorrectly remove
    // restored persistent triggers. The backstop timer below runs at the
    // configured (slower) refresh cadence, giving DDS time to converge
    // before any orphan check.
  });

  // Safety backstop: unconditional refresh at the configured (low) cadence.
  // Guarantees liveness if a graph event is ever missed for any reason
  // (lost wakeup, rclcpp anomaly, etc.). Default 30 s is rare enough to be
  // negligible on idle CPU yet bounded enough to recover quickly. This is
  // also the only place we sweep orphan triggers, because by this cadence
  // the entity cache is guaranteed to reflect a settled DDS view.
  backstop_timer_ = create_wall_timer(std::chrono::milliseconds(refresh_interval_ms_), [this]() {
    refresh_cache();

    // Sweep triggers whose entities disappeared from discovery
    if (trigger_mgr_) {
      trigger_mgr_->sweep_orphaned_triggers();
    }
  });

  // Setup periodic cleanup of old action goals (every 60 seconds, remove goals older than 5 minutes)
  cleanup_timer_ = create_wall_timer(60s, [this]() {
    operation_mgr_->cleanup_old_goals(std::chrono::seconds(300));
  });

  // Setup periodic cleanup of expired cyclic subscriptions (every 30 seconds)
  subscription_cleanup_timer_ = create_wall_timer(30s, [this]() {
    size_t removed = subscription_mgr_->cleanup_expired();
    if (removed > 0) {
      RCLCPP_DEBUG(get_logger(), "Cleaned up %zu expired cyclic subscriptions", removed);
    }
  });

  // Start REST server with configured host, port, CORS, auth, and TLS
  rest_server_ = std::make_unique<RESTServer>(this, server_host_, server_port_, cors_config_, auth_config_,
                                              rate_limit_config_, tls_config_);

  // Wire trigger handlers into REST server
  if (trigger_mgr_) {
    rest_server_->set_trigger_handlers(*trigger_mgr_);
  }

  // Wire aggregation manager into REST server handler context
  if (aggregation_mgr_) {
    rest_server_->set_aggregation_manager(aggregation_mgr_.get());
  }

  start_rest_server();

  std::string protocol = tls_config_.enabled ? "HTTPS" : "HTTP";
  RCLCPP_INFO(get_logger(), "ROS 2 Medkit Gateway ready on %s://%s:%d", protocol.c_str(), server_host_.c_str(),
              server_port_);

  // One-shot discovery summary a few seconds after startup. The delay lets DDS
  // discovery settle so the counts (and any empty-graph warning) reflect a
  // converged view rather than the cold-start moment.
  startup_summary_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    startup_summary_timer_->cancel();
    startup_summary_timer_.reset();
    log_startup_summary();
  });
}

size_t GatewayNode::count_peer_nodes(const std::vector<std::pair<std::string, std::string>> & nodes_and_namespaces,
                                     const std::string & self_fqn) {
  size_t count = 0;
  for (const auto & [name, ns] : nodes_and_namespaces) {
    if (!name.empty() && name.front() == '_') {
      continue;  // hidden node (name part starts with '_')
    }
    std::string fqn = ns;
    if (ns != "/") {
      fqn += "/";
    }
    fqn += name;
    // Exclude the gateway's own nodes by exact FQN: the main node and its known
    // internal helpers. A plain prefix match would also drop a genuine peer whose
    // name starts with the gateway name (e.g. "<fqn>_monitor" or "<fqn>2").
    if (fqn == self_fqn || fqn == self_fqn + "_sub" || fqn == self_fqn + "_fault_clients") {
      continue;
    }
    ++count;
  }
  return count;
}

std::string GatewayNode::connectable_host(const std::string & bind_host) {
  // A bind-all address is not a valid connect target, so the sample curl must use
  // a loopback address the operator can actually reach.
  if (bind_host.empty() || bind_host == "0.0.0.0") {
    return "127.0.0.1";
  }
  if (bind_host == "::" || bind_host == "[::]") {
    return "[::1]";
  }
  // Bracket a bare IPv6 literal so "host:port" is a valid URL authority for curl.
  if (bind_host.front() != '[' && bind_host.find(':') != std::string::npos) {
    return "[" + bind_host + "]";
  }
  return bind_host;
}

void GatewayNode::log_startup_summary() {
  size_t topic_count = 0;
  size_t peer_node_count = 0;
  try {
    for (const auto & entry : get_topic_names_and_types()) {
      const std::string & topic = entry.first;
      if (topic.find("/parameter_events") == std::string::npos && topic.find("/rosout") == std::string::npos) {
        ++topic_count;
      }
    }
    peer_node_count =
        count_peer_nodes(get_node_graph_interface()->get_node_names_and_namespaces(), get_fully_qualified_name());
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(get_logger(), "Startup summary: graph query failed: %s", e.what());
  }

  const size_t entity_count = thread_safe_cache_.get_areas().size() + thread_safe_cache_.get_components().size() +
                              thread_safe_cache_.get_apps().size() + thread_safe_cache_.get_functions().size();

  const std::string protocol = tls_config_.enabled ? "https" : "http";
  const std::string url = protocol + "://" + connectable_host(server_host_) + ":" + std::to_string(server_port_);

  RCLCPP_INFO(get_logger(),
              "Discovery summary: %zu peer node(s), %zu topic(s), %zu medkit entit%s. REST API: %s  Try: curl "
              "%s/api/v1/areas",
              peer_node_count, topic_count, entity_count, entity_count == 1 ? "y" : "ies", url.c_str(), url.c_str());

  if (peer_node_count == 0) {
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    const char * rmw = std::getenv("RMW_IMPLEMENTATION");
    const char * localhost_only = std::getenv("ROS_LOCALHOST_ONLY");
    RCLCPP_WARN(get_logger(),
                "No application nodes are visible on the ROS graph - the gateway sees only its own nodes. "
                "Active ROS environment: ROS_DOMAIN_ID=%s, RMW_IMPLEMENTATION=%s, ROS_LOCALHOST_ONLY=%s. Check that "
                "the gateway shares the robot stack's ROS_DOMAIN_ID and RMW_IMPLEMENTATION and can reach it on the "
                "network (in containers: matching domain/RMW and DDS discovery not blocked).",
                domain ? domain : "0 (default)", rmw ? rmw : "(default)", localhost_only ? localhost_only : "(unset)");
  }
}

GatewayNode::~GatewayNode() {
  RCLCPP_INFO(get_logger(), "Shutting down ROS 2 Medkit Gateway...");
  // 0. Stop mDNS discovery first (prevents new peer additions during shutdown)
  if (mdns_discovery_) {
    mdns_discovery_->stop();
  }
  // 0b. Cancel in-flight peer HTTP calls so REST worker threads that are
  //     mid-forward unblock promptly. Without this, stop_rest_server() below
  //     waits for those workers to finish their full peer read timeout,
  //     which under TSan / heavy load exceeds the launch_test SIGINT grace
  //     window and forces a SIGKILL with non-zero exit. The handlers still
  //     hold references to aggregation_mgr_, so we do not destroy it here
  //     - only cancel its outbound calls.
  if (aggregation_mgr_) {
    aggregation_mgr_->shutdown();
  }
  // 1. Stop REST server (kills HTTP connections, SSE streams exit)
  //    Handlers may reference aggregation_mgr_, so it must outlive the server.
  stop_rest_server();
  // 2. Shutdown subscriptions via transport registry (calls sub_mgr.shutdown(),
  //    which triggers on_removed -> transport->stop() for each active subscription)
  if (transport_registry_) {
    transport_registry_->shutdown_all(*subscription_mgr_);
  }
  // 3. Shutdown trigger topic subscriber (stops producing data notifications)
  if (trigger_topic_subscriber_) {
    trigger_topic_subscriber_->shutdown();
  }
  // 4. Shutdown trigger subsystem (wakes SSE trigger streams)
  if (trigger_mgr_) {
    trigger_mgr_->shutdown();
  }
  // 5. Shutdown resource change notifier (stops worker thread)
  if (resource_change_notifier_) {
    resource_change_notifier_->shutdown();
  }
  // 5b. Stop the fault-trigger evaluation loop BEFORE plugin/fault shutdown:
  //     it resolves into plugin routes and the FaultManager, neither of which
  //     may be used once shutdown_all() has run.
  fault_trigger_stop_.store(true, std::memory_order_relaxed);
  if (fault_trigger_thread_.joinable()) {
    fault_trigger_thread_.join();
  }
  fault_trigger_engine_.reset();
  // 6. Drop the entity freeze-frame capture BEFORE plugin shutdown: its
  //    capture thread (joined here) resolves into plugin DataProviders and
  //    routes, which must not be reachable once shutdown_all() has run.
  entity_freeze_frame_capture_.reset();
  //    Shutdown plugins
  if (plugin_mgr_) {
    plugin_mgr_->shutdown_all();
  }
  // 7. Shutdown OperationManager (clears action status subscriptions and
  //    service clients while executor may still be spinning - prevents
  //    "terminate called without an active exception" during member destruction)
  if (operation_mgr_) {
    operation_mgr_->shutdown();
  }
  // 8. Shutdown ConfigurationManager (clears param_node_ and cached clients
  //    before rclcpp context is destroyed - prevents use-after-free)
  if (config_mgr_) {
    config_mgr_->shutdown();
  }
  // 9. Cancel the graph timers and drop our graph_event_ reference before
  //    any other member destruction begins. rclcpp's graph listener thread
  //    holds shared_ptrs to all registered graph events (graph_events_ in
  //    NodeGraph) and may concurrently iterate them via notify_graph_change()
  //    while we're tearing down. TSan flagged a Write/Read race on the
  //    Event's control block between that thread and our shared_ptr release
  //    during automatic member destruction (see TestGatewayNode TSan run).
  //    Doing the reset here gives the timer a chance to drain before its
  //    [this]-captured graph_event_ disappears, and shrinks the window where
  //    the graph listener can still see a live shared_ptr from us.
  if (graph_check_timer_) {
    graph_check_timer_->cancel();
    graph_check_timer_.reset();
  }
  if (backstop_timer_) {
    backstop_timer_->cancel();
    backstop_timer_.reset();
  }
  if (startup_summary_timer_) {
    startup_summary_timer_->cancel();
    startup_summary_timer_.reset();
  }
  graph_event_.reset();
  // 10. Normal member destruction (managers safe - all transports stopped)
}

const ThreadSafeEntityCache & GatewayNode::get_thread_safe_cache() const {
  return thread_safe_cache_;
}

DataAccessManager * GatewayNode::get_data_access_manager() const {
  return data_access_mgr_.get();
}

void GatewayNode::set_topic_data_provider(std::shared_ptr<TopicDataProvider> provider) {
  topic_data_provider_ = std::move(provider);
  if (data_access_mgr_) {
    data_access_mgr_->set_topic_data_provider(topic_data_provider_.get());
  }
  if (topic_transport_) {
    topic_transport_->set_data_provider(topic_data_provider_.get());
  }
  if (discovery_mgr_) {
    discovery_mgr_->set_topic_data_provider(topic_data_provider_.get());
  }
  if (topic_data_provider_) {
    RCLCPP_INFO(get_logger(), "TopicDataProvider attached (race-free subscription pool active)");
  } else {
    RCLCPP_INFO(get_logger(), "TopicDataProvider detached");
  }
}

OperationManager * GatewayNode::get_operation_manager() const {
  return operation_mgr_.get();
}

DiscoveryManager * GatewayNode::get_discovery_manager() const {
  return discovery_mgr_.get();
}

ConfigurationManager * GatewayNode::get_configuration_manager() const {
  return config_mgr_.get();
}

FaultManager * GatewayNode::get_fault_manager() const {
  return fault_mgr_.get();
}

LogManager * GatewayNode::get_log_manager() const {
  return log_mgr_.get();
}

BulkDataStore * GatewayNode::get_bulk_data_store() const {
  return bulk_data_store_.get();
}

SubscriptionManager * GatewayNode::get_subscription_manager() const {
  return subscription_mgr_.get();
}

UpdateManager * GatewayNode::get_update_manager() const {
  return update_mgr_.get();
}

LockManager * GatewayNode::get_lock_manager() const {
  return lock_manager_.get();
}

ScriptManager * GatewayNode::get_script_manager() const {
  return script_mgr_.get();
}

PluginManager * GatewayNode::get_plugin_manager() const {
  return plugin_mgr_.get();
}

void GatewayNode::init_entity_freeze_frame_capture(ros2_common::Ros2SubscriptionExecutor & exec) {
  // Snapshot the faulting entity's current data values when its fault confirms.
  // Only useful when a plugin can own entities, so gated on loaded plugins.
  if (!get_parameter("entity_freeze_frame.enabled").as_bool() || !plugin_mgr_ || !plugin_mgr_->has_plugins()) {
    return;
  }
  entity_freeze_frame_capture_ = std::make_unique<EntityFreezeFrameCapture>(
      this, exec,
      [this](const std::string & entity_id) {
        return plugin_mgr_ ? plugin_mgr_->get_data_provider_for_entity(entity_id) : nullptr;
      },
      // Plugins without a DataProvider (the commercial PLC bridges) serve live
      // values only through their own x-plc-data route - dispatch it in-process
      // as the capture fallback.
      [this](const std::string & entity_id) -> std::optional<nlohmann::json> {
        return plugin_mgr_ ? plugin_mgr_->fetch_entity_data_via_route(entity_id) : std::nullopt;
      },
      256,  // retained-frame bound (constructor default)
      // Plugins start polling - and reporting - while this object is still
      // being wired, and a device that is in fault when the box boots confirms
      // immediately, to nobody. The capture asks for those on its own thread,
      // which it joins on destruction; should_abort lets that join interrupt
      // the service wait.
      [this](const std::function<bool()> & should_abort) {
        std::vector<EntityFreezeFrameCapture::StandingFault> standing;
        if (!fault_service_transport_) {
          RCLCPP_WARN(get_logger(), "Standing-fault freeze-frame catch-up skipped: fault transport not initialised");
          return standing;
        }
        // One deadline for the whole wait, polled in slices so shutdown can
        // abort it (wait_for_services itself shares the slice across clients).
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        bool ready = false;
        while (!ready && !should_abort()) {
          const std::chrono::duration<double> remaining = deadline - std::chrono::steady_clock::now();
          if (remaining <= std::chrono::duration<double>::zero()) {
            break;
          }
          ready = fault_service_transport_->wait_for_services(std::min(std::chrono::duration<double>(0.5), remaining));
        }
        if (!ready) {
          if (!should_abort()) {
            RCLCPP_WARN(get_logger(),
                        "Standing-fault freeze-frame catch-up skipped: fault services not available within 10s");
          }
          return standing;
        }
        auto result = fault_service_transport_->list_faults("", false, true, false, false, false, false);
        if (!result.success) {
          RCLCPP_WARN(get_logger(), "Standing-fault freeze-frame catch-up skipped: ListFaults failed: %s",
                      result.error_message.c_str());
          return standing;
        }
        auto parsed = EntityFreezeFrameCapture::standing_faults_from_list_reply(result.data);
        if (!parsed) {
          RCLCPP_WARN(get_logger(), "Standing-fault freeze-frame catch-up skipped: malformed ListFaults reply");
          return standing;
        }
        if (parsed->empty()) {
          RCLCPP_INFO(get_logger(), "Standing-fault freeze-frame catch-up: no confirmed faults at startup");
        }
        return std::move(*parsed);
      });
}

EntityFreezeFrameCapture * GatewayNode::get_entity_freeze_frame_capture() const {
  return entity_freeze_frame_capture_.get();
}

void GatewayNode::set_trigger_subscription_executor(ros2_common::Ros2SubscriptionExecutor & exec) {
  if (trigger_topic_subscriber_) {
    trigger_topic_subscriber_->set_subscription_executor(&exec);
  }
}

void GatewayNode::shutdown_trigger_subscriber() {
  if (trigger_topic_subscriber_) {
    trigger_topic_subscriber_->shutdown();
  }
}

namespace {

// Coerce a discovered data point's JSON value to a double for threshold
// comparison. Handles the numeric, boolean and numeric-string shapes the PLC
// bridges emit; returns nullopt for anything non-numeric.
std::optional<double> value_to_double(const nlohmann::json & v) {
  if (v.is_number()) {
    return v.get<double>();
  }
  if (v.is_boolean()) {
    return v.get<bool>() ? 1.0 : 0.0;
  }
  if (v.is_string()) {
    try {
      size_t pos = 0;
      const double d = std::stod(v.get<std::string>(), &pos);
      if (pos > 0) {
        return d;
      }
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

}  // namespace

void GatewayNode::init_fault_trigger_engine() {
  if (!get_parameter("fault_triggers.enabled").as_bool() || !plugin_mgr_ || !plugin_mgr_->has_plugins()) {
    return;
  }
  const std::string storage_path = get_parameter("fault_triggers.storage.path").as_string();
  int poll_ms = static_cast<int>(get_parameter("fault_triggers.poll_interval_ms").as_int());
  if (poll_ms < 50) {
    poll_ms = 50;  // floor: never busy-spin the evaluation loop
  }

  // Value fetch: same in-process x-plc-data route the freeze-frame capture uses,
  // so a rule always sees exactly the value the /data endpoint would serve.
  auto fetcher = [this](const std::string & app_id, const std::string & data_name) -> std::optional<double> {
    if (!plugin_mgr_) {
      return std::nullopt;
    }
    const auto content = plugin_mgr_->fetch_entity_data_via_route(app_id);
    // A down link serves frozen last-known values; nullopt holds rule state
    // (fault_trigger_engine) instead of firing on a stale number all outage.
    if (!content || !EntityFreezeFrameCapture::content_has_live_data(*content) ||
        EntityFreezeFrameCapture::content_reports_disconnected(*content)) {
      return std::nullopt;
    }
    const auto values = EntityFreezeFrameCapture::values_from_list_content(*content);
    if (!values.is_object() || !values.contains(data_name)) {
      return std::nullopt;
    }
    return value_to_double(values[data_name]);
  };

  auto report = [this](const std::string & app_id, const std::string & fault_code, const std::string & severity,
                       const std::string & description) {
    if (!fault_mgr_) {
      return;
    }
    uint8_t sev = 1;  // WARNING default
    if (severity == "INFO") {
      sev = 0;
    } else if (severity == "WARNING") {
      sev = 1;
    } else if (severity == "ERROR") {
      sev = 2;
    } else if (severity == "CRITICAL") {
      sev = 3;
    }
    fault_mgr_->report_fault(fault_code, sev, description, app_id);
  };

  auto clear = [this](const std::string & /*app_id*/, const std::string & fault_code) {
    if (fault_mgr_) {
      // A trigger-rule auto-clear (falling edge / rule deletion) is a local,
      // rule-scoped event: skip the correlation engine's cascade so clearing
      // one rule's fault can never wipe correlated faults owned by anything
      // else. fault_code is globally unique (create() rejects duplicates), so
      // the app scope is already encoded in the code itself.
      fault_mgr_->clear_fault(fault_code, /*skip_correlation_auto_clear=*/true);
    }
  };

  // Data-point enumeration for create-time validation: same in-process route as
  // the fetcher, so "exists" means exactly "the fetcher could ever read it".
  auto data_point_names = [this](const std::string & app_id) -> std::optional<std::vector<std::string>> {
    if (!plugin_mgr_) {
      return std::nullopt;
    }
    const auto content = plugin_mgr_->fetch_entity_data_via_route(app_id);
    if (!content || !EntityFreezeFrameCapture::content_has_live_data(*content)) {
      return std::nullopt;
    }
    const auto values = EntityFreezeFrameCapture::values_from_list_content(*content);
    if (!values.is_object()) {
      return std::nullopt;
    }
    std::vector<std::string> names;
    names.reserve(values.size());
    for (auto it = values.begin(); it != values.end(); ++it) {
      names.push_back(it.key());
    }
    return names;
  };

  // Entity existence for create-time validation: an app is real iff the entity
  // registry knows its id. Distinct from the data route (which returns nullopt
  // for both an unknown app and a transient outage), so create() can 404 a
  // bogus app instead of reserving a global fault_code for it.
  auto entity_exists = [this](const std::string & app_id) -> bool {
    return get_thread_safe_cache().find_entity(app_id).has_value();
  };

  fault_trigger_engine_ = std::make_unique<FaultTriggerEngine>(
      storage_path, std::move(fetcher), std::move(report), std::move(clear),
      [this](const std::string & m) {
        RCLCPP_INFO(get_logger(), "%s", m.c_str());
      },
      std::move(data_point_names), std::move(entity_exists));

  // Dedicated evaluation loop: the value fetch + synchronous ReportFault call
  // must not run on the main ROS executor (mirrors the freeze-frame capture's
  // own thread). Joined in ~GatewayNode before plugin/fault shutdown.
  const auto interval = std::chrono::milliseconds(poll_ms);
  fault_trigger_thread_ = std::thread([this, interval]() {
    while (!fault_trigger_stop_.load(std::memory_order_relaxed)) {
      try {
        fault_trigger_engine_->evaluate_once();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "fault-trigger evaluation error: %s", e.what());
      }
      // Sleep in short slices so shutdown does not wait a whole interval.
      auto slept = std::chrono::milliseconds(0);
      while (slept < interval && !fault_trigger_stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        slept += std::chrono::milliseconds(25);
      }
    }
  });
  RCLCPP_INFO(get_logger(), "fault-trigger engine active (poll %d ms, store '%s')", poll_ms,
              storage_path.empty() ? "in-memory" : storage_path.c_str());
}

FaultTriggerEngine * GatewayNode::get_fault_trigger_engine() const {
  return fault_trigger_engine_.get();
}

ResourceSamplerRegistry * GatewayNode::get_sampler_registry() const {
  return sampler_registry_.get();
}

TransportRegistry * GatewayNode::get_transport_registry() const {
  return transport_registry_.get();
}

std::shared_ptr<SSEClientTracker> GatewayNode::get_sse_client_tracker() const {
  return sse_client_tracker_;
}

ResourceChangeNotifier * GatewayNode::get_resource_change_notifier() const {
  return resource_change_notifier_.get();
}

TriggerManager * GatewayNode::get_trigger_manager() const {
  return trigger_mgr_.get();
}

ConditionRegistry * GatewayNode::get_condition_registry() const {
  return condition_registry_.get();
}

AggregationManager * GatewayNode::get_aggregation_manager() const {
  return aggregation_mgr_.get();
}

namespace {
// Per-thread flag set while the refresh pipeline is running on this thread.
// Used to short-circuit notify_entities_changed when a plugin calls it from
// within its own IntrospectionProvider::introspect() callback, which
// otherwise would recurse (notify -> reload_manifest -> refresh_cache ->
// introspect -> notify -> ...) until stack exhaustion.
thread_local bool s_in_entity_change_refresh_pass = false;

struct InRefreshPassGuard {
  bool owned{false};
  InRefreshPassGuard() {
    if (!s_in_entity_change_refresh_pass) {
      s_in_entity_change_refresh_pass = true;
      owned = true;
    }
  }
  ~InRefreshPassGuard() {
    if (owned) {
      s_in_entity_change_refresh_pass = false;
    }
  }
  InRefreshPassGuard(const InRefreshPassGuard &) = delete;
  InRefreshPassGuard & operator=(const InRefreshPassGuard &) = delete;
  InRefreshPassGuard(InRefreshPassGuard &&) = delete;
  InRefreshPassGuard & operator=(InRefreshPassGuard &&) = delete;
};
}  // namespace

void GatewayNode::handle_entity_change_notification(const EntityChangeScope & scope) {
  // Reentrancy guard: a plugin may invoke notify_entities_changed from
  // inside its own `IntrospectionProvider::introspect()` callback, which
  // `refresh_cache()` runs for every registered plugin. Because
  // `refresh_mutex_` is recursive, the lock itself would happily re-enter,
  // leading to an unbounded notify/reload/refresh/introspect loop. Detect
  // that we are already inside a refresh pass on the same thread and skip
  // the nested request with a warning - the outer pass will cover the
  // surface change anyway.
  if (s_in_entity_change_refresh_pass) {
    RCLCPP_WARN(get_logger(),
                "Nested notify_entities_changed from within refresh pass - skipping to prevent recursion "
                "(scope=%s). Do not call PluginContext::notify_entities_changed from "
                "IntrospectionProvider::introspect() or other provider callbacks.",
                scope.is_full_refresh() ? "full_refresh" : "scoped");
    return;
  }

  // Take the recursive refresh mutex for the whole notification so that
  // `reload_manifest()` and the subsequent `refresh_cache()` (which takes
  // the same mutex re-entrantly) see a consistent discovery state even when
  // several plugins notify concurrently or a notification overlaps with the
  // periodic refresh timer. ThreadSafeEntityCache's own mutex is not
  // sufficient because the refresh pipeline reaches beyond the cache.
  std::lock_guard<std::recursive_mutex> refresh_lock(refresh_mutex_);

  // Mark the thread as "inside a refresh pass" for the duration of the
  // notification. refresh_cache() below will find the flag already set and
  // leave it alone; plugin introspect callbacks invoked from refresh_cache
  // see the flag and short-circuit any nested notify.
  InRefreshPassGuard pass_guard;

  if (scope.is_full_refresh()) {
    RCLCPP_INFO(get_logger(), "Plugin entity-change notification: full refresh");
  } else {
    RCLCPP_INFO(get_logger(), "Plugin entity-change notification: area=%s component=%s",
                scope.area_id.value_or("<any>").c_str(), scope.component_id.value_or("<any>").c_str());
  }
  // Scope hints are informational in v7. Future versions may restrict the
  // refresh to the named area / component subtree when the discovery
  // pipeline gains that capability. For now every notification triggers a
  // single full pass, which is idempotent and cheap relative to the network
  // side-effects the caller typically just performed.
  //
  // The pass has to re-parse the manifest source (base + fragments dir)
  // BEFORE running the cache refresh, otherwise fragments added or removed
  // since the last load stay invisible to the pipeline.
  if (discovery_mgr_) {
    if (auto * manifest = discovery_mgr_->get_manifest_manager()) {
      if (!manifest->get_manifest_path().empty()) {
        // reload_manifest returns false and restores the previous in-memory
        // manifest when the base file or a fragment fails validation. Log
        // it at WARN so the operator sees "notify requested but my fragment
        // was rejected" instead of a silent no-op. The plugin itself can
        // also inspect `ManifestManager::get_validation_result()` for the
        // specific error (covered by
        // InvalidFragmentOnNotifyLeavesErrorVisibleToPlugin).
        if (!manifest->reload_manifest()) {
          RCLCPP_WARN(get_logger(),
                      "notify_entities_changed: manifest reload failed (see validation errors); "
                      "continuing refresh against the previously loaded manifest");
        }
      }
    }
  }
  refresh_cache();
}

void GatewayNode::trigger_reentrant_notification_for_testing(const EntityChangeScope & scope) {
  // Simulate the "plugin calls notify from inside its introspect callback"
  // scenario without needing a full plugin harness: force the thread-local
  // in-refresh flag on, invoke the normal handler, and reset the flag when
  // done. handle_entity_change_notification should observe the flag and
  // short-circuit with a warning, without reloading the manifest.
  s_in_entity_change_refresh_pass = true;
  struct ResetGuard {
    ResetGuard() = default;
    ~ResetGuard() {
      s_in_entity_change_refresh_pass = false;
    }
    ResetGuard(const ResetGuard &) = delete;
    ResetGuard & operator=(const ResetGuard &) = delete;
    ResetGuard(ResetGuard &&) = delete;
    ResetGuard & operator=(ResetGuard &&) = delete;
  } reset_guard;
  handle_entity_change_notification(scope);
}

void GatewayNode::stop_discovery_refresh_for_testing() {
  // Cancel the refresh drivers and drop the graph event so no further
  // refresh_cache() pass can clobber a test-injected entity cache. Safe to
  // call right after construction (the constructor already ran one refresh).
  //
  // Mirror the destructor's teardown order: cancel AND reset all three timers
  // before releasing graph_event_. The racing party is rclcpp's graph-listener
  // thread (started by get_graph_event() in the constructor, independent of the
  // executor), which may concurrently iterate the registered graph events.
  // Resetting the timers first drops their [this]-captured graph_event_ copies,
  // so the reset below shrinks the window on the Event control block exactly as
  // ~GatewayNode does.
  if (graph_check_timer_) {
    graph_check_timer_->cancel();
    graph_check_timer_.reset();
  }
  if (backstop_timer_) {
    backstop_timer_->cancel();
    backstop_timer_.reset();
  }
  if (startup_summary_timer_) {
    startup_summary_timer_->cancel();
    startup_summary_timer_.reset();
  }
  graph_event_.reset();
}

void GatewayNode::refresh_cache() {
  // Serialize refresh passes across the refresh timer, plugin
  // `notify_entities_changed` notifications, and any other caller. The
  // discovery pipeline cached inside DiscoveryManager is not thread-safe on
  // its own - holding this lock for the full pass is cheap compared with
  // the network I/O the caller typically just performed.
  std::lock_guard<std::recursive_mutex> refresh_lock(refresh_mutex_);

  // Mark the thread as "inside a refresh pass" so that a plugin calling
  // PluginContext::notify_entities_changed from within its introspect
  // callback is detected and skipped rather than recursing back into
  // refresh_cache. If the flag is already set (we were invoked from
  // handle_entity_change_notification) this is a no-op; the outer frame
  // owns the lifetime.
  InRefreshPassGuard pass_guard;

  RCLCPP_DEBUG(get_logger(), "Refreshing entity cache...");

  try {
    // Refresh topic map first (rebuilds the cached map, triggers pipeline in hybrid mode)
    discovery_mgr_->refresh_topic_map();

    // Discover entities - in HYBRID mode the pipeline merges all sources,
    // in RUNTIME_ONLY mode we manually merge node + topic components
    auto areas = discovery_mgr_->discover_areas();
    auto apps = discovery_mgr_->discover_apps();
    auto functions = discovery_mgr_->discover_functions(apps);

    // In RUNTIME_ONLY mode: HostInfoProvider component or empty.
    // In HYBRID/MANIFEST_ONLY: pipeline-merged or manifest components.
    auto all_components = discovery_mgr_->discover_components();

    // Link Apps to default Component (is-located-on relationship)
    // In RUNTIME_ONLY mode with host info provider, ALL apps belong to
    // the single host-derived Component. Override any namespace-derived
    // component_id since those synthetic components no longer exist.
    // In MANIFEST_ONLY and HYBRID modes, apps keep their manifest-assigned
    // component_id (is_located_on relationship).
    if (discovery_mgr_->get_mode() == DiscoveryMode::RUNTIME_ONLY && discovery_mgr_->has_host_info_provider()) {
      auto default_comp = discovery_mgr_->get_default_component();
      if (default_comp) {
        for (auto & app : apps) {
          app.component_id = default_comp->id;
        }
      }
    }

    // Merge remote peer entities if aggregation is active.
    // Keep the routing table accessible for the internal-node filter below.
    std::unordered_map<std::string, std::string> peer_routing_table;
    if (aggregation_mgr_ && aggregation_mgr_->peer_count() > 0) {
      aggregation_mgr_->check_all_health();
      auto logger = get_logger();
      auto merged =
          aggregation_mgr_->fetch_and_merge_peer_entities(areas, all_components, apps, functions, 10000, &logger);
      areas = std::move(merged.areas);
      all_components = std::move(merged.components);
      apps = std::move(merged.apps);
      functions = std::move(merged.functions);
      peer_routing_table = std::move(merged.routing_table);
      aggregation_mgr_->update_routing_table(peer_routing_table);
      aggregation_mgr_->set_leaf_warnings(std::move(merged.leaf_warnings));

      // Track per-entity peer contributors: entity id -> list of peer
      // names that host or contribute to it. Covers both routed leaves
      // (also tracked via routing_table) and merged / hierarchical entities
      // (Areas, Functions, parent Components). Fan-out helpers use this map
      // to target per-entity collection requests at the peers that actually
      // own the entity, avoiding spurious 404s from non-contributing peers.
      //
      // The "peer:" prefix is stripped from the contributors list produced
      // by EntityMerger. Entries starting with anything else (including
      // "local") are ignored.
      std::unordered_map<std::string, std::vector<std::string>> peer_contributors;
      static constexpr std::string_view kPeerPrefix = "peer:";
      auto collect_contributors = [&peer_contributors](const auto & entities) {
        for (const auto & e : entities) {
          for (const auto & c : e.contributors) {
            if (c.rfind(kPeerPrefix, 0) != 0) {
              continue;
            }
            std::string peer_name = c.substr(kPeerPrefix.size());
            if (peer_name.empty()) {
              continue;
            }
            auto & list = peer_contributors[e.id];
            if (std::find(list.begin(), list.end(), peer_name) == list.end()) {
              list.push_back(std::move(peer_name));
            }
          }
        }
      };
      collect_contributors(areas);
      collect_contributors(all_components);
      collect_contributors(apps);
      collect_contributors(functions);
      aggregation_mgr_->update_peer_contributors(std::move(peer_contributors));
    }

    // Inject plugin entities (non-hybrid) and refresh entity ownership (all modes).
    // Single introspect() call per plugin handles both concerns.
    if (plugin_mgr_ && plugin_mgr_->has_plugins()) {
      bool inject_entities = (discovery_mgr_->get_mode() != DiscoveryMode::HYBRID);
      auto providers = plugin_mgr_->get_named_introspection_providers();
      for (auto & [name, provider] : providers) {
        IntrospectionInput input;
        IntrospectionResult result;
        try {
          result = provider->introspect(input);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Plugin '%s' introspect() threw during refresh: %s", name.c_str(), e.what());
          plugin_mgr_->clear_entity_ownership(name);
          continue;
        }

        // In non-hybrid modes, inject plugin entities directly (with validation).
        // Collect validated entity IDs for ownership registration.
        std::vector<std::string> entity_ids;
        if (inject_entities) {
          for (auto & area : result.new_entities.areas) {
            if (!validate_entity_id(area.id)) {
              RCLCPP_WARN(get_logger(), "Plugin '%s': dropping area with invalid ID '%s'", name.c_str(),
                          area.id.c_str());
              continue;
            }
            entity_ids.push_back(area.id);
            if (area.source.empty()) {
              area.source = "plugin";
            }
            areas.push_back(std::move(area));
          }
          for (auto & comp : result.new_entities.components) {
            if (!validate_entity_id(comp.id)) {
              RCLCPP_WARN(get_logger(), "Plugin '%s': dropping component with invalid ID '%s'", name.c_str(),
                          comp.id.c_str());
              continue;
            }
            entity_ids.push_back(comp.id);
            if (comp.source.empty()) {
              comp.source = "plugin";
            }
            all_components.push_back(std::move(comp));
          }
          for (auto & app : result.new_entities.apps) {
            if (!validate_entity_id(app.id)) {
              RCLCPP_WARN(get_logger(), "Plugin '%s': dropping app with invalid ID '%s'", name.c_str(), app.id.c_str());
              continue;
            }
            entity_ids.push_back(app.id);
            if (app.source.empty()) {
              app.source = "plugin";
            }
            apps.push_back(std::move(app));
          }
          for (auto & func : result.new_entities.functions) {
            if (!validate_entity_id(func.id)) {
              RCLCPP_WARN(get_logger(), "Plugin '%s': dropping function with invalid ID '%s'", name.c_str(),
                          func.id.c_str());
              continue;
            }
            entity_ids.push_back(func.id);
            if (func.source.empty()) {
              func.source = "plugin";
            }
            functions.push_back(std::move(func));
          }
        } else {
          // Hybrid mode: PluginLayer validates in discover(). Collect IDs
          // with validation for ownership (entities are not injected here).
          for (const auto & area : result.new_entities.areas) {
            if (validate_entity_id(area.id)) {
              entity_ids.push_back(area.id);
            }
          }
          for (const auto & comp : result.new_entities.components) {
            if (validate_entity_id(comp.id)) {
              entity_ids.push_back(comp.id);
            }
          }
          for (const auto & app : result.new_entities.apps) {
            if (validate_entity_id(app.id)) {
              entity_ids.push_back(app.id);
            }
          }
          for (const auto & func : result.new_entities.functions) {
            if (validate_entity_id(func.id)) {
              entity_ids.push_back(func.id);
            }
          }
        }

        // Refresh ownership with validated IDs only
        plugin_mgr_->clear_entity_ownership(name);
        plugin_mgr_->register_entity_ownership(name, entity_ids);
      }
    }

    // Filter ROS 2 internal nodes (underscore prefix convention).
    // Controlled by discovery.runtime.filter_internal_nodes parameter (default: true).
    // Covers local heuristic apps (which bypass the merge pipeline orphan filter
    // in runtime_only mode) and any peer apps that slipped through fetch_entities.
    if (filter_internal_nodes_) {
      auto removed = filter_internal_node_apps(apps, peer_routing_table);
      if (removed > 0) {
        RCLCPP_DEBUG(get_logger(), "Filtered %zu internal node apps (_ prefix)", removed);
      }
    }

    // Capture sizes for logging
    const size_t area_count = areas.size();
    const size_t component_count = all_components.size();
    const size_t app_count = apps.size();
    const size_t function_count = functions.size();

    // Populate node_to_app mapping for trigger entity resolution (#305)
    std::unordered_map<std::string, std::string> node_to_app;
    auto linking = discovery_mgr_->get_linking_result();
    if (linking) {
      node_to_app = std::move(linking->node_to_app);
    }

    // Update ThreadSafeEntityCache atomically (entities + node_to_app under single lock)
    thread_safe_cache_.update_all(std::move(areas), std::move(all_components), std::move(apps), std::move(functions),
                                  std::move(node_to_app));

    // Emit a one-shot WARN when the entity cache grew beyond its reserved capacity.
    // Growing is harmless but causes structural reallocation; embedded / memory-
    // constrained deployments should raise entity_cache.capacity to prevent it.
    if (!warned_cache_grow_) {
      const auto stats = thread_safe_cache_.get_stats();
      if (stats.grew) {
        RCLCPP_WARN(get_logger(),
                    "entity_cache capacity %zu exceeded (grew); raise entity_cache.capacity "
                    "for embedded determinism",
                    stats.capacity);
        warned_cache_grow_ = true;
      }
    }

    // Update topic type cache (avoids expensive ROS graph queries on /data requests)
    if (data_access_mgr_) {
      std::vector<TopicInfo> all_topics;
      if (auto * provider = data_access_mgr_->get_topic_data_provider()) {
        all_topics = provider->discover_all();
      }
      std::unordered_map<std::string, std::string> topic_types;
      topic_types.reserve(all_topics.size());
      for (const auto & topic : all_topics) {
        if (!topic.type.empty()) {
          topic_types[topic.name] = topic.type;
        }
      }
      thread_safe_cache_.update_topic_types(std::move(topic_types));
    }

    RCLCPP_DEBUG(get_logger(), "Cache refreshed: %zu areas, %zu components, %zu apps, %zu functions", area_count,
                 component_count, app_count, function_count);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to refresh cache: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to refresh cache: unknown exception");
  }
}

void GatewayNode::start_rest_server() {
  server_thread_ = std::make_unique<std::thread>([this]() {
    try {
      rest_server_->start();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "REST server failed to start: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "REST server failed to start: unknown exception");
    }
  });

  // Wait for the server to actually reach cpp-httplib's accept loop before
  // returning. is_running() becomes true only after listen() has passed
  // bind and entered its select/poll loop; using a "thread started" flag
  // here is not enough because stop() called before listen() entered its
  // loop could be missed, leaving listen() blocking forever and the
  // subsequent join() hanging indefinitely.
  using namespace std::chrono_literals;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!rest_server_->is_running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (!rest_server_->is_running()) {
    RCLCPP_ERROR(get_logger(), "REST server did not become ready within 5s");
  }
}

void GatewayNode::stop_rest_server() {
  if (rest_server_) {
    rest_server_->stop();
  }
  if (server_thread_ && server_thread_->joinable()) {
    server_thread_->join();
  }
}

size_t filter_internal_node_apps(std::vector<App> & apps,
                                 const std::unordered_map<std::string, std::string> & peer_routing_table) {
  auto before = apps.size();
  auto end = std::remove_if(apps.begin(), apps.end(), [&peer_routing_table](const App & app) {
    std::string original_id = app.id;
    auto rt_it = peer_routing_table.find(app.id);
    if (rt_it != peer_routing_table.end()) {
      // Known remote entity: strip "peer_name__" prefix if present
      const std::string & peer_name = rt_it->second;
      std::string prefix = peer_name + "__";
      if (original_id.size() > prefix.size() && original_id.compare(0, prefix.size(), prefix) == 0) {
        original_id = original_id.substr(prefix.size());
      }
    }
    // ROS 2 internal nodes use _ prefix convention
    return !original_id.empty() && original_id[0] == '_';
  });
  apps.erase(end, apps.end());
  return before - apps.size();
}

}  // namespace ros2_medkit_gateway
