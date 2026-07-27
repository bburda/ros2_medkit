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

#include <iomanip>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <string>
#include <tl/expected.hpp>

#include <vector>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Entity type enumeration for SOVD entities
 * @note This is the legacy EntityType enum. For new code, prefer using SovdEntityType.
 */
enum class EntityType { COMPONENT, APP, AREA, FUNCTION, UNKNOWN };

/**
 * @brief Convert legacy EntityType to SovdEntityType
 */
inline SovdEntityType to_sovd_entity_type(EntityType type) {
  switch (type) {
    case EntityType::COMPONENT:
      return SovdEntityType::COMPONENT;
    case EntityType::APP:
      return SovdEntityType::APP;
    case EntityType::AREA:
      return SovdEntityType::AREA;
    case EntityType::FUNCTION:
      return SovdEntityType::FUNCTION;
    case EntityType::UNKNOWN:
    default:
      return SovdEntityType::UNKNOWN;
  }
}

/**
 * @brief Information about a resolved entity
 */
struct EntityInfo {
  EntityType type{EntityType::UNKNOWN};
  std::string id;
  std::string namespace_path;
  std::string fqn;         ///< Fully qualified name (for ROS 2 nodes)
  std::string id_field;    ///< JSON field name for ID ("component_id", "app_id", etc.)
  std::string error_name;  ///< Human-readable name for errors ("Component", "App", etc.)

  // Peer aggregation fields
  bool is_remote{false};  ///< True if entity was discovered from a peer gateway
  std::string peer_url;   ///< Base URL of the peer (e.g., "http://localhost:8081")
  std::string peer_name;  ///< Peer name for metadata (e.g., "subsystem_b")

  // Plugin routing fields
  bool is_plugin{false};    ///< True if entity is owned by a plugin
  std::string plugin_name;  ///< Plugin name (empty if not plugin-owned)

  /**
   * @brief Get SovdEntityType equivalent
   */
  SovdEntityType sovd_type() const {
    return to_sovd_entity_type(type);
  }

  /**
   * @brief Check if entity supports a resource collection
   */
  bool supports_collection(ResourceCollection col) const {
    return EntityCapabilities::for_type(sovd_type()).supports_collection(col);
  }
};

// Forward declarations
class GatewayNode;
class AuthManager;
class BulkDataStore;
class AggregationManager;

namespace handlers {

/**
 * @brief Shared context for all HTTP handlers
 *
 * Provides access to gateway node, configuration, and common utilities
 * that handlers need to process requests.
 */
class HandlerContext {
 public:
  HandlerContext(GatewayNode * node, const CorsConfig & cors_config, const AuthConfig & auth_config,
                 const TlsConfig & tls_config, AuthManager * auth_manager, BulkDataStore * bulk_data_store = nullptr)
    : node_(node)
    , cors_config_(cors_config)
    , auth_config_(auth_config)
    , tls_config_(tls_config)
    , auth_manager_(auth_manager)
    , bulk_data_store_(bulk_data_store) {
  }

  /// Get gateway node
  GatewayNode * node() const {
    return node_;
  }

  /// Get CORS configuration
  const CorsConfig & cors_config() const {
    return cors_config_;
  }

  /// Get authentication configuration
  const AuthConfig & auth_config() const {
    return auth_config_;
  }

  /// Get TLS configuration
  const TlsConfig & tls_config() const {
    return tls_config_;
  }

  /// Get authentication manager (may be nullptr if auth disabled)
  AuthManager * auth_manager() const {
    return auth_manager_;
  }

  /// Get bulk data store (may be nullptr if not configured)
  BulkDataStore * bulk_data_store() const {
    return bulk_data_store_;
  }

  /// Set the aggregation manager (non-owning, null when aggregation disabled)
  void set_aggregation_manager(AggregationManager * mgr) {
    aggregation_mgr_ = mgr;
  }

  /// Get the aggregation manager (may be nullptr if aggregation disabled)
  AggregationManager * aggregation_manager() const {
    return aggregation_mgr_;
  }

  /**
   * @brief Validate entity ID (component_id, area_id, etc.)
   * @param entity_id The ID to validate
   * @return Empty if valid, error message otherwise
   */
  tl::expected<void, std::string> validate_entity_id(const std::string & entity_id) const;

  /**
   * @brief Get namespace path for a component
   * @param component_id Component ID
   * @return Namespace path or error message
   * @deprecated Use get_entity_info instead for unified entity handling
   */
  tl::expected<std::string, std::string> get_component_namespace_path(const std::string & component_id) const;

  /**
   * @brief Get information about any entity (Component, App, Area, Function)
   *
   * If expected_type is specified, searches ONLY in that collection.
   * If expected_type is UNKNOWN (default), searches all types in order:
   * Component, App, Area, Function - returns the first match found.
   *
   * @param entity_id Entity ID to look up
   * @param expected_type Optional: restrict search to specific entity type
   * @return EntityInfo with resolved details, or EntityInfo with UNKNOWN type if not found
   */
  EntityInfo get_entity_info(const std::string & entity_id,
                             SovdEntityType expected_type = SovdEntityType::UNKNOWN) const;

  /**
   * @brief Validate that entity supports a resource collection (typed variant).
   *
   * Returns a populated ErrorInfo (HTTP 400, ERR_COLLECTION_NOT_SUPPORTED) when
   * the entity type does not support the collection. The typed router serializes
   * the error onto the wire; this method never touches an httplib::Response.
   *
   * @param entity Entity information (from get_entity_info)
   * @param collection Resource collection to validate
   * @return Empty success on support, ErrorInfo on rejection.
   */
  static tl::expected<void, ErrorInfo> validate_collection_access_typed(const EntityInfo & entity,
                                                                        ResourceCollection collection);

  /**
   * @brief Validate lock access for a mutating operation.
   *
   * Two-phase check:
   * 1. If locking disabled (no LockManager), allows access.
   * 2. Extracts X-Client-Id header, checks LockManager::check_access().
   *
   * On denial, returns a populated ErrorInfo (HTTP 409 with either
   * ERR_LOCK_BROKEN or ERR_INVALID_REQUEST). The method never touches an
   * httplib::Response; the typed router serializes the ErrorInfo onto the
   * wire. Locking is always local, so there is no Forwarded variant.
   *
   * @param req Typed HTTP request (for X-Client-Id header).
   * @param entity Entity being accessed.
   * @param collection Resource collection being mutated (e.g. "configurations").
   * @return Empty success on allow, ErrorInfo on denial.
   */
  tl::expected<void, ErrorInfo> validate_lock_access(const http::TypedRequest & req, const EntityInfo & entity,
                                                     const std::string & collection);

  /**
   * @brief Validate entity exists and matches expected route type (typed variant).
   *
   * Unified validation helper that:
   * 1. Validates the entity ID format.
   * 2. Looks up the entity in the expected collection (based on route path).
   * 3. For remote entities (aggregation), forwards the request to the peer
   *    gateway, writes the proxied response to the underlying httplib::Response,
   *    and returns Forwarded so the caller knows the wire is committed.
   * 4. On local failure, returns a populated ErrorInfo without touching the
   *    response. The typed router serializes the error.
   *
   * The success branch carries the resolved EntityInfo. The error branch is a
   * variant of:
   *   - ErrorInfo  - local validation failure (400 invalid-parameter, 404
   *                  entity-not-found). Caller serializes via write_generic_error.
   *   - Forwarded  - request was proxied to a peer; the response is already
   *                  written and the caller must return immediately without
   *                  touching the response further.
   *
   * The Forwarded path is the one place the validator still mutates the
   * underlying httplib::Response. Aggregation could not be modeled cleanly any
   * other way without either (a) returning the proxied bytes as a buffered
   * payload (loses streaming and headers) or (b) exposing the response object
   * to the caller (defeats the typed surface). Keeping the proxy write inside
   * the validator preserves the historical aggregation behavior bit-for-bit.
   *
   * @param req Typed HTTP request (used to extract expected type from path and
   *            to drive the forward proxy).
   * @param entity_id Entity ID to validate.
   * @return EntityInfo on local success, or an error variant indicating local
   *         failure (ErrorInfo) vs. peer forwarding completed (Forwarded).
   */
  http::ValidatorResult<EntityInfo> validate_entity_for_route(const http::TypedRequest & req,
                                                              const std::string & entity_id) const;

  /**
   * @brief Build the framework-internal sentinel error that typed handlers
   *        return after the validator's Forwarded path already committed the
   *        proxied wire response.
   *
   * Typed handlers use this helper so the typed `Result<T>` shape can
   * propagate the validator's `Forwarded` variant without re-rendering the
   * body. The wrapper in RouteRegistry detects the sentinel via its error
   * code (ERR_X_INTERNAL_FORWARDED) and skips error rendering.
   *
   * @return ErrorInfo with the sentinel code and `http_status == 0`.
   */
  static ErrorInfo forwarded_sentinel_error() {
    ErrorInfo info;
    info.code = ERR_X_INTERNAL_FORWARDED;
    info.message = "internal: response already forwarded to peer";
    info.http_status = 0;
    return info;
  }

  /**
   * @brief Set CORS headers on response if origin is allowed
   * @param res HTTP response
   * @param origin Origin header value
   */
  void set_cors_headers(httplib::Response & res, const std::string & origin) const;

  /**
   * @brief Check if origin is allowed by CORS config
   * @param origin Origin header value
   * @return true if allowed
   */
  bool is_origin_allowed(const std::string & origin) const;

  /**
   * @brief Get logger for handlers
   */
  static rclcpp::Logger logger() {
    return rclcpp::get_logger("rest_server");
  }

  /**
   * @brief Resolve a list of app IDs to the fault-reporting sources they own.
   *
   * Each app resolves through `faults::resolve_app_source_fqn()`: its
   * `effective_fqn()`, or its bare entity id when the app is external (a
   * plugin-introspected asset that reports under that id). Apps that are
   * missing from the cache or that own no source are skipped silently. The
   * returned vector preserves the input app_ids order (minus skipped entries)
   * and may be empty. Duplicate sources from manifest / runtime double-binds
   * are collapsed so they do not produce repeated downstream queries.
   *
   * Order-preserving wrapper over the shared per-app rule for callers that
   * resolve an explicit app-id list; whole-entity resolution goes through
   * `resolve_entity_source_fqns`. Static + public for direct unit testing
   * without standing up a full GatewayNode fixture.
   *
   * @param cache Entity cache to look up apps in
   * @param app_ids App IDs to resolve
   * @return Source FQNs for the apps that resolved
   */
  static std::vector<std::string> resolve_app_host_fqns(const ThreadSafeEntityCache & cache,
                                                        const std::vector<std::string> & app_ids);

  /**
   * @brief Resolve an entity to the set of reporting sources it owns.
   *
   * Thin adapter over `faults::resolve_entity_source_fqns` (the per-type
   * mapping - incl. the external bare-id rule, subarea recursion and
   * component-hosted functions - is documented there). Used by fault, log and
   * bulk-data handlers so per-entity routes never expose resources reported
   * from outside the addressed entity.
   *
   * Apps that own no source are silently dropped from the set so the scope
   * check cannot match arbitrary FQNs against an empty prefix.
   *
   * An empty result means "no sources are in scope" and callers must NEVER
   * interpret that as "no filter" - any fault must be treated as out of
   * scope. The exact response (404 for per-fault routes, an empty `items`
   * array for collection lists, 204 for collection clears) is up to the
   * caller.
   *
   * @param cache Entity cache for lookups
   * @param entity Resolved entity info (from `validate_entity_for_route`)
   * @return Set of sources that uniquely scopes faults to this entity
   */
  static std::set<std::string> resolve_entity_source_fqns(const ThreadSafeEntityCache & cache,
                                                          const EntityInfo & entity);

 private:
  GatewayNode * node_;
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  TlsConfig tls_config_;
  AuthManager * auth_manager_;
  BulkDataStore * bulk_data_store_;
  AggregationManager * aggregation_mgr_{nullptr};  ///< Non-owning, null when aggregation disabled
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
