// Copyright 2025 bburda
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

#include <memory>
#include <mutex>
#include <string>
#include <tl/expected.hpp>
#include <unordered_map>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/auth/auth_requirement_policy.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Manages authentication and authorization for the gateway
 *
 * Implements JWT-based authentication with RBAC (Role-Based Access Control).
 * Supports both symmetric (HS256) and asymmetric (RS256) JWT signing.
 *
 * @verifies REQ_INTEROP_086, REQ_INTEROP_087
 */
class AuthManager {
 public:
  /**
   * @brief Construct AuthManager with configuration
   * @param config Authentication configuration
   */
  explicit AuthManager(const AuthConfig & config);

  ~AuthManager() = default;

  // Disable copy and move (contains non-movable mutex members)
  AuthManager(const AuthManager &) = delete;
  AuthManager & operator=(const AuthManager &) = delete;
  AuthManager(AuthManager &&) = delete;
  AuthManager & operator=(AuthManager &&) = delete;

  /**
   * @brief Check if authentication is enabled
   * @return true if auth is enabled
   */
  bool is_enabled() const {
    return config_.enabled;
  }

  /**
   * @brief Get the auth requirement level
   * @return AuthRequirement level
   */
  AuthRequirement get_requirement() const {
    return config_.require_auth_for;
  }

  /**
   * @brief Authenticate client credentials and generate tokens
   * @param client_id Client identifier
   * @param client_secret Client secret
   * @return TokenResponse on success, AuthErrorResponse on failure
   */
  tl::expected<TokenResponse, AuthErrorResponse> authenticate(const std::string & client_id,
                                                              const std::string & client_secret);

  /**
   * @brief Refresh an access token using a refresh token
   * @param refresh_token The refresh token
   * @return TokenResponse on success, AuthErrorResponse on failure
   */
  tl::expected<TokenResponse, AuthErrorResponse> refresh_access_token(const std::string & refresh_token);

  /**
   * @brief Validate a JWT access token
   * @param token The JWT token string
   * @param expected_type Expected token type (defaults to ACCESS)
   * @return TokenValidationResult with claims if valid
   */
  TokenValidationResult validate_token(const std::string & token, TokenType expected_type = TokenType::ACCESS) const;

  /**
   * @brief Check if a role is authorized for a specific HTTP method and path
   * @param role User role
   * @param method HTTP method (GET, POST, PUT, DELETE)
   * @param path Request path (e.g., /api/v1/components/engine/data)
   * @return AuthorizationResult indicating if authorized
   */
  AuthorizationResult check_authorization(UserRole role, const std::string & method, const std::string & path) const;

  /**
   * @brief Merge permission entries into the table check_authorization reads.
   *
   * The table starts empty and the check fails closed, so a manager nobody
   * feeds authorizes nothing. That is deliberate: the entries belong to the
   * route set actually mounted, and only the caller that mounts the routes
   * knows what that is. `RESTServer::setup_routes()` merges two sources - the
   * registry's derivation for the gateway's own routes and
   * `AuthConfig::residual_route_permissions()` for the ones mounted outside it
   * - before the HTTP server starts listening.
   *
   * Merging, not replacing, so those two calls compose. Not thread-safe, and
   * not made so: every call happens on the constructing thread before
   * `RESTServer::start()`, which is what publishes the table to the request
   * threads that read it.
   *
   * @param permissions Entries to add, keyed by role
   */
  void add_route_permissions(const RoutePermissions & permissions);

  /**
   * @brief Check if authentication is required for a request
   * @param method HTTP method
   * @param path Request path
   * @return true if authentication is required
   */
  bool requires_authentication(const std::string & method, const std::string & path) const;

  /**
   * @brief Revoke a refresh token
   * @param refresh_token The refresh token to revoke
   * @return true if revoked, false if not found
   */
  bool revoke_refresh_token(const std::string & refresh_token);

  /**
   * @brief Clean up expired refresh tokens
   * @return Number of tokens cleaned up
   */
  size_t cleanup_expired_tokens();

  /**
   * @brief Register a new client (for dynamic client registration)
   * @param client_id Client identifier
   * @param client_secret Client secret
   * @param role Role to assign
   * @return true if registered, false if client_id already exists
   */
  bool register_client(const std::string & client_id, const std::string & client_secret, UserRole role);

  /**
   * @brief Get client credentials by ID
   * @param client_id Client identifier
   * @return ClientCredentials if found
   */
  std::optional<ClientCredentials> get_client(const std::string & client_id) const;

  /**
   * @brief Disable a client (all tokens become invalid immediately)
   * @param client_id Client identifier
   * @return true if disabled, false if client not found
   */
  bool disable_client(const std::string & client_id);

  /**
   * @brief Enable a previously disabled client
   * @param client_id Client identifier
   * @return true if enabled, false if client not found
   */
  bool enable_client(const std::string & client_id);

 private:
  /**
   * @brief Generate a JWT token
   * @param claims Token claims
   * @return JWT token string
   */
  std::string generate_jwt(const JwtClaims & claims) const;

  /**
   * @brief Decode and verify a JWT token
   * @param token JWT token string
   * @return JwtClaims if valid
   */
  tl::expected<JwtClaims, std::string> decode_jwt(const std::string & token) const;

  /**
   * @brief Generate a unique token ID
   * @return UUID string
   */
  static std::string generate_token_id();

  /**
   * @brief Check if a permission pattern matches a path
   * @param pattern Permission pattern (may contain *)
   * @param path Actual request path
   * @return true if matches
   */
  static bool matches_path(const std::string & pattern, const std::string & path);

  /**
   * @brief Store a refresh token
   * @param record Refresh token record
   */
  void store_refresh_token(const RefreshTokenRecord & record);

  /**
   * @brief Get a refresh token record by token ID
   * @param token_id Token ID (jti)
   * @return RefreshTokenRecord if found
   */
  std::optional<RefreshTokenRecord> get_refresh_token(const std::string & token_id) const;

  AuthConfig config_;

  // RBAC entries check_authorization matches against, populated via
  // add_route_permissions() before the server starts listening. Empty until
  // then, and an empty set authorizes nothing - see add_route_permissions().
  RoutePermissions permissions_;

  // Auth requirement policy (created from config)
  std::unique_ptr<IAuthRequirementPolicy> auth_policy_;

  // Client credentials storage (thread-safe)
  mutable std::mutex clients_mutex_;
  std::unordered_map<std::string, ClientCredentials> clients_;

  // Refresh token storage (thread-safe)
  mutable std::mutex refresh_tokens_mutex_;
  std::unordered_map<std::string, RefreshTokenRecord> refresh_tokens_;
};

}  // namespace ros2_medkit_gateway
