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

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief JWT signing algorithm
 */
enum class JwtAlgorithm {
  HS256,  ///< HMAC-SHA256 (symmetric)
  RS256   ///< RSA-SHA256 (asymmetric)
};

/**
 * @brief User role for RBAC
 * @verifies REQ_INTEROP_086
 */
enum class UserRole {
  VIEWER,        ///< Read-only access: GET on areas, components, data, faults
  OPERATOR,      ///< Viewer + trigger operations, acknowledge faults
  CONFIGURATOR,  ///< Operator + modify configurations
  ADMIN          ///< Full access including auth management
};

/**
 * @brief RBAC permission entries, keyed by role.
 *
 * Each entry is `"<METHOD>:<path pattern>"`, where the pattern is matched by
 * `AuthManager::matches_path`: `*` covers one path segment and `**` covers any
 * number. There is no role inheritance - a role's set has to list everything
 * that role may do, which is why the derivation in
 * `RouteRegistry::route_permissions()` expands each declaration upward.
 */
using RoutePermissions = std::unordered_map<UserRole, std::unordered_set<std::string>>;

/**
 * @brief Authentication requirement level
 */
enum class AuthRequirement {
  NONE,   ///< No authentication required
  WRITE,  ///< Authentication required only for write operations
  ALL     ///< Authentication required for all operations
};

/**
 * @brief Client credentials for authentication
 */
struct ClientCredentials {
  std::string client_id;
  std::string client_secret;
  UserRole role{UserRole::VIEWER};
  bool enabled{true};

  bool operator==(const ClientCredentials & other) const {
    return client_id == other.client_id;
  }
};

/**
 * @brief Authentication configuration
 * @verifies REQ_INTEROP_086, REQ_INTEROP_087
 */
struct AuthConfig {
  bool enabled{false};                                       ///< Whether authentication is enabled
  std::string jwt_secret;                                    ///< Secret for HS256 or path to private key for RS256
  std::string jwt_public_key;                                ///< Path to public key for RS256 (optional)
  JwtAlgorithm jwt_algorithm{JwtAlgorithm::HS256};           ///< JWT signing algorithm
  int token_expiry_seconds{3600};                            ///< Access token validity (default: 1 hour)
  int refresh_token_expiry_seconds{86400};                   ///< Refresh token validity (default: 24 hours)
  AuthRequirement require_auth_for{AuthRequirement::WRITE};  ///< When to require authentication
  std::string issuer{"ros2_medkit_gateway"};                 ///< JWT issuer claim

  // Pre-configured clients (for development/testing)
  std::vector<ClientCredentials> clients;

  /// Permission entries for the routes the `RouteRegistry` does not hold.
  ///
  /// The gateway's own routes derive their entries from their registration
  /// (`RouteRegistry::route_permissions()`). Three families of route are
  /// mounted straight onto the cpp-httplib server instead and so cannot:
  /// plugin routes (`PluginManager::register_routes`), the Swagger UI pages
  /// (`-DENABLE_SWAGGER_UI=ON` builds only) and, in test builds, the emitted
  /// status recorder. This is the whole residual list for them, and it is
  /// deliberately short: ADMIN's four `**` entries, which is what has always
  /// covered those paths and is why a plugin operation's published role reads
  /// `admin`. Nothing weaker reaches them - a viewer token on a plugin route
  /// is 403, before and after the derivation.
  static const RoutePermissions & residual_route_permissions();
};

/**
 * @brief Builder for AuthConfig with fluent interface
 */
class AuthConfigBuilder {
 public:
  AuthConfigBuilder & with_enabled(bool enabled);
  AuthConfigBuilder & with_jwt_secret(const std::string & secret);
  AuthConfigBuilder & with_jwt_public_key(const std::string & public_key);
  AuthConfigBuilder & with_algorithm(JwtAlgorithm algorithm);
  AuthConfigBuilder & with_token_expiry(int seconds);
  AuthConfigBuilder & with_refresh_token_expiry(int seconds);
  AuthConfigBuilder & with_require_auth_for(AuthRequirement requirement);
  AuthConfigBuilder & with_issuer(const std::string & issuer);
  AuthConfigBuilder & add_client(const std::string & client_id, const std::string & client_secret, UserRole role);
  AuthConfig build();

 private:
  AuthConfig config_;
};

/**
 * @brief Convert UserRole to string
 */
std::string role_to_string(UserRole role);

/**
 * @brief Convert string to UserRole
 * @throws std::invalid_argument if string is not a valid role
 */
UserRole string_to_role(const std::string & role_str);

/**
 * @brief Convert JwtAlgorithm to string
 */
std::string algorithm_to_string(JwtAlgorithm algorithm);

/**
 * @brief Convert string to JwtAlgorithm
 * @throws std::invalid_argument if string is not a valid algorithm
 */
JwtAlgorithm string_to_algorithm(const std::string & alg_str);

/**
 * @brief Convert string to AuthRequirement
 * @throws std::invalid_argument if string is not a valid requirement
 */
AuthRequirement string_to_auth_requirement(const std::string & req_str);

}  // namespace ros2_medkit_gateway
