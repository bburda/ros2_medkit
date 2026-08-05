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

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"

#include <algorithm>
#include <stdexcept>

namespace ros2_medkit_gateway {

const RoutePermissions & AuthConfig::residual_route_permissions() {
  // Format: "HTTP_METHOD:/path/pattern", where `*` is one path segment and
  // `**` is any number of them (`AuthManager::matches_path`).
  //
  // The gateway's own routes are NOT here. Their entries are derived from the
  // registration that mounts them - `RouteRegistry::route_permissions()` - and
  // merged into the manager at startup by `RESTServer::setup_routes()`. What
  // is left is the residual: the paths served by something the registry never
  // sees, which is every route mounted straight onto the cpp-httplib server.
  //
  // ADMIN's four wildcards are that residual, and they are also the reason a
  // plugin-served operation publishes `admin` as its role: `*` stops at a
  // segment boundary, so no weaker role's entry reaches
  // `/api/v1/functions/<id>/x-medkit-graph`, and only `GET:/api/v1/**` does.
  // Widening this to a weaker role would hand that role every plugin route the
  // deployment happens to load, sight unseen.
  //
  // @verifies REQ_INTEROP_086
  static const RoutePermissions permissions = {{UserRole::ADMIN,
                                                {
                                                    "GET:/api/v1/**",
                                                    "POST:/api/v1/**",
                                                    "PUT:/api/v1/**",
                                                    "DELETE:/api/v1/**",
                                                }}};
  return permissions;
}

AuthConfigBuilder & AuthConfigBuilder::with_enabled(bool enabled) {
  config_.enabled = enabled;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_jwt_secret(const std::string & secret) {
  config_.jwt_secret = secret;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_jwt_public_key(const std::string & public_key) {
  config_.jwt_public_key = public_key;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_algorithm(JwtAlgorithm algorithm) {
  config_.jwt_algorithm = algorithm;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_token_expiry(int seconds) {
  config_.token_expiry_seconds = seconds;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_refresh_token_expiry(int seconds) {
  config_.refresh_token_expiry_seconds = seconds;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_require_auth_for(AuthRequirement requirement) {
  config_.require_auth_for = requirement;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_issuer(const std::string & issuer) {
  config_.issuer = issuer;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::add_client(const std::string & client_id, const std::string & client_secret,
                                                  UserRole role) {
  ClientCredentials creds;
  creds.client_id = client_id;
  creds.client_secret = client_secret;
  creds.role = role;
  creds.enabled = true;
  config_.clients.push_back(creds);
  return *this;
}

AuthConfig AuthConfigBuilder::build() {
  // Validate configuration
  if (config_.enabled) {
    if (config_.jwt_secret.empty()) {
      throw std::invalid_argument("JWT secret is required when authentication is enabled");
    }

    // Enforce a minimum secret length for HS256 to avoid weak symmetric keys
    if (config_.jwt_algorithm == JwtAlgorithm::HS256 && config_.jwt_secret.size() < 32U) {
      throw std::invalid_argument("JWT secret must be at least 32 characters for HS256 algorithm");
    }

    if (config_.token_expiry_seconds <= 0) {
      throw std::invalid_argument("Token expiry must be positive");
    }

    if (config_.refresh_token_expiry_seconds <= 0) {
      throw std::invalid_argument("Refresh token expiry must be positive");
    }

    if (config_.refresh_token_expiry_seconds < config_.token_expiry_seconds) {
      throw std::invalid_argument("Refresh token expiry must be greater than or equal to token expiry");
    }

    // For RS256, validate key paths exist (actual file check done at runtime)
    if (config_.jwt_algorithm == JwtAlgorithm::RS256) {
      if (config_.jwt_public_key.empty()) {
        throw std::invalid_argument("Public key path is required for RS256 algorithm");
      }
    }
  }

  return config_;
}

std::string role_to_string(UserRole role) {
  switch (role) {
    case UserRole::VIEWER:
      return "viewer";
    case UserRole::OPERATOR:
      return "operator";
    case UserRole::CONFIGURATOR:
      return "configurator";
    case UserRole::ADMIN:
      return "admin";
    default:
      return "unknown";
  }
}

UserRole string_to_role(const std::string & role_str) {
  std::string lower_role = role_str;
  std::transform(lower_role.begin(), lower_role.end(), lower_role.begin(), ::tolower);

  if (lower_role == "viewer") {
    return UserRole::VIEWER;
  }
  if (lower_role == "operator") {
    return UserRole::OPERATOR;
  }
  if (lower_role == "configurator") {
    return UserRole::CONFIGURATOR;
  }
  if (lower_role == "admin") {
    return UserRole::ADMIN;
  }
  throw std::invalid_argument("Invalid role: " + role_str);
}

std::string algorithm_to_string(JwtAlgorithm algorithm) {
  switch (algorithm) {
    case JwtAlgorithm::HS256:
      return "HS256";
    case JwtAlgorithm::RS256:
      return "RS256";
    default:
      return "unknown";
  }
}

JwtAlgorithm string_to_algorithm(const std::string & alg_str) {
  std::string upper_alg = alg_str;
  std::transform(upper_alg.begin(), upper_alg.end(), upper_alg.begin(), ::toupper);

  if (upper_alg == "HS256") {
    return JwtAlgorithm::HS256;
  }
  if (upper_alg == "RS256") {
    return JwtAlgorithm::RS256;
  }
  throw std::invalid_argument("Invalid algorithm: " + alg_str + ". Supported: HS256, RS256");
}

AuthRequirement string_to_auth_requirement(const std::string & req_str) {
  std::string lower_req = req_str;
  std::transform(lower_req.begin(), lower_req.end(), lower_req.begin(), ::tolower);

  if (lower_req == "none") {
    return AuthRequirement::NONE;
  }
  if (lower_req == "write") {
    return AuthRequirement::WRITE;
  }
  if (lower_req == "all") {
    return AuthRequirement::ALL;
  }
  throw std::invalid_argument("Invalid auth requirement: " + req_str + ". Supported: none, write, all");
}

}  // namespace ros2_medkit_gateway
