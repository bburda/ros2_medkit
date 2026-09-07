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

#include <tl/expected.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Interface for authentication requirement policies
 *
 * Allows customizing when authentication is required for different
 * HTTP methods and paths using the Strategy pattern.
 *
 * @verifies REQ_INTEROP_086
 */
class IAuthRequirementPolicy {
 public:
  virtual ~IAuthRequirementPolicy() = default;

  /**
   * @brief Check if authentication is required for a request
   * @param method HTTP method (GET, POST, PUT, DELETE, etc.)
   * @param path Request path
   * @return true if authentication is required
   */
  virtual bool requires_authentication(const std::string & method, const std::string & path) const = 0;

  /**
   * @brief Get a human-readable description of this policy
   * @return Description string
   */
  virtual std::string description() const = 0;
};

/**
 * @brief Policy that never requires authentication
 */
class NoAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    (void)method;
    (void)path;
    return false;
  }

  std::string description() const override {
    return "NoAuth: Authentication is never required";
  }
};

/**
 * @brief Policy that always requires authentication
 *
 * The only exception is anything under `/api/v1/auth/`. Authentication cannot
 * bootstrap through a door that demands the credential it exists to hand out.
 *
 * Nothing else is public here, health probes included. An operator who needs
 * a route answered without a credential names it in `auth.public_routes`,
 * which layers over this policy - see PublicRouteExemptionPolicy.
 */
class AllAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    (void)method;
    return path.find("/api/v1/auth/") != 0;
  }

  std::string description() const override {
    return "AllAuth: Authentication required for all endpoints except /auth/*";
  }
};

/**
 * @brief Policy that requires authentication only for write operations
 *
 * Write operations: POST, PUT, DELETE, PATCH
 * Read operations: GET, HEAD, OPTIONS (no auth required)
 *
 * Auth endpoints are always public.
 */
class WriteOnlyAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    // Auth endpoints are always public
    if (path.find("/api/v1/auth/") == 0) {
      return false;
    }

    // Write operations require auth
    return method == "POST" || method == "PUT" || method == "DELETE" || method == "PATCH";
  }

  std::string description() const override {
    return "WriteOnly: Authentication required for POST, PUT, DELETE, PATCH operations";
  }
};

/**
 * @brief Policy with configurable public paths
 *
 * Allows specifying a list of paths that don't require authentication.
 * Supports wildcards (* for single segment, ** for multiple segments).
 */
class ConfigurableAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  /**
   * @brief Construct with list of public paths
   * @param public_paths Paths that don't require authentication (supports wildcards)
   * @param require_for_reads If true, require auth even for GET requests
   */
  explicit ConfigurableAuthRequirementPolicy(const std::vector<std::string> & public_paths,
                                             bool require_for_reads = false);

  /**
   * @brief Construct from auth requirements map
   * @param auth_requirements Map of path patterns to auth requirement levels
   */
  explicit ConfigurableAuthRequirementPolicy(
      const std::unordered_map<std::string, AuthRequirement> & auth_requirements);

  bool requires_authentication(const std::string & method, const std::string & path) const override;

  std::string description() const override {
    return "Configurable: Custom per-path authentication requirements";
  }

  /**
   * @brief Add a public path
   * @param path Path pattern (supports * and ** wildcards)
   */
  void add_public_path(const std::string & path);

 private:
  bool is_public_path(const std::string & path) const;
  AuthRequirement get_path_requirement(const std::string & path) const;
  static bool matches_path(const std::string & pattern, const std::string & path);

  std::vector<std::string> public_paths_;
  std::unordered_map<std::string, AuthRequirement> auth_requirements_;
  bool require_for_reads_;
  bool use_requirements_map_;
};

/// One entry of `auth.public_routes`: a method and a path this gateway answers
/// with no credential at all.
struct PublicRoute {
  std::string method;  ///< Upper-case HTTP method, e.g. "GET"
  std::string path;    ///< Full request path, e.g. "/api/v1/health"

  bool operator==(const PublicRoute & other) const {
    return method == other.method && path == other.path;
  }
};

/// Parses one `auth.public_routes` entry, written "METHOD /path".
///
/// Matching is exact and there are no wildcards, so an operator cannot open a
/// subtree by accident: every route that stops requiring a credential is a
/// line somebody wrote and a reviewer can read. `GET /api/v1/health` opens the
/// health probe and nothing else, where `GET /api/v1/*` would have opened the
/// whole read surface with one character.
///
/// @return the parsed route, or a message naming what is wrong with the entry.
tl::expected<PublicRoute, std::string> parse_public_route(const std::string & entry);

/// Parses a whole `auth.public_routes` list, dropping entries that do not
/// parse. Dropping keeps the route protected, which is the safe reading of a
/// malformed entry; GatewayNode validates the list first and refuses to start
/// rather than let a typo silently protect a route the operator wanted open.
std::vector<PublicRoute> parse_public_routes(const std::vector<std::string> & entries);

/**
 * @brief Layers an operator's `auth.public_routes` over another policy
 *
 * `require_auth_for` stays the primary axis; this only ever *removes* the
 * credential requirement, never adds one, so wrapping cannot make a gateway
 * stricter than the policy underneath and cannot be used to shadow it.
 *
 * The gateway ships with an empty list, which makes this a no-op: closed by
 * default, and open exactly where somebody said so.
 */
class PublicRouteExemptionPolicy : public IAuthRequirementPolicy {
 public:
  PublicRouteExemptionPolicy(std::unique_ptr<IAuthRequirementPolicy> inner, std::vector<PublicRoute> public_routes);

  bool requires_authentication(const std::string & method, const std::string & path) const override;

  std::string description() const override;

  /// The routes this layer exempts, in the order they were configured.
  const std::vector<PublicRoute> & public_routes() const {
    return public_routes_;
  }

 private:
  std::unique_ptr<IAuthRequirementPolicy> inner_;
  std::vector<PublicRoute> public_routes_;
};

/**
 * @brief Factory to create auth requirement policies from configuration
 */
class AuthRequirementPolicyFactory {
 public:
  /**
   * @brief Create policy from AuthRequirement enum
   * @param requirement The auth requirement level
   * @return Policy implementation
   */
  static std::unique_ptr<IAuthRequirementPolicy> create(AuthRequirement requirement);

  /**
   * @brief Create policy from AuthConfig
   * @param config Full auth configuration
   * @return Policy implementation based on config.enabled and config.auth_requirements
   */
  static std::unique_ptr<IAuthRequirementPolicy> create(const AuthConfig & config);

  /**
   * @brief Create policy for a requirement level, exempting configured routes
   * @param requirement The auth requirement level
   * @param public_routes Entries of `auth.public_routes`, already parsed
   * @return The requirement policy, wrapped only when the list is non-empty
   */
  static std::unique_ptr<IAuthRequirementPolicy> create(AuthRequirement requirement,
                                                        const std::vector<PublicRoute> & public_routes);
};

}  // namespace ros2_medkit_gateway
