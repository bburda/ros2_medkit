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

#include "ros2_medkit_gateway/core/auth/auth_requirement_policy.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

ConfigurableAuthRequirementPolicy::ConfigurableAuthRequirementPolicy(const std::vector<std::string> & public_paths,
                                                                     bool require_for_reads)
  : public_paths_(public_paths), require_for_reads_(require_for_reads), use_requirements_map_(false) {
  // Always add auth endpoints as public
  public_paths_.push_back("/api/v1/auth/**");
}

ConfigurableAuthRequirementPolicy::ConfigurableAuthRequirementPolicy(
    const std::unordered_map<std::string, AuthRequirement> & auth_requirements)
  : auth_requirements_(auth_requirements), require_for_reads_(false), use_requirements_map_(true) {
  // Always add auth endpoints as public
  auth_requirements_["/api/v1/auth/*"] = AuthRequirement::NONE;
}

bool ConfigurableAuthRequirementPolicy::requires_authentication(const std::string & method,
                                                                const std::string & path) const {
  if (use_requirements_map_) {
    AuthRequirement requirement = get_path_requirement(path);
    // NONE means no auth required
    return requirement != AuthRequirement::NONE;
  }

  // Original logic for public_paths mode
  // Check if this is a public path
  if (is_public_path(path)) {
    return false;
  }

  // If require_for_reads is false, GET/HEAD/OPTIONS don't need auth
  if (!require_for_reads_) {
    if (method == "GET" || method == "HEAD" || method == "OPTIONS") {
      return false;
    }
  }

  return true;
}

AuthRequirement ConfigurableAuthRequirementPolicy::get_path_requirement(const std::string & path) const {
  // Try exact match first
  auto it = auth_requirements_.find(path);
  if (it != auth_requirements_.end()) {
    return it->second;
  }

  // Find the longest matching pattern
  size_t longest_match_length = 0;
  AuthRequirement best_match = AuthRequirement::ALL;  // Default to requiring auth

  for (const auto & [pattern, requirement] : auth_requirements_) {
    if (matches_path(pattern, path) && pattern.size() > longest_match_length) {
      longest_match_length = pattern.size();
      best_match = requirement;
    }
  }

  return best_match;
}

void ConfigurableAuthRequirementPolicy::add_public_path(const std::string & path) {
  public_paths_.push_back(path);
}

bool ConfigurableAuthRequirementPolicy::is_public_path(const std::string & path) const {
  for (const auto & pattern : public_paths_) {
    if (matches_path(pattern, path)) {
      return true;
    }
  }
  return false;
}

bool ConfigurableAuthRequirementPolicy::matches_path(const std::string & pattern, const std::string & path) {
  // Exact match
  if (pattern == path) {
    return true;
  }

  // Convert pattern to regex
  std::string regex_pattern;
  regex_pattern.reserve(pattern.size() * 2);

  for (size_t i = 0; i < pattern.size(); ++i) {
    char c = pattern[i];
    if (c == '*') {
      // Check for ** (multi-segment wildcard)
      if (i + 1 < pattern.size() && pattern[i + 1] == '*') {
        regex_pattern += ".*";  // Match anything including slashes
        ++i;                    // Skip the second *
      } else {
        regex_pattern += "[^/]+";  // Match any non-slash characters (single segment)
      }
    } else {
      switch (c) {
        case '.':
        case '[':
        case ']':
        case '(':
        case ')':
        case '{':
        case '}':
        case '\\':
        case '^':
        case '$':
        case '|':
        case '?':
        case '+':
          regex_pattern += '\\';
          regex_pattern += c;
          break;
        default:
          regex_pattern += c;
      }
    }
  }

  // Anchor the pattern
  regex_pattern = "^" + regex_pattern + "$";

  try {
    std::regex re(regex_pattern);
    return std::regex_match(path, re);
  } catch (const std::regex_error &) {
    return false;
  }
}

std::unique_ptr<IAuthRequirementPolicy> AuthRequirementPolicyFactory::create(AuthRequirement requirement) {
  switch (requirement) {
    case AuthRequirement::NONE:
      return std::make_unique<NoAuthRequirementPolicy>();

    case AuthRequirement::WRITE:
      return std::make_unique<WriteOnlyAuthRequirementPolicy>();

    case AuthRequirement::ALL:
      return std::make_unique<AllAuthRequirementPolicy>();

    default:
      return std::make_unique<NoAuthRequirementPolicy>();
  }
}

std::unique_ptr<IAuthRequirementPolicy> AuthRequirementPolicyFactory::create(const AuthConfig & config) {
  // If auth is disabled, return NoAuth policy
  if (!config.enabled) {
    return std::make_unique<NoAuthRequirementPolicy>();
  }

  return create(config.require_auth_for, parse_public_routes(config.public_routes));
}

std::vector<PublicRoute> parse_public_routes(const std::vector<std::string> & entries) {
  std::vector<PublicRoute> routes;
  routes.reserve(entries.size());
  for (const auto & entry : entries) {
    auto parsed = parse_public_route(entry);
    if (parsed.has_value()) {
      routes.push_back(*parsed);
    }
  }
  return routes;
}

std::unique_ptr<IAuthRequirementPolicy>
AuthRequirementPolicyFactory::create(AuthRequirement requirement, const std::vector<PublicRoute> & public_routes) {
  auto policy = create(requirement);
  if (public_routes.empty()) {
    return policy;
  }
  return std::make_unique<PublicRouteExemptionPolicy>(std::move(policy), public_routes);
}

tl::expected<PublicRoute, std::string> parse_public_route(const std::string & entry) {
  auto is_space = [](unsigned char c) {
    return std::isspace(c) != 0;
  };
  auto begin = std::find_if_not(entry.begin(), entry.end(), is_space);
  auto end = std::find_if_not(entry.rbegin(), entry.rend(), is_space).base();
  const std::string trimmed = (begin < end) ? std::string(begin, end) : std::string();

  if (trimmed.empty()) {
    return tl::unexpected<std::string>("entry is empty");
  }

  const size_t space = trimmed.find(' ');
  if (space == std::string::npos) {
    return tl::unexpected<std::string>("expected \"METHOD /path\", e.g. \"GET /api/v1/health\"");
  }

  PublicRoute route;
  route.method = trimmed.substr(0, space);
  route.path = trimmed.substr(space + 1);

  // The path is compared against what cpp-httplib hands the middleware, which
  // is a single token. A second space means two paths or a stray argument, and
  // either way the entry does not describe one route.
  if (route.path.find(' ') != std::string::npos) {
    return tl::unexpected<std::string>("path contains a space: \"" + route.path + "\"");
  }

  std::transform(route.method.begin(), route.method.end(), route.method.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  static const std::vector<std::string> kMethods = {"GET", "HEAD", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"};
  if (std::find(kMethods.begin(), kMethods.end(), route.method) == kMethods.end()) {
    return tl::unexpected<std::string>("unknown HTTP method \"" + route.method + "\"");
  }

  if (route.path.empty() || route.path.front() != '/') {
    return tl::unexpected<std::string>("path must start with \"/\": \"" + route.path + "\"");
  }

  // Wildcards are refused rather than ignored. Accepting the character and
  // then matching it literally would read as "this opens the subtree" and
  // silently open nothing; refusing says so while the operator is watching.
  if (route.path.find('*') != std::string::npos) {
    return tl::unexpected<std::string>("wildcards are not supported; name each route exactly: \"" + route.path + "\"");
  }

  return route;
}

PublicRouteExemptionPolicy::PublicRouteExemptionPolicy(std::unique_ptr<IAuthRequirementPolicy> inner,
                                                       std::vector<PublicRoute> public_routes)
  : inner_(std::move(inner)), public_routes_(std::move(public_routes)) {
}

bool PublicRouteExemptionPolicy::requires_authentication(const std::string & method, const std::string & path) const {
  for (const auto & route : public_routes_) {
    if (route.method == method && route.path == path) {
      return false;
    }
  }
  return inner_->requires_authentication(method, path);
}

std::string PublicRouteExemptionPolicy::description() const {
  std::string desc = inner_->description() + "; public_routes:";
  for (const auto & route : public_routes_) {
    desc += " " + route.method + " " + route.path;
  }
  return desc;
}

}  // namespace ros2_medkit_gateway
