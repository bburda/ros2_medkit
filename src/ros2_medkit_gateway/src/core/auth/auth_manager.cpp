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

#include "ros2_medkit_gateway/core/auth/auth_manager.hpp"

#include <jwt-cpp/jwt.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <regex>
#include <set>
#include <sstream>

namespace ros2_medkit_gateway {

// Helper to read file contents
static std::string read_file_contents(const std::string & path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

// Helper to validate RSA key file exists and is readable
static void validate_key_file(const std::string & path, const std::string & key_type) {
  if (path.empty()) {
    throw std::runtime_error(key_type + " path is empty");
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open " + key_type + " file: " + path);
  }
  // Check file is not empty
  file.seekg(0, std::ios::end);
  if (file.tellg() == 0) {
    throw std::runtime_error(key_type + " file is empty: " + path);
  }
}

AuthManager::AuthManager(const AuthConfig & config) : config_(config) {
  // Validate RS256 key files at startup (fail fast)
  if (config_.enabled && config_.jwt_algorithm == JwtAlgorithm::RS256) {
    validate_key_file(config_.jwt_secret, "RS256 private key");
    validate_key_file(config_.jwt_public_key, "RS256 public key");
  }

  // Initialize clients from config
  for (const auto & client : config_.clients) {
    clients_[client.client_id] = client;
  }

  // Create auth requirement policy from config
  auth_policy_ = AuthRequirementPolicyFactory::create(config_.require_auth_for);
}

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::authenticate(const std::string & client_id,
                                                                         const std::string & client_secret) {
  // Find client
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Unknown client_id"));
  }

  const auto & client = it->second;

  // Check if client is enabled
  if (!client.enabled) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Client is disabled"));
  }

  // Verify secret
  if (client.client_secret != client_secret) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Invalid client_secret"));
  }

  // Generate tokens
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  // Generate refresh token first
  std::string refresh_token_id = generate_token_id();
  JwtClaims refresh_claims;
  refresh_claims.iss = config_.issuer;
  refresh_claims.sub = client_id;
  refresh_claims.iat = now_ts;
  refresh_claims.exp = now_ts + config_.refresh_token_expiry_seconds;
  refresh_claims.jti = refresh_token_id;
  refresh_claims.typ = TokenType::REFRESH;  // Mark as refresh token
  refresh_claims.role = client.role;

  std::string refresh_token = generate_jwt(refresh_claims);

  // Store refresh token record
  RefreshTokenRecord refresh_record;
  refresh_record.token_id = refresh_token_id;
  refresh_record.client_id = client_id;
  refresh_record.role = client.role;
  refresh_record.issued_at = now_ts;
  refresh_record.expires_at = refresh_claims.exp;
  refresh_record.revoked = false;
  store_refresh_token(refresh_record);

  // Generate access token
  JwtClaims access_claims;
  access_claims.iss = config_.issuer;
  access_claims.sub = client_id;
  access_claims.iat = now_ts;
  access_claims.exp = now_ts + config_.token_expiry_seconds;
  access_claims.jti = generate_token_id();
  access_claims.typ = TokenType::ACCESS;  // Mark as access token
  access_claims.role = client.role;
  access_claims.refresh_token_id = refresh_token_id;

  std::string access_token = generate_jwt(access_claims);

  // Build response
  TokenResponse response;
  response.access_token = access_token;
  response.token_type = "Bearer";
  response.expires_in = config_.token_expiry_seconds;
  response.refresh_token = refresh_token;
  response.scope = role_to_string(client.role);

  return response;
}

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::refresh_access_token(const std::string & refresh_token) {
  // Decode and validate refresh token
  auto decode_result = decode_jwt(refresh_token);
  if (!decode_result) {
    return tl::unexpected(AuthErrorResponse::invalid_grant(decode_result.error()));
  }

  const auto & claims = decode_result.value();

  // Verify this is actually a refresh token, not an access token
  if (claims.typ != TokenType::REFRESH) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Token is not a refresh token"));
  }

  // Check if refresh token exists and is not revoked
  auto record = get_refresh_token(claims.jti);
  if (!record.has_value()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token not found"));
  }

  if (record->revoked) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has been revoked"));
  }

  // Check expiration
  if (claims.is_expired()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has expired"));
  }

  // Get client to check if still enabled
  auto client = get_client(claims.sub);
  if (!client.has_value()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Client no longer exists"));
  }

  if (!client->enabled) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Client is disabled"));
  }

  // Generate new access token
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  JwtClaims access_claims;
  access_claims.iss = config_.issuer;
  access_claims.sub = claims.sub;
  access_claims.iat = now_ts;
  access_claims.exp = now_ts + config_.token_expiry_seconds;
  access_claims.jti = generate_token_id();
  access_claims.typ = TokenType::ACCESS;  // Mark as access token
  access_claims.role = record->role;      // Use role from refresh token record
  access_claims.refresh_token_id = claims.jti;

  std::string access_token = generate_jwt(access_claims);

  // Build response (no new refresh token on refresh)
  TokenResponse response;
  response.access_token = access_token;
  response.token_type = "Bearer";
  response.expires_in = config_.token_expiry_seconds;
  response.scope = role_to_string(record->role);
  // Return the existing refresh token so clients can continue to use it
  response.refresh_token = refresh_token;

  return response;
}

TokenValidationResult AuthManager::validate_token(const std::string & token, TokenType expected_type) const {
  TokenValidationResult result;

  auto decode_result = decode_jwt(token);
  if (!decode_result) {
    result.valid = false;
    result.error = decode_result.error();
    return result;
  }

  const auto & claims = decode_result.value();

  // Check token type matches expected
  if (claims.typ != expected_type) {
    result.valid = false;
    result.error = "Invalid token type: expected " + token_type_to_string(expected_type) + ", got " +
                   token_type_to_string(claims.typ);
    return result;
  }

  // Check expiration
  if (claims.is_expired()) {
    result.valid = false;
    result.error = "Token has expired";
    return result;
  }

  // Check if client is still enabled (security: validate on every request)
  auto client = get_client(claims.sub);
  if (!client.has_value()) {
    result.valid = false;
    result.error = "Client no longer exists";
    return result;
  }
  if (!client->enabled) {
    result.valid = false;
    result.error = "Client has been disabled";
    return result;
  }

  // Check if associated refresh token is revoked (for access tokens)
  if (claims.refresh_token_id.has_value()) {
    auto record = get_refresh_token(claims.refresh_token_id.value());
    if (record.has_value() && record->revoked) {
      result.valid = false;
      result.error = "Associated refresh token has been revoked";
      return result;
    }
  }

  result.valid = true;
  result.claims = claims;
  return result;
}

void AuthManager::add_route_permissions(const RoutePermissions & permissions) {
  for (const auto & [role, entries] : permissions) {
    permissions_[role].insert(entries.begin(), entries.end());
  }
}

AuthorizationResult AuthManager::check_authorization(UserRole role, const std::string & method,
                                                     const std::string & path) const {
  AuthorizationResult result;

  auto it = permissions_.find(role);
  if (it == permissions_.end()) {
    result.authorized = false;
    // Not "unknown role" any more: every enumerator is a role the gateway
    // knows, and reaching here means the table was never given entries for it
    // (see add_route_permissions). Saying "unknown role" would send a reader
    // hunting for a typo in the token instead of a gap in the table.
    result.error = "No permissions are configured for this role";
    return result;
  }

  const auto & permissions = it->second;
  std::string permission_key = method + ":" + path;

  // Check exact match first
  if (permissions.count(permission_key) > 0) {
    result.authorized = true;
    return result;
  }

  // Check wildcard patterns
  for (const auto & pattern : permissions) {
    // Extract method and path pattern
    size_t colon_pos = pattern.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string pattern_method = pattern.substr(0, colon_pos);
    std::string pattern_path = pattern.substr(colon_pos + 1);

    // Method must match exactly
    if (pattern_method != method) {
      continue;
    }

    // Check path pattern
    if (matches_path(pattern_path, path)) {
      result.authorized = true;
      return result;
    }
  }

  result.authorized = false;
  result.error = "Insufficient permissions";
  result.required_permission = permission_key;
  return result;
}

bool AuthManager::requires_authentication(const std::string & method, const std::string & path) const {
  if (!config_.enabled) {
    return false;
  }

  // Delegate to policy
  return auth_policy_->requires_authentication(method, path);
}

bool AuthManager::revoke_refresh_token(const std::string & refresh_token) {
  // Decode token to get the jti
  auto decode_result = decode_jwt(refresh_token);
  if (!decode_result) {
    return false;
  }

  const auto & claims = decode_result.value();

  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  auto it = refresh_tokens_.find(claims.jti);
  if (it == refresh_tokens_.end()) {
    return false;
  }

  it->second.revoked = true;
  return true;
}

size_t AuthManager::cleanup_expired_tokens() {
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  size_t count = 0;

  for (auto it = refresh_tokens_.begin(); it != refresh_tokens_.end();) {
    if (it->second.expires_at < now_ts) {
      it = refresh_tokens_.erase(it);
      ++count;
    } else {
      ++it;
    }
  }

  return count;
}

bool AuthManager::register_client(const std::string & client_id, const std::string & client_secret, UserRole role) {
  std::lock_guard<std::mutex> lock(clients_mutex_);

  if (clients_.count(client_id) > 0) {
    return false;
  }

  ClientCredentials creds;
  creds.client_id = client_id;
  creds.client_secret = client_secret;
  creds.role = role;
  creds.enabled = true;

  clients_[client_id] = creds;
  return true;
}

std::optional<ClientCredentials> AuthManager::get_client(const std::string & client_id) const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool AuthManager::disable_client(const std::string & client_id) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return false;
  }
  it->second.enabled = false;
  return true;
}

bool AuthManager::enable_client(const std::string & client_id) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return false;
  }
  it->second.enabled = true;
  return true;
}

std::string AuthManager::generate_jwt(const JwtClaims & claims) const {
  auto builder = jwt::create()
                     .set_type(token_type_to_string(claims.typ))  // Set typ in JWT header
                     .set_issuer(claims.iss)
                     .set_subject(claims.sub)
                     .set_issued_at(std::chrono::system_clock::from_time_t(claims.iat))
                     .set_expires_at(std::chrono::system_clock::from_time_t(claims.exp))
                     .set_id(claims.jti)
                     .set_payload_claim("role", jwt::claim(role_to_string(claims.role)));

  if (!claims.permissions.empty()) {
    // Convert vector to set for jwt-cpp
    std::set<std::string> perms_set(claims.permissions.begin(), claims.permissions.end());
    builder.set_payload_claim("permissions", jwt::claim(perms_set));
  }

  if (claims.refresh_token_id.has_value()) {
    builder.set_payload_claim("refresh_token_id", jwt::claim(claims.refresh_token_id.value()));
  }

  // Sign based on algorithm
  switch (config_.jwt_algorithm) {
    case JwtAlgorithm::HS256:
      return builder.sign(jwt::algorithm::hs256{config_.jwt_secret});

    case JwtAlgorithm::RS256: {
      std::string private_key = read_file_contents(config_.jwt_secret);
      return builder.sign(jwt::algorithm::rs256("", private_key, "", ""));
    }

    default:
      throw std::runtime_error("Unsupported JWT algorithm");
  }
}

tl::expected<JwtClaims, std::string> AuthManager::decode_jwt(const std::string & token) const {
  try {
    // Decode token first
    auto decoded = jwt::decode(token);

    // Verify signature
    try {
      switch (config_.jwt_algorithm) {
        case JwtAlgorithm::HS256: {
          auto verifier =
              jwt::verify().allow_algorithm(jwt::algorithm::hs256{config_.jwt_secret}).with_issuer(config_.issuer);
          verifier.verify(decoded);
          break;
        }

        case JwtAlgorithm::RS256: {
          std::string public_key = read_file_contents(config_.jwt_public_key);
          auto verifier =
              jwt::verify().allow_algorithm(jwt::algorithm::rs256(public_key, "", "", "")).with_issuer(config_.issuer);
          verifier.verify(decoded);
          break;
        }

        default:
          return tl::unexpected("Unsupported JWT algorithm");
      }
    } catch (const jwt::error::token_verification_exception & e) {
      return tl::unexpected("Token verification failed: " + std::string(e.what()));
    }

    // Extract claims
    JwtClaims claims;
    claims.iss = decoded.get_issuer();
    claims.sub = decoded.get_subject();
    claims.jti = decoded.get_id();

    // Extract typ from header
    if (decoded.has_type()) {
      try {
        claims.typ = string_to_token_type(decoded.get_type());
      } catch (const std::invalid_argument &) {
        claims.typ = TokenType::ACCESS;  // Default for backward compatibility
      }
    }

    auto exp_claim = decoded.get_expires_at();
    claims.exp = std::chrono::duration_cast<std::chrono::seconds>(exp_claim.time_since_epoch()).count();

    auto iat_claim = decoded.get_issued_at();
    claims.iat = std::chrono::duration_cast<std::chrono::seconds>(iat_claim.time_since_epoch()).count();

    if (decoded.has_payload_claim("role")) {
      claims.role = string_to_role(decoded.get_payload_claim("role").as_string());
    }

    if (decoded.has_payload_claim("permissions")) {
      auto perms = decoded.get_payload_claim("permissions").as_array();
      for (const auto & p : perms) {
        claims.permissions.push_back(p.get<std::string>());
      }
    }

    if (decoded.has_payload_claim("refresh_token_id")) {
      claims.refresh_token_id = decoded.get_payload_claim("refresh_token_id").as_string();
    }

    return claims;
  } catch (const std::exception & e) {
    return tl::unexpected("JWT decode error: " + std::string(e.what()));
  }
}

std::string AuthManager::generate_token_id() {
  // Generate UUID-like string using a per-call RNG seeded from std::random_device
  // to avoid reuse of time-based seeds and thread-local initialization issues.
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dis;

  std::stringstream ss;
  ss << std::hex << std::setfill('0');

  uint64_t part1 = dis(gen);
  uint64_t part2 = dis(gen);

  ss << std::setw(8) << (part1 >> 32) << "-";
  ss << std::setw(4) << ((part1 >> 16) & 0xFFFF) << "-";
  ss << std::setw(4) << (part1 & 0xFFFF) << "-";
  ss << std::setw(4) << (part2 >> 48) << "-";
  ss << std::setw(12) << (part2 & 0xFFFFFFFFFFFF);

  return ss.str();
}

bool AuthManager::matches_path(const std::string & pattern, const std::string & path) {
  // Simple wildcard matching
  // * matches any single path segment
  // ** matches any number of path segments (including none)
  // Pattern: /api/v1/components/*/data
  // Path: /api/v1/components/engine/data
  // Pattern: /api/v1/**
  // Path: /api/v1/components/engine/data/temperature (matches)

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
    std::regex compiled(regex_pattern);
    return std::regex_match(path, compiled);
  } catch (const std::regex_error &) {
    // If regex compilation fails, treat as non-match
    return false;
  }
}

void AuthManager::store_refresh_token(const RefreshTokenRecord & record) {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  refresh_tokens_[record.token_id] = record;
}

std::optional<RefreshTokenRecord> AuthManager::get_refresh_token(const std::string & token_id) const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  auto it = refresh_tokens_.find(token_id);
  if (it == refresh_tokens_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace ros2_medkit_gateway
