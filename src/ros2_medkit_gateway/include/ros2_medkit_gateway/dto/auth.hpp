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

#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// AuthCredentials - request body for POST /auth/authorize and POST /auth/token.
//
// Wire shape (from AuthorizeRequest::from_json + AuthorizeRequest::from_form_data):
//   grant_type    - OAuth2 grant type (required)
//                   "client_credentials" for /auth/authorize,
//                   "refresh_token" for /auth/token
//   client_id     - client identifier (optional; required by handler for
//                   client_credentials flow)
//   client_secret - client secret (optional; required by handler for
//                   client_credentials flow)
//   refresh_token - refresh token string (optional; required by handler for
//                   refresh_token flow)
//   scope         - requested scope / role (optional)
//
// grant_type is NOT field_enum: the handler performs bespoke grant_type
// validation with an OAuth2 "unsupported_grant_type" error response that
// is distinct from the SOVD GenericError format used by parse_body.
// =============================================================================
struct AuthCredentials {
  std::string grant_type;
  std::optional<std::string> client_id;
  std::optional<std::string> client_secret;
  std::optional<std::string> refresh_token;
  std::optional<std::string> scope;
};

template <>
inline constexpr auto dto_fields<AuthCredentials> =
    std::make_tuple(field("grant_type", &AuthCredentials::grant_type), field("client_id", &AuthCredentials::client_id),
                    field("client_secret", &AuthCredentials::client_secret),
                    field("refresh_token", &AuthCredentials::refresh_token), field("scope", &AuthCredentials::scope));

template <>
inline constexpr std::string_view dto_name<AuthCredentials> = "AuthCredentials";

// =============================================================================
// AuthTokenResponse - success response for POST /auth/authorize and
// POST /auth/token.
//
// Wire shape (from TokenResponse::to_json in auth_models.hpp):
//   access_token  - JWT access token string (required)
//   token_type    - always "Bearer" (required)
//   expires_in    - seconds until access token expires (required, integer)
//   scope         - role-based scope string (required)
//   refresh_token - JWT refresh token string (optional; absent on token refresh
//                   if the existing refresh token is reused)
// =============================================================================
struct AuthTokenResponse {
  std::string access_token;
  std::string token_type;
  int expires_in{0};
  std::string scope;
  std::optional<std::string> refresh_token;
};

template <>
inline constexpr auto dto_fields<AuthTokenResponse> =
    std::make_tuple(field("access_token", &AuthTokenResponse::access_token),
                    field("token_type", &AuthTokenResponse::token_type),
                    field("expires_in", &AuthTokenResponse::expires_in), field("scope", &AuthTokenResponse::scope),
                    field("refresh_token", &AuthTokenResponse::refresh_token));

template <>
inline constexpr std::string_view dto_name<AuthTokenResponse> = "AuthTokenResponse";

// =============================================================================
// AuthRevokeRequest - request body for POST /auth/revoke (RFC 7009).
//
// Wire shape:
//   token            - the token to revoke (required)
//   token_type_hint  - "access_token" or "refresh_token" (optional). RFC 7009
//                      lets clients hint the token type so the server can short-
//                      circuit lookup; the gateway currently always treats the
//                      token as a refresh token, but the field is parsed for
//                      forward compatibility.
//
// Like AuthCredentials, this DTO is parsed manually in the handler so the route
// can emit the OAuth2 `invalid_request` code (underscore) rather than the SOVD
// `invalid-request` code (dash) the framework's auto-parser would produce.
// =============================================================================
struct AuthRevokeRequest {
  std::string token;
  std::optional<std::string> token_type_hint;
};

template <>
inline constexpr auto dto_fields<AuthRevokeRequest> = std::make_tuple(
    field("token", &AuthRevokeRequest::token), field("token_type_hint", &AuthRevokeRequest::token_type_hint));

template <>
inline constexpr std::string_view dto_name<AuthRevokeRequest> = "AuthRevokeRequest";

// =============================================================================
// AuthRevokeResponse - response body for POST /auth/revoke.
//
// RFC 7009 §2.2 mandates an empty response body, but the existing gateway wire
// format ships `{"status": "revoked"}` for parity with the SOVD status pattern.
// Integration tests assert on this field; preserve it byte-for-byte.
// =============================================================================
struct AuthRevokeResponse {
  std::string status;
};

template <>
inline constexpr auto dto_fields<AuthRevokeResponse> = std::make_tuple(field("status", &AuthRevokeResponse::status));

template <>
inline constexpr std::string_view dto_name<AuthRevokeResponse> = "AuthRevokeResponse";

// =============================================================================
// OAuth2Error - RFC 6749 section 5.2 error body. Deliberately NOT the SOVD
// GenericError shape.
//
// Two emitters put these exact two keys on the wire, and both used to be
// documented as a GenericError:
//   * the /auth/* routes, via `write_oauth2_error`
//     (core/http/detail/primitives.cpp), selected by
//     `.error_renderer(kOAuth2Error)` on the registration; and
//   * AuthMiddleware, whose 401 (`invalid_token`) and 403
//     (`insufficient_scope`) both serialise `AuthErrorResponse::to_json()`
//     = {error, error_description} ahead of any handler, on every route.
//
// So this shape is not confined to the auth endpoints: with authentication on,
// it is what any route answers when the token is missing or under-scoped.
// =============================================================================
// `error_description` is required, not optional. RFC 6749 leaves it optional,
// but both emitters set it unconditionally - `write_oauth2_error` from
// `ErrorInfo::message`, `AuthErrorResponse::to_json()` from its own member - so
// declaring it optional would tell a client to handle an absence this gateway
// cannot produce.
struct OAuth2Error {
  std::string error;
  std::string error_description;
};

template <>
inline constexpr auto dto_fields<OAuth2Error> =
    std::make_tuple(field("error", &OAuth2Error::error,
                          "RFC 6749 error code, e.g. `invalid_grant`, `invalid_request`, `unsupported_grant_type`, "
                          "`invalid_token`, `insufficient_scope`, `server_error`."),
                    field("error_description", &OAuth2Error::error_description,
                          "Human-readable cause. Always present, though RFC 6749 permits its absence."));

template <>
inline constexpr std::string_view dto_name<OAuth2Error> = "OAuth2Error";

}  // namespace dto
}  // namespace ros2_medkit_gateway
