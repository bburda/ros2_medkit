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

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/core/auth/auth.hpp"
#include "ros2_medkit_gateway/core/config.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Residual route permissions
// =============================================================================
//
// What is left in AuthConfig after the per-role table moved to the
// registrations. These tests are about the residual list itself - what it
// covers and, more usefully, what it deliberately does not. The gateway's real
// per-route grants are derived by `RouteRegistry::route_permissions()` and
// checked end to end against enforcement in `test_rbac_contract.test.py`;
// asserting them here would only restate the derivation's input.

TEST(AuthConfigResidualPermissionsTest, AdminWildcardsCoverEveryMethod) {
  // @verifies REQ_INTEROP_086
  const auto & permissions = AuthConfig::residual_route_permissions();

  auto it = permissions.find(UserRole::ADMIN);
  ASSERT_NE(it, permissions.end());

  const auto & admin_perms = it->second;
  EXPECT_TRUE(admin_perms.count("GET:/api/v1/**") > 0);
  EXPECT_TRUE(admin_perms.count("POST:/api/v1/**") > 0);
  EXPECT_TRUE(admin_perms.count("PUT:/api/v1/**") > 0);
  EXPECT_TRUE(admin_perms.count("DELETE:/api/v1/**") > 0);
}

TEST(AuthConfigResidualPermissionsTest, NoRoleBelowAdminHasResidualEntries) {
  // The residual list exists for routes mounted outside the RouteRegistry -
  // plugin routes above all. Granting one of those to a weaker role would hand
  // that role every route whichever plugins the deployment happens to load,
  // sight unseen, so the list stops at ADMIN. This is also why a plugin
  // operation's published role reads `admin`.
  const auto & permissions = AuthConfig::residual_route_permissions();

  EXPECT_EQ(permissions.count(UserRole::VIEWER), 0u);
  EXPECT_EQ(permissions.count(UserRole::OPERATOR), 0u);
  EXPECT_EQ(permissions.count(UserRole::CONFIGURATOR), 0u);
}

TEST(AuthConfigResidualPermissionsTest, CarriesNoPerRouteEntries) {
  // The old literal table lived here and listed every gateway route by hand.
  // It is gone, and this pins that it stays gone: a per-route entry added back
  // here would be a second source for a grant the registration already
  // declares, and the two would drift.
  const auto & permissions = AuthConfig::residual_route_permissions();

  ASSERT_EQ(permissions.size(), 1u);
  EXPECT_EQ(permissions.at(UserRole::ADMIN).size(), 4u);
}

// =============================================================================
// AuthConfigBuilder additional tests
// =============================================================================

TEST(AuthConfigBuilderExtendedTest, NegativeTokenExpiryThrows) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("secret_key_with_at_least_32_characters")
            .with_token_expiry(-100)
            .with_refresh_token_expiry(86400)
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderExtendedTest, ZeroTokenExpiryThrows) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("secret_key_with_at_least_32_characters")
            .with_token_expiry(0)
            .with_refresh_token_expiry(86400)
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderExtendedTest, ZeroRefreshExpiryThrows) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("secret_key_with_at_least_32_characters")
            .with_token_expiry(3600)
            .with_refresh_token_expiry(0)
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderExtendedTest, ShortHS256SecretThrows) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("short")  // Less than 32 chars
            .with_algorithm(JwtAlgorithm::HS256)
            .with_token_expiry(3600)
            .with_refresh_token_expiry(86400)
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderExtendedTest, RS256RequiresPublicKey) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("/path/to/private.pem")
            .with_algorithm(JwtAlgorithm::RS256)
            // No public key set
            .with_token_expiry(3600)
            .with_refresh_token_expiry(86400)
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderExtendedTest, RS256WithPublicKeySucceeds) {
  // Note: This only validates config building, not actual key validation
  // Key validation happens at AuthManager construction
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("/path/to/private.pem")
                    .with_jwt_public_key("/path/to/public.pem")
                    .with_algorithm(JwtAlgorithm::RS256)
                    .with_token_expiry(3600)
                    .with_refresh_token_expiry(86400)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.jwt_algorithm, JwtAlgorithm::RS256);
  EXPECT_EQ(config.jwt_public_key, "/path/to/public.pem");
}

TEST(AuthConfigBuilderExtendedTest, AddMultipleClients) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("secret_key_with_at_least_32_characters")
                    .with_token_expiry(3600)
                    .with_refresh_token_expiry(86400)
                    .add_client("admin", "admin_pass", UserRole::ADMIN)
                    .add_client("viewer1", "viewer1_pass", UserRole::VIEWER)
                    .add_client("viewer2", "viewer2_pass", UserRole::VIEWER)
                    .add_client("operator", "operator_pass", UserRole::OPERATOR)
                    .build();

  EXPECT_EQ(config.clients.size(), 4);

  // Verify each client
  EXPECT_EQ(config.clients[0].client_id, "admin");
  EXPECT_EQ(config.clients[0].role, UserRole::ADMIN);
  EXPECT_TRUE(config.clients[0].enabled);

  EXPECT_EQ(config.clients[1].client_id, "viewer1");
  EXPECT_EQ(config.clients[1].role, UserRole::VIEWER);

  EXPECT_EQ(config.clients[2].client_id, "viewer2");
  EXPECT_EQ(config.clients[2].role, UserRole::VIEWER);

  EXPECT_EQ(config.clients[3].client_id, "operator");
  EXPECT_EQ(config.clients[3].role, UserRole::OPERATOR);
}

TEST(AuthConfigBuilderExtendedTest, SetIssuer) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("secret_key_with_at_least_32_characters")
                    .with_token_expiry(3600)
                    .with_refresh_token_expiry(86400)
                    .with_issuer("my-gateway-issuer")
                    .build();

  EXPECT_EQ(config.issuer, "my-gateway-issuer");
}

TEST(AuthConfigBuilderExtendedTest, DefaultIssuer) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("secret_key_with_at_least_32_characters")
                    .with_token_expiry(3600)
                    .with_refresh_token_expiry(86400)
                    .build();

  EXPECT_EQ(config.issuer, "ros2_medkit_gateway");  // Default value
}

TEST(AuthConfigBuilderExtendedTest, AllAuthRequirementModes) {
  // Test NONE
  auto config_none = AuthConfigBuilder()
                         .with_enabled(true)
                         .with_jwt_secret("secret_key_with_at_least_32_characters")
                         .with_require_auth_for(AuthRequirement::NONE)
                         .build();
  EXPECT_EQ(config_none.require_auth_for, AuthRequirement::NONE);

  // Test WRITE
  auto config_write = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("secret_key_with_at_least_32_characters")
                          .with_require_auth_for(AuthRequirement::WRITE)
                          .build();
  EXPECT_EQ(config_write.require_auth_for, AuthRequirement::WRITE);

  // Test ALL
  auto config_all = AuthConfigBuilder()
                        .with_enabled(true)
                        .with_jwt_secret("secret_key_with_at_least_32_characters")
                        .with_require_auth_for(AuthRequirement::ALL)
                        .build();
  EXPECT_EQ(config_all.require_auth_for, AuthRequirement::ALL);
}

// =============================================================================
// ClientCredentials tests
// =============================================================================

TEST(ClientCredentialsTest, DefaultValues) {
  ClientCredentials creds;

  EXPECT_TRUE(creds.client_id.empty());
  EXPECT_TRUE(creds.client_secret.empty());
  EXPECT_EQ(creds.role, UserRole::VIEWER);
  EXPECT_TRUE(creds.enabled);
}

TEST(ClientCredentialsTest, SetValues) {
  ClientCredentials creds;
  creds.client_id = "test_client";
  creds.client_secret = "test_secret";
  creds.role = UserRole::ADMIN;
  creds.enabled = false;

  EXPECT_EQ(creds.client_id, "test_client");
  EXPECT_EQ(creds.client_secret, "test_secret");
  EXPECT_EQ(creds.role, UserRole::ADMIN);
  EXPECT_FALSE(creds.enabled);
}

// =============================================================================
// Conversion function edge cases
// =============================================================================

TEST(ConversionFunctionsTest, RoleToStringAll) {
  EXPECT_EQ(role_to_string(UserRole::VIEWER), "viewer");
  EXPECT_EQ(role_to_string(UserRole::OPERATOR), "operator");
  EXPECT_EQ(role_to_string(UserRole::CONFIGURATOR), "configurator");
  EXPECT_EQ(role_to_string(UserRole::ADMIN), "admin");
}

TEST(ConversionFunctionsTest, StringToRoleMixedCase) {
  EXPECT_EQ(string_to_role("VIEWER"), UserRole::VIEWER);
  EXPECT_EQ(string_to_role("Viewer"), UserRole::VIEWER);
  EXPECT_EQ(string_to_role("vIeWeR"), UserRole::VIEWER);

  EXPECT_EQ(string_to_role("OPERATOR"), UserRole::OPERATOR);
  EXPECT_EQ(string_to_role("Operator"), UserRole::OPERATOR);

  EXPECT_EQ(string_to_role("CONFIGURATOR"), UserRole::CONFIGURATOR);
  EXPECT_EQ(string_to_role("Admin"), UserRole::ADMIN);
}

TEST(ConversionFunctionsTest, StringToRoleInvalidValues) {
  EXPECT_THROW(string_to_role(""), std::invalid_argument);
  EXPECT_THROW(string_to_role("superuser"), std::invalid_argument);
  EXPECT_THROW(string_to_role("root"), std::invalid_argument);
  EXPECT_THROW(string_to_role("guest"), std::invalid_argument);
}

TEST(ConversionFunctionsTest, AlgorithmToStringAll) {
  EXPECT_EQ(algorithm_to_string(JwtAlgorithm::HS256), "HS256");
  EXPECT_EQ(algorithm_to_string(JwtAlgorithm::RS256), "RS256");
}

TEST(ConversionFunctionsTest, StringToAlgorithmMixedCase) {
  EXPECT_EQ(string_to_algorithm("hs256"), JwtAlgorithm::HS256);
  EXPECT_EQ(string_to_algorithm("HS256"), JwtAlgorithm::HS256);
  EXPECT_EQ(string_to_algorithm("Hs256"), JwtAlgorithm::HS256);

  EXPECT_EQ(string_to_algorithm("rs256"), JwtAlgorithm::RS256);
  EXPECT_EQ(string_to_algorithm("RS256"), JwtAlgorithm::RS256);
}

TEST(ConversionFunctionsTest, StringToAlgorithmInvalidValues) {
  EXPECT_THROW(string_to_algorithm(""), std::invalid_argument);
  EXPECT_THROW(string_to_algorithm("HS384"), std::invalid_argument);
  EXPECT_THROW(string_to_algorithm("RS384"), std::invalid_argument);
  EXPECT_THROW(string_to_algorithm("ES256"), std::invalid_argument);
  EXPECT_THROW(string_to_algorithm("none"), std::invalid_argument);
}

TEST(ConversionFunctionsTest, StringToAuthRequirementMixedCase) {
  EXPECT_EQ(string_to_auth_requirement("NONE"), AuthRequirement::NONE);
  EXPECT_EQ(string_to_auth_requirement("None"), AuthRequirement::NONE);
  EXPECT_EQ(string_to_auth_requirement("none"), AuthRequirement::NONE);

  EXPECT_EQ(string_to_auth_requirement("WRITE"), AuthRequirement::WRITE);
  EXPECT_EQ(string_to_auth_requirement("Write"), AuthRequirement::WRITE);

  EXPECT_EQ(string_to_auth_requirement("ALL"), AuthRequirement::ALL);
  EXPECT_EQ(string_to_auth_requirement("All"), AuthRequirement::ALL);
}

TEST(ConversionFunctionsTest, StringToAuthRequirementInvalidValues) {
  EXPECT_THROW(string_to_auth_requirement(""), std::invalid_argument);
  EXPECT_THROW(string_to_auth_requirement("read"), std::invalid_argument);
  EXPECT_THROW(string_to_auth_requirement("partial"), std::invalid_argument);
  EXPECT_THROW(string_to_auth_requirement("true"), std::invalid_argument);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
