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

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

// -----------------------------------------------------------------------------
// Local seed DTO + dto_fields / dto_name specialisations so the typed registry
// overloads can be used to populate test routes. The DTO body is irrelevant
// for the metadata tests below; what matters is that
// `reg.get<RouteRegistryTestSeedDto>(...)` registers a route with the same
// path/tag/summary surface the legacy raw overloads used to expose.
// -----------------------------------------------------------------------------

namespace ros2_medkit_gateway {
namespace dto {

struct RouteRegistryTestSeedDto {
  int value{0};
};

template <>
inline constexpr auto dto_fields<RouteRegistryTestSeedDto> =
    std::make_tuple(field("value", &RouteRegistryTestSeedDto::value));

template <>
inline constexpr std::string_view dto_name<RouteRegistryTestSeedDto> = "RouteRegistryTestSeedDto";

}  // namespace dto
}  // namespace ros2_medkit_gateway

using namespace ros2_medkit_gateway::openapi;
using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::dto::FaultEntityListQuery;
using ros2_medkit_gateway::dto::FaultListQuery;
using ros2_medkit_gateway::dto::RouteRegistryTestSeedDto;
using ros2_medkit_gateway::http::Result;
using ros2_medkit_gateway::http::TypedRequest;
using json = nlohmann::json;

namespace {

// Typed seed handler: returns a default-constructed RouteRegistryTestSeedDto.
// All tests in this file exercise OpenAPI metadata and registry bookkeeping;
// the handler body is never invoked except by the trailing-slash routing test.
Result<RouteRegistryTestSeedDto> seed_get_handler(TypedRequest /*req*/) {
  return RouteRegistryTestSeedDto{};
}

Result<RouteRegistryTestSeedDto> seed_post_handler(TypedRequest /*req*/, RouteRegistryTestSeedDto /*body*/) {
  return RouteRegistryTestSeedDto{};
}

Result<ros2_medkit_gateway::http::NoContent> seed_del_handler(TypedRequest /*req*/) {
  return ros2_medkit_gateway::http::NoContent{};
}

// Tiny helper to keep call sites short. The std::function indirection matches
// the deduction shape the typed overloads expect.
RouteEntry & seed_get(RouteRegistry & reg, const std::string & path) {
  std::function<Result<RouteRegistryTestSeedDto>(TypedRequest)> h = &seed_get_handler;
  return reg.get<RouteRegistryTestSeedDto>(path, std::move(h));
}

RouteEntry & seed_post(RouteRegistry & reg, const std::string & path) {
  std::function<Result<RouteRegistryTestSeedDto>(TypedRequest, RouteRegistryTestSeedDto)> h = &seed_post_handler;
  return reg.post<RouteRegistryTestSeedDto, RouteRegistryTestSeedDto>(path, std::move(h));
}

RouteEntry & seed_del(RouteRegistry & reg, const std::string & path) {
  std::function<Result<ros2_medkit_gateway::http::NoContent>(TypedRequest)> h = &seed_del_handler;
  return reg.del<ros2_medkit_gateway::http::NoContent>(path, std::move(h));
}

}  // namespace

// =============================================================================
// Test fixture
// =============================================================================

class RouteRegistryTest : public ::testing::Test {
 protected:
  RouteRegistry registry_;
};

// =============================================================================
// to_openapi_paths - basic structure
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToOpenapiPathsContainsRegisteredRoute) {
  seed_get(registry_, "/health").tag("Server").summary("Health check");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/health"));
  ASSERT_TRUE(paths["/health"].contains("get"));
  EXPECT_EQ(paths["/health"]["get"]["tags"][0], "Server");
  EXPECT_EQ(paths["/health"]["get"]["summary"], "Health check");
}

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToOpenapiPathsMultipleMethodsSamePath) {
  seed_get(registry_, "/data").tag("Data").summary("List data");
  seed_post(registry_, "/data").tag("Data").summary("Create data");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/data"));
  EXPECT_TRUE(paths["/data"].contains("get"));
  EXPECT_TRUE(paths["/data"].contains("post"));
}

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToOpenapiPathsEmitsQueryParameters) {
  seed_get(registry_, "/components/{component_id}/logs")
      .tag("Logs")
      .query_param("severity", "Filter by minimum severity")
      .query_param("context", "Filter by logger context")
      .query_param("include_muted", "Include muted entries", "boolean");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/components/{component_id}/logs"));
  auto & get_op = paths["/components/{component_id}/logs"]["get"];
  ASSERT_TRUE(get_op.contains("parameters"));

  // Collect the query parameters by name (the path param is auto-generated).
  std::vector<std::string> query_names;
  const nlohmann::json * severity = nullptr;
  const nlohmann::json * include_muted = nullptr;
  for (const auto & p : get_op["parameters"]) {
    if (p["in"] == "query") {
      query_names.push_back(p["name"].get<std::string>());
      if (p["name"] == "severity") {
        severity = &p;
      }
      if (p["name"] == "include_muted") {
        include_muted = &p;
      }
    }
  }

  EXPECT_EQ(query_names.size(), 3u);
  ASSERT_NE(severity, nullptr);
  EXPECT_EQ((*severity)["in"], "query");
  EXPECT_FALSE((*severity)["required"].get<bool>());
  EXPECT_EQ((*severity)["schema"]["type"], "string");
  ASSERT_NE(include_muted, nullptr);
  EXPECT_EQ((*include_muted)["schema"]["type"], "boolean");
}

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, TypedQueryDeclaresParametersFromDto) {
  // .query<T>() derives the OpenAPI parameters straight from dto_fields<T> - the
  // same descriptor a handler reads via TypedRequest::query<T>(), so the two
  // cannot drift.
  seed_get(registry_, "/faults").tag("Faults").query<FaultListQuery>();

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/faults"));
  auto & get_op = paths["/faults"]["get"];
  ASSERT_TRUE(get_op.contains("parameters"));

  const nlohmann::json * status = nullptr;
  const nlohmann::json * include_muted = nullptr;
  std::size_t query_count = 0;
  for (const auto & p : get_op["parameters"]) {
    if (p["in"] == "query") {
      ++query_count;
      if (p["name"] == "status") {
        status = &p;
      }
      if (p["name"] == "include_muted") {
        include_muted = &p;
      }
    }
  }

  // status + include_muted + include_clusters.
  EXPECT_EQ(query_count, 3u);
  ASSERT_NE(status, nullptr);
  EXPECT_EQ((*status)["in"], "query");
  EXPECT_FALSE((*status)["required"].get<bool>());  // optional<std::string> -> not required
  EXPECT_EQ((*status)["schema"]["type"], "string");
  ASSERT_NE(include_muted, nullptr);
  EXPECT_EQ((*include_muted)["schema"]["type"], "boolean");  // bool member, kOptional presence
  EXPECT_FALSE((*include_muted)["required"].get<bool>());
}

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, PerEntityFaultsQueryAdvertisesStatusOnly) {
  // The per-entity /faults route uses the narrower FaultEntityListQuery so the
  // spec advertises exactly what the handler reads: status only. The
  // correlation flags (include_muted / include_clusters) are global-only, so
  // they must NOT appear on the per-entity route.
  seed_get(registry_, "/apps/{app_id}/faults").tag("Faults").query<FaultEntityListQuery>();

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/apps/{app_id}/faults"));
  auto & get_op = paths["/apps/{app_id}/faults"]["get"];
  ASSERT_TRUE(get_op.contains("parameters"));

  std::vector<std::string> query_names;
  for (const auto & p : get_op["parameters"]) {
    if (p["in"] == "query") {
      query_names.push_back(p["name"].get<std::string>());
    }
  }

  ASSERT_EQ(query_names.size(), 1u);
  EXPECT_EQ(query_names[0], "status");
}

// =============================================================================
// to_regex_path - path conversion
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToRegexPathRootBecomesRootAnchored) {
  // Register the root path and verify the regex conversion via
  // the handler registration (to_regex_path is private, test indirectly)
  seed_get(registry_, "/").tag("Server");
  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/"));
}

TEST_F(RouteRegistryTest, ToRegexPathAppIdUsesNonGreedyCapture) {
  seed_get(registry_, "/apps/{app_id}").tag("Discovery");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/apps/{app_id}"));
  // The path param should be auto-generated
  auto & get_op = paths["/apps/{app_id}"]["get"];
  ASSERT_TRUE(get_op.contains("parameters"));

  bool found = false;
  for (const auto & p : get_op["parameters"]) {
    if (p["name"] == "app_id") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected auto-generated app_id path parameter";
}

TEST_F(RouteRegistryTest, ToRegexPathDataIdAtEndIsMultiSegment) {
  // data_id at end of path should use (.+) for multi-segment topic names
  seed_get(registry_, "/apps/{app_id}/data/{data_id}").tag("Data");

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/apps/{app_id}/data/{data_id}"));
}

TEST_F(RouteRegistryTest, ToRegexPathConfigIdAtEndIsMultiSegment) {
  // config_id at end of path should use (.+) for slash-containing param names
  seed_get(registry_, "/apps/{app_id}/configurations/{config_id}").tag("Configuration");

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/apps/{app_id}/configurations/{config_id}"));
}

TEST_F(RouteRegistryTest, ToRegexPathHealthConvertsCleanly) {
  seed_get(registry_, "/health").tag("Server");

  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/health"));
}

// =============================================================================
// Trailing slash tolerance (Fix 322)
// =============================================================================

TEST_F(RouteRegistryTest, RoutesMatchWithAndWithoutTrailingSlash) {
  RouteRegistry reg;
  seed_get(reg, "/").tag("Server");
  seed_get(reg, "/health").tag("Server");

  httplib::Server server;
  reg.register_all(server, "/api/v1");

  int port = server.bind_to_any_port("127.0.0.1");
  std::thread t([&server]() {
    server.listen_after_bind();
  });
  server.wait_until_ready();

  httplib::Client cli("127.0.0.1", port);

  // Root with trailing slash
  auto r1 = cli.Get("/api/v1/");
  ASSERT_TRUE(r1);
  EXPECT_EQ(r1->status, 200) << "GET /api/v1/ should match root route";

  // Root without trailing slash (#322)
  auto r2 = cli.Get("/api/v1");
  ASSERT_TRUE(r2);
  EXPECT_EQ(r2->status, 200) << "GET /api/v1 should also match root route";

  // /health without trailing slash
  auto r3 = cli.Get("/api/v1/health");
  ASSERT_TRUE(r3);
  EXPECT_EQ(r3->status, 200) << "GET /api/v1/health should work";

  // /health with trailing slash
  auto r4 = cli.Get("/api/v1/health/");
  ASSERT_TRUE(r4);
  EXPECT_EQ(r4->status, 200) << "GET /api/v1/health/ should also work";

  server.stop();
  t.join();
}

// =============================================================================
// set_auth_enabled - 401/403 responses
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, AuthEnabledAdds401And403Responses) {
  registry_.set_auth_enabled(true);
  seed_get(registry_, "/health").tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("401"));
  EXPECT_TRUE(responses.contains("403"));
}

TEST_F(RouteRegistryTest, AuthDisabledNo401Or403Responses) {
  registry_.set_auth_enabled(false);
  seed_get(registry_, "/health").tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_FALSE(responses.contains("401"));
  EXPECT_FALSE(responses.contains("403"));
}

// =============================================================================
// tags - unique tag names
// =============================================================================

TEST_F(RouteRegistryTest, TagsReturnsUniqueTags) {
  seed_get(registry_, "/health").tag("Server");
  seed_get(registry_, "/areas").tag("Discovery");
  seed_get(registry_, "/apps").tag("Discovery");
  seed_get(registry_, "/data").tag("Data");

  auto tags = registry_.tags();

  // Should have exactly 3 unique tags (Server, Discovery, Data)
  EXPECT_EQ(tags.size(), 3u);

  // Check each tag is present (tags() returns sorted via std::set)
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Server"), tags.end());
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Discovery"), tags.end());
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Data"), tags.end());
}

// =============================================================================
// Auto-generated path params have correct names
// =============================================================================

TEST_F(RouteRegistryTest, AutoGeneratedPathParamsHaveCorrectNames) {
  seed_get(registry_, "/areas/{area_id}/components/{component_id}").tag("Discovery");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/areas/{area_id}/components/{component_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 2u);

  std::vector<std::string> param_names;
  for (const auto & p : params) {
    param_names.push_back(p["name"].get<std::string>());
  }

  EXPECT_NE(std::find(param_names.begin(), param_names.end(), "area_id"), param_names.end());
  EXPECT_NE(std::find(param_names.begin(), param_names.end(), "component_id"), param_names.end());
}

// =============================================================================
// Default 200 response when no explicit responses
// =============================================================================

TEST_F(RouteRegistryTest, Default200ResponseWhenNoExplicitResponses) {
  // Typed GET<T> automatically attaches a 200 response with the DTO $ref.
  seed_get(registry_, "/health").tag("Server").summary("Health check");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("200"));
  // Schema $ref is auto-populated to the seed DTO's component.
  auto & schema = responses["200"]["content"]["application/json"]["schema"];
  ASSERT_TRUE(schema.contains("$ref"));
  EXPECT_EQ(schema["$ref"], "#/components/schemas/RouteRegistryTestSeedDto");
}

TEST_F(RouteRegistryTest, ExplicitResponseOverridesDefault) {
  seed_get(registry_, "/health").tag("Server").response(200, "Gateway is healthy");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("200"));
  EXPECT_EQ(responses["200"]["description"], "Gateway is healthy");
}

// =============================================================================
// operationId auto-generation (Fix 14)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, OperationIdIsGenerated) {
  seed_get(registry_, "/apps/{app_id}/data").tag("Data");

  auto paths = registry_.to_openapi_paths();
  auto & get_op = paths["/apps/{app_id}/data"]["get"];

  ASSERT_TRUE(get_op.contains("operationId"));
  std::string op_id = get_op["operationId"];
  EXPECT_FALSE(op_id.empty());
  // Auto-generator strips {param} segments and produces camelCase: getAppsData
  EXPECT_NE(op_id.find("get"), std::string::npos);
  EXPECT_NE(op_id.find("Apps"), std::string::npos);
  EXPECT_NE(op_id.find("Data"), std::string::npos);
}

TEST_F(RouteRegistryTest, OperationIdForRootPath) {
  seed_get(registry_, "/").tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & get_op = paths["/"]["get"];

  ASSERT_TRUE(get_op.contains("operationId"));
  std::string op_id = get_op["operationId"];
  EXPECT_FALSE(op_id.empty());
}

TEST_F(RouteRegistryTest, OperationIdUniquePerMethodPath) {
  seed_get(registry_, "/data").tag("Data");
  seed_post(registry_, "/data").tag("Data");

  auto paths = registry_.to_openapi_paths();

  std::string get_id = paths["/data"]["get"]["operationId"];
  std::string post_id = paths["/data"]["post"]["operationId"];

  EXPECT_NE(get_id, post_id);
}

// =============================================================================
// Path param descriptions (Fix 15)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, PathParamDescriptionsArePresent) {
  seed_get(registry_, "/apps/{app_id}/data/{data_id}").tag("Data");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}/data/{data_id}"]["get"]["parameters"];

  for (const auto & p : params) {
    EXPECT_TRUE(p.contains("description")) << "Missing description for param: " << p["name"];
    EXPECT_FALSE(p["description"].get<std::string>().empty());
  }
}

TEST_F(RouteRegistryTest, KnownParamHasSpecificDescription) {
  seed_get(registry_, "/apps/{app_id}").tag("Discovery");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0]["name"], "app_id");
  EXPECT_EQ(params[0]["description"], "The app identifier");
}

TEST_F(RouteRegistryTest, UnknownParamGetsGenericDescription) {
  seed_get(registry_, "/widgets/{widget_id}").tag("Custom");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/widgets/{widget_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0]["description"], "The widget_id value");
}

// =============================================================================
// Header params (Fix 328)
// =============================================================================

TEST_F(RouteRegistryTest, HeaderParamAppearsInOpenApiOutput) {
  seed_post(registry_, "/apps/{app_id}/locks")
      .tag("Locking")
      .header_param("X-Client-Id", "Client identifier")
      .response(201, "Created", {{"type", "object"}});

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}/locks"]["post"]["parameters"];

  // Should have both auto-generated path param and explicit header param
  bool found_header = false;
  for (const auto & p : params) {
    if (p["name"] == "X-Client-Id") {
      found_header = true;
      EXPECT_EQ(p["in"], "header");
      EXPECT_TRUE(p["required"].get<bool>());
      EXPECT_EQ(p["description"], "Client identifier");
      EXPECT_EQ(p["schema"]["type"], "string");
    }
  }
  EXPECT_TRUE(found_header) << "X-Client-Id header param not found in OpenAPI output";
}

TEST_F(RouteRegistryTest, OptionalHeaderParamHasRequiredFalse) {
  seed_get(registry_, "/apps/{app_id}/locks")
      .tag("Locking")
      .header_param("X-Client-Id", "Optional client identifier", false)
      .response(200, "OK", {{"type", "object"}});

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}/locks"]["get"]["parameters"];

  bool found_header = false;
  for (const auto & p : params) {
    if (p["name"] == "X-Client-Id") {
      found_header = true;
      EXPECT_FALSE(p["required"].get<bool>());
    }
  }
  EXPECT_TRUE(found_header) << "Optional header param not found";
}

// =============================================================================
// lock_guarded() / fan_out_aware()
// =============================================================================

namespace {

/// Find a parameter by name in an operation's parameter array.
const nlohmann::json * find_param(const nlohmann::json & params, const std::string & name) {
  for (const auto & p : params) {
    if (p.contains("name") && p["name"] == name) {
      return &p;
    }
  }
  return nullptr;
}

}  // namespace

TEST_F(RouteRegistryTest, LockGuardedDeclaresHeaderStatusAndMarker) {
  seed_post(registry_, "/apps/{app_id}/data/{data_id}").tag("Data").lock_guarded();

  auto paths = registry_.to_openapi_paths();
  auto & op = paths["/apps/{app_id}/data/{data_id}"]["post"];

  // contains() first: get<bool>() on an absent key throws json::type_error,
  // which reports as a crashed test rather than a failed expectation.
  ASSERT_TRUE(op.contains("x-medkit-lock-guarded")) << "marker not emitted";
  EXPECT_TRUE(op["x-medkit-lock-guarded"].get<bool>());
  ASSERT_TRUE(op["responses"].contains("409")) << "the 409 the marker promises is missing";

  const auto * client_id = find_param(op["parameters"], "X-Client-Id");
  ASSERT_NE(client_id, nullptr) << "X-Client-Id not declared";
  EXPECT_EQ((*client_id)["in"], "header");
  // Optional: a header-less request succeeds while nothing is locked, so
  // declaring it required would describe a gateway that rejects it outright.
  EXPECT_FALSE((*client_id)["required"].get<bool>());
  EXPECT_EQ((*client_id)["schema"]["type"], "string");
}

TEST_F(RouteRegistryTest, UnguardedRouteCarriesNoLockMarker) {
  seed_get(registry_, "/apps/{app_id}/data").tag("Data");

  auto paths = registry_.to_openapi_paths();
  auto & op = paths["/apps/{app_id}/data"]["get"];

  EXPECT_FALSE(op.contains("x-medkit-lock-guarded"));
  EXPECT_FALSE(op["responses"].contains("409"));
}

TEST_F(RouteRegistryTest, LockGuardedMarkerWithoutA409IsReported) {
  // only_status() clears declared_errors_, so it is the one way to keep the
  // marker while dropping the status it promises. The registry has to report
  // that rather than publish a marker no client can act on.
  seed_del(registry_, "/apps/{app_id}/bulk-data/{file_id}")
      .tag("Bulk Data")
      .lock_guarded()
      .only_status(501, "Not implemented");

  auto issues = registry_.validate_completeness();
  bool reported = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError &&
        issue.message.find("lock_guarded() marker without a declared 409") != std::string::npos) {
      reported = true;
    }
  }
  EXPECT_TRUE(reported) << "a marker without its 409 passed validate_completeness()";
}

TEST_F(RouteRegistryTest, FanOutAwareDeclaresPresenceOnlyHeader) {
  seed_get(registry_, "/apps/{app_id}/logs").tag("Logs").fan_out_aware();

  auto paths = registry_.to_openapi_paths();
  const auto * param = find_param(paths["/apps/{app_id}/logs"]["get"]["parameters"], "X-Medkit-No-Fan-Out");

  ASSERT_NE(param, nullptr) << "X-Medkit-No-Fan-Out not declared";
  EXPECT_EQ((*param)["in"], "header");
  EXPECT_FALSE((*param)["required"].get<bool>());
  // The gateway tests has_header and never reads the value, so `false` also
  // suppresses fan-out. A boolean schema would promise the opposite.
  EXPECT_EQ((*param)["schema"]["type"], "string");
}

// =============================================================================
// to_endpoint_list (Fix 23)
// =============================================================================

TEST_F(RouteRegistryTest, ToEndpointListProducesCorrectFormat) {
  seed_get(registry_, "/health").tag("Server");
  seed_post(registry_, "/auth/token").tag("Authentication");
  seed_del(registry_, "/faults").tag("Faults");

  auto endpoints = registry_.to_endpoint_list("/api/v1");

  ASSERT_EQ(endpoints.size(), 3u);
  EXPECT_EQ(endpoints[0], "GET /api/v1/health");
  EXPECT_EQ(endpoints[1], "POST /api/v1/auth/token");
  EXPECT_EQ(endpoints[2], "DELETE /api/v1/faults");
}

// =============================================================================
// size() and empty registry
// =============================================================================

TEST_F(RouteRegistryTest, EmptyRegistryHasZeroSize) {
  EXPECT_EQ(registry_.size(), 0u);
}

TEST_F(RouteRegistryTest, SizeReflectsRegisteredRoutes) {
  seed_get(registry_, "/health");
  seed_post(registry_, "/data");
  seed_post(registry_, "/config");

  EXPECT_EQ(registry_.size(), 3u);
}

TEST_F(RouteRegistryTest, EmptyRegistryProducesEmptyPaths) {
  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.empty());
}

// =============================================================================
// Deprecated flag
// =============================================================================

TEST_F(RouteRegistryTest, DeprecatedFlagAppearsInOutput) {
  seed_get(registry_, "/old-endpoint").tag("Server").deprecated();

  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths["/old-endpoint"]["get"]["deprecated"].get<bool>());
}

// =============================================================================
// validate_completeness
// =============================================================================

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForCompleteRoute) {
  seed_get(registry_, "/health")
      .tag("Server")
      .summary("Health check")
      .response(200, "Healthy", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  // No errors expected
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessErrorOnMissingTag) {
  seed_get(registry_, "/health").summary("Health check").response(200, "Healthy", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  bool has_tag_error = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError && issue.message == "Missing tag") {
      has_tag_error = true;
    }
  }
  EXPECT_TRUE(has_tag_error);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForDeleteWith204) {
  seed_del(registry_, "/items/{id}").tag("Items").summary("Delete item");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForSSEEndpoint) {
  seed_get(registry_, "/events/stream").tag("Events").summary("SSE events stream");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessWarnsOnMissingSummary) {
  seed_get(registry_, "/health").tag("Server").response(200, "OK", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  bool has_summary_warning = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kWarning && issue.message == "Missing summary") {
      has_summary_warning = true;
    }
  }
  EXPECT_TRUE(has_summary_warning);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForCompletePostRoute) {
  seed_post(registry_, "/items").tag("Items").summary("Create item");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

// =============================================================================
// operationId explicit vs auto-generated (Task 4)
// =============================================================================

TEST_F(RouteRegistryTest, ExplicitOperationIdUsedWhenSet) {
  seed_get(registry_, "/health")
      .tag("Server")
      .summary("Health")
      .operation_id("getHealth")
      .response(200, "OK", json{{"type", "object"}});
  auto paths = registry_.to_openapi_paths();
  EXPECT_EQ(paths["/health"]["get"]["operationId"].get<std::string>(), "getHealth");
}

TEST_F(RouteRegistryTest, AutoGeneratedOperationIdStripParams) {
  seed_get(registry_, "/apps/{app_id}/faults")
      .tag("Faults")
      .summary("Faults")
      .response(200, "OK", json{{"type", "object"}});
  auto paths = registry_.to_openapi_paths();
  auto op_id = paths["/apps/{app_id}/faults"]["get"]["operationId"].get<std::string>();
  EXPECT_EQ(op_id, "getAppsFaults");
}

// =============================================================================
// Hidden routes
// =============================================================================

TEST_F(RouteRegistryTest, HiddenRouteExcludedFromOpenapiPaths) {
  seed_get(registry_, "/visible").tag("Test").summary("Visible").response(200, "OK", json{{"type", "object"}});
  seed_post(registry_, "/hidden-405").tag("Test").summary("Not supported").hidden();

  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/visible"));
  EXPECT_FALSE(paths.contains("/hidden-405"));
}

TEST_F(RouteRegistryTest, HiddenRouteStillCountedInSize) {
  seed_get(registry_, "/visible").tag("Test").summary("Visible").response(200, "OK", json{{"type", "object"}});
  seed_post(registry_, "/hidden").tag("Test").summary("Hidden").hidden();

  EXPECT_EQ(registry_.size(), 2u);
}

TEST_F(RouteRegistryTest, HiddenRouteSkippedByValidateCompleteness) {
  // Hidden route without required metadata should NOT trigger validation errors
  seed_post(registry_, "/hidden").hidden();

  auto issues = registry_.validate_completeness();
  EXPECT_TRUE(issues.empty());
}

// =============================================================================
// Error responses use GenericError $ref (Task 2)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ErrorResponsesUseGenericErrorRef) {
  seed_get(registry_, "/test").tag("Test").summary("Test").response(200, "OK", json{{"type", "object"}});

  auto paths = registry_.to_openapi_paths();
  auto & resp_400 = paths["/test"]["get"]["responses"]["400"];

  // Error responses should $ref the GenericError response component
  ASSERT_TRUE(resp_400.contains("$ref"));
  EXPECT_EQ(resp_400["$ref"].get<std::string>(), "#/components/responses/GenericError");
  // No inline description when using $ref
  EXPECT_FALSE(resp_400.contains("description"));
}

// -----------------------------------------------------------------------------
// only_status / gated_on interaction
// -----------------------------------------------------------------------------

TEST_F(RouteRegistryTest, OnlyStatusPublishesErrorBodySchema) {
  // A 501 stub answers with a GenericError body. Publishing the status without
  // `content` would describe a bodyless response, which a generated client
  // models as void and then receives JSON into.
  seed_get(registry_, "/stub").tag("Test").summary("Stub").only_status(501, "Not implemented for ROS 2");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/stub"]["get"]["responses"];

  ASSERT_TRUE(responses.contains("501"));
  EXPECT_EQ(responses["501"]["description"].get<std::string>(), "Not implemented for ROS 2");
  ASSERT_TRUE(responses["501"].contains("content"));
  EXPECT_EQ(responses["501"]["content"]["application/json"]["schema"]["$ref"].get<std::string>(),
            "#/components/schemas/GenericError");

  // The single-outcome claim still drops the blanket set and the derived 200.
  EXPECT_FALSE(responses.contains("200"));
  EXPECT_FALSE(responses.contains("400"));
  EXPECT_FALSE(responses.contains("404"));
  EXPECT_FALSE(responses.contains("500"));
}

TEST_F(RouteRegistryTest, OnlyStatusKeepsAnErrorStubComplete) {
  // The stub declares no 2xx at all; validate_completeness must not ask it for
  // a success schema it can never have.
  seed_get(registry_, "/stub").tag("Test").summary("Stub").only_status(501, "Not implemented");

  for (const auto & issue : registry_.validate_completeness()) {
    EXPECT_NE(issue.severity, ValidationIssue::Severity::kError) << issue.route << ": " << issue.message;
  }
}

TEST_F(RouteRegistryTest, GatedRouteDeclaresTheGateStatus) {
  ErrorInfo unavailable;
  unavailable.code = "not-implemented";
  unavailable.message = "Feature not available";
  unavailable.http_status = 501;

  seed_get(registry_, "/gated")
      .tag("Test")
      .summary("Gated")
      .gated_on(
          [] {
            return false;
          },
          unavailable);

  auto paths = registry_.to_openapi_paths();
  auto & resp_501 = paths["/gated"]["get"]["responses"]["501"];
  ASSERT_TRUE(resp_501.contains("$ref"));
  EXPECT_EQ(resp_501["$ref"].get<std::string>(), "#/components/responses/GenericError");
}

TEST_F(RouteRegistryTest, GatedStatusSurvivesEitherBuilderOrder) {
  // A live gate is a second reachable outcome, so only_status() must not drop
  // it - regardless of which call comes last. Without this the very status the
  // gate exists to return disappears from the document on one ordering.
  ErrorInfo unavailable;
  unavailable.code = "not-implemented";
  unavailable.message = "Feature not available";
  unavailable.http_status = 501;
  auto never_available = [] {
    return false;
  };

  seed_get(registry_, "/gate-then-only")
      .tag("Test")
      .summary("Gate then only")
      .gated_on(never_available, unavailable)
      .only_status(405, "Method not allowed");

  seed_get(registry_, "/only-then-gate")
      .tag("Test")
      .summary("Only then gate")
      .only_status(405, "Method not allowed")
      .gated_on(never_available, unavailable);

  auto paths = registry_.to_openapi_paths();
  for (const char * path : {"/gate-then-only", "/only-then-gate"}) {
    auto & responses = paths[path]["get"]["responses"];
    EXPECT_TRUE(responses.contains("501")) << path << " dropped the gate status";
    EXPECT_TRUE(responses.contains("405")) << path << " dropped the only_status code";
  }
}

TEST_F(RouteRegistryTest, GateWithSubErrorStatusIsReportedNotPublished) {
  // gated_on() declares through errors(), so a sub-400 status is rejected and
  // surfaced instead of being published as a phantom success.
  ErrorInfo misconfigured;
  misconfigured.code = "nonsense";
  misconfigured.http_status = 302;

  seed_get(registry_, "/bad-gate")
      .tag("Test")
      .summary("Bad gate")
      .gated_on(
          [] {
            return true;
          },
          misconfigured);

  bool reported = false;
  for (const auto & issue : registry_.validate_completeness()) {
    if (issue.message.find("302") != std::string::npos) {
      reported = true;
    }
  }
  EXPECT_TRUE(reported) << "sub-400 gate status was silently dropped";
  EXPECT_FALSE(registry_.to_openapi_paths()["/bad-gate"]["get"]["responses"].contains("302"));
}

// =============================================================================
// Response headers
// =============================================================================

namespace {

// Seed handler returning `Created<SeedDto>` so the registry derives 201 - the
// status that carries the automatic `Location` declaration. It must be the
// attachments form: the non-attachments overloads static_assert against 201/202
// precisely because they give the handler no way to send the header the
// document then advertises.
using SeedCreated = ros2_medkit_gateway::http::Created<RouteRegistryTestSeedDto>;
using SeedCreatedPair = std::pair<SeedCreated, ros2_medkit_gateway::http::ResponseAttachments>;

Result<SeedCreatedPair> seed_created_handler(TypedRequest req, RouteRegistryTestSeedDto /*body*/) {
  ros2_medkit_gateway::http::ResponseAttachments att;
  att.with_location(req.path() + "/seed");
  return SeedCreatedPair{SeedCreated{RouteRegistryTestSeedDto{}}, std::move(att)};
}

RouteEntry & seed_created_post(RouteRegistry & reg, const std::string & path) {
  std::function<Result<SeedCreatedPair>(TypedRequest, RouteRegistryTestSeedDto)> h = &seed_created_handler;
  return reg.post<RouteRegistryTestSeedDto, SeedCreated>(path, std::move(h));
}

}  // namespace

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, DerivedCreatedResponseDeclaresLocationHeader) {
  // The 201 comes from `Created<T>`; the `Location` declaration is derived from
  // that same status, so a route cannot ship one without the other.
  seed_created_post(registry_, "/items").tag("Test").summary("Create item");

  auto paths = registry_.to_openapi_paths();
  auto & created = paths["/items"]["post"]["responses"]["201"];

  ASSERT_TRUE(created.contains("headers")) << created.dump();
  ASSERT_TRUE(created["headers"].contains("Location"));
  EXPECT_FALSE(created["headers"]["Location"]["description"].get<std::string>().empty());
  EXPECT_EQ(created["headers"]["Location"]["schema"]["type"].get<std::string>(), "string");
}

TEST_F(RouteRegistryTest, PlainSuccessResponseDeclaresNoLocationHeader) {
  // 200 does not identify a newly created resource, so declaring `Location`
  // there would advertise a header the handler never sets.
  seed_get(registry_, "/items").tag("Test").summary("List items");

  // Bind the document to a local: `to_openapi_paths()` returns by value, and a
  // reference into the temporary dangles the moment the statement ends.
  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths["/items"]["get"]["responses"].contains("200"));
  auto & ok = paths["/items"]["get"]["responses"]["200"];
  EXPECT_FALSE(ok.contains("headers")) << ok.dump();
}

TEST_F(RouteRegistryTest, ResponseHeaderReplacesSameNameOnSameStatus) {
  seed_created_post(registry_, "/items")
      .tag("Test")
      .summary("Create item")
      .response_header(201, ResponseHeader{"Location", "Bespoke prose"});

  auto paths = registry_.to_openapi_paths();
  auto & headers = paths["/items"]["post"]["responses"]["201"]["headers"];
  EXPECT_EQ(headers.size(), 1u) << headers.dump();
  EXPECT_EQ(headers["Location"]["description"].get<std::string>(), "Bespoke prose");
}

TEST_F(RouteRegistryTest, ResponseHeaderOnUndeclaredStatusIsReportedNotPublished) {
  // A header aimed at a status no response declares would otherwise mint a
  // description-less response object and publish a status the handler cannot
  // return. It is dropped and surfaced by validate_completeness() instead - an
  // assert would be compiled out of the release build the gateway ships.
  seed_get(registry_, "/items")
      .tag("Test")
      .summary("List items")
      .response_header(418, ResponseHeader{"X-Teapot", "Never declared"});

  bool reported = false;
  for (const auto & issue : registry_.validate_completeness()) {
    if (issue.message.find("418") != std::string::npos) {
      reported = true;
      EXPECT_EQ(issue.severity, ValidationIssue::Severity::kError);
    }
  }
  EXPECT_TRUE(reported) << "undeclared-status header was silently dropped";
  auto paths = registry_.to_openapi_paths();
  EXPECT_FALSE(paths["/items"]["get"]["responses"].contains("418"));
}

TEST_F(RouteRegistryTest, BodylessResponseKeepsNoSchemaKeyWhenHeadersAreDeclared) {
  // A default-constructed nlohmann::json is `null`. Emitting headers must not
  // drag a `"schema": null` onto a 204 that legitimately has no body.
  seed_del(registry_, "/items")
      .tag("Test")
      .summary("Delete item")
      .response_header(204, ResponseHeader{"X-Medkit-Local-Only", "Local scope only"});

  auto paths = registry_.to_openapi_paths();
  auto & no_content = paths["/items"]["delete"]["responses"]["204"];
  EXPECT_FALSE(no_content.contains("content")) << no_content.dump();
  ASSERT_TRUE(no_content.contains("headers"));
  EXPECT_TRUE(no_content["headers"].contains("X-Medkit-Local-Only"));
}

TEST_F(RouteRegistryTest, MiddlewareStatusesReferenceTheirOwnComponents) {
  // 401/403/429 never reach a handler, so their headers (WWW-Authenticate,
  // Retry-After, X-RateLimit-*) can only live on shared components. Pointing
  // them at GenericError would describe the wrong body and no headers at all.
  registry_.set_auth_enabled(true);
  registry_.set_rate_limit_enabled(true);
  seed_get(registry_, "/items").tag("Test").summary("List items");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/items"]["get"]["responses"];
  EXPECT_EQ(responses["401"]["$ref"].get<std::string>(), "#/components/responses/Unauthorized");
  EXPECT_EQ(responses["403"]["$ref"].get<std::string>(), "#/components/responses/Forbidden");
  EXPECT_EQ(responses["429"]["$ref"].get<std::string>(), "#/components/responses/RateLimited");
  EXPECT_EQ(responses["400"]["$ref"].get<std::string>(), "#/components/responses/GenericError");
}

TEST_F(RouteRegistryTest, RateLimitedStatusAbsentWhenLimiterIsOff) {
  seed_get(registry_, "/items").tag("Test").summary("List items");
  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths["/items"]["get"]["responses"].contains("200")) << "route missing; absence check would be vacuous";
  EXPECT_FALSE(paths["/items"]["get"]["responses"].contains("429"));
}

TEST_F(RouteRegistryTest, PeerFailureStatusesFollowTheAggregationGate) {
  // With aggregation on, an entity can be owned by a peer, and the proxy path
  // inside validate_entity_for_route answers 502 (peer unknown / unreachable /
  // oversized response) or 503 (this gateway is shutting down) instead of the
  // handler. Only entity-scoped routes can reach it: the entity id has to come
  // from the path for the lookup to happen at all.
  registry_.set_aggregation_enabled(true);
  seed_get(registry_, "/apps/{app_id}/data").tag("Test").summary("List data");
  seed_get(registry_, "/health").tag("Test").summary("Health");

  auto paths = registry_.to_openapi_paths();
  auto & entity = paths["/apps/{app_id}/data"]["get"]["responses"];
  EXPECT_EQ(entity["502"]["$ref"].get<std::string>(), "#/components/responses/GenericError");
  EXPECT_EQ(entity["503"]["$ref"].get<std::string>(), "#/components/responses/GenericError");

  // A route with no entity to resolve never forwards, so declaring the peer
  // statuses there would document an outcome it cannot produce.
  auto & health = paths["/health"]["get"]["responses"];
  ASSERT_TRUE(health.contains("200")) << "route missing; absence check would be vacuous";
  EXPECT_FALSE(health.contains("502"));
  EXPECT_FALSE(health.contains("503"));
}

TEST_F(RouteRegistryTest, PeerFailureStatusesRespectOnlyStatus) {
  // `only_status` asserts the route has exactly one outcome. The forward
  // happens inside the handler, not ahead of it, so a single-outcome route -
  // the data-categories / data-groups stubs ignore the request entirely and
  // never resolve an entity - cannot reach the peer path. Adding 502/503 there
  // would publish two statuses the route can never return, on the one
  // declaration whose whole meaning is that it returns one.
  registry_.set_aggregation_enabled(true);
  seed_get(registry_, "/apps/{app_id}/data-categories")
      .tag("Test")
      .summary("Data categories")
      .only_status(501, "Not implemented");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/apps/{app_id}/data-categories"]["get"]["responses"];
  EXPECT_TRUE(responses.contains("501"));
  EXPECT_FALSE(responses.contains("502"));
  EXPECT_FALSE(responses.contains("503"));
  // What `only_status` constrains is the set of statuses the *handler* can
  // produce, and 501 is still the whole of that set. It was previously spelled
  // `responses.size() == 1`, which silently also asserted that nothing outside
  // the handler contributes a status - true until cpp-httplib's pre-routing
  // Range rejection was declared. 416 is answered before this route is reached
  // (Server::process_request, vendored httplib.h:6616-6622), so excluding it
  // here would document the stub as unable to answer a status it demonstrably
  // does answer. Narrowed to the claim that was actually meant, rather than
  // exempting the route.
  auto handler_statuses = responses;
  handler_statuses.erase("416");
  EXPECT_EQ(handler_statuses.size(), 1U) << responses.dump();
}

TEST_F(RouteRegistryTest, PeerFailureStatusesAbsentWhenAggregationIsOff) {
  // aggregation.enabled defaults false and the AggregationManager is only
  // constructed when it is set, so no entity can be remote and neither status
  // is reachable.
  seed_get(registry_, "/apps/{app_id}/data").tag("Test").summary("List data");
  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/apps/{app_id}/data"]["get"]["responses"];
  ASSERT_TRUE(responses.contains("200")) << "route missing; absence check would be vacuous";
  EXPECT_FALSE(responses.contains("502"));
  EXPECT_FALSE(responses.contains("503"));
}

TEST_F(RouteRegistryTest, RouteDeclaredStatusWinsOverTheMiddlewareComponent) {
  // A handler can answer a status the middleware also owns: the script
  // manager's concurrency 429 collides with the rate limiter's, and a
  // lifecycle provider's AccessDenied 403 collides with the auth middleware's.
  // OpenAPI allows exactly one response object per status, so one of the two
  // descriptions has to lose. The route's own declaration wins, because
  // `add_response_ref` is first-wins and the `errors()` loop runs first.
  //
  // What that costs is the *headers*: the operation below documents 429 as the
  // route's GenericError and therefore without `Retry-After` / `X-RateLimit-*`.
  // The body shape is unaffected - Unauthorized, Forbidden and RateLimited all
  // point at the same GenericError schema - so the loss is the header list and
  // the prose, not the payload. This test exists so that precedence is a
  // decision on record rather than an accident of statement order.
  registry_.set_auth_enabled(true);
  registry_.set_rate_limit_enabled(true);
  seed_get(registry_, "/items").tag("Test").summary("List items").errors({403, 429});

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/items"]["get"]["responses"];
  EXPECT_EQ(responses["429"]["$ref"].get<std::string>(), "#/components/responses/GenericError");
  EXPECT_EQ(responses["403"]["$ref"].get<std::string>(), "#/components/responses/GenericError");
  // 401 is not declared by the route, so the middleware component still wins
  // there - which is what makes this a precedence test rather than a test that
  // the middleware refs were dropped altogether.
  EXPECT_EQ(responses["401"]["$ref"].get<std::string>(), "#/components/responses/Unauthorized");
}

// =============================================================================
// Request-body completeness reads the registration, not the HTTP method
// =============================================================================

namespace {

using SeedAccepted = ros2_medkit_gateway::http::Accepted<ros2_medkit_gateway::http::NoContent>;
using SeedAcceptedPair = std::pair<SeedAccepted, ros2_medkit_gateway::http::ResponseAttachments>;

Result<ros2_medkit_gateway::http::NoContent> seed_body_less_handler(TypedRequest /*req*/) {
  return ros2_medkit_gateway::http::NoContent{};
}

// The payload-free state-machine kick: 202 + Location, no request body at all.
Result<SeedAcceptedPair> seed_kick_handler(TypedRequest req) {
  ros2_medkit_gateway::http::ResponseAttachments att;
  att.with_location(req.path() + "/status");
  return SeedAcceptedPair{SeedAccepted{ros2_medkit_gateway::http::NoContent{}}, std::move(att)};
}

/// Plain body-less PUT - the overload whose callers parse a body by hand.
RouteEntry & seed_body_less_put(RouteRegistry & reg, const std::string & path) {
  std::function<Result<ros2_medkit_gateway::http::NoContent>(TypedRequest)> h = &seed_body_less_handler;
  return reg.put<ros2_medkit_gateway::http::NoContent>(path, std::move(h));
}

/// Attachments body-less PUT - the overload reserved for payload-free routes.
RouteEntry & seed_kick_put(RouteRegistry & reg, const std::string & path) {
  std::function<Result<SeedAcceptedPair>(TypedRequest)> h = &seed_kick_handler;
  return reg.put<SeedAccepted>(path, std::move(h));
}

RouteEntry & seed_body_less_post(RouteRegistry & reg, const std::string & path) {
  std::function<Result<ros2_medkit_gateway::http::NoContent>(TypedRequest)> h = &seed_body_less_handler;
  return reg.post<ros2_medkit_gateway::http::NoContent>(path, std::move(h));
}

bool has_error_mentioning(const RouteRegistry & reg, const std::string & needle) {
  for (const auto & issue : reg.validate_completeness()) {
    if (issue.severity == ValidationIssue::Severity::kError && issue.message.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST_F(RouteRegistryTest, PayloadFreePutIsNotAskedForARequestBody) {
  // The attachments body-less PUT is the payload-free state-machine kick
  // (/updates/{id}/prepare, the lifecycle transitions). Inferring "PUT
  // therefore a request body" from the method reported 13 shipped routes that
  // are correct as written, which is how a diagnostic channel gets ignored.
  seed_kick_put(registry_, "/items/prepare").tag("Test").summary("Prepare");

  EXPECT_FALSE(has_error_mentioning(registry_, "request body"));
}

TEST_F(RouteRegistryTest, PlainBodyLessPutIsStillAskedForARequestBody) {
  // The plain body-less PUT is NOT the payload-free shape: its caller reads the
  // body itself (PUT /{entity}/data/{data_id} parses free-form JSON by hand).
  // Exempting it would blind the check for PUT to exactly the case kept covered
  // for POST - a manual-body route that forgets `.request_body(...)`.
  seed_body_less_put(registry_, "/items/value").tag("Test").summary("Write value");

  EXPECT_TRUE(has_error_mentioning(registry_, "request body"));
}

TEST_F(RouteRegistryTest, PlainBodyLessPutIsSatisfiedByAManualDeclaration) {
  // ...and declaring the body by hand, which is what the data route does, is
  // what clears it. Without this the test above could be satisfied by a check
  // that reports the route no matter what.
  seed_body_less_put(registry_, "/items/value")
      .tag("Test")
      .summary("Write value")
      .request_body("Free-form value", json{{"type", "object"}});

  EXPECT_FALSE(has_error_mentioning(registry_, "request body"));
}

TEST_F(RouteRegistryTest, BodyLessPostIsStillAskedForARequestBody) {
  // The body-less POST overload means the same thing: the handler parses a
  // non-JSON body itself (form-urlencoded auth). A missing declaration there is
  // a real gap in the document, so the check must stay.
  seed_body_less_post(registry_, "/items/token").tag("Test").summary("Token");

  EXPECT_TRUE(has_error_mentioning(registry_, "request body"));
}

TEST_F(RouteRegistryTest, TypedPutWithABodyIsStillSatisfiedAutomatically) {
  // Guard against the exemption leaking to the overload that does parse a body:
  // it declares its schema from TBody, so it must never be reported either.
  std::function<Result<RouteRegistryTestSeedDto>(TypedRequest, RouteRegistryTestSeedDto)> h = &seed_post_handler;
  registry_.put<RouteRegistryTestSeedDto, RouteRegistryTestSeedDto>("/items", std::move(h))
      .tag("Test")
      .summary("Replace item");

  EXPECT_FALSE(has_error_mentioning(registry_, "request body"));
  EXPECT_TRUE(registry_.to_openapi_paths()["/items"]["put"].contains("requestBody"));
}

// =============================================================================
// Media types: a non-JSON body is declared by media type, without a schema
// =============================================================================

namespace {

Result<ros2_medkit_gateway::http::BinaryResponse> seed_blob_handler(TypedRequest /*req*/) {
  ros2_medkit_gateway::http::BinaryResponse resp;
  resp.content_type = "application/octet-stream";
  resp.total_size = 0;
  resp.provider = [](uint64_t, uint64_t, httplib::DataSink & sink) -> bool {
    sink.done();
    return false;
  };
  return resp;
}

RouteEntry & seed_blob(RouteRegistry & reg, const std::string & path, const std::vector<std::string> & media_types) {
  std::function<Result<ros2_medkit_gateway::http::BinaryResponse>(TypedRequest)> h = &seed_blob_handler;
  return reg.binary_download(path, std::move(h), media_types);
}

}  // namespace

TEST_F(RouteRegistryTest, NonJsonResponseIsDeclaredUnderItsOwnMediaTypeWithoutASchema) {
  // `format: binary` under `application/json` was wrong twice over: the media
  // type is not JSON, and `{"type":"string","format":"binary"}` is an OpenAPI
  // 3.0 idiom that 3.1 dropped. The absence of a schema is the assertion, not
  // an oversight - there is nothing truthful to put there for raw bytes.
  seed_blob(registry_, "/blob", {"application/octet-stream"}).tag("Test").summary("Download");

  auto paths = registry_.to_openapi_paths();
  auto & content = paths["/blob"]["get"]["responses"]["200"]["content"];
  ASSERT_TRUE(content.contains("application/octet-stream"));
  EXPECT_FALSE(content.contains("application/json")) << "a binary body must not be advertised as JSON";
  EXPECT_FALSE(content["application/octet-stream"].contains("schema"));
  EXPECT_EQ(nlohmann::json(content["application/octet-stream"]), nlohmann::json::object());
}

TEST_F(RouteRegistryTest, EveryDeclaredMediaTypeReachesTheDocument) {
  // The open-set case: the concrete types AND the catch-all have to survive to
  // the document, because the route's whole claim is that it serves the three
  // it can name plus anything the store recorded.
  seed_blob(registry_, "/blob", {"application/x-mcap", "application/x-sqlite3", "application/octet-stream", "*/*"})
      .tag("Test")
      .summary("Download");

  auto paths = registry_.to_openapi_paths();
  auto & content = paths["/blob"]["get"]["responses"]["200"]["content"];
  for (const char * expected : {"application/x-mcap", "application/x-sqlite3", "application/octet-stream", "*/*"}) {
    EXPECT_TRUE(content.contains(expected)) << expected << " was declared but is missing from the document";
  }
}

TEST_F(RouteRegistryTest, MultiRangeMediaTypeIsDeclaredOnThe206Only) {
  // cpp-httplib answers a multi-range request by rewriting Content-Type to
  // `multipart/byteranges` and generating a boundary (apply_ranges, the
  // req.ranges.size() > 1 branch). The 200 can never carry it, so declaring it
  // there would be the same over-declaration this document is being cleaned of.
  seed_blob(registry_, "/blob", {"application/octet-stream"}).tag("Test").summary("Download");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/blob"]["get"]["responses"];
  EXPECT_TRUE(responses["206"]["content"].contains("multipart/byteranges"));
  EXPECT_TRUE(responses["206"]["content"].contains("application/octet-stream"));
  EXPECT_FALSE(responses["200"]["content"].contains("multipart/byteranges"));
}

TEST_F(RouteRegistryTest, BinaryDownloadDeclaresTheRangeRequestHeader) {
  // The request half of the contract the 206/Content-Range/Accept-Ranges
  // declarations answer. Optional, because omitting it serves the whole body.
  seed_blob(registry_, "/blob", {"application/octet-stream"}).tag("Test").summary("Download");

  auto paths = registry_.to_openapi_paths();
  const auto & params = paths["/blob"]["get"]["parameters"];
  bool found = false;
  for (const auto & p : params) {
    if (p.value("name", "") == "Range") {
      found = true;
      EXPECT_EQ(p.value("in", ""), "header");
      EXPECT_FALSE(p.value("required", true)) << "a download without Range must still serve the whole body";
      EXPECT_FALSE(p.value("description", "").empty());
    }
  }
  EXPECT_TRUE(found) << "Accept-Ranges is advertised but no Range parameter is declared";
}

TEST_F(RouteRegistryTest, NonJsonResponseGivenASchemaReportsItInsteadOfPublishingIt) {
  // The overload takes a schema argument for symmetry with the JSON ones. A
  // caller that passes one has misunderstood the contract, and the schema is
  // dropped rather than attached to a media type it may not describe - so the
  // miscall must surface through the same channel as the other route-metadata
  // defects, not vanish.
  registry_
      .get<RouteRegistryTestSeedDto>("/thing",
                                     std::function<Result<RouteRegistryTestSeedDto>(TypedRequest)>(&seed_get_handler))
      .tag("Test")
      .summary("Thing")
      .response(200, "Bytes", json{{"type", "string"}}, {"application/octet-stream"});

  EXPECT_TRUE(has_error_mentioning(registry_, "dropped the schema"));
  auto paths = registry_.to_openapi_paths();
  EXPECT_FALSE(paths["/thing"]["get"]["responses"]["200"]["content"]["application/octet-stream"].contains("schema"));
}

TEST_F(RouteRegistryTest, NonJsonSuccessSatisfiesTheSchemaCompletenessCheck) {
  // The completeness gate reads the declared media type rather than sniffing
  // the summary for the word "stream", so an SSE or download route is complete
  // because of what it declares, not because of what it is called.
  seed_blob(registry_, "/blob", {"application/octet-stream"}).tag("Test").summary("Download");

  EXPECT_FALSE(has_error_mentioning(registry_, "Missing response schema"));
}

TEST_F(RouteRegistryTest, SchemaLessJsonSuccessIsStillReported) {
  // The exemption is for a body the media type already describes, so it turns
  // on the type being non-JSON - not merely on `content_types` being set. A 2xx
  // declaring `application/json` with no schema is exactly what the check
  // exists to catch, and accepting it would make the C++ gate looser than both
  // the documented rule and the served-document gate in test_health.
  registry_
      .get<RouteRegistryTestSeedDto>("/thing",
                                     std::function<Result<RouteRegistryTestSeedDto>(TypedRequest)>(&seed_get_handler))
      .tag("Test")
      .summary("Thing")
      .response(200, "A JSON body with no shape", json{}, {"application/json"});

  EXPECT_TRUE(has_error_mentioning(registry_, "Missing response schema"));
}

TEST_F(RouteRegistryTest, MixedMediaTypeSuccessIsExemptOnTheNonJsonEntry) {
  // ...and one non-JSON entry alongside JSON is still exempt, matching how the
  // served-document gate walks the content map. Without this the tightening
  // above could be over-read into "any mention of application/json disqualifies
  // the route", which would report a route that legitimately serves both.
  seed_blob(registry_, "/blob", {"application/json", "application/octet-stream"}).tag("Test").summary("Download");

  EXPECT_FALSE(has_error_mentioning(registry_, "Missing response schema"));
}

TEST_F(RouteRegistryTest, AJsonRouteWithNoSchemaIsStillReported) {
  // ...and the exemption must not have widened into "any 2xx is fine". A route
  // declaring a 200 with neither a schema nor a media type is the case the
  // check exists for.
  registry_.raw("get", "/plain", [](const httplib::Request &, httplib::Response &) {})
      .tag("Test")
      .summary("Plain")
      .response(200, "Something");

  EXPECT_TRUE(has_error_mentioning(registry_, "Missing response schema"));
}

TEST_F(RouteRegistryTest, EveryDocumentedRouteDeclaresTheFrameworkAnsweredRangeRejection) {
  // 416 comes from cpp-httplib's own pre-routing Range parsing
  // (Server::process_request, vendored httplib.h:6616-6622), so it is reachable
  // on every operation - including paths that do not exist - and no handler
  // ever runs. That puts it outside what the status recorder can observe, which
  // is why this framework-level constant is pinned here instead.
  //
  // It refs GenericError like the other error statuses. Note that the vendored
  // header alone does NOT show that: cpp-httplib writes 416 with an empty body,
  // and it is the gateway's own `set_error_handler`
  // (RESTServer::setup_global_error_handlers) that fills every body-less error
  // response with a GenericError. This registry harness installs no such
  // handler, so a wire assertion made *here* would show an empty body and
  // mislead; the wire half lives in the integration suite, against a real
  // gateway.
  seed_get(registry_, "/health").tag("Server").summary("Health");
  seed_post(registry_, "/items").tag("Test").summary("Create");
  seed_del(registry_, "/items/{item_id}").tag("Test").summary("Delete");

  auto paths = registry_.to_openapi_paths();
  for (const auto & [path, item] : paths.items()) {
    for (const auto & [method, op] : item.items()) {
      ASSERT_TRUE(op["responses"].contains("416")) << method << " " << path << " does not declare 416";
      EXPECT_EQ(op["responses"]["416"].value("$ref", ""), "#/components/responses/GenericError")
          << method << " " << path << ": 416 carries the same body as every other error status";
    }
  }
}

TEST_F(RouteRegistryTest, TheRangeRejectionSurvivesOnlyStatus) {
  // only_status() says the *handler* has one outcome. 416 is decided before the
  // handler is reached, so - like the auth middleware's 401/403 - it is outside
  // that guard. Clearing it here would document the 501 stubs as unable to
  // answer a status they demonstrably do answer.
  seed_get(registry_, "/stub").tag("Test").summary("Stub").only_status(501, "Not implemented");

  auto paths = registry_.to_openapi_paths();
  const auto & responses = paths["/stub"]["get"]["responses"];
  EXPECT_TRUE(responses.contains("416"));
  EXPECT_TRUE(responses.contains("501"));
  EXPECT_FALSE(responses.contains("400")) << "only_status must still suppress the blanket handler statuses";
}
