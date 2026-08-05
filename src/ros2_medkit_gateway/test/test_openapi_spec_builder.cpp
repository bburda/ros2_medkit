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

#include <string>

#include "../src/openapi/openapi_spec_builder.hpp"

using ros2_medkit_gateway::openapi::OpenApiSpecBuilder;

class OpenApiSpecBuilderTest : public ::testing::Test {
 protected:
  OpenApiSpecBuilder builder_;
};

// =============================================================================
// Basic document structure
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, BuildProducesOpenApi310Version) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("Test API", "1.0.0").build();
  ASSERT_TRUE(spec.contains("openapi"));
  EXPECT_EQ(spec["openapi"], "3.1.0");
}

TEST_F(OpenApiSpecBuilderTest, BuildContainsInfoBlock) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("ros2_medkit Gateway", "0.5.0").build();
  ASSERT_TRUE(spec.contains("info"));
  EXPECT_EQ(spec["info"]["title"], "ros2_medkit Gateway");
  EXPECT_EQ(spec["info"]["version"], "0.5.0");
}

TEST_F(OpenApiSpecBuilderTest, BuildContainsSovdVersion) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("API", "1.0.0").sovd_version("R24-11").build();
  ASSERT_TRUE(spec["info"].contains("x-sovd-version"));
  EXPECT_EQ(spec["info"]["x-sovd-version"], "R24-11");
}

TEST_F(OpenApiSpecBuilderTest, BuildWithoutSovdVersionOmitsExtension) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_FALSE(spec["info"].contains("x-sovd-version"));
}

TEST_F(OpenApiSpecBuilderTest, DescriptionAppearsInInfoBlock) {
  auto spec = builder_.info("API", "1.0.0").description("A test API description.").build();
  ASSERT_TRUE(spec["info"].contains("description"));
  EXPECT_EQ(spec["info"]["description"], "A test API description.");
}

TEST_F(OpenApiSpecBuilderTest, NoDescriptionOmitsField) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_FALSE(spec["info"].contains("description"));
}

TEST_F(OpenApiSpecBuilderTest, TagsArrayAppearsInSpec) {
  using ros2_medkit_gateway::openapi::TagInfo;
  auto spec = builder_.info("API", "1.0.0").tags({{"Server", "Server endpoints"}, {"Data", "Data access"}}).build();
  ASSERT_TRUE(spec.contains("tags"));
  ASSERT_TRUE(spec["tags"].is_array());
  ASSERT_EQ(spec["tags"].size(), 2u);
  EXPECT_EQ(spec["tags"][0]["name"], "Server");
  EXPECT_EQ(spec["tags"][0]["description"], "Server endpoints");
  EXPECT_EQ(spec["tags"][1]["name"], "Data");
  EXPECT_EQ(spec["tags"][1]["description"], "Data access");
}

TEST_F(OpenApiSpecBuilderTest, NoTagsOmitsField) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_FALSE(spec.contains("tags"));
}

// =============================================================================
// Servers
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, BuildContainsServersArray) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("API", "1.0.0").server("http://localhost:8080/api/v1").build();
  ASSERT_TRUE(spec.contains("servers"));
  ASSERT_TRUE(spec["servers"].is_array());
  ASSERT_EQ(spec["servers"].size(), 1u);
  EXPECT_EQ(spec["servers"][0]["url"], "http://localhost:8080/api/v1");
}

TEST_F(OpenApiSpecBuilderTest, ServerWithDescription) {
  auto spec = builder_.info("API", "1.0.0").server("http://localhost:8080/api/v1", "Local gateway").build();
  EXPECT_EQ(spec["servers"][0]["description"], "Local gateway");
}

TEST_F(OpenApiSpecBuilderTest, ServerWithoutDescriptionOmitsField) {
  auto spec = builder_.info("API", "1.0.0").server("http://localhost:8080/api/v1").build();
  EXPECT_FALSE(spec["servers"][0].contains("description"));
}

TEST_F(OpenApiSpecBuilderTest, MultipleServers) {
  auto spec = builder_.info("API", "1.0.0")
                  .server("http://localhost:8080/api/v1", "Local")
                  .server("https://robot.local:8443/api/v1", "Production")
                  .build();
  ASSERT_EQ(spec["servers"].size(), 2u);
  EXPECT_EQ(spec["servers"][0]["url"], "http://localhost:8080/api/v1");
  EXPECT_EQ(spec["servers"][1]["url"], "https://robot.local:8443/api/v1");
}

TEST_F(OpenApiSpecBuilderTest, NoServersProducesEmptyArray) {
  auto spec = builder_.info("API", "1.0.0").build();
  ASSERT_TRUE(spec.contains("servers"));
  EXPECT_TRUE(spec["servers"].is_array());
  EXPECT_TRUE(spec["servers"].empty());
}

// =============================================================================
// Paths
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, BuildContainsPathsObject) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("API", "1.0.0").build();
  ASSERT_TRUE(spec.contains("paths"));
  EXPECT_TRUE(spec["paths"].is_object());
}

TEST_F(OpenApiSpecBuilderTest, AddPathsMergesIntoPaths) {
  nlohmann::json paths1 = {{"/areas", {{"get", {{"summary", "List areas"}}}}}};
  nlohmann::json paths2 = {{"/components", {{"get", {{"summary", "List components"}}}}}};

  auto spec = builder_.info("API", "1.0.0").add_paths(paths1).add_paths(paths2).build();
  ASSERT_TRUE(spec["paths"].contains("/areas"));
  ASSERT_TRUE(spec["paths"].contains("/components"));
  EXPECT_EQ(spec["paths"]["/areas"]["get"]["summary"], "List areas");
  EXPECT_EQ(spec["paths"]["/components"]["get"]["summary"], "List components");
}

TEST_F(OpenApiSpecBuilderTest, AddPathsMergesOverlappingPaths) {
  nlohmann::json paths1 = {{"/areas", {{"get", {{"summary", "Old"}}}}}};
  nlohmann::json paths2 = {{"/areas", {{"get", {{"summary", "New"}}}}}};

  auto spec = builder_.info("API", "1.0.0").add_paths(paths1).add_paths(paths2).build();
  // Later add_paths should overwrite (json merge_patch behavior)
  EXPECT_EQ(spec["paths"]["/areas"]["get"]["summary"], "New");
}

TEST_F(OpenApiSpecBuilderTest, EmptyPathsWhenNoneAdded) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_TRUE(spec["paths"].empty());
}

// =============================================================================
// Components / Schemas
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, BuildContainsComponentsSchemas) {
  // @verifies REQ_INTEROP_002
  nlohmann::json schemas = {{"EntityDetail", {{"type", "object"}}}};
  auto spec = builder_.info("API", "1.0.0").add_schemas(schemas).build();
  ASSERT_TRUE(spec.contains("components"));
  ASSERT_TRUE(spec["components"].contains("schemas"));
  ASSERT_TRUE(spec["components"]["schemas"].contains("EntityDetail"));
}

TEST_F(OpenApiSpecBuilderTest, AddSchemasMergesMultipleCalls) {
  nlohmann::json schemas1 = {{"EntityDetail", {{"type", "object"}}}};
  nlohmann::json schemas2 = {{"FaultItem", {{"type", "object"}}}};

  auto spec = builder_.info("API", "1.0.0").add_schemas(schemas1).add_schemas(schemas2).build();
  EXPECT_TRUE(spec["components"]["schemas"].contains("EntityDetail"));
  EXPECT_TRUE(spec["components"]["schemas"].contains("FaultItem"));
}

// =============================================================================
// GenericError response always present
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, AlwaysIncludesGenericErrorResponse) {
  // @verifies REQ_INTEROP_002
  auto spec = builder_.info("API", "1.0.0").build();
  ASSERT_TRUE(spec["components"].contains("responses"));
  ASSERT_TRUE(spec["components"]["responses"].contains("GenericError"));

  auto & error_resp = spec["components"]["responses"]["GenericError"];
  ASSERT_TRUE(error_resp.contains("description"));
  ASSERT_TRUE(error_resp.contains("content"));
  // Schema is a $ref to components/schemas/GenericError (defined once, referenced everywhere)
  auto & schema = error_resp["content"]["application/json"]["schema"];
  EXPECT_TRUE(schema.contains("$ref"));
  EXPECT_EQ(schema["$ref"].get<std::string>(), "#/components/schemas/GenericError");
}

// =============================================================================
// Middleware-owned responses always present
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, AlwaysIncludesMiddlewareOwnedResponses) {
  // The auth and rate-limit middleware answer 401/403/429 ahead of routing, so
  // no route's return type can describe them and no RouteEntry can carry their
  // headers. Every document defines them, which is also what keeps the
  // per-route $refs to them resolvable.
  auto spec = builder_.info("API", "1.0.0").build();
  auto & responses = spec["components"]["responses"];

  ASSERT_TRUE(responses.contains("Unauthorized"));
  EXPECT_TRUE(responses["Unauthorized"]["headers"].contains("WWW-Authenticate"));

  ASSERT_TRUE(responses.contains("Forbidden"));
  EXPECT_FALSE(responses["Forbidden"]["description"].get<std::string>().empty());

  ASSERT_TRUE(responses.contains("RateLimited"));
  for (const char * header : {"Retry-After", "X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"}) {
    EXPECT_TRUE(responses["RateLimited"]["headers"].contains(header)) << header;
  }

  // Every one of them refs a schema this document also defines, so a generated
  // client can resolve the error body.
  for (const char * name : {"Unauthorized", "Forbidden", "RateLimited"}) {
    const auto ref = responses[name]["content"]["application/json"]["schema"]["$ref"].get<std::string>();
    const auto schema_name = ref.substr(ref.rfind('/') + 1);
    EXPECT_TRUE(spec["components"]["schemas"].contains(schema_name)) << name << " -> " << ref;
  }
}

// =============================================================================
// Contact info
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, ContactInfoAppearsInSpec) {
  auto spec = builder_.info("API", "1.0.0").contact("selfpatch.ai", "https://selfpatch.ai").build();

  ASSERT_TRUE(spec["info"].contains("contact"));
  EXPECT_EQ(spec["info"]["contact"]["name"].get<std::string>(), "selfpatch.ai");
  EXPECT_EQ(spec["info"]["contact"]["url"].get<std::string>(), "https://selfpatch.ai");
}

TEST_F(OpenApiSpecBuilderTest, NoContactWhenNotSet) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_FALSE(spec["info"].contains("contact"));
}

// =============================================================================
// Security scheme
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, SecuritySchemeAddsToComponents) {
  // @verifies REQ_INTEROP_002
  nlohmann::json bearer_scheme = {{"type", "http"}, {"scheme", "bearer"}, {"bearerFormat", "JWT"}};
  auto spec = builder_.info("API", "1.0.0").security_scheme("bearerAuth", bearer_scheme).build();

  ASSERT_TRUE(spec["components"].contains("securitySchemes"));
  ASSERT_TRUE(spec["components"]["securitySchemes"].contains("bearerAuth"));
  EXPECT_EQ(spec["components"]["securitySchemes"]["bearerAuth"]["scheme"], "bearer");
}

TEST_F(OpenApiSpecBuilderTest, SecuritySchemeAddsSecurity) {
  nlohmann::json bearer_scheme = {{"type", "http"}, {"scheme", "bearer"}, {"bearerFormat", "JWT"}};
  auto spec = builder_.info("API", "1.0.0").security_scheme("bearerAuth", bearer_scheme).build();

  ASSERT_TRUE(spec.contains("security"));
  ASSERT_TRUE(spec["security"].is_array());
  ASSERT_EQ(spec["security"].size(), 1u);
  ASSERT_TRUE(spec["security"][0].contains("bearerAuth"));
}

TEST_F(OpenApiSpecBuilderTest, NoSecuritySchemeOmitsSecurityField) {
  auto spec = builder_.info("API", "1.0.0").build();
  EXPECT_FALSE(spec.contains("security"));
}

TEST_F(OpenApiSpecBuilderTest, MultipleSecuritySchemes) {
  nlohmann::json bearer = {{"type", "http"}, {"scheme", "bearer"}};
  nlohmann::json api_key = {{"type", "apiKey"}, {"in", "header"}, {"name", "X-API-Key"}};
  auto spec =
      builder_.info("API", "1.0.0").security_scheme("bearerAuth", bearer).security_scheme("apiKey", api_key).build();

  ASSERT_TRUE(spec["components"]["securitySchemes"].contains("bearerAuth"));
  ASSERT_TRUE(spec["components"]["securitySchemes"].contains("apiKey"));
  ASSERT_EQ(spec["security"].size(), 2u);
}

// =============================================================================
// Builder chaining (fluent interface)
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, FluentInterfaceChaining) {
  // @verifies REQ_INTEROP_002
  nlohmann::json paths = {{"/health", {{"get", {{"summary", "Health check"}}}}}};
  nlohmann::json schemas = {{"Health", {{"type", "object"}}}};
  nlohmann::json bearer = {{"type", "http"}, {"scheme", "bearer"}};

  auto spec = builder_.info("ros2_medkit Gateway", "0.5.0")
                  .sovd_version("R24-11")
                  .server("http://localhost:8080/api/v1", "Local gateway")
                  .add_paths(paths)
                  .add_schemas(schemas)
                  .security_scheme("bearerAuth", bearer)
                  .build();

  EXPECT_EQ(spec["openapi"], "3.1.0");
  EXPECT_EQ(spec["info"]["title"], "ros2_medkit Gateway");
  EXPECT_EQ(spec["info"]["version"], "0.5.0");
  EXPECT_EQ(spec["info"]["x-sovd-version"], "R24-11");
  ASSERT_EQ(spec["servers"].size(), 1u);
  EXPECT_TRUE(spec["paths"].contains("/health"));
  EXPECT_TRUE(spec["components"]["schemas"].contains("Health"));
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
  EXPECT_TRUE(spec["components"]["securitySchemes"].contains("bearerAuth"));
  ASSERT_EQ(spec["security"].size(), 1u);
}

// =============================================================================
// Multiple builds (builder is reusable)
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, BuildCanBeCalledMultipleTimes) {
  builder_.info("API", "1.0.0");
  auto spec1 = builder_.build();
  auto spec2 = builder_.build();
  EXPECT_EQ(spec1, spec2);
}

// =============================================================================
// Edge cases
// =============================================================================

TEST_F(OpenApiSpecBuilderTest, MinimalBuildHasRequiredFields) {
  // Even without calling info(), build() should produce a valid document structure
  auto spec = builder_.build();
  EXPECT_EQ(spec["openapi"], "3.1.0");
  EXPECT_TRUE(spec.contains("info"));
  EXPECT_TRUE(spec.contains("paths"));
  EXPECT_TRUE(spec.contains("servers"));
  EXPECT_TRUE(spec.contains("components"));
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
}

TEST_F(OpenApiSpecBuilderTest, AddEmptyPaths) {
  nlohmann::json empty_paths = nlohmann::json::object();
  auto spec = builder_.info("API", "1.0.0").add_paths(empty_paths).build();
  EXPECT_TRUE(spec["paths"].empty());
}

TEST_F(OpenApiSpecBuilderTest, AddEmptySchemas) {
  nlohmann::json empty_schemas = nlohmann::json::object();
  auto spec = builder_.info("API", "1.0.0").add_schemas(empty_schemas).build();
  // Should still have GenericError in responses but schemas should be absent
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
}

// @verifies REQ_INTEROP_002
TEST_F(OpenApiSpecBuilderTest, GenericErrorSchemaAlwaysPresent) {
  auto spec = builder_.info("Test", "1.0").server("http://localhost").build();
  // GenericError schema is always emitted (needed by the GenericError response $ref)
  ASSERT_TRUE(spec.contains("components"));
  ASSERT_TRUE(spec["components"].contains("schemas"));
  EXPECT_TRUE(spec["components"]["schemas"].contains("GenericError"));
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
}

// @verifies REQ_INTEROP_002
TEST_F(OpenApiSpecBuilderTest, NonEmptySchemasArePresentInOutput) {
  nlohmann::json test_schemas = {{"MyType", {{"type", "object"}}}};
  auto spec = builder_.info("Test", "1.0").server("http://localhost").add_schemas(test_schemas).build();
  ASSERT_TRUE(spec["components"].contains("schemas"));
  EXPECT_TRUE(spec["components"]["schemas"].contains("MyType"));
}
