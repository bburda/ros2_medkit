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
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>

#include "../src/openapi/capability_generator.hpp"
#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/version.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::openapi;

using namespace std::chrono_literals;

namespace {

// Typed seed handler - never invoked by CapabilityGenerator tests; the
// generator only consumes the route's path / tag / summary metadata.
http::Result<dto::Health> noop_cap_handler(http::TypedRequest /*req*/) {
  return dto::Health{};
}

void seed_get(RouteRegistry & reg, const std::string & path, const std::string & tag, const std::string & summary) {
  std::function<http::Result<dto::Health>(http::TypedRequest)> h = &noop_cap_handler;
  reg.get<dto::Health>(path, std::move(h)).tag(tag).summary(summary);
}

void seed_get_no_summary(RouteRegistry & reg, const std::string & path, const std::string & tag) {
  std::function<http::Result<dto::Health>(http::TypedRequest)> h = &noop_cap_handler;
  reg.get<dto::Health>(path, std::move(h)).tag(tag);
}

/// A route whose only outcome is 501, the shape `/data-categories` and
/// `/data-groups` are registered with in `rest_server.cpp`.
void seed_get_501(RouteRegistry & reg, const std::string & path, const std::string & tag) {
  std::function<http::Result<dto::Health>(http::TypedRequest)> h = &noop_cap_handler;
  reg.get<dto::Health>(path, std::move(h)).tag(tag).only_status(501, "Not implemented for ROS 2");
}

// Populate a RouteRegistry with representative routes matching what the real
// gateway registers. The sub-documents are a projection of this registry, so
// what it holds is what the tests below can observe - a collection missing
// here is missing from every sub-document, which is the property the
// `EntityDocumentOffersNoCollectionWithoutARoute` case turns into an
// assertion.
void populate_test_routes(RouteRegistry & reg) {
  seed_get(reg, "/health", "Server", "Health check");
  seed_get(reg, "/", "Server", "API overview");
  seed_get(reg, "/version-info", "Server", "SOVD version information");

  for (const auto * et : {"areas", "components", "apps", "functions"}) {
    std::string base = std::string("/") + et;
    std::string singular = et;
    if (!singular.empty() && singular.back() == 's') {
      singular.pop_back();
    }
    std::string entity_path = base;
    entity_path.append("/{").append(singular).append("_id}");

    seed_get(reg, base, "Discovery", std::string("List ") + et);
    seed_get(reg, entity_path, "Discovery", std::string("Get ") + singular);
    seed_get_no_summary(reg, entity_path + "/data", "Data");
    seed_get_no_summary(reg, entity_path + "/data/{data_id}", "Data");
    seed_get_501(reg, entity_path + "/data-categories", "Data");
    seed_get_501(reg, entity_path + "/data-groups", "Data");
    seed_get_no_summary(reg, entity_path + "/operations", "Operations");
    seed_get_no_summary(reg, entity_path + "/operations/{operation_id}", "Operations");
    seed_get_no_summary(reg, entity_path + "/configurations", "Configuration");
    seed_get_no_summary(reg, entity_path + "/faults", "Faults");
    seed_get_no_summary(reg, entity_path + "/faults/{fault_code}", "Faults");
    seed_get_no_summary(reg, entity_path + "/logs", "Logs");
    seed_get_no_summary(reg, entity_path + "/logs/configuration", "Logs");
    seed_get_no_summary(reg, entity_path + "/bulk-data", "Bulk Data");
    seed_get_no_summary(reg, entity_path + "/triggers", "Triggers");
    seed_get_no_summary(reg, entity_path + "/cyclic-subscriptions", "Subscriptions");
  }

  seed_get(reg, "/faults", "Faults", "List all faults globally");
  seed_get(reg, "/faults/stream", "Faults", "Stream fault events (SSE)");
}

/// The id of some component the running node discovered, or an empty string.
std::string first_component_id(const GatewayNode & node) {
  const auto & cache = node.get_thread_safe_cache();
  auto components = cache.get_components();
  return components.empty() ? std::string{} : components.front().id;
}

}  // namespace

// =============================================================================
// Test fixture - creates a full GatewayNode for integration-level tests
// =============================================================================

class CapabilityGeneratorTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<GatewayNode>();

    // Wait briefly for node initialization
    std::this_thread::sleep_for(100ms);

    CorsConfig cors_config;
    AuthConfig auth_config;
    TlsConfig tls_config;

    ctx_ = std::make_unique<handlers::HandlerContext>(node_.get(), cors_config, auth_config, tls_config, nullptr);

    // Create a test route registry with representative routes
    route_registry_ = std::make_unique<RouteRegistry>();
    populate_test_routes(*route_registry_);

    generator_ =
        std::make_unique<CapabilityGenerator>(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());
  }

  void TearDown() override {
    generator_.reset();
    route_registry_.reset();
    ctx_.reset();
    node_.reset();
  }

  std::shared_ptr<GatewayNode> node_;
  std::unique_ptr<handlers::HandlerContext> ctx_;
  std::unique_ptr<RouteRegistry> route_registry_;
  std::unique_ptr<CapabilityGenerator> generator_;
};

// =============================================================================
// Root path generation
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootReturnsValidOpenApiSpec) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  // Verify OpenAPI 3.1.0 version
  ASSERT_TRUE(spec.contains("openapi"));
  EXPECT_EQ(spec["openapi"], "3.1.0");

  // Verify info block
  ASSERT_TRUE(spec.contains("info"));
  EXPECT_EQ(spec["info"]["title"], "ROS 2 Medkit Gateway");
  EXPECT_EQ(spec["info"]["version"], ros2_medkit_gateway::kGatewayVersion);
  ASSERT_TRUE(spec["info"].contains("x-sovd-version"));
  EXPECT_EQ(spec["info"]["x-sovd-version"], "1.0.0");

  // Verify servers array
  ASSERT_TRUE(spec.contains("servers"));
  ASSERT_TRUE(spec["servers"].is_array());
  ASSERT_FALSE(spec["servers"].empty());

  // Verify paths object exists
  ASSERT_TRUE(spec.contains("paths"));
  ASSERT_TRUE(spec["paths"].is_object());
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsHealthEndpoint) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec["paths"].contains("/health"));
  ASSERT_TRUE(spec["paths"]["/health"].contains("get"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsVersionInfo) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec["paths"].contains("/version-info"));
  ASSERT_TRUE(spec["paths"]["/version-info"].contains("get"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsEntityCollections) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas"));
  EXPECT_TRUE(spec["paths"].contains("/components"));
  EXPECT_TRUE(spec["paths"].contains("/apps"));
  EXPECT_TRUE(spec["paths"].contains("/functions"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsEntityDetailPaths) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas/{area_id}"));
  EXPECT_TRUE(spec["paths"].contains("/components/{component_id}"));
  EXPECT_TRUE(spec["paths"].contains("/apps/{app_id}"));
  EXPECT_TRUE(spec["paths"].contains("/functions/{function_id}"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsGlobalFaults) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/faults"));
  EXPECT_TRUE(spec["paths"].contains("/faults/stream"));
}

TEST_F(CapabilityGeneratorTest, GenerateRootContainsGenericErrorResponse) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec.contains("components"));
  ASSERT_TRUE(spec["components"].contains("responses"));
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
}

// =============================================================================
// Empty path treated as root
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateEmptyPathReturnsRoot) {
  auto result = generator_->generate("");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_EQ(spec["openapi"], "3.1.0");
  EXPECT_TRUE(spec["paths"].contains("/health"));
}

// =============================================================================
// Invalid / unresolvable paths
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateInvalidPathReturnsNullopt) {
  auto result = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateReservedSegmentReturnsNullopt) {
  auto result = generator_->generate("/apps/docs");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateSingleInvalidSegmentReturnsNullopt) {
  auto result = generator_->generate("/foobar");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Entity collection generation
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateEntityCollectionReturnsSpec) {
  auto result = generator_->generate("/areas");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_EQ(spec["openapi"], "3.1.0");
  ASSERT_TRUE(spec["paths"].contains("/areas"));
}

TEST_F(CapabilityGeneratorTest, GenerateComponentCollectionReturnsSpec) {
  auto result = generator_->generate("/components");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/components"));
}

TEST_F(CapabilityGeneratorTest, GenerateAppCollectionReturnsSpec) {
  auto result = generator_->generate("/apps");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/apps"));
}

TEST_F(CapabilityGeneratorTest, GenerateFunctionCollectionReturnsSpec) {
  auto result = generator_->generate("/functions");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/functions"));
}

// =============================================================================
// Specific entity - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateNonexistentEntityReturnsNullopt) {
  // Entity doesn't exist in cache
  auto result = generator_->generate("/apps/nonexistent_app_12345");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateNonexistentComponentReturnsNullopt) {
  auto result = generator_->generate("/components/nonexistent_component_xyz");
  EXPECT_FALSE(result.has_value());
}

// The entity sub-document is the second surface that advertises an entity's
// collections - the `capabilities` array on the detail response is the first.
//
// Since the sub-document became a projection of the routes the gateway
// registers, the guarantee is structural rather than a matter of keeping two
// lists agreeing: a collection appears here exactly when a route answers it.
// `/data-lists`, `/modes`, `/updates` and `/communication-logs` used to be
// published with a fabricated 200 while no route served any of them.
TEST_F(CapabilityGeneratorTest, EntityDocumentOffersNoCollectionWithoutARoute) {
  const std::string component_id = first_component_id(*node_);
  ASSERT_FALSE(component_id.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + component_id;

  auto result = generator_->generate(entity_path);
  ASSERT_TRUE(result.has_value());

  for (const auto * phantom : {"/data-lists", "/modes", "/updates", "/communication-logs"}) {
    EXPECT_FALSE(result->at("paths").contains(entity_path + phantom)) << phantom;
  }
  // ... while every collection the registry does hold is offered.
  for (const auto * served : {"/data", "/data-categories", "/data-groups", "/operations", "/configurations", "/faults",
                              "/logs", "/bulk-data", "/triggers", "/cyclic-subscriptions"}) {
    EXPECT_TRUE(result->at("paths").contains(entity_path + served)) << served;
  }
}

// data-categories and data-groups are registered with `.only_status(501, ...)`,
// so the sub-document must not offer a success a client can never observe. It
// used to fall through to a generic listing that declared a 200.
//
// The response set is not asserted to be *only* 501: the framework declares
// 416 on every operation, `only_status` or not, because cpp-httplib answers an
// unparseable `Range` before any handler runs. What must be absent is a 2xx.
TEST_F(CapabilityGeneratorTest, NotImplementedCollectionsDeclareNoSuccess) {
  const std::string component_id = first_component_id(*node_);
  ASSERT_FALSE(component_id.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + component_id;

  auto result = generator_->generate(entity_path);
  ASSERT_TRUE(result.has_value());

  for (const auto * col : {"/data-categories", "/data-groups"}) {
    const auto & responses = result->at("paths").at(entity_path + col).at("get").at("responses");
    EXPECT_TRUE(responses.contains("501")) << col;
    for (auto it = responses.begin(); it != responses.end(); ++it) {
      EXPECT_FALSE(!it.key().empty() && it.key().front() == '2') << col << " declares success " << it.key();
    }
  }
}

// Substituting the id into the path key without removing the parameter that
// described it publishes a parameter the path no longer has - a document a
// strict validator rejects and a generated client builds a call signature
// from.
TEST_F(CapabilityGeneratorTest, EntityDocumentDropsTheParameterItSubstituted) {
  const std::string component_id = first_component_id(*node_);
  ASSERT_FALSE(component_id.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + component_id;

  auto result = generator_->generate(entity_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->at("paths").contains(entity_path));

  for (const auto & [path, item] : result->at("paths").items()) {
    EXPECT_EQ(path.find("{component_id}"), std::string::npos) << path;
    for (const auto & [method, operation] : item.items()) {
      if (!operation.is_object() || !operation.contains("parameters")) {
        continue;
      }
      for (const auto & param : operation.at("parameters")) {
        EXPECT_FALSE(param.at("in") == "path" && param.at("name") == "component_id") << path << " " << method;
      }
    }
  }
  // The parameters a substitution did not answer stay: the data item route
  // still templates `{data_id}` and still declares it.
  const auto & item = result->at("paths").at(entity_path + "/data/{data_id}").at("get");
  ASSERT_TRUE(item.contains("parameters"));
  bool declares_data_id = false;
  for (const auto & param : item.at("parameters")) {
    declares_data_id = declares_data_id || (param.at("in") == "path" && param.at("name") == "data_id");
  }
  EXPECT_TRUE(declares_data_id);
}

// A prefix match on the string would let `/data-groups` and `/data-categories`
// answer under `/data`, so the data collection's sub-document would describe
// two collections it is not about.
TEST_F(CapabilityGeneratorTest, ResourceCollectionDocumentStopsAtTheSegmentBoundary) {
  const std::string component_id = first_component_id(*node_);
  ASSERT_FALSE(component_id.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + component_id;

  auto result = generator_->generate(entity_path + "/data");
  ASSERT_TRUE(result.has_value());

  const auto & paths = result->at("paths");
  EXPECT_TRUE(paths.contains(entity_path + "/data"));
  EXPECT_TRUE(paths.contains(entity_path + "/data/{data_id}"));
  EXPECT_FALSE(paths.contains(entity_path + "/data-groups"));
  EXPECT_FALSE(paths.contains(entity_path + "/data-categories"));
}

// A sub-document that names a schema it does not carry publishes a `$ref` no
// client can resolve. Every one of these documents used to do exactly that.
//
// Rooted at the whole document, not at `paths`. `referenced_schemas()` walks
// only `paths`, so a chain that leaves through `components/responses` is
// precisely the case a paths-rooted check could not see - the one this is named
// for. It happens to be safe today (the five response components
// `OpenApiSpecBuilder` always emits name only `GenericError` and `OAuth2Error`,
// which it always backfills), but "happens to be safe" is what the walk is here
// to stop depending on.
TEST_F(CapabilityGeneratorTest, SubDocumentCarriesTheSchemasItReferences) {
  const std::string component_id = first_component_id(*node_);
  ASSERT_FALSE(component_id.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + component_id;

  for (const auto & sub_path : {std::string("/components"), entity_path, entity_path + "/faults", entity_path + "/logs",
                                entity_path + "/data"}) {
    auto result = generator_->generate(sub_path);
    ASSERT_TRUE(result.has_value()) << sub_path;

    size_t refs_seen = 0;
    std::function<void(const nlohmann::json &)> check = [&](const nlohmann::json & node) {
      if (node.is_array()) {
        for (const auto & element : node) {
          check(element);
        }
        return;
      }
      if (!node.is_object()) {
        return;
      }
      for (const auto & [key, value] : node.items()) {
        static const std::string kPrefix = "#/components/";
        if (key == "$ref" && value.is_string() && value.get_ref<const std::string &>().rfind(kPrefix, 0) == 0) {
          const std::string & ref = value.get_ref<const std::string &>();
          const auto slash = ref.rfind('/');
          const std::string section = ref.substr(kPrefix.size(), slash - kPrefix.size());
          ++refs_seen;
          EXPECT_TRUE(result->at("components").contains(section) &&
                      result->at("components").at(section).contains(ref.substr(slash + 1)))
              << sub_path << " -> " << ref;
          continue;
        }
        check(value);
      }
    };
    check(*result);
    // A document whose walk found nothing would pass vacuously.
    EXPECT_GT(refs_seen, 0u) << sub_path;
  }
}

// =============================================================================
// Resource collection - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateResourceCollectionOfNonexistentEntityReturnsNullopt) {
  auto result = generator_->generate("/apps/nonexistent_app_12345/data");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Specific resource - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateSpecificResourceOfNonexistentEntityReturnsNullopt) {
  auto result = generator_->generate("/apps/nonexistent_app_12345/data/temperature");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Nested path validation
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateNestedPathWithNonexistentParentReturnsNullopt) {
  auto result = generator_->generate("/areas/nonexistent_area/components/nonexistent_comp");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Server URL construction
// =============================================================================

TEST_F(CapabilityGeneratorTest, ServerUrlContainsApiV1Prefix) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_FALSE(spec["servers"].empty());
  std::string server_url = spec["servers"][0]["url"];
  EXPECT_NE(server_url.find("/api/v1"), std::string::npos);
}

TEST_F(CapabilityGeneratorTest, ServerUrlUsesHttpProtocol) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_FALSE(spec["servers"].empty());
  std::string server_url = spec["servers"][0]["url"];
  EXPECT_EQ(server_url.substr(0, 4), "http");
}

// =============================================================================
// Entity collection generates both list and detail paths
// =============================================================================

TEST_F(CapabilityGeneratorTest, EntityCollectionContainsBothListAndDetailPaths) {
  auto result = generator_->generate("/apps");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  // Should have both the collection listing and the detail path
  EXPECT_TRUE(spec["paths"].contains("/apps"));
  EXPECT_TRUE(spec["paths"].contains("/apps/{app_id}"));
}

TEST_F(CapabilityGeneratorTest, AreaCollectionContainsBothListAndDetailPaths) {
  auto result = generator_->generate("/areas");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas"));
  EXPECT_TRUE(spec["paths"].contains("/areas/{area_id}"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, EntityCollectionDetailPathHasParameters) {
  // Entity collection specs use template paths (/apps/{app_id}), so they should
  // include path parameters for the entity ID
  auto result = generator_->generate("/apps");
  ASSERT_TRUE(result.has_value());
  auto & spec = *result;
  ASSERT_TRUE(spec["paths"].contains("/apps/{app_id}"));
  auto & detail = spec["paths"]["/apps/{app_id}"];
  ASSERT_TRUE(detail["get"].contains("parameters"));
  EXPECT_EQ(detail["get"]["parameters"][0]["name"], "app_id");
}

// =============================================================================
// Caching tests
// =============================================================================

TEST_F(CapabilityGeneratorTest, CacheReturnsSameResultForSamePath) {
  auto spec1 = generator_->generate("/apps");
  auto spec2 = generator_->generate("/apps");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheReturnsSameResultForRootPath) {
  auto spec1 = generator_->generate("/");
  auto spec2 = generator_->generate("/");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheReturnsDifferentResultsForDifferentPaths) {
  auto spec1 = generator_->generate("/apps");
  auto spec2 = generator_->generate("/components");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  // Different paths should produce different specs (different titles at minimum)
  EXPECT_NE(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheDoesNotReturnResultForInvalidPath) {
  auto spec1 = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(spec1.has_value());
  // Calling again should also return nullopt (invalid results are not cached)
  auto spec2 = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(spec2.has_value());
}

// =============================================================================
// ThreadSafeEntityCache generation counter tests (standalone, no GatewayNode)
// =============================================================================

TEST(ThreadSafeEntityCacheGenerationTest, GenerationStartsAtZero) {
  ThreadSafeEntityCache cache;
  EXPECT_EQ(cache.generation(), 0u);
}

TEST(ThreadSafeEntityCacheGenerationTest, GenerationIncrementsOnUpdateAll) {
  // Empty-into-empty is a no-op; a real change (inserting an entity) must bump.
  ThreadSafeEntityCache cache;
  Area ar;
  ar.id = "ar1";
  ar.name = "Area 1";
  cache.update_all({ar}, {}, {}, {});
  EXPECT_EQ(cache.generation(), 1u);

  // Identical second call - no-op, generation must NOT advance.
  cache.update_all({ar}, {}, {}, {});
  EXPECT_EQ(cache.generation(), 1u);
}

TEST(ThreadSafeEntityCacheGenerationTest, GenerationIncrementsOnEachUpdate) {
  // Each per-type updater bumps generation ONLY when it changes the cache.
  // We seed distinct entities per call so every call is a real change.
  ThreadSafeEntityCache cache;

  Area ar;
  ar.id = "ar1";
  ar.name = "Area 1";
  cache.update_areas({ar});
  EXPECT_EQ(cache.generation(), 1u);

  Component comp;
  comp.id = "comp1";
  comp.name = "Comp 1";
  cache.update_components({comp});
  EXPECT_EQ(cache.generation(), 2u);

  App app;
  app.id = "app1";
  app.name = "App 1";
  app.component_id = "comp1";
  cache.update_apps({app});
  EXPECT_EQ(cache.generation(), 3u);

  Function func;
  func.id = "func1";
  func.name = "Func 1";
  cache.update_functions({func});
  EXPECT_EQ(cache.generation(), 4u);

  // update_all with different data (remove the area, keep rest) - real change.
  cache.update_all({}, {comp}, {app}, {func});
  EXPECT_EQ(cache.generation(), 5u);

  // Repeat the same update_all - identical, must NOT bump.
  cache.update_all({}, {comp}, {app}, {func});
  EXPECT_EQ(cache.generation(), 5u);
}

TEST(ThreadSafeEntityCacheGenerationTest, TopicTypesUpdateIncrementsGeneration) {
  // Topic types affect OpenAPI schemas via SchemaBuilder::from_ros_msg(),
  // so changes must invalidate the spec cache.
  ThreadSafeEntityCache cache;
  cache.update_topic_types({{"topic", "std_msgs/msg/String"}});
  EXPECT_EQ(cache.generation(), 1u);
}

// =============================================================================
// Cache size limit tests
// =============================================================================

TEST_F(CapabilityGeneratorTest, RepeatedCacheHitsDoNotGrow) {
  // Verify that repeated requests for the same small set of paths
  // are served from cache without unbounded growth.
  // Note: with generation-based keys only 4 distinct entries are created,
  // so this primarily tests cache hit behavior, not eviction.
  const std::vector<std::string> entity_types = {"areas", "components", "apps", "functions"};
  for (int i = 0; i < 300; ++i) {
    auto path = "/" + entity_types[static_cast<size_t>(i) % entity_types.size()];
    auto result = generator_->generate(path);
    ASSERT_TRUE(result.has_value()) << "Failed to generate spec for: " << path;
  }
  // Four distinct paths were requested, so four entries is the whole cache.
  EXPECT_EQ(generator_->cache_entry_count(), 4u);
}

// What the cache holds is the serialized document, and the byte total it
// bounds itself with has to agree with that. Asserting the total equals the
// summed key+text lengths is what makes `cache_byte_size` a measurement
// rather than a counter that happens to go up.
TEST_F(CapabilityGeneratorTest, CacheAccountsForTheBytesItHolds) {
  const std::vector<std::string> paths = {"/", "/areas", "/apps"};
  size_t serialized = 0;
  for (const auto & path : paths) {
    auto document = generator_->generate_serialized(path);
    ASSERT_TRUE(document.has_value()) << "Failed to generate document for: " << path;
    serialized += document->size();
  }
  ASSERT_EQ(generator_->cache_entry_count(), paths.size());

  // Each key is "<generation>:<path>", so the accounted total is the documents
  // plus those keys - never less than the documents alone.
  EXPECT_GT(generator_->cache_byte_size(), serialized);
  size_t keys = 0;
  for (const auto & path : paths) {
    keys += std::to_string(node_->get_thread_safe_cache().generation()).size() + 1 + path.size();
  }
  EXPECT_EQ(generator_->cache_byte_size(), serialized + keys);
}

// The bound the entry count could not give: with documents this size, 256
// entries is tens of megabytes. Driven through the overridable budget rather
// than by generating 16 MiB of documents.
TEST_F(CapabilityGeneratorTest, ByteBudgetEvictsBeforeTheEntryCountWould) {
  auto first = generator_->generate_serialized("/areas");
  ASSERT_TRUE(first.has_value());

  // A budget that fits one document of this size but not two.
  openapi::DocsCacheBounds bounds;
  bounds.max_bytes = first->size() + 64;
  CapabilityGenerator bounded(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get(), bounds);

  ASSERT_TRUE(bounded.generate_serialized("/areas").has_value());
  ASSERT_EQ(bounded.cache_entry_count(), 1u);

  ASSERT_TRUE(bounded.generate_serialized("/apps").has_value());
  // Well under kDocsCacheMaxEntries, so only the byte budget can have evicted.
  EXPECT_EQ(bounded.cache_entry_count(), 1u);
  EXPECT_LE(bounded.cache_byte_size(), bounds.max_bytes);
}

// A single document larger than the whole budget is served but not stored -
// caching it would hold the cache over its bound indefinitely.
TEST_F(CapabilityGeneratorTest, DocumentLargerThanTheBudgetIsServedUncached) {
  openapi::DocsCacheBounds bounds;
  bounds.max_bytes = 16;
  CapabilityGenerator bounded(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get(), bounds);

  auto document = bounded.generate_serialized("/areas");
  ASSERT_TRUE(document.has_value());
  EXPECT_GT(document->size(), bounds.max_bytes);
  EXPECT_EQ(bounded.cache_entry_count(), 0u);
  EXPECT_EQ(bounded.cache_byte_size(), 0u);
}

// The entry count is the other half of the bound, and it still has to bite.
TEST_F(CapabilityGeneratorTest, EntryCountEvictsWhenTheByteBudgetIsSlack) {
  openapi::DocsCacheBounds bounds;
  bounds.max_entries = 2;
  CapabilityGenerator bounded(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get(), bounds);

  ASSERT_TRUE(bounded.generate_serialized("/areas").has_value());
  ASSERT_TRUE(bounded.generate_serialized("/apps").has_value());
  ASSERT_EQ(bounded.cache_entry_count(), 2u);

  // Third distinct path trips the clear-all, leaving just the new entry.
  ASSERT_TRUE(bounded.generate_serialized("/components").has_value());
  EXPECT_EQ(bounded.cache_entry_count(), 1u);
  EXPECT_LT(bounded.cache_byte_size(), bounds.max_bytes);
}

// The parsed accessor must describe the same document the routes serve.
TEST_F(CapabilityGeneratorTest, SerializedAndParsedFormsAgree) {
  auto document = generator_->generate_serialized("/apps");
  auto parsed = generator_->generate("/apps");
  ASSERT_TRUE(document.has_value());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*document, parsed->dump(2));
}

// =============================================================================
// Cache invalidation - verify generation-based key prevents stale results
// =============================================================================

TEST_F(CapabilityGeneratorTest, DifferentGenerationsProduceDifferentCacheKeys) {
  // First generate should work and be cacheable
  auto spec1 = generator_->generate("/");
  ASSERT_TRUE(spec1.has_value());
  EXPECT_EQ((*spec1)["openapi"], "3.1.0");

  // GatewayNode::get_thread_safe_cache() returns const ref, so we cannot directly
  // call update_areas() to trigger a generation change from outside the node.
  // The generation counter mechanism is verified separately in
  // ThreadSafeEntityCacheGenerationTest. Here we verify that the cache returns
  // identical results when the generation has not changed.
  auto spec2 = generator_->generate("/");
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ((*spec2)["openapi"], "3.1.0");

  // Specs should be identical when entity cache hasn't changed
  EXPECT_EQ(*spec1, *spec2);
}

TEST(CacheGenerationTest, GenerationCounterTracksEntityUpdates) {
  // Standalone test verifying the generation counter mechanism
  // that CapabilityGenerator depends on for cache invalidation.
  // Only REAL changes (insert / modify / remove) advance the counter;
  // identical repeated calls must not, so the spec cache stays valid
  // across no-op discovery ticks.
  ThreadSafeEntityCache cache;
  EXPECT_EQ(cache.generation(), 0u);

  Area ar;
  ar.id = "ar1";
  ar.name = "Area 1";
  cache.update_areas({ar});
  EXPECT_EQ(cache.generation(), 1u);

  Component comp;
  comp.id = "comp1";
  comp.name = "Comp 1";
  cache.update_components({comp});
  EXPECT_EQ(cache.generation(), 2u);

  // Identical repeated calls are no-ops - generation must not advance.
  cache.update_areas({ar});
  EXPECT_EQ(cache.generation(), 2u);

  cache.update_components({comp});
  EXPECT_EQ(cache.generation(), 2u);

  // A real modification (rename) DOES bump - CapabilityGenerator would
  // see a new cache key and regenerate the spec.
  ar.name = "Area 1 Renamed";
  cache.update_areas({ar});
  EXPECT_EQ(cache.generation(), 3u);
}
