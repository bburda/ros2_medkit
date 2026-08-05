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

// Populate a RouteRegistry with representative routes matching what the real
// gateway registers, so that generate_root() produces the paths the tests expect.
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
    seed_get_no_summary(reg, entity_path + "/operations", "Operations");
    seed_get_no_summary(reg, entity_path + "/configurations", "Configuration");
    seed_get_no_summary(reg, entity_path + "/faults", "Faults");
    seed_get_no_summary(reg, entity_path + "/logs", "Logs");
    seed_get_no_summary(reg, entity_path + "/bulk-data", "Bulk Data");
    seed_get_no_summary(reg, entity_path + "/cyclic-subscriptions", "Subscriptions");
  }

  seed_get(reg, "/faults", "Faults", "List all faults globally");
  seed_get(reg, "/faults/stream", "Faults", "Stream fault events (SSE)");
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
// It is driven by `EntityCapabilities::for_type`, and `/data-lists`, `/modes`
// and `/updates` used to reach it and be published with a fabricated 200: no
// entity type registers them.
//
// What this pins is the generator's own half of the guarantee. The switch in
// `add_resource_collection_paths` is exhaustive and gives the four unrouted
// collections an empty arm, so even a capability list that named one again
// would produce no path here - `EntityCapabilities.NoEntityAdvertises...`
// covers the list itself, and the two together are what keep both surfaces
// clean.
TEST_F(CapabilityGeneratorTest, EntityDocumentOffersNoCollectionWithoutARoute) {
  const auto & cache = node_->get_thread_safe_cache();
  auto components = cache.get_components();
  ASSERT_FALSE(components.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + components.front().id;

  auto result = generator_->generate(entity_path);
  ASSERT_TRUE(result.has_value());

  for (const auto * phantom : {"/data-lists", "/modes", "/updates", "/communication-logs"}) {
    EXPECT_FALSE(result->at("paths").contains(entity_path + phantom)) << phantom;
  }
  // ... while the collections the loop does register are still offered.
  for (const auto * served : {"/data", "/data-categories", "/data-groups", "/operations", "/configurations", "/faults",
                              "/logs", "/bulk-data", "/triggers", "/cyclic-subscriptions"}) {
    EXPECT_TRUE(result->at("paths").contains(entity_path + served)) << served;
  }
}

// data-categories and data-groups are registered with `.only_status(501, ...)`,
// so the sub-document has to say 501 and nothing else. The generic listing it
// used to fall through to declared a 200 no client can ever observe.
TEST_F(CapabilityGeneratorTest, NotImplementedCollectionsDeclareOnly501) {
  const auto & cache = node_->get_thread_safe_cache();
  auto components = cache.get_components();
  ASSERT_FALSE(components.empty()) << "no component discovered - the assertions below would prove nothing";
  const std::string entity_path = "/components/" + components.front().id;

  auto result = generator_->generate(entity_path);
  ASSERT_TRUE(result.has_value());

  for (const auto * col : {"/data-categories", "/data-groups"}) {
    const auto & op = result->at("paths").at(entity_path + col).at("get");
    EXPECT_EQ(op.at("responses").size(), 1u) << col;
    EXPECT_TRUE(op.at("responses").contains("501")) << col;
    EXPECT_EQ(op.at("responses").at("501").at("$ref"), "#/components/responses/GenericError") << col;
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
