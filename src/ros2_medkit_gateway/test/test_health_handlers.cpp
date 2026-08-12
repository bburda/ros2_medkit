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

#include <algorithm>
#include <arpa/inet.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/http/handlers/health_handlers.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/registry.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using namespace std::chrono_literals;

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::JsonWriter;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::HealthHandlers;
using ros2_medkit_gateway::http::TypedRequest;
using ros2_medkit_gateway::openapi::RouteRegistry;

namespace dto = ros2_medkit_gateway::dto;

// HealthHandlers has no dependency on GatewayNode or AuthManager:
// - get_health builds dto::Health (free-standing)
// - get_version_info builds dto::VersionInfo (free-standing)
// - get_root reads ctx_.auth_config() and ctx_.tls_config() (both disabled by default)
// All tests use a null GatewayNode and null AuthManager which is safe for these handlers.

namespace {

// Typed seed handlers used to populate a test route registry with the routes
// `handle_root` enumerates. The handler bodies are never invoked - the tests
// only inspect the registry's endpoint list.
ros2_medkit_gateway::http::Result<dto::Health> noop_get_health(ros2_medkit_gateway::http::TypedRequest /*req*/) {
  return dto::Health{};
}

ros2_medkit_gateway::http::Result<dto::Health> noop_post_health(ros2_medkit_gateway::http::TypedRequest /*req*/,
                                                                const dto::Health & /*body*/) {
  return dto::Health{};
}

void seed_get(RouteRegistry & reg, const std::string & path, const std::string & tag, const std::string & summary) {
  std::function<ros2_medkit_gateway::http::Result<dto::Health>(ros2_medkit_gateway::http::TypedRequest)> h =
      &noop_get_health;
  reg.get<dto::Health>(path, std::move(h)).tag(tag).summary(summary);
}

void seed_post(RouteRegistry & reg, const std::string & path, const std::string & tag) {
  std::function<ros2_medkit_gateway::http::Result<dto::Health>(ros2_medkit_gateway::http::TypedRequest, dto::Health)>
      h = &noop_post_health;
  reg.post<dto::Health, dto::Health>(path, std::move(h)).tag(tag);
}

// Populate a test route registry with representative routes
void populate_test_routes(RouteRegistry & reg) {
  seed_get(reg, "/health", "Server", "Health check");
  seed_get(reg, "/", "Server", "API overview");
  seed_get(reg, "/version-info", "Server", "SOVD version information");
  seed_get(reg, "/areas", "Discovery", "List areas");
  seed_get(reg, "/apps", "Discovery", "List apps");
  seed_get(reg, "/components", "Discovery", "List components");
  seed_get(reg, "/functions", "Discovery", "List functions");
  seed_get(reg, "/faults", "Faults", "List all faults");
}

}  // namespace

class HealthHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_config_{};
  AuthConfig auth_config_{};  // enabled = false by default
  TlsConfig tls_config_{};    // enabled = false by default
  RouteRegistry route_registry_;
  HandlerContext ctx_{nullptr, cors_config_, auth_config_, tls_config_, nullptr};
  HealthHandlers handlers_{ctx_, &route_registry_};

  httplib::Request req_;
  TypedRequest typed_req_{req_};

  void SetUp() override {
    populate_test_routes(route_registry_);
  }

  HandlerContext make_context(const AuthConfig & auth, const TlsConfig & tls) {
    return HandlerContext(nullptr, cors_config_, auth, tls, nullptr);
  }
};

// --- get_health ---

TEST_F(HealthHandlersTest, HandleHealthResponseContainsStatusHealthy) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "healthy");
}

TEST_F(HealthHandlersTest, HandleHealthNullNodeOmitsDiscovery) {
  // ctx_ uses nullptr for GatewayNode, so discovery info should not be present
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "healthy");
  EXPECT_FALSE(result->discovery.has_value());
}

TEST_F(HealthHandlersTest, HandleHealthResponseContainsTimestamp) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->timestamp, 0);
}

TEST_F(HealthHandlersTest, HandleHealthResponseIsValidJson) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  // The DTO writer must produce a valid JSON object for the success body.
  auto body = JsonWriter<dto::Health>::write(result.value());
  EXPECT_TRUE(body.is_object());
  EXPECT_EQ(body["status"], "healthy");
}

// --- get_version_info ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoContainsItemsArray) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->items.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVersionField) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  EXPECT_FALSE(result->items[0].version.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasBaseUri) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  EXPECT_FALSE(result->items[0].base_uri.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVendorInfo) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  ASSERT_TRUE(result->items[0].vendor_info.has_value());
  EXPECT_EQ(result->items[0].vendor_info->name, "ros2_medkit");
}

// --- get_root ---

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootResponseContainsRequiredTopLevelFields) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->name.empty());
  EXPECT_FALSE(result->version.empty());
  EXPECT_FALSE(result->api_base.empty());
  // endpoints + capabilities are required by the DTO schema; non-emptiness of
  // endpoints is checked by the dedicated test below.
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootEndpointsIsNonEmptyArray) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->endpoints.empty());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesContainsDiscovery) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->capabilities.discovery);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthDisabledNoAuthEndpoints) {
  // With auth disabled (default), auth endpoints must not appear in the list
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  for (const auto & ep : result->endpoints) {
    EXPECT_EQ(ep.find("/auth/"), std::string::npos) << "Unexpected auth endpoint when auth is disabled: " << ep;
  }
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesAuthDisabled) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->capabilities.authentication);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesTlsDisabled) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->capabilities.tls);
  EXPECT_FALSE(result->tls.has_value());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledAddsAuthEndpoints) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auto ctx_auth = make_context(auth_enabled, tls_config_);

  // Create registry with auth routes
  RouteRegistry auth_reg;
  populate_test_routes(auth_reg);
  seed_post(auth_reg, "/auth/authorize", "Authentication");
  seed_post(auth_reg, "/auth/token", "Authentication");
  seed_post(auth_reg, "/auth/revoke", "Authentication");

  HealthHandlers handlers_auth(ctx_auth, &auth_reg);

  auto result = handlers_auth.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  bool has_auth_endpoint = false;
  for (const auto & ep : result->endpoints) {
    if (ep.find("/auth/") != std::string::npos) {
      has_auth_endpoint = true;
      break;
    }
  }
  EXPECT_TRUE(has_auth_endpoint);
  EXPECT_TRUE(result->capabilities.authentication);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledIncludesAuthMetadataBlock) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auth_enabled.require_auth_for = ros2_medkit_gateway::AuthRequirement::ALL;
  auth_enabled.jwt_algorithm = ros2_medkit_gateway::JwtAlgorithm::HS256;
  auto ctx_auth = make_context(auth_enabled, tls_config_);
  HealthHandlers handlers_auth(ctx_auth, &route_registry_);

  auto result = handlers_auth.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  ASSERT_TRUE(result->auth.has_value());
  EXPECT_TRUE(result->auth->enabled);
  EXPECT_EQ(result->auth->algorithm, "HS256");
  EXPECT_EQ(result->auth->require_auth_for, "all");
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootTlsEnabledIncludesTlsMetadataBlock) {
  TlsConfig tls_enabled{};
  tls_enabled.enabled = true;
  tls_enabled.min_version = "1.3";
  auto ctx_tls = make_context(auth_config_, tls_enabled);
  HealthHandlers handlers_tls(ctx_tls, &route_registry_);

  auto result = handlers_tls.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  ASSERT_TRUE(result->tls.has_value());
  EXPECT_TRUE(result->tls->enabled);
  EXPECT_EQ(result->tls->min_version, "1.3");
  EXPECT_TRUE(result->capabilities.tls);
}

// --- live discovery block (requires live GatewayNode + real HTTP server) ---

static constexpr const char * API_BASE_PATH = "/api/v1";

static int reserve_free_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }
  socklen_t len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

// rclcpp is brought up once for the whole binary rather than per fixture: two
// fixtures here need a live context, and cycling init/shutdown between them
// would tear the default context down under the next suite.
class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const ::testing::Environment * kRclcppEnvironment = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

class HealthHandlersLiveTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int free_port = reserve_free_port();
    ASSERT_NE(free_port, 0) << "Failed to reserve a free port for test";

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("server.port", free_port)});
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

    server_port_ = free_port;

    // Wait for the server to be ready
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    httplib::Client client("127.0.0.1", server_port_);
    const std::string health_ep = std::string(API_BASE_PATH) + "/health";
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (auto res = client.Get(health_ep)) {
        if (res->status == 200) {
          return;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  void TearDown() override {
    node_.reset();
  }

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  int server_port_{0};
};

TEST_F(HealthHandlersLiveTest, HealthDiscoveryBlockContainsExpectedFields) {
  httplib::Client client("127.0.0.1", server_port_);
  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_EQ(body["status"], "healthy");
  EXPECT_TRUE(body.contains("timestamp"));

  // With a live GatewayNode, the discovery block must be present
  ASSERT_TRUE(body.contains("discovery"));
  auto & disc = body["discovery"];

  // Must contain mode and strategy strings
  ASSERT_TRUE(disc.contains("mode"));
  EXPECT_TRUE(disc["mode"].is_string());
  EXPECT_FALSE(disc["mode"].get<std::string>().empty());
  // Default mode is runtime_only
  EXPECT_EQ(disc["mode"].get<std::string>(), "runtime_only");

  ASSERT_TRUE(disc.contains("strategy"));
  EXPECT_TRUE(disc["strategy"].is_string());
  EXPECT_FALSE(disc["strategy"].get<std::string>().empty());

  // In runtime_only mode, pipeline and linking are not present (only in hybrid mode)
  EXPECT_FALSE(disc.contains("pipeline"));
  EXPECT_FALSE(disc.contains("linking"));
}

TEST_F(HealthHandlersLiveTest, HealthEntityCacheStatsPresent) {
  // x-medkit-entity-cache block is present with the required fields
  httplib::Client client("127.0.0.1", server_port_);
  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);

  ASSERT_TRUE(body.contains("x-medkit-entity-cache"))
      << "/health must include x-medkit-entity-cache when entity_cache.capacity is configured";
  const auto & ec = body["x-medkit-entity-cache"];

  EXPECT_TRUE(ec.contains("capacity")) << "x-medkit-entity-cache must include capacity";
  EXPECT_TRUE(ec["capacity"].is_number_unsigned());
  // Default capacity is 256 (from declare_parameter("entity_cache.capacity", 256))
  EXPECT_EQ(ec["capacity"].get<std::size_t>(), 256u);

  EXPECT_TRUE(ec.contains("areas")) << "x-medkit-entity-cache must include areas";
  EXPECT_TRUE(ec["areas"].is_number_unsigned());

  EXPECT_TRUE(ec.contains("components")) << "x-medkit-entity-cache must include components";
  EXPECT_TRUE(ec["components"].is_number_unsigned());

  EXPECT_TRUE(ec.contains("apps")) << "x-medkit-entity-cache must include apps";
  EXPECT_TRUE(ec["apps"].is_number_unsigned());

  EXPECT_TRUE(ec.contains("functions")) << "x-medkit-entity-cache must include functions";
  EXPECT_TRUE(ec["functions"].is_number_unsigned());

  EXPECT_TRUE(ec.contains("generation")) << "x-medkit-entity-cache must include generation";
  EXPECT_TRUE(ec["generation"].is_number_unsigned());

  EXPECT_TRUE(ec.contains("grew")) << "x-medkit-entity-cache must include grew";
  EXPECT_TRUE(ec["grew"].is_boolean());
  // A fresh cache with default capacity=256 must not have grown
  EXPECT_FALSE(ec["grew"].get<bool>());
}

TEST_F(HealthHandlersLiveTest, RuntimeOnlyOmitsLinkingAndTheUnmanifestedPolicyField) {
  // runtime_only has no RuntimeLinker, so there is nothing to report a policy
  // about. The policy field must not appear on its own anywhere in the body.
  httplib::Client client("127.0.0.1", server_port_);
  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  ASSERT_TRUE(body.contains("discovery"));
  EXPECT_FALSE(body["discovery"].contains("linking"));
  EXPECT_EQ(res->body.find("unmanifested_policy"), std::string::npos)
      << "runtime_only must not emit unmanifested_policy; body: " << res->body;
}

TEST_F(HealthHandlersLiveTest, WarningsArrayAndSchemaVersionPresentWithoutAggregation) {
  // Aggregation is disabled on this gateway. The warnings contract is no
  // longer gated on it: the array and its schema version are always served.
  httplib::Client client("127.0.0.1", server_port_);
  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  ASSERT_TRUE(body.contains("warnings")) << "/health must always carry a warnings array; body: " << res->body;
  EXPECT_TRUE(body["warnings"].is_array());
  EXPECT_TRUE(body["warnings"].empty()) << "a healthy runtime_only gateway has nothing to warn about";

  ASSERT_TRUE(body.contains("warning_schema_version"));
  EXPECT_EQ(body["warning_schema_version"].get<int>(), 2);
}

// ---------------------------------------------------------------------------
// Schema surface: the warning object is published under its own component
// name, and the linking object carries the configured policy.
// ---------------------------------------------------------------------------

TEST(HealthSchemaSurface, WarningIsPublishedAsHealthWarning) {
  auto schemas = ros2_medkit_gateway::dto::collect_component_schemas();

  EXPECT_FALSE(schemas.contains("HealthAggregationWarning"))
      << "the warning object is no longer aggregation-specific and must not keep the old component name";
  ASSERT_TRUE(schemas.contains("HealthWarning"));

  const auto & props = schemas["HealthWarning"]["properties"];
  EXPECT_EQ(props.size(), 5u);
  EXPECT_TRUE(props.contains("code"));
  EXPECT_TRUE(props.contains("message"));
  EXPECT_TRUE(props.contains("entity_ids"));
  EXPECT_TRUE(props.contains("ros_node_fqns"));
  EXPECT_TRUE(props.contains("peer_names"));

  // All three identifier arrays are plain arrays, never anyOf:[array,null].
  // std::optional would publish a null state the server cannot emit and give
  // every generated client a tri-state to unpack.
  for (const char * key : {"entity_ids", "ros_node_fqns", "peer_names"}) {
    ASSERT_TRUE(props.contains(key)) << key;
    EXPECT_EQ(props[key].value("type", std::string{}), "array") << key;
    EXPECT_FALSE(props[key].contains("anyOf")) << key << " must not be optional";
  }
}

TEST(HealthSchemaSurface, LinkingCarriesTheUnmanifestedPolicy) {
  auto schemas = ros2_medkit_gateway::dto::collect_component_schemas();

  ASSERT_TRUE(schemas.contains("HealthDiscoveryLinking"));
  const auto & props = schemas["HealthDiscoveryLinking"]["properties"];
  ASSERT_TRUE(props.contains("unmanifested_policy"));
  EXPECT_EQ(props["unmanifested_policy"]["type"], "string");
}

// ---------------------------------------------------------------------------
// Hybrid-mode /health: the unmanifested-node policy and its warning.
//
// Driven through a real GatewayNode over real HTTP, because the claim is about
// the served document: which policy string it reports, and whether the warning
// appears for the orphans the linker actually found. A handler test holding a
// hand-built DTO could not tell whether the handler ever reads the manifest.
//
// Discovery is advanced by calling refresh_topic_map() - the same entry point
// the gateway's own refresh timer uses - so the test does not have to run an
// executor to make a pass happen.
// ---------------------------------------------------------------------------

namespace {

constexpr const char * kBoundNodeName = "linking_bound_node";
constexpr const char * kOrphanAlpha = "linking_orphan_alpha";
constexpr const char * kOrphanBeta = "linking_orphan_beta";

std::string linking_manifest(const std::string & policy, const std::vector<std::string> & extra_bound_fqns = {},
                             const std::string & extra_config = "") {
  std::string yaml =
      "manifest_version: \"1.0\"\n"
      "metadata:\n"
      "  name: \"Health linking fixture\"\n"
      "  version: \"1.0.0\"\n"
      "config:\n"
      "  unmanifested_nodes: \"" +
      policy + "\"\n" + extra_config +
      "areas:\n"
      "  - id: linkarea\n"
      "    name: \"Linking Area\"\n"
      "components:\n"
      "  - id: linkecu\n"
      "    name: \"Linking ECU\"\n"
      "    area: linkarea\n"
      "apps:\n"
      "  - id: bound_app\n"
      "    name: \"Bound App\"\n"
      "    is_located_on: linkecu\n"
      "    ros_binding:\n"
      "      node_name: " +
      std::string(kBoundNodeName) +
      "\n"
      "      namespace: \"/\"\n";

  for (const auto & fqn : extra_bound_fqns) {
    auto slash = fqn.rfind('/');
    std::string node_name = fqn.substr(slash + 1);
    std::string ns = (slash == 0) ? "/" : fqn.substr(0, slash);
    std::string id = "app_" + fqn.substr(1);
    std::replace(id.begin(), id.end(), '/', '_');
    yaml += "  - id: " + id + "\n";
    yaml += "    name: \"" + node_name + "\"\n";
    yaml += "    is_located_on: linkecu\n";
    yaml += "    ros_binding:\n";
    yaml += "      node_name: " + node_name + "\n";
    yaml += "      namespace: \"" + ns + "\"\n";
  }
  return yaml;
}

}  // namespace

class HealthLinkingTest : public ::testing::Test {
 protected:
  void TearDown() override {
    stop_gateway();
    graph_nodes_.clear();
    for (const auto & path : manifests_) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }
    manifests_.clear();
  }

  /// Put nodes on the ROS graph. Nothing declares them unless a manifest says so.
  void spawn_nodes(const std::vector<std::string> & names) {
    for (const auto & name : names) {
      graph_nodes_.push_back(std::make_shared<rclcpp::Node>(name));
    }
  }

  void start_gateway(const std::string & manifest_yaml) {
    const auto path = write_manifest(manifest_yaml);
    server_port_ = reserve_free_port();
    ASSERT_NE(server_port_, 0) << "Failed to reserve a free port for test";

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("server.port", server_port_),
        rclcpp::Parameter("discovery.mode", std::string("hybrid")),
        rclcpp::Parameter("discovery.manifest_path", path),
        rclcpp::Parameter("discovery.manifest_strict_validation", false),
    });
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    httplib::Client client("127.0.0.1", server_port_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (auto res = client.Get(std::string(API_BASE_PATH) + "/health"); res && res->status == 200) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  void stop_gateway() {
    node_.reset();
  }

  /// Run discovery passes until *predicate* accepts the /health document.
  json poll_health(const std::function<bool(const json &)> & predicate, const std::string & what) {
    httplib::Client client("127.0.0.1", server_port_);
    json last = json::object();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline) {
      if (auto * dm = node_->get_discovery_manager()) {
        dm->refresh_topic_map();
      }
      if (auto res = client.Get(std::string(API_BASE_PATH) + "/health"); res && res->status == 200) {
        last = json::parse(res->body);
        if (predicate(last)) {
          return last;
        }
      }
      std::this_thread::sleep_for(200ms);
    }
    ADD_FAILURE() << "timed out waiting for " << what << "; last /health: " << last.dump();
    return last;
  }

  /// Every node FQN currently on the graph, as the linker would see them.
  std::vector<std::string> graph_fqns() {
    std::vector<std::string> fqns;
    if (graph_nodes_.empty()) {
      return fqns;  // caller asserts on the result; this needs a node to query from
    }
    fqns = graph_nodes_.front()->get_node_names();
    std::sort(fqns.begin(), fqns.end());
    return fqns;
  }

  // Pins the HealthWarning invariant across every code the document carries:
  // entity_ids holds addressable SOVD ids only. A ROS node FQN there would
  // fail validate_entity_id on the '/' that "conflicts with URL routing".
  static void expect_entity_ids_are_addressable(const json & health) {
    ASSERT_TRUE(health.contains("warnings")) << health.dump();
    ASSERT_TRUE(health["warnings"].is_array());
    for (const auto & warning : health["warnings"]) {
      const std::string code = warning.value("code", std::string{});
      ASSERT_TRUE(warning.contains("entity_ids")) << code;
      ASSERT_TRUE(warning.contains("ros_node_fqns")) << code;
      ASSERT_TRUE(warning.contains("peer_names")) << code;
      for (const auto & id : warning["entity_ids"]) {
        const auto valid = ros2_medkit_gateway::validate_entity_id(id.get<std::string>());
        EXPECT_TRUE(valid.has_value()) << "warning '" << code
                                       << "' put a non-addressable value in entity_ids: " << id.dump() << " ("
                                       << (valid.has_value() ? std::string{} : valid.error()) << ")";
      }
    }
  }

  static const json * find_warning(const json & health, const std::string & code) {
    if (!health.contains("warnings") || !health["warnings"].is_array()) {
      return nullptr;
    }
    for (const auto & w : health["warnings"]) {
      if (w.value("code", std::string{}) == code) {
        return &w;
      }
    }
    return nullptr;
  }

  std::string write_manifest(const std::string & yaml) {
    auto path = std::filesystem::temp_directory_path() / ("medkit-health-linking-" + std::to_string(::getpid()) + "-" +
                                                          std::to_string(manifests_.size()) + ".yaml");
    std::ofstream(path) << yaml;
    manifests_.push_back(path.string());
    return path.string();
  }

  /// Overwrite the manifest the running gateway was started from.
  void rewrite_current_manifest(const std::string & yaml) {
    ASSERT_FALSE(manifests_.empty()) << "no manifest to rewrite";
    std::ofstream(manifests_.front(), std::ios::trunc) << yaml;
  }

  /// Drive the real reload path: this is the entry point a plugin reaches
  /// through PluginContext::notify_entities_changed, which is the only way a
  /// manifest is re-read while the gateway runs.
  void trigger_reload() {
    node_->handle_entity_change_notification(ros2_medkit_gateway::EntityChangeScope::full_refresh());
  }

  /// App ids currently served by GET /apps.
  std::set<std::string> served_app_ids() {
    httplib::Client client("127.0.0.1", server_port_);
    auto res = client.Get(std::string(API_BASE_PATH) + "/apps");
    if (!res || res->status != 200) {
      return {};
    }
    std::set<std::string> ids;
    for (const auto & item : json::parse(res->body).value("items", json::array())) {
      ids.insert(item.value("id", std::string{}));
    }
    return ids;
  }

  std::vector<std::shared_ptr<rclcpp::Node>> graph_nodes_;
  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  std::vector<std::string> manifests_;
  int server_port_{0};
};

class HealthLinkingPolicyTest : public HealthLinkingTest, public ::testing::WithParamInterface<std::string> {};

// The configured policy reaches the served document for every legal value, and
// only "error" turns orphans into a warning.
TEST_P(HealthLinkingPolicyTest, PolicyIsReportedAndOnlyErrorWarns) {
  const std::string policy = GetParam();
  spawn_nodes({kBoundNodeName, kOrphanAlpha, kOrphanBeta});
  start_gateway(linking_manifest(policy));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  auto health = poll_health(
      [](const json & h) {
        return h.contains("discovery") && h["discovery"].contains("linking") &&
               h["discovery"]["linking"].value("orphan_count", 0) >= 2;
      },
      "at least two orphan nodes under policy " + policy);
  ASSERT_TRUE(health.contains("discovery")) << health.dump();
  ASSERT_TRUE(health["discovery"].contains("linking")) << health.dump();

  const auto & linking = health["discovery"]["linking"];
  EXPECT_EQ(linking.value("unmanifested_policy", std::string{}), policy);

  // R6: an "error" policy is a report, never an outage.
  EXPECT_EQ(health.value("status", std::string{}), "healthy");
  EXPECT_EQ(health.value("warning_schema_version", 0), 2);

  const auto * warning = find_warning(health, "unmanifested_nodes");
  if (policy != "error") {
    EXPECT_EQ(warning, nullptr) << "policy '" << policy
                                << "' must not raise the unmanifested-node warning: " << health["warnings"].dump();
    return;
  }

  ASSERT_NE(warning, nullptr) << "policy 'error' with orphans must raise the warning: " << health.dump();

  // The subjects are ROS nodes, so they are carried as node FQNs.
  ASSERT_TRUE(warning->contains("ros_node_fqns"));
  const auto fqns = (*warning)["ros_node_fqns"].get<std::set<std::string>>();

  // Scale: every orphan is listed, not just the first one.
  EXPECT_EQ(fqns.size(), static_cast<size_t>(linking.value("orphan_count", 0)));
  EXPECT_TRUE(fqns.count(std::string("/") + kOrphanAlpha)) << "listed: " << (*warning)["ros_node_fqns"].dump();
  EXPECT_TRUE(fqns.count(std::string("/") + kOrphanBeta)) << "listed: " << (*warning)["ros_node_fqns"].dump();
  EXPECT_FALSE(fqns.count(std::string("/") + kBoundNodeName)) << "a linked node is not an orphan";

  // ...and entity_ids stays empty: a node FQN is not an addressable entity id.
  ASSERT_TRUE(warning->contains("entity_ids"));
  EXPECT_TRUE((*warning)["entity_ids"].is_array());
  EXPECT_TRUE((*warning)["entity_ids"].empty())
      << "entity_ids must carry addressable SOVD ids only: " << (*warning)["entity_ids"].dump();

  // A non-aggregation warning carries no peers, but keeps the field.
  ASSERT_TRUE(warning->contains("peer_names"));
  EXPECT_TRUE((*warning)["peer_names"].is_array());
  EXPECT_TRUE((*warning)["peer_names"].empty());

  EXPECT_FALSE(warning->value("message", std::string{}).empty());

  // The invariant, checked against every warning this gateway emits rather
  // than only the one under test: anything in entity_ids must be a value a
  // client can put in a URL. validate_entity_id is the same function the
  // routing layer applies, so this cannot drift from it.
  expect_entity_ids_are_addressable(health);
}

// A reload must move the pipeline, not just the served field. /health reads
// the manifest config live from the ManifestManager, while the pipeline
// captured it when it was built - so a reload that refreshed only one of them
// would make the documented monitor recipe
// (orphan_count > 0 && unmanifested_policy == "error") report a policy the
// orphan filter is not running.
//
// Asserting the field alone would pass against exactly that bug, so the entity
// tree is asserted too: `warn` keeps undeclared nodes visible, `ignore` hides
// them, and the difference is only produced by the pipeline's own copy.
// entity_ids promises every element is addressable. The leaf-collision code is
// the one path that can break that promise: the Component id comes verbatim
// from a remote peer and nothing on the ingest path validates it. A peer
// serving an id with a '/' in it would hand clients a value that builds a
// malformed GET /components/{id}.
class HealthPeerWarningTest : public ::testing::Test {
 protected:
  void SetUp() override {
    server_port_ = reserve_free_port();
    ASSERT_NE(server_port_, 0) << "Failed to reserve a free port for test";

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("server.port", server_port_),
        rclcpp::Parameter("aggregation.enabled", true),
        rclcpp::Parameter("aggregation.peer_urls", std::vector<std::string>{"http://127.0.0.1:1"}),
        rclcpp::Parameter("aggregation.peer_names", std::vector<std::string>{"rogue_peer"}),
    });
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

    httplib::Client client("127.0.0.1", server_port_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      if (auto res = client.Get(std::string(API_BASE_PATH) + "/health"); res && res->status == 200) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  void TearDown() override {
    node_.reset();
  }

  /// Seed a leaf-collision warning as if a peer had announced *entity_id*.
  json health_with_peer_collision(const std::string & entity_id) {
    auto * agg = node_->get_aggregation_manager();
    EXPECT_NE(agg, nullptr) << "aggregation.enabled=true must give us a manager";
    agg->set_leaf_warnings({{entity_id, {"rogue_peer"}}});

    httplib::Client client("127.0.0.1", server_port_);
    auto res = client.Get(std::string(API_BASE_PATH) + "/health");
    EXPECT_TRUE(res && res->status == 200);
    return json::parse(res->body);
  }

  // Returns a pointer INTO *health*, so it must index the document itself -
  // value() would hand back a temporary array and the pointer would dangle.
  static const json * warning_with_code(const json & health, const std::string & code) {
    if (!health.contains("warnings") || !health["warnings"].is_array()) {
      return nullptr;
    }
    for (const auto & w : health["warnings"]) {
      if (w.value("code", std::string{}) == code) {
        return &w;
      }
    }
    return nullptr;
  }

  /// Same invariant check the linking suite applies, over the whole document.
  static void expect_entity_ids_are_addressable(const json & health) {
    ASSERT_TRUE(health.contains("warnings")) << health.dump();
    for (const auto & warning : health["warnings"]) {
      for (const auto & id : warning["entity_ids"]) {
        const auto valid = ros2_medkit_gateway::validate_entity_id(id.get<std::string>());
        EXPECT_TRUE(valid.has_value()) << "non-addressable id in entity_ids: " << id.dump();
      }
    }
  }

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  int server_port_{0};
};

TEST_F(HealthPeerWarningTest, ValidPeerComponentIdIsPublishedInEntityIds) {
  const auto health = health_with_peer_collision("ecu-x");

  const auto * warning = warning_with_code(health, "leaf_id_collision");
  ASSERT_NE(warning, nullptr) << health.dump();
  EXPECT_EQ((*warning)["entity_ids"], json::array({"ecu-x"}));
  expect_entity_ids_are_addressable(health);
}

TEST_F(HealthPeerWarningTest, UnaddressablePeerComponentIdIsWithheldFromEntityIds) {
  // A '/' is rejected in an entity id because it conflicts with URL routing,
  // so this is exactly the value a client could not address.
  const auto health = health_with_peer_collision("fleet/ecu-x");

  const auto * warning = warning_with_code(health, "leaf_id_collision");
  ASSERT_NE(warning, nullptr) << "the warning must still be raised: " << health.dump();

  EXPECT_TRUE((*warning)["entity_ids"].empty())
      << "an id a client cannot put in a URL must not be published as one: " << (*warning)["entity_ids"].dump();

  // Nothing an operator needs is lost - the prose still names the component.
  EXPECT_NE(warning->value("message", std::string{}).find("fleet/ecu-x"), std::string::npos)
      << "the message must still identify the component: " << warning->value("message", std::string{});
  EXPECT_EQ((*warning)["peer_names"], json::array({"rogue_peer"}));

  // And the document-wide invariant still holds.
  expect_entity_ids_are_addressable(health);
}

TEST_F(HealthLinkingTest, ReloadMovesBothTheReportedPolicyAndTheServedTree) {
  spawn_nodes({kBoundNodeName, kOrphanAlpha, kOrphanBeta});
  start_gateway(linking_manifest("warn"));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  // Before: warn keeps the undeclared nodes in the tree.
  poll_health(
      [](const json & h) {
        return h["discovery"]["linking"].value("orphan_count", 0) >= 2;
      },
      "the orphans to be discovered under warn");
  EXPECT_EQ(node_->get_discovery_manager()->get_manifest_config().unmanifested_nodes,
            ros2_medkit_gateway::discovery::ManifestConfig::UnmanifestedNodePolicy::WARN);

  const auto before = served_app_ids();
  EXPECT_TRUE(before.count(kOrphanAlpha)) << "warn must leave the undeclared node visible; got: " << before.size();
  EXPECT_TRUE(before.count(kOrphanBeta));

  // Change the policy on disk and drive the real reload path.
  rewrite_current_manifest(linking_manifest("ignore"));
  trigger_reload();

  // After: the served field says ignore ...
  auto health = poll_health(
      [](const json & h) {
        return h["discovery"]["linking"].value("unmanifested_policy", std::string{}) == "ignore";
      },
      "/health to report the reloaded policy");
  EXPECT_EQ(health["discovery"]["linking"].value("unmanifested_policy", std::string{}), "ignore");

  // ... and the tree the pipeline produces agrees with it.
  std::set<std::string> after;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline) {
    node_->get_discovery_manager()->refresh_topic_map();
    after = served_app_ids();
    if (after.count(kOrphanAlpha) == 0 && after.count(kOrphanBeta) == 0) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(after.count(kOrphanAlpha), 0u)
      << "the orphan filter is still running the pre-reload policy; served: " << after.size() << " apps";
  EXPECT_EQ(after.count(kOrphanBeta), 0u);
  EXPECT_TRUE(after.count("bound_app")) << "the declared app must survive the reload";
}

// The reload path carries more than the unmanifested policy. These two cover
// the parts that only a reload exercises: the set_manifest_config push (which
// is what makes inherit_runtime_resources take effect on the new config) and
// reset_policies_to_defaults (which exists solely so a manifest turning
// allow_manifest_override back ON undoes the previous demotion).

TEST_F(HealthLinkingTest, ReloadTurningOverrideBackOnRestoresManifestExclusivity) {
  spawn_nodes({kBoundNodeName, kOrphanAlpha});
  start_gateway(linking_manifest("warn", {}, "  allow_manifest_override: false\n"));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  poll_health(
      [](const json & h) {
        return h["discovery"]["linking"].value("orphan_count", 0) >= 1;
      },
      "discovery to settle with override disabled");
  EXPECT_FALSE(node_->get_discovery_manager()->get_manifest_config().allow_manifest_override);

  // Flip it back on and reload. Without reset_policies_to_defaults the layer
  // would still carry the FALLBACK demotion from the first load.
  rewrite_current_manifest(linking_manifest("warn", {}, "  allow_manifest_override: true\n"));
  trigger_reload();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  bool restored = false;
  while (std::chrono::steady_clock::now() < deadline) {
    node_->get_discovery_manager()->refresh_topic_map();
    if (node_->get_discovery_manager()->get_manifest_config().allow_manifest_override) {
      restored = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_TRUE(restored) << "the reloaded allow_manifest_override must reach the pipeline's config";

  // And the declared app is still served - the reload did not lose the tree.
  EXPECT_TRUE(served_app_ids().count("bound_app"));
}

TEST_F(HealthLinkingTest, ReloadTurningInheritOffStopsCopyingRuntimeResources) {
  spawn_nodes({kBoundNodeName});
  start_gateway(linking_manifest("warn", {}, "  inherit_runtime_resources: true\n"));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  poll_health(
      [](const json & h) {
        return h["discovery"].contains("linking");
      },
      "discovery to settle with inheritance on");
  EXPECT_TRUE(node_->get_discovery_manager()->get_manifest_config().inherit_runtime_resources);

  rewrite_current_manifest(linking_manifest("warn", {}, "  inherit_runtime_resources: false\n"));
  trigger_reload();

  // The push that carries this into the pipeline is the `else` branch that
  // exists only for reload; without it the pipeline keeps the boot-time value.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  bool applied = false;
  while (std::chrono::steady_clock::now() < deadline) {
    node_->get_discovery_manager()->refresh_topic_map();
    if (!node_->get_discovery_manager()->get_manifest_config().inherit_runtime_resources) {
      applied = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_TRUE(applied) << "the reloaded inherit_runtime_resources must reach the pipeline's config";

  // The app is still linked and served; only the resource copy is gated.
  EXPECT_TRUE(served_app_ids().count("bound_app"));
}

INSTANTIATE_TEST_SUITE_P(AllPolicies, HealthLinkingPolicyTest,
                         ::testing::Values("ignore", "warn", "error", "include_as_orphan"),
                         [](const ::testing::TestParamInfo<std::string> & param_info) {
                           return param_info.param;
                         });

// The warning is about orphans, not about the policy: with everything on the
// graph declared, "error" says nothing.
TEST_F(HealthLinkingTest, ErrorPolicyWithoutOrphansRaisesNoWarning) {
  spawn_nodes({kBoundNodeName});
  start_gateway(linking_manifest("error"));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  // A gateway process puts several nodes of its own on the graph (parameter
  // client, fault clients, subscription worker, ...). Read them off the live
  // graph instead of hard-coding a list that would rot across distros.
  poll_health(
      [](const json & h) {
        return h.contains("discovery") && h["discovery"].contains("linking");
      },
      "the linking block to appear");
  auto fqns = graph_fqns();
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  std::vector<std::string> to_declare;
  for (const auto & fqn : fqns) {
    if (fqn != std::string("/") + kBoundNodeName) {
      to_declare.push_back(fqn);
    }
  }
  ASSERT_FALSE(to_declare.empty()) << "expected the gateway process to own at least one node";

  stop_gateway();
  start_gateway(linking_manifest("error", to_declare));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());

  auto health = poll_health(
      [](const json & h) {
        return h.contains("discovery") && h["discovery"].contains("linking") &&
               h["discovery"]["linking"].value("orphan_count", -1) == 0;
      },
      "every node on the graph to be declared");

  ASSERT_TRUE(health.contains("discovery")) << health.dump();
  ASSERT_TRUE(health["discovery"].contains("linking")) << health.dump();
  EXPECT_EQ(health["discovery"]["linking"].value("orphan_count", -1), 0);
  EXPECT_EQ(health["discovery"]["linking"].value("unmanifested_policy", std::string{}), "error");
  EXPECT_EQ(find_warning(health, "unmanifested_nodes"), nullptr)
      << "no orphans means nothing to warn about: " << health["warnings"].dump();
  EXPECT_EQ(health.value("status", std::string{}), "healthy");
}
