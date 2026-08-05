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

#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"
#include "typed_test_fixture.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::DiscoveryConfig;
using ros2_medkit_gateway::DiscoveryMode;
using ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND;
using ros2_medkit_gateway::ERR_INVALID_PARAMETER;
using ros2_medkit_gateway::ERR_INVALID_REQUEST;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::JsonWriter;
using ros2_medkit_gateway::handlers::DiscoveryHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::http::TypedRequest;

namespace dto = ros2_medkit_gateway::dto;

namespace {

const char * kManifestYaml = R"(
manifest_version: "1.0"
metadata:
  name: "discovery-handlers-test"
  version: "1.0.0"
areas:
  - id: "vehicle"
    name: "Vehicle"
  - id: "sensors"
    name: "Sensors"
    parent_area: "vehicle"
components:
  - id: "main_ecu"
    name: "Main ECU"
    namespace: "/vehicle"
    area: "vehicle"
    description: "Vehicle control unit"
    tags: ["compute", "control"]
    depends_on: ["lidar_unit", "ghost_component"]
    identity:
      manufacturer: "Acme Robotics"
      model: "ECU-9000"
      serial_number: "SN-MAIN-001"
      role: "controller"
      extra:
        slot: "1"
  - id: "lidar_unit"
    name: "Lidar Unit"
    namespace: "/sensors"
    area: "sensors"
    parent_component_id: "main_ecu"
    description: "Lidar aggregation"
    tags: ["sensor"]
  - id: ext_plc
    name: "External PLC"
    external: true
    identity:
      manufacturer: "Siemens"
      serial_number: "SN-1"
  - id: ext_sensor
    name: "External Sensor"
    parent_component_id: "main_ecu"
    external: true
    identity:
      manufacturer: "Bosch"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "main_ecu"
    description: "Path planning"
  - id: "mapper"
    name: "Mapper"
    is_located_on: "lidar_unit"
    depends_on: ["planner", "ghost_app"]
  - id: "standalone"
    name: "Standalone"
    description: "Standalone app without a hosting component"
  - id: ext_app
    name: "External App"
    external: true
    is_located_on: ext_plc
functions:
  - id: "navigation"
    name: "Navigation"
    hosts: ["planner"]
  - id: "perception"
    name: "Perception"
    hosts: ["mapper"]
)";

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "Failed to create socket for test port reservation: " << std::strerror(errno);
    return 0;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;

  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ADD_FAILURE() << "Failed to bind socket for test port reservation: " << std::strerror(errno);
    close(sock);
    return 0;
  }

  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    ADD_FAILURE() << "Failed to inspect reserved test port: " << std::strerror(errno);
    close(sock);
    return 0;
  }

  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

// PR-403 commit 17: typed handlers take TypedRequest, which wraps an
// httplib::Request. The request's `matches` array is populated by
// std::regex_match before the wrapper is built so the typed handler's
// `path_param("1")` can return the first capture group exactly the way the
// production routing layer does. The shared helper lives in
// typed_test_fixture.hpp so other handler tests don't have to redefine it.
using ros2_medkit_gateway::test::make_typed_request;

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_discovery_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    ADD_FAILURE() << "Failed to create temp manifest file: " << std::strerror(errno);
    return {};
  }
  close(fd);

  std::ofstream out(path_template);
  if (!out) {
    ADD_FAILURE() << "Failed to open temp manifest file for writing: " << path_template;
    std::remove(path_template);
    return {};
  }
  out << contents;
  if (!out.good()) {
    ADD_FAILURE() << "Failed to write manifest contents to: " << path_template;
    out.close();
    std::remove(path_template);
    return {};
  }
  out.close();
  return path_template;
}

// Convert a successful typed handler result to the JSON body the wire would
// see. Tests assert on the same json shape they did pre-migration via this
// helper instead of inspecting httplib::Response::body directly.
template <class T>
json body_json(const ros2_medkit_gateway::http::Result<T> & result) {
  EXPECT_TRUE(result.has_value());
  return JsonWriter<T>::write(result.value());
}

}  // namespace

class DiscoveryHandlersValidationTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  DiscoveryHandlers handlers_{ctx_};
};

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersValidationTest, GetAreaMissingMatchesReturns400) {
  // No path supplied -> regex_match runs over an empty path with no captures.
  // path_param("1") yields ERR_INVALID_REQUEST/400 (the legacy `req.matches.size() < 2`
  // branch). The new typed handler signals that via tl::unexpected with
  // status 400.
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_.get_area(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

class DiscoveryHandlersFixtureTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {
        "test_discovery_handlers",   "--ros-args", "-p", "server.port:=" + std::to_string(suite_server_port_), "-p",
        "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }

    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    suite_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifestYaml);
    ASSERT_FALSE(manifest_path_.empty());
    ASSERT_NE(suite_node_, nullptr);

    DiscoveryConfig config;
    config.mode = DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(suite_node_->get_discovery_manager()->initialize(config));

    auto areas = suite_node_->get_discovery_manager()->discover_areas();
    auto components = suite_node_->get_discovery_manager()->discover_components();
    auto apps = suite_node_->get_discovery_manager()->discover_apps();
    auto functions = suite_node_->get_discovery_manager()->discover_functions();

    ASSERT_EQ(apps.size(), 4u);
    apps[0].is_online = true;
    apps[0].bound_fqn = "/vehicle/main_ecu/planner";
    apps[1].bound_fqn = "/sensors/lidar_unit/mapper";

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    ctx_ = std::make_unique<HandlerContext>(suite_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<DiscoveryHandlers>(*ctx_);
  }

  void TearDown() override {
    handlers_.reset();
    ctx_.reset();

    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<DiscoveryHandlers> handlers_;
  std::string manifest_path_;
};

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, ListAreasReturnsSeededItems) {
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_->get_areas(typed_req);
  ASSERT_TRUE(result.has_value());
  // "sensors" has parent_area "vehicle", so it's filtered from top-level list
  ASSERT_EQ(result->items.size(), 1u);
  EXPECT_EQ(result->items[0].id, "vehicle");
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersValidationTest, GetAreaInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/bad@id", R"(/api/v1/areas/([^/]+))");

  auto result = handlers_.get_area(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetAreaUnknownIdReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/unknown", R"(/api/v1/areas/([^/]+))");

  auto result = handlers_->get_area(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetAreaReturnsCapabilitiesAndLinks) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/vehicle", R"(/api/v1/areas/([^/]+))");

  auto result = handlers_->get_area(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["components"], "/api/v1/areas/vehicle/components");
  EXPECT_EQ(body["contains"], "/api/v1/areas/vehicle/contains");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/areas/vehicle");
  EXPECT_EQ(body["_links"]["collection"], "/api/v1/areas");
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersFixtureTest, AreaComponentsReturnsMatchingComponentsOnly) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/sensors/components", R"(/api/v1/areas/([^/]+)/components)");

  auto result = handlers_->get_area_components(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersValidationTest, AreaComponentsInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/bad@id/components", R"(/api/v1/areas/(.+)/components)");

  auto result = handlers_.get_area_components(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersFixtureTest, AreaComponentsUnknownAreaReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/unknown/components", R"(/api/v1/areas/([^/]+)/components)");

  auto result = handlers_->get_area_components(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_004
TEST_F(DiscoveryHandlersFixtureTest, GetSubareasReturnsChildAreas) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/vehicle/subareas", R"(/api/v1/areas/([^/]+)/subareas)");

  auto result = handlers_->get_subareas(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "sensors");
  EXPECT_EQ(body["_links"]["parent"], "/api/v1/areas/vehicle");
}

// @verifies REQ_INTEROP_004
TEST_F(DiscoveryHandlersValidationTest, GetSubareasInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/bad@id/subareas", R"(/api/v1/areas/(.+)/subareas)");

  auto result = handlers_.get_subareas(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_004
TEST_F(DiscoveryHandlersFixtureTest, GetSubareasUnknownAreaReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/unknown/subareas", R"(/api/v1/areas/([^/]+)/subareas)");

  auto result = handlers_->get_subareas(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersFixtureTest, GetContainsReturnsAreaComponents) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/vehicle/contains", R"(/api/v1/areas/([^/]+)/contains)");

  auto result = handlers_->get_area_contains(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 2u);
  EXPECT_EQ(body["items"][0]["id"], "main_ecu");
  EXPECT_EQ(body["items"][1]["id"], "lidar_unit");
  EXPECT_EQ(body["_links"]["area"], "/api/v1/areas/vehicle");
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersValidationTest, GetContainsInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/bad@id/contains", R"(/api/v1/areas/(.+)/contains)");

  auto result = handlers_.get_area_contains(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_006
TEST_F(DiscoveryHandlersFixtureTest, GetContainsUnknownAreaReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/areas/unknown/contains", R"(/api/v1/areas/([^/]+)/contains)");

  auto result = handlers_->get_area_contains(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, ListComponentsReturnsMetadata) {
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_->get_components(typed_req);
  auto body = body_json(result);
  // lidar_unit has parent_component_id, so it's filtered from top-level list.
  ASSERT_EQ(body["items"].size(), 2u);
  // get_components() does not sort; find main_ecu by id rather than assuming order.
  const auto & items = body["items"];
  size_t main_ecu_idx = items.size();
  for (size_t i = 0; i < items.size(); ++i) {
    if (items[i]["id"] == "main_ecu") {
      main_ecu_idx = i;
      break;
    }
  }
  ASSERT_LT(main_ecu_idx, items.size()) << "main_ecu not found in /components list";
  const auto & main_ecu = items[main_ecu_idx];
  EXPECT_EQ(main_ecu["description"], "Vehicle control unit");
  EXPECT_EQ(main_ecu["x-medkit"]["source"], "manifest");
}

// @verifies REQ_INTEROP_003
// INV2: manifest-driven asset identity surfaces over SOVD with per-field
// provenance. The list handler emits x-medkit.identity for a component whose
// manifest declares an identity block.
TEST_F(DiscoveryHandlersFixtureTest, ListComponentsIncludesManifestIdentity) {
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_->get_components(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 2u);
  // get_components() does not sort; find main_ecu by id rather than assuming order.
  const auto & items = body["items"];
  size_t main_ecu_idx = items.size();
  for (size_t i = 0; i < items.size(); ++i) {
    if (items[i]["id"] == "main_ecu") {
      main_ecu_idx = i;
      break;
    }
  }
  ASSERT_LT(main_ecu_idx, items.size()) << "main_ecu not found in /components list";
  const auto & identity = items[main_ecu_idx]["x-medkit"]["identity"];
  // Typed fields serialize camelCase (AssetIdentity::to_json).
  EXPECT_EQ(identity["manufacturer"], "Acme Robotics");
  EXPECT_EQ(identity["model"], "ECU-9000");
  EXPECT_EQ(identity["serialNumber"], "SN-MAIN-001");
  EXPECT_EQ(identity["role"], "controller");
  EXPECT_EQ(identity["extra"]["slot"], "1");
  // Provenance keys use the internal snake_case field names.
  EXPECT_EQ(identity["_provenance"]["manufacturer"], "manifest");
  EXPECT_EQ(identity["_provenance"]["serial_number"], "manifest");
  EXPECT_EQ(identity["_provenance"]["extra.slot"], "manifest");
}

// @verifies REQ_INTEROP_003
// INV2: the same identity is present on the component detail endpoint.
TEST_F(DiscoveryHandlersFixtureTest, GetComponentIncludesManifestIdentity) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu", R"(/api/v1/components/([^/]+))");

  auto result = handlers_->get_component(typed_req);
  auto body = body_json(result);
  const auto & identity = body["x-medkit"]["identity"];
  EXPECT_EQ(identity["manufacturer"], "Acme Robotics");
  EXPECT_EQ(identity["role"], "controller");
  EXPECT_EQ(identity["_provenance"]["model"], "manifest");
}

// @verifies REQ_INTEROP_003
// A component without a manifest identity block emits no x-medkit.identity.
TEST_F(DiscoveryHandlersFixtureTest, GetComponentWithoutIdentityOmitsField) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/lidar_unit", R"(/api/v1/components/([^/]+))");

  auto result = handlers_->get_component(typed_req);
  auto body = body_json(result);
  EXPECT_FALSE(body["x-medkit"].contains("identity"));
}

// @verifies REQ_INTEROP_003
// A manifest component explicitly marked `external: true` surfaces
// x-medkit.external on the wire (#516).
TEST_F(DiscoveryHandlersFixtureTest, GetComponentEmitsExternalOnWire) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/ext_plc", R"(/api/v1/components/([^/]+))");

  auto result = handlers_->get_component(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["x-medkit"]["external"], true);
}

// @verifies REQ_INTEROP_003
// A ROS-native manifest component omits x-medkit.external entirely (absence
// means "not external", never emitted as an explicit `false`) (#516).
TEST_F(DiscoveryHandlersFixtureTest, GetComponentOmitsExternalForRosComponent) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu", R"(/api/v1/components/([^/]+))");

  auto result = handlers_->get_component(typed_req);
  auto body = body_json(result);
  EXPECT_FALSE(body["x-medkit"].contains("external"));
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersValidationTest, GetComponentInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/bad/id", R"(/api/v1/components/(.+))");

  auto result = handlers_.get_component(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetComponentReturnsRelationshipsAndCapabilities) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu", R"(/api/v1/components/([^/]+))");

  auto result = handlers_->get_component(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["belongs-to"], "/api/v1/areas/vehicle");
  EXPECT_EQ(body["subcomponents"], "/api/v1/components/main_ecu/subcomponents");
  EXPECT_EQ(body["hosts"], "/api/v1/components/main_ecu/hosts");
  EXPECT_EQ(body["depends-on"], "/api/v1/components/main_ecu/depends-on");
  EXPECT_EQ(body["_links"]["area"], "/api/v1/areas/vehicle");
}

// @verifies REQ_INTEROP_005
TEST_F(DiscoveryHandlersFixtureTest, GetSubcomponentsReturnsChildren) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu/subcomponents",
                                      R"(/api/v1/components/([^/]+)/subcomponents)");

  auto result = handlers_->get_subcomponents(typed_req);
  auto body = body_json(result);
  // main_ecu has two subcomponents: lidar_unit (ROS) and ext_sensor (external).
  ASSERT_EQ(body["items"].size(), 2u);
  bool has_lidar = false;
  bool has_ext_sensor = false;
  for (const auto & item : body["items"]) {
    if (item["id"] == "lidar_unit") {
      has_lidar = true;
    }
    if (item["id"] == "ext_sensor") {
      has_ext_sensor = true;
    }
  }
  EXPECT_TRUE(has_lidar);
  EXPECT_TRUE(has_ext_sensor);
}

// A device Component browsed via a relationship/sub-collection route must still
// read as external (the "absence == not external" invariant), so the external
// subcomponent carries x-medkit.external while the plain ROS one does not.
TEST_F(DiscoveryHandlersFixtureTest, SubcomponentsEmitExternalForExternalChild) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu/subcomponents",
                                      R"(/api/v1/components/([^/]+)/subcomponents)");

  auto result = handlers_->get_subcomponents(typed_req);
  auto body = body_json(result);
  const auto & items = body["items"];
  size_t ext_idx = items.size();
  size_t lidar_idx = items.size();
  for (size_t i = 0; i < items.size(); ++i) {
    if (items[i]["id"] == "ext_sensor") {
      ext_idx = i;
    }
    if (items[i]["id"] == "lidar_unit") {
      lidar_idx = i;
    }
  }
  ASSERT_LT(ext_idx, items.size()) << "ext_sensor not found in subcomponents";
  ASSERT_LT(lidar_idx, items.size()) << "lidar_unit not found in subcomponents";
  EXPECT_EQ(items[ext_idx]["x-medkit"]["external"], true);
  EXPECT_FALSE(items[lidar_idx]["x-medkit"].contains("external"));
}

// The hosts (App) projection must also carry x-medkit.external: ext_plc hosts
// the external app ext_app, so a consumer browsing hosts sees the classification.
TEST_F(DiscoveryHandlersFixtureTest, ComponentHostsEmitExternalForExternalApp) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/ext_plc/hosts", R"(/api/v1/components/([^/]+)/hosts)");

  auto result = handlers_->get_component_hosts(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "ext_app");
  EXPECT_EQ(body["items"][0]["x-medkit"]["external"], true);
}

// @verifies REQ_INTEROP_005
TEST_F(DiscoveryHandlersValidationTest, GetSubcomponentsInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/components/bad/id/subcomponents", R"(/api/v1/components/(.+)/subcomponents)");

  auto result = handlers_.get_subcomponents(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_005
TEST_F(DiscoveryHandlersFixtureTest, GetSubcomponentsUnknownComponentReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/unknown/subcomponents",
                                      R"(/api/v1/components/([^/]+)/subcomponents)");

  auto result = handlers_->get_subcomponents(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersFixtureTest, GetHostsReturnsHostedApps) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/main_ecu/hosts", R"(/api/v1/components/([^/]+)/hosts)");

  auto result = handlers_->get_component_hosts(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  // Reads from cache where planner app has is_online=true (set in SetUp)
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], true);
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersValidationTest, GetHostsInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/bad/id/hosts", R"(/api/v1/components/(.+)/hosts)");

  auto result = handlers_.get_component_hosts(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersFixtureTest, GetHostsUnknownComponentReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/components/unknown/hosts", R"(/api/v1/components/([^/]+)/hosts)");

  auto result = handlers_->get_component_hosts(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_008
TEST_F(DiscoveryHandlersFixtureTest, ComponentDependsOnReturnsResolvedAndMissingDependencies) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/components/main_ecu/depends-on", R"(/api/v1/components/([^/]+)/depends-on)");

  auto result = handlers_->get_component_depends_on(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 2u);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
  EXPECT_EQ(body["items"][1]["id"], "ghost_component");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
}

// @verifies REQ_INTEROP_008
TEST_F(DiscoveryHandlersValidationTest, ComponentDependsOnInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/components/bad/id/depends-on", R"(/api/v1/components/(.+)/depends-on)");

  auto result = handlers_.get_component_depends_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_008
TEST_F(DiscoveryHandlersFixtureTest, ComponentDependsOnUnknownComponentReturns404) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/components/unknown/depends-on", R"(/api/v1/components/([^/]+)/depends-on)");

  auto result = handlers_->get_component_depends_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersValidationTest, GetAppInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/bad/id", R"(/api/v1/apps/(.+))");

  auto result = handlers_.get_app(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, ListAppsReturnsSeededMetadata) {
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_->get_apps(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 4u);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["component_id"], "main_ecu");
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], true);
  EXPECT_EQ(body["items"][0]["x-medkit"]["ros2"]["node"], "/vehicle/main_ecu/planner");
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetAppUnknownIdReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/unknown", R"(/api/v1/apps/([^/]+))");

  auto result = handlers_->get_app(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
// A manifest app explicitly marked `external: true` surfaces
// x-medkit.external on the wire (#516).
TEST_F(DiscoveryHandlersFixtureTest, GetAppEmitsExternalOnWire) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/ext_app", R"(/api/v1/apps/([^/]+))");

  auto result = handlers_->get_app(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["x-medkit"]["external"], true);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetAppReturnsLinksAndCapabilities) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper", R"(/api/v1/apps/([^/]+))");

  auto result = handlers_->get_app(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["is-located-on"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["belongs-to"], "/api/v1/apps/mapper/belongs-to");
  EXPECT_EQ(body["depends-on"], "/api/v1/apps/mapper/depends-on");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/mapper");
  EXPECT_EQ(body["_links"]["is-located-on"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["_links"]["belongs-to"], "/api/v1/apps/mapper/belongs-to");
  EXPECT_EQ(body["_links"]["depends-on"][0], "/api/v1/apps/planner");
  EXPECT_EQ(body["_links"]["depends-on"][1], "/api/v1/apps/ghost_app");
  EXPECT_EQ(body["x-medkit"]["source"], "manifest");
  EXPECT_EQ(body["x-medkit"]["is_online"], false);
}

// @verifies REQ_INTEROP_105
TEST_F(DiscoveryHandlersFixtureTest, AppIsLocatedOnReturnsParentComponent) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/apps/mapper/is-located-on", R"(/api/v1/apps/([^/]+)/is-located-on)");

  auto result = handlers_->get_app_is_located_on(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
  EXPECT_EQ(body["items"][0]["name"], "Lidar Unit");
  EXPECT_EQ(body["items"][0]["href"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["x-medkit"]["total_count"], 1u);
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/mapper/is-located-on");
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/mapper");
}

// @verifies REQ_INTEROP_105
TEST_F(DiscoveryHandlersFixtureTest, AppIsLocatedOnReturnsEmptyWhenAppHasNoComponent) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/apps/standalone/is-located-on", R"(/api/v1/apps/([^/]+)/is-located-on)");

  auto result = handlers_->get_app_is_located_on(typed_req);
  auto body = body_json(result);
  ASSERT_TRUE(body["items"].empty());
  EXPECT_EQ(body["x-medkit"]["total_count"], 0u);
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/standalone/is-located-on");
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/standalone");
}

// @verifies REQ_INTEROP_105
TEST_F(DiscoveryHandlersFixtureTest, AppIsLocatedOnReturnsMissingItemWhenHostComponentUnresolved) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  auto apps = cache.get_apps();
  ASSERT_FALSE(apps.empty());

  bool updated = false;
  for (auto & app : apps) {
    if (app.id == "mapper") {
      app.component_id = "ghost_component";
      updated = true;
      break;
    }
  }
  ASSERT_TRUE(updated);
  cache.update_apps(apps);

  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/apps/mapper/is-located-on", R"(/api/v1/apps/([^/]+)/is-located-on)");

  auto result = handlers_->get_app_is_located_on(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "ghost_component");
  EXPECT_EQ(body["items"][0]["name"], "ghost_component");
  EXPECT_EQ(body["items"][0]["href"], "/api/v1/components/ghost_component");
  EXPECT_EQ(body["items"][0]["x-medkit"]["missing"], true);
}

// @verifies REQ_INTEROP_105
TEST_F(DiscoveryHandlersValidationTest, AppIsLocatedOnInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/bad/id/is-located-on", R"(/api/v1/apps/(.+)/is-located-on)");

  auto result = handlers_.get_app_is_located_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_105
TEST_F(DiscoveryHandlersFixtureTest, AppIsLocatedOnUnknownAppReturns404) {
  httplib::Request req;
  auto typed_req =
      make_typed_request(req, "/api/v1/apps/unknown/is-located-on", R"(/api/v1/apps/([^/]+)/is-located-on)");

  auto result = handlers_->get_app_is_located_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToReturnsParentArea) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "sensors");
  EXPECT_EQ(body["items"][0]["name"], "Sensors");
  EXPECT_EQ(body["items"][0]["href"], "/api/v1/areas/sensors");
  EXPECT_EQ(body["x-medkit"]["total_count"], 1u);
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/mapper/belongs-to");
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/mapper");
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToReturnsEmptyWhenAppHasNoComponent) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/standalone/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  auto body = body_json(result);
  ASSERT_TRUE(body["items"].empty());
  EXPECT_EQ(body["x-medkit"]["total_count"], 0u);
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/standalone/belongs-to");
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/standalone");
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToReturnsMissingItemWhenParentComponentUnresolved) {
  // Mirror of AppIsLocatedOnReturnsMissingItemWhenHostComponentUnresolved: surface a broken
  // parent reference instead of silently returning items=[]. Lets HATEOAS clients tell
  // 'app has no parent area' from 'manifest broken, component gone'.
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  auto apps = cache.get_apps();
  ASSERT_FALSE(apps.empty());

  bool updated = false;
  for (auto & app : apps) {
    if (app.id == "mapper") {
      app.component_id = "ghost_component";
      updated = true;
      break;
    }
  }
  ASSERT_TRUE(updated);
  cache.update_apps(apps);

  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["x-medkit"]["missing"], true);
  EXPECT_EQ(body["items"][0]["x-medkit"]["unresolved_component"], "ghost_component");
  EXPECT_EQ(body["items"][0]["href"], "");
  EXPECT_EQ(body["x-medkit"]["total_count"], 1u);
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToReturnsEmptyWhenComponentHasNoArea) {
  // Strip the parent component's area assignment so the chain App->Component->Area breaks.
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  auto components = cache.get_components();
  bool updated = false;
  for (auto & comp : components) {
    if (comp.id == "lidar_unit") {
      comp.area.clear();
      updated = true;
      break;
    }
  }
  ASSERT_TRUE(updated);
  cache.update_components(components);

  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  auto body = body_json(result);
  ASSERT_TRUE(body["items"].empty());
  EXPECT_EQ(body["x-medkit"]["total_count"], 0u);
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToReturnsMissingItemWhenAreaUnresolved) {
  // Reassign the parent component to an area id that does not exist in the catalogue.
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  auto components = cache.get_components();
  bool updated = false;
  for (auto & comp : components) {
    if (comp.id == "lidar_unit") {
      comp.area = "ghost_area";
      updated = true;
      break;
    }
  }
  ASSERT_TRUE(updated);
  cache.update_components(components);

  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "ghost_area");
  EXPECT_EQ(body["items"][0]["name"], "ghost_area");
  EXPECT_EQ(body["items"][0]["href"], "/api/v1/areas/ghost_area");
  EXPECT_EQ(body["items"][0]["x-medkit"]["missing"], true);
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersValidationTest, AppBelongsToInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/bad/id/belongs-to", R"(/api/v1/apps/(.+)/belongs-to)");

  auto result = handlers_.get_app_belongs_to(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_106
TEST_F(DiscoveryHandlersFixtureTest, AppBelongsToUnknownAppReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/unknown/belongs-to", R"(/api/v1/apps/([^/]+)/belongs-to)");

  auto result = handlers_->get_app_belongs_to(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_009
TEST_F(DiscoveryHandlersFixtureTest, AppDependsOnReturnsResolvedAndMissingDependencies) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/mapper/depends-on", R"(/api/v1/apps/([^/]+)/depends-on)");

  auto result = handlers_->get_app_depends_on(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 2u);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  // Reads from cache where planner app has is_online=true (set in SetUp)
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], true);
  EXPECT_EQ(body["items"][1]["id"], "ghost_app");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/mapper");
}

// @verifies REQ_INTEROP_009
TEST_F(DiscoveryHandlersValidationTest, AppDependsOnInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/bad/id/depends-on", R"(/api/v1/apps/(.+)/depends-on)");

  auto result = handlers_.get_app_depends_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_009
TEST_F(DiscoveryHandlersFixtureTest, AppDependsOnUnknownAppReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/apps/unknown/depends-on", R"(/api/v1/apps/([^/]+)/depends-on)");

  auto result = handlers_->get_app_depends_on(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersValidationTest, GetFunctionInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/bad/id", R"(/api/v1/functions/(.+))");

  auto result = handlers_.get_function(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, ListFunctionsReturnsSeededFunctions) {
  httplib::Request req;
  TypedRequest typed_req(req);

  auto result = handlers_->get_functions(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 2u);
  EXPECT_EQ(body["items"][0]["id"], "navigation");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
}

// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetFunctionUnknownIdReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/unknown", R"(/api/v1/functions/([^/]+))");

  auto result = handlers_->get_function(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_003
// The `x-medkit-graph` link is emitted off the entity's capability list, and
// nothing in the gateway puts that capability there - only a plugin does. This
// fixture loads none, so the key must be absent: it used to be written
// unconditionally, which published a URI that answered 404 on every gateway
// running without the graph provider. The other half of the pair - the link
// present, and equal to the capability href, with the plugin loaded - is
// `test_graph_provider_plugin.test.py::test_01_function_detail_includes_graph_capability`,
// because registering a plugin capability needs a real loaded plugin.
//
// @verifies REQ_INTEROP_003
TEST_F(DiscoveryHandlersFixtureTest, GetFunctionOmitsGraphLinkWithoutAPlugin) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/navigation", R"(/api/v1/functions/([^/]+))");

  auto result = handlers_->get_function(typed_req);
  auto body = body_json(result);
  EXPECT_EQ(body["hosts"], "/api/v1/functions/navigation/hosts");
  EXPECT_FALSE(body.contains("x-medkit-graph"));
  EXPECT_EQ(body["_links"]["self"], "/api/v1/functions/navigation");
  EXPECT_EQ(body["x-medkit"]["source"], "manifest");
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersValidationTest, FunctionHostsInvalidIdReturns400) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/bad/id/hosts", R"(/api/v1/functions/(.+)/hosts)");

  auto result = handlers_.get_function_hosts(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersFixtureTest, FunctionHostsUnknownFunctionReturns404) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/unknown/hosts", R"(/api/v1/functions/([^/]+)/hosts)");

  auto result = handlers_->get_function_hosts(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
}

// @verifies REQ_INTEROP_007
TEST_F(DiscoveryHandlersFixtureTest, FunctionHostsReturnsHostingApps) {
  httplib::Request req;
  auto typed_req = make_typed_request(req, "/api/v1/functions/navigation/hosts", R"(/api/v1/functions/([^/]+)/hosts)");

  auto result = handlers_->get_function_hosts(typed_req);
  auto body = body_json(result);
  ASSERT_EQ(body["items"].size(), 1u);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  // Reads from cache where planner app has is_online=true (set in SetUp)
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], true);
  EXPECT_EQ(body["_links"]["function"], "/api/v1/functions/navigation");
}
