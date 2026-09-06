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

// Tests for plugin_context.cpp aggregation fan-out in:
//   - list_entity_faults (peers queried via /api/v1/<type>/<id>/faults)
//   - list_all_faults (peers queried via /api/v1/faults)
//   - get_entity_snapshot (peer entities already in cache from discovery refresh)

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <httplib.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::GatewayNode;

namespace {

// ---------------------------------------------------------------------------
// Port / temp manifest helpers (same pattern as test_plugin_notify_integration)
// ---------------------------------------------------------------------------

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "socket() failed: " << std::strerror(errno);
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ADD_FAILURE() << "bind() failed: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  socklen_t len = sizeof(addr);
  getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len);
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

// Write a temp manifest file; returns the path.
std::string write_temp_manifest(const std::string & contents) {
  char tmpl[] = "/tmp/ros2_medkit_plugin_ctx_agg_XXXXXX.yaml";
  int fd = mkstemps(tmpl, 5);
  if (fd < 0) {
    ADD_FAILURE() << "mkstemps() failed: " << std::strerror(errno);
    return {};
  }
  close(fd);
  std::ofstream out(tmpl);
  out << contents;
  return tmpl;
}

// ---------------------------------------------------------------------------
// RAII mock HTTP peer server (same pattern as test_aggregation_manager)
// ---------------------------------------------------------------------------

class MockPeerServer {
 public:
  MockPeerServer() = default;

  ~MockPeerServer() {
    if (server_) {
      server_->stop();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  MockPeerServer(const MockPeerServer &) = delete;
  MockPeerServer & operator=(const MockPeerServer &) = delete;
  MockPeerServer(MockPeerServer &&) = delete;
  MockPeerServer & operator=(MockPeerServer &&) = delete;

  httplib::Server & server() {
    if (!server_) {
      server_ = std::make_unique<httplib::Server>();
    }
    return *server_;
  }

  int start() {
    server();  // ensure server_ is constructed even if start() is called before server()
    port_ = server_->bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this]() {
      server_->listen_after_bind();
    });
    // stop() only interrupts a server that is already listening.
    server_->wait_until_ready();
    return port_;
  }

  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

 private:
  std::unique_ptr<httplib::Server> server_;
  std::thread thread_;
  int port_{0};
};

// ---------------------------------------------------------------------------
// Offline fake fault transport - lets tests exercise the LOCAL fault path of
// list_entity_faults / list_all_faults without a live ros2_medkit_fault_manager.
// Returns the same {"faults":[...], "count":N} envelope shape as the real
// Ros2FaultServiceTransport. The plugin context always queries with an empty
// source_id and scopes locally, so the full set is returned regardless.
// ---------------------------------------------------------------------------

class FakeFaultTransport : public ros2_medkit_gateway::FaultServiceTransport {
 public:
  explicit FakeFaultTransport(json faults) : faults_(std::move(faults)) {
  }

  ros2_medkit_gateway::FaultResult list_faults(const std::string & /*source_id*/, bool /*include_prefailed*/,
                                               bool /*include_confirmed*/, bool /*include_cleared*/,
                                               bool /*include_healed*/, bool /*include_muted*/,
                                               bool /*include_clusters*/) override {
    ros2_medkit_gateway::FaultResult r;
    r.success = true;
    r.data = {{"faults", faults_}, {"count", faults_.size()}};
    return r;
  }

  bool is_available() const override {
    return true;
  }

  // Unused by these tests - return benign failures.
  ros2_medkit_gateway::FaultResult report_fault(const std::string & /*fault_code*/, uint8_t /*severity*/,
                                                const std::string & /*description*/,
                                                const std::string & /*source_id*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::FaultWithEnvJsonResult get_fault_with_env(const std::string & /*fault_code*/,
                                                                 const std::string & /*source_id*/) override {
    return {false, "not implemented", json::object()};
  }
  ros2_medkit_gateway::FaultResult get_fault(const std::string & /*fault_code*/,
                                             const std::string & /*source_id*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::FaultResult clear_fault(const std::string & /*fault_code*/,
                                               bool /*skip_correlation_auto_clear*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::FaultResult get_snapshots(const std::string & /*fault_code*/,
                                                 const std::string & /*topic*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::FaultResult get_rosbag(const std::string & /*fault_code*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::FaultResult list_rosbags(const std::string & /*entity_fqn*/) override {
    return {false, json::object(), "not implemented"};
  }
  ros2_medkit_gateway::PlannedStopResult
  declare_planned_stop(const ros2_medkit_gateway::faults::PlannedStopWindow & /*request*/) override {
    return {};
  }
  ros2_medkit_gateway::PlannedStopResult end_planned_stop(const std::string & /*id*/) override {
    return {};
  }
  /// Succeeds with no windows, so a fault list built through this fake carries
  /// `expected: false` rather than no flag at all - the shape a gateway with a
  /// reachable fault manager and nothing declared actually serves.
  ros2_medkit_gateway::PlannedStopResult list_planned_stops(bool /*active_only*/) override {
    ros2_medkit_gateway::PlannedStopResult r;
    r.success = true;
    return r;
  }
  bool wait_for_services(std::chrono::duration<double> /*timeout*/) override {
    return true;
  }

 private:
  json faults_;
};

// Two canned faults: one reported by the manifest app "brakes" (/vehicle/brakes),
// one by an unrelated node (/other/sensor) used to prove entity scoping excludes
// out-of-scope faults.
json make_local_faults() {
  return json::array({
      {{"fault_code", "LOCAL_BRAKE_FAULT"},
       {"description", "Local brake fault"},
       {"status", "ACTIVE"},
       {"reporting_sources", json::array({"/vehicle/brakes"})}},
      {{"fault_code", "OTHER_NODE_FAULT"},
       {"description", "Fault from an unrelated node"},
       {"status", "ACTIVE"},
       {"reporting_sources", json::array({"/other/sensor"})}},
  });
}

// ---------------------------------------------------------------------------
// Base manifest with one component ("brakes_ecu") + one app ("brakes")
// ---------------------------------------------------------------------------

constexpr const char * kManifest = R"(
manifest_version: "1.0"
metadata:
  name: plugin-ctx-agg-test
  version: "0.0.1"
areas:
  - id: vehicle
    name: Vehicle
components:
  - id: brakes_ecu
    name: Brakes ECU
    area: vehicle
apps:
  - id: brakes
    name: Brakes
    is_located_on: brakes_ecu
    ros_binding:
      node_name: brakes
      namespace: /vehicle
functions:
  - id: braking
    name: Braking
    hosts:
      - brakes
)";

// ---------------------------------------------------------------------------
// Fixture: GatewayNode without aggregation (local-only behavior)
// ---------------------------------------------------------------------------

class PluginContextNoAggTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifest);
    ASSERT_FALSE(manifest_path_.empty());

    const int port = reserve_local_port();
    ASSERT_GT(port, 0);

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "manifest_only"},
        {"discovery.manifest_path", manifest_path_},
        {"discovery.manifest_strict_validation", false},
        {"discovery.runtime.enabled", false},
        {"discovery.runtime.default_component.enabled", false},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", port},
        // No aggregation.enabled -> get_aggregation_manager() returns nullptr
    });
    node_ = std::make_shared<GatewayNode>(opts);
  }

  void TearDown() override {
    node_.reset();
    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  std::shared_ptr<GatewayNode> node_;
  std::string manifest_path_;
};

// ---------------------------------------------------------------------------
// Fixture: GatewayNode with aggregation enabled, mock peer provides faults
// ---------------------------------------------------------------------------

class PluginContextWithPeerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifest);
    ASSERT_FALSE(manifest_path_.empty());

    // Install fault endpoints on the mock peer.
    // /api/v1/health - required for peer health check
    mock_.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"status":"healthy"})", "application/json");
    });

    // /api/v1/apps/brakes/faults - returns a PROTECTIVE_STOP fault
    mock_.server().Get(R"(/api/v1/apps/brakes/faults)", [](const httplib::Request &, httplib::Response & res) {
      json item = {
          {"fault_code", "PROTECTIVE_STOP"}, {"description", "Emergency stop triggered"}, {"status", "ACTIVE"}};
      res.set_content(json({{"items", json::array({item})}}).dump(), "application/json");
    });

    // /api/v1/faults - global faults list (items array)
    mock_.server().Get("/api/v1/faults", [](const httplib::Request &, httplib::Response & res) {
      json item = {{"fault_code", "PEER_FAULT"}, {"description", "Fault from peer"}, {"status", "ACTIVE"}};
      res.set_content(json({{"items", json::array({item})}}).dump(), "application/json");
    });

    // Discovery endpoints (empty) - needed during GatewayNode startup discovery refresh
    mock_.server().Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"items":[]})", "application/json");
    });
    mock_.server().Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"items":[]})", "application/json");
    });
    mock_.server().Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"items":[]})", "application/json");
    });
    mock_.server().Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"items":[]})", "application/json");
    });

    int peer_port = mock_.start();
    ASSERT_GT(peer_port, 0);
    peer_url_ = mock_.url();

    const int gw_port = reserve_local_port();
    ASSERT_GT(gw_port, 0);

    // Configure the GatewayNode with aggregation pointing at the mock peer.
    // Static peers use parallel arrays: aggregation.peer_urls / aggregation.peer_names.
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "manifest_only"},
        {"discovery.manifest_path", manifest_path_},
        {"discovery.manifest_strict_validation", false},
        {"discovery.runtime.enabled", false},
        {"discovery.runtime.default_component.enabled", false},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", gw_port},
        {"aggregation.enabled", true},
        {"aggregation.timeout_ms", static_cast<int64_t>(5000)},
        {"aggregation.peer_urls", std::vector<std::string>{peer_url_}},
        {"aggregation.peer_names", std::vector<std::string>{"mock_peer"}},
    });
    node_ = std::make_shared<GatewayNode>(opts);

    // Make the peer healthy so fan-out actually sends requests to it.
    auto * agg = node_->get_aggregation_manager();
    ASSERT_NE(agg, nullptr) << "AggregationManager must be non-null when aggregation.enabled=true";
    agg->check_all_health();
    ASSERT_EQ(agg->healthy_peer_count(), 1u) << "mock peer must be healthy for fan-out to work";
  }

  void TearDown() override {
    node_.reset();
    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  std::shared_ptr<GatewayNode> node_;
  MockPeerServer mock_;
  std::string manifest_path_;
  std::string peer_url_;
};

// ---------------------------------------------------------------------------
// Fixture: aggregation enabled, peer is healthy at /health but FAILS every
// fault request (503). Exercises the partial-fan-out path - the peer
// contributes nothing and local faults must still be returned.
// ---------------------------------------------------------------------------

class PluginContextFailingPeerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifest);
    ASSERT_FALSE(manifest_path_.empty());

    mock_.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"status":"healthy"})", "application/json");
    });

    // Every fault query 503s, so fan-out marks the peer failed (is_partial).
    auto fail = [](const httplib::Request &, httplib::Response & res) {
      res.status = 503;
      res.set_content(R"({"error":"unavailable"})", "application/json");
    };
    mock_.server().Get(R"(/api/v1/apps/brakes/faults)", fail);
    mock_.server().Get("/api/v1/faults", fail);

    // Discovery endpoints (empty) - needed during GatewayNode startup discovery refresh
    auto empty_items = [](const httplib::Request &, httplib::Response & res) {
      res.set_content(R"({"items":[]})", "application/json");
    };
    mock_.server().Get("/api/v1/components", empty_items);
    mock_.server().Get("/api/v1/areas", empty_items);
    mock_.server().Get("/api/v1/apps", empty_items);
    mock_.server().Get("/api/v1/functions", empty_items);

    int peer_port = mock_.start();
    ASSERT_GT(peer_port, 0);
    peer_url_ = mock_.url();

    const int gw_port = reserve_local_port();
    ASSERT_GT(gw_port, 0);

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "manifest_only"},
        {"discovery.manifest_path", manifest_path_},
        {"discovery.manifest_strict_validation", false},
        {"discovery.runtime.enabled", false},
        {"discovery.runtime.default_component.enabled", false},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", gw_port},
        {"aggregation.enabled", true},
        {"aggregation.timeout_ms", static_cast<int64_t>(5000)},
        {"aggregation.peer_urls", std::vector<std::string>{peer_url_}},
        {"aggregation.peer_names", std::vector<std::string>{"failing_peer"}},
    });
    node_ = std::make_shared<GatewayNode>(opts);

    auto * agg = node_->get_aggregation_manager();
    ASSERT_NE(agg, nullptr);
    agg->check_all_health();
    ASSERT_EQ(agg->healthy_peer_count(), 1u) << "peer must pass /health so the fault request reaches it and 503s";
  }

  void TearDown() override {
    node_.reset();
    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  std::shared_ptr<GatewayNode> node_;
  MockPeerServer mock_;
  std::string manifest_path_;
  std::string peer_url_;
};

}  // namespace

// =============================================================================
// Tests: no aggregation manager (nullptr) - local-only behavior unchanged
// =============================================================================

TEST_F(PluginContextNoAggTest, ListEntityFaultsNoAgg_ReturnsEmptyArrayWithoutFaultManager) {
  // Fault manager is null, no aggregation. Result: empty array.
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  auto faults = ctx->list_entity_faults("brakes");
  EXPECT_TRUE(faults.is_array());
  EXPECT_EQ(faults.size(), 0u);
}

TEST_F(PluginContextNoAggTest, ListAllFaultsNoAgg_ReturnsEmptyObjectWithoutFaultManager) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  auto result = ctx->list_all_faults();
  // No fault manager, no aggregation -> empty object with no "faults" key
  EXPECT_TRUE(result.is_object());
  EXPECT_FALSE(result.contains("faults"));
}

TEST_F(PluginContextNoAggTest, GetEntitySnapshot_ContainsManifestEntities) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  auto snapshot = ctx->get_entity_snapshot();

  bool found_area = false;
  for (const auto & a : snapshot.areas) {
    if (a.id == "vehicle") {
      found_area = true;
    }
  }
  EXPECT_TRUE(found_area) << "area 'vehicle' should be in snapshot";

  bool found_comp = false;
  for (const auto & c : snapshot.components) {
    if (c.id == "brakes_ecu") {
      found_comp = true;
    }
  }
  EXPECT_TRUE(found_comp) << "component 'brakes_ecu' should be in snapshot";

  bool found_app = false;
  for (const auto & a : snapshot.apps) {
    if (a.id == "brakes") {
      found_app = true;
    }
  }
  EXPECT_TRUE(found_app) << "app 'brakes' should be in snapshot";
}

TEST_F(PluginContextNoAggTest, ListEntityFaultsNoAgg_UnknownEntityReturnsEmpty) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  // "nonexistent" is not in the manifest -> empty array
  auto faults = ctx->list_entity_faults("nonexistent");
  EXPECT_TRUE(faults.is_array());
  EXPECT_EQ(faults.size(), 0u);
}

// =============================================================================
// Tests: with aggregation enabled and a live mock peer
// =============================================================================

// @verifies REQ_INTEROP_013
TEST_F(PluginContextWithPeerTest, ListEntityFaults_IncludesPeerFault) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  // "brakes" is in the local manifest. No local fault manager (null), but the
  // peer returns a PROTECTIVE_STOP fault via /api/v1/apps/brakes/faults.
  auto faults = ctx->list_entity_faults("brakes");

  ASSERT_TRUE(faults.is_array());
  ASSERT_GE(faults.size(), 1u) << "peer fault should be included in the result";

  bool found_peer_fault = false;
  for (const auto & f : faults) {
    if (f.contains("fault_code") && f["fault_code"] == "PROTECTIVE_STOP") {
      found_peer_fault = true;
    }
  }
  EXPECT_TRUE(found_peer_fault) << "PROTECTIVE_STOP fault from peer not found in list_entity_faults result";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextWithPeerTest, ListAllFaults_IncludesPeerFault) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  // No local fault manager (null). Peer returns PEER_FAULT via /api/v1/faults.
  auto result = ctx->list_all_faults();

  // The fan-out appends peer items to the "faults" key.
  ASSERT_TRUE(result.is_object());
  ASSERT_TRUE(result.contains("faults")) << "list_all_faults result must have 'faults' key when peer provides faults";
  ASSERT_TRUE(result["faults"].is_array());
  ASSERT_GE(result["faults"].size(), 1u) << "peer fault should be in 'faults' array";

  bool found_peer_fault = false;
  for (const auto & f : result["faults"]) {
    if (f.contains("fault_code") && f["fault_code"] == "PEER_FAULT") {
      found_peer_fault = true;
    }
  }
  EXPECT_TRUE(found_peer_fault) << "PEER_FAULT from peer not found in list_all_faults result";
}

TEST_F(PluginContextWithPeerTest, GetEntitySnapshot_ContainsLocalManifestEntities) {
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), nullptr, nullptr);
  ASSERT_NE(ctx, nullptr);

  // Snapshot comes from the entity cache which is populated from the manifest
  // (and merged with peer entities during discovery refresh).
  // At minimum, local manifest entities must be present.
  auto snapshot = ctx->get_entity_snapshot();

  bool found_app = false;
  for (const auto & a : snapshot.apps) {
    if (a.id == "brakes") {
      found_app = true;
    }
  }
  EXPECT_TRUE(found_app) << "local app 'brakes' should be in snapshot even with aggregation enabled";
}

// =============================================================================
// Tests: LOCAL fault path with a real FaultManager (fake transport, no peers).
// These exercise the branch that list_*Faults take when fault_manager_ != null,
// which the nullptr-fault-manager tests above never reach.
// =============================================================================

namespace {
bool contains_fault_code(const json & arr, const std::string & code) {
  for (const auto & f : arr) {
    if (f.contains("fault_code") && f["fault_code"] == code) {
      return true;
    }
  }
  return false;
}
}  // namespace

// @verifies REQ_INTEROP_013
TEST_F(PluginContextNoAggTest, ListEntityFaults_LocalFaultManager_IncludesScopedFaultOnly) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  // App "brakes" resolves to /vehicle/brakes. The local fault reported by that
  // node must be present; the out-of-scope /other/sensor fault must NOT be.
  auto faults = ctx->list_entity_faults("brakes");
  ASSERT_TRUE(faults.is_array());
  EXPECT_TRUE(contains_fault_code(faults, "LOCAL_BRAKE_FAULT"))
      << "local fault for app 'brakes' must be returned (regression guard for the .data shape bug)";
  EXPECT_FALSE(contains_fault_code(faults, "OTHER_NODE_FAULT"))
      << "fault from /other/sensor must be scoped out of app 'brakes'";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextNoAggTest, ListEntityFaults_AreaResolvesToDescendantApps) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  // Area "vehicle" -> component "brakes_ecu" -> app "brakes" (/vehicle/brakes).
  // The brakes fault must surface for the area; /other/sensor must not leak in.
  auto faults = ctx->list_entity_faults("vehicle");
  ASSERT_TRUE(faults.is_array());
  EXPECT_TRUE(contains_fault_code(faults, "LOCAL_BRAKE_FAULT"))
      << "area 'vehicle' must aggregate faults from its descendant app 'brakes'";
  EXPECT_FALSE(contains_fault_code(faults, "OTHER_NODE_FAULT"))
      << "out-of-area fault from /other/sensor must not leak into area 'vehicle'";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextWithPeerTest, ListEntityFaults_MergesLocalAndPeerFaults) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  // Local /vehicle/brakes fault AND the peer's PROTECTIVE_STOP must both appear.
  auto faults = ctx->list_entity_faults("brakes");
  ASSERT_TRUE(faults.is_array());
  EXPECT_TRUE(contains_fault_code(faults, "LOCAL_BRAKE_FAULT")) << "local fault missing from merged result";
  EXPECT_TRUE(contains_fault_code(faults, "PROTECTIVE_STOP")) << "peer fault missing from merged result";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextWithPeerTest, ListAllFaults_MergesLocalAndPeerFaults) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  auto result = ctx->list_all_faults();
  ASSERT_TRUE(result.is_object());
  ASSERT_TRUE(result.contains("faults"));
  ASSERT_TRUE(result["faults"].is_array());
  EXPECT_TRUE(contains_fault_code(result["faults"], "LOCAL_BRAKE_FAULT"))
      << "local faults must be present in list_all_faults";
  EXPECT_TRUE(contains_fault_code(result["faults"], "PEER_FAULT")) << "peer faults must be merged into list_all_faults";

  // "count" must reflect the merged (local + peer) array, not the local-only query.
  if (result.contains("count")) {
    EXPECT_EQ(result["count"].get<size_t>(), result["faults"].size())
        << "count must be recomputed after peer faults are merged in";
  }
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextNoAggTest, ListEntityFaults_FunctionResolvesToHostAppFaults) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  // Function "braking" hosts app "brakes" (/vehicle/brakes); its fault must
  // surface, and the unrelated /other/sensor fault must not.
  auto faults = ctx->list_entity_faults("braking");
  ASSERT_TRUE(faults.is_array());
  EXPECT_TRUE(contains_fault_code(faults, "LOCAL_BRAKE_FAULT"))
      << "function must aggregate faults from its host app 'brakes'";
  EXPECT_FALSE(contains_fault_code(faults, "OTHER_NODE_FAULT"))
      << "out-of-scope fault must not leak into function 'braking'";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextFailingPeerTest, ListEntityFaults_FailingPeerPreservesLocalFaults) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  // The peer 503s on the fault query (partial fan-out), so it contributes
  // nothing - but the local fault must still be returned, not silently lost.
  auto faults = ctx->list_entity_faults("brakes");
  ASSERT_TRUE(faults.is_array());
  EXPECT_TRUE(contains_fault_code(faults, "LOCAL_BRAKE_FAULT"))
      << "local faults must survive when a peer fails the fan-out";
}

// @verifies REQ_INTEROP_013
TEST_F(PluginContextFailingPeerTest, ListAllFaults_FailingPeerPreservesLocalFaults) {
  ros2_medkit_gateway::FaultManager fm(std::make_shared<FakeFaultTransport>(make_local_faults()));
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node_.get(), &fm, nullptr);
  ASSERT_NE(ctx, nullptr);

  auto result = ctx->list_all_faults();
  ASSERT_TRUE(result.is_object());
  ASSERT_TRUE(result.contains("faults"));
  EXPECT_TRUE(contains_fault_code(result["faults"], "LOCAL_BRAKE_FAULT"))
      << "local faults must survive when a peer fails the fan-out";
}
