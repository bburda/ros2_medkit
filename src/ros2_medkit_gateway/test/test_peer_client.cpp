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
#include <httplib.h>

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/core/aggregation/peer_client.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

using namespace ros2_medkit_gateway;

// Use a port that nothing listens on for connection-refused tests
constexpr int DEAD_PORT = 59999;
static const std::string DEAD_URL = "http://127.0.0.1:" + std::to_string(DEAD_PORT);

// =============================================================================
// Construction and basic accessor tests
// =============================================================================

TEST(PeerClient, constructs_with_url_and_name) {
  PeerClient client("http://localhost:8081", "subsystem_b", 500);

  EXPECT_EQ(client.url(), "http://localhost:8081");
  EXPECT_EQ(client.name(), "subsystem_b");
}

TEST(PeerClient, initially_not_healthy) {
  PeerClient client(DEAD_URL, "dead_peer", 500);

  EXPECT_FALSE(client.is_healthy());
}

// =============================================================================
// Health check tests (connection refused path)
// =============================================================================

TEST(PeerClient, health_check_marks_unhealthy_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  // Should not throw, just mark unhealthy
  client.check_health();
  EXPECT_FALSE(client.is_healthy());
}

// =============================================================================
// Forward request tests (connection failure path)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(PeerClient, forward_sets_502_on_connection_error) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components";
  httplib::Response res;

  client.forward_request(req, res);

  EXPECT_EQ(res.status, 502);

  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-peer-unavailable");
  EXPECT_TRUE(body["message"].get<std::string>().find("dead_peer") != std::string::npos);
}

// =============================================================================
// forward_and_get_json tests (connection failure path)
// =============================================================================

TEST(PeerClient, forward_and_get_json_returns_error_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  auto result = client.forward_and_get_json("GET", "/api/v1/health");

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("dead_peer") != std::string::npos);
  EXPECT_TRUE(result.error().find("Failed to connect") != std::string::npos);
}

// =============================================================================
// fetch_entities tests (connection failure path)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(PeerClient, fetch_entities_returns_error_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  auto result = client.fetch_entities();

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("dead_peer") != std::string::npos);
}

// =============================================================================
// EntityInfo remote field tests
// =============================================================================

TEST(EntityInfoRemote, defaults_to_local) {
  EntityInfo info;
  EXPECT_FALSE(info.is_remote);
  EXPECT_TRUE(info.peer_url.empty());
  EXPECT_TRUE(info.peer_name.empty());
}

TEST(EntityInfoRemote, can_be_marked_remote) {
  EntityInfo info;
  info.is_remote = true;
  info.peer_url = "http://192.168.1.10:8081";
  info.peer_name = "subsystem_a";

  EXPECT_TRUE(info.is_remote);
  EXPECT_EQ(info.peer_url, "http://192.168.1.10:8081");
  EXPECT_EQ(info.peer_name, "subsystem_a");
}

// =============================================================================
// Mock server happy-path tests (local httplib::Server)
// =============================================================================

#include <thread>

TEST(PeerClientHappyPath, health_check_marks_healthy) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  EXPECT_FALSE(client.is_healthy());

  client.check_health();
  EXPECT_TRUE(client.is_healthy());

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, health_check_unhealthy_on_500) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
    res.set_content(R"({"status":"error"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  client.check_health();
  EXPECT_FALSE(client.is_healthy());

  svr.stop();
  t.join();
}

// @verifies REQ_INTEROP_003
TEST(PeerClientHappyPath, fetch_entities_parses_collections) {
  httplib::Server svr;
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"zone_a","name":"Zone A"}]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu_1","name":"ECU 1"}]})", "application/json");
  });
  // A peer that names a Component in its list describes it on the detail route.
  svr.Get(R"(/api/v1/components/ecu_1)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu_1","name":"ECU 1"})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"nav","name":"Navigation"}]})", "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->areas.size(), 1u);
  EXPECT_EQ(result->areas[0].id, "zone_a");
  EXPECT_EQ(result->areas[0].name, "Zone A");
  EXPECT_EQ(result->areas[0].source, "peer:test_peer");
  EXPECT_EQ(result->components.size(), 1u);
  EXPECT_EQ(result->components[0].id, "ecu_1");
  EXPECT_EQ(result->components[0].source, "peer:test_peer");
  EXPECT_EQ(result->apps.size(), 1u);
  EXPECT_EQ(result->apps[0].id, "nav");
  EXPECT_EQ(result->apps[0].source, "peer:test_peer");
  EXPECT_EQ(result->functions.size(), 0u);

  svr.stop();
  t.join();
}

// @verifies REQ_INTEROP_003
TEST(PeerClientHappyPath, fetch_entities_parses_relationship_fields) {
  httplib::Server svr;

  // Areas: top-level only in list (subareas filtered)
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"vehicle","name":"Vehicle"}
        ]})",
        "application/json");
  });
  // Subareas endpoint for "vehicle"
  svr.Get(R"(/api/v1/areas/vehicle/subareas)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"sensors","name":"Sensors","x-medkit":{"namespace":"/sensors"}}
        ]})",
        "application/json");
  });

  // Components: top-level only in list (subcomponents filtered)
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"robot-alpha","name":"Robot Alpha","x-medkit":{"source":"manifest"}}
        ]})",
        "application/json");
  });
  // Component detail (for relationship data)
  svr.Get(R"(/api/v1/components/robot-alpha)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"robot-alpha","name":"Robot Alpha","x-medkit":{"source":"manifest"}})",
                    "application/json");
  });
  // Subcomponents endpoint for "robot-alpha"
  svr.Get(R"(/api/v1/components/robot-alpha/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"perception-ecu","name":"Perception ECU",
           "x-medkit":{"source":"manifest","namespace":"/perception"}}
        ]})",
        "application/json");
  });
  // Subcomponent detail
  svr.Get(R"(/api/v1/components/perception-ecu)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"id":"perception-ecu","name":"Perception ECU",
           "x-medkit":{"parentComponentId":"robot-alpha","dependsOn":["compute-unit"],
                       "source":"manifest","namespace":"/perception"}})",
        "application/json");
  });
  // No subcomponents for perception-ecu (leaf)
  svr.Get(R"(/api/v1/components/perception-ecu/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  // Apps response matches what real SOVD peers emit: is-located-on is the
  // standard app->component relationship (ISO 17978-3, §7.6), and the gateway
  // also emits x-medkit.component_id (snake_case) as a vendor extension.
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"lidar-driver","name":"Lidar Driver",
           "is-located-on":"/api/v1/components/perception-ecu",
           "x-medkit":{"component_id":"perception-ecu","source":"manifest","is_online":true}}
        ]})",
        "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"autonomous-navigation","name":"Autonomous Navigation","x-medkit":{"source":"manifest"}}
        ]})",
        "application/json");
  });
  // hosts are carried by the detail route only, which is where a real gateway
  // puts them.
  svr.Get(R"(/api/v1/functions/autonomous-navigation)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"id":"autonomous-navigation","name":"Autonomous Navigation",
            "x-medkit":{"hosts":["lidar-driver","path-planner"],"source":"manifest"}})",
        "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "peer_ecu", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());

  // Areas: top-level + subareas fetched
  ASSERT_EQ(result->areas.size(), 2u);
  EXPECT_EQ(result->areas[0].id, "vehicle");
  EXPECT_EQ(result->areas[1].id, "sensors");

  // Components: top-level + subcomponents fetched (robot-alpha + perception-ecu)
  ASSERT_EQ(result->components.size(), 2u);
  EXPECT_EQ(result->components[0].id, "robot-alpha");
  EXPECT_EQ(result->components[1].id, "perception-ecu");
  EXPECT_EQ(result->components[1].parent_component_id, "robot-alpha");
  ASSERT_EQ(result->components[1].depends_on.size(), 1u);
  EXPECT_EQ(result->components[1].depends_on[0], "compute-unit");

  // App: component_id parsed
  ASSERT_EQ(result->apps.size(), 1u);
  EXPECT_EQ(result->apps[0].component_id, "perception-ecu");

  // Function: hosts parsed
  ASSERT_EQ(result->functions.size(), 1u);
  EXPECT_EQ(result->functions[0].id, "autonomous-navigation");
  ASSERT_EQ(result->functions[0].hosts.size(), 2u);
  EXPECT_EQ(result->functions[0].hosts[0], "lidar-driver");
  EXPECT_EQ(result->functions[0].hosts[1], "path-planner");

  svr.stop();
  t.join();
}

// App->component binding via SOVD-standard is-located-on relationship only
// (no x-medkit vendor extension). Exercises the parse path used by peers
// implementing the ISO 17978-3 standard from other vendors.
// @verifies REQ_INTEROP_018
TEST(PeerClientHappyPath, fetch_entities_parses_is_located_on_without_vendor_extension) {
  httplib::Server svr;

  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu-a","name":"ECU A"}]})", "application/json");
  });
  svr.Get(R"(/api/v1/components/ecu-a)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu-a","name":"ECU A"})", "application/json");
  });
  svr.Get(R"(/api/v1/components/ecu-a/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  // Exercises multiple URI forms: absolute URL, path-only _links (string),
  // HAL+JSON object-form _links, trailing slash, query string, and fragment.
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"app-standard","name":"Standard App",
           "is-located-on":"http://peer.local:8080/api/v1/components/ecu-a"},
          {"id":"app-hateoas","name":"HATEOAS App",
           "_links":{"is-located-on":"/api/v1/components/ecu-a"}},
          {"id":"app-hal-object","name":"HAL Object App",
           "_links":{"is-located-on":{"href":"/api/v1/components/ecu-a"}}},
          {"id":"app-trailing-slash","name":"Trailing Slash",
           "is-located-on":"/api/v1/components/ecu-a/"},
          {"id":"app-query","name":"Query Param",
           "is-located-on":"/api/v1/components/ecu-a?foo=bar"},
          {"id":"app-fragment","name":"Fragment",
           "is-located-on":"/api/v1/components/ecu-a#section"}
        ]})",
        "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "peer_sovd", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->apps.size(), 6u);
  EXPECT_EQ(result->apps[0].id, "app-standard");
  EXPECT_EQ(result->apps[0].component_id, "ecu-a");
  EXPECT_EQ(result->apps[1].id, "app-hateoas");
  EXPECT_EQ(result->apps[1].component_id, "ecu-a");
  EXPECT_EQ(result->apps[2].id, "app-hal-object");
  EXPECT_EQ(result->apps[2].component_id, "ecu-a");
  EXPECT_EQ(result->apps[3].id, "app-trailing-slash");
  EXPECT_EQ(result->apps[3].component_id, "ecu-a");
  EXPECT_EQ(result->apps[4].id, "app-query");
  EXPECT_EQ(result->apps[4].component_id, "ecu-a");
  EXPECT_EQ(result->apps[5].id, "app-fragment");
  EXPECT_EQ(result->apps[5].component_id, "ecu-a");

  svr.stop();
  t.join();
}

// Malicious peer attempts to smuggle path traversal / percent-encoding /
// oversize IDs through the app->component binding. Both the SOVD-standard
// is-located-on parser and the x-medkit vendor fallback must reject these
// candidates (leaving App::component_id empty) so they cannot propagate into
// the aggregator's entity cache or response-side path construction.
// @verifies REQ_INTEROP_018
TEST(PeerClientHappyPath, fetch_entities_rejects_malicious_component_id_in_located_on) {
  httplib::Server svr;

  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"app-traversal","name":"A","is-located-on":"/api/v1/components/..%2Fetc%2Fpasswd"},
          {"id":"app-percent","name":"A","is-located-on":"/api/v1/components/bad%20id"},
          {"id":"app-vendor-slash","name":"A",
           "x-medkit":{"component_id":"bad/id","source":"manifest"}},
          {"id":"app-vendor-empty","name":"A",
           "x-medkit":{"component_id":"","source":"manifest"}}
        ]})",
        "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "peer_hostile", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->apps.size(), 4u);
  for (const auto & app : result->apps) {
    EXPECT_TRUE(app.component_id.empty())
        << "app " << app.id << " should have empty component_id, got '" << app.component_id << "'";
  }

  svr.stop();
  t.join();
}

// Vendor-only fallback: app has x-medkit.component_id but no is-located-on.
// Covers the happy-path for peers using the gateway's own vendor extension
// without the SOVD standard relationship field.
// @verifies REQ_INTEROP_018
TEST(PeerClientHappyPath, fetch_entities_parses_vendor_only_component_id_fallback) {
  httplib::Server svr;

  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"perception-ecu","name":"Perception ECU"}]})", "application/json");
  });
  svr.Get(R"(/api/v1/components/perception-ecu)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"perception-ecu","name":"Perception ECU"})", "application/json");
  });
  svr.Get(R"(/api/v1/components/perception-ecu/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"lidar-driver","name":"Lidar Driver",
           "x-medkit":{"component_id":"perception-ecu","source":"manifest","is_online":true}}
        ]})",
        "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "peer_vendor", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->apps.size(), 1u);
  EXPECT_EQ(result->apps[0].id, "lidar-driver");
  EXPECT_EQ(result->apps[0].component_id, "perception-ecu");

  svr.stop();
  t.join();
}

// @verifies REQ_INTEROP_018
TEST(PeerClientHappyPath, forward_request_proxies_response_with_auth) {
  httplib::Server svr;
  svr.Get(R"(/api/v1/apps/nav/data)", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["forwarded"] = true;
    body["has_auth"] = req.has_header("Authorization");
    if (req.has_header("Authorization")) {
      body["auth"] = req.get_header_value("Authorization");
    }
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  // forward_auth=true: Authorization header should be forwarded
  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000, true);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/nav/data";
  req.set_header("Authorization", "Bearer test-token");

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body["forwarded"].get<bool>());
  EXPECT_TRUE(body["has_auth"].get<bool>());
  EXPECT_EQ(body["auth"].get<std::string>(), "Bearer test-token");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_request_does_not_forward_auth_by_default) {
  httplib::Server svr;
  svr.Get(R"(/api/v1/apps/nav/data)", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["forwarded"] = true;
    body["has_auth"] = req.has_header("Authorization");
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  // forward_auth=false (default): Authorization header should NOT be forwarded
  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/nav/data";
  req.set_header("Authorization", "Bearer test-token");

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body["forwarded"].get<bool>());
  EXPECT_FALSE(body["has_auth"].get<bool>());

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_filters_response_headers) {
  httplib::Server svr;
  svr.Get("/api/v1/test", [](const httplib::Request &, httplib::Response & res) {
    res.set_header("ETag", "\"abc123\"");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Set-Cookie", "evil=1");
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("X-Medkit-Local-Only", "true");
    res.set_header("X-Medkit-Custom", "injected");
    res.set_content("{}", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/test";

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  // Safe headers should be present
  EXPECT_TRUE(res.has_header("ETag"));
  EXPECT_TRUE(res.has_header("Cache-Control"));
  // Allowlisted x-medkit headers should be forwarded
  EXPECT_TRUE(res.has_header("X-Medkit-Local-Only"));
  // Non-allowlisted x-medkit headers should be blocked (injection prevention)
  EXPECT_FALSE(res.has_header("X-Medkit-Custom"));
  // Dangerous headers should be filtered
  EXPECT_FALSE(res.has_header("Set-Cookie"));
  EXPECT_FALSE(res.has_header("Access-Control-Allow-Origin"));

  svr.stop();
  t.join();
}

// @verifies REQ_INTEROP_018
TEST(PeerClientHappyPath, forward_and_get_json_returns_parsed_json) {
  httplib::Server svr;
  svr.Get("/api/v1/components/ecu/data", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"temperature":42.5,"status":"ok"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/components/ecu/data");

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->at("temperature").get<double>(), 42.5);
  EXPECT_EQ(result->at("status").get<std::string>(), "ok");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_with_auth_header_when_enabled) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["has_auth"] = req.has_header("Authorization");
    if (req.has_header("Authorization")) {
      body["auth_value"] = req.get_header_value("Authorization");
    }
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  // forward_auth=true: auth header should be sent
  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000, true);
  auto result = client.forward_and_get_json("GET", "/api/v1/health", "Bearer my-jwt");

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->at("has_auth").get<bool>());
  EXPECT_EQ(result->at("auth_value").get<std::string>(), "Bearer my-jwt");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_does_not_forward_auth_by_default) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["has_auth"] = req.has_header("Authorization");
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  // forward_auth=false (default): auth header should NOT be sent
  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/health", "Bearer my-jwt");

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->at("has_auth").get<bool>());

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_error_on_non_2xx) {
  httplib::Server svr;
  svr.Get("/api/v1/missing", [](const httplib::Request &, httplib::Response & res) {
    res.status = 404;
    res.set_content(R"({"error":"not found"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/missing");

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("404") != std::string::npos);
  EXPECT_TRUE(result.error().find("test_peer") != std::string::npos);

  svr.stop();
  t.join();
}

// @verifies REQ_INTEROP_003
TEST(PeerClientHappyPath, forward_request_rejects_oversized_response) {
  httplib::Server svr;

  // Respond with a body slightly over MAX_PEER_RESPONSE_SIZE (10MB)
  svr.Get("/api/v1/apps/big/data", [](const httplib::Request &, httplib::Response & res) {
    // 10MB + 1 byte to exceed the limit
    std::string large_body(10 * 1024 * 1024 + 1, 'x');
    res.set_content(large_body, "application/octet-stream");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "big_peer", 10000);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/big/data";
  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 502);

  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-peer-unavailable");
  EXPECT_TRUE(body["message"].get<std::string>().find("size limit") != std::string::npos);

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_rejects_oversized_response) {
  httplib::Server svr;

  // Respond with a body slightly over MAX_PEER_RESPONSE_SIZE (10MB)
  svr.Get("/api/v1/components/big/data", [](const httplib::Request &, httplib::Response & res) {
    // Build a valid JSON body that exceeds 10MB
    std::string padding(10 * 1024 * 1024, 'a');
    nlohmann::json j;
    j["payload"] = padding;
    res.set_content(j.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "big_peer", 10000);
  auto result = client.forward_and_get_json("GET", "/api/v1/components/big/data");

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("size limit") != std::string::npos);
  EXPECT_TRUE(result.error().find("big_peer") != std::string::npos);

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_post_request) {
  httplib::Server svr;
  svr.Post("/api/v1/apps/nav/operations/restart", [](const httplib::Request & req, httplib::Response & res) {
    auto body = nlohmann::json::parse(req.body, nullptr, false);
    nlohmann::json response;
    response["executed"] = true;
    response["received_body"] = body;
    res.set_content(response.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "POST";
  req.path = "/api/v1/apps/nav/operations/restart";
  req.body = R"({"force":true})";
  req.set_header("Content-Type", "application/json");

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body["executed"].get<bool>());
  EXPECT_TRUE(body["received_body"]["force"].get<bool>());

  svr.stop();
  t.join();
}

// =============================================================================
// Entity ID validation tests - path traversal IDs are filtered
// =============================================================================

TEST(PeerClientHappyPath, fetch_entities_skips_entities_with_invalid_ids) {
  httplib::Server svr;
  // Return entities with path-traversal and other invalid IDs
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "valid_area"}, {"name", "Valid Area"}});
    items.push_back({{"id", "../etc/passwd"}, {"name", "Traversal Area"}});
    items.push_back({{"id", "area with spaces"}, {"name", "Spaced Area"}});
    items.push_back({{"id", "area/slash"}, {"name", "Slash Area"}});
    items.push_back({{"id", ""}, {"name", "Empty ID"}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  svr.Get(R"(/api/v1/areas/([^/]+)/subareas)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "valid-comp"}, {"name", "Valid"}});
    items.push_back({{"id", "comp%00null"}, {"name", "Null Byte"}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  svr.Get(R"(/api/v1/components/([^/]+))", [](const httplib::Request & req, httplib::Response & res) {
    std::string match = req.matches[1].str();
    res.set_content(nlohmann::json({{"id", match}, {"name", match}}).dump(), "application/json");
  });
  svr.Get(R"(/api/v1/components/([^/]+)/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "good_app"}, {"name", "Good"}});
    items.push_back({{"id", "../../traversal"}, {"name", "Bad"}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "ok_func"}, {"name", "OK"}});
    items.push_back({{"id", "func;injection"}, {"name", "Injected"}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  svr.Get(R"(/api/v1/functions/([^/]+))", [](const httplib::Request & req, httplib::Response & res) {
    std::string match = req.matches[1].str();
    res.set_content(nlohmann::json({{"id", match}, {"name", match}}).dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "malicious_peer", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  // Only valid entities should survive
  EXPECT_EQ(result->areas.size(), 1u);
  EXPECT_EQ(result->areas[0].id, "valid_area");
  EXPECT_EQ(result->components.size(), 1u);
  EXPECT_EQ(result->components[0].id, "valid-comp");
  EXPECT_EQ(result->apps.size(), 1u);
  EXPECT_EQ(result->apps[0].id, "good_app");
  EXPECT_EQ(result->functions.size(), 1u);
  EXPECT_EQ(result->functions[0].id, "ok_func");

  svr.stop();
  t.join();
}

// =============================================================================
// Per-collection limit tests
// =============================================================================

TEST(PeerClientHappyPath, fetch_entities_rejects_collection_exceeding_limit) {
  httplib::Server svr;
  // Return 1001 areas (exceeds MAX_ENTITIES_PER_COLLECTION = 1000)
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    for (int i = 0; i < 1001; ++i) {
      items.push_back({{"id", "area_" + std::to_string(i)}, {"name", "Area " + std::to_string(i)}});
    }
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "oversized_peer", 5000);
  auto result = client.fetch_entities();

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("1001") != std::string::npos);
  EXPECT_TRUE(result.error().find("areas") != std::string::npos);
  EXPECT_TRUE(result.error().find("1000") != std::string::npos);

  svr.stop();
  t.join();
}

// Asset-identity survives a full peer round-trip: a Component with an identity
// nameplate is serialized via Component::to_json (as a peer would emit it under
// x-medkit.identity), parsed back by PeerClient::fetch_entities, then merged into a
// local entity by EntityMerger. The remote nameplate must be preserved end-to-end.
// Refs #482
TEST(PeerClientHappyPath, asset_identity_survives_fetch_and_merge) {
  // Build the remote Component with a populated identity and serialize it exactly the
  // way the gateway emits component detail responses.
  Component remote;
  remote.id = "plc_1";
  remote.name = "Line PLC";
  remote.identity.manufacturer = "Siemens AG";
  remote.identity.serial_number = "SN-42";
  remote.identity.firmware_version = "2.9.4";
  remote.identity.extra["slot"] = "3";
  remote.identity.provenance["serial_number"] = "opcua";
  const std::string remote_detail = remote.to_json().dump();

  httplib::Server svr;
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [&](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"plc_1","name":"Line PLC"}]})", "application/json");
  });
  svr.Get("/api/v1/components/plc_1", [&](const httplib::Request &, httplib::Response & res) {
    res.set_content(remote_detail, "application/json");
  });
  svr.Get("/api/v1/components/plc_1/subcomponents", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "peer_plc", 5000);
  auto result = client.fetch_entities();
  svr.stop();
  t.join();

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->components.size(), 1u);
  const auto & parsed = result->components[0].identity;

  // parse_component must have read x-medkit.identity back into the model.
  EXPECT_EQ(parsed.manufacturer, "Siemens AG");
  EXPECT_EQ(parsed.serial_number, "SN-42");
  EXPECT_EQ(parsed.firmware_version, "2.9.4");
  EXPECT_EQ(parsed.extra.at("slot"), "3");
  EXPECT_EQ(parsed.provenance.at("serial_number"), "opcua");

  // Collision merge: a local component with the same id but no identity must inherit
  // the remote nameplate (gap-fill), and a remote-only component carries it directly.
  Component local;
  local.id = "plc_1";
  local.name = "Line PLC";
  local.source = "manifest";

  EntityMerger merger("peer_plc");
  auto merged = merger.merge_components({local}, result->components);
  ASSERT_EQ(merged.size(), 1u);
  const auto & merged_id = merged[0].identity;
  EXPECT_EQ(merged_id.manufacturer, "Siemens AG");
  EXPECT_EQ(merged_id.serial_number, "SN-42");
  EXPECT_EQ(merged_id.firmware_version, "2.9.4");
  EXPECT_EQ(merged_id.extra.at("slot"), "3");
}

// =============================================================================
// Availability read-back
//
// `x-medkit.available` is emitted only when false, so absence is the peer
// saying the entity is reachable. Both directions are pinned here because the
// two failures are opposite and equally silent: dropping the read reports every
// unreachable entity of every peer as reachable, and defaulting to false
// reports every healthy one as unreachable.
// =============================================================================

namespace {

/// Serve the four collection roots a fetch always reads, so a test only has to
/// describe the routes it is actually about. httplib answers with the first
/// handler that matches, so this is installed AFTER the routes a test defines
/// itself and only fills in the ones it left out.
void install_empty_roots(httplib::Server & svr) {
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
}

/// A SOVD error body carrying the ``not-responding`` code, which is what a
/// gateway answers on every route of an entity whose contributor went quiet.
std::string not_responding_body(const std::string & member_id) {
  return nlohmann::json(
             {{"error_code", ERR_NOT_RESPONDING}, {"message", "Member '" + member_id + "' is not available"}})
      .dump();
}

/// Listen on a random port for the life of the scope. A failed ASSERT returns
/// from the test body, so a hand-written stop/join at the end of it never runs
/// and the listening thread outlives its server.
class ScopedServer {
 public:
  explicit ScopedServer(httplib::Server & svr) : svr_(svr) {
    port_ = svr_.bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this]() {
      svr_.listen_after_bind();
    });
  }

  ~ScopedServer() {
    svr_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ScopedServer(const ScopedServer &) = delete;
  ScopedServer & operator=(const ScopedServer &) = delete;
  ScopedServer(ScopedServer &&) = delete;
  ScopedServer & operator=(ScopedServer &&) = delete;

  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

 private:
  httplib::Server & svr_;
  std::thread thread_;
  int port_{0};
};

}  // namespace

TEST(PeerClientAvailability, a_peers_statement_that_an_entity_is_unreachable_is_carried) {
  httplib::Server svr;

  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu-live","name":"ecu-live"},{"id":"ecu-quiet","name":"ecu-quiet"}]})",
                    "application/json");
  });
  // A 200 detail that names the entity unreachable. This is the only account of
  // the entity the aggregator gets, so the flag has to come off the body.
  svr.Get("/api/v1/components/ecu-quiet", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu-quiet","name":"ecu-quiet","x-medkit":{"source":"manifest","available":false}})",
                    "application/json");
  });
  svr.Get("/api/v1/components/ecu-live", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu-live","name":"ecu-live","x-medkit":{"source":"manifest"}})", "application/json");
  });
  svr.Get(R"(/api/v1/components/([^/]+)/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(
        R"({"items":[
          {"id":"app-live","name":"app-live","x-medkit":{"source":"manifest","is_online":true}},
          {"id":"app-quiet","name":"app-quiet","x-medkit":{"source":"manifest","available":false}}
        ]})",
        "application/json");
  });
  svr.Get(R"(/api/v1/apps/([^/]+)/operations)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  install_empty_roots(svr);

  ScopedServer running(svr);
  PeerClient client(running.url(), "peer_b", 5000);
  auto result = client.fetch_entities();
  ASSERT_TRUE(result.has_value()) << result.error();

  ASSERT_EQ(result->components.size(), 2u);
  EXPECT_EQ(result->components[0].id, "ecu-live");
  EXPECT_TRUE(result->components[0].available) << "an entity the peer said nothing about was marked unreachable";
  EXPECT_EQ(result->components[1].id, "ecu-quiet");
  EXPECT_FALSE(result->components[1].available) << "the peer's statement that ecu-quiet is unreachable was dropped";

  ASSERT_EQ(result->apps.size(), 2u);
  EXPECT_EQ(result->apps[0].id, "app-live");
  EXPECT_TRUE(result->apps[0].available) << "an app the peer said nothing about was marked unreachable";
  EXPECT_EQ(result->apps[1].id, "app-quiet");
  EXPECT_FALSE(result->apps[1].available) << "the peer's statement that app-quiet is unreachable was dropped";
}

TEST(PeerClientAvailability, a_nested_collection_that_says_not_responding_does_not_abort_the_fetch) {
  httplib::Server svr;

  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu-quiet","name":"ecu-quiet"}]})", "application/json");
  });
  svr.Get("/api/v1/components/ecu-quiet", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu-quiet","name":"ecu-quiet","x-medkit":{"source":"manifest","available":false}})",
                    "application/json");
  });
  // Every route of a retained member answers 504 not-responding, the nested
  // collections included. Read as a failed request it would discard the peer's
  // whole picture, including the entities that are perfectly reachable.
  svr.Get("/api/v1/components/ecu-quiet/subcomponents", [](const httplib::Request &, httplib::Response & res) {
    res.status = 504;
    res.set_content(not_responding_body("ecu-quiet"), "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"app-quiet","name":"app-quiet","x-medkit":{"available":false}}]})",
                    "application/json");
  });
  svr.Get("/api/v1/apps/app-quiet/operations", [](const httplib::Request &, httplib::Response & res) {
    res.status = 504;
    res.set_content(not_responding_body("app-quiet"), "application/json");
  });
  install_empty_roots(svr);

  ScopedServer running(svr);
  PeerClient client(running.url(), "peer_b", 5000);
  auto result = client.fetch_entities();
  ASSERT_TRUE(result.has_value()) << "one unreachable member discarded the peer's whole picture: " << result.error();

  ASSERT_EQ(result->components.size(), 1u);
  EXPECT_FALSE(result->components[0].available);
  ASSERT_EQ(result->apps.size(), 1u);
  EXPECT_FALSE(result->apps[0].available);
}

TEST(PeerClientAvailability, a_504_that_is_not_a_statement_about_an_entity_still_fails_the_fetch) {
  httplib::Server svr;

  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu-a","name":"ecu-a"}]})", "application/json");
  });
  svr.Get("/api/v1/components/ecu-a", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"id":"ecu-a","name":"ecu-a"})", "application/json");
  });
  // 504 without the not-responding code is an upstream proxy timing out, which
  // says nothing about the entity and leaves a hole in the picture.
  svr.Get("/api/v1/components/ecu-a/subcomponents", [](const httplib::Request &, httplib::Response & res) {
    res.status = 504;
    res.set_content(R"({"error_code":"vendor-error","message":"upstream timed out"})", "application/json");
  });
  install_empty_roots(svr);

  ScopedServer running(svr);
  PeerClient client(running.url(), "peer_b", 5000);
  auto result = client.fetch_entities();
  EXPECT_FALSE(result.has_value()) << "a gateway timeout was read as a statement that an entity is unreachable";
}
