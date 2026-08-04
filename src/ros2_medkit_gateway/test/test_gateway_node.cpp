// Copyright 2025 bburda
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
#include <httplib.h>  // NOLINT(build/include_order)

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using namespace std::chrono_literals;

// API version prefix - must match rest_server.cpp
static constexpr const char * API_BASE_PATH = "/api/v1";

/// Reserve a free TCP port by binding to port 0, reading the assigned port, then closing the socket.
/// The port is briefly available for the next caller; works reliably on Linux with SO_REUSEADDR.
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

class TestGatewayNode : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Reserve a free port for this test to avoid cross-test port collisions
    int free_port = reserve_free_port();
    ASSERT_NE(free_port, 0) << "Failed to reserve a free port for test";

    create_node_with_overrides({rclcpp::Parameter("server.port", free_port)});
  }

  void create_node_with_overrides(const std::vector<rclcpp::Parameter> & overrides) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

    // Get server configuration from node parameters
    server_host_ = node_->get_parameter("server.host").as_string();
    server_port_ = static_cast<int>(node_->get_parameter("server.port").as_int());

    // Wait for the server to be ready
    wait_for_server_ready();
  }

  void TearDown() override {
    for (const auto & path : temp_files_) {
      std::remove(path.c_str());
    }
    node_.reset();
  }

  void wait_for_server_ready() {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    httplib::Client client(server_host_, server_port_);
    const std::string health_endpoint = std::string(API_BASE_PATH) + "/health";

    while (std::chrono::steady_clock::now() - start < timeout) {
      if (auto res = client.Get((health_endpoint).c_str())) {
        if (res->status == 200) {
          return;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  httplib::Client create_client() {
    return httplib::Client(server_host_, server_port_);
  }

  bool wait_for_subscriber_count(const std::string & topic_name, size_t expected_count,
                                 std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (node_->count_subscribers(topic_name) == expected_count) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return node_->count_subscribers(topic_name) == expected_count;
  }

  std::string write_temp_manifest(const std::string & contents) {
    char path_template[] = "/tmp/ros2_medkit_gateway_manifest_XXXXXX";
    int fd = mkstemp(path_template);
    EXPECT_GE(fd, 0);
    if (fd >= 0) {
      close(fd);
    }

    std::ofstream out(path_template);
    out << contents;
    out.close();
    temp_files_.emplace_back(path_template);
    return path_template;
  }

  void load_function_fixture(const std::string & function_id) {
    const std::string manifest =
        "manifest_version: \"1.0\"\n"
        "functions:\n"
        "  - id: \"" +
        function_id +
        "\"\n"
        "    name: \"Function Test\"\n";

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = write_temp_manifest(manifest);
    config.manifest_strict_validation = false;

    ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

    ros2_medkit_gateway::Function func;
    func.id = function_id;
    func.name = "Function Test";
    auto & cache = const_cast<ros2_medkit_gateway::ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_functions({func});
  }

  void load_app_location_fixture() {
    const std::string manifest = R"(
manifest_version: "1.0"
areas:
  - id: "vehicle"
    name: "Vehicle"
components:
  - id: "main_ecu"
    name: "Main ECU"
    area: "vehicle"
    namespace: "/vehicle"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "main_ecu"
  - id: "standalone"
    name: "Standalone"
)";

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = write_temp_manifest(manifest);
    config.manifest_strict_validation = false;

    ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

    auto areas = node_->get_discovery_manager()->discover_areas();
    auto components = node_->get_discovery_manager()->discover_components();
    auto apps = node_->get_discovery_manager()->discover_apps();
    auto functions = node_->get_discovery_manager()->discover_functions();

    auto & cache = const_cast<ros2_medkit_gateway::ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);
  }

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  std::string server_host_;
  int server_port_;
  std::vector<std::string> temp_files_;
};

TEST_F(TestGatewayNode, test_health_endpoint) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/health").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["status"], "healthy");
  EXPECT_TRUE(json_response.contains("timestamp"));
  EXPECT_TRUE(json_response["timestamp"].is_number());
}

TEST_F(TestGatewayNode, test_root_endpoint) {
  // @verifies REQ_INTEROP_010
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["name"], "ROS 2 Medkit Gateway");
  EXPECT_TRUE(json_response.contains("version"));
  EXPECT_TRUE(json_response.contains("endpoints"));
  EXPECT_TRUE(json_response["endpoints"].is_array());
  EXPECT_TRUE(json_response.contains("capabilities"));
  EXPECT_TRUE(json_response["capabilities"]["discovery"]);
  EXPECT_TRUE(json_response["capabilities"]["data_access"]);
}

TEST_F(TestGatewayNode, test_fault_manager_namespace_configures_event_subscribers) {
  node_.reset();

  int free_port = reserve_free_port();
  ASSERT_NE(free_port, 0) << "Failed to reserve a free port for test";

  create_node_with_overrides({
      rclcpp::Parameter("server.port", free_port),
      rclcpp::Parameter("fault_manager.namespace", "robot5"),
  });

  // Verify the fault manager namespace is correctly resolved through the
  // deterministic path-building functions (no DDS graph discovery dependency).
  auto base_path = ros2_medkit_gateway::build_fault_manager_base_path(node_.get());
  auto events_topic = ros2_medkit_gateway::build_fault_manager_events_topic(node_.get());
  EXPECT_EQ(base_path, "/robot5/fault_manager");
  EXPECT_EQ(events_topic, "/robot5/fault_manager/events");

  // Verify the REST API is functional with the namespaced fault manager.
  // The health endpoint works regardless of fault manager namespace.
  auto client = create_client();
  auto res = client.Get((std::string(API_BASE_PATH) + "/health").c_str());
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
}

// `service_call_timeout_sec` is the whole cancel budget since the 15s floor
// was removed (issue #576), yet it was declared with no bounds at all: 0 and
// negatives were accepted silently (a cancel wait of 0 ms) and an absurd
// value would pin an HTTP worker for as long as it says. Sweep the documented
// range's endpoints and both degenerate directions, and observe the budget
// the gateway actually applies rather than the value that was configured.
TEST_F(TestGatewayNode, test_service_call_timeout_sec_is_clamped_to_the_documented_range) {
  const std::pair<int64_t, int> cases[] = {
      {0, 1},        // degenerate: no budget at all -> floor
      {-5, 1},       // negative -> floor
      {1, 1},        // range floor, accepted as-is
      {10, 10},      // default, accepted as-is
      {3600, 3600},  // range ceiling, accepted as-is
      {100000, 3600}
      // above the ceiling -> ceiling
  };

  for (const auto & [configured, expected] : cases) {
    SCOPED_TRACE("service_call_timeout_sec=" + std::to_string(configured));
    node_.reset();

    int free_port = reserve_free_port();
    ASSERT_NE(free_port, 0) << "Failed to reserve a free port for test";
    create_node_with_overrides({
        rclcpp::Parameter("server.port", free_port),
        rclcpp::Parameter("service_call_timeout_sec", configured),
    });

    auto * operation_mgr = node_->get_operation_manager();
    ASSERT_NE(operation_mgr, nullptr);
    EXPECT_EQ(operation_mgr->service_call_timeout_sec(), expected);
  }
}

TEST_F(TestGatewayNode, test_version_info_endpoint) {
  // @verifies REQ_INTEROP_001
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/version-info").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  // Check for items array (SOVD-standard wrapper key)
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
  EXPECT_GE(json_response["items"].size(), 1);

  // Check first items entry
  const auto & info = json_response["items"][0];
  EXPECT_TRUE(info.contains("version"));
  EXPECT_TRUE(info.contains("base_uri"));
  EXPECT_TRUE(info.contains("vendor_info"));
  EXPECT_TRUE(info["vendor_info"].contains("version"));
  EXPECT_TRUE(info["vendor_info"].contains("name"));
}

TEST_F(TestGatewayNode, test_list_areas_endpoint) {
  // @verifies REQ_INTEROP_003
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

TEST_F(TestGatewayNode, test_list_components_endpoint) {
  // @verifies REQ_INTEROP_003
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

TEST_F(TestGatewayNode, test_nonexistent_endpoint_404) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/nonexistent/path").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_invalid_area_id_bad_request) {
  auto client = create_client();

  // Test with special characters that should be rejected
  // Note: URL-encoded slash (%2F) is decoded by server, but @#$ characters are invalid
  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/test@invalid").c_str());

  ASSERT_TRUE(res);
  // Should return 400 for invalid entity ID with special characters
  EXPECT_EQ(res->status, 400);
}

TEST_F(TestGatewayNode, test_nonexistent_area_404) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/nonexistent_area").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_nonexistent_component_404) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_component").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_invalid_component_id_bad_request) {
  auto client = create_client();

  // Test with special characters that are invalid
  auto res = client.Get((std::string(API_BASE_PATH) + "/components/test@invalid#id").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error_code"));
  EXPECT_TRUE(json_response.contains("message"));
}

TEST_F(TestGatewayNode, test_list_apps_endpoint) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/apps").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

TEST_F(TestGatewayNode, test_list_functions_endpoint) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/functions").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
}

TEST_F(TestGatewayNode, test_options_request_for_cors) {
  auto client = create_client();

  // OPTIONS request for CORS preflight
  auto res = client.Options((std::string(API_BASE_PATH) + "/components").c_str());

  ASSERT_TRUE(res);
  // Without CORS configured, server may return various success codes or 404
  // The main purpose is to verify the server handles OPTIONS without crashing
  EXPECT_TRUE(res->status == 200 || res->status == 204 || res->status == 404 || res->status == 405);
}

TEST_F(TestGatewayNode, test_component_data_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_component/data").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_operations_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_component/operations").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_configurations_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_component/configurations").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_faults_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_component/faults").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

// Note: test_global_faults_endpoint is skipped in unit tests because it requires
// the fault_manager external service which isn't available. This endpoint is
// properly tested in test_integration.test.py which starts all required nodes.

TEST_F(TestGatewayNode, test_root_endpoint_contains_api_base_path) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/").c_str());

  ASSERT_TRUE(res);
  auto json_response = nlohmann::json::parse(res->body);

  EXPECT_TRUE(json_response.contains("api_base"));
  EXPECT_EQ(json_response["api_base"], API_BASE_PATH);
}

TEST_F(TestGatewayNode, test_root_endpoint_lists_all_capabilities) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/").c_str());

  ASSERT_TRUE(res);
  auto json_response = nlohmann::json::parse(res->body);

  EXPECT_TRUE(json_response["capabilities"]["discovery"]);
  EXPECT_TRUE(json_response["capabilities"]["data_access"]);
  EXPECT_TRUE(json_response["capabilities"]["operations"]);
  EXPECT_TRUE(json_response["capabilities"]["async_actions"]);
  EXPECT_TRUE(json_response["capabilities"]["configurations"]);
  EXPECT_TRUE(json_response["capabilities"]["faults"]);
}

TEST_F(TestGatewayNode, test_area_components_nonexistent_area) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/nonexistent_area/components").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_area_subareas_nonexistent_area) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/nonexistent_area/subareas").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_post_to_nonexistent_operation) {
  auto client = create_client();

  auto res = client.Post((std::string(API_BASE_PATH) + "/components/nonexistent/operations/do_something").c_str(), "",
                         "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

// =============================================================================
// Configuration endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_set_configuration_invalid_json) {
  auto client = create_client();

  // POST with invalid JSON body
  auto res = client.Put((std::string(API_BASE_PATH) + "/components/gateway_node/configurations/test_param").c_str(),
                        "{ invalid json }", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error_code"));
  EXPECT_TRUE(json_response.contains("message"));
}

TEST_F(TestGatewayNode, test_set_configuration_missing_value_field) {
  auto client = create_client();

  // POST with valid JSON but missing 'data' field
  auto res = client.Put((std::string(API_BASE_PATH) + "/components/gateway_node/configurations/test_param").c_str(),
                        R"({"name": "test_param"})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error_code"));
  EXPECT_TRUE(json_response.contains("message"));
  // Format expects 'data' field, error should mention it
  EXPECT_TRUE(json_response["message"].get<std::string>().find("data") != std::string::npos ||
              (json_response.contains("x-medkit") && json_response["x-medkit"].contains("details")));
}

TEST_F(TestGatewayNode, test_set_configuration_invalid_component_id) {
  auto client = create_client();

  auto res = client.Put((std::string(API_BASE_PATH) + "/components/invalid@id/configurations/param").c_str(),
                        R"({"value": 42})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

TEST_F(TestGatewayNode, test_get_configuration_nonexistent_component) {
  auto client = create_client();

  auto res =
      client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_comp/configurations/some_param").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_delete_configuration_nonexistent_component) {
  auto client = create_client();

  auto res =
      client.Delete((std::string(API_BASE_PATH) + "/components/nonexistent_comp/configurations/some_param").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

// =============================================================================
// Operation endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_post_operation_invalid_json_body) {
  auto client = create_client();

  // First we need a valid component - use the gateway_node itself
  auto res = client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/operations/some_service").c_str(),
                         "{ not valid json }", "application/json");

  ASSERT_TRUE(res);
  // Should be 400 for invalid JSON or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

TEST_F(TestGatewayNode, test_post_operation_invalid_component_id) {
  auto client = create_client();

  auto res =
      client.Post((std::string(API_BASE_PATH) + "/components/invalid@component/operations/test/executions").c_str(),
                  R"({})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

TEST_F(TestGatewayNode, test_post_operation_invalid_operation_id) {
  auto client = create_client();

  auto res =
      client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/operations/invalid@op/executions").c_str(),
                  R"({})", "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid operation name or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

TEST_F(TestGatewayNode, test_execution_status_invalid_component_id) {
  auto client = create_client();

  auto res =
      client.Get((std::string(API_BASE_PATH) + "/components/invalid@id/operations/test/executions/some-id").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

TEST_F(TestGatewayNode, test_execution_cancel_invalid_component_id) {
  auto client = create_client();

  // Cancel with invalid component ID
  auto res =
      client.Delete((std::string(API_BASE_PATH) + "/components/invalid@id/operations/test/executions/some-id").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

TEST_F(TestGatewayNode, test_execution_not_found) {
  auto client = create_client();

  // Get execution status for non-existent execution
  const std::string path =
      std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/executions/nonexistent-id";
  auto res = client.Get((path).c_str());

  ASSERT_TRUE(res);
  // Should be 404 for not found execution
  EXPECT_EQ(res->status, 404);
}

// @verifies REQ_INTEROP_038
TEST_F(TestGatewayNode, test_execution_update_invalid_component_id) {
  auto client = create_client();

  // PUT with invalid component ID
  auto res =
      client.Put((std::string(API_BASE_PATH) + "/components/invalid@id/operations/test/executions/some-id").c_str(),
                 R"({"capability": "stop"})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

// @verifies REQ_INTEROP_038
TEST_F(TestGatewayNode, test_execution_update_missing_capability) {
  auto client = create_client();

  // PUT without capability field
  auto res =
      client.Put((std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/executions/some-id").c_str(),
                 R"({"timeout": 60})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

// @verifies REQ_INTEROP_038
TEST_F(TestGatewayNode, test_execution_update_unsupported_capability) {
  auto client = create_client();

  // PUT with unsupported capability (freeze is I/O control specific)
  auto res =
      client.Put((std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/executions/some-id").c_str(),
                 R"({"capability": "freeze"})", "application/json");

  ASSERT_TRUE(res);
  // Either 400 for unsupported capability or 404 if execution not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

// @verifies REQ_INTEROP_038
TEST_F(TestGatewayNode, test_execution_update_execution_not_found) {
  auto client = create_client();

  // PUT with stop capability for non-existent execution
  auto res = client.Put(
      (std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/executions/nonexistent").c_str(),
      R"({"capability": "stop"})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

// =============================================================================
// Data endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_publish_to_topic_invalid_json) {
  auto client = create_client();

  auto res = client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic").c_str(),
                         "invalid{json}", "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid JSON or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_missing_type_field) {
  auto client = create_client();

  // Missing 'type' field in request body
  auto res = client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic").c_str(),
                         R"({"data": {"value": 1.0}})", "application/json");

  ASSERT_TRUE(res);
  // 400 for missing type or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_missing_data_field) {
  auto client = create_client();

  // Missing 'data' field in request body
  auto res = client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic").c_str(),
                         R"({"type": "std_msgs/msg/Float32"})", "application/json");

  ASSERT_TRUE(res);
  // 400 for missing data or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_invalid_message_type_format) {
  auto client = create_client();

  // Invalid message type format (should be pkg/msg/Type)
  auto res = client.Post((std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic").c_str(),
                         R"({"type": "invalid_type", "data": {}})", "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid type format or 404 if component not found
  EXPECT_TRUE(res->status == 400 || res->status == 404);
}

// =============================================================================
// Subresource tests
// =============================================================================

TEST_F(TestGatewayNode, test_area_related_components_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/nonexistent_area/related-components").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_subcomponents_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_comp/subcomponents").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_related_apps_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_comp/related-apps").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_area_contains_nonexistent) {
  // @verifies REQ_INTEROP_006
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/areas/nonexistent_area/contains").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_component_hosts_nonexistent) {
  // @verifies REQ_INTEROP_007
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/components/nonexistent_comp/hosts").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_app_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/apps/nonexistent_app").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_function_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/functions/nonexistent_function").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

TEST_F(TestGatewayNode, test_function_detail_includes_cyclic_subscriptions_capability) {
  load_function_fixture("graph_func");
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/functions/graph_func").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = nlohmann::json::parse(res->body);
  ASSERT_TRUE(body.contains("cyclic-subscriptions"));
  EXPECT_EQ(body["cyclic-subscriptions"], "/api/v1/functions/graph_func/cyclic-subscriptions");

  ASSERT_TRUE(body.contains("capabilities"));
  const auto & caps = body["capabilities"];
  auto has_cyclic_subscriptions = std::any_of(caps.begin(), caps.end(), [](const auto & cap) {
    return cap.contains("name") && cap["name"] == "cyclic-subscriptions";
  });
  EXPECT_TRUE(has_cyclic_subscriptions);
}

TEST_F(TestGatewayNode, test_function_hosts_nonexistent) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/functions/nonexistent_function/hosts").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 404);
}

// =============================================================================
// Invalid ID pattern tests for various entity types
// =============================================================================

TEST_F(TestGatewayNode, test_invalid_app_id_bad_request) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/apps/invalid@app#id").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

// @verifies REQ_INTEROP_105
TEST_F(TestGatewayNode, test_app_is_located_on_endpoint) {
  load_app_location_fixture();

  auto client = create_client();
  auto res = client.Get((std::string(API_BASE_PATH) + "/apps/planner/is-located-on").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto json_response = nlohmann::json::parse(res->body);
  ASSERT_TRUE(json_response.contains("items"));
  ASSERT_EQ(json_response["items"].size(), 1);
  EXPECT_EQ(json_response["items"][0]["id"], "main_ecu");
  EXPECT_EQ(json_response["_links"]["self"], "/api/v1/apps/planner/is-located-on");
  EXPECT_EQ(json_response["_links"]["app"], "/api/v1/apps/planner");
}

TEST_F(TestGatewayNode, test_invalid_function_id_bad_request) {
  auto client = create_client();

  auto res = client.Get((std::string(API_BASE_PATH) + "/functions/invalid@func#id").c_str());

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 400);
}

// =============================================================================
// Secure-profile fail-closed tests (issue #477): when auth/TLS is explicitly
// enabled but its configuration is invalid, the node must refuse to start
// rather than silently fall back to an unauthenticated / plaintext server.
// =============================================================================

TEST_F(TestGatewayNode, auth_enabled_with_empty_secret_fails_closed) {
  // auth.enabled=true with an empty HS256 secret: the AuthConfigBuilder rejects
  // it, and the node must propagate that as a fatal (throw) instead of disabling
  // auth and serving unauthenticated.
  node_.reset();
  int free_port = reserve_free_port();
  ASSERT_NE(free_port, 0);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("server.port", free_port),
      rclcpp::Parameter("auth.enabled", true),
      rclcpp::Parameter("auth.jwt_secret", std::string("")),
      rclcpp::Parameter("auth.jwt_algorithm", std::string("HS256")),
  });
  EXPECT_THROW(std::make_shared<ros2_medkit_gateway::GatewayNode>(options), std::exception);
}

TEST_F(TestGatewayNode, tls_enabled_with_missing_cert_fails_closed) {
  // server.tls.enabled=true with no cert/key: the TlsConfigBuilder rejects it,
  // and the node must refuse to start in plaintext.
  node_.reset();
  int free_port = reserve_free_port();
  ASSERT_NE(free_port, 0);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("server.port", free_port),
      rclcpp::Parameter("server.tls.enabled", true),
      rclcpp::Parameter("server.tls.cert_file", std::string("")),
      rclcpp::Parameter("server.tls.key_file", std::string("")),
  });
  EXPECT_THROW(std::make_shared<ros2_medkit_gateway::GatewayNode>(options), std::exception);
}

// =============================================================================
// Entity type path extraction tests
// =============================================================================

TEST(ExtractEntityTypePath, components_route) {
  using ros2_medkit_gateway::extract_entity_type_from_path;
  using ros2_medkit_gateway::SovdEntityType;

  EXPECT_EQ(extract_entity_type_from_path("/api/v1/components"), SovdEntityType::COMPONENT);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/components/my_comp"), SovdEntityType::COMPONENT);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/components/my_comp/data"), SovdEntityType::COMPONENT);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/components/test/operations"), SovdEntityType::COMPONENT);
}

TEST(ExtractEntityTypePath, apps_route) {
  using ros2_medkit_gateway::extract_entity_type_from_path;
  using ros2_medkit_gateway::SovdEntityType;

  EXPECT_EQ(extract_entity_type_from_path("/api/v1/apps"), SovdEntityType::APP);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/apps/my_app"), SovdEntityType::APP);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/apps/my_app/data"), SovdEntityType::APP);
}

TEST(ExtractEntityTypePath, areas_route) {
  using ros2_medkit_gateway::extract_entity_type_from_path;
  using ros2_medkit_gateway::SovdEntityType;

  EXPECT_EQ(extract_entity_type_from_path("/api/v1/areas"), SovdEntityType::AREA);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/areas/root"), SovdEntityType::AREA);
}

TEST(ExtractEntityTypePath, functions_route) {
  using ros2_medkit_gateway::extract_entity_type_from_path;
  using ros2_medkit_gateway::SovdEntityType;

  EXPECT_EQ(extract_entity_type_from_path("/api/v1/functions"), SovdEntityType::FUNCTION);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/functions/my_func"), SovdEntityType::FUNCTION);
}

TEST(ExtractEntityTypePath, unknown_routes) {
  using ros2_medkit_gateway::extract_entity_type_from_path;
  using ros2_medkit_gateway::SovdEntityType;

  EXPECT_EQ(extract_entity_type_from_path("/api/v1/health"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/faults"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/version-info"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/other/path"), SovdEntityType::UNKNOWN);

  // Verify paths that look similar to entity collections but aren't
  // (segment boundary validation)
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/componentship"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/applications"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/areascan"), SovdEntityType::UNKNOWN);
  EXPECT_EQ(extract_entity_type_from_path("/api/v1/functional"), SovdEntityType::UNKNOWN);
}

// Startup-summary helper logic (no ROS graph needed - pure functions).

TEST(GatewayStartupSummary, CountPeerNodesExcludesOwnAndHidden) {
  const std::vector<std::pair<std::string, std::string>> nodes = {
      {"ros2_medkit_gateway", "/"},                // the gateway's own node (self)
      {"ros2_medkit_gateway_sub", "/"},            // own internal helper (FQN prefix)
      {"ros2_medkit_gateway_fault_clients", "/"},  // own internal helper (FQN prefix)
      {"_hidden_node", "/"},                       // hidden node
      {"camera", "/sensors"},                      // peer (hidden node name check is on the name part)
      {"robot_planner", "/"},                      // peer
      {"ros2_medkit_gateway_monitor", "/"},        // peer that merely shares the gateway name prefix
  };
  // Three genuine peers: /sensors/camera, /robot_planner and the prefix-sharing
  // /ros2_medkit_gateway_monitor (only the exact self FQN and known helpers drop out).
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::count_peer_nodes(nodes, "/ros2_medkit_gateway"), 3u);
}

TEST(GatewayStartupSummary, CountPeerNodesZeroWhenOnlyOwnNodes) {
  const std::vector<std::pair<std::string, std::string>> nodes = {
      {"ros2_medkit_gateway", "/"},
      {"ros2_medkit_gateway_sub", "/"},
      {"ros2_medkit_gateway_fault_clients", "/"},
  };
  // Zero peers is the condition that triggers the empty-graph warning.
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::count_peer_nodes(nodes, "/ros2_medkit_gateway"), 0u);
}

TEST(GatewayStartupSummary, ConnectableHostTranslatesBindAll) {
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("0.0.0.0"), "127.0.0.1");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host(""), "127.0.0.1");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("192.168.1.5"), "192.168.1.5");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("127.0.0.1"), "127.0.0.1");
  // IPv6 bind addresses must be bracketed so host:port is a valid URL authority.
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("::"), "[::1]");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("[::]"), "[::1]");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("2001:db8::1"), "[2001:db8::1]");
  EXPECT_EQ(ros2_medkit_gateway::GatewayNode::connectable_host("[2001:db8::1]"), "[2001:db8::1]");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
