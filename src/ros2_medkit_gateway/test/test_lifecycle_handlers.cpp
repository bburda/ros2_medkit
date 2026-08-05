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
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <regex>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lifecycle_handlers.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/lifecycle_provider.hpp"
#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::LifecycleProvider;
using ros2_medkit_gateway::LifecycleProviderError;
using ros2_medkit_gateway::LifecycleProviderErrorInfo;
using ros2_medkit_gateway::PluginManager;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::JsonWriter;
using ros2_medkit_gateway::handlers::DiscoveryHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LifecycleHandlers;
namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

namespace {

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

class StubLifecycleStateReader : public ros2_medkit_gateway::LifecycleStateReader {
 public:
  std::optional<std::string> next;
  int calls = 0;
  std::optional<std::string> get_state(const std::string & /*get_state_service_path*/) override {
    ++calls;
    return next;
  }
};

}  // namespace

// =============================================================================
// Fixture-based tests against a live GatewayNode.
// =============================================================================

class LifecycleHandlersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::vector<std::string> args = {"test_lifecycle_handlers", "--ros-args", "-p", "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<LifecycleHandlers>(*ctx_, nullptr);  // no plugin manager

    // Give the executor a moment to start before seeding cache.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    seed_cache();
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    handlers_.reset();
    ctx_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  void seed_cache() {
    // Online app
    App online_app;
    online_app.id = "online_sensor";
    online_app.name = "OnlineSensor";
    online_app.is_online = true;

    // Offline app
    App offline_app;
    offline_app.id = "offline_sensor";
    offline_app.name = "OfflineSensor";
    offline_app.is_online = false;

    // Component with host_metadata (host component -> "ready")
    Component host_comp;
    host_comp.id = "host_controller";
    host_comp.name = "HostController";
    host_comp.host_metadata = json{{"hostname", "robot-main"}};

    // Component without host_metadata and without hosted apps (non-host -> "notReady")
    Component non_host_comp;
    non_host_comp.id = "bridge_component";
    non_host_comp.name = "BridgeComponent";
    // host_metadata left as nullopt

    // Non-host component with one online hosted app -> liveness "ready".
    Component live_comp;
    live_comp.id = "live_subsystem";
    live_comp.name = "LiveSubsystem";
    App hosted_online_app;
    hosted_online_app.id = "hosted_online_node";
    hosted_online_app.name = "HostedOnlineNode";
    hosted_online_app.is_online = true;
    hosted_online_app.component_id = "live_subsystem";

    // Non-host component whose only hosted app is offline -> "notReady".
    Component dead_comp;
    dead_comp.id = "dead_subsystem";
    dead_comp.name = "DeadSubsystem";
    App hosted_offline_app;
    hosted_offline_app.id = "hosted_offline_node";
    hosted_offline_app.name = "HostedOfflineNode";
    hosted_offline_app.is_online = false;
    hosted_offline_app.component_id = "dead_subsystem";

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {host_comp, non_host_comp, live_comp, dead_comp},
                     {online_app, offline_app, hosted_online_app, hosted_offline_app}, {});
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<LifecycleHandlers> handlers_;
};

// GET /apps/{online_sensor}/status -> 200, status == "ready", no restart URI
TEST_F(LifecycleHandlersTest, GetStatusOnlineAppReturnsReady) {
  auto raw = make_request_with_match("/api/v1/apps/online_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
  EXPECT_FALSE(result->restart.has_value());
}

// GET /apps/{offline_sensor}/status -> 200, status == "notReady"
TEST_F(LifecycleHandlersTest, GetStatusOfflineAppReturnsNotReady) {
  auto raw = make_request_with_match("/api/v1/apps/offline_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");
}

// GET /components/{host_controller}/status -> 200, status == "ready" (reachable local component)
TEST_F(LifecycleHandlersTest, GetStatusHostComponentReturnsReady) {
  auto raw =
      make_request_with_match("/api/v1/components/host_controller/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
}

// GET /components/{bridge_component}/status -> 200, status == "ready" (reachable local component, zero apps)
TEST_F(LifecycleHandlersTest, GetStatusComponentZeroAppsReturnsReady) {
  auto raw =
      make_request_with_match("/api/v1/components/bridge_component/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");  // reachable local component, zero apps -> ready
}

// GET /components/{live_subsystem}/status -> "ready" (reachable local component; one hosted App online)
TEST_F(LifecycleHandlersTest, GetStatusComponentWithOnlineHostedAppReturnsReady) {
  auto raw =
      make_request_with_match("/api/v1/components/live_subsystem/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
}

// GET /components/{dead_subsystem}/status -> "notReady" (hosts apps, all offline -> subsystem down)
TEST_F(LifecycleHandlersTest, GetStatusComponentAllAppsOfflineReturnsNotReady) {
  auto raw =
      make_request_with_match("/api/v1/components/dead_subsystem/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");  // hosts apps and every one is offline -> the subsystem is down
}

// PUT /apps/{online_sensor}/status/restart -> 501 (no provider registered)
TEST_F(LifecycleHandlersTest, TransitionRestartWithNoProviderReturns501) {
  // Route regex: the action is captured as param "1" = entity_id from /apps/{id}/status/restart
  // We register a route with {app_id} as capture group 1. In the handler, path_param("1") gives entity_id.
  auto raw =
      make_request_with_match("/api/v1/apps/online_sensor/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// PUT /apps/{online_sensor}/status/force-shutdown -> 501 (no provider registered)
TEST_F(LifecycleHandlersTest, TransitionForceShutdownWithNoProviderReturns501) {
  auto raw = make_request_with_match("/api/v1/apps/online_sensor/status/force-shutdown",
                                     R"(/api/v1/apps/([^/]+)/status/force-shutdown)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "force-shutdown");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// PUT /apps/{unknown}/status/restart -> 404 (validate_entity_for_route before provider)
TEST_F(LifecycleHandlersTest, TransitionUnknownEntityReturns404) {
  auto raw =
      make_request_with_match("/api/v1/apps/does_not_exist/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
}

// =============================================================================
// Entity detail status link advertisement tests.
// Verify GET /apps/{id} and GET /components/{id} include "status" URI field.
// =============================================================================

namespace {

template <class T>
json detail_body_json(const ros2_medkit_gateway::http::Result<T> & result) {
  if (!result.has_value()) {
    ADD_FAILURE() << "Expected result to have value but it was empty";
    return json{};
  }
  return JsonWriter<T>::write(result.value());
}

}  // namespace

class EntityDetailStatusLinkTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      std::vector<std::string> args = {"test_lifecycle_handlers", "--ros-args", "-p", "refresh_interval_ms:=60000"};
      std::vector<char *> argv;
      argv.reserve(args.size());
      for (auto & arg : args) {
        argv.push_back(arg.data());
      }
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
  }

  static void TearDownTestSuite() {
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    App app;
    app.id = "online_sensor";
    app.name = "OnlineSensor";
    app.is_online = true;

    Component comp;
    comp.id = "host_controller";
    comp.name = "HostController";

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {comp}, {app}, {});

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    discovery_ = std::make_unique<DiscoveryHandlers>(*ctx_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    discovery_.reset();
    ctx_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<DiscoveryHandlers> discovery_;
};

// GET /apps/{id} must include "status" -> "/api/v1/apps/{id}/status"
TEST_F(EntityDetailStatusLinkTest, AppDetailAdvertisesStatusLink) {
  httplib::Request req;
  req.path = "/api/v1/apps/online_sensor";
  std::regex re(R"(/api/v1/apps/([^/]+))");
  std::regex_match(req.path, req.matches, re);
  http::TypedRequest typed_req(req);

  auto result = discovery_->get_app(typed_req);
  auto body = detail_body_json(result);
  EXPECT_EQ(body["status"], "/api/v1/apps/online_sensor/status");
}

// GET /components/{id} must include "status" -> "/api/v1/components/{id}/status"
TEST_F(EntityDetailStatusLinkTest, ComponentDetailAdvertisesStatusLink) {
  httplib::Request req;
  req.path = "/api/v1/components/host_controller";
  std::regex re(R"(/api/v1/components/([^/]+))");
  std::regex_match(req.path, req.matches, re);
  http::TypedRequest typed_req(req);

  auto result = discovery_->get_component(typed_req);
  auto body = detail_body_json(result);
  EXPECT_EQ(body["status"], "/api/v1/components/host_controller/status");
}

// =============================================================================
// Provider-present handler path tests.
// Wire a mock LifecycleProvider into the handler via PluginManager and verify
// the handler correctly delegates, fills transition URIs, returns 202, and maps
// provider errors to the right HTTP codes.
// =============================================================================

namespace {

/// Configurable mock plugin implementing LifecycleProvider.
/// Set `status_to_return` for get_status(), or `error_to_return` for failures.
/// Set `transition_error` for request_transition() failures.
class MockLifecyclePlugin : public ros2_medkit_gateway::GatewayPlugin, public LifecycleProvider {
 public:
  std::string name() const override {
    return "mock_lifecycle";
  }
  void configure(const json & /*cfg*/) override {
  }
  void shutdown() override {
  }

  // LifecycleProvider

  tl::expected<dto::LifecycleStatusResponse, LifecycleProviderErrorInfo>
  get_status(const std::string & /*entity_id*/) override {
    if (get_status_error) {
      return tl::make_unexpected(*get_status_error);
    }
    return status_response;
  }

  tl::expected<std::monostate, LifecycleProviderErrorInfo>
  request_transition(const std::string & /*entity_id*/, std::string_view /*transition*/) override {
    if (transition_error) {
      return tl::make_unexpected(*transition_error);
    }
    return std::monostate{};
  }

  // Configurable state
  dto::LifecycleStatusResponse status_response;
  std::optional<LifecycleProviderErrorInfo> get_status_error;
  std::optional<LifecycleProviderErrorInfo> transition_error;
};

}  // namespace

class LifecycleHandlersWithProviderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      std::vector<std::string> args = {"test_lifecycle_handlers", "--ros-args", "-p", "refresh_interval_ms:=60000"};
      std::vector<char *> argv;
      argv.reserve(args.size());
      for (auto & arg : args) {
        argv.push_back(arg.data());
      }
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
  }

  static void TearDownTestSuite() {
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Seed cache with one online app.
    App app;
    app.id = "plugin_app";
    app.name = "PluginApp";
    app.is_online = true;
    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {}, {app}, {});

    // Wire plugin manager: add mock plugin and register entity ownership.
    plugin_mgr_ = std::make_unique<PluginManager>();
    auto mock = std::make_unique<MockLifecyclePlugin>();
    mock_ = mock.get();
    plugin_mgr_->add_plugin(std::move(mock));
    plugin_mgr_->register_entity_ownership("mock_lifecycle", {"plugin_app"});

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<LifecycleHandlers>(*ctx_, plugin_mgr_.get());
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    handlers_.reset();
    ctx_.reset();
    plugin_mgr_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<PluginManager> plugin_mgr_;
  MockLifecyclePlugin * mock_{nullptr};
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<LifecycleHandlers> handlers_;
};

// GET /apps/{plugin_app}/status with provider returning "ready" + restart transition:
// - status == "ready"
// - restart field filled with absolute URI "/api/v1/apps/plugin_app/status/restart"
TEST_F(LifecycleHandlersWithProviderTest, GetStatusProviderReadyFillsTransitionURI) {
  mock_->status_response.status = "ready";
  mock_->status_response.restart = "";  // non-nullopt signals provider supports it

  auto raw = make_request_with_match("/api/v1/apps/plugin_app/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
  ASSERT_TRUE(result->restart.has_value());
  EXPECT_EQ(*result->restart, "/api/v1/apps/plugin_app/status/restart");
  // start was not set by provider, so it should remain absent.
  EXPECT_FALSE(result->start.has_value());
}

// PUT /apps/{plugin_app}/status/restart with provider accepting -> 202 + Location header.
TEST_F(LifecycleHandlersWithProviderTest, TransitionAcceptedReturns202WithLocation) {
  auto raw =
      make_request_with_match("/api/v1/apps/plugin_app/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_TRUE(result.has_value());
  const auto & att = result->second;
  // 202 is declared by the Accepted<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result->first)>::value, 202);
  EXPECT_FALSE(att.status_override.has_value());
  bool found_location = false;
  for (const auto & [name, value] : att.headers) {
    if (name == "Location") {
      EXPECT_EQ(value, "/api/v1/apps/plugin_app/status");
      found_location = true;
    }
  }
  EXPECT_TRUE(found_location);
}

// PUT /apps/{plugin_app}/status/restart with provider returning PreconditionFailed -> 409.
TEST_F(LifecycleHandlersWithProviderTest, TransitionPreconditionFailedReturns409) {
  mock_->transition_error =
      LifecycleProviderErrorInfo{LifecycleProviderError::PreconditionFailed, "not in correct state", 409};
  auto raw =
      make_request_with_match("/api/v1/apps/plugin_app/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_PRECONDITION_NOT_FULFILLED);
}

// PUT /apps/{plugin_app}/status/restart with provider returning AccessDenied -> 403.
TEST_F(LifecycleHandlersWithProviderTest, TransitionAccessDeniedReturns403) {
  mock_->transition_error =
      LifecycleProviderErrorInfo{LifecycleProviderError::AccessDenied, "operator role required", 403};
  auto raw =
      make_request_with_match("/api/v1/apps/plugin_app/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 403);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INSUFFICIENT_ACCESS_RIGHTS);
}

// PUT /apps/{plugin_app}/status/force-restart with provider returning Unsupported -> 501.
// Distinct from the no-provider 501: here a provider exists but rejects this transition.
TEST_F(LifecycleHandlersWithProviderTest, TransitionUnsupportedReturns501) {
  mock_->transition_error =
      LifecycleProviderErrorInfo{LifecycleProviderError::Unsupported, "force-restart not supported", 501};
  auto raw = make_request_with_match("/api/v1/apps/plugin_app/status/force-restart",
                                     R"(/api/v1/apps/([^/]+)/status/force-restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "force-restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// PUT with provider returning Internal and an out-of-range status -> clamped to 599, plugin-error.
TEST_F(LifecycleHandlersWithProviderTest, TransitionInternalErrorClampsStatus) {
  mock_->transition_error = LifecycleProviderErrorInfo{LifecycleProviderError::Internal, "boom", 650};
  auto raw =
      make_request_with_match("/api/v1/apps/plugin_app/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 599);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_PLUGIN_ERROR);
}

// GET /apps/{plugin_app}/status with provider returning an error -> handler maps it.
// Covers the GET provider-error branch (the no-error path is covered above).
TEST_F(LifecycleHandlersWithProviderTest, GetStatusProviderErrorReturnsMappedCode) {
  mock_->get_status_error =
      LifecycleProviderErrorInfo{LifecycleProviderError::AccessDenied, "viewer cannot read status", 403};
  auto raw = make_request_with_match("/api/v1/apps/plugin_app/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 403);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INSUFFICIENT_ACCESS_RIGHTS);
}

// PUT with provider reporting EntityNotFound -> 404 entity-not-found (not a 500 plugin-error).
TEST_F(LifecycleHandlersWithProviderTest, TransitionEntityNotFoundReturns404) {
  mock_->transition_error =
      LifecycleProviderErrorInfo{LifecycleProviderError::EntityNotFound, "no such substrate unit", 500};
  auto raw =
      make_request_with_match("/api/v1/apps/plugin_app/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
}

// GET with a provider returning a status outside {ready, notReady} -> 500 (contract guard).
TEST_F(LifecycleHandlersWithProviderTest, GetStatusProviderInvalidStatusReturns500) {
  mock_->status_response.status = "running";  // not a valid SOVD lifecycle status

  auto raw = make_request_with_match("/api/v1/apps/plugin_app/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 500);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_PLUGIN_ERROR);
}

// =============================================================================
// Stub-reader lifecycle app status tests.
// Verify that handle_get_status reads lifecycle state for managed nodes and falls
// back to is_online for plain apps.
// =============================================================================

TEST_F(LifecycleHandlersTest, GetStatusLifecycleAppActiveIsReady) {
  auto stub = std::make_shared<StubLifecycleStateReader>();
  stub->next = "active";
  LifecycleHandlers lc_handlers(*ctx_, nullptr, stub);

  App lc_app;
  lc_app.id = "lc_sensor";
  lc_app.name = "LcSensor";
  lc_app.is_online = true;
  lc_app.bound_fqn = "/lc_sensor";
  ServiceInfo gs;
  gs.full_path = "/lc_sensor/get_state";
  gs.type = "lifecycle_msgs/srv/GetState";
  ServiceInfo cs;
  cs.full_path = "/lc_sensor/change_state";
  cs.type = "lifecycle_msgs/srv/ChangeState";
  lc_app.services.push_back(gs);
  lc_app.services.push_back(cs);
  auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
  cache.update_all({}, {}, {lc_app}, {});

  auto raw = make_request_with_match("/api/v1/apps/lc_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);
  auto result = lc_handlers.handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
}

TEST_F(LifecycleHandlersTest, GetStatusLifecycleAppInactiveIsNotReady) {
  auto stub = std::make_shared<StubLifecycleStateReader>();
  stub->next = "inactive";
  LifecycleHandlers lc_handlers(*ctx_, nullptr, stub);

  App lc_app;
  lc_app.id = "lc_idle";
  lc_app.name = "LcIdle";
  lc_app.is_online = true;  // in the graph, but inactive
  lc_app.bound_fqn = "/lc_idle";
  ServiceInfo gs;
  gs.full_path = "/lc_idle/get_state";
  gs.type = "lifecycle_msgs/srv/GetState";
  ServiceInfo cs;
  cs.full_path = "/lc_idle/change_state";
  cs.type = "lifecycle_msgs/srv/ChangeState";
  lc_app.services.push_back(gs);
  lc_app.services.push_back(cs);
  auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
  cache.update_all({}, {}, {lc_app}, {});

  auto raw = make_request_with_match("/api/v1/apps/lc_idle/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);
  auto result = lc_handlers.handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");
}

TEST_F(LifecycleHandlersTest, GetStatusManagedAppOfflineSkipsReaderAndIsNotReady) {
  // A crashed managed node: its get_state/change_state services still linger in
  // the last-cached App::services, but it is gone from the graph (is_online ==
  // false). The handler must short-circuit to "notReady" WITHOUT calling the
  // reader (an offline node cannot be active, and the reader would otherwise
  // block on wait_for_service + spin to the timeout on every poll).
  auto stub = std::make_shared<StubLifecycleStateReader>();
  stub->next = "active";  // would map to "ready" if (wrongly) consulted
  LifecycleHandlers lc_handlers(*ctx_, nullptr, stub);

  App lc_app;
  lc_app.id = "lc_crashed";
  lc_app.name = "LcCrashed";
  lc_app.is_online = false;  // gone from the graph, services still cached
  lc_app.bound_fqn = "/lc_crashed";
  ServiceInfo gs;
  gs.full_path = "/lc_crashed/get_state";
  gs.type = "lifecycle_msgs/srv/GetState";
  ServiceInfo cs;
  cs.full_path = "/lc_crashed/change_state";
  cs.type = "lifecycle_msgs/srv/ChangeState";
  lc_app.services.push_back(gs);
  lc_app.services.push_back(cs);
  auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
  cache.update_all({}, {}, {lc_app}, {});

  auto raw = make_request_with_match("/api/v1/apps/lc_crashed/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);
  auto result = lc_handlers.handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");
  EXPECT_EQ(stub->calls, 0);  // reader must not be consulted for an offline node
}

TEST_F(LifecycleHandlersTest, GetStatusPlainAppStillUsesIsOnline) {
  auto stub = std::make_shared<StubLifecycleStateReader>();
  stub->next = "inactive";  // must be ignored for a plain app
  LifecycleHandlers lc_handlers(*ctx_, nullptr, stub);
  // online_sensor (seeded in SetUp) has no lifecycle services.
  auto raw = make_request_with_match("/api/v1/apps/online_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);
  auto result = lc_handlers.handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");  // is_online == true, lifecycle read not used
}
