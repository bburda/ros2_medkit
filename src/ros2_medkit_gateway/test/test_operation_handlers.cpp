// Copyright 2026 sewon
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
#include <example_interfaces/action/fibonacci.hpp>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/operation_handlers.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::ActionGoalInfo;
using ros2_medkit_gateway::ActionGoalStatus;
using ros2_medkit_gateway::ActionInfo;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::OperationHandlers;
namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

namespace {

using namespace std::chrono_literals;

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

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

class TestLongCalibrationActionServer : public rclcpp::Node {
 public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  TestLongCalibrationActionServer() : rclcpp::Node("test_long_calibration_action", "/powertrain/engine") {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "long_calibration",
        [this](const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const Fibonacci::Goal> & goal) {
          return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
          return handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
          handle_accepted(goal_handle);
        });
  }

  void prepare_shutdown() {
    shutdown_.store(true);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    action_server_.reset();
  }

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
                                          const std::shared_ptr<const Fibonacci::Goal> & goal) {
    if (goal->order > 50) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> & /*goal_handle*/) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&TestLongCalibrationActionServer::execute, this, goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> & goal_handle) {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    const auto goal = goal_handle->get_goal();
    const size_t target_length = static_cast<size_t>(std::max<int32_t>(goal->order, 2));
    feedback->sequence = {0, 1};

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok() && !shutdown_.load()) {
      if (goal_handle->is_canceling()) {
        result->sequence = feedback->sequence;
        goal_handle->canceled(result);
        return;
      }

      feedback->sequence.push_back(static_cast<int32_t>(feedback->sequence.size()));
      goal_handle->publish_feedback(feedback);

      if (feedback->sequence.size() >= target_length) {
        result->sequence = feedback->sequence;
        goal_handle->succeed(result);
        return;
      }

      loop_rate.sleep();
    }

    // Exited loop without succeed/cancel (shutdown or !rclcpp::ok()).
    // Must abort the goal to prevent "terminate called without an active
    // exception" when goal_handle is destroyed with unfinished state.
    try {
      auto abort_result = std::make_shared<Fibonacci::Result>();
      goal_handle->abort(abort_result);
    } catch (...) {
      // Ignore errors during shutdown abort
    }
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  std::thread execution_thread_;
  std::atomic<bool> shutdown_{false};
};

}  // namespace

// =============================================================================
// Validation-only tests (no GatewayNode). These cover the path_param("1")
// short-circuit at the top of each typed handler. Default-constructed
// TypedRequest carries no captures, so the handler returns ERR_INVALID_REQUEST
// (400) before touching the cache.
// =============================================================================

class OperationHandlersValidationTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  OperationHandlers handlers_{ctx_};
};

TEST_F(OperationHandlersValidationTest, ListOperationsMissingMatchesReturns400) {
  httplib::Request raw_req;
  raw_req.path = "/api/v1/components/engine/operations";
  http::TypedRequest req(raw_req);

  auto result = handlers_.list_operations(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(OperationHandlersValidationTest, ListOperationsInvalidEntityReturns400) {
  auto raw_req =
      make_request_with_match("/api/v1/components/engine!/operations", R"(/api/v1/components/([^/]+)/operations)");
  http::TypedRequest req(raw_req);

  auto result = handlers_.list_operations(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

// =============================================================================
// Fixture-based tests against a live GatewayNode + ROS 2 graph.
// =============================================================================

class OperationHandlersFixtureTest : public ::testing::Test {
 protected:
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {"test_operation_handlers", "--ros-args", "-p",
                                     "server.port:=" + std::to_string(suite_server_port_), "-p",
                                     "refresh_interval_ms:=60000", "-p",
                                     // Pin the graph-event refresh debounce at its 60s maximum so the
                                     // single startup refresh_cache() that reconciles the test nodes is
                                     // the only one for the test's lifetime. Without this, a refresh
                                     // fired by the goal's client/subscription graph churn could wipe the
                                     // manually seeded component between the seed and list_executions.
                                     "discovery.refresh_debounce_ms:=60000", "-p", "service_call_timeout_sec:=1"};

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

    // Cache generation right after construction: only the gateway's own graph is
    // discovered at this point. The first graph-event-driven refresh_cache()
    // after the test nodes join advances it, which is the signal we wait on
    // before seeding.
    const uint64_t base_generation = gateway_node_->get_thread_safe_cache().generation();

    service_node_ = std::make_shared<rclcpp::Node>("test_calibrate_service", "/powertrain/engine");
    trigger_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
        "calibrate", [](const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
          response->success = true;
          response->message = "calibration complete";
        });

    action_server_node_ = std::make_shared<TestLongCalibrationActionServer>();

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    executor_->add_node(service_node_);
    executor_->add_node(action_server_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<OperationHandlers>(*ctx_);

    // Deterministic readiness, replacing a fixed discovery sleep. Two signals
    // must hold before we seed and drive operations:
    //   1. The action's send_goal service and the trigger service are visible
    //      on the gateway participant, so create_execution's send_goal
    //      (wait_for_service) resolves instead of racing DDS discovery.
    //   2. The cache generation advanced past construction, proving the single
    //      graph-event-driven refresh_cache() that reconciles these nodes has
    //      already run. With discovery.refresh_debounce_ms pinned at 60s, that
    //      first refresh is the only one for the test's lifetime, so seeding
    //      afterwards is the final cache write and cannot be clobbered by a
    //      concurrent refresh mid-test (the race that made ListExecutions flake
    //      under load).
    ASSERT_TRUE(wait_for_discovery_settled(base_generation))
        << "action/service discovery did not settle before seeding";
    seed_component_cache();
  }

  void TearDown() override {
    if (executor_ != nullptr) {
      executor_->cancel();
    }

    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    if (action_server_node_ != nullptr) {
      action_server_node_->prepare_shutdown();
    }

    handlers_.reset();
    ctx_.reset();
    trigger_service_.reset();

    executor_.reset();

    action_server_node_.reset();
    service_node_.reset();
    gateway_node_.reset();
  }

  /// Block until DDS discovery has surfaced the action's send_goal service and
  /// the trigger service on the gateway participant AND the gateway has run its
  /// first post-startup refresh_cache() (cache generation advanced past
  /// `base_generation`). Returns false if either signal is missing at the
  /// deadline. This is a real readiness gate, not a fixed sleep.
  bool wait_for_discovery_settled(uint64_t base_generation, std::chrono::seconds timeout = std::chrono::seconds(15)) {
    const std::string send_goal_srv = "/powertrain/engine/long_calibration/_action/send_goal";
    const std::string calibrate_srv = "/powertrain/engine/calibrate";
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool graph_ready = false;
    bool refresh_ran = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!graph_ready) {
        const auto services = gateway_node_->get_service_names_and_types();
        graph_ready = services.count(send_goal_srv) > 0 && services.count(calibrate_srv) > 0;
      }
      if (!refresh_ran) {
        refresh_ran = gateway_node_->get_thread_safe_cache().generation() > base_generation;
      }
      if (graph_ready && refresh_ran) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void seed_component_cache() {
    Component component;
    component.id = "engine";
    component.name = "Engine";
    component.namespace_path = "/powertrain/engine";
    component.fqn = "/powertrain/engine";
    component.area = "powertrain";
    component.source = "manifest";
    component.services = {
        ServiceInfo{"calibrate", "/powertrain/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt}};
    component.actions = {ActionInfo{"long_calibration", "/powertrain/engine/long_calibration",
                                    "example_interfaces/action/Fibonacci", std::nullopt}};

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {component}, {}, {});
  }

  /// Drive `create_execution` and assert the typed response carries the async
  /// (202) branch. Returns the goal UUID.
  std::string create_action_execution(int order = 6) {
    auto raw_req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions",
                                           R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
    http::TypedRequest typed(raw_req);
    dto::ExecutionCreateRequest body;
    body.parameters = json{{"order", order}};

    auto result = handlers_->create_execution(typed, body);
    EXPECT_TRUE(result.has_value());
    if (!result.has_value()) {
      return {};
    }
    // 202 async branch.
    const auto * async_ptr = std::get_if<dto::ExecutionCreateAsync>(&result.value().first);
    EXPECT_NE(async_ptr, nullptr);
    if (async_ptr == nullptr) {
      return {};
    }
    EXPECT_FALSE(async_ptr->id.empty());

    // No re-seed needed: with the refresh debounce pinned at 60s, the goal's
    // client/subscription graph churn cannot trigger a refresh that wipes the
    // seed within the test's lifetime.
    return async_ptr->id;
  }

  ActionGoalInfo get_tracked_goal_or_fail(const std::string & execution_id) {
    auto goal_info = gateway_node_->get_operation_manager()->get_tracked_goal(execution_id);
    EXPECT_TRUE(goal_info.has_value());
    return *goal_info;
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<TestLongCalibrationActionServer> action_server_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<OperationHandlers> handlers_;
};

TEST_F(OperationHandlersFixtureTest, ListOperationsReturnsServiceAndActionItems) {
  auto raw_req =
      make_request_with_match("/api/v1/components/engine/operations", R"(/api/v1/components/([^/]+)/operations)");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->list_operations(typed);
  ASSERT_TRUE(result.has_value());
  const auto & collection = *result;
  ASSERT_EQ(collection.items.size(), 2u);

  std::set<std::string> ids;
  for (const auto & item : collection.items) {
    ids.insert(item.id);
  }
  EXPECT_EQ(ids, std::set<std::string>({"calibrate", "long_calibration"}));
}

TEST_F(OperationHandlersFixtureTest, ListOperationsUnknownEntityReturns404) {
  auto raw_req =
      make_request_with_match("/api/v1/components/unknown/operations", R"(/api/v1/components/([^/]+)/operations)");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->list_operations(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, GetOperationReturnsActionMetadata) {
  auto raw_req = make_request_with_match("/api/v1/components/engine/operations/long_calibration",
                                         R"(/api/v1/components/([^/]+)/operations/([^/]+))");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->get_operation(typed);
  ASSERT_TRUE(result.has_value());
  const auto & detail = *result;
  EXPECT_EQ(detail.item.id, "long_calibration");
  EXPECT_TRUE(detail.item.asynchronous_execution);
  ASSERT_TRUE(detail.item.x_medkit.has_value());
  ASSERT_TRUE(detail.item.x_medkit->ros2.has_value());
  EXPECT_EQ(detail.item.x_medkit->ros2->kind, "action");
  EXPECT_EQ(detail.item.x_medkit->ros2->action, "/powertrain/engine/long_calibration");
}

TEST_F(OperationHandlersFixtureTest, GetOperationUnknownOperationReturns404) {
  auto raw_req = make_request_with_match("/api/v1/components/engine/operations/does_not_exist",
                                         R"(/api/v1/components/([^/]+)/operations/([^/]+))");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->get_operation(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_OPERATION_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, CreateExecutionOnServiceReturnsSynchronousResponse) {
  auto raw_req = make_request_with_match("/api/v1/components/engine/operations/calibrate/executions",
                                         R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
  http::TypedRequest typed(raw_req);
  dto::ExecutionCreateRequest body;
  body.parameters = json::object();

  auto result = handlers_->create_execution(typed, body);
  ASSERT_TRUE(result.has_value());
  // Synchronous service -> OperationExecutionResult branch (200).
  const auto * sync_ptr = std::get_if<dto::OperationExecutionResult>(&result.value().first);
  ASSERT_NE(sync_ptr, nullptr);
  ASSERT_TRUE(sync_ptr->content.contains("parameters"));
  EXPECT_TRUE(sync_ptr->content["parameters"]["success"].get<bool>());
  EXPECT_EQ(sync_ptr->content["parameters"]["message"], "calibration complete");
}

TEST_F(OperationHandlersFixtureTest, ListExecutionsReturnsTrackedActionGoal) {
  const auto execution_id = create_action_execution();
  ASSERT_FALSE(execution_id.empty());

  auto raw_req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions",
                                         R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->list_executions(typed);
  ASSERT_TRUE(result.has_value());
  const auto & collection = *result;
  ASSERT_EQ(collection.items.size(), 1u);
  EXPECT_EQ(collection.items[0].id, execution_id);
}

TEST_F(OperationHandlersFixtureTest, GetExecutionContainsStatusFields) {
  const auto execution_id = create_action_execution();
  ASSERT_FALSE(execution_id.empty());
  gateway_node_->get_operation_manager()->update_goal_feedback(execution_id, json{{"progress", 50}});

  auto raw_req =
      make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions/" + execution_id,
                              R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->get_execution(typed);
  ASSERT_TRUE(result.has_value());
  const auto & exec = *result;
  EXPECT_EQ(exec.status, "running");
  ASSERT_TRUE(exec.capability.has_value());
  EXPECT_EQ(*exec.capability, "execute");
  ASSERT_TRUE(exec.parameters.has_value());
  EXPECT_EQ((*exec.parameters)["progress"], 50);
  ASSERT_TRUE(exec.x_medkit.has_value());
  EXPECT_EQ(exec.x_medkit->goal_id, execution_id);
  ASSERT_TRUE(exec.x_medkit->ros2.has_value());
  EXPECT_EQ(exec.x_medkit->ros2->action, "/powertrain/engine/long_calibration");
}

TEST_F(OperationHandlersFixtureTest, CancelExecutionUnknownIdReturns404) {
  auto raw_req = make_request_with_match(
      "/api/v1/components/engine/operations/long_calibration/executions/0123456789abcdef0123456789abcdef",
      R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  http::TypedRequest typed(raw_req);

  auto result = handlers_->cancel_execution(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, UpdateExecutionStopReturnsAcceptedAndLocation) {
  const auto execution_id = create_action_execution(20);
  ASSERT_FALSE(execution_id.empty());

  auto raw_req =
      make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions/" + execution_id,
                              R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  http::TypedRequest typed(raw_req);
  dto::ExecutionUpdateRequest body;
  body.capability = "stop";

  auto result = handlers_->update_execution(typed, body);
  auto goal_info = get_tracked_goal_or_fail(execution_id);

  if (result.has_value()) {
    const auto & exec = result.value().first;
    const auto & att = result.value().second;
    ASSERT_TRUE(att.status_override.has_value());
    EXPECT_EQ(*att.status_override, 202);
    bool has_location = false;
    for (const auto & [k, v] : att.headers) {
      if (k == "Location") {
        EXPECT_EQ(v, "/api/v1/components/engine/operations/long_calibration/executions/" + execution_id);
        has_location = true;
      }
    }
    EXPECT_TRUE(has_location);
    ASSERT_TRUE(exec.id.has_value());
    EXPECT_EQ(*exec.id, execution_id);
    EXPECT_EQ(exec.status, "running");
    EXPECT_EQ(goal_info.status, ActionGoalStatus::CANCELING);
  } else {
    // The fixture's action server always ACCEPTS cancels, so the only
    // realistic failure here is a lost/late CancelGoal response whose
    // timeout could not be reconciled against the status stream: 504 +
    // standard `not-responding` (issue #576). The old expectation asserted
    // ERR_VENDOR_ERROR, which the handler never produced - that constant
    // only ever existed on the wire after the renderer's remap.
    EXPECT_EQ(result.error().http_status, 504);
    EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_RESPONDING);
  }
}

TEST_F(OperationHandlersFixtureTest, UpdateExecutionMissingCapabilityReturns400AtFrameworkLevel) {
  // The framework's typed `put<TBody>` overload parses the body via
  // JsonReader<ExecutionUpdateRequest> before the handler is invoked; a body
  // missing the required `capability` field never reaches the handler. We
  // exercise that contract by trying to read the body directly and asserting
  // the read fails (same wire effect: 400 ERR_INVALID_REQUEST).
  json bad_body = json{{"parameters", {{"order", 8}}}};
  auto parsed = dto::JsonReader<dto::ExecutionUpdateRequest>::read(bad_body);
  EXPECT_FALSE(parsed.has_value());
}
