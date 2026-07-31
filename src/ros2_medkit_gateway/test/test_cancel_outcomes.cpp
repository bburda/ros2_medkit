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

// Falsifiers for the cancel-outcome mapping (issue #576).
//
// A real rclcpp action server answers CancelGoal automatically through rcl
// machinery, so it can never be made to swallow the request deterministically.
// This fixture therefore offers `<action>/_action/cancel_goal` as a RAW
// `action_msgs/srv/CancelGoal` service whose callback either blocks past the
// configured budget (the "response lost" case) or rejects immediately, and
// plants the tracked goal via `inject_tracked_goal_for_testing`. The wire
// contract is pinned with literal status codes and code strings on purpose:
//
// - a timed-out cancel is NOT a rejection: 504 + standard `not-responding`,
//   tracked status untouched (the /_action/status stream stays the authority);
// - a timed-out cancel whose status stream already shows CANCELING/CANCELED
//   is a cancellation in progress: success (204 / 202 for PUT-stop);
// - a genuine server rejection stays 400 + `x-medkit-ros2-action-rejected`.

#include <gtest/gtest.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <utility>

#include "ros2_medkit_gateway/core/http/handlers/operation_handlers.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::ActionGoalInfo;
using ros2_medkit_gateway::ActionGoalStatus;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
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

/// Raw CancelGoal service + status publisher standing in for an action
/// server whose cancel path misbehaves. Two modes:
/// - kBlockUntilReleased: the service callback parks on a condition variable
///   (bounded, releasable) so the caller's response future must time out -
///   the deterministic "cancel response lost" case a real action server
///   cannot produce.
/// - kRejectImmediately: replies ERROR_REJECTED (return_code=1) at once -
///   the definitive-rejection case.
class PhantomCancelFixtureNode : public rclcpp::Node {
 public:
  enum class CancelMode { kBlockUntilReleased, kRejectImmediately };

  PhantomCancelFixtureNode() : rclcpp::Node("phantom_cancel_fixture", "/powertrain/engine") {
    cancel_service_ = create_service<action_msgs::srv::CancelGoal>(
        "phantom_calibration/_action/cancel_goal",
        [this](const std::shared_ptr<action_msgs::srv::CancelGoal::Request> & /*request*/,
               const std::shared_ptr<action_msgs::srv::CancelGoal::Response> & response) {
          if (mode_.load() == CancelMode::kRejectImmediately) {
            response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_REJECTED;
            return;
          }
          // Swallow the request past any realistic budget so the caller's
          // future times out. Bounded and releasable so teardown never hangs
          // an executor thread.
          std::unique_lock<std::mutex> lock(release_mutex_);
          release_cv_.wait_for(lock, std::chrono::seconds(30), [this] {
            return released_;
          });
          response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_NONE;
        });
    status_pub_ =
        create_publisher<action_msgs::msg::GoalStatusArray>("phantom_calibration/_action/status", rclcpp::QoS(10));
  }

  // Subscription-destructor pattern: the service callback captures `this`,
  // so it must be released and dropped before member destruction begins.
  ~PhantomCancelFixtureNode() override {
    release_blocked_cancels();
    cancel_service_.reset();
    status_pub_.reset();
  }

  PhantomCancelFixtureNode(const PhantomCancelFixtureNode &) = delete;
  PhantomCancelFixtureNode & operator=(const PhantomCancelFixtureNode &) = delete;
  PhantomCancelFixtureNode(PhantomCancelFixtureNode &&) = delete;
  PhantomCancelFixtureNode & operator=(PhantomCancelFixtureNode &&) = delete;

  void set_mode(CancelMode mode) {
    mode_.store(mode);
  }

  void release_blocked_cancels() {
    {
      std::lock_guard<std::mutex> lock(release_mutex_);
      released_ = true;
    }
    release_cv_.notify_all();
  }

  void publish_status(const std::array<uint8_t, 16> & uuid, int8_t status_byte) {
    action_msgs::msg::GoalStatusArray msg;
    action_msgs::msg::GoalStatus status;
    status.goal_info.goal_id.uuid = uuid;
    status.status = status_byte;
    msg.status_list.push_back(status);
    status_pub_->publish(msg);
  }

 private:
  rclcpp::Service<action_msgs::srv::CancelGoal>::SharedPtr cancel_service_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub_;
  std::atomic<CancelMode> mode_{CancelMode::kBlockUntilReleased};
  std::mutex release_mutex_;
  std::condition_variable release_cv_;
  bool released_{false};
};

}  // namespace

class CancelOutcomesFixtureTest : public ::testing::Test {
 protected:
  static constexpr const char * kActionPath = "/powertrain/engine/phantom_calibration";
  static constexpr const char * kGoalIdHex = "00112233445566778899aabbccddeeff";
  static constexpr const char * kCancelServiceName = "/powertrain/engine/phantom_calibration/_action/cancel_goal";

  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    // service_call_timeout_sec:=1 keeps the blocking-cancel tests fast: the
    // cancel budget must follow the configured timeout (no hidden floor).
    // Refresh + debounce are pinned high so discovery churn cannot interfere.
    std::vector<std::string> args = {"test_cancel_outcomes",
                                     "--ros-args",
                                     "-p",
                                     "server.port:=" + std::to_string(suite_server_port_),
                                     "-p",
                                     "refresh_interval_ms:=60000",
                                     "-p",
                                     "discovery.refresh_debounce_ms:=60000",
                                     "-p",
                                     "service_call_timeout_sec:=1"};

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
    fixture_node_ = std::make_shared<PhantomCancelFixtureNode>();

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    executor_->add_node(fixture_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<OperationHandlers>(*ctx_);

    // The transport's wait_for_service must resolve the phantom cancel
    // service, otherwise every test would exercise the unavailability path
    // instead of its intended outcome.
    ASSERT_TRUE(wait_for_cancel_service()) << "phantom cancel service not discovered by the gateway participant";

    prime_action_clients();
  }

  void TearDown() override {
    // Release any parked cancel callback BEFORE cancelling the executor:
    // a thread stuck inside the blocking service callback would otherwise
    // hold up spin_thread_.join() for the fixture's 30s backstop.
    if (fixture_node_ != nullptr) {
      fixture_node_->release_blocked_cancels();
    }
    if (executor_ != nullptr) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    handlers_.reset();
    ctx_.reset();
    executor_.reset();
    fixture_node_.reset();
    gateway_node_.reset();
  }

  /// Cache the transport's client trio for the phantom action with its real
  /// interface type. In production, cancel always follows a send through this
  /// gateway, so the trio is already cached with the correct action type;
  /// injected goals bypass send, and a cancel without cached clients would
  /// take the placeholder-type path instead of the outcome under test. The
  /// send itself fails fast (no send_goal server exists) - only its
  /// client-creation side effect matters.
  void prime_action_clients() {
    auto sent = gateway_node_->get_operation_manager()->send_action_goal(
        kActionPath, "example_interfaces/action/Fibonacci", json::object(), "engine");
    ASSERT_FALSE(sent.success) << "no send_goal server exists - the priming send must fail";
  }

  bool wait_for_cancel_service(std::chrono::seconds timeout = std::chrono::seconds(15)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto services = gateway_node_->get_service_names_and_types();
      if (services.count(kCancelServiceName) > 0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void inject_goal(ActionGoalStatus status = ActionGoalStatus::EXECUTING) {
    ActionGoalInfo info;
    info.goal_id = kGoalIdHex;
    info.action_path = kActionPath;
    info.action_type = "example_interfaces/action/Fibonacci";
    info.entity_id = "engine";
    info.status = status;
    info.created_at = std::chrono::system_clock::now();
    info.last_update = info.created_at;
    gateway_node_->get_operation_manager()->inject_tracked_goal_for_testing(std::move(info));
  }

  static std::array<uint8_t, 16> goal_id_bytes() {
    std::array<uint8_t, 16> bytes{};
    const std::string hex = kGoalIdHex;
    for (size_t i = 0; i < bytes.size(); ++i) {
      bytes[i] = static_cast<uint8_t>(std::stoi(hex.substr(i * 2, 2), nullptr, 16));
    }
    return bytes;
  }

  /// Subscribe the manager to the phantom action's status stream (the seam
  /// `send_action_goal` uses in production - public on OperationManager) and
  /// publish raw GoalStatusArray frames until the tracked goal reflects
  /// `wanted`. Publishing repeats because the best-effort subscription may
  /// not have matched yet on the first frame.
  bool deliver_status_until_tracked(ActionGoalStatus wanted, int8_t status_byte) {
    auto * operation_mgr = gateway_node_->get_operation_manager();
    operation_mgr->subscribe_to_action_status(kActionPath);
    const auto uuid = goal_id_bytes();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      fixture_node_->publish_status(uuid, status_byte);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto tracked = operation_mgr->get_tracked_goal(kGoalIdHex);
      if (tracked.has_value() && tracked->status == wanted) {
        return true;
      }
    }
    return false;
  }

  http::TypedRequest make_execution_request() {
    raw_req_ = make_request_with_match(
        std::string("/api/v1/components/engine/operations/phantom_calibration/executions/") + kGoalIdHex,
        R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
    return http::TypedRequest(raw_req_);
  }

  ActionGoalStatus tracked_status_or_fail() {
    auto tracked = gateway_node_->get_operation_manager()->get_tracked_goal(kGoalIdHex);
    EXPECT_TRUE(tracked.has_value());
    return tracked.has_value() ? tracked->status : ActionGoalStatus::UNKNOWN;
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  httplib::Request raw_req_;
  std::shared_ptr<GatewayNode> gateway_node_;
  std::shared_ptr<PhantomCancelFixtureNode> fixture_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<OperationHandlers> handlers_;
};

// ---------------------------------------------------------------------------
// DELETE /{entity}/operations/{op}/executions/{id}
// ---------------------------------------------------------------------------

TEST_F(CancelOutcomesFixtureTest, CancelTimeoutReturns504NotRespondingAndLeavesTrackedStatus) {
  inject_goal();
  auto typed = make_execution_request();

  auto result = handlers_->cancel_execution(typed);

  ASSERT_FALSE(result.has_value()) << "a swallowed cancel must not be reported as success";
  EXPECT_EQ(result.error().http_status, 504) << result.error().code << ": " << result.error().message;
  EXPECT_EQ(result.error().code, "not-responding");
  // The outcome is unknown - the handler must not fabricate a tracked status;
  // the /_action/status stream stays the authority.
  EXPECT_EQ(tracked_status_or_fail(), ActionGoalStatus::EXECUTING);
}

TEST_F(CancelOutcomesFixtureTest, CancelTimeoutWithCancelingStatusReturns204) {
  inject_goal();
  ASSERT_TRUE(deliver_status_until_tracked(ActionGoalStatus::CANCELING, action_msgs::msg::GoalStatus::STATUS_CANCELING))
      << "status stream never reached the tracked goal";

  auto typed = make_execution_request();
  auto result = handlers_->cancel_execution(typed);

  ASSERT_TRUE(result.has_value()) << "cancel must reconcile against the status stream: " << result.error().code << ": "
                                  << result.error().message;
  EXPECT_EQ(tracked_status_or_fail(), ActionGoalStatus::CANCELING);

  // GET execution status must agree with what the cancel response implied.
  auto get_typed = make_execution_request();
  auto exec = handlers_->get_execution(get_typed);
  ASSERT_TRUE(exec.has_value());
  EXPECT_EQ(exec->status, "running");  // CANCELING renders as SOVD "running"
  ASSERT_TRUE(exec->x_medkit.has_value());
  ASSERT_TRUE(exec->x_medkit->ros2_status.has_value());
  EXPECT_EQ(*exec->x_medkit->ros2_status, "canceling");
}

TEST_F(CancelOutcomesFixtureTest, CancelRejectedByServerReturns400Rejected) {
  fixture_node_->set_mode(PhantomCancelFixtureNode::CancelMode::kRejectImmediately);
  inject_goal();
  auto typed = make_execution_request();

  auto result = handlers_->cancel_execution(typed);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "x-medkit-ros2-action-rejected");
  EXPECT_EQ(result.error().message, "Cancel request rejected");
  EXPECT_EQ(result.error().params["return_code"], 1);
}

// ---------------------------------------------------------------------------
// PUT /{entity}/operations/{op}/executions/{id} with {"capability": "stop"}
// ---------------------------------------------------------------------------

TEST_F(CancelOutcomesFixtureTest, PutStopTimeoutReturns504NotResponding) {
  inject_goal();
  auto typed = make_execution_request();
  dto::ExecutionUpdateRequest body;
  body.capability = "stop";

  auto result = handlers_->update_execution(typed, body);

  ASSERT_FALSE(result.has_value()) << "a swallowed stop must not be reported as accepted";
  EXPECT_EQ(result.error().http_status, 504) << result.error().code << ": " << result.error().message;
  EXPECT_EQ(result.error().code, "not-responding");
  EXPECT_EQ(tracked_status_or_fail(), ActionGoalStatus::EXECUTING);
}

TEST_F(CancelOutcomesFixtureTest, PutStopTimeoutWithCancelingStatusReturns202) {
  inject_goal();
  ASSERT_TRUE(deliver_status_until_tracked(ActionGoalStatus::CANCELING, action_msgs::msg::GoalStatus::STATUS_CANCELING))
      << "status stream never reached the tracked goal";

  auto typed = make_execution_request();
  dto::ExecutionUpdateRequest body;
  body.capability = "stop";

  auto result = handlers_->update_execution(typed, body);

  ASSERT_TRUE(result.has_value()) << "stop must reconcile against the status stream: " << result.error().code << ": "
                                  << result.error().message;
  const auto & att = result.value().second;
  ASSERT_TRUE(att.status_override.has_value());
  EXPECT_EQ(*att.status_override, 202);
  EXPECT_EQ(result.value().first.status, "running");
  EXPECT_EQ(tracked_status_or_fail(), ActionGoalStatus::CANCELING);
}

TEST_F(CancelOutcomesFixtureTest, PutStopRejectedByServerReturns400Rejected) {
  fixture_node_->set_mode(PhantomCancelFixtureNode::CancelMode::kRejectImmediately);
  inject_goal();
  auto typed = make_execution_request();
  dto::ExecutionUpdateRequest body;
  body.capability = "stop";

  auto result = handlers_->update_execution(typed, body);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "x-medkit-ros2-action-rejected");
  EXPECT_EQ(result.error().message, "Stop request rejected");
}
