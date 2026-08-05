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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/service_action_resolver.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/operations/operation_types.hpp"
#include "ros2_medkit_gateway/core/transports/action_transport.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

class MockServiceTransport : public ServiceTransport {
 public:
  MockServiceTransport() = default;
  ~MockServiceTransport() override = default;
  MockServiceTransport(const MockServiceTransport &) = delete;
  MockServiceTransport & operator=(const MockServiceTransport &) = delete;
  MockServiceTransport(MockServiceTransport &&) = delete;
  MockServiceTransport & operator=(MockServiceTransport &&) = delete;

  ServiceCallResult call(const std::string & service_path, const std::string & service_type, const json & request,
                         std::chrono::duration<double> timeout) override {
    last_service_path_ = service_path;
    last_service_type_ = service_type;
    last_request_ = request;
    last_timeout_ = timeout.count();
    call_count_ += 1;
    ServiceCallResult r;
    r.success = success_;
    r.response = response_;
    r.error_message = error_message_;
    return r;
  }

  std::string last_service_path_;
  std::string last_service_type_;
  json last_request_;
  double last_timeout_ = 0.0;
  int call_count_ = 0;
  bool success_ = true;
  json response_ = json{{"result", "ok"}};
  std::string error_message_;
};

class MockActionTransport : public ActionTransport {
 public:
  MockActionTransport() = default;
  ~MockActionTransport() override = default;
  MockActionTransport(const MockActionTransport &) = delete;
  MockActionTransport & operator=(const MockActionTransport &) = delete;
  MockActionTransport(MockActionTransport &&) = delete;
  MockActionTransport & operator=(MockActionTransport &&) = delete;

  ActionSendGoalResult send_goal(const std::string & action_path, const std::string & action_type, const json & goal,
                                 std::chrono::duration<double> timeout) override {
    last_send_path_ = action_path;
    last_send_type_ = action_type;
    last_send_goal_ = goal;
    last_send_timeout_ = timeout.count();
    send_calls_ += 1;
    ActionSendGoalResult r;
    r.success = send_success_;
    r.goal_accepted = send_accepted_;
    r.goal_id = send_goal_id_;
    r.error_message = send_error_;
    return r;
  }

  ActionCancelResult cancel_goal(const std::string & action_path, const std::string & goal_id,
                                 std::chrono::duration<double> timeout) override {
    last_cancel_path_ = action_path;
    last_cancel_goal_id_ = goal_id;
    last_cancel_timeout_ = timeout.count();
    cancel_calls_ += 1;
    ActionCancelResult r;
    r.success = cancel_success_;
    r.return_code = cancel_return_code_;
    r.outcome = cancel_success_ && cancel_return_code_ == 0 ? CancelOutcome::kOk : CancelOutcome::kErrorResponse;
    r.error_message = cancel_error_;
    return r;
  }

  ActionGetResultResult get_result(const std::string & action_path, const std::string & action_type,
                                   const std::string & goal_id, std::chrono::duration<double> timeout) override {
    last_get_path_ = action_path;
    last_get_type_ = action_type;
    last_get_goal_id_ = goal_id;
    last_get_timeout_ = timeout.count();
    get_calls_ += 1;
    ActionGetResultResult r;
    r.success = get_success_;
    r.status = get_status_;
    r.result = get_payload_;
    r.error_message = get_error_;
    return r;
  }

  void subscribe_status(const std::string & action_path, StatusCallback callback) override {
    subscribed_paths_.push_back(action_path);
    // Idempotent, exactly like Ros2ActionTransport::subscribe_status: while a
    // subscription for the path still exists, re-subscribing keeps the first
    // callback and creates nothing. A double that silently re-armed itself
    // here could not observe the window this models.
    if (callbacks_.count(action_path) > 0) {
      return;
    }
    callbacks_[action_path] = std::move(callback);
  }

  void unsubscribe_status(const std::string & action_path) override {
    // One-shot re-entry: stand in for an HTTP worker that lands between the
    // manager releasing subscriptions_mutex_ and this call. Production reaches
    // exactly this interleaving - the manager holds no lock here.
    if (reentry_ != nullptr) {
      auto reentry = std::move(reentry_);
      reentry_ = nullptr;
      reentry();
    }
    unsubscribed_paths_.push_back(action_path);
    callbacks_.erase(action_path);
  }

  /// Runs once, inside the next unsubscribe_status call, before it erases.
  std::function<void()> reentry_;

  /// Test helper to fire a status update on a previously subscribed path.
  void fire_status(const std::string & action_path, const std::string & goal_id, ActionGoalStatus status) {
    auto it = callbacks_.find(action_path);
    if (it != callbacks_.end() && it->second) {
      it->second(action_path, goal_id, status);
    }
  }

  // Send-goal knobs.
  bool send_success_ = true;
  bool send_accepted_ = true;
  std::string send_goal_id_ = "0123456789abcdef0123456789abcdef";
  std::string send_error_;
  int send_calls_ = 0;
  std::string last_send_path_;
  std::string last_send_type_;
  json last_send_goal_;
  double last_send_timeout_ = 0.0;

  // Cancel knobs.
  bool cancel_success_ = true;
  int8_t cancel_return_code_ = 0;
  std::string cancel_error_;
  int cancel_calls_ = 0;
  std::string last_cancel_path_;
  std::string last_cancel_goal_id_;
  double last_cancel_timeout_ = 0.0;

  // Get-result knobs.
  bool get_success_ = true;
  ActionGoalStatus get_status_ = ActionGoalStatus::SUCCEEDED;
  json get_payload_ = json{{"value", 42}};
  std::string get_error_;
  int get_calls_ = 0;
  std::string last_get_path_;
  std::string last_get_type_;
  std::string last_get_goal_id_;
  double last_get_timeout_ = 0.0;

  std::vector<std::string> subscribed_paths_;
  std::vector<std::string> unsubscribed_paths_;
  std::map<std::string, StatusCallback> callbacks_;
};

class MockResolver : public ServiceActionResolver {
 public:
  MockResolver() = default;
  std::optional<ServiceInfo> find_service(const std::string & /*entity_id*/,
                                          const std::string & /*operation_id*/) const override {
    return service_;
  }
  std::optional<ActionInfo> find_action(const std::string & /*entity_id*/,
                                        const std::string & /*operation_id*/) const override {
    return action_;
  }

  std::optional<ServiceInfo> service_;
  std::optional<ActionInfo> action_;
};

}  // namespace

TEST(OperationManagerRoutingTest, CallServiceDelegatesToServiceTransport) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/3);
  auto out = mgr.call_service("/foo/srv", "std_srvs/srv/Trigger", json{{"x", 1}});
  EXPECT_TRUE(out.success);
  EXPECT_EQ(svc->last_service_path_, "/foo/srv");
  EXPECT_EQ(svc->last_service_type_, "std_srvs/srv/Trigger");
  EXPECT_EQ(svc->last_request_["x"], 1);
  EXPECT_DOUBLE_EQ(svc->last_timeout_, 3.0);
  EXPECT_EQ(svc->call_count_, 1);
}

TEST(OperationManagerRoutingTest, CallComponentServiceUsesResolverWhenTypeMissing) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  MockResolver resolver;
  ServiceInfo info;
  info.full_path = "/cmp/op";
  info.type = "std_srvs/srv/Trigger";
  resolver.service_ = info;
  OperationManager mgr(svc, act, &resolver, /*timeout=*/2);
  auto out = mgr.call_component_service("/cmp", "op", std::nullopt, json{});
  EXPECT_TRUE(out.success);
  EXPECT_EQ(svc->last_service_path_, "/cmp/op");
  EXPECT_EQ(svc->last_service_type_, "std_srvs/srv/Trigger");
}

TEST(OperationManagerRoutingTest, CallComponentServiceRejectsActionType) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/1);
  auto out = mgr.call_component_service("/cmp", "op", std::optional<std::string>{"example_interfaces/action/Fibonacci"},
                                        json{});
  EXPECT_FALSE(out.success);
  EXPECT_NE(out.error_message.find("not a service type"), std::string::npos);
  EXPECT_EQ(svc->call_count_, 0);
}

TEST(OperationManagerRoutingTest, SendActionGoalRoutesAndTracksGoal) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/4);
  auto out = mgr.send_action_goal("/path/act", "example_interfaces/action/Fibonacci", json{{"order", 5}}, "engine");
  EXPECT_TRUE(out.success);
  EXPECT_TRUE(out.goal_accepted);
  EXPECT_EQ(act->last_send_path_, "/path/act");
  EXPECT_EQ(act->last_send_type_, "example_interfaces/action/Fibonacci");
  EXPECT_EQ(act->last_send_goal_["order"], 5);
  EXPECT_DOUBLE_EQ(act->last_send_timeout_, 4.0);

  // Manager tracks the goal returned by the transport.
  auto tracked = mgr.get_tracked_goal(out.goal_id);
  ASSERT_TRUE(tracked.has_value());
  EXPECT_EQ(tracked->action_path, "/path/act");
  EXPECT_EQ(tracked->entity_id, "engine");
  EXPECT_EQ(tracked->status, ActionGoalStatus::EXECUTING);

  // And subscribed to status updates on that path.
  ASSERT_EQ(act->subscribed_paths_.size(), 1u);
  EXPECT_EQ(act->subscribed_paths_[0], "/path/act");
}

TEST(OperationManagerRoutingTest, StatusCallbackUpdatesTrackedGoal) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/4);
  auto out = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "engine");
  ASSERT_TRUE(out.success);

  // Drive a status update through the transport's wired callback.
  act->fire_status("/p/a", out.goal_id, ActionGoalStatus::SUCCEEDED);

  auto tracked = mgr.get_tracked_goal(out.goal_id);
  ASSERT_TRUE(tracked.has_value());
  EXPECT_EQ(tracked->status, ActionGoalStatus::SUCCEEDED);
}

TEST(OperationManagerRoutingTest, CancelActionGoalRoutesToActionTransport) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);
  auto sent = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(sent.success);

  auto out = mgr.cancel_action_goal("/p/a", sent.goal_id);
  EXPECT_TRUE(out.success);
  EXPECT_EQ(act->cancel_calls_, 1);
  EXPECT_EQ(act->last_cancel_path_, "/p/a");
  EXPECT_EQ(act->last_cancel_goal_id_, sent.goal_id);
  // The cancel budget must follow the configured service timeout exactly,
  // like every other action RPC - no hidden floor (issue #576).
  EXPECT_DOUBLE_EQ(act->last_cancel_timeout_, 2.0);

  auto tracked = mgr.get_tracked_goal(sent.goal_id);
  ASSERT_TRUE(tracked.has_value());
  EXPECT_EQ(tracked->status, ActionGoalStatus::CANCELING);
}

TEST(OperationManagerRoutingTest, CancelActionGoalRejectsInvalidUuid) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);
  auto out = mgr.cancel_action_goal("/p/a", "not_a_uuid");
  EXPECT_FALSE(out.success);
  EXPECT_NE(out.error_message.find("Invalid goal_id format"), std::string::npos);
  EXPECT_EQ(act->cancel_calls_, 0);
}

TEST(OperationManagerRoutingTest, GetActionResultRoutesAndUpdatesTracking) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/5);
  auto sent = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(sent.success);

  act->get_status_ = ActionGoalStatus::SUCCEEDED;
  act->get_payload_ = json{{"answer", 42}};
  auto out = mgr.get_action_result("/p/a", "example_interfaces/action/Fibonacci", sent.goal_id);
  EXPECT_TRUE(out.success);
  EXPECT_EQ(out.status, ActionGoalStatus::SUCCEEDED);
  EXPECT_EQ(out.result["answer"], 42);
  EXPECT_EQ(act->get_calls_, 1);
  EXPECT_EQ(act->last_get_path_, "/p/a");
  EXPECT_EQ(act->last_get_goal_id_, sent.goal_id);
  EXPECT_DOUBLE_EQ(act->last_get_timeout_, 5.0);

  auto tracked = mgr.get_tracked_goal(sent.goal_id);
  ASSERT_TRUE(tracked.has_value());
  EXPECT_EQ(tracked->status, ActionGoalStatus::SUCCEEDED);
}

TEST(OperationManagerRoutingTest, CleanupOldGoalsRemovesCompletedAndUnsubscribes) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);
  auto sent = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(sent.success);

  // Mark the goal as terminal.
  mgr.update_goal_status(sent.goal_id, ActionGoalStatus::SUCCEEDED);

  // The cleanup compares `now - last_update > max_age`, so the negative max_age
  // forces the comparison to succeed without sleeping.
  mgr.cleanup_old_goals(std::chrono::seconds(-1));

  EXPECT_TRUE(mgr.list_tracked_goals().empty());
  ASSERT_EQ(act->unsubscribed_paths_.size(), 1u);
  EXPECT_EQ(act->unsubscribed_paths_[0], "/p/a");
}

TEST(OperationManagerRoutingTest, ShutdownClearsTrackedGoals) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);
  auto sent = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(sent.success);
  ASSERT_FALSE(mgr.list_tracked_goals().empty());

  mgr.shutdown();
  EXPECT_TRUE(mgr.list_tracked_goals().empty());
  // Shutdown asks the transport to drop the subscription.
  EXPECT_FALSE(act->unsubscribed_paths_.empty());
  // Idempotency: a second shutdown is a no-op.
  EXPECT_NO_THROW(mgr.shutdown());
}

TEST(OperationManagerRoutingTest, StaticUuidValidatorWorksWithoutRos) {
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcdef"));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0123"));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcdez"));
}

TEST(OperationManagerRoutingTest, StaticTypeValidatorsWorkWithoutRos) {
  EXPECT_TRUE(OperationManager::is_valid_message_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_service_type("std_srvs/srv/Trigger"));
  EXPECT_FALSE(OperationManager::is_action_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_action_type("example_interfaces/action/Fibonacci"));
  EXPECT_FALSE(OperationManager::is_service_type("example_interfaces/action/Fibonacci"));
}

TEST(OperationManagerRoutingTest, GetLatestGoalForActionPicksMostRecent) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);

  act->send_goal_id_ = "00000000000000000000000000000001";
  auto first = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(first.success);
  // Sleep briefly so created_at differs - chrono::system_clock::now() is
  // monotonic enough at millisecond granularity for this assertion.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  act->send_goal_id_ = "00000000000000000000000000000002";
  auto second = mgr.send_action_goal("/p/a", "example_interfaces/action/Fibonacci", json{}, "");
  ASSERT_TRUE(second.success);

  auto latest = mgr.get_latest_goal_for_action("/p/a");
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->goal_id, second.goal_id);
}

// The unsubscribe veto only helps if it holds for the whole operation. The
// manager releases subscriptions_mutex_ before calling the transport, so a
// goal sent into that gap re-flags the path (a no-op at the transport, whose
// subscription still exists) and then has that subscription destroyed by the
// unsubscribe already in flight. End state: goal tracked, manager believes the
// path is subscribed, no status stream - and nothing repairs it, because
// subscribe_to_action_status short-circuits on the flag. That is precisely the
// state the cancel-timeout reconciliation cannot survive.
TEST(OperationManagerRoutingTest, UnsubscribeKeepsTheStreamForAGoalThatArrivesDuringTheTransportCall) {
  auto svc = std::make_shared<MockServiceTransport>();
  auto act = std::make_shared<MockActionTransport>();
  OperationManager mgr(svc, act, nullptr, /*timeout=*/2);
  const std::string path = "/p/a";
  const std::string late_goal = "00000000000000000000000000000009";

  // A path that was subscribed and whose goals have all just been cleaned up.
  mgr.subscribe_to_action_status(path);
  ASSERT_TRUE(mgr.get_goals_for_action(path).empty());

  act->reentry_ = [&mgr, &path, &late_goal]() {
    ActionGoalInfo info;
    info.goal_id = late_goal;
    info.action_path = path;
    info.action_type = "example_interfaces/action/Fibonacci";
    info.entity_id = "engine";
    info.status = ActionGoalStatus::EXECUTING;
    info.created_at = std::chrono::system_clock::now();
    info.last_update = info.created_at;
    mgr.inject_tracked_goal_for_testing(std::move(info));
    mgr.subscribe_to_action_status(path);
  };

  mgr.unsubscribe_from_action_status(path);

  ASSERT_FALSE(mgr.get_goals_for_action(path).empty()) << "the late goal must still be tracked";
  EXPECT_EQ(act->callbacks_.count(path), 1u) << "a tracked goal lost its status stream";

  // The stream must still reach the tracking map - a live callback that is
  // never invoked would be an equally dead stream.
  act->fire_status(path, late_goal, ActionGoalStatus::CANCELED);
  auto tracked = mgr.get_tracked_goal(late_goal);
  ASSERT_TRUE(tracked.has_value());
  EXPECT_EQ(tracked->status, ActionGoalStatus::CANCELED)
      << "status frames no longer reach a goal the manager believes is subscribed";
}

}  // namespace ros2_medkit_gateway
