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
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"  // App, IntrospectionInput, ServiceInfo
#include "ros2_medkit_graph_watchdog/lifecycle_watcher.hpp"

class LifecycleWatcherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("lifecycle_watcher_test_node");
  }
  void TearDown() override {
    node_.reset();
  }
  rclcpp::Node::SharedPtr node_;
  std::mutex mtx_;
};

TEST_F(LifecycleWatcherTest, UntrackedNodeIsAlwaysOk) {
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  EXPECT_TRUE(w.node_ok("/not_a_lifecycle_node"));  // non-managed -> never gated
  EXPECT_FALSE(w.state_of("/not_a_lifecycle_node").has_value());
}

TEST_F(LifecycleWatcherTest, ActiveAllowedInactiveSuppressed) {
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  w.set_state_for_test("/nav/planner", "active");
  w.set_state_for_test("/nav/controller", "inactive");
  EXPECT_TRUE(w.node_ok("/nav/planner"));
  EXPECT_FALSE(w.node_ok("/nav/controller"));
  EXPECT_EQ(w.state_of("/nav/planner").value(), "active");
}

TEST_F(LifecycleWatcherTest, EmptyStateIsTreatedAsUnknownNotGated) {
  // A tracked node whose one-shot GetState seed failed has an empty label; since
  // transition_event is volatile and never backfills an already-active node,
  // gating on empty would suppress its faults forever. node_ok must NOT gate it.
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  w.set_state_for_test("/seed_failed", "");
  EXPECT_TRUE(w.node_ok("/seed_failed"));
}

namespace {
// Build a snapshot whose single App looks like a managed lifecycle node (GetState +
// ChangeState services), so update() discovers it, seeds it, and subscribes to its
// ~/transition_event.
ros2_medkit_gateway::IntrospectionInput managed_node_snapshot(const std::string & id) {
  ros2_medkit_gateway::ServiceInfo get_state;
  get_state.full_path = id + "/get_state";
  get_state.type = "lifecycle_msgs/srv/GetState";
  ros2_medkit_gateway::ServiceInfo change_state;
  change_state.full_path = id + "/change_state";
  change_state.type = "lifecycle_msgs/srv/ChangeState";

  ros2_medkit_gateway::App app;
  app.id = id;
  app.bound_fqn = id;  // `id` already looks like a full fqn in every caller (e.g. "/tnode")
  app.services = {get_state, change_state};

  ros2_medkit_gateway::IntrospectionInput in;
  in.apps.push_back(app);
  return in;
}

// N managed nodes in one snapshot, so a single update() queues more GetState work than the
// per-tick blocking budget can possibly run.
ros2_medkit_gateway::IntrospectionInput managed_nodes_snapshot(const std::vector<std::string> & ids) {
  ros2_medkit_gateway::IntrospectionInput in;
  for (const auto & id : ids) {
    in.apps.push_back(managed_node_snapshot(id).apps.front());
  }
  return in;
}
}  // namespace

// Exercises the REAL mechanism (not the set_state_for_test shim): update() discovers a
// managed node, derives its ~/transition_event topic, subscribes, and the callback writes
// the cached label from a live TransitionEvent - then the drop path forgets a vanished
// node. A broken topic derivation or a callback that never updates would fail here where
// the shim-based tests stay green.
TEST_F(LifecycleWatcherTest, UpdateSubscribesAndCallbackUpdatesState) {
  const std::string id = "/tnode";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // Publisher matching the subscriber QoS (reliable, volatile), created on the gateway
  // node so its own subscription receives the message.
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // No GetState service exists, so the seed times out to an empty label (ungated). The
  // subscription is what we are testing.
  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());  // now tracked
  EXPECT_TRUE(w.node_ok(id));               // empty seed -> ungated

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  // Drive an "inactive" transition. Retry because a volatile publisher only delivers to a
  // subscription that has finished endpoint matching.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  bool flipped = false;
  for (int i = 0; i < 100 && !flipped; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flipped = (w.state_of(id).value_or("") == "inactive");
  }
  exec.cancel();
  spin.join();

  EXPECT_TRUE(flipped);         // callback wrote the label from the live message
  EXPECT_FALSE(w.node_ok(id));  // known non-active -> gated

  // Drop path: the node vanishes from a later snapshot and is forgotten.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  EXPECT_FALSE(w.state_of(id).has_value());
}

// The failed-on_error death, over a live TransitionEvent rather than the injection seam:
// on_activate fails, on_error fails, the node lands in `finalized` and the process exits.
// The departure record must carry that it went through `errorprocessing`, otherwise the
// last label alone reads exactly like a lifecycle-manager shutdown.
TEST_F(LifecycleWatcherTest, DepartureThroughErrorProcessingIsRecordedAsErrorTerminated) {
  const std::string id = "/failing_driver";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  // errorprocessing -> finalized is the ON_ERROR_FAILURE / ON_ERROR_ERROR edge.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "errorprocessing";
  msg.goal_state.label = "finalized";
  bool arrived = false;
  for (int i = 0; i < 100 && !arrived; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    arrived = (w.state_of(id).value_or("") == "finalized");
  }
  exec.cancel();
  spin.join();
  ASSERT_TRUE(arrived);

  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  const auto departed = w.departed_state_of(id);
  ASSERT_TRUE(departed.has_value());
  EXPECT_EQ(departed->label, "finalized");
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_TRUE(departed->error_terminated) << "the errorprocessing edge must survive into the departure record";
}

// The ordinary shutdown, same path: active -> shuttingdown -> finalized never touches the
// error branch, so the departure stays classifiable as clean.
TEST_F(LifecycleWatcherTest, OrdinaryShutdownDepartureIsNotErrorTerminated) {
  const std::string id = "/managed_ok";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "shuttingdown";
  msg.goal_state.label = "finalized";
  bool arrived = false;
  for (int i = 0; i < 100 && !arrived; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    arrived = (w.state_of(id).value_or("") == "finalized");
  }
  exec.cancel();
  spin.join();
  ASSERT_TRUE(arrived);

  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  const auto departed = w.departed_state_of(id);
  ASSERT_TRUE(departed.has_value());
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_FALSE(departed->error_terminated);
}

// The self-heal budget must be charged for reads that HAPPENED, not for jobs that were
// merely selected. None of these nodes has a real GetState service, so every read blocks to
// the reader's timeout (~500ms) while the per-tick blocking budget is 150ms: the very first
// read overruns it and the loop breaks, leaving the rest of the queue unread. Charging at
// selection time spent an attempt on every one of them. Two ticks of that and the whole
// budget was gone with almost no reads issued - and both outcomes are permanent for the
// process, because transition_event only fires on a FUTURE transition.
TEST_F(LifecycleWatcherTest, ReseedBudgetIsNotChargedForAJobTheTickBudgetSkipped) {
  const std::vector<std::string> ids{"/m0", "/m1", "/m2", "/m3", "/m4", "/m5", "/m6", "/m7"};
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // Tick 1: all are new -> subscribed, each with a full self-heal budget, label "" (seeds
  // either timed out or never ran).
  w.update(managed_nodes_snapshot(ids), /*tick=*/1);
  int initial_total = 0;
  for (const auto & id : ids) {
    ASSERT_TRUE(w.state_of(id).has_value()) << id << " must be tracked";
    initial_total += w.reseeds_remaining_for_test(id);
  }
  ASSERT_GT(initial_total, 0) << "every newly tracked node starts with a self-heal budget";

  // Tick 2: every node is tracked and non-active, so all of them are selected as re-seed
  // jobs - but the budget only allows the first read.
  w.update(managed_nodes_snapshot(ids), /*tick=*/2);
  int remaining_total = 0;
  for (const auto & id : ids) {
    remaining_total += w.reseeds_remaining_for_test(id);
  }
  EXPECT_GT(remaining_total, initial_total - static_cast<int>(ids.size()))
      << "charging every selected job would leave " << initial_total - static_cast<int>(ids.size())
      << "; a job the per-tick budget skipped must keep its attempts";
}

// fqn capture + departed-lifecycle retention: a tracked node's stable fqn
// (App::effective_fqn(), captured at first sighting) is what departed_state_of() is
// keyed by, and its last-known label stays retrievable through that fqn for exactly
// `retention_ticks` ticks past its departure tick, then ages out.
TEST_F(LifecycleWatcherTest, DepartedNodeRetainsLastLabelByFqnThenPrunes) {
  const std::string fqn = "/managed_departed";
  constexpr int kRetentionTicks = 3;
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_, kRetentionTicks);

  // Track it (managed_node_snapshot() sets bound_fqn == id, so effective_fqn() == fqn).
  w.update(managed_node_snapshot(fqn), /*tick=*/1);
  ASSERT_TRUE(w.state_of(fqn).has_value());
  EXPECT_FALSE(w.departed_state_of(fqn).has_value())  // still present -> never "departed"
      << "a still-tracked node must never be reported by departed_state_of()";

  w.set_state_for_test(fqn, "finalized");

  // Vanishes at tick 10: recorded into recently_departed_ with departed_tick=10.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10);
  ASSERT_TRUE(w.departed_state_of(fqn).has_value());
  EXPECT_EQ(w.departed_state_of(fqn).value().label, "finalized");
  EXPECT_FALSE(w.state_of(fqn).has_value());  // no longer "currently tracked"

  // Still within retention (tick - departed_tick == retention_ticks, not yet pruned -
  // pruning only fires on strictly GREATER than retention_ticks).
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10 + kRetentionTicks);
  EXPECT_TRUE(w.departed_state_of(fqn).has_value())
      << "an entry exactly at the retention boundary must still be retrievable";

  // One tick past retention -> pruned.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10 + kRetentionTicks + 1);
  EXPECT_FALSE(w.departed_state_of(fqn).has_value());
}
