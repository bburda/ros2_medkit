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
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
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

// Same shape, but with App::id decoupled from the node it is bound to - the re-bind
// cases below move `fqn` between snapshots while `id` stays put.
ros2_medkit_gateway::IntrospectionInput managed_app_snapshot(const std::string & id, const std::string & fqn) {
  auto in = managed_node_snapshot(fqn);
  in.apps.front().id = id;
  return in;
}

// Same App::id and same fqn, but the lifecycle services live somewhere else - a
// `~/get_state` remap in the launch file, or a manifest-bound app re-pointed at another
// node's services. The ~/transition_event topic is DERIVED from that path, so this is a
// different node behind an unchanged name: the half of the binding identity that no other
// test moves on its own.
ros2_medkit_gateway::IntrospectionInput remapped_service_snapshot(const std::string & fqn,
                                                                  const std::string & service_prefix) {
  auto in = managed_node_snapshot(service_prefix);
  in.apps.front().id = fqn;
  in.apps.front().bound_fqn = fqn;
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

constexpr auto kPumpBudget = std::chrono::milliseconds(100);

// Publish `msg` and pump the watcher until `id`'s cached label becomes `label`, or the
// retry budget runs out. Two reasons this is a retry loop and not a single publish:
// a volatile publisher only delivers to a subscription that has finished DDS endpoint
// matching, and the watcher's subscriptions are in a callback group NO node-wide executor
// collects - pump_events() is the only thing that runs them, exactly as the plugin's tick
// thread does in production.
// Cancels the executor and joins its spin thread on every exit path, including the one a
// fatal gtest assertion takes.
class ExecutorJoin {
 public:
  ExecutorJoin(rclcpp::executors::SingleThreadedExecutor & exec, std::thread & spin) : exec_(exec), spin_(spin) {
  }
  ExecutorJoin(const ExecutorJoin &) = delete;
  ExecutorJoin & operator=(const ExecutorJoin &) = delete;
  ExecutorJoin(ExecutorJoin &&) = delete;
  ExecutorJoin & operator=(ExecutorJoin &&) = delete;
  ~ExecutorJoin() {
    exec_.cancel();
    if (spin_.joinable()) {
      spin_.join();
    }
  }

 private:
  rclcpp::executors::SingleThreadedExecutor & exec_;
  std::thread & spin_;
};

bool publish_until_label(ros2_medkit_graph_watchdog::LifecycleWatcher & w,
                         const rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr & pub,
                         const lifecycle_msgs::msg::TransitionEvent & msg, const std::string & id,
                         const std::string & label) {
  for (int i = 0; i < 100; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    w.pump_events(kPumpBudget);
    if (w.state_of(id).value_or("") == label) {
      return true;
    }
  }
  return false;
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

  // Drive an "inactive" transition.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  const bool flipped = publish_until_label(w, pub, msg, id, "inactive");

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

  // errorprocessing -> finalized is the ON_ERROR_FAILURE / ON_ERROR_ERROR edge.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "errorprocessing";
  msg.goal_state.label = "finalized";
  ASSERT_TRUE(publish_until_label(w, pub, msg, id, "finalized"));

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

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "shuttingdown";
  msg.goal_state.label = "finalized";
  ASSERT_TRUE(publish_until_label(w, pub, msg, id, "finalized"));

  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  const auto departed = w.departed_state_of(id);
  ASSERT_TRUE(departed.has_value());
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_FALSE(departed->error_terminated);
}

// The structural property the private callback group exists for: an executor that has the
// gateway node added must never collect these subscriptions. If it did, rclcpp's
// AnyExecutable (a local of the executor thread, holding a STRONG ref to the subscription
// it dispatches) would routinely release the last reference on an executor thread, running
// ~Subscription - which mutates the node's rcl entity registry - concurrently with the
// tick thread's create_subscription on the same node. No lock in this class can reach an
// executor thread, so keeping the subscriptions out of every gateway executor is the fix,
// and this test is what fails if the callback-group wiring is ever dropped.
TEST_F(LifecycleWatcherTest, NodeWideExecutorNeverRunsTheLifecycleCallbacks) {
  const std::string id = "/detached_group";
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

  // Wait for endpoint matching first: without it, "no callback ran" would prove nothing
  // (the messages would simply never have reached the subscription).
  bool matched = false;
  for (int i = 0; i < 100 && !matched; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    matched = pub->get_subscription_count() > 0;
  }
  ASSERT_TRUE(matched) << "the watcher's subscription never matched; the rest is vacuous";

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  for (int i = 0; i < 5; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(w.state_of(id).value_or("<untracked>"), "")
      << "an executor spinning the gateway node executed a lifecycle callback: the subscription is back "
         "in the gateway's executor, where an executor thread can run ~Subscription against a concurrent "
         "create_subscription";

  exec.cancel();
  spin.join();

  // The messages WERE delivered - KeepLast(10) is holding them - so the watcher's own
  // pump must now produce them without a single further publish. That is what separates
  // "the callbacks are correctly detached" from "delivery is simply broken".
  bool arrived = false;
  for (int i = 0; i < 40 && !arrived; ++i) {
    w.pump_events(kPumpBudget);
    arrived = (w.state_of(id).value_or("") == "inactive");
    if (!arrived) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  EXPECT_TRUE(arrived) << "pump_events() must deliver the events the node-wide executor left queued";
}

// Steady state is not enough: in production nodes appear mid-run, so a subscription is
// routinely added to a private executor that has ALREADY been pumped and has a built
// entity collection. If that addition does not make the executor re-collect, the second
// node's transitions are silently never delivered while the first node's keep working -
// a failure mode no single-node test can see.
TEST_F(LifecycleWatcherTest, NodeDiscoveredAfterTheFirstPumpStillGetsItsEvents) {
  const std::string first = "/lc_first";
  const std::string late = "/lc_late";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  auto first_pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      first + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  w.update(managed_node_snapshot(first), /*tick=*/1);
  lifecycle_msgs::msg::TransitionEvent inactive;
  inactive.goal_state.label = "inactive";
  ASSERT_TRUE(publish_until_label(w, first_pub, inactive, first, "inactive"))
      << "the first node's events must arrive before this test can say anything about a later one";

  // A second managed node shows up on a later tick, long after the executor was primed.
  auto late_pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      late + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  w.update(managed_nodes_snapshot({first, late}), /*tick=*/2);
  ASSERT_TRUE(w.state_of(late).has_value()) << late << " must be tracked after the second update";

  lifecycle_msgs::msg::TransitionEvent active;
  active.goal_state.label = "active";
  EXPECT_TRUE(publish_until_label(w, late_pub, active, late, "active"))
      << "a subscription added to an already-pumped private executor never got collected";
  EXPECT_EQ(w.state_of(first).value_or(""), "inactive") << "the first node's cached label must survive";
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

// The sharp form of the claim above, and the one the whole presence-ownership bound rests on:
// not "some job kept its attempts" but "exactly the read that RAN was charged". Two managed
// nodes, neither answering: the first read overruns the 150ms per-tick budget on its own
// (~500ms reader timeout), the loop breaks, and the second node is left queued and unread. Its
// budget must be untouched, not merely non-zero. An aggregate over eight nodes cannot say that
// - a loop that charged half its queue would still leave the total above the floor - and an
// implementation that charged every SELECTED job would drain a node that was never asked
// anything, which is precisely the fact the bound reads as "we asked and failed".
TEST_F(LifecycleWatcherTest, OnlyTheJobWhoseReadRanIsChargedWhenTheTickBudgetCutsTheQueue) {
  const std::vector<std::string> ids{"/q0", "/q1"};
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(managed_nodes_snapshot(ids), /*tick=*/1);  // both new: subscribed, full budget
  for (const auto & id : ids) {
    ASSERT_EQ(w.reseeds_remaining_for_test(id), ros2_medkit_graph_watchdog::LifecycleWatcher::kReseedAttempts)
        << id << " must start with the full self-heal budget";
  }

  w.update(managed_nodes_snapshot(ids), /*tick=*/2);  // both selected; only one read fits
  const int spent0 =
      ros2_medkit_graph_watchdog::LifecycleWatcher::kReseedAttempts - w.reseeds_remaining_for_test("/q0");
  const int spent1 =
      ros2_medkit_graph_watchdog::LifecycleWatcher::kReseedAttempts - w.reseeds_remaining_for_test("/q1");
  EXPECT_EQ(spent0 + spent1, 1) << "exactly one GetState can have run inside the per-tick blocking "
                                   "budget, so exactly one attempt may have been charged - "
                                   "/q0 spent "
                                << spent0 << ", /q1 spent " << spent1;
}

// What the departed-lifecycle record - the only input LifecycleShutdownSuppressor has - is
// actually keyed on. Not the node being gone, and not App::is_online: an entry departs when its
// GetState path stops appearing in App::services, which is what update() builds current_paths
// from. That distinction decides whether a clean shutdown can be suppressed at all, and the
// shape it matters for is a manifest-bound app: the App survives its node, keeping its id and
// fqn, and only its runtime-derived collections empty out.
//
// So the manifest shape is driven here directly - the app stays in the snapshot, goes offline,
// and loses its services - and the record must be written all the same. ManifestParser::parse_app
// reads no services/topics/actions at all, so a manifest app's services can only ever come from
// a live runtime match (RuntimeLinker::enrich_app), which is exactly what stops existing the
// tick the node dies.
TEST_F(LifecycleWatcherTest, DepartureIsRecordedWhenTheServicesGoEvenWhileTheAppRemains) {
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  auto managed = managed_node_snapshot("/mf_app");
  managed.apps.front().is_online = true;
  w.update(managed, /*tick=*/1);
  w.set_state_for_test("/mf_app", "shuttingdown");
  ASSERT_TRUE(w.state_of("/mf_app").has_value()) << "the app must be tracked before it departs";
  ASSERT_FALSE(w.departed_state_of("/mf_app").has_value())
      << "nothing has departed yet, so a record here would make the assertion below vacuous";

  // The node dies. The App itself survives (a manifest declares it), so the snapshot still
  // carries the id and the fqn - only is_online and the runtime-derived services go.
  ros2_medkit_gateway::IntrospectionInput after;
  ros2_medkit_gateway::App app;
  app.id = "/mf_app";
  app.bound_fqn = "/mf_app";
  app.is_online = false;
  after.apps.push_back(app);
  w.update(after, /*tick=*/2);

  EXPECT_FALSE(w.state_of("/mf_app").has_value())
      << "an app with no GetState service left is not a managed lifecycle node any more";
  const auto departed = w.departed_state_of("/mf_app");
  ASSERT_TRUE(departed.has_value())
      << "no departure was recorded for an app that stopped advertising its lifecycle services, "
         "so LifecycleShutdownSuppressor would have nothing to read and a clean shutdown an "
         "operator opted to suppress would be reported as a death";
  EXPECT_EQ(departed->label, "shuttingdown");
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

// A tracked id whose BINDING moves (same App::id, different fqn / get_state path) is a
// different node wearing the same name: the old node's label and ~/transition_event
// subscription must be dropped and the id re-seeded through the new binding's GetState.
// The old label here is a KNOWN "inactive" with the self-heal budget already spent, so
// nothing else can heal it - an entry kept across the re-bind would keep gating the id
// on a node that is no longer behind it.
TEST_F(LifecycleWatcherTest, RebindToALiveNodeDropsTheOldLabelAndSeedsTheNewBinding) {
  const std::string id = "/appk";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // The OLD binding: no GetState service (seeds fail), label driven over a live
  // ~/transition_event. The NEW binding: a GetState service that answers "active".
  auto old_pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      "/rb_old/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  auto new_srv = node_->create_service<lifecycle_msgs::srv::GetState>(
      "/rb_new/get_state", [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*req*/,
                              const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & resp) {
        resp->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        resp->current_state.label = "active";
      });

  w.update(managed_app_snapshot(id, "/rb_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  const int fresh_budget = w.reseeds_remaining_for_test(id);
  ASSERT_GT(fresh_budget, 0);

  // The NEW binding's GetState service lives on node_, so node_ has to be spun for the
  // watcher's reader to get an answer out of it. That executor never runs the watcher's
  // own subscriptions - they sit in a callback group it does not collect - so the
  // transition events below still go through pump_events(), exactly as in production.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });
  // A fatal assertion below returns straight out of the test body, and a joinable
  // std::thread destroyed that way calls std::terminate - which aborts the whole binary
  // and takes every remaining case in this file with it, hiding the failure that started
  // it. The guard makes the join happen on that path too.
  ExecutorJoin join_guard{exec, spin};

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  ASSERT_TRUE(publish_until_label(w, old_pub, msg, id, "inactive"));

  // Drain the self-heal budget against the OLD (serviceless) binding, so the assertion
  // below cannot be satisfied by a leftover re-seed happening to hit the new path - the
  // re-bind handling itself has to do the healing.
  for (std::uint64_t tick = 2; w.reseeds_remaining_for_test(id) > 0 && tick < 10; ++tick) {
    w.update(managed_app_snapshot(id, "/rb_old"), tick);
  }
  ASSERT_EQ(w.reseeds_remaining_for_test(id), 0);
  ASSERT_EQ(w.state_of(id).value_or(""), "inactive");

  // The re-bind: same id, now bound to the live "/rb_new" node.
  w.update(managed_app_snapshot(id, "/rb_new"), /*tick=*/10);
  EXPECT_EQ(w.state_of(id).value_or(""), "active")
      << "after a re-bind the id must carry the NEW binding's seeded state, not the old node's label";
  EXPECT_TRUE(w.node_ok(id)) << "the old binding's non-active label must stop gating the id";
  EXPECT_EQ(w.reseeds_remaining_for_test(id), fresh_budget)
      << "a re-bound id must be re-seeded as a NEW entry (fresh self-heal budget), not healed in place";

  // The old subscription must be gone with the old entry: the old node's transitions
  // must no longer reach this id.
  for (int i = 0; i < 10; ++i) {
    old_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    w.pump_events(kPumpBudget);
  }
  EXPECT_EQ(w.state_of(id).value_or(""), "active")
      << "a transition published by the OLD binding after the re-bind overwrote the new binding's state";
}

// Re-bind to a binding with NO live node behind it: the id follows the new-binding
// seeding semantics - tracked with an empty (unknown, ungated) label after the failed
// GetState - and must NOT keep the old node's label.
TEST_F(LifecycleWatcherTest, RebindToADeadBindingLeavesStateUnknownNotTheOldLabel) {
  const std::string id = "/appd";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(managed_app_snapshot(id, "/rbd_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  w.set_state_for_test(id, "inactive");  // the OLD binding's observed label

  w.update(managed_app_snapshot(id, "/rbd_new"), /*tick=*/2);
  EXPECT_EQ(w.state_of(id).value_or("<untracked>"), "")
      << "a re-bind to a dead binding must leave the id unknown (empty seed), not wearing the old label";
  EXPECT_TRUE(w.node_ok(id)) << "unknown must not gate - only a KNOWN non-active label does";
}

// The guard against overcorrection: an UNCHANGED binding across updates must never be
// treated as a re-bind. The instrument is the self-heal budget, which counts what the
// remote actually pays (GetState round trips): a drop + re-create would reset it to the
// fresh value on every tick and record a phantom departure.
TEST_F(LifecycleWatcherTest, SameBindingAcrossUpdatesIsNeverTreatedAsARebind) {
  const std::string fqn = "/stable";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(managed_node_snapshot(fqn), /*tick=*/1);
  const int fresh_budget = w.reseeds_remaining_for_test(fqn);
  ASSERT_GT(fresh_budget, 0);

  // Non-active label + budget left -> exactly one charged re-seed per update.
  w.update(managed_node_snapshot(fqn), /*tick=*/2);
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), fresh_budget - 1)
      << "one update with the same binding must charge exactly one re-seed - a re-created entry "
         "would reset the budget to "
      << fresh_budget;
  w.update(managed_node_snapshot(fqn), /*tick=*/3);
  w.update(managed_node_snapshot(fqn), /*tick=*/4);
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), 0)
      << "the budget must drain monotonically across same-binding updates and stay drained";
  EXPECT_TRUE(w.state_of(fqn).has_value()) << "the entry itself must survive every same-binding update";
  EXPECT_FALSE(w.departed_state_of(fqn).has_value()) << "an unchanged binding must never be recorded as a departure";
}

// The OTHER half of the binding identity, on its own. Every re-bind test above moves the
// fqn and the get_state path together (the snapshot builder derives one from the other), so
// the path arm of the drop condition is never the thing that fires. Here only the SERVICE
// path moves - the id and the fqn are unchanged - and it is still a different node: the
// ~/transition_event subscription is derived from that path, so an entry kept across this
// would keep enforcing the old node's label and listening on the old node's topic.
TEST_F(LifecycleWatcherTest, MovingOnlyTheServicePathIsStillARebind) {
  const std::string fqn = "/remapped";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(remapped_service_snapshot(fqn, "/svc_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(fqn).has_value());
  const int fresh_budget = w.reseeds_remaining_for_test(fqn);
  ASSERT_GT(fresh_budget, 0);
  w.set_state_for_test(fqn, "inactive");  // the OLD binding's observed label

  // Same id, same fqn - only the services moved.
  w.update(remapped_service_snapshot(fqn, "/svc_new"), /*tick=*/2);
  EXPECT_EQ(w.state_of(fqn).value_or("<untracked>"), "")
      << "a node whose lifecycle services moved kept the old binding's label - the get_state path is half "
         "the binding identity, and the transition_event topic is derived from it";
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), fresh_budget)
      << "the entry must be re-created (fresh self-heal budget), not healed in place";
}

// What a re-bind means for departed_state_of(): the OLD binding effectively departed, so
// its last observed state is recorded under ITS fqn - same record shape and same
// retention mechanics as a node vanishing from the snapshot.
TEST_F(LifecycleWatcherTest, RebindRecordsTheOldBindingAsDepartedUnderItsOwnFqn) {
  const std::string id = "/appr";
  constexpr int kRetentionTicks = 3;
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_, kRetentionTicks);

  w.update(managed_app_snapshot(id, "/rbr_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  w.set_state_for_test(id, "inactive");

  w.update(managed_app_snapshot(id, "/rbr_new"), /*tick=*/5);
  const auto departed = w.departed_state_of("/rbr_old");
  ASSERT_TRUE(departed.has_value()) << "a re-bind must record the OLD binding as departed under its fqn";
  EXPECT_EQ(departed->label, "inactive");
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_FALSE(departed->error_terminated);
  EXPECT_FALSE(w.departed_state_of("/rbr_new").has_value())
      << "the NEW binding is present, not departed - only the old one may be recorded";

  // The record ages out through the ordinary retention prune.
  w.update(managed_app_snapshot(id, "/rbr_new"), /*tick=*/5 + kRetentionTicks + 1);
  EXPECT_FALSE(w.departed_state_of("/rbr_old").has_value());
}
