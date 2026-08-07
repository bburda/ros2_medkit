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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/graph_watchdog_plugin.hpp"

// The plugin's extern "C" exports (graph_watchdog_plugin_exports.cpp) are compiled
// directly into this test binary (see CMakeLists.txt); forward-declare their
// prototypes here since there is no installed header for out-of-process dlopen callers.
extern "C" int plugin_api_version();
extern "C" ros2_medkit_gateway::GatewayPlugin * create_plugin();
extern "C" ros2_medkit_gateway::IntrospectionProvider *
get_introspection_provider(ros2_medkit_gateway::GatewayPlugin * plugin);

namespace {

// Test-only detectors proving the fan-out loop: fan-out over multiple detectors,
// per-detector try/catch isolation, and Off-mode skip. Not shipped. Self-register
// via REGISTER_DETECTOR, so every test in this binary that calls
// DetectorRegistry::instance().create_all() (i.e. every GraphWatchdogPlugin
// instance built in this file) picks them up.
std::atomic<int> g_counting_ticks{0};
std::atomic<int> g_off_ticks{0};

class CountingDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "counting";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
    g_counting_ticks.fetch_add(1);
  }
};

class ThrowingDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "throwing";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
    throw std::runtime_error("boom");
  }
};

class OffDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "offd";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
    g_off_ticks.fetch_add(1);
  }
};

// Records the exact config the plugin hands to configure(), so the injection rules for
// plugin-scope keys can be asserted on the detector side rather than inferred.
std::mutex g_captured_mutex;
nlohmann::json g_captured_config;

class ConfigCapturingDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "capturing";
  }
  void configure(const nlohmann::json & config) override {
    std::lock_guard<std::mutex> lk(g_captured_mutex);
    g_captured_config = config;
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
  }
};

// Raises one fault per tick through the PLUGIN's own fault client, so a test can observe
// what actually leaves the process and what the client is left holding afterwards. Harmless
// in every other test in this binary: raise_fault() bails on !service_is_ready() when no
// ReportFault server exists, which is the case everywhere except the test that starts one.
class RaisingDetector : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "raising";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & ctx) override {
    ctx.raise_fault(ros2_medkit_graph_watchdog::graph_fault_codes::kOrphan, 2, "fault client probe", "/watched_app");
  }
};

REGISTER_DETECTOR(CountingDetector, "counting")
REGISTER_DETECTOR(RaisingDetector, "raising")
REGISTER_DETECTOR(ThrowingDetector, "throwing")
REGISTER_DETECTOR(OffDetector, "offd")
REGISTER_DETECTOR(ConfigCapturingDetector, "capturing")

}  // namespace

namespace {
// In-memory sink standing in for the real httplib::Response the gateway would pass.
struct TestResponseSink {
  int status = 0;
  nlohmann::json body;
};
}  // namespace

// Minimal stand-in bodies for PluginRequest/PluginResponse: the real implementations
// live in ros2_medkit_gateway's gateway_core, which this test binary does not link (see
// the GATEWAY_SRC_DIR comment in CMakeLists.txt - only headers are exported by the
// installed package). The x-medkit-watchdog route handler under test only calls
// res.send_json()/res.send_error(), so a tiny in-memory sink is enough - no real HTTP
// server needed (contrast with graph_provider's route test, which spins up httplib for
// an end-to-end check; not needed here since the handler ignores the request entirely).
namespace ros2_medkit_gateway {
PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & data) {
  auto * sink = static_cast<TestResponseSink *>(impl_);
  sink->status = 200;
  sink->body = data;
}
void PluginResponse::send_error(int status, const std::string & /*error_code*/, const std::string & message,
                                const nlohmann::json & /*parameters*/) {
  auto * sink = static_cast<TestResponseSink *>(impl_);
  sink->status = status;
  sink->body = {{"error", message}};
}
}  // namespace ros2_medkit_gateway

// Fake context. Copy the pure-virtual overrides from the gateway's own
// BareContext in ros2_medkit_gateway/test/test_entity_change_scope.cpp
// (which now derives from RosPluginContext), then point node() at test_node_.
// Only node() is exercised here; the rest throw / return empty.
class FakeContext : public ros2_medkit_gateway::RosPluginContext {
 public:
  explicit FakeContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo> get_entity(const std::string &) const override {
    return std::nullopt;
  }
  std::vector<ros2_medkit_gateway::PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string &) const override {
    return nlohmann::json::array();
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo>
  validate_entity_for_route(const ros2_medkit_gateway::PluginRequest &, ros2_medkit_gateway::PluginResponse &,
                            const std::string &) const override {
    return std::nullopt;
  }
  void register_capability(ros2_medkit_gateway::SovdEntityType, const std::string &) override {
  }
  void register_entity_capability(const std::string &, const std::string &) override {
  }
  std::vector<std::string> get_type_capabilities(ros2_medkit_gateway::SovdEntityType) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string &) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string &, const std::string &,
                                                   const std::string &) const override {
    return {};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string &, const std::string &, const std::vector<std::string> &, int) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string &, const std::string &) override {
    return {};
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }

 private:
  rclcpp::Node * node_;
};

// FakeContext variant that always reports one app in the entity snapshot, so the
// ReliabilityGate the plugin owns has something real to warm up and arm.
class FakeContextWithApp : public FakeContext {
 public:
  using FakeContext::FakeContext;
  ros2_medkit_gateway::IntrospectionInput get_entity_snapshot() const override {
    ros2_medkit_gateway::IntrospectionInput input;
    ros2_medkit_gateway::App app;
    app.id = "/watched_app";
    input.apps.push_back(app);
    return input;
  }
};

// Reports one MANAGED lifecycle node: GetState + ChangeState services, the exact shape
// LifecycleWatcher keys on. Nothing answers that GetState, so the blocking seed times out
// to an empty label and a live ~/transition_event is the only thing that can ever set it.
class FakeContextWithManagedApp : public FakeContext {
 public:
  using FakeContext::FakeContext;
  ros2_medkit_gateway::IntrospectionInput get_entity_snapshot() const override {
    ros2_medkit_gateway::ServiceInfo get_state;
    get_state.full_path = "/managed_app/get_state";
    get_state.type = "lifecycle_msgs/srv/GetState";
    ros2_medkit_gateway::ServiceInfo change_state;
    change_state.full_path = "/managed_app/change_state";
    change_state.type = "lifecycle_msgs/srv/ChangeState";

    ros2_medkit_gateway::App app;
    app.id = "/managed_app";
    app.bound_fqn = "/managed_app";
    app.services = {get_state, change_state};

    ros2_medkit_gateway::IntrospectionInput input;
    input.apps.push_back(app);
    return input;
  }
};

// Reports a sliding window of app ids that CHANGES every snapshot, so the gate's
// WarmupTracker inserts + erases map entries each tick (not just refreshes one). This
// makes the concurrent-status test exercise the insert/erase-vs-range-for iterator flavor
// of the gate race, not only the scalar/field-write flavor.
class FakeContextChurningApps : public FakeContext {
 public:
  using FakeContext::FakeContext;
  ros2_medkit_gateway::IntrospectionInput get_entity_snapshot() const override {
    const int n = counter_.fetch_add(1);
    ros2_medkit_gateway::IntrospectionInput input;
    for (int k = 0; k < 3; ++k) {
      ros2_medkit_gateway::App app;
      app.id = "/churn_" + std::to_string((n + k) % 8);  // sliding window -> one id leaves + one enters
      input.apps.push_back(app);
    }
    return input;
  }

 private:
  mutable std::atomic<int> counter_{0};
};

class GraphWatchdogPluginTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    gateway_node_ = std::make_shared<rclcpp::Node>("fake_gateway_node");
  }
  void TearDown() override {
    gateway_node_.reset();
  }
  rclcpp::Node::SharedPtr gateway_node_;
};

// prune_grace is documented in each prune-aware detector's own key table, and every one of
// them reads it off the config it is handed. Injecting the plugin-scope value unconditionally
// meant `detectors.<id>.prune_grace` was copied in by extract_detector_config() and then
// silently overwritten, with no warning - the operator set a key that could never take effect.
TEST_F(GraphWatchdogPluginTest, PerDetectorPruneGraceWinsOverThePluginScopeDefault) {
  {
    std::lock_guard<std::mutex> lk(g_captured_mutex);
    g_captured_config = nlohmann::json::object();
  }
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());
  plugin.configure(nlohmann::json{
      {"tick_interval_ms", 50}, {"prune_grace", 60}, {"detectors", {{"capturing", {{"prune_grace", 7}}}}}});
  plugin.set_context(ctx);
  plugin.shutdown();

  std::lock_guard<std::mutex> lk(g_captured_mutex);
  ASSERT_TRUE(g_captured_config.contains("prune_grace"));
  EXPECT_EQ(g_captured_config["prune_grace"].get<int>(), 7)
      << "detectors.capturing.prune_grace must reach the detector, not the plugin-scope 60";
}

TEST_F(GraphWatchdogPluginTest, PluginScopePruneGraceStillAppliesWhenTheDetectorSetsNone) {
  {
    std::lock_guard<std::mutex> lk(g_captured_mutex);
    g_captured_config = nlohmann::json::object();
  }
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());
  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}, {"prune_grace", 42}});
  plugin.set_context(ctx);
  plugin.shutdown();

  std::lock_guard<std::mutex> lk(g_captured_mutex);
  ASSERT_TRUE(g_captured_config.contains("prune_grace"));
  EXPECT_EQ(g_captured_config["prune_grace"].get<int>(), 42) << "the plugin-scope value is still the default";
}

TEST_F(GraphWatchdogPluginTest, ExportsAdvertiseCorrectApiVersion) {
  EXPECT_EQ(plugin_api_version(), ros2_medkit_gateway::PLUGIN_API_VERSION);
}

TEST_F(GraphWatchdogPluginTest, FactoryCreatesNamedPlugin) {
  std::unique_ptr<ros2_medkit_gateway::GatewayPlugin> plugin(create_plugin());
  ASSERT_NE(plugin, nullptr);
  EXPECT_EQ(plugin->name(), "graph_watchdog");
}

TEST_F(GraphWatchdogPluginTest, LifecycleRunsTickAndShutsDownCleanly) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());

  // The gateway would spin its own node; the test plays that role so the
  // plugin's wall timer (created on the gateway node) actually fires.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}});
  plugin.set_context(ctx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (plugin.tick_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_GT(plugin.tick_count(), 0u);

  EXPECT_NO_THROW(plugin.shutdown());
  EXPECT_NO_THROW(plugin.shutdown());  // idempotent

  exec.cancel();
  spin.join();
}

TEST_F(GraphWatchdogPluginTest, FansOutTicksSkipsOffModeAndIsolatesThrowingDetector) {
  g_counting_ticks.store(0);
  g_off_ticks.store(0);

  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}, {"detectors", {{"offd", {{"mode", "off"}}}}}});
  plugin.set_context(ctx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (g_counting_ticks.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Fan-out runs: the counting detector ticked.
  EXPECT_GT(g_counting_ticks.load(), 0);
  // Off-mode detector is skipped entirely, never built into the active set.
  EXPECT_EQ(g_off_ticks.load(), 0);
  // The throwing detector fires on every tick alongside the others; the loop's
  // per-detector try/catch isolates it, so the plugin keeps ticking without
  // crashing the process.
  EXPECT_GT(plugin.tick_count(), 0u);

  EXPECT_NO_THROW(plugin.shutdown());

  exec.cancel();
  spin.join();
}

namespace {
std::vector<ros2_medkit_gateway::GatewayPlugin::PluginRoute>::const_iterator
find_watchdog_route(const std::vector<ros2_medkit_gateway::GatewayPlugin::PluginRoute> & routes) {
  return std::find_if(routes.begin(), routes.end(), [](const ros2_medkit_gateway::GatewayPlugin::PluginRoute & r) {
    return r.method == "GET" && r.pattern.find("x-medkit-watchdog") != std::string::npos;
  });
}
}  // namespace

TEST_F(GraphWatchdogPluginTest, ExposesWatchdogStatusRoute) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());

  // Spin the gateway node so the tick timer actually fires + the gate updates,
  // same pattern as LifecycleRunsTickAndShutsDownCleanly above.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}, {"warmup_cycles", 2}});
  plugin.set_context(ctx);

  const auto routes = plugin.get_routes();
  const auto route_it = find_watchdog_route(routes);
  ASSERT_NE(route_it, routes.end()) << "expected a GET x-medkit-watchdog route";

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (plugin.tick_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(plugin.tick_count(), 0u);

  // Drive the handler directly (FakeContext reports an empty entity snapshot, so this
  // exercises the empty-entities-array shape; WatchdogStatusRouteReportsEntityLifecycle
  // AndArmedState below exercises a populated snapshot).
  TestResponseSink sink;
  ros2_medkit_gateway::PluginRequest req(nullptr);
  ros2_medkit_gateway::PluginResponse res(&sink);
  route_it->handler(req, res);

  EXPECT_EQ(sink.status, 200);
  ASSERT_TRUE(sink.body.contains("x-medkit-watchdog"));
  const auto & doc = sink.body["x-medkit-watchdog"];
  EXPECT_TRUE(doc.contains("schema_version"));
  EXPECT_TRUE(doc.contains("warmup_cycles"));
  EXPECT_TRUE(doc.contains("global_state"));
  ASSERT_TRUE(doc.contains("entities"));
  EXPECT_TRUE(doc["entities"].is_array());

  EXPECT_NO_THROW(plugin.shutdown());
  exec.cancel();
  spin.join();
}

TEST_F(GraphWatchdogPluginTest, WatchdogStatusRouteReportsEntityLifecycleAndArmedState) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContextWithApp ctx(gateway_node_.get());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  // warmup_cycles=1 with a fast tick: "/watched_app" (constant across every snapshot)
  // should be armed well within the poll window below.
  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}, {"warmup_cycles", 1}});
  plugin.set_context(ctx);

  const auto routes = plugin.get_routes();
  const auto route_it = find_watchdog_route(routes);
  ASSERT_NE(route_it, routes.end());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (plugin.tick_count() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GE(plugin.tick_count(), 3u);

  TestResponseSink sink;
  ros2_medkit_gateway::PluginRequest req(nullptr);
  ros2_medkit_gateway::PluginResponse res(&sink);
  route_it->handler(req, res);

  ASSERT_TRUE(sink.body.contains("x-medkit-watchdog"));
  const auto & doc = sink.body["x-medkit-watchdog"];
  ASSERT_TRUE(doc["entities"].is_array());
  ASSERT_FALSE(doc["entities"].empty());
  const auto & entity = doc["entities"][0];
  EXPECT_EQ(entity["id"], "/watched_app");
  EXPECT_EQ(entity["state"], "armed");
  EXPECT_TRUE(entity["armed"].get<bool>());
  EXPECT_TRUE(entity.contains("lifecycle"));  // untracked (non-lifecycle) node -> null, but key present

  EXPECT_NO_THROW(plugin.shutdown());
  exec.cancel();
  spin.join();
}

// The plugin's own fault client, end to end against a REAL ReportFault server. Two
// separate claims, and the test would be worth little without both:
//
//  1. Requests still leave the process now that the client is out of the node's default
//     callback group. A stub client can only show that a request was constructed; a real
//     service callback firing is what shows it was actually sent.
//  2. The responses nobody reads are still drained. raise_fault() is fire-and-forget - it
//     discards every future - but rclcpp keeps per-request state until an executor
//     processes the response. Moving the client off the gateway executor without pumping
//     it would trade the destruction race for a slow leak of one pending entry per raised
//     fault, for the life of the process.
//
// No executor spins the gateway node here, deliberately. The whole point of the fix is
// that the plugin does not depend on a gateway executor for its own client, and running
// without one is exactly what makes this test fail if the client ever drifts back into the
// node's default callback group.
TEST_F(GraphWatchdogPluginTest, FaultClientDeliversRequestsAndDrainsItsOwnResponses) {
  auto sink = std::make_shared<rclcpp::Node>("fault_sink");
  std::mutex received_mutex;
  int received = 0;
  auto srv = sink->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault",
      [&received, &received_mutex](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request>,
                                   std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> resp) {
        {
          std::lock_guard<std::mutex> lk(received_mutex);
          ++received;
        }
        resp->accepted = true;
      });

  // Only the SINK is spun. The plugin drives its own client off its own tick thread.
  rclcpp::executors::SingleThreadedExecutor sink_exec;
  sink_exec.add_node(sink);
  std::thread sink_spin([&sink_exec] {
    sink_exec.spin();
  });

  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContextWithApp ctx(gateway_node_.get());  // "/watched_app", the id RaisingDetector reports on
  plugin.configure(nlohmann::json{{"tick_interval_ms", 50}, {"warmup_cycles", 0}});
  plugin.set_context(ctx);

  // Enough raises that an undrained pending map is unmistakably larger than the handful of
  // requests that can legitimately be in flight at any instant.
  constexpr int kWantRequests = 25;
  int seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (seen < kWantRequests && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::lock_guard<std::mutex> lk(received_mutex);
    seen = received;
  }
  EXPECT_GE(seen, kWantRequests) << "the plugin's fault client never reached a real /fault_manager/report_fault "
                                    "server; requests are not leaving the process";

  // Destructive read, so it happens exactly once and last. With the drain working this is
  // the number of requests genuinely in flight right now (0 or 1 at a 50 ms cadence);
  // without it, it is every request ever sent.
  const std::size_t pending = plugin.prune_pending_fault_requests_for_test();
  EXPECT_LE(pending, 5u) << "the fault client is holding " << pending << " pending responses after " << seen
                         << " requests; nothing is draining it, so the client leaks one entry per raised fault";

  EXPECT_NO_THROW(plugin.shutdown());
  sink_exec.cancel();
  sink_spin.join();
}

// The production wiring for lifecycle events, end to end. The watcher's
// ~/transition_event subscriptions live in a callback group no gateway executor collects,
// so the plugin's own tick thread - draining them from wait_for_next_tick() - is the ONLY
// thing that can deliver a transition. A gateway-style executor spins the node throughout
// this test precisely because it must NOT be the one delivering it. If that drain is ever
// removed, weakened to once per tick with too coarse a poll, or moved off the tick thread,
// the cached label stays empty here and every lifecycle-gated suppression silently stops
// working on a real robot while the unit tests below still pass.
TEST_F(GraphWatchdogPluginTest, TickThreadDeliversRealTransitionEventsToTheGate) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContextWithManagedApp ctx(gateway_node_.get());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  auto pub = gateway_node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      "/managed_app/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  plugin.configure(nlohmann::json{{"tick_interval_ms", 200}, {"warmup_cycles", 1}});
  plugin.set_context(ctx);

  const auto routes = plugin.get_routes();
  const auto route_it = find_watchdog_route(routes);
  ASSERT_NE(route_it, routes.end());

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "active";
  msg.goal_state.label = "inactive";

  // Republish while polling: a volatile publisher only reaches a subscription that has
  // finished DDS endpoint matching, and the subscription is only created on the plugin's
  // first tick.
  std::string observed;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (observed != "inactive" && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    TestResponseSink sink;
    ros2_medkit_gateway::PluginRequest req(nullptr);
    ros2_medkit_gateway::PluginResponse res(&sink);
    route_it->handler(req, res);
    if (!sink.body.contains("x-medkit-watchdog")) {
      continue;
    }
    for (const auto & entity : sink.body["x-medkit-watchdog"]["entities"]) {
      if (entity["id"] == "/managed_app" && entity["lifecycle"].is_string()) {
        observed = entity["lifecycle"].get<std::string>();
      }
    }
  }
  EXPECT_EQ(observed, "inactive") << "the tick thread never delivered a live ~/transition_event; nothing else "
                                     "runs these subscriptions, so the gate is now blind to lifecycle state";

  EXPECT_NO_THROW(plugin.shutdown());
  exec.cancel();
  spin.join();
}

// The tick now runs on a dedicated thread that mutates the gate's warmup_ map + scalars,
// while the x-medkit-watchdog handler reads them on a different (HTTP) thread. gate_mutex_
// must make that race-free. This hammers the handler concurrently with fast ticks so CI's
// TSan job flags any missing synchronization (and it must not crash under a normal build).
TEST_F(GraphWatchdogPluginTest, StatusHandlerRaceFreeAgainstConcurrentTicks) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContextChurningApps ctx(gateway_node_.get());  // churns warmup_ (insert+erase) every tick

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  plugin.configure(nlohmann::json{{"tick_interval_ms", 5}, {"warmup_cycles", 1}});
  plugin.set_context(ctx);

  const auto routes = plugin.get_routes();
  const auto route_it = find_watchdog_route(routes);
  ASSERT_NE(route_it, routes.end());

  int calls = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    TestResponseSink sink;
    ros2_medkit_gateway::PluginRequest req(nullptr);
    ros2_medkit_gateway::PluginResponse res(&sink);
    route_it->handler(req, res);  // reads gate state concurrently with the ticking worker
    ASSERT_EQ(sink.status, 200);
    ASSERT_TRUE(sink.body.contains("x-medkit-watchdog"));
    ++calls;
  }
  EXPECT_GT(calls, 0);

  EXPECT_NO_THROW(plugin.shutdown());
  exec.cancel();
  spin.join();
}

// Every GRAPH_* fault is raised under one entity id (aggregated_fault.hpp). If the plugin
// never publishes that entity, the faults are reachable from no scoped endpoint at all -
// which is where they were before: scoped to the host Component, whose bare id the fault
// scope only accepts when `external` is set, and HostInfoProvider never sets it.
TEST_F(GraphWatchdogPluginTest, PublishesTheEntityItsFaultsAreScopedTo) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  ros2_medkit_gateway::IntrospectionInput input;
  ros2_medkit_gateway::Component host;
  host.id = "revpi5";
  input.components.push_back(host);

  const auto result = plugin.introspect(input);
  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  const auto & app = result.new_entities.apps.front();
  EXPECT_EQ(app.id, ros2_medkit_graph_watchdog::kGraphWatchdogEntityId);
  // Without external, resolve_app_source_fqn() returns effective_fqn() - empty for an App
  // with no ROS binding - and the fault is dropped from every rollup.
  EXPECT_TRUE(app.external.value_or(false));
  EXPECT_TRUE(app.is_online);
  // Attached to the host Component so the faults also roll up to /components/<host>/faults.
  EXPECT_EQ(app.component_id, "revpi5");
}

TEST_F(GraphWatchdogPluginTest, PublishesTheEntityEvenWithNoHostComponent) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  const auto result = plugin.introspect(ros2_medkit_gateway::IntrospectionInput{});
  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  EXPECT_EQ(result.new_entities.apps.front().id, ros2_medkit_graph_watchdog::kGraphWatchdogEntityId);
  EXPECT_TRUE(result.new_entities.apps.front().component_id.empty());
}

// The gateway only calls introspect() through this export. Without it the entity is never
// published and the scoping fix above is dead code.
TEST_F(GraphWatchdogPluginTest, ExportsTheIntrospectionProvider) {
  auto * plugin = create_plugin();
  ASSERT_NE(plugin, nullptr);
  auto * provider = get_introspection_provider(plugin);
  ASSERT_NE(provider, nullptr);
  EXPECT_EQ(provider->introspect(ros2_medkit_gateway::IntrospectionInput{}).new_entities.apps.size(), 1u);
  delete plugin;
}
