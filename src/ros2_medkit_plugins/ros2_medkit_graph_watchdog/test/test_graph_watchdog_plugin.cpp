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

// ---- The presence-view handoff: publisher, and one observer on each side of it ----------
//
// DetectorContext::presence_tracked is produced by the PLUGIN, from whichever emitting
// detector exposes tracked_departure_keys(). Every other test of that seam stages the view by
// hand, which cannot tell a working publication from a deleted one, so these three stand in for
// the two real detectors and let the plugin do the work.

/// Stands in for the presence class. Its key set GROWS inside its own tick(), which is what
/// makes "pointer into live state" and "copy taken at one point" distinguishable from the
/// outside: with a pointer, an observer that ticks before it and one that ticks after it read
/// different contents on the same sweep.
std::atomic<int> g_owner_ticks{0};

class FakePresenceOwner : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "fake_owner";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & /*ctx*/) override {
    keys_.insert("/grown_" + std::to_string(g_owner_ticks.fetch_add(1)));
  }
  const std::set<std::string> * tracked_departure_keys() const override {
    return &keys_;
  }

 private:
  std::set<std::string> keys_{"/seeded"};
};

/// What one observer saw, in sweep order: the Nth entry of two observers' logs belong to the
/// same sweep, because the plugin ticks every detector once per sweep in registry order.
struct ViewLog {
  std::mutex mutex;
  std::vector<std::optional<std::set<std::string>>> seen;
};
ViewLog g_view_before;
ViewLog g_view_after;

void record_view(ViewLog & log, const ros2_medkit_graph_watchdog::DetectorContext & ctx) {
  std::lock_guard<std::mutex> lk(log.mutex);
  if (ctx.presence_tracked == nullptr) {
    log.seen.emplace_back(std::nullopt);
  } else {
    log.seen.emplace_back(*ctx.presence_tracked);
  }
}

std::vector<std::optional<std::set<std::string>>> view_snapshot(ViewLog & log) {
  std::lock_guard<std::mutex> lk(log.mutex);
  return log.seen;
}

void reset_view_logs() {
  {
    std::lock_guard<std::mutex> lk(g_view_before.mutex);
    g_view_before.seen.clear();
  }
  std::lock_guard<std::mutex> lk(g_view_after.mutex);
  g_view_after.seen.clear();
}

class ViewObserverBefore : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "view_obs_before";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & ctx) override {
    record_view(g_view_before, ctx);
  }
};

class ViewObserverAfter : public ros2_medkit_graph_watchdog::Detector {
 public:
  std::string id() const override {
    return "view_obs_after";
  }
  void tick(ros2_medkit_graph_watchdog::DetectorContext & ctx) override {
    record_view(g_view_after, ctx);
  }
};

REGISTER_DETECTOR(CountingDetector, "counting")
REGISTER_DETECTOR(RaisingDetector, "raising")
REGISTER_DETECTOR(ThrowingDetector, "throwing")
REGISTER_DETECTOR(OffDetector, "offd")
REGISTER_DETECTOR(ConfigCapturingDetector, "capturing")
// Order matters here and nowhere else in this file: DetectorRegistry keeps its factories in a
// vector, so these three tick in exactly this order and the owner sits BETWEEN its observers.
REGISTER_DETECTOR(ViewObserverBefore, "view_obs_before")
REGISTER_DETECTOR(FakePresenceOwner, "fake_owner")
REGISTER_DETECTOR(ViewObserverAfter, "view_obs_after")

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
  std::optional<ros2_medkit_gateway::PluginEntityInfo> get_entity(const std::string & /*id*/) const override {
    return std::nullopt;
  }
  std::vector<ros2_medkit_gateway::PluginEntityInfo>
  get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo>
  validate_entity_for_route(const ros2_medkit_gateway::PluginRequest & /*req*/,
                            ros2_medkit_gateway::PluginResponse & /*res*/,
                            const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  void register_capability(ros2_medkit_gateway::SovdEntityType /*entity_type*/,
                           const std::string & /*capability_name*/) override {
  }
  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*capability_name*/) override {
  }
  std::vector<std::string> get_type_capabilities(ros2_medkit_gateway::SovdEntityType /*entity_type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                   const std::string & /*collection*/) const override {
    return {};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
               const std::vector<std::string> & /*scopes*/, int /*expiration_seconds*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string & /*entity_id*/,
                                                                  const std::string & /*client_id*/) override {
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

// Reports a permanently-present "/anchor" app (so the snapshot is never empty - an empty
// snapshot re-arms ReliabilityGate's global bringup grace, which would delay arming for a
// reason unrelated to what this context exists to drive) plus one "/victim" app on the VERY
// FIRST call only - departed on every call after that. Built for
// MalformedNodeDeathPruneGraceUsesThePluginScopeFallbackNotTheDetectorsOwnDefault, which
// needs a real death to drive real reclaim timing, not merely a config-read seam.
class FakeContextVictimDepartsAfterFirstTick : public FakeContext {
 public:
  using FakeContext::FakeContext;
  ros2_medkit_gateway::IntrospectionInput get_entity_snapshot() const override {
    ros2_medkit_gateway::IntrospectionInput input;
    ros2_medkit_gateway::App anchor;
    anchor.id = "/anchor";
    anchor.bound_fqn = "/anchor";
    anchor.is_online = true;
    input.apps.push_back(anchor);
    if (calls_.fetch_add(1) == 0) {
      ros2_medkit_gateway::App victim;
      victim.id = "/victim";
      victim.bound_fqn = "/victim";
      victim.is_online = true;
      input.apps.push_back(victim);
    }
    return input;
  }

 private:
  mutable std::atomic<int> calls_{0};
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

// compute_departed_retention_ticks() needs no ROS node at all - it reads only its own
// config_snapshot parameter plus tick_interval_ms_/prune_grace_, both left at their
// constructor defaults (1000, 60) when configure()/set_context() are never called.
TEST(ComputeDepartedRetentionTicks, OversizedNodeDeathPruneGraceIsRejectedNotTruncated) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  const int ticks = plugin.compute_departed_retention_ticks_for_test(
      nlohmann::json{{"detectors", {{"node_death", {{"prune_grace", 4294967296LL}}}}}});
  // 4294967296 (2^32) narrowed to int BEFORE a range check wraps to exactly 0, which passes
  // a bare ">= 0" check silently - node_death_detector.cpp's own configure() rejects this
  // same value and keeps its default (60), so the correct retention here is computed from
  // that same default: prune_ticks=max(60, miss_grace+1), miss_grace floored to 2 at the
  // 1000ms default tick (unaffected, since the floor only bites below ~1500ms), so
  // 60+2+1=63. A narrow-then-validate bug would instead compute from the wrapped 0:
  // max(0,3)+2+1=6.
  EXPECT_EQ(ticks, 63);
}

TEST(ComputeDepartedRetentionTicks, OversizedNodeDeathMissGraceIsRejectedNotTruncated) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  const int ticks = plugin.compute_departed_retention_ticks_for_test(
      nlohmann::json{{"detectors", {{"node_death", {{"miss_grace", 4294967296LL}}}}}});
  // Same shape as the prune_grace case above, for the other field this function reads the
  // same way. miss_grace stays at its default (2, unaffected by the floor at the 1000ms
  // default tick), prune_grace stays at the plugin's own default (60):
  // prune_ticks=max(60,3)=60, retention=60+2+1=63.
  EXPECT_EQ(ticks, 63);
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

// ---- DetectorContext::presence_tracked, as the PLUGIN actually produces it ----------------

namespace {
/// Runs a real plugin until both observers have logged at least `sweeps` ticks. Returns their
/// logs. node_death is turned OFF in every caller: it is linked into this binary, exposes a
/// view of its own, and the publication loop takes the FIRST emitting detector that has one -
/// which would make these tests depend on static-init order ACROSS translation units.
std::pair<std::vector<std::optional<std::set<std::string>>>, std::vector<std::optional<std::set<std::string>>>>
run_until_observed(rclcpp::Node::SharedPtr gateway_node, const nlohmann::json & detectors_config, std::size_t sweeps) {
  reset_view_logs();
  g_owner_ticks.store(0);

  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node.get());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node);
  std::thread spin([&exec] {
    exec.spin();
  });

  nlohmann::json config{{"tick_interval_ms", 50}, {"detectors", detectors_config}};
  config["detectors"]["node_death"] = {{"mode", "off"}};
  plugin.configure(config);
  plugin.set_context(ctx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (view_snapshot(g_view_before).size() >= sweeps && view_snapshot(g_view_after).size() >= sweeps) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  plugin.shutdown();
  exec.cancel();
  spin.join();
  return {view_snapshot(g_view_before), view_snapshot(g_view_after)};
}
}  // namespace

// The publication itself. Nothing in this test stages a view by hand, so deleting the plugin's
// publication loop - or having it hand out a view the owning detector never produced - leaves
// every observation null and fails here.
TEST_F(GraphWatchdogPluginTest, ThePluginPublishesTheOwningDetectorsKeysToEveryDetector) {
  const auto [before, after] = run_until_observed(gateway_node_, nlohmann::json::object(), 3);
  ASSERT_GE(before.size(), 3u) << "the plugin never completed enough sweeps to observe a handoff";
  ASSERT_GE(after.size(), 3u);

  EXPECT_FALSE(before.front().has_value())
      << "the very first sweep has no previous sweep to publish, so the view must be null there "
         "rather than an empty set a reader would take for 'tracked nothing'";
  ASSERT_TRUE(before[1].has_value()) << "the owner's key set never reached a detector at all - the plugin is not "
                                        "publishing what tracked_departure_keys() returns";
  EXPECT_EQ(before[1]->count("/seeded"), 1u) << "the published view is not the owner's own set";
  ASSERT_TRUE(after[1].has_value());
  EXPECT_EQ(after[1]->count("/seeded"), 1u);
}

// A SNAPSHOT, not a pointer into the owner's live set. The owner grows its set inside its own
// tick(), and these two observers sit on either side of it in the registry: with a pointer they
// read different contents on the same sweep, and which one a real detector gets would depend on
// where it happens to be registered.
TEST_F(GraphWatchdogPluginTest, EveryDetectorInASweepSeesTheSameViewWhateverItsRegistryPosition) {
  const auto [before, after] = run_until_observed(gateway_node_, nlohmann::json::object(), 5);
  ASSERT_GE(before.size(), 5u);
  ASSERT_GE(after.size(), 5u);

  const std::size_t sweeps = std::min(before.size(), after.size());
  for (std::size_t i = 0; i < sweeps; ++i) {
    ASSERT_EQ(before[i].has_value(), after[i].has_value()) << "sweep " << i
                                                           << ": one side saw a view and the other "
                                                              "did not";
    if (before[i].has_value()) {
      EXPECT_EQ(*before[i], *after[i])
          << "sweep " << i
          << ": the detector that ticks BEFORE the owner and the one that ticks AFTER it read "
             "different contents, so what a detector sees depends on where it sits in the "
             "registry - the view is a pointer into live state, not a snapshot";
    }
  }

  // And the snapshot is genuinely refreshed, not frozen: the owner adds a key per tick, so a
  // later sweep must see more than an earlier one. Without this, publishing a copy ONCE would
  // satisfy the equality above.
  ASSERT_TRUE(before[1].has_value() && before[sweeps - 1].has_value());
  EXPECT_GT(before[sweeps - 1]->size(), before[1]->size())
      << "the published view stopped tracking the owner's set after the first sweep";
}

// An Advisory owner speaks for nobody: it tracks keys it will never file a fault for, and a
// reader treating that as "somebody has this covered" would hold back its own report.
TEST_F(GraphWatchdogPluginTest, AnAdvisoryOwnersKeysAreNeverPublished) {
  const auto [before, after] = run_until_observed(gateway_node_, {{"fake_owner", {{"mode", "advisory"}}}}, 3);
  ASSERT_GE(before.size(), 3u);
  ASSERT_GE(after.size(), 3u);
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_FALSE(before[i].has_value()) << "sweep " << i
                                        << ": an Advisory detector's tracked keys were published as "
                                           "though somebody would report those departures";
  }
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_FALSE(after[i].has_value()) << "sweep " << i;
  }
  EXPECT_GT(g_owner_ticks.load(), 0) << "the Advisory detector never ticked, so this test never had a view to "
                                        "exclude in the first place";
}

// And with no owner at all the view is null rather than an empty set: "nobody is tracking
// anything" and "somebody is tracking nothing" select opposite behaviour in the reader.
TEST_F(GraphWatchdogPluginTest, WithNoPresenceDetectorAtAllTheViewIsNull) {
  const auto [before, after] = run_until_observed(gateway_node_, {{"fake_owner", {{"mode", "off"}}}}, 3);
  ASSERT_GE(before.size(), 3u);
  ASSERT_GE(after.size(), 3u);
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_FALSE(before[i].has_value()) << "sweep " << i;
  }
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_FALSE(after[i].has_value()) << "sweep " << i;
  }
  EXPECT_EQ(g_owner_ticks.load(), 0) << "an Off detector must not be built into the active set at all";
}

// Plugin-scope integer keys are checked WIDE. JSON and ROS parameters both carry 64-bit
// integers, so narrowing first lets a huge value wrap back into the accepted band and pass as
// though the operator had asked for something legal. 4294967346 is 2^32 + 50: narrowed first it
// becomes a perfectly plausible 50 ms cadence, which is why the retention window - the one
// observable that reads tick_interval_ms_ without running the tick loop - is what this asserts
// on rather than the absence of a log line.
TEST_F(GraphWatchdogPluginTest, AnOutOfRangeTickIntervalIsRejectedRatherThanWrappedIntoTheBand) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContext ctx(gateway_node_.get());
  plugin.configure(nlohmann::json{{"tick_interval_ms", 4294967346LL}});
  plugin.set_context(ctx);  // load_parameters() runs here, not in configure()
  plugin.shutdown();

  // The default 1000 ms cadence: miss_grace defaults to 2 (its floor at this tick is 2), so
  // prune_ticks = max(60, 3) = 60 and retention = 60 + 2 + 1.
  EXPECT_EQ(plugin.compute_departed_retention_ticks_for_test(nlohmann::json::object()), 63)
      << "tick_interval_ms=4294967346 was narrowed to 50 and accepted, so every window sized "
         "from the cadence is sized for a configuration nobody asked for";
}

// Both ends of the band, against the same observable. The upper endpoint (INT_MAX) is
// deliberately NOT asserted here: at any tick past 1500 ms the 3000 ms floor needs no ticks at
// all, so its retention is arithmetically identical to the default's and the check could not
// tell an applied value from an ignored one.
TEST_F(GraphWatchdogPluginTest, TheTickIntervalBandIsEnforcedAtBothEnds) {
  {  // the smallest accepted cadence, and it must genuinely take effect
    ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
    FakeContext ctx(gateway_node_.get());
    plugin.configure(nlohmann::json{{"tick_interval_ms", 1}});
    plugin.set_context(ctx);
    plugin.shutdown();
    // A 1 ms tick needs 2999 ticks to span the 3000 ms floor, so miss_grace is raised from 2 to
    // 2999: prune_ticks = max(60, 3000) = 3000, retention = 3000 + 2999 + 1.
    EXPECT_EQ(plugin.compute_departed_retention_ticks_for_test(nlohmann::json::object()), 6000)
        << "the low endpoint of the documented band was not applied";
  }
  {  // one below it, which must be refused and leave the default standing
    ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
    FakeContext ctx(gateway_node_.get());
    plugin.configure(nlohmann::json{{"tick_interval_ms", 0}});
    plugin.set_context(ctx);
    plugin.shutdown();
    EXPECT_EQ(plugin.compute_departed_retention_ticks_for_test(nlohmann::json::object()), 63)
        << "a zero cadence was accepted";
  }
  {  // and the negative that a narrowing read turns 2147483648 into
    ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
    FakeContext ctx(gateway_node_.get());
    plugin.configure(nlohmann::json{{"tick_interval_ms", 2147483648LL}});
    plugin.set_context(ctx);
    plugin.shutdown();
    EXPECT_EQ(plugin.compute_departed_retention_ticks_for_test(nlohmann::json::object()), 63)
        << "a value one past INT_MAX was not rejected on the wide integer";
  }
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

// The plugin injects its own prune_grace default only when the
// per-detector key is ABSENT (see set_context()'s configure loop), so a PRESENT but
// malformed detectors.node_death.prune_grace used to reach node_death's own configure()
// unfiltered, which falls back to ITS OWN hardcoded default (60) rather than the plugin's.
// compute_departed_retention_ticks() independently falls back to the plugin's own
// prune_grace_ for the exact same malformed value, so the two disagreed whenever an
// operator's plugin-scope prune_grace was anything other than the coincidentally-matching
// 60 - sizing the lifecycle-departed retention window for a reclaim tick node_death would
// not actually reach for roughly another 57 ticks. Proven here through REAL reclaim timing
// (an allowlisted, durably-suppressed death) rather than by inspecting either side's
// computation in isolation - either side alone can look right while still disagreeing with
// the other, which is exactly why OversizedNodeDeathPruneGraceIsRejectedNotTruncated below
// (plugin-scope prune_grace left at the coincidentally-matching default 60) could not catch
// this.
TEST_F(GraphWatchdogPluginTest, MalformedNodeDeathPruneGraceUsesThePluginScopeFallbackNotTheDetectorsOwnDefault) {
  ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
  FakeContextVictimDepartsAfterFirstTick ctx(gateway_node_.get());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway_node_);
  std::thread spin([&exec] {
    exec.spin();
  });

  // tick_interval_ms MUST be 3000: node_death's own wall-clock floor
  // (min_node_death_miss_grace) bumps a fast-tick miss_grace up regardless of what is
  // configured, and prune_ticks = max(prune_grace, miss_grace + 1) - a bumped-up miss_grace
  // would dominate that max() identically whether prune_grace fell back to 1 or to 60,
  // masking the exact divergence this test exists to catch. At 3000ms the floor is exactly
  // 0, so the configured miss_grace(0) is used as written.
  plugin.configure(
      nlohmann::json{{"tick_interval_ms", 3000},
                     {"warmup_cycles", 0},
                     {"prune_grace", 0},  // plugin-scope, deliberately far from node_death's own hardcoded 60
                     {"detectors",
                      {{"node_death",
                        {{"miss_grace", 0},
                         {"prune_grace", "invalid"},  // malformed: must fall back, not pass through unfiltered
                         {"allowlist", nlohmann::json::array({"/victim"})},
                         {"suppress", nlohmann::json::array({"allowlist"})}}}}}});
  plugin.set_context(ctx);

  const auto routes = plugin.get_routes();
  const auto route_it = find_watchdog_route(routes);
  ASSERT_NE(route_it, routes.end());

  // node_death's tracked_count_ atomic default-initializes to 0 - the SAME value a reclaimed
  // /victim would leave it at - so reading it before the first tick has actually run would
  // make the deadline below pass vacuously, having proven nothing. Wait for the first tick to
  // publish "both anchor and victim are tracked" (2) before timing anything past it.
  const auto armed_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::size_t tracked_count = 0;
  while (std::chrono::steady_clock::now() < armed_deadline) {
    TestResponseSink armed_sink;
    ros2_medkit_gateway::PluginRequest armed_req(nullptr);
    ros2_medkit_gateway::PluginResponse armed_res(&armed_sink);
    route_it->handler(armed_req, armed_res);
    const auto & armed_detectors = armed_sink.body["x-medkit-watchdog"]["detectors"];
    if (armed_detectors.contains("node_death")) {
      tracked_count = armed_detectors["node_death"]["tracked_count"].get<std::size_t>();
      if (tracked_count >= 2) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(tracked_count, 2u) << "precondition: anchor and victim must both have been observed "
                                  "tracked at least once, or the reclaim timing below proves "
                                  "nothing";

  // prune_ticks = max(0, miss_grace(0)+1=1) = 1 if the plugin-scope fallback is used: victim
  // departs and mis-suppresses on tick 2 (streak 1), reclaimed on tick 3 (streak 2 > 1) - two
  // more 3000ms ticks past the precondition above, ~6-7s including scheduling slack. If
  // node_death instead fell back to its own hardcoded 60, reclaim needs streak 61 - roughly
  // 180s at this tick rate. The deadline sits well inside that gap: generous for the fixed
  // case, a small fraction of the hardcoded-fallback case, so this discriminates rather than
  // merely giving both enough time to pass.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  while (std::chrono::steady_clock::now() < deadline) {
    TestResponseSink sink;
    ros2_medkit_gateway::PluginRequest req(nullptr);
    ros2_medkit_gateway::PluginResponse res(&sink);
    route_it->handler(req, res);
    const auto & detectors = sink.body["x-medkit-watchdog"]["detectors"];
    if (detectors.contains("node_death")) {
      tracked_count = detectors["node_death"]["tracked_count"].get<std::size_t>();
      if (tracked_count == 1) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(tracked_count, 1u) << "the allowlisted, durably-suppressed /victim must be reclaimed within a few prune "
                                  "ticks sized off the plugin's OWN prune_grace default, not node_death's unrelated "
                                  "hardcoded fallback - only the anchor (never departed) should remain tracked";

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
      [&received, &received_mutex](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & /*req*/,
                                   const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & resp) {
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
