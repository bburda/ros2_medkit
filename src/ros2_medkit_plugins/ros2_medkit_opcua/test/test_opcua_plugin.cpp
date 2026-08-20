// Copyright 2026 mfaferek93
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

// Unit tests for OpcuaPlugin's DataProvider/OperationProvider/FaultProvider
// methods. These test the provider layer directly (no HTTP, no OPC-UA server)
// by constructing the plugin with a pre-loaded NodeMap and a fake poller
// snapshot. The gateway's PluginContext is stubbed with a minimal
// FakePluginContext that returns entities and faults on demand.

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

// -- Stub PluginRequest/PluginResponse (same pattern as graph_provider test) --

namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t) const {
  return {};
}
std::string PluginRequest::header(const std::string &) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
std::string PluginRequest::query_param(const std::string &) const {
  return {};
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json &) {
}
void PluginResponse::send_error(int, const std::string &, const std::string &, const nlohmann::json &) {
}

// -- FakePluginContext --

class FakePluginContext : public RosPluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;
  nlohmann::json all_faults = nlohmann::json::object();

  rclcpp::Node * node() const override {
    return nullptr;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    if (it != entities.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*entity_id*/) const override {
    return {};
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    // Contract: a bare JSON array of fault objects scoped to the entity.
    nlohmann::json result = nlohmann::json::array();
    if (all_faults.contains("faults")) {
      for (const auto & f : all_faults["faults"]) {
        bool in_scope = f.value("source_id", "") == entity_id;
        if (!in_scope && f.contains("reporting_sources") && f["reporting_sources"].is_array()) {
          for (const auto & src : f["reporting_sources"]) {
            in_scope = in_scope || (src.is_string() && src.get<std::string>() == entity_id);
          }
        }
        if (in_scope) {
          result.push_back(f);
        }
      }
    }
    return result;
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & /*res*/,
                                                            const std::string & entity_id) const override {
    return get_entity(entity_id);
  }

  // Recorded so tests can assert exactly which entities get which vendor
  // capability (e.g. x-plc-data must not be registered for a def with no
  // data points - see the auto_alarms fallback entity below).
  std::vector<std::pair<std::string, std::string>> registered_entity_capabilities;

  void register_capability(SovdEntityType /*type*/, const std::string & /*capability*/) override {
  }
  void register_entity_capability(const std::string & entity_id, const std::string & capability) override {
    registered_entity_capabilities.emplace_back(entity_id, capability);
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType /*type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                              const std::string & /*client_id*/) const override {
    return {true, "", "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                                                 const std::vector<std::string> & /*scopes*/,
                                                 int /*expiration_s*/) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  tl::expected<void, LockError> release_lock(const std::string & /*entity_id*/,
                                             const std::string & /*lock_id*/) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return all_faults;
  }
  void register_sampler(
      const std::string & /*topic*/,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &
      /*sampler*/) override {
  }
  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

// -- Test fixture --

class OpcuaPluginTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Write a test node map YAML
    yaml_path_ = "/tmp/test_opcua_plugin_nodemap.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: test_plc
component_id: test_runtime
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: true
    min_value: 0.0
    max_value: 1000.0
  - node_id: "ns=2;i=2"
    entity_id: tank
    data_name: pressure
    display_name: Tank Pressure
    unit: bar
    data_type: float
    writable: false
)";
    f.close();

    // Configure plugin with the test node map
    nlohmann::json config;
    config["node_map_path"] = yaml_path_;
    config["endpoint_url"] = "opc.tcp://nonexistent:4840";
    plugin_.configure(config);

    // Set up fake context with PLC entities
    ctx_.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
    ctx_.entities["test_runtime"] = {SovdEntityType::COMPONENT, "test_runtime", "/test_plc", "/test_plc/test_runtime"};
    plugin_.set_context(ctx_);
  }

  std::string yaml_path_;
  OpcuaPlugin plugin_;
  FakePluginContext ctx_;
};

// -- DataProvider tests --

TEST_F(OpcuaPluginTest, ListDataReturnsItems) {
  auto result = plugin_.list_data("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->content.contains("items"));
  EXPECT_EQ(result->content["items"].size(), 2u);
  EXPECT_EQ(result->content["items"][0]["id"], "level");
  EXPECT_EQ(result->content["items"][1]["id"], "pressure");
}

TEST(OpcuaPluginIntrospectTopics, DeclaresValueBridgeTopicsOnOwningEntity) {
  // #584: graph discovery attributes /plc/* topics to the gateway node (their
  // publisher), so the plugin must declare them on the entity that owns the
  // data points or entity-scoped topic lookup (data trigger resolution) stays
  // empty. String-typed entries get no publisher and must not be declared.
  const std::string yaml_path = "/tmp/test_opcua_introspect_topics.yaml";
  {
    std::ofstream f(yaml_path);
    f << R"(
area_id: test_plc
component_id: test_runtime
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    data_type: float
  - node_id: "ns=2;i=2"
    entity_id: tank
    data_name: label
    data_type: string
)";
  }
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = "opc.tcp://127.0.0.1:1";
  plugin.configure(config);

  auto result = plugin.introspect({});
  const auto it = std::find_if(result.new_entities.apps.begin(), result.new_entities.apps.end(), [](const App & a) {
    return a.id == "tank";
  });
  ASSERT_NE(it, result.new_entities.apps.end());
  ASSERT_EQ(it->topics.publishes.size(), 1u) << "only the float entry has a publisher";
  EXPECT_EQ(it->topics.publishes[0], "/plc/tank/level");
  EXPECT_TRUE(it->topics.subscribes.empty());
  // Both call sites (create_value_publishers and the declaration above) share
  // entry_has_value_publisher(), so the predicate itself must classify these
  // two fixture entries the way the declaration did - if either call site
  // stops using the predicate, review must catch it here.
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path));
  std::vector<std::string> predicate_topics;
  for (const auto & e : map.entries()) {
    if (entry_has_value_publisher(e)) {
      predicate_topics.push_back(e.ros2_topic);
    }
  }
  EXPECT_EQ(it->topics.publishes, predicate_topics);
}

TEST_F(OpcuaPluginTest, ListDataEntityNotFound) {
  auto result = plugin_.list_data("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::EntityNotFound);
  EXPECT_EQ(result.error().http_status, 404);
}

TEST_F(OpcuaPluginTest, ReadDataReturnsValue) {
  auto result = plugin_.read_data("tank", "level");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content["id"], "level");
  EXPECT_EQ(result->content["unit"], "mm");
  EXPECT_EQ(result->content["data_type"], "float");
  EXPECT_EQ(result->content["writable"], true);
}

TEST_F(OpcuaPluginTest, ReadDataNotFound) {
  auto result = plugin_.read_data("tank", "nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ResourceNotFound);
}

TEST_F(OpcuaPluginTest, WriteDataReadOnly) {
  nlohmann::json body = {{"value", 5.0}};
  auto result = plugin_.write_data("tank", "pressure", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ReadOnly);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST_F(OpcuaPluginTest, WriteDataMissingValue) {
  nlohmann::json body = {{"not_value", 42}};
  auto result = plugin_.write_data("tank", "level", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::InvalidValue);
}

// has_data gates PluginManager::get_data_provider_for_entity (and, through
// it, the gateway's `data` capability advertisement) at entity granularity -
// see discovery_handlers.cpp's prune_plugin_unserved_capabilities. It must
// track list_data's own "no entries" check exactly.
TEST_F(OpcuaPluginTest, HasDataTrueForDataBearingEntity) {
  EXPECT_TRUE(plugin_.has_data("tank"));
}

TEST_F(OpcuaPluginTest, HasDataFalseForComponentEntity) {
  // "test_runtime" is the PLC runtime Component id, not a data-bearing App -
  // it owns no NodeMap entries.
  EXPECT_FALSE(plugin_.has_data("test_runtime"));
}

TEST_F(OpcuaPluginTest, HasDataFalseForUnknownEntity) {
  EXPECT_FALSE(plugin_.has_data("nonexistent"));
}

// -- OperationProvider tests --

TEST_F(OpcuaPluginTest, ListOperationsOnlyWritable) {
  auto result = plugin_.list_operations("tank");
  ASSERT_TRUE(result.has_value());
  // Only 'level' is writable, 'pressure' is read-only
  EXPECT_EQ(result->items.size(), 1u);
  EXPECT_EQ(result->items[0].id, "set_level");
}

TEST_F(OpcuaPluginTest, ListOperationsEntityNotFound) {
  auto result = plugin_.list_operations("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::EntityNotFound);
}

// has_operations mirrors list_operations' own entity_defs() lookup so it
// gates PluginManager::get_operation_provider_for_entity at the same
// granularity list_operations already uses to 404.
TEST_F(OpcuaPluginTest, HasOperationsTrueForKnownEntityDef) {
  EXPECT_TRUE(plugin_.has_operations("tank"));
}

TEST_F(OpcuaPluginTest, HasOperationsFalseForComponentEntity) {
  // "test_runtime" has no entity_defs entry (see build_entity_defs' comment
  // on why the Component id is deliberately excluded), so it has no
  // operations even though the plugin implements OperationProvider.
  EXPECT_FALSE(plugin_.has_operations("test_runtime"));
}

TEST_F(OpcuaPluginTest, HasOperationsFalseForUnknownEntity) {
  EXPECT_FALSE(plugin_.has_operations("nonexistent"));
}

TEST_F(OpcuaPluginTest, ExecuteOperationMissingValue) {
  nlohmann::json params = {{"not_value", 42}};
  auto result = plugin_.execute_operation("tank", "set_level", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::InvalidParameters);
}

TEST_F(OpcuaPluginTest, ExecuteOperationReadOnly) {
  nlohmann::json params = {{"value", 5.0}};
  auto result = plugin_.execute_operation("tank", "set_pressure", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::Rejected);
}

// -- auto_alarms fallback entity: has data/operations fitness + introspect
// -- capability registration --
//
// build_entity_defs() gives the auto_alarms fallback entity ("<component>_
// alarms") an entity_defs entry purely to host faults (has_faults=true) -
// it owns no NodeMap entries. Before this fix introspect() unconditionally
// registered x-plc-data for every entity_defs entry, so this fallback
// entity advertised x-plc-data (and, via the plugin-wide DataProvider, the
// base SOVD `data` capability) despite list_data 404ing on it.

class OpcuaPluginAlarmsFitnessTest : public ::testing::Test {
 protected:
  void SetUp() override {
    yaml_path_ = "/tmp/test_opcua_plugin_alarms_fitness_nodemap.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: test_plc
component_id: test_runtime
auto_alarms:
  enabled: true
  source_node_id: "ns=2;i=999"
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    data_type: float
    writable: false
)";
    f.close();

    nlohmann::json config;
    config["node_map_path"] = yaml_path_;
    config["endpoint_url"] = "opc.tcp://nonexistent:4840";
    plugin_.configure(config);
    plugin_.set_context(ctx_);
  }

  std::string yaml_path_;
  OpcuaPlugin plugin_;
  FakePluginContext ctx_;
  // "<component_id>_alarms" per NodeMap::finalize_auto_alarms_overlay's
  // default when auto_alarms.entity_id is not set explicitly.
  static constexpr const char * kAlarmsEntityId = "test_runtime_alarms";
};

TEST_F(OpcuaPluginAlarmsFitnessTest, HasDataFalseForAlarmsFallbackEntity) {
  EXPECT_FALSE(plugin_.has_data(kAlarmsEntityId));
}

TEST_F(OpcuaPluginAlarmsFitnessTest, HasOperationsTrueForAlarmsFallbackEntity) {
  // The fallback entity DOES have an entity_defs entry (it exists purely to
  // host faults), so list_operations succeeds (with an empty or
  // ack/confirm-only list) rather than 404ing - unlike the Component, whose
  // id never appears in entity_defs() at all.
  EXPECT_TRUE(plugin_.has_operations(kAlarmsEntityId));
}

TEST_F(OpcuaPluginAlarmsFitnessTest, IntrospectSkipsXPlcDataForAlarmsFallbackEntity) {
  auto result = plugin_.introspect({});

  bool alarms_got_x_plc_data = false;
  bool tank_got_x_plc_data = false;
  for (const auto & [entity_id, capability] : ctx_.registered_entity_capabilities) {
    if (capability != "x-plc-data") {
      continue;
    }
    if (entity_id == kAlarmsEntityId) {
      alarms_got_x_plc_data = true;
    }
    if (entity_id == "tank") {
      tank_got_x_plc_data = true;
    }
  }
  EXPECT_FALSE(alarms_got_x_plc_data) << "auto_alarms fallback entity has no data points - "
                                         "x-plc-data must not be registered for it";
  EXPECT_TRUE(tank_got_x_plc_data) << "data-bearing entities must keep x-plc-data";
}

// -- FaultProvider tests --

TEST_F(OpcuaPluginTest, ListFaultsEmpty) {
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->content.contains("items"));
  EXPECT_TRUE(result->content["items"].empty());
}

TEST_F(OpcuaPluginTest, ListFaultsWithData) {
  ctx_.all_faults = {{"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"source_id", "tank"}, {"severity", 2}}}}};
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content["items"].size(), 1u);
  EXPECT_EQ(result->content["items"][0]["code"], "PLC_LOW_LEVEL");
}

TEST_F(OpcuaPluginTest, ListFaultsPassesThroughOccurrenceData) {
  // The entity-scoped route must not drop the occurrence fields the fault
  // manager provides - clients rendered "time unknown" when the serializer
  // projected only {code, severity, description, status, source_id}.
  ctx_.all_faults = {{"faults",
                      {{{"fault_code", "PLC_LOW_LEVEL"},
                        {"source_id", "tank"},
                        {"severity", 2},
                        {"severity_label", "ERROR"},
                        {"status", "CONFIRMED"},
                        {"first_occurred", 100.5},
                        {"last_occurred", 200.25},
                        {"occurrence_count", 7},
                        {"reporting_sources", {"tank"}}}}}};
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->content["items"].size(), 1u);
  const auto & item = result->content["items"][0];
  EXPECT_EQ(item["code"], "PLC_LOW_LEVEL");
  EXPECT_EQ(item["first_occurred"], 100.5);
  EXPECT_EQ(item["last_occurred"], 200.25);
  EXPECT_EQ(item["occurrence_count"], 7);
  EXPECT_EQ(item["severity_label"], "ERROR");
  EXPECT_EQ(item["reporting_sources"][0], "tank");
  EXPECT_EQ(item["source_id"], "tank");
}

TEST_F(OpcuaPluginTest, ListFaultsDerivesSourceIdFromReportingSources) {
  // Fault-manager records carry reporting_sources, not source_id; the item's
  // source_id must fall back to the first reporting source instead of "".
  ctx_.all_faults = {
      {"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"severity", 2}, {"reporting_sources", {"tank", "other"}}}}}};
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->content["items"].size(), 1u);
  EXPECT_EQ(result->content["items"][0]["source_id"], "tank");
}

TEST_F(OpcuaPluginTest, GetFaultNotFound) {
  auto result = plugin_.get_fault("tank", "NONEXISTENT");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, FaultProviderError::FaultNotFound);
}

TEST_F(OpcuaPluginTest, GetFaultFound) {
  // Regression pin for the shape contract: list_entity_faults yields a bare
  // array of fault objects scoped to the entity, not a {"faults": [...]}
  // object. get_fault must locate the matching fault in that bare array.
  ctx_.all_faults = {{"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"source_id", "tank"}, {"severity", 2}}}}};
  auto result = plugin_.get_fault("tank", "PLC_LOW_LEVEL");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content.value("fault_code", ""), "PLC_LOW_LEVEL");
}

// -- configure() validation (issue #481) --

TEST(OpcuaPluginConfigureTest, ThrowsOnInvalidNodeMap) {
  // A node map that fails validation (here a duplicate fault_code) must make
  // configure() throw so PluginManager disables the plugin instead of starting
  // the poller against a map whose global fault-code uniqueness was rejected.
  std::string path = "/tmp/test_opcua_plugin_invalid_nodemap.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent_a
    data_name: a
    alarm:
      fault_code: DUP_CODE
      threshold: 1.0
  - node_id: "ns=1;i=2"
    entity_id: ent_b
    data_name: b
    alarm:
      fault_code: DUP_CODE
      threshold: 2.0
)";
  f.close();

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = path;
  config["endpoint_url"] = "opc.tcp://nonexistent:4840";
  EXPECT_THROW(plugin.configure(config), std::runtime_error);
}

// Issue #389: condition-replay strategy parsing (used by the poller's
// reconnect / restart active-condition replay).
TEST(ConditionReplayStrategyParse, KnownValues) {
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("method"), ConditionReplayStrategy::Method);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("read"), ConditionReplayStrategy::Read);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("read_fallback"), ConditionReplayStrategy::Read);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("off"), ConditionReplayStrategy::Off);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("auto"), ConditionReplayStrategy::Auto);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("AUTO"), ConditionReplayStrategy::Auto);
  // Unknown -> safe default (Auto: method with read fallback).
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("bogus"), ConditionReplayStrategy::Auto);
}

// -- Issue #479/#478: read-replay reconcile must not falsely clear faults -----

TEST(ReconcileShouldClearCondition, ClearsActiveConditionAbsentFromSuccessfulScan) {
  // An active condition that the (successful) scan did not observe is gone -
  // but only when its source is positively known to model condition nodes.
  std::set<std::string> seen;                      // condition not seen this scan
  std::set<std::string> failed_sources;            // its source scanned fine
  std::set<std::string> modeled_sources{"src-1"};  // source exposes condition nodes
  EXPECT_TRUE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                  modeled_sources));
  EXPECT_TRUE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                  modeled_sources));
}

TEST(ReconcileShouldClearCondition, KeepsConditionStillSeen) {
  std::set<std::string> seen{"cond-1"};
  std::set<std::string> failed_sources;
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

TEST(ReconcileShouldClearCondition, SkipsClearWhenSourceScanFailed) {
  // The false-clear guard: source scan failed (browse error / disconnect),
  // so the condition's absence from ``seen`` must NOT clear it.
  std::set<std::string> seen;                     // not observed...
  std::set<std::string> failed_sources{"src-1"};  // ...because its source failed
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

// Issue #496: comms-lost debounce gate.
TEST(CommsLostShouldRaise, RaisesAfterDebounceElapsed) {
  const auto t0 = std::chrono::steady_clock::time_point{};
  const auto debounce = std::chrono::milliseconds(5000);
  // Continuously down for the full window -> raise.
  EXPECT_TRUE(OpcuaPoller::comms_lost_should_raise(/*enabled=*/true, /*already_raised=*/false, t0,
                                                   t0 + std::chrono::milliseconds(5000), debounce));
  EXPECT_TRUE(OpcuaPoller::comms_lost_should_raise(true, false, t0, t0 + std::chrono::seconds(9), debounce));
}

TEST(CommsLostShouldRaise, HoldsDuringDebounceWindow) {
  const auto t0 = std::chrono::steady_clock::time_point{};
  const auto debounce = std::chrono::milliseconds(5000);
  // A blip shorter than the window must not raise.
  EXPECT_FALSE(OpcuaPoller::comms_lost_should_raise(true, false, t0, t0 + std::chrono::milliseconds(4999), debounce));
}

TEST(CommsLostShouldRaise, IdempotentAndDisabled) {
  const auto t0 = std::chrono::steady_clock::time_point{};
  const auto debounce = std::chrono::milliseconds(5000);
  const auto late = t0 + std::chrono::seconds(30);
  // Already raised -> do not raise again (one-shot).
  EXPECT_FALSE(OpcuaPoller::comms_lost_should_raise(true, /*already_raised=*/true, t0, late, debounce));
  // Disabled -> never raise.
  EXPECT_FALSE(OpcuaPoller::comms_lost_should_raise(/*enabled=*/false, false, t0, late, debounce));
}

// Issue #478 safety-gate: an empty scan from a source that has NEVER yielded a
// condition instance node (EventNotifier-only server, e.g. S7-1500) must NOT
// clear the still-active tracked fault. This is the single most important
// outcome of the hardening.
TEST(ReconcileShouldClearCondition, NeverClearsWhenSourceNotPositivelyModeled) {
  std::set<std::string> seen;             // condition not seen (server has no nodes)
  std::set<std::string> failed_sources;   // the browse itself succeeded (empty)
  std::set<std::string> modeled_sources;  // src-1 has never yielded a condition node
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

TEST(ReconcileShouldClearCondition, NeverClearsInactiveCondition) {
  std::set<std::string> seen;
  std::set<std::string> failed_sources;
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Cleared, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Suppressed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

// -- Issue #480: ConditionRefresh burst reconcile must clear a tracked fault
// that the server did NOT replay (it cleared while we were offline). Unlike the
// read fallback, a delivered RefreshEnd is an authoritative subscription-wide
// replay, so there is no per-source modeling gate. --------------------------

TEST(ReconcileAfterRefresh, ClearsActiveConditionNotReplayed) {
  // Active before the reconnect, absent from the RefreshStart..RefreshEnd burst
  // => the alarm cleared offline and its latched fault must reconcile away.
  std::set<std::string> seen;  // server replayed nothing (no active conditions)
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-1", seen));
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Healed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, KeepsConditionReplayedInBurst) {
  // The server replayed this ConditionId during the burst => still active, keep.
  std::set<std::string> seen{"cond-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-1", seen));
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Healed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, NeverClearsInactiveCondition) {
  // A condition that was never active (Suppressed/Cleared) is not a latched
  // fault and must not be touched even when absent from the burst.
  std::set<std::string> seen;
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Cleared, "cond-1", seen));
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Suppressed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, KeepsOtherActiveConditionsWhenOneReplayed) {
  // Peer condition (same fault_code family, different ConditionId) replayed;
  // the un-replayed one still clears. Guards the distinct-ConditionId keying.
  std::set<std::string> seen{"cond-keep"};
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-keep", seen));
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-gone", seen));
}

// -- Issue #478: read-scan snapshot classification (Retain / EnabledState /
// transient-keep filter that mirrors ConditionRefresh semantics). --------------

namespace {
OpcuaClient::ConditionStateSnapshot make_snap(bool enabled, bool active, bool retain, bool read_failed = false) {
  OpcuaClient::ConditionStateSnapshot s;
  s.enabled_state = enabled;
  s.active_state = active;
  s.retain = retain;
  s.state_read_failed = read_failed;
  return s;
}
}  // namespace

TEST(ReadSnapshotClassify, RetainTrueActiveIsFed) {
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, RetainTrueInactiveStillInterestingIsFed) {
  // ActiveState=false but Retain=true => unacked/unconfirmed, still of interest
  // (ConditionRefresh would replay it). Fed so the HEALED latch is preserved.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/false, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, RetainFalseIsSkipped) {
  // Retain=false => no longer interesting; not seen so a stale fault reconciles.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/false, /*retain=*/false));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Skip);
}

TEST(ReadSnapshotClassify, DisabledIsSkippedEvenIfActiveAndRetained) {
  // EnabledState=false must never be treated as active in the read path, even
  // when ActiveState/Retain would otherwise mark it interesting.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/false, /*active=*/true, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Skip);
}

TEST(ReadSnapshotClassify, TransientReadFailureIsKeptNotFed) {
  // A transient read/browse failure (flagged) keeps the condition (so reconcile
  // does not clear it) but does not feed an unreliable state into the machine.
  // KeepOnly takes precedence over every other field.
  auto d = OpcuaPoller::classify_read_snapshot(
      make_snap(/*enabled=*/false, /*active=*/false, /*retain=*/false, /*read_failed=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::KeepOnly);
}

TEST(ReadSnapshotClassify, ActiveWithUnreadableRetainIsFedNotSkipped) {
  // Regression (issue #478): Retain defaults false when its optional node is
  // unreadable this scan. A reliably-active condition must be Fed (so it lands
  // in ``seen``), never Skipped on the Retain check - otherwise reconcile would
  // clear a still-active fault on a modeled source.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/false));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, ActiveButFlaggedUnreliableIsKeptNotFed) {
  // KeepOnly still wins over the active fast-path: an active condition whose
  // snapshot is flagged unreliable (e.g. a transient Retain read failure) is
  // preserved via ``seen`` but not fed an untrustworthy state.
  auto d = OpcuaPoller::classify_read_snapshot(
      make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/false, /*read_failed=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::KeepOnly);
}

// -- OpcuaPlugin::map_severity (event-alarm severity resolution) -------------

TEST(MapSeverity, ConfiguredOverrideWinsOverLiveSeverity) {
  // An explicit configured severity wins regardless of the live event Severity.
  EXPECT_EQ(OpcuaPlugin::map_severity(1000, "INFO"), "INFO");
  EXPECT_EQ(OpcuaPlugin::map_severity(1, "CRITICAL"), "CRITICAL");
}

TEST(MapSeverity, LowLiveSeverityMapsToInfo) {
  // The Siemens trap: many servers emit Program_Alarm events at Severity 1, so
  // with no configured override the fault lands as INFO. This is exactly why a
  // silently dropped ``severity:`` key mattered.
  EXPECT_EQ(OpcuaPlugin::map_severity(1, ""), "INFO");
}

TEST(MapSeverity, LiveSeverityBandBoundaries) {
  // Band map: >=801 CRITICAL, >=501 ERROR, >=201 WARNING, else INFO.
  EXPECT_EQ(OpcuaPlugin::map_severity(200, ""), "INFO");
  EXPECT_EQ(OpcuaPlugin::map_severity(201, ""), "WARNING");
  EXPECT_EQ(OpcuaPlugin::map_severity(500, ""), "WARNING");
  EXPECT_EQ(OpcuaPlugin::map_severity(501, ""), "ERROR");
  EXPECT_EQ(OpcuaPlugin::map_severity(800, ""), "ERROR");
  EXPECT_EQ(OpcuaPlugin::map_severity(801, ""), "CRITICAL");
  EXPECT_EQ(OpcuaPlugin::map_severity(1000, ""), "CRITICAL");
}

// -- OpcuaPlugin::apply_auto_alarms_param (plugins.opcua.auto_alarms param) ---
// The zero-config native A&C param surface: mirrors the node-map YAML
// ``auto_alarms:`` loader field-for-field but reads the plugin's JSON/ROS
// param, overlaying on top of (or standing in for) a node map.

TEST(ApplyAutoAlarmsParam, BareBooleanTrueEnablesWithDefaults) {
  AutoAlarmsConfig cfg;
  int warns = 0;
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json(true), cfg, [&](const std::string &) {
    ++warns;
  });
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.source_node_id_str, "i=2253");  // untouched default
  EXPECT_TRUE(cfg.auto_clear);
  EXPECT_EQ(cfg.severity_bands.critical_min, 801);
  EXPECT_EQ(warns, 0);
}

TEST(ApplyAutoAlarmsParam, BareBooleanFalseStaysDisabled) {
  AutoAlarmsConfig cfg;
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json(false), cfg, [](const std::string &) {});
  EXPECT_FALSE(cfg.enabled);
}

TEST(ApplyAutoAlarmsParam, MapFormOverridesEveryField) {
  AutoAlarmsConfig cfg;
  const nlohmann::json param = {
      {"enabled", true},
      {"source_node_id", "ns=3;i=1000"},
      {"entity_id", "alarms_catchall"},
      {"auto_clear", false},
      {"severity_bands", {{"critical", 900}, {"error", 700}, {"warning", 400}}},
      {"include", {"Program_Alarm"}},
      {"exclude", {"CPU not in RUN"}},
  };
  int warns = 0;
  OpcuaPlugin::apply_auto_alarms_param(param, cfg, [&](const std::string &) {
    ++warns;
  });
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.source_node_id_str, "ns=3;i=1000");
  EXPECT_EQ(cfg.entity_id, "alarms_catchall");
  EXPECT_FALSE(cfg.auto_clear);
  EXPECT_EQ(cfg.severity_bands.critical_min, 900);
  EXPECT_EQ(cfg.severity_bands.error_min, 700);
  EXPECT_EQ(cfg.severity_bands.warning_min, 400);
  ASSERT_EQ(cfg.include_patterns.size(), 1u);
  EXPECT_EQ(cfg.include_patterns[0], "Program_Alarm");
  ASSERT_EQ(cfg.exclude_patterns.size(), 1u);
  EXPECT_EQ(cfg.exclude_patterns[0], "CPU not in RUN");
  EXPECT_EQ(warns, 0);
}

TEST(ApplyAutoAlarmsParam, MapPresenceImpliesEnabledWhenEnabledKeyOmitted) {
  AutoAlarmsConfig cfg;
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json{{"entity_id", "alarms_catchall"}}, cfg,
                                       [](const std::string &) {});
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.entity_id, "alarms_catchall");
}

TEST(ApplyAutoAlarmsParam, PartialMapKeepsUnsetFields) {
  // A param that only sets max-ish knobs must not wipe the rest (mirrors the
  // "only overwrite keys actually present" contract of the YAML loader).
  AutoAlarmsConfig cfg;
  cfg.include_patterns = {"kept"};
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json{{"auto_clear", false}}, cfg, [](const std::string &) {});
  EXPECT_FALSE(cfg.auto_clear);
  EXPECT_EQ(cfg.source_node_id_str, "i=2253");
  ASSERT_EQ(cfg.include_patterns.size(), 1u);
  EXPECT_EQ(cfg.include_patterns[0], "kept");  // include omitted -> untouched
}

TEST(ApplyAutoAlarmsParam, InvalidSeverityBandOrderResetsToDefaults) {
  AutoAlarmsConfig cfg;
  int warns = 0;
  OpcuaPlugin::apply_auto_alarms_param(
      nlohmann::json{{"severity_bands", {{"critical", 300}, {"error", 500}, {"warning", 700}}}}, cfg,
      [&](const std::string &) {
        ++warns;
      });
  EXPECT_EQ(cfg.severity_bands.critical_min, 801);
  EXPECT_EQ(cfg.severity_bands.error_min, 501);
  EXPECT_EQ(cfg.severity_bands.warning_min, 201);
  EXPECT_GE(warns, 1);
}

TEST(ApplyAutoAlarmsParam, UnknownKeyWarns) {
  AutoAlarmsConfig cfg;
  int warns = 0;
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json{{"severty", 900}}, cfg, [&](const std::string &) {
    ++warns;
  });
  EXPECT_EQ(warns, 1);
}

TEST(ApplyAutoAlarmsParam, NonBooleanNonObjectIsIgnoredWithWarning) {
  AutoAlarmsConfig cfg;
  int warns = 0;
  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json("nonsense"), cfg, [&](const std::string &) {
    ++warns;
  });
  EXPECT_FALSE(cfg.enabled);
  EXPECT_EQ(warns, 1);
}

TEST(ApplyAutoAlarmsParam, JsonParamWinsOverNodeMapYamlConfigBlock) {
  // Precedence: the JSON/ROS param overlays and WINS over the node-map YAML
  // ``auto_alarms:`` block, identical to how plugins.opcua.auto_browse
  // overlays its config. (The node map still wins at the ENTRY level -
  // explicit ``event_alarms:`` mappings beat auto-derivation - but that is the
  // poller's effective_alarm_sources precedence, tested separately.)
  const std::string path = "/tmp/test_opcua_plugin_auto_alarms_overlay.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: plc_runtime
auto_alarms:
  enabled: true
  source_node_id: "ns=1;i=100"
  entity_id: yaml_entity
  auto_clear: true
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  ASSERT_EQ(map.auto_alarms().source_node_id_str, "ns=1;i=100");

  OpcuaPlugin::apply_auto_alarms_param(nlohmann::json{{"source_node_id", "ns=2;i=200"}, {"auto_clear", false}},
                                       map.mutable_auto_alarms(), [](const std::string &) {});
  ASSERT_TRUE(map.finalize_auto_alarms_overlay());

  const auto & cfg = map.auto_alarms();
  EXPECT_TRUE(cfg.enabled);
  EXPECT_EQ(cfg.source_node_id_str, "ns=2;i=200");  // param won
  EXPECT_FALSE(cfg.auto_clear);                     // param won
  EXPECT_EQ(cfg.entity_id, "yaml_entity");          // untouched by param -> YAML value survives
  std::remove(path.c_str());
}

// -- Thread-safety regression: pending_reports_ ------------------------------

// A PluginContext that hands the plugin a real rclcpp::Node so set_context()
// creates the ReportFault / ClearFault service clients. With a null node (the
// default FakePluginContext) send_clear_fault() short-circuits before touching
// the pending-reports buffer, so the race below can only be reproduced with a
// live node.
class RealNodePluginContext : public FakePluginContext {
 public:
  explicit RealNodePluginContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }

 private:
  rclcpp::Node * node_;
};

// RAII: pair rclcpp init with a shutdown so global ROS state never leaks into
// the rest of the process, even if an assertion returns from the test early.
// Only tears down the init this guard performed.
struct ScopedRclcpp {
  const bool owned_;
  ScopedRclcpp() : owned_(!rclcpp::ok()) {
    if (owned_) {
      rclcpp::init(0, nullptr);
    }
  }
  ~ScopedRclcpp() {
    if (owned_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  ScopedRclcpp(const ScopedRclcpp &) = delete;
  ScopedRclcpp & operator=(const ScopedRclcpp &) = delete;
};

// RAII: owns the executor and its spin thread so cleanup cannot be skipped by an
// early ASSERT return. stop() is called explicitly before the plugin/nodes are
// torn down so no entity is destroyed while the executor is still spinning it.
struct ScopedExecutorSpin {
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread thread;
  explicit ScopedExecutorSpin(const std::vector<rclcpp::Node::SharedPtr> & nodes) {
    for (const auto & node : nodes) {
      executor.add_node(node);
    }
    thread = std::thread([this] {
      executor.spin();
    });
  }
  void stop() {
    executor.cancel();
    if (thread.joinable()) {
      thread.join();
    }
  }
  ~ScopedExecutorSpin() {
    stop();
  }
  ScopedExecutorSpin(const ScopedExecutorSpin &) = delete;
  ScopedExecutorSpin & operator=(const ScopedExecutorSpin &) = delete;
};

// The SOVD DELETE /faults/{code} route lands on FaultProvider::clear_fault(),
// which buffers a dispatch into pending_reports_ - the SAME vector the poll
// thread drains in publish_values()/flush_pending_reports(). Before the fix the
// buffer had no lock, so a push_back that reallocated the vector while another
// thread iterated/swapped it corrupted the heap (the crash observed in
// libros2_medkit_opcua_plugin.so under UI load).
//
// The buffer has two locked paths that must BOTH be raced: the push_back/erase
// in send_or_buffer() and the batch.swap(pending_reports_) drain in
// flush_pending_reports(). The drain only runs once report->service_is_ready()
// is true, so the test stands up a real ReportFault/ClearFault server (a stub
// fault_manager) to open that gate; only then does every clear_fault() both push
// into the buffer AND swap it out under concurrent pushes from the other worker
// threads. It must complete without heap corruption and is clean under
// ThreadSanitizer - the DDS/rclcpp machinery it drives is covered by
// tsan_suppressions.txt, the same paths the gateway service tests exercise.
TEST(OpcuaPluginConcurrency, ClearFaultBufferIsThreadSafe) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_pending_reports_regression");

  // Stub fault_manager on a second node. ReportFault existing is what flips the
  // plugin's report client to ready so flush_pending_reports() gets past its
  // early return; ClearFault counts the flushed dispatches so the test can prove
  // the drain (swap) path actually ran - a guard against the sink silently never
  // matching, which would drop coverage back to push-only.
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_pending_reports_faultmgr");
  std::atomic<int> cleared_received{0};
  auto report_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request>,
                                        std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> res) {
        res->accepted = true;
      });
  auto clear_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ClearFault>(
      "/fault_manager/clear_fault",
      [&cleared_received](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request>,
                          std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> res) {
        cleared_received.fetch_add(1, std::memory_order_relaxed);
        res->success = true;
      });

  const std::string yaml_path = "/tmp/test_opcua_race_nodemap.yaml";
  {
    std::ofstream f(yaml_path);
    f << R"(
area_id: race_plc
component_id: race_runtime
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    data_type: float
)";
  }

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  // Nothing listening on the OPC-UA side: connect fails fast (ECONNREFUSED) and
  // the poll thread keeps retrying in the background. The fault sink above, not
  // the (dead) endpoint, is what drives the drain path.
  config["endpoint_url"] = "opc.tcp://127.0.0.1:1";
  config["poll_interval_ms"] = 100;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/race_plc", "/race_plc/race_runtime/tank"};
  plugin.set_context(ctx);

  // Executor spins the plugin node and the stub fault_manager so requests flow
  // and the sink is discoverable; added after set_context() so it picks up the
  // clients the plugin just created.
  ScopedExecutorSpin spinner({node, fault_manager});

  // Wait until the stub server is discoverable. A probe client on the SAME node
  // stands in for the plugin's (private) report client; service_is_ready() reads
  // the RMW graph directly, so it flips without racing the spinning executor's
  // wait set.
  auto probe = node->create_client<ros2_medkit_msgs::srv::ReportFault>("/fault_manager/report_fault");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!probe->service_is_ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(probe->service_is_ready()) << "stub ReportFault server never became discoverable";

  std::atomic<bool> stop{false};
  std::vector<std::thread> threads;
  for (int t = 0; t < 6; ++t) {
    threads.emplace_back([&plugin, &stop, t] {
      int i = 0;
      while (!stop.load(std::memory_order_relaxed)) {
        // Sink is ready, so each call pushes into pending_reports_ AND drains it
        // via flush_pending_reports() -> batch.swap(): worker threads race
        // push_back against swap on the shared buffer.
        // The result is deliberately dropped: this test races the shared
        // pending-report buffer, it does not assert on any single clear.
        static_cast<void>(plugin.clear_fault("tank", "RACE_" + std::to_string(t) + "_" + std::to_string(i++ & 0x3f)));
      }
    });
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));
  stop.store(true, std::memory_order_relaxed);
  for (auto & th : threads) {
    th.join();
  }

  // Stop the executor before any node/client is destroyed to avoid a teardown
  // race between the spin thread and entity destruction.
  spinner.stop();
  plugin.shutdown();

  // Proves the drain path was actually exercised (not silently skipped by an
  // unmatched sink): the buffer was swapped out and dispatched to the server.
  EXPECT_GT(cleared_received.load(std::memory_order_relaxed), 0)
      << "flush_pending_reports never dispatched - swap-vs-push path not covered";
}

}  // namespace ros2_medkit_gateway
