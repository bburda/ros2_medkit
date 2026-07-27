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

#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <string>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/transports/log_source.hpp"

using json = nlohmann::json;
using namespace ros2_medkit_gateway;

namespace {

class NullLogSource : public LogSource {
 public:
  void start(EntryCallback) override {
  }
  void stop() override {
  }
};

std::set<std::string> messages_of(const json & items) {
  std::set<std::string> out;
  for (const auto & item : items) {
    out.insert(item.value("message", ""));
  }
  return out;
}

}  // namespace

// =============================================================================
// logs::query_entity_logs
//
// Pins the shared entity -> log-source scope used by GET /{entity}/logs and
// the cyclic subscription "logs" sampler: exact sources follow the fault-scope
// rule (external app -> bare id, external component owns its own id), while
// COMPONENT / AREA merge the namespace-prefix query on top.
// =============================================================================

class LogScopeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Manifest component with a namespace hosting one external app and one
    // ROS-bound app.
    Component cell;
    cell.id = "plant_cell";
    cell.name = "Plant Cell";
    cell.namespace_path = "/plant";
    cell.fqn = "/plant";
    cell.area = "cell_area";

    // External component with no apps (protocol bridge asset).
    Component box;
    box.id = "plc-box";
    box.name = "PLC Box";
    box.external = true;

    App ext;
    ext.id = "ext-plc";
    ext.name = "External PLC";
    ext.component_id = "plant_cell";
    ext.external = true;

    App temp;
    temp.id = "temp";
    temp.name = "Temp Sensor";
    temp.component_id = "plant_cell";
    App::RosBinding rb;
    rb.node_name = "temp";
    rb.namespace_pattern = "/plant";
    temp.ros_binding = rb;

    Area area;
    area.id = "cell_area";
    area.name = "Cell Area";
    area.namespace_path = "/plant";

    Function line;
    line.id = "line_func";
    line.name = "Line Function";
    line.hosts = {"plant_cell"};  // Component host, not an app

    cache_.update_all({area}, {cell, box}, {ext, temp}, {line});

    mgr_.add_log_entry("ext-plc", "info", "plc entry", {});
    mgr_.add_log_entry("/plant/node1", "info", "node1 entry", {});
    mgr_.add_log_entry("/plant/temp", "info", "temp entry", {});
    mgr_.add_log_entry("plc-box", "info", "box entry", {});
    mgr_.add_log_entry("/other/node", "info", "other entry", {});
  }

  ThreadSafeEntityCache cache_;
  LogManager mgr_{std::make_shared<NullLogSource>()};
};

// COMPONENT merges the exact query over resolved sources (bare id for the
// external app, FQN for the bound app) with the namespace-prefix query.
// Before the merge, a non-empty resolved set disabled the prefix path and the
// /plant/* node entries silently vanished.
TEST_F(LogScopeTest, ComponentMergesExternalAppAndNamespaceEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("node1 entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("other entry"));
}

// The bound app's entry is hit by both the exact and the prefix query - it
// must appear exactly once, and the merged list stays sorted by id.
TEST_F(LogScopeTest, ComponentDedupesEntriesMatchedByBothQueries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  int temp_count = 0;
  long long prev = -1;
  for (const auto & item : *result) {
    if (item.value("message", "") == "temp entry") {
      ++temp_count;
    }
    const long long n = std::stoll(item.value("id", "log_0").substr(4));
    EXPECT_GT(n, prev);
    prev = n;
  }
  EXPECT_EQ(temp_count, 1);
}

// An external component owns entries stored under its own entity id even
// with no hosted apps.
TEST_F(LogScopeTest, ExternalComponentOwnsItsOwnIdEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plc-box", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"box entry"});
}

// An external app has an empty effective_fqn; its entries live under its bare
// entity id. The raw {entity.fqn} query returned nothing for it.
TEST_F(LogScopeTest, ExternalAppResolvesToBareEntityId) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, "ext-plc", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"plc entry"});
}

// A ROS-bound app keeps its exact-FQN scope - no namespace bleed.
TEST_F(LogScopeTest, BoundAppKeepsExactFqnScope) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, "temp", "/plant/temp", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"temp entry"});
}

// A function hosting a Component resolves the component's apps (incl. the
// external one) but never runs a namespace query.
TEST_F(LogScopeTest, FunctionFollowsComponentHosts) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::FUNCTION, "line_func", "", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("node1 entry"));
}

// AREA resolves hosted apps through its components and merges the namespace
// prefix query, mirroring COMPONENT.
TEST_F(LogScopeTest, AreaMergesHostedAppsAndNamespaceEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::AREA, "cell_area", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("node1 entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("other entry"));
}

// The resolved_sources out-param carries the exact-match set for x-medkit
// metadata.
TEST_F(LogScopeTest, ResolvedSourcesOutParamExposesExactSet) {
  std::set<std::string> resolved;
  auto result =
      logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "", &resolved);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(resolved.count("ext-plc"));
  EXPECT_TRUE(resolved.count("/plant/temp"));
}

// Unknown entity types resolve to nothing.
TEST_F(LogScopeTest, UnknownTypeReturnsEmpty) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::UNKNOWN, "whatever", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}
