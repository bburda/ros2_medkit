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

/**
 * @file test_discovery_models.cpp
 * @brief Unit tests for SOVD discovery model serialization
 *
 * @verifies REQ_INTEROP_003
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Area;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::Function;
using ros2_medkit_gateway::json;

// =============================================================================
// Area Model Tests
// =============================================================================

class AreaModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    area_.id = "powertrain";
    area_.name = "Powertrain System";
    area_.namespace_path = "/powertrain";
    area_.type = "Area";
    area_.translation_id = "area.powertrain";
    area_.description = "Powertrain control systems";
    area_.tags = {"critical", "automotive"};
    area_.parent_area_id = "vehicle";
  }

  Area area_;
};

// @verifies REQ_INTEROP_003
TEST_F(AreaModelTest, ToJson_ContainsRequiredFields) {
  json j = area_.to_json();

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["namespace"], "/powertrain");
  EXPECT_EQ(j["x-medkit"]["entityType"], "Area");
}

TEST_F(AreaModelTest, ToJson_ContainsOptionalFields) {
  json j = area_.to_json();

  EXPECT_EQ(j["name"], "Powertrain System");
  EXPECT_EQ(j["translationId"], "area.powertrain");
  EXPECT_EQ(j["x-medkit"]["description"], "Powertrain control systems");
  EXPECT_EQ(j["tags"].size(), 2);
  EXPECT_EQ(j["x-medkit"]["parentAreaId"], "vehicle");
}

TEST_F(AreaModelTest, ToJson_OmitsEmptyOptionalFields) {
  Area minimal;
  minimal.id = "test";
  minimal.namespace_path = "/test";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("name"));
  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("parentAreaId"));
}

TEST_F(AreaModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = area_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/areas/powertrain");
  EXPECT_FALSE(j.contains("type"));
}

TEST_F(AreaModelTest, ToCapabilities_ContainsSubResources) {
  json j = area_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Area");
  EXPECT_TRUE(j.contains("subareas"));
  // `/areas/{area_id}/components` is the registered route. The former
  // "related-components" key named a segment no entity type serves, so a green
  // assertion on it read as evidence that the segment was real.
  EXPECT_TRUE(j.contains("components"));
  EXPECT_FALSE(j.contains("related-components"));
}

// =============================================================================
// Component Model Tests
// =============================================================================

class ComponentModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    comp_.id = "motor_controller";
    comp_.name = "Motor Controller";
    comp_.namespace_path = "/powertrain";
    comp_.fqn = "/powertrain/motor_controller";
    comp_.type = "Component";
    comp_.area = "powertrain";
    comp_.source = "node";
    comp_.translation_id = "comp.motor";
    comp_.description = "Controls the electric motor";
    comp_.variant = "v2";
    comp_.tags = {"actuator"};
  }

  Component comp_;
};

// @verifies REQ_INTEROP_003
TEST_F(ComponentModelTest, ToJson_ContainsRequiredFields) {
  json j = comp_.to_json();

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["namespace"], "/powertrain");
  EXPECT_EQ(j["x-medkit"]["fqn"], "/powertrain/motor_controller");
  EXPECT_EQ(j["x-medkit"]["entityType"], "Component");
  EXPECT_EQ(j["x-medkit"]["area"], "powertrain");
  EXPECT_EQ(j["x-medkit"]["source"], "node");
}

TEST_F(ComponentModelTest, ToJson_ContainsOptionalFields) {
  json j = comp_.to_json();

  EXPECT_EQ(j["name"], "Motor Controller");
  EXPECT_EQ(j["translationId"], "comp.motor");
  EXPECT_EQ(j["x-medkit"]["description"], "Controls the electric motor");
  EXPECT_EQ(j["x-medkit"]["variant"], "v2");
  EXPECT_EQ(j["tags"].size(), 1);
}

TEST_F(ComponentModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = comp_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/components/motor_controller");
  EXPECT_FALSE(j.contains("type"));
}

TEST_F(ComponentModelTest, ToCapabilities_ContainsConfigurationsForNodes) {
  json j = comp_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Component");

  // Node-based components should have configurations capability
  EXPECT_TRUE(j.contains("configurations"));
  EXPECT_EQ(j["configurations"], "http://localhost:8080/api/v1/components/motor_controller/configurations");

  // `/hosts` is registered for components; "related-apps" named no route.
  EXPECT_TRUE(j.contains("hosts"));
  EXPECT_FALSE(j.contains("related-apps"));
}

TEST_F(ComponentModelTest, ToJson_OmitsExternalWhenUnsetOrFalse) {
  // Default: external unset -> field absent (stable wire shape).
  json j = comp_.to_json();
  EXPECT_FALSE(j["x-medkit"].contains("external"));

  comp_.external = false;  // explicit ROS component -> still absent
  json j2 = comp_.to_json();
  EXPECT_FALSE(j2["x-medkit"].contains("external"));
}

TEST_F(ComponentModelTest, ToJson_ExternalWhenTrue) {
  comp_.external = true;
  json j = comp_.to_json();
  EXPECT_EQ(j["x-medkit"]["external"], true);
}

TEST_F(ComponentModelTest, Equality_DistinguishesExternal) {
  Component other = comp_;
  ASSERT_EQ(comp_, other);
  other.external = true;
  EXPECT_NE(comp_, other);
}

// =============================================================================
// App Model Tests
// =============================================================================

class AppModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    app_.id = "nav2";
    app_.name = "Navigation 2";
    app_.source = "manifest";
    app_.translation_id = "app.nav2";
    app_.description = "Navigation stack for ROS 2";
    app_.tags = {"navigation", "autonomous"};

    // ROS binding
    App::RosBinding binding;
    binding.node_name = "nav2_controller";
    binding.namespace_pattern = "/nav2";
    app_.ros_binding = binding;

    // Runtime state
    app_.bound_fqn = "/nav2/controller_server";
    app_.is_online = true;
    app_.external = false;

    // Component relationship
    app_.component_id = "navigation_server";
    app_.depends_on = {"localization", "mapping"};
  }

  App app_;
};

// @verifies REQ_INTEROP_003
TEST_F(AppModelTest, ToJson_ContainsRequiredFields) {
  json j = app_.to_json();

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "App");
  EXPECT_EQ(j["x-medkit"]["source"], "manifest");
}

TEST_F(AppModelTest, ToJson_ContainsOptionalFields) {
  json j = app_.to_json();

  EXPECT_EQ(j["translationId"], "app.nav2");
  EXPECT_EQ(j["x-medkit"]["description"], "Navigation stack for ROS 2");
  EXPECT_EQ(j["tags"].size(), 2);
}

TEST_F(AppModelTest, ToJson_ContainsRosBinding) {
  json j = app_.to_json();

  ASSERT_TRUE(j["x-medkit"].contains("rosBinding"));
  EXPECT_EQ(j["x-medkit"]["rosBinding"]["nodeName"], "nav2_controller");
  EXPECT_EQ(j["x-medkit"]["rosBinding"]["namespace"], "/nav2");
}

TEST_F(AppModelTest, ToJson_ContainsRuntimeState) {
  json j = app_.to_json();

  EXPECT_EQ(j["x-medkit"]["boundFqn"], "/nav2/controller_server");
  EXPECT_EQ(j["x-medkit"]["isOnline"], true);
  // external is only included when true, so should not be present when false
  EXPECT_FALSE(j["x-medkit"].contains("external"));
}

TEST_F(AppModelTest, ToJson_ExternalWhenTrue) {
  app_.external = true;
  json j = app_.to_json();

  EXPECT_EQ(j["x-medkit"]["external"], true);
}

// @verifies REQ_INTEROP_105 REQ_INTEROP_106
TEST_F(AppModelTest, ToCapabilities_ContainsRelationshipLinks) {
  json j = app_.to_capabilities("http://localhost:8080/api/v1");

  // Both is-located-on and belongs-to are gated on the same component_id
  // condition - handler emits the same set in handle_get_app, so
  // to_capabilities must stay in sync for notification and snapshot
  // consumers that go through the model.
  EXPECT_EQ(j["is-located-on"], "http://localhost:8080/api/v1/components/navigation_server");
  EXPECT_EQ(j["belongs-to"], "http://localhost:8080/api/v1/apps/nav2/belongs-to");
  EXPECT_EQ(j["depends-on"], "http://localhost:8080/api/v1/apps/nav2/depends-on");
}

TEST_F(AppModelTest, ToCapabilities_OmitsRelationshipLinksWhenAppIsStandalone) {
  App standalone;
  standalone.id = "standalone";
  standalone.name = "Standalone";
  standalone.source = "manifest";
  // No component_id, no depends_on - mirrors discovery_handlers.cpp gating.

  json j = standalone.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_FALSE(j.contains("is-located-on"));
  EXPECT_FALSE(j.contains("belongs-to"));
  EXPECT_FALSE(j.contains("depends-on"));
}

TEST_F(AppModelTest, ToJson_OmitsEmptyOptionalFields) {
  App minimal;
  minimal.id = "test";
  minimal.name = "Test App";
  minimal.source = "manifest";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("rosBinding"));
  EXPECT_FALSE(j["x-medkit"].contains("boundFqn"));
}

TEST_F(AppModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = app_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/apps/nav2");
}

TEST_F(AppModelTest, ToCapabilities_ContainsExpectedResources) {
  // Add some topics and services to get data and operations capabilities
  app_.topics.publishes.push_back("/nav2/path");
  ros2_medkit_gateway::ServiceInfo svc;
  svc.name = "get_plan";
  svc.full_path = "/nav2/get_plan";
  svc.type = "nav2_msgs/srv/GetPlan";
  app_.services.push_back(svc);

  json j = app_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_TRUE(j.contains("data"));
  EXPECT_TRUE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
}

TEST_F(AppModelTest, ToCapabilities_OmitsDataWithoutTopics) {
  // No topics, no services
  App minimal;
  minimal.id = "test";
  minimal.name = "Test App";
  minimal.source = "manifest";

  json j = minimal.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_FALSE(j.contains("data"));
  EXPECT_FALSE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
  EXPECT_TRUE(j.contains("configurations"));
  EXPECT_TRUE(j.contains("x-medkit"));
}

// =============================================================================
// App effective_fqn() Tests
// =============================================================================

TEST_F(AppModelTest, EffectiveFqn_ReturnsBoundFqn) {
  EXPECT_EQ(app_.effective_fqn(), "/nav2/controller_server");
}

TEST_F(AppModelTest, EffectiveFqn_FallsBackToRosBinding) {
  app_.bound_fqn = std::nullopt;
  EXPECT_EQ(app_.effective_fqn(), "/nav2/nav2_controller");
}

TEST_F(AppModelTest, EffectiveFqn_EmptyNamespaceHasLeadingSlash) {
  app_.bound_fqn = std::nullopt;
  app_.ros_binding = App::RosBinding{"my_node", "", ""};
  EXPECT_EQ(app_.effective_fqn(), "/my_node");
}

TEST_F(AppModelTest, EffectiveFqn_WildcardReturnsEmpty) {
  app_.bound_fqn = std::nullopt;
  app_.ros_binding = App::RosBinding{"my_node", "*", ""};
  EXPECT_EQ(app_.effective_fqn(), "");
}

TEST_F(AppModelTest, EffectiveFqn_GlobPatternReturnsEmpty) {
  app_.bound_fqn = std::nullopt;
  app_.ros_binding = App::RosBinding{"my_node", "/nav**", ""};
  EXPECT_EQ(app_.effective_fqn(), "");
}

TEST_F(AppModelTest, EffectiveFqn_PrefixGlobReturnsEmpty) {
  app_.bound_fqn = std::nullopt;
  app_.ros_binding = App::RosBinding{"my_node", "prefix*", ""};
  EXPECT_EQ(app_.effective_fqn(), "");
}

TEST_F(AppModelTest, EffectiveFqn_NoBindingReturnsEmpty) {
  app_.bound_fqn = std::nullopt;
  app_.ros_binding = std::nullopt;
  EXPECT_EQ(app_.effective_fqn(), "");
}

// =============================================================================
// Function Model Tests
// =============================================================================

class FunctionModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    func_.id = "path_planning";
    func_.name = "Path Planning";
    func_.source = "manifest";
    func_.translation_id = "func.path_planning";
    func_.description = "Computes optimal paths to goal";
    func_.tags = {"planning", "core"};
    func_.hosts = {"nav2_planner_server"};
    func_.depends_on = {"localization"};
  }

  Function func_;
};

// @verifies REQ_INTEROP_003
TEST_F(FunctionModelTest, ToJson_ContainsRequiredFields) {
  json j = func_.to_json();

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Function");
  EXPECT_EQ(j["x-medkit"]["source"], "manifest");
}

TEST_F(FunctionModelTest, ToJson_ContainsOptionalFields) {
  json j = func_.to_json();

  EXPECT_EQ(j["translationId"], "func.path_planning");
  EXPECT_EQ(j["x-medkit"]["description"], "Computes optimal paths to goal");
  EXPECT_EQ(j["tags"].size(), 2);
  EXPECT_EQ(j["x-medkit"]["hosts"].size(), 1);
  EXPECT_EQ(j["x-medkit"]["dependsOn"].size(), 1);
}

TEST_F(FunctionModelTest, ToJson_OmitsEmptyOptionalFields) {
  Function minimal;
  minimal.id = "test";
  minimal.name = "Test Function";
  minimal.source = "manifest";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("hosts"));
  EXPECT_FALSE(j["x-medkit"].contains("dependsOn"));
}

TEST_F(FunctionModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = func_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/functions/path_planning");
  EXPECT_FALSE(j.contains("type"));
}

TEST_F(FunctionModelTest, ToCapabilities_ContainsExpectedResources) {
  json j = func_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_TRUE(j.contains("data"));
  EXPECT_TRUE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
  EXPECT_TRUE(j.contains("x-medkit"));
}

// =============================================================================
// Common Types Tests
// =============================================================================

TEST(CommonTypesTest, ServiceInfo_ToJson) {
  ros2_medkit_gateway::ServiceInfo service;
  service.name = "set_speed";
  service.full_path = "/motor/set_speed";
  service.type = "std_srvs/srv/SetBool";

  json j = service.to_json();

  EXPECT_EQ(j["name"], "set_speed");
  EXPECT_EQ(j["path"], "/motor/set_speed");
  EXPECT_EQ(j["type"], "std_srvs/srv/SetBool");
  EXPECT_EQ(j["kind"], "service");
}

TEST(CommonTypesTest, ActionInfo_ToJson) {
  ros2_medkit_gateway::ActionInfo action;
  action.name = "navigate_to_pose";
  action.full_path = "/navigate_to_pose";
  action.type = "nav2_msgs/action/NavigateToPose";

  json j = action.to_json();

  EXPECT_EQ(j["name"], "navigate_to_pose");
  EXPECT_EQ(j["path"], "/navigate_to_pose");
  EXPECT_EQ(j["type"], "nav2_msgs/action/NavigateToPose");
  EXPECT_EQ(j["kind"], "action");
}

TEST(CommonTypesTest, ComponentTopics_ToJson) {
  ros2_medkit_gateway::ComponentTopics topics;
  topics.publishes.push_back("/odom");
  topics.subscribes.push_back("/cmd_vel");

  json j = topics.to_json();

  EXPECT_EQ(j["publishes"].size(), 1);
  EXPECT_EQ(j["subscribes"].size(), 1);
  EXPECT_EQ(j["publishes"][0], "/odom");
  EXPECT_EQ(j["subscribes"][0], "/cmd_vel");
}

TEST(CommonTypesTest, QosProfile_ToJson) {
  ros2_medkit_gateway::QosProfile qos;
  qos.reliability = "reliable";
  qos.durability = "volatile";
  qos.history = "keep_last";
  qos.depth = 10;
  qos.liveliness = "automatic";

  json j = qos.to_json();

  EXPECT_EQ(j["reliability"], "reliable");
  EXPECT_EQ(j["durability"], "volatile");
  EXPECT_EQ(j["history"], "keep_last");
  EXPECT_EQ(j["depth"], 10);
  EXPECT_EQ(j["liveliness"], "automatic");
}

// =============================================================================
// sorted_contributors() ordering - local first, then peers alphabetical
// =============================================================================
//
// Regression guard for the local-first + alphabetical sort. Feeds reverse- and
// unsorted input so a flipped comparator (the failure this guards against) is
// caught; asserting only on already-sorted input would not detect it. This
// coverage previously lived in the deleted test_x_medkit.cpp.

// @verifies REQ_INTEROP_003
TEST(CommonTypesTest, SortedContributors_LocalFirstThenAlphabetical) {
  EXPECT_EQ(ros2_medkit_gateway::sorted_contributors({"peer:zulu", "peer:alpha", "local"}),
            (std::vector<std::string>{"local", "peer:alpha", "peer:zulu"}));
}

// @verifies REQ_INTEROP_003
TEST(CommonTypesTest, SortedContributors_NoLocalIsAlphabetical) {
  EXPECT_EQ(ros2_medkit_gateway::sorted_contributors({"peer:charlie", "peer:alpha", "peer:bravo"}),
            (std::vector<std::string>{"peer:alpha", "peer:bravo", "peer:charlie"}));
}

// @verifies REQ_INTEROP_003
TEST(CommonTypesTest, SortedContributors_EmptyStaysEmpty) {
  EXPECT_TRUE(ros2_medkit_gateway::sorted_contributors({}).empty());
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
