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

#include <string>

#include "../src/openapi/path_builder.hpp"
#include "../src/openapi/schema_builder.hpp"

using ros2_medkit_gateway::ActionInfo;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_gateway::TopicData;
using ros2_medkit_gateway::openapi::PathBuilder;
using ros2_medkit_gateway::openapi::SchemaBuilder;

class PathBuilderTest : public ::testing::Test {
 protected:
  SchemaBuilder schema_builder_;
  PathBuilder path_builder_{schema_builder_};
};

// =============================================================================
// Data item tests
// =============================================================================

TEST_F(PathBuilderTest, DataItemGetAlwaysPresent) {
  // @verifies REQ_INTEROP_002
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  ASSERT_TRUE(result.contains("get"));
  // No 200: the gateway answers the `DataValue` envelope, not the bare
  // message, so the read body is the projected sibling's to state.
  // `adopt_projected_framework` copies it in.
  EXPECT_FALSE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, DataItemPutForSubscribeTopic) {
  // @verifies REQ_INTEROP_002
  TopicData topic{"command", "std_msgs/msg/String", "subscribe"};
  auto result = path_builder_.build_data_item("apps/actuator", topic);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("put"));
  EXPECT_TRUE(result["put"].contains("requestBody"));
  EXPECT_TRUE(result["put"]["requestBody"]["required"].get<bool>());
}

TEST_F(PathBuilderTest, DataItemPutForBothDirection) {
  TopicData topic{"sensor_data", "std_msgs/msg/Float32", "both"};
  auto result = path_builder_.build_data_item("apps/node", topic);
  ASSERT_TRUE(result.contains("put"));
}

TEST_F(PathBuilderTest, DataItemNoPutForPublishOnly) {
  TopicData topic{"output", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_FALSE(result.contains("put"));
}

TEST_F(PathBuilderTest, DataItemHasSovdExtensions) {
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  EXPECT_EQ(result["x-sovd-data-category"], "currentData");
  EXPECT_TRUE(result["x-sovd-cyclic-subscription-supported"].get<bool>());
  EXPECT_EQ(result["x-sovd-name"], "temperature");
}

TEST_F(PathBuilderTest, DataItemWriteBodyOmittedWhenTheTypeIsUnknown) {
  // `TopicData::type` is empty on every gateway today - every construction site
  // in `thread_safe_entity_cache.cpp` pushes `{topic, "", direction}` - so this
  // is the branch production actually takes, and it had no coverage at all.
  // With nothing to say beyond `DataWriteRequest`, the builder says nothing and
  // the projected sibling's named `$ref` is inherited instead.
  TopicData topic{"command", "", "subscribe"};
  auto result = path_builder_.build_data_item("apps/actuator", topic);
  ASSERT_TRUE(result.contains("put"));
  EXPECT_FALSE(result["put"].contains("requestBody"));
  // And the description does not render an empty type as "(type: )".
  EXPECT_EQ(result["get"]["description"], "Read current value of topic command.");
}

// =============================================================================
// Operation item (service) tests
// =============================================================================

TEST_F(PathBuilderTest, ServiceOperationHasGetOnly) {
  // @verifies REQ_INTEROP_002
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  ASSERT_TRUE(result.contains("get"));
  // The gateway registers no POST at `/{entity}/operations/{operation_id}` -
  // execution is `POST .../{operation_id}/executions` - so publishing one here
  // named an operation that answers 404.
  EXPECT_FALSE(result.contains("post"));
  // And no 200: `GET .../operations/{operation_id}` answers `OperationDetail`
  // (`{"item": {...}}` on the wire), never the ROS service response. Building
  // one from `from_ros_srv_response` put two contradictory bodies on one route.
  EXPECT_FALSE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, DataItemPutBodyIsTheWriteEnvelopeNotTheBareMessage) {
  // The handler reads `DataWriteRequest{type, data}`. Publishing the bare
  // message schema was answered `400 "type: missing required field"` on the
  // wire, so the shape - not merely the status set - has to match.
  TopicData topic{"command", "std_msgs/msg/String", "subscribe"};
  auto result = path_builder_.build_data_item("apps/actuator", topic);
  auto schema = result["put"]["requestBody"]["content"]["application/json"]["schema"];
  EXPECT_EQ(schema["type"], "object");
  EXPECT_EQ(schema["required"], nlohmann::json::array({"type", "data"}));
  EXPECT_EQ(schema["properties"]["type"]["const"], "std_msgs/msg/String");
  // `data` carries what the bare schema used to sit at the top level.
  EXPECT_EQ(schema["properties"]["data"]["type"], "object");
  EXPECT_TRUE(schema["properties"]["data"].contains("properties"));
}

TEST_F(PathBuilderTest, ServiceOperationHasSovdName) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  EXPECT_EQ(result["x-sovd-name"], "calibrate");
}

TEST_F(PathBuilderTest, ServiceOperationNotAsynchronous) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  EXPECT_FALSE(result.contains("x-sovd-asynchronous-execution"));
}

// =============================================================================
// Operation item (action) tests
// =============================================================================

TEST_F(PathBuilderTest, ActionOperationHasGetOnly) {
  // @verifies REQ_INTEROP_002
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_FALSE(result.contains("post"));
}

TEST_F(PathBuilderTest, ActionOperationIsAsynchronous) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("x-sovd-asynchronous-execution"));
  EXPECT_TRUE(result["x-sovd-asynchronous-execution"].get<bool>());
}

TEST_F(PathBuilderTest, ActionOperationDeclaresNoResponseBodyOfItsOwn) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  EXPECT_FALSE(result["get"]["responses"].contains("200"));
  EXPECT_FALSE(result["get"]["responses"].contains("202"));
}

TEST_F(PathBuilderTest, ActionOperationHasSovdName) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  EXPECT_EQ(result["x-sovd-name"], "navigate");
}

// =============================================================================
// Error responses tests
// =============================================================================

TEST_F(PathBuilderTest, ErrorResponsesWithoutAuth) {
  // @verifies REQ_INTEROP_002
  auto errors = path_builder_.error_responses();
  EXPECT_TRUE(errors.contains("400"));
  EXPECT_TRUE(errors.contains("404"));
  EXPECT_TRUE(errors.contains("500"));
  EXPECT_FALSE(errors.contains("401"));
  EXPECT_FALSE(errors.contains("403"));
}

TEST_F(PathBuilderTest, ErrorResponsesNeverCarryTheMiddlewareStatuses) {
  // 401/403 are the auth middleware's, answered in the RFC 6749 shape and only
  // where the policy enforces. This builder cannot know either, so it declares
  // neither; `CapabilityGenerator::adopt_projected_framework` copies them in
  // from the templated sibling, which does know.
  auto errors = path_builder_.error_responses();
  EXPECT_FALSE(errors.contains("401"));
  EXPECT_FALSE(errors.contains("403"));
  EXPECT_FALSE(errors.contains("409"));
  EXPECT_FALSE(errors.contains("416"));
}

TEST_F(PathBuilderTest, ErrorResponsesUseGenericErrorSchema) {
  auto errors = path_builder_.error_responses();
  for (const auto & [code, resp] : errors.items()) {
    ASSERT_TRUE(resp.contains("content")) << "Error " << code << " missing content";
    auto schema = resp["content"]["application/json"]["schema"];
    EXPECT_EQ(schema["type"], "object") << "Error " << code << " schema type mismatch";
    EXPECT_TRUE(schema["properties"].contains("error_code")) << "Error " << code << " missing error_code";
    EXPECT_TRUE(schema["properties"].contains("message")) << "Error " << code << " missing message";
  }
}
