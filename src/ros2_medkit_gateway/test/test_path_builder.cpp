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
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
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

TEST_F(PathBuilderTest, DataItemSchemaFromRosType) {
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  // std_msgs/msg/Float32 has a "data" field
  EXPECT_EQ(schema["type"], "object");
  EXPECT_TRUE(schema.contains("properties"));
}

// =============================================================================
// Operation item (service) tests
// =============================================================================

TEST_F(PathBuilderTest, ServiceOperationHasGetAndPost) {
  // @verifies REQ_INTEROP_002
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("post"));
}

TEST_F(PathBuilderTest, ServiceOperationPostHasRequestBody) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  ASSERT_TRUE(result["post"].contains("requestBody"));
  EXPECT_TRUE(result["post"]["requestBody"]["required"].get<bool>());
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

TEST_F(PathBuilderTest, ActionOperationHasGetAndPost) {
  // @verifies REQ_INTEROP_002
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("post"));
}

TEST_F(PathBuilderTest, ActionOperationIsAsynchronous) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("x-sovd-asynchronous-execution"));
  EXPECT_TRUE(result["x-sovd-asynchronous-execution"].get<bool>());
}

TEST_F(PathBuilderTest, ActionOperationPostReturns202) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  EXPECT_TRUE(result["post"]["responses"].contains("202"));
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

TEST_F(PathBuilderTest, ErrorResponsesWithAuth) {
  PathBuilder auth_builder(schema_builder_, true);
  auto errors = auth_builder.error_responses();
  EXPECT_TRUE(errors.contains("400"));
  EXPECT_TRUE(errors.contains("404"));
  EXPECT_TRUE(errors.contains("500"));
  EXPECT_TRUE(errors.contains("401"));
  EXPECT_TRUE(errors.contains("403"));
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
