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

#include "ros2_medkit_gateway/core/openapi/route_descriptions.hpp"

using namespace ros2_medkit_gateway::openapi;

// Test helper in same namespace as RouteDescriptions to match friend declaration
namespace ros2_medkit_gateway::openapi {
class RouteDescriptionsTestAccess {
 public:
  static nlohmann::json to_json(const RouteDescriptions & desc) {
    return desc.to_json();
  }
  static bool empty(const RouteDescriptions & desc) {
    return desc.empty();
  }
};
}  // namespace ros2_medkit_gateway::openapi

using ros2_medkit_gateway::openapi::RouteDescriptionsTestAccess;

// @verifies REQ_INTEROP_002
TEST(RouteDescriptionsTest, EmptyBuilderProducesEmptyDescriptions) {
  auto desc = RouteDescriptionBuilder{}.build();
  EXPECT_TRUE(RouteDescriptionsTestAccess::empty(desc));
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  EXPECT_TRUE(json.is_object());
  EXPECT_TRUE(json.empty());
}

// @verifies REQ_INTEROP_002
TEST(RouteDescriptionsTest, SingleGetEndpoint) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-beacon/{entity_id}")
      .summary("Get beacon data")
      .get(OperationDesc()
               .description("Returns beacon discovery data")
               .path_param("entity_id", "Entity identifier")
               .response(200, SchemaDesc::object()
                                  .property("beacons", SchemaDesc::array(SchemaDesc::string()))
                                  .required({"beacons"}))
               .response(404, SchemaDesc::response_ref("GenericError")));
  auto desc = b.build();
  EXPECT_FALSE(RouteDescriptionsTestAccess::empty(desc));

  auto json = RouteDescriptionsTestAccess::to_json(desc);
  ASSERT_TRUE(json.contains("/x-medkit-beacon/{entity_id}"));
  auto & path = json["/x-medkit-beacon/{entity_id}"];
  ASSERT_TRUE(path.contains("get"));
  EXPECT_EQ(path["get"]["summary"], "Get beacon data");
  EXPECT_EQ(path["get"]["responses"]["200"]["content"]["application/json"]["schema"]["type"], "object");
  EXPECT_TRUE(path["get"]["responses"].contains("404"));
}

// @verifies REQ_INTEROP_002
TEST(RouteDescriptionsTest, MultipleEndpoints) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-foo").summary("Foo").get(OperationDesc().response(200, SchemaDesc::string()));
  b.add("/x-medkit-bar").summary("Bar").post(OperationDesc().response(201, SchemaDesc::string()));
  auto desc = b.build();
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  EXPECT_EQ(json.size(), 2);
  EXPECT_TRUE(json.contains("/x-medkit-foo"));
  EXPECT_TRUE(json.contains("/x-medkit-bar"));
}

// @verifies REQ_INTEROP_002
TEST(RouteDescriptionsTest, PostWithRequestBody) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-action")
      .summary("Execute action")
      .post(OperationDesc()
                .request_body(SchemaDesc::object().property("command", SchemaDesc::string()))
                .response(202, SchemaDesc::object().property("status", SchemaDesc::string())));
  auto desc = b.build();
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  auto & post = json["/x-medkit-action"]["post"];
  EXPECT_TRUE(post.contains("requestBody"));
  EXPECT_TRUE(post["requestBody"]["content"]["application/json"].contains("schema"));
  EXPECT_TRUE(post["requestBody"]["required"].get<bool>());
}

// @verifies REQ_INTEROP_002
TEST(RouteDescriptionsTest, SchemaTypes) {
  auto str = SchemaDesc::string().to_json();
  EXPECT_EQ(str["type"], "string");
  auto num = SchemaDesc::number().to_json();
  EXPECT_EQ(num["type"], "number");
  auto integer = SchemaDesc::integer().to_json();
  EXPECT_EQ(integer["type"], "integer");
  auto boolean = SchemaDesc::boolean().to_json();
  EXPECT_EQ(boolean["type"], "boolean");
  auto arr = SchemaDesc::array(SchemaDesc::string()).to_json();
  EXPECT_EQ(arr["type"], "array");
  EXPECT_EQ(arr["items"]["type"], "string");
  auto obj = SchemaDesc::object()
                 .property("name", SchemaDesc::string())
                 .property("count", SchemaDesc::integer())
                 .required({"name"})
                 .to_json();
  EXPECT_EQ(obj["type"], "object");
  EXPECT_TRUE(obj["properties"].contains("name"));
  EXPECT_EQ(obj["required"][0], "name");
}

// The identity fields the document contract requires of every operation.
// Without them a folded plugin operation fails
// `test_openapi_contract::test_every_operation_is_identified` and
// `test_every_tag_used_is_declared`.
TEST(RouteDescriptionsTest, OperationCarriesTagOperationIdAndRole) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-thing/{entity_id}")
      .summary("Get thing")
      .get(OperationDesc()
               .tag("Thing")
               .operation_id("getThing")
               .requires_role("admin")
               .description("Returns the thing.")
               .path_param("entity_id", "Entity identifier")
               .response(200, SchemaDesc::object().property("id", SchemaDesc::string()), "The thing")
               .error_response(404, "GenericError"));
  auto json = RouteDescriptionsTestAccess::to_json(b.build());
  auto & get = json["/x-medkit-thing/{entity_id}"]["get"];

  EXPECT_EQ(get["tags"], nlohmann::json::array({"Thing"}));
  EXPECT_EQ(get["operationId"], "getThing");
  EXPECT_EQ(get["summary"], "Get thing");
  EXPECT_EQ(get["description"], "Returns the thing.");
  // Role as the scope of a `bearerAuth` requirement - the shape a generated
  // client and `test_openapi_contract` both read.
  EXPECT_EQ(get["security"], nlohmann::json::array({{{"bearerAuth", nlohmann::json::array({"admin"})}}}));
  // An error status is a $ref to the shared component, never an inline body.
  EXPECT_EQ(get["responses"]["404"]["$ref"], "#/components/responses/GenericError");
  EXPECT_EQ(get["responses"]["200"]["description"], "The thing");
}

// `security: []` is the OpenAPI spelling of "no token needed", and it is
// distinct from declaring no security at all: the latter falls back to the
// document-level requirement.
TEST(RouteDescriptionsTest, PublicRouteDeclaresAnEmptySecurityRequirement) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-open").summary("Open").get(OperationDesc().public_route().response(200, SchemaDesc::string()));
  auto json = RouteDescriptionsTestAccess::to_json(b.build());
  auto & get = json["/x-medkit-open"]["get"];
  ASSERT_TRUE(get.contains("security"));
  EXPECT_TRUE(get["security"].is_array());
  EXPECT_TRUE(get["security"].empty());
}

TEST(RouteDescriptionsTest, OperationWithoutASecurityDeclarationOmitsTheKey) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-quiet").summary("Quiet").get(OperationDesc().response(200, SchemaDesc::string()));
  auto json = RouteDescriptionsTestAccess::to_json(b.build());
  EXPECT_FALSE(json["/x-medkit-quiet"]["get"].contains("security"));
}

TEST(RouteDescriptionsTest, SchemaDescriptionEnumAndNullability) {
  auto described = SchemaDesc::string().description("What it means").to_json();
  EXPECT_EQ(described["description"], "What it means");

  auto enumerated = SchemaDesc::string().enum_values({"a", "b"}).to_json();
  EXPECT_EQ(enumerated["enum"], nlohmann::json::array({"a", "b"}));

  // `or_null` widens in place, and a description written after it lands on the
  // wrapper rather than on the non-null branch - which is where a client that
  // does not walk `anyOf` will look.
  auto nullable = SchemaDesc::number().or_null().description("Null until measured").to_json();
  EXPECT_FALSE(nullable.contains("type"));
  EXPECT_EQ(nullable["anyOf"][0]["type"], "number");
  EXPECT_EQ(nullable["anyOf"][1]["type"], "null");
  EXPECT_EQ(nullable["description"], "Null until measured");
}
