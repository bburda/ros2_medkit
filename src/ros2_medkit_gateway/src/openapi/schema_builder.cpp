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

#include "schema_builder.hpp"

#include "ros2_medkit_gateway/dto/registry.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

SchemaBuilder::SchemaBuilder() : serializer_() {
}

nlohmann::json SchemaBuilder::from_ros_msg(const std::string & type_string) const {
  try {
    return serializer_.get_schema(type_string);
  } catch (const std::exception &) {
    // Expected for unknown ROS 2 message types - graceful degradation
    return {{"type", "object"}, {"x-medkit-schema-unavailable", true}};
  } catch (...) {
    return {{"type", "object"}, {"x-medkit-schema-unavailable", true}};
  }
}

nlohmann::json SchemaBuilder::from_ros_srv_request(const std::string & srv_type) const {
  // Service request type: "pkg/srv/Name" -> "pkg/srv/Name_Request"
  return from_ros_msg(srv_type + "_Request");
}

nlohmann::json SchemaBuilder::from_ros_srv_response(const std::string & srv_type) const {
  // Service response type: "pkg/srv/Name" -> "pkg/srv/Name_Response"
  return from_ros_msg(srv_type + "_Response");
}

nlohmann::json SchemaBuilder::generic_error() {
  return dto::SchemaWriter<dto::GenericError>::schema();
}

nlohmann::json SchemaBuilder::items_wrapper(const nlohmann::json & item_schema) {
  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", item_schema}}}}},
          {"required", {"items"}}};
}

nlohmann::json SchemaBuilder::generic_object_schema() {
  return {{"type", "object"}};
}

nlohmann::json SchemaBuilder::ref(const std::string & schema_name) {
  return {{"$ref", "#/components/schemas/" + schema_name}};
}

nlohmann::json SchemaBuilder::items_wrapper_ref(const std::string & schema_name) {
  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", ref(schema_name)}}}}},
          {"required", {"items"}}};
}

const std::map<std::string, nlohmann::json> & SchemaBuilder::component_schemas() {
  static const std::map<std::string, nlohmann::json> schemas = []() {
    // All domain schemas come from the DTO registry (dto/registry.hpp).
    // PR-403 commit 27 dropped the last hand-written survivor
    // (OperationExecutionList) - it is now produced by
    // `Collection<ExecutionId>` via the standard DTO schema path.
    std::map<std::string, nlohmann::json> m;
    auto dto_schemas = dto::collect_component_schemas();
    for (auto & [name, schema] : dto_schemas.items()) {
      m[name] = schema;
    }
    return m;
  }();
  return schemas;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
