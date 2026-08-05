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

#pragma once

#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

/// Wraps JsonSerializer::get_schema() with OpenAPI-specific formatting,
/// graceful degradation, and static schema factories for SOVD types.
class SchemaBuilder {
 public:
  /// Constructs SchemaBuilder with its own JsonSerializer instance.
  /// JsonSerializer is stateless and thread-safe - no sharing needed.
  SchemaBuilder();

  // Runtime schema generation from ROS 2 types

  /// Generate JSON Schema for a ROS 2 message type.
  /// Returns a generic object schema with x-medkit-schema-unavailable on failure.
  nlohmann::json from_ros_msg(const std::string & type_string) const;

  /// Generate JSON Schema for a ROS 2 service request type.
  /// @param srv_type Service type string, e.g. "std_srvs/srv/SetBool"
  nlohmann::json from_ros_srv_request(const std::string & srv_type) const;

  /// Generate JSON Schema for a ROS 2 service response type.
  /// @param srv_type Service type string, e.g. "std_srvs/srv/SetBool"
  nlohmann::json from_ros_srv_response(const std::string & srv_type) const;

  // Static SOVD schemas

  /// SOVD GenericError schema (7.4.2)
  static nlohmann::json generic_error();

  /// Wrap an item schema in a SOVD collection response: {"items": [item_schema]}
  static nlohmann::json items_wrapper(const nlohmann::json & item_schema);

  /// Generic object schema (for dynamic ROS 2 message payloads)
  static nlohmann::json generic_object_schema();

  /// Returns a $ref JSON object pointing to a named component schema.
  static nlohmann::json ref(const std::string & schema_name);

  /// Wraps a $ref in an items collection.
  static nlohmann::json items_wrapper_ref(const std::string & schema_name);

  /// Returns all named schemas for registration in components/schemas.
  /// Key = schema name (becomes the type name in generated clients).
  /// List-type schemas use internal $ref to avoid duplicate types.
  static const std::map<std::string, nlohmann::json> & component_schemas();

 private:
  ros2_medkit_serialization::JsonSerializer serializer_;  // Owns its own instance
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
