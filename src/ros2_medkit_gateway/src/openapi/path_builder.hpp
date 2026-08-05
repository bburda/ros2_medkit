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

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

class SchemaBuilder;

/// Builds the OpenAPI PathItem for one *discovered* resource - a single ROS 2
/// topic, service or action.
///
/// What is left here after the `<entity-path>/docs` sub-documents became a
/// projection of `RouteRegistry::to_openapi_paths()`: the route registry holds
/// `/apps/{app_id}/data/{data_id}` with a payload schema that has to cover
/// every topic at once, and it has no way to learn that *this* entity's
/// `temperature` carries a `std_msgs/msg/Float32` or that a publish-only topic
/// cannot be written. That comes from the entity cache, so these three
/// builders do, and nothing else in this class does.
///
/// A path item built here is not a projection of anything and so does not
/// carry what a projected operation carries - the framework-level error set,
/// the declared role. Everything the registry *can* answer is answered by the
/// projection; adding a fourth builder here is how the hand-written half grows
/// back.
class PathBuilder {
 public:
  explicit PathBuilder(const SchemaBuilder & schema_builder, bool auth_enabled = false);

  nlohmann::json build_data_item(const std::string & entity_path, const TopicData & topic) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ServiceInfo & service) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ActionInfo & action) const;

  /// The 400/404/500 set every item path carries, plus 401/403 when
  /// authentication is on.
  nlohmann::json error_responses() const;

 private:
  const SchemaBuilder & schema_builder_;
  bool auth_enabled_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
