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
/// A path item built here is not a projection of anything, so it builds only
/// what the entity cache knows and the registration cannot: the concrete path
/// key, the `x-sovd-*` extensions, and a request body where the topic's ROS 2
/// type is available. Everything else - the declared role, every response
/// including the 2xx, the lock contract, the middleware statuses - is copied
/// in afterwards from the projected sibling by
/// `CapabilityGenerator::adopt_projected_framework`, because the built item
/// and that sibling are the *same route*.
///
/// **`TopicData::type` is empty on every gateway today.** All four sites that
/// build one push `{topic, "", direction}`
/// (`thread_safe_entity_cache.cpp`), so the `!topic.type.empty()` branch below
/// never runs outside unit tests and a built data item currently contributes
/// its path and nothing else. That is worth knowing before adding to it.
///
/// Do not restate framework facts here, and do not build a response body: the
/// gateway envelopes every read (`DataValue`, `OperationDetail`), so a body
/// derived from the ROS type is a second, contradictory answer for one route -
/// which is what these items used to publish, alongside an inline
/// `GenericError` for a 401 the middleware answers in the RFC 6749 shape.
class PathBuilder {
 public:
  explicit PathBuilder(const SchemaBuilder & schema_builder);

  nlohmann::json build_data_item(const std::string & entity_path, const TopicData & topic) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ServiceInfo & service) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ActionInfo & action) const;

  /// The handler-level 400/404/500 every item path carries.
  ///
  /// Deliberately *not* 401/403: those come from the auth middleware, which
  /// decides per gateway configuration whether it answers at all and uses the
  /// RFC 6749 body shape rather than the SOVD `GenericError` these carry.
  /// `adopt_projected_framework` supplies them, together with 409, 416 and
  /// anything else the templated sibling declares.
  nlohmann::json error_responses() const;

 private:
  const SchemaBuilder & schema_builder_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
