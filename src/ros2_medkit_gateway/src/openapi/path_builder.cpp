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

#include "path_builder.hpp"

#include "schema_builder.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

PathBuilder::PathBuilder(const SchemaBuilder & schema_builder) : schema_builder_(schema_builder) {
}

// -----------------------------------------------------------------------------
// Data item
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_data_item(const std::string & /*entity_path*/, const TopicData & topic) const {
  nlohmann::json path_item;

  // GET - always available
  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Data"});
  get_op["summary"] = "Read data: " + topic.name;
  get_op["description"] =
      "Read current value of topic " + topic.name + (topic.type.empty() ? "." : " (type: " + topic.type + ").");
  // No 200 body. The gateway does not return the bare message: `GET .../data/{data_id}`
  // answers the `DataValue` envelope, which is what the templated sibling declares and
  // what `adopt_projected_framework` copies in. Building one here published a second,
  // contradictory answer for one route - and with `TopicData::type` never populated it
  // was an anonymous `x-medkit-schema-unavailable` object replacing a named `$ref`.

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // PUT - only if writable (subscribe or both)
  if (topic.direction == "subscribe" || topic.direction == "both") {
    nlohmann::json put_op;
    put_op["tags"] = nlohmann::json::array({"Data"});
    put_op["summary"] = "Write data: " + topic.name;
    put_op["description"] = "Publish a value to topic " + topic.name + ".";
    // The envelope `DataHandlers::put_data_item` actually reads, and only when
    // this builder has something the templated sibling's `$ref: DataWriteRequest`
    // does not: the schema of *this* topic's payload under `data`. With the type
    // unknown - which is every topic on every gateway today, see the class
    // comment - the two say exactly the same thing and the named `$ref` says it
    // better, so the sibling's is inherited instead.
    if (!topic.type.empty()) {
      put_op["requestBody"]["required"] = true;
      put_op["requestBody"]["content"]["application/json"]["schema"] =
          nlohmann::json{{"type", "object"},
                         {"required", nlohmann::json::array({"type", "data"})},
                         {"properties",
                          {{"type",
                            {{"type", "string"},
                             {"const", topic.type},
                             {"description", "ROS 2 message type of the topic being written."}}},
                           {"data", schema_builder_.from_ros_msg(topic.type)}}}};
    }

    auto put_errors = error_responses();
    for (auto & [code, val] : put_errors.items()) {
      put_op["responses"][code] = val;
    }

    path_item["put"] = std::move(put_op);
  }

  // SOVD extensions
  path_item["x-sovd-data-category"] = "currentData";
  path_item["x-sovd-cyclic-subscription-supported"] = true;
  path_item["x-sovd-name"] = topic.name;

  return path_item;
}

// -----------------------------------------------------------------------------
// Operation item (service)
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_operation_item(const std::string & /*entity_path*/,
                                                 const ServiceInfo & service) const {
  nlohmann::json path_item;

  // GET - get operation details and result
  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Operations"});
  get_op["summary"] = "Get operation: " + service.name;
  get_op["description"] = "Get details and last result of service " + service.name + " (type: " + service.type + ").";
  // No 200 body, for the same reason as the data item: `GET .../operations/{operation_id}`
  // answers the `OperationDetail` envelope - measured on the wire as `{"item": {...}}` -
  // not the ROS service or action response. Publishing `from_ros_srv_response` here put a
  // second, contradictory 200 on one route in one document. The service response schema
  // is genuinely useful, but it belongs to the execution result under
  // `POST .../{operation_id}/executions`, which no builder writes today.

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // No POST. The gateway registers `GET /{entity}/operations/{operation_id}`
  // and nothing else at that key - execution goes through
  // `POST /{entity}/operations/{operation_id}/executions` - so the POST this
  // builder used to publish here named an operation the gateway does not serve.
  // Measured: `POST /apps/calibration/operations/calibrate` answers 404 while
  // `POST /apps/calibration/operations/calibrate/executions` answers 200. Its
  // request schema was the bare service-request shape, which compounded the
  // problem rather than causing it; correcting the body would have left a 404
  // operation published with a better-looking body.
  path_item["x-sovd-name"] = service.name;
  return path_item;
}

// -----------------------------------------------------------------------------
// Operation item (action)
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_operation_item(const std::string & /*entity_path*/, const ActionInfo & action) const {
  nlohmann::json path_item;

  // GET - get action status/result
  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Operations"});
  get_op["summary"] = "Get action status: " + action.name;
  get_op["description"] = "Get status and result of action " + action.name + " (type: " + action.type + ").";

  // No 200 body, for the same reason as the data item: `GET .../operations/{operation_id}`
  // answers the `OperationDetail` envelope - measured on the wire as `{"item": {...}}` -
  // not the ROS service or action response. Publishing `from_ros_srv_response` here put a
  // second, contradictory 200 on one route in one document. The service response schema
  // is genuinely useful, but it belongs to the execution result under
  // `POST .../{operation_id}/executions`, which no builder writes today.

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // No POST. The gateway registers `GET /{entity}/operations/{operation_id}`
  // and nothing else at that key - execution goes through
  // `POST /{entity}/operations/{operation_id}/executions` - so the POST this
  // builder used to publish here named an operation the gateway does not serve.
  // Measured: `POST /apps/calibration/operations/calibrate` answers 404 while
  // `POST /apps/calibration/operations/calibrate/executions` answers 200. Its
  // request schema was the bare service-request shape, which compounded the
  // problem rather than causing it; correcting the body would have left a 404
  // operation published with a better-looking body.
  path_item["x-sovd-name"] = action.name;
  path_item["x-sovd-asynchronous-execution"] = true;
  return path_item;
}

// -----------------------------------------------------------------------------
// Error responses
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::error_responses() const {
  nlohmann::json errors;

  errors["400"]["description"] = "Bad request";
  errors["400"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["404"]["description"] = "Not found";
  errors["404"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["500"]["description"] = "Internal server error";
  errors["500"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  return errors;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
