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

PathBuilder::PathBuilder(const SchemaBuilder & schema_builder, bool auth_enabled)
  : schema_builder_(schema_builder), auth_enabled_(auth_enabled) {
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
  get_op["description"] = "Read current value of topic " + topic.name + " (type: " + topic.type + ").";
  get_op["responses"]["200"]["description"] = "Current topic value";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_msg(topic.type);

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
    put_op["requestBody"]["required"] = true;
    put_op["requestBody"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_msg(topic.type);
    put_op["responses"]["200"]["description"] = "Value written successfully";
    put_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_object_schema();

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
  get_op["responses"]["200"]["description"] = "Operation details";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_srv_response(service.type);

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - execute operation
  nlohmann::json post_op;
  post_op["tags"] = nlohmann::json::array({"Operations"});
  post_op["summary"] = "Execute operation: " + service.name;
  post_op["description"] = "Execute service " + service.name + " synchronously.";
  post_op["requestBody"]["required"] = true;
  post_op["requestBody"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_srv_request(service.type);
  post_op["responses"]["200"]["description"] = "Operation result";
  post_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_srv_response(service.type);

  auto post_errors = error_responses();
  for (auto & [code, val] : post_errors.items()) {
    post_op["responses"][code] = val;
  }

  path_item["post"] = std::move(post_op);
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

  // Action goal result type: "pkg/action/Name" -> "pkg/action/Name_GetResult_Response"
  get_op["responses"]["200"]["description"] = "Action status";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_msg(action.type + "_GetResult_Response");

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - execute action (asynchronous)
  nlohmann::json post_op;
  post_op["tags"] = nlohmann::json::array({"Operations"});
  post_op["summary"] = "Execute action: " + action.name;
  post_op["description"] = "Start action " + action.name + " asynchronously.";
  post_op["requestBody"]["required"] = true;
  // Action goal type: "pkg/action/Name" -> "pkg/action/Name_SendGoal_Request"
  post_op["requestBody"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_msg(action.type + "_SendGoal_Request");
  post_op["responses"]["202"]["description"] = "Action accepted";
  post_op["responses"]["202"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("OperationExecution");

  auto post_errors = error_responses();
  for (auto & [code, val] : post_errors.items()) {
    post_op["responses"][code] = val;
  }

  path_item["post"] = std::move(post_op);
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

  if (auth_enabled_) {
    errors["401"]["description"] = "Unauthorized - authentication required";
    errors["401"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

    errors["403"]["description"] = "Forbidden - insufficient permissions";
    errors["403"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();
  }

  return errors;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
