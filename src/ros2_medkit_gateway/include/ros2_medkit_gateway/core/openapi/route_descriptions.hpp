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
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

class CapabilityGenerator;  // Forward declaration for friend access

// =============================================================================
// SchemaDesc - describes an OpenAPI schema object
// =============================================================================

class SchemaDesc {
 public:
  // Factory methods for primitive types
  static SchemaDesc string() {
    SchemaDesc s;
    s.json_["type"] = "string";
    return s;
  }

  static SchemaDesc number() {
    SchemaDesc s;
    s.json_["type"] = "number";
    return s;
  }

  static SchemaDesc integer() {
    SchemaDesc s;
    s.json_["type"] = "integer";
    return s;
  }

  static SchemaDesc boolean() {
    SchemaDesc s;
    s.json_["type"] = "boolean";
    return s;
  }

  // Factory method for arrays
  static SchemaDesc array(SchemaDesc items) {
    SchemaDesc s;
    s.json_["type"] = "array";
    s.json_["items"] = std::move(items.json_);
    return s;
  }

  // Factory method for objects
  static SchemaDesc object() {
    SchemaDesc s;
    s.json_["type"] = "object";
    return s;
  }

  // Factory method for $ref to a schema component
  static SchemaDesc ref(const std::string & ref_name) {
    SchemaDesc s;
    s.json_["$ref"] = "#/components/schemas/" + ref_name;
    return s;
  }

  // Factory method for $ref to a response component
  static SchemaDesc response_ref(const std::string & ref_name) {
    SchemaDesc s;
    s.json_["$ref"] = "#/components/responses/" + ref_name;
    return s;
  }

  // Add a property to an object schema
  SchemaDesc & property(const std::string & name, SchemaDesc schema) {
    json_["properties"][name] = std::move(schema.json_);
    return *this;
  }

  // Set required fields for an object schema
  SchemaDesc & required(const std::vector<std::string> & fields) {
    json_["required"] = fields;
    return *this;
  }

  /// Attach prose. Applied last in a chain, so it lands on the outermost
  /// node - including the `anyOf` wrapper `or_null()` produces, which is
  /// where a client reads it from.
  SchemaDesc & description(const std::string & text) {
    json_["description"] = text;
    return *this;
  }

  /// Constrain a string schema to a closed set of values. Only write this
  /// where the emitter genuinely cannot produce anything else - an enum a
  /// handler can step outside makes a generated client reject a real body.
  SchemaDesc & enum_values(const std::vector<std::string> & values) {
    json_["enum"] = values;
    return *this;
  }

  /// Widen the schema to `<current> | null`, the OpenAPI 3.1 spelling of a
  /// key that is always present and sometimes JSON `null`. Not the same as
  /// leaving a key out of `required`: that says the key may be absent.
  SchemaDesc & or_null() {
    nlohmann::json inner = std::move(json_);
    json_ = nlohmann::json::object();
    json_["anyOf"] = nlohmann::json::array({std::move(inner), nlohmann::json{{"type", "null"}}});
    return *this;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    return json_;
  }

 private:
  nlohmann::json json_;
};

// =============================================================================
// OperationDesc - describes a single HTTP operation (GET, POST, etc.)
// =============================================================================

class OperationDesc {
 public:
  // Set the operation description text
  OperationDesc & description(std::string desc) {
    description_ = std::move(desc);
    return *this;
  }

  /// Set the OpenAPI tag. The gateway declares every tag a folded plugin
  /// operation uses in the document's global tag list, so a plugin may pick
  /// its own name here without the document acquiring an undeclared tag.
  OperationDesc & tag(std::string name) {
    tag_ = std::move(name);
    return *this;
  }

  /// Set the operationId. It has to be unique across the whole document, not
  /// just across this plugin: generated clients turn it into a method name,
  /// and the gateway's own operationIds share the namespace. Prefixing with
  /// the plugin's name is the way to stay out of their way.
  OperationDesc & operation_id(std::string id) {
    operation_id_ = std::move(id);
    return *this;
  }

  /// Declare the role a caller needs. Published as a security requirement
  /// naming `bearerAuth` with the role as its scope - the same shape the
  /// gateway's own routes use. The role must be the one `AuthConfig`'s
  /// permission table actually grants for this path: the document is read as
  /// a statement about the gateway's enforcement, not about what the plugin
  /// would prefer.
  OperationDesc & requires_role(std::string role) {
    role_ = std::move(role);
    security_declared_ = true;
    return *this;
  }

  /// Declare the operation reachable with no token at all (`security: []`).
  OperationDesc & public_route() {
    role_.clear();
    security_declared_ = true;
    return *this;
  }

  // Add a path parameter
  OperationDesc & path_param(const std::string & name, const std::string & desc) {
    nlohmann::json param;
    param["name"] = name;
    param["in"] = "path";
    param["required"] = true;
    param["description"] = desc;
    param["schema"]["type"] = "string";
    parameters_.push_back(std::move(param));
    return *this;
  }

  // Add a query parameter
  OperationDesc & query_param(const std::string & name, const std::string & desc, const SchemaDesc & schema,
                              bool is_required = false) {
    nlohmann::json param;
    param["name"] = name;
    param["in"] = "query";
    param["required"] = is_required;
    param["description"] = desc;
    param["schema"] = schema.to_json();
    parameters_.push_back(std::move(param));
    return *this;
  }

  // Set the request body schema
  OperationDesc & request_body(SchemaDesc schema) {
    request_body_ = std::move(schema);
    has_request_body_ = true;
    return *this;
  }

  // Add a response for a given HTTP status code
  OperationDesc & response(int status_code, const SchemaDesc & schema, const std::string & desc = "") {
    nlohmann::json resp;
    resp["description"] = desc.empty() ? "Response" : desc;
    resp["content"]["application/json"]["schema"] = schema.to_json();
    responses_[std::to_string(status_code)] = std::move(resp);
    return *this;
  }

  /// Declare an error status as a reference to one of the document's shared
  /// response components (`GenericError`, `Unauthorized`, ...). A plugin
  /// handler's errors reach the wire through `PluginResponse::send_error`,
  /// which writes the SOVD `GenericError` body, so `GenericError` is the
  /// component that describes them - spelling the body out inline instead
  /// would publish a second, drifting copy of it.
  OperationDesc & error_response(int status_code, const std::string & component) {
    responses_[std::to_string(status_code)] = nlohmann::json{{"$ref", "#/components/responses/" + component}};
    return *this;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    nlohmann::json j;

    if (!tag_.empty()) {
      j["tags"] = nlohmann::json::array({tag_});
    }

    if (!operation_id_.empty()) {
      j["operationId"] = operation_id_;
    }

    if (!description_.empty()) {
      j["description"] = description_;
    }

    if (security_declared_) {
      if (role_.empty()) {
        j["security"] = nlohmann::json::array();
      } else {
        nlohmann::json requirement = nlohmann::json::object();
        requirement["bearerAuth"] = nlohmann::json::array({role_});
        j["security"] = nlohmann::json::array({std::move(requirement)});
      }
    }

    if (!parameters_.empty()) {
      j["parameters"] = parameters_;
    }

    if (has_request_body_) {
      j["requestBody"]["content"]["application/json"]["schema"] = request_body_.to_json();
      j["requestBody"]["required"] = true;
    }

    if (!responses_.empty()) {
      j["responses"] = responses_;
    }

    return j;
  }

 private:
  std::string description_;
  std::string tag_;
  std::string operation_id_;
  std::string role_;
  bool security_declared_ = false;
  std::vector<nlohmann::json> parameters_;
  SchemaDesc request_body_;
  bool has_request_body_ = false;
  std::map<std::string, nlohmann::json> responses_;
};

// =============================================================================
// PathDescBuilder - builds a path item with method operations
// =============================================================================

class PathDescBuilder {
 public:
  // Set the summary for this path
  PathDescBuilder & summary(std::string s) {
    summary_ = std::move(s);
    return *this;
  }

  // Set the GET operation
  PathDescBuilder & get(OperationDesc op) {
    operations_["get"] = std::move(op);
    return *this;
  }

  // Set the POST operation
  PathDescBuilder & post(OperationDesc op) {
    operations_["post"] = std::move(op);
    return *this;
  }

  // Set the PUT operation
  PathDescBuilder & put(OperationDesc op) {
    operations_["put"] = std::move(op);
    return *this;
  }

  // Set the DELETE operation
  PathDescBuilder & delete_op(OperationDesc op) {
    operations_["delete"] = std::move(op);
    return *this;
  }

  /// The method keys this path item carries, lower-case as OpenAPI spells
  /// them. Read by `RouteDescriptions::endpoints()`; see the comment there.
  std::vector<std::string> methods() const {
    std::vector<std::string> out;
    out.reserve(operations_.size());
    for (const auto & [method, op] : operations_) {
      (void)op;
      out.push_back(method);
    }
    return out;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    nlohmann::json j;

    for (const auto & [method, op] : operations_) {
      auto op_json = op.to_json();
      if (!summary_.empty()) {
        op_json["summary"] = summary_;
      }
      j[method] = std::move(op_json);
    }

    return j;
  }

 private:
  std::string summary_;
  std::map<std::string, OperationDesc> operations_;
};

// =============================================================================
// RouteDescriptions - immutable collection of path descriptions
// =============================================================================

class RouteDescriptionsTestAccess;  // Forward declaration for test friend

class RouteDescriptions {
  friend class RouteDescriptionBuilder;
  friend class CapabilityGenerator;
  friend class RouteDescriptionsTestAccess;

 public:
  ~RouteDescriptions() = default;
  RouteDescriptions(const RouteDescriptions &) = default;
  RouteDescriptions(RouteDescriptions &&) = default;
  RouteDescriptions & operator=(const RouteDescriptions &) = default;
  RouteDescriptions & operator=(RouteDescriptions &&) = default;

  /// The (UPPERCASE method, path) pair of every operation described here.
  ///
  /// Public where `to_json()` is not, and deliberately so: the root endpoint
  /// list needs to say that these routes are mounted, and nothing more. A
  /// plugin route is mounted straight onto the HTTP server, so without this it
  /// was the one served route the root did not advertise while `/docs`
  /// documented it. Handing out the whole document instead would give the
  /// endpoint list a second, richer view of the same routes that could then
  /// drift from the one `CapabilityGenerator` folds.
  std::vector<std::pair<std::string, std::string>> endpoints() const {
    std::vector<std::pair<std::string, std::string>> out;
    for (const auto & [path, builder] : paths_) {
      for (std::string method : builder.methods()) {
        for (char & c : method) {
          c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        out.emplace_back(std::move(method), path);
      }
    }
    return out;
  }

 private:
  RouteDescriptions() = default;

  nlohmann::json to_json() const {
    nlohmann::json result = nlohmann::json::object();
    for (const auto & [path, builder] : paths_) {
      result[path] = builder.to_json();
    }
    return result;
  }

  bool empty() const {
    return paths_.empty();
  }

  std::map<std::string, PathDescBuilder> paths_;
};

// =============================================================================
// RouteDescriptionBuilder - fluent builder for RouteDescriptions
// =============================================================================

class RouteDescriptionBuilder {
 public:
  // Add a new path and return a reference to its PathDescBuilder for chaining
  PathDescBuilder & add(const std::string & path) {
    return paths_[path];
  }

  // Build the immutable RouteDescriptions
  RouteDescriptions build() {
    RouteDescriptions desc;
    desc.paths_ = std::move(paths_);
    return desc;
  }

 private:
  std::map<std::string, PathDescBuilder> paths_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
