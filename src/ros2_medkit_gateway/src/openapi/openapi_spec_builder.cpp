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

#include "openapi_spec_builder.hpp"

#include "schema_builder.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

OpenApiSpecBuilder & OpenApiSpecBuilder::info(const std::string & title, const std::string & version) {
  title_ = title;
  version_ = version;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::description(const std::string & desc) {
  description_ = desc;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::tags(const std::vector<TagInfo> & tags) {
  tags_ = tags;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::server(const std::string & url, const std::string & description) {
  servers_.push_back({url, description});
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::contact(const std::string & name, const std::string & url) {
  contact_name_ = name;
  contact_url_ = url;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::sovd_version(const std::string & version) {
  sovd_version_ = version;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::add_paths(const nlohmann::json & paths) {
  for (auto & [key, val] : paths.items()) {
    paths_[key] = val;
  }
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::add_schemas(const nlohmann::json & schemas) {
  for (auto & [key, val] : schemas.items()) {
    schemas_[key] = val;
  }
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::security_scheme(const std::string & name, const nlohmann::json & scheme) {
  security_schemes_.push_back({name, scheme});
  return *this;
}

nlohmann::json OpenApiSpecBuilder::build() const {
  nlohmann::json spec;

  // 1. Always OpenAPI 3.1.0
  spec["openapi"] = "3.1.0";

  // 2. Info block
  spec["info"]["title"] = title_;
  spec["info"]["version"] = version_;
  if (!description_.empty()) {
    spec["info"]["description"] = description_;
  }
  if (!sovd_version_.empty()) {
    spec["info"]["x-sovd-version"] = sovd_version_;
  }
  if (!contact_name_.empty()) {
    spec["info"]["contact"]["name"] = contact_name_;
    if (!contact_url_.empty()) {
      spec["info"]["contact"]["url"] = contact_url_;
    }
  }

  // 2b. Top-level tags array
  if (!tags_.empty()) {
    spec["tags"] = nlohmann::json::array();
    for (const auto & t : tags_) {
      nlohmann::json tag_obj;
      tag_obj["name"] = t.name;
      if (!t.description.empty()) {
        tag_obj["description"] = t.description;
      }
      spec["tags"].push_back(std::move(tag_obj));
    }
  }

  // 3. Servers
  spec["servers"] = nlohmann::json::array();
  for (const auto & s : servers_) {
    nlohmann::json entry;
    entry["url"] = s.url;
    if (!s.description.empty()) {
      entry["description"] = s.description;
    }
    spec["servers"].push_back(std::move(entry));
  }

  // 4. Paths
  spec["paths"] = paths_;

  // 5. Components - schemas (only if any were added)
  if (!schemas_.empty()) {
    spec["components"]["schemas"] = schemas_;
  }

  // 6. Components - GenericError response (always present)
  // Ensure GenericError schema exists for sub-page specs that don't call add_schemas()
  if (!spec["components"].contains("schemas") || !spec["components"]["schemas"].contains("GenericError")) {
    spec["components"]["schemas"]["GenericError"] = SchemaBuilder::generic_error();
  }
  // Uses $ref to components/schemas/GenericError so the schema is defined once
  spec["components"]["responses"]["GenericError"]["description"] = "SOVD GenericError response";
  spec["components"]["responses"]["GenericError"]["content"]["application/json"]["schema"] =
      SchemaBuilder::ref("GenericError");

  // 6b. Middleware-owned responses. 401/403/429 never reach a handler - the
  // auth and rate-limit middleware answer them ahead of routing - so no route's
  // return type can describe them and no `RouteEntry` can carry their headers.
  // They are declared once here and referenced from every route the middleware
  // guards.
  auto & responses = spec["components"]["responses"];

  // AuthMiddleware puts the RFC 6749 `{error, error_description}` shape on the
  // wire for 401/403, which no component schema describes yet; these two point
  // at GenericError until that schema exists, because referencing an undefined
  // component would break the document's ref-resolution contract outright.
  responses["Unauthorized"]["description"] = "Authentication is missing or the bearer token is invalid.";
  responses["Unauthorized"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("GenericError");
  responses["Unauthorized"]["headers"]["WWW-Authenticate"] = {
      {"description", "Bearer challenge, e.g. `Bearer realm=\"ros2_medkit_gateway\", error=\"invalid_token\"`."},
      {"schema", {{"type", "string"}}}};

  responses["Forbidden"]["description"] = "The token is valid but lacks the scope this operation requires.";
  responses["Forbidden"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("GenericError");

  // The rate limiter emits the SOVD GenericError shape, so unlike the two
  // above this schema already matches the wire.
  responses["RateLimited"]["description"] = "The client exceeded its request quota.";
  responses["RateLimited"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("GenericError");
  responses["RateLimited"]["headers"] = {
      {"Retry-After", {{"description", "Seconds to wait before retrying."}, {"schema", {{"type", "string"}}}}},
      {"X-RateLimit-Limit", {{"description", "Requests permitted per window."}, {"schema", {{"type", "string"}}}}},
      {"X-RateLimit-Remaining",
       {{"description", "Requests left in the current window."}, {"schema", {{"type", "string"}}}}},
      {"X-RateLimit-Reset",
       {{"description", "Unix timestamp at which the window resets."}, {"schema", {{"type", "string"}}}}}};

  // 7. Security schemes (if any)
  if (!security_schemes_.empty()) {
    spec["security"] = nlohmann::json::array();
    for (const auto & ss : security_schemes_) {
      spec["components"]["securitySchemes"][ss.name] = ss.scheme;
      spec["security"].push_back({{ss.name, nlohmann::json::array()}});
    }
  }

  return spec;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
