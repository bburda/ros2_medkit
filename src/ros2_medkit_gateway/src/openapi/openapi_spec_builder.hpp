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
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

/// Tag metadata for the top-level OpenAPI tags array.
struct TagInfo {
  std::string name;
  std::string description;
};

/// Assembles a complete OpenAPI 3.1.0 document from paths, schemas, and server info.
/// Uses builder pattern for fluent configuration.
class OpenApiSpecBuilder {
 public:
  /// Set the API info block (title and version).
  OpenApiSpecBuilder & info(const std::string & title, const std::string & version);

  /// Set the API description (info.description).
  OpenApiSpecBuilder & description(const std::string & desc);

  /// Add a server entry with URL and optional description.
  OpenApiSpecBuilder & server(const std::string & url, const std::string & description = "");

  /// Set the API contact info (info.contact).
  OpenApiSpecBuilder & contact(const std::string & name, const std::string & url);

  /// Set the SOVD specification version (x-sovd-version extension in info).
  OpenApiSpecBuilder & sovd_version(const std::string & version);

  /// Set the top-level tags array with name and description for each tag.
  OpenApiSpecBuilder & tags(const std::vector<TagInfo> & tags);

  /// Merge path items into the paths object.
  OpenApiSpecBuilder & add_paths(const nlohmann::json & paths);

  /// Merge schemas into components/schemas.
  OpenApiSpecBuilder & add_schemas(const nlohmann::json & schemas);

  /// Add a security scheme, and by default the matching document-level
  /// security requirement.
  ///
  /// The two are separable because they say different things. The scheme is
  /// a definition - "this document refers to a bearer token called `name`" -
  /// and an operation cannot name a scheme the document does not define, so
  /// any per-operation `security` requires it. The document-level
  /// requirement is a claim about enforcement: it says *every* request needs
  /// that token. Pass `document_level_requirement = false` to register the
  /// definition without making that claim.
  OpenApiSpecBuilder & security_scheme(const std::string & name, const nlohmann::json & scheme,
                                       bool document_level_requirement = true);

  /// Build the complete OpenAPI 3.1.0 document.
  nlohmann::json build() const;

 private:
  std::string title_;
  std::string version_;
  std::string description_;
  std::string sovd_version_;
  std::string contact_name_;
  std::string contact_url_;

  struct ServerEntry {
    std::string url;
    std::string description;
  };
  std::vector<ServerEntry> servers_;

  nlohmann::json paths_ = nlohmann::json::object();
  nlohmann::json schemas_ = nlohmann::json::object();

  struct SecuritySchemeEntry {
    std::string name;
    nlohmann::json scheme;
    bool document_level_requirement;
  };
  std::vector<SecuritySchemeEntry> security_schemes_;

  std::vector<TagInfo> tags_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
