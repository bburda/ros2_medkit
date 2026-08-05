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

#include "path_resolver.hpp"

#include <sstream>
#include <unordered_set>

namespace ros2_medkit_gateway {
namespace openapi {

namespace {

/// Top-level and nested entity type keywords recognized by the SOVD API.
const std::unordered_set<std::string> & entity_type_keywords() {
  static const std::unordered_set<std::string> keywords = {
      "areas", "subareas", "components", "subcomponents", "apps", "functions",
  };
  return keywords;
}

/// Resource collection keywords recognized by the SOVD API.
///
/// Hand-written, and **narrower than the collections the gateway serves** -
/// which is a live gap, not an oversight in this list alone. `locks`,
/// `status`, `scripts` and `fault-triggers` all answer 200 on the collection
/// and are advertised as URI fields on the entity detail response, yet
/// `<entity-path>/docs` answers 404 for them because this set does not name
/// them. Adding a keyword here is not sufficient to close that: the resolved
/// category has to have a producer in `CapabilityGenerator`, so widening this
/// set is a feature rather than a one-line fix. Until then the route's own
/// description says which prefixes resolve, rather than claiming the 404 means
/// the gateway does not serve the path.
const std::unordered_set<std::string> & resource_collection_keywords() {
  static const std::unordered_set<std::string> keywords = {
      "data", "data-categories", "data-groups",          "operations", "faults",  "configurations",
      "logs", "bulk-data",       "cyclic-subscriptions", "triggers",   "updates", "hosts",
  };
  return keywords;
}

/// Reserved path segments that must not be used as entity IDs.
const std::unordered_set<std::string> & reserved_segments() {
  static const std::unordered_set<std::string> segments = {
      "docs",
  };
  return segments;
}

/// Split a path string on '/' into non-empty segments.
std::vector<std::string> split_path(const std::string & path) {
  std::vector<std::string> segments;
  std::istringstream stream(path);
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    if (!segment.empty()) {
      segments.push_back(std::move(segment));
    }
  }
  return segments;
}

}  // namespace

bool PathResolver::is_entity_type_keyword(const std::string & segment) {
  return entity_type_keywords().count(segment) > 0;
}

bool PathResolver::is_resource_collection_keyword(const std::string & segment) {
  return resource_collection_keywords().count(segment) > 0;
}

bool PathResolver::is_reserved_segment(const std::string & segment) {
  return reserved_segments().count(segment) > 0;
}

ResolvedPath PathResolver::resolve(const std::string & path) {
  ResolvedPath result;

  // Handle root path
  if (path == "/" || path.empty()) {
    result.category = PathCategory::kRoot;
    return result;
  }

  auto segments = split_path(path);
  if (segments.empty()) {
    result.category = PathCategory::kRoot;
    return result;
  }

  if (segments.size() > 10) {
    result.category = PathCategory::kError;
    result.error = "Path too deep (max 10 segments)";
    return result;
  }

  // Walk segments pairwise: entity_type/entity_id or resource_collection/resource_id
  size_t i = 0;
  while (i < segments.size()) {
    const auto & seg = segments[i];

    if (is_entity_type_keyword(seg)) {
      // Push previous entity to parent chain if one was already parsed
      if (!result.entity_id.empty()) {
        result.parent_chain.push_back({result.entity_type, result.entity_id});
        result.entity_type.clear();
        result.entity_id.clear();
      }

      if (i + 1 >= segments.size()) {
        // No ID follows - this is an entity collection endpoint
        result.entity_type = seg;
        result.category = PathCategory::kEntityCollection;
        return result;
      }

      // Next segment is the entity ID
      const auto & id_seg = segments[i + 1];

      // Check for reserved segments used as entity IDs
      if (is_reserved_segment(id_seg)) {
        result.category = PathCategory::kError;
        result.error = "Reserved path segment: " + id_seg;
        return result;
      }

      // Record current entity
      result.entity_type = seg;
      result.entity_id = id_seg;

      if (i + 2 >= segments.size()) {
        // No more segments - this is a specific entity
        result.category = PathCategory::kSpecificEntity;
        return result;
      }

      i += 2;  // Skip past type + id
      continue;
    }

    if (is_resource_collection_keyword(seg)) {
      // This is a resource collection keyword
      result.resource_collection = seg;

      if (i + 1 >= segments.size()) {
        // No resource ID follows - this is a resource collection endpoint
        result.category = PathCategory::kResourceCollection;
        return result;
      }

      // Remaining segments form the resource ID (supports multi-segment topic names
      // like /apps/my_app/data/sensors/temperature)
      result.resource_id = segments[i + 1];
      for (size_t j = i + 2; j < segments.size(); ++j) {
        result.resource_id += "/" + segments[j];
      }
      result.category = PathCategory::kSpecificResource;
      return result;
    }

    // Segment is neither entity type nor resource collection - unresolved
    result.category = PathCategory::kUnresolved;
    return result;
  }

  // If we consumed all segments through entity type/id pairs without reaching
  // a terminal condition, the last entity is the specific entity
  result.category = PathCategory::kSpecificEntity;
  return result;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
