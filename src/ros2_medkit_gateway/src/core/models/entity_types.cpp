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

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

#include <algorithm>
#include <optional>
#include <unordered_map>

namespace ros2_medkit_gateway {

std::string to_string(SovdEntityType type) {
  switch (type) {
    case SovdEntityType::SERVER:
      return "Server";
    case SovdEntityType::AREA:
      return "Area";
    case SovdEntityType::COMPONENT:
      return "Component";
    case SovdEntityType::APP:
      return "App";
    case SovdEntityType::FUNCTION:
      return "Function";
    case SovdEntityType::UNKNOWN:
    default:
      return "Unknown";
  }
}

std::string to_string(ResourceCollection col) {
  switch (col) {
    case ResourceCollection::CONFIGURATIONS:
      return "configurations";
    case ResourceCollection::DATA:
      return "data";
    case ResourceCollection::DATA_CATEGORIES:
      return "data-categories";
    case ResourceCollection::DATA_GROUPS:
      return "data-groups";
    case ResourceCollection::FAULTS:
      return "faults";
    case ResourceCollection::FAULT_TRIGGERS:
      return "fault-triggers";
    case ResourceCollection::OPERATIONS:
      return "operations";
    case ResourceCollection::BULK_DATA:
      return "bulk-data";
    case ResourceCollection::DATA_LISTS:
      return "data-lists";
    case ResourceCollection::LOCKS:
      return "locks";
    case ResourceCollection::MODES:
      return "modes";
    case ResourceCollection::CYCLIC_SUBSCRIPTIONS:
      return "cyclic-subscriptions";
    case ResourceCollection::LOGS:
      return "logs";
    case ResourceCollection::COMMUNICATION_LOGS:
      return "communication-logs";
    case ResourceCollection::TRIGGERS:
      return "triggers";
    case ResourceCollection::SCRIPTS:
      return "scripts";
    case ResourceCollection::UPDATES:
      return "updates";
    default:
      return "unknown";
  }
}

std::string to_path_segment(ResourceCollection col) {
  // Path segments use hyphens, same as to_string for most cases
  return to_string(col);
}

std::optional<ResourceCollection> parse_resource_collection(const std::string & segment) {
  static const std::unordered_map<std::string, ResourceCollection> mapping = {
      {"configurations", ResourceCollection::CONFIGURATIONS},
      {"data", ResourceCollection::DATA},
      {"data-categories", ResourceCollection::DATA_CATEGORIES},
      {"data-groups", ResourceCollection::DATA_GROUPS},
      {"faults", ResourceCollection::FAULTS},
      {"fault-triggers", ResourceCollection::FAULT_TRIGGERS},
      {"operations", ResourceCollection::OPERATIONS},
      {"bulk-data", ResourceCollection::BULK_DATA},
      {"data-lists", ResourceCollection::DATA_LISTS},
      {"locks", ResourceCollection::LOCKS},
      {"modes", ResourceCollection::MODES},
      {"cyclic-subscriptions", ResourceCollection::CYCLIC_SUBSCRIPTIONS},
      {"logs", ResourceCollection::LOGS},
      {"communication-logs", ResourceCollection::COMMUNICATION_LOGS},
      {"triggers", ResourceCollection::TRIGGERS},
      {"scripts", ResourceCollection::SCRIPTS},
      {"updates", ResourceCollection::UPDATES},
  };

  auto it = mapping.find(segment);
  if (it != mapping.end()) {
    return it->second;
  }
  return std::nullopt;
}

SovdEntityType parse_entity_type(const std::string & str) {
  // Case-insensitive comparison
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "server" || lower == "sovdserver") {
    return SovdEntityType::SERVER;
  }
  if (lower == "area") {
    return SovdEntityType::AREA;
  }
  if (lower == "component") {
    return SovdEntityType::COMPONENT;
  }
  if (lower == "app" || lower == "application") {
    return SovdEntityType::APP;
  }
  if (lower == "function") {
    return SovdEntityType::FUNCTION;
  }
  return SovdEntityType::UNKNOWN;
}

}  // namespace ros2_medkit_gateway
