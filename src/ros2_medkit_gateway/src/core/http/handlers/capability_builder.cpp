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

#include "ros2_medkit_gateway/core/http/handlers/capability_builder.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

std::string CapabilityBuilder::capability_to_name(Capability cap) {
  switch (cap) {
    case Capability::DATA:
      return "data";
    case Capability::DATA_CATEGORIES:
      return "data-categories";
    case Capability::DATA_GROUPS:
      return "data-groups";
    case Capability::OPERATIONS:
      return "operations";
    case Capability::CONFIGURATIONS:
      return "configurations";
    case Capability::FAULTS:
      return "faults";
    case Capability::FAULT_TRIGGERS:
      return "fault-triggers";
    case Capability::SUBAREAS:
      return "subareas";
    case Capability::SUBCOMPONENTS:
      return "subcomponents";
    case Capability::COMPONENTS:
      return "components";
    case Capability::CONTAINS:
      return "contains";
    case Capability::HOSTS:
      return "hosts";
    case Capability::DEPENDS_ON:
      return "depends-on";
    case Capability::IS_LOCATED_ON:
      return "is-located-on";
    case Capability::BELONGS_TO:
      return "belongs-to";
    case Capability::LOGS:
      return "logs";
    case Capability::BULK_DATA:
      return "bulk-data";
    case Capability::CYCLIC_SUBSCRIPTIONS:
      return "cyclic-subscriptions";
    case Capability::LOCKS:
      return "locks";
    case Capability::SCRIPTS:
      return "scripts";
    case Capability::TRIGGERS:
      return "triggers";
    case Capability::STATUS:
      return "status";
    default:
      return "unknown";
  }
}

std::string CapabilityBuilder::capability_to_path(Capability cap) {
  // Path segments match names for most capabilities
  return capability_to_name(cap);
}

std::vector<dto::EntityCapability> CapabilityBuilder::build_capabilities(const std::string & entity_type,
                                                                         const std::string & entity_id,
                                                                         const std::vector<Capability> & capabilities) {
  std::vector<dto::EntityCapability> result;
  result.reserve(capabilities.size());

  // Shared href prefix: /api/v1/{entity_type}/{entity_id}/
  std::string href_prefix = "/api/v1/";
  href_prefix.append(entity_type).append("/").append(entity_id).append("/");

  for (const auto & cap : capabilities) {
    dto::EntityCapability item;
    item.name = capability_to_name(cap);
    item.href = href_prefix;
    item.href.append(capability_to_path(cap));
    result.push_back(std::move(item));
  }

  return result;
}

LinksBuilder & LinksBuilder::self(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["self"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::parent(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["parent"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::collection(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["collection"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::add(const std::string & rel, const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_[rel] = href;
  return *this;
}

nlohmann::json LinksBuilder::build() const {
  if (links_.is_null()) {
    return nlohmann::json::object();
  }
  return links_;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
