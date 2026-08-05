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

#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"

namespace ros2_medkit_gateway {

EntityCapabilities EntityCapabilities::for_type(SovdEntityType type) {
  EntityCapabilities caps;

  // Every entry below names a route that exists for that entity type. The lists
  // are read back out as `href`s in an entity's `capabilities` array and as
  // paths in its `/docs` sub-document, so an entry with no route is a link a
  // client follows into a 404. `rest_server.cpp::setup_routes()` is the source
  // of truth: the four-entity-type loop registers data / data-categories /
  // data-groups / operations / configurations / faults / logs / bulk-data /
  // triggers for every type, and gates cyclic-subscriptions (not areas), locks
  // and scripts (components and apps) and fault-triggers (apps) behind an
  // entity-type check.
  switch (type) {
    case SovdEntityType::SERVER:
      // Server-scoped collections are the two mounted at the API root:
      // `/faults` (+ `/faults/stream`) and `/updates`. Everything else in this
      // enum is entity-scoped only - `/logs`, `/data`, `/operations`,
      // `/configurations`, `/bulk-data`, `/locks`, `/triggers`, `/scripts` and
      // `/cyclic-subscriptions` all answer 404 at the root.
      caps.collections_ = {
          ResourceCollection::FAULTS,
          ResourceCollection::UPDATES,
      };
      // SERVER resources. SOVD (ISO 17978-3 §7.6) does not define
      // /belongs-to for server, only for apps - advertising it here would
      // make supports_resource("belongs-to") return true and clients would
      // get 404 when following it. The same argument removed /logs,
      // /depends-on, /data-categories and /data-groups: all four are
      // entity-scoped routes with nothing mounted at the root.
      caps.resources_ = {"docs", "version-info"};
      break;

    case SovdEntityType::AREA:
      // ros2_medkit extension: areas support resource collections via aggregation
      // (SOVD spec defines collections only for apps/components)
      caps.collections_ = {
          ResourceCollection::DATA,       ResourceCollection::DATA_CATEGORIES, ResourceCollection::DATA_GROUPS,
          ResourceCollection::OPERATIONS, ResourceCollection::CONFIGURATIONS,  ResourceCollection::FAULTS,
          ResourceCollection::LOGS,       ResourceCollection::BULK_DATA,       ResourceCollection::TRIGGERS,
      };
      caps.aggregated_collections_ = {
          ResourceCollection::DATA,   ResourceCollection::OPERATIONS, ResourceCollection::CONFIGURATIONS,
          ResourceCollection::FAULTS, ResourceCollection::LOGS,
      };
      // The route is `/areas/{area_id}/components`; "related-components" was a
      // name no registration ever used.
      caps.resources_ = {"docs", "contains", "subareas", "components"};
      break;

    case SovdEntityType::COMPONENT:
      caps.collections_ = {
          ResourceCollection::CONFIGURATIONS, ResourceCollection::DATA,     ResourceCollection::DATA_CATEGORIES,
          ResourceCollection::DATA_GROUPS,    ResourceCollection::FAULTS,   ResourceCollection::OPERATIONS,
          ResourceCollection::BULK_DATA,      ResourceCollection::LOCKS,    ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::LOGS,           ResourceCollection::TRIGGERS, ResourceCollection::SCRIPTS,
      };
      // SOVD (ISO 17978-3 §7.6) defines /belongs-to only for apps; component
      // exposes parent area via /is-located-on (which is itself app-only in
      // the spec, but ros2_medkit treats it as the canonical area pointer).
      // Listing belongs-to here would be a 404 promise.
      caps.resources_ = {"docs", "logs", "hosts", "depends-on", "subcomponents"};
      break;

    case SovdEntityType::APP:
      // Apps carry everything a component does plus the fault-trigger rule
      // collection, whose routes are registered for `/apps` alone.
      caps.collections_ = {
          ResourceCollection::CONFIGURATIONS,
          ResourceCollection::DATA,
          ResourceCollection::DATA_CATEGORIES,
          ResourceCollection::DATA_GROUPS,
          ResourceCollection::FAULTS,
          ResourceCollection::FAULT_TRIGGERS,
          ResourceCollection::OPERATIONS,
          ResourceCollection::BULK_DATA,
          ResourceCollection::LOCKS,
          ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::LOGS,
          ResourceCollection::TRIGGERS,
          ResourceCollection::SCRIPTS,
      };
      caps.resources_ = {"docs", "logs", "is-located-on", "belongs-to", "depends-on"};
      break;

    case SovdEntityType::FUNCTION:
      // ros2_medkit extension: functions support additional collections via aggregation
      // (SOVD spec only defines data/operations for functions)
      caps.collections_ = {
          ResourceCollection::DATA,       ResourceCollection::DATA_CATEGORIES, ResourceCollection::DATA_GROUPS,
          ResourceCollection::OPERATIONS, ResourceCollection::CONFIGURATIONS,  ResourceCollection::FAULTS,
          ResourceCollection::LOGS,       ResourceCollection::BULK_DATA,       ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::TRIGGERS,
      };
      caps.aggregated_collections_ = {
          ResourceCollection::DATA,   ResourceCollection::OPERATIONS, ResourceCollection::CONFIGURATIONS,
          ResourceCollection::FAULTS, ResourceCollection::LOGS,
      };
      // /depends-on is registered for components and apps only - a function
      // that listed it handed clients a 404.
      caps.resources_ = {"docs", "hosts"};
      break;

    case SovdEntityType::UNKNOWN:
    default:
      // Unknown type has no capabilities
      caps.collections_ = {};
      caps.resources_ = {};
      break;
  }

  return caps;
}

bool EntityCapabilities::supports_collection(ResourceCollection col) const {
  return collections_.count(col) > 0;
}

bool EntityCapabilities::supports_resource(const std::string & name) const {
  return resources_.count(name) > 0;
}

std::vector<ResourceCollection> EntityCapabilities::collections() const {
  return std::vector<ResourceCollection>(collections_.begin(), collections_.end());
}

std::vector<std::string> EntityCapabilities::resources() const {
  return std::vector<std::string>(resources_.begin(), resources_.end());
}

bool EntityCapabilities::is_aggregated(ResourceCollection col) const {
  return aggregated_collections_.count(col) > 0;
}

}  // namespace ros2_medkit_gateway
