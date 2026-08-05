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

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/dto/entity_capability.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Utility class for building capability arrays.
 *
 * Generates capabilities JSON for entity responses, ensuring consistent
 * format across all entity types (areas, components, apps, functions).
 *
 * @example
 * using Cap = CapabilityBuilder::Capability;
 * std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS};
 * auto items = CapabilityBuilder::build_capabilities("components", "my-comp", caps);
 *
 * @verifies REQ_DISCOVERY_003 Entity capabilities
 */
class CapabilityBuilder {
 public:
  /**
   * @brief Available capability types for SOVD entities.
   *
   * Each enumerator names a route registered for the entity types that list it
   * in `discovery_handlers.cpp`. The name doubles as the path segment, so an
   * enumerator with no route produces an `href` that 404s - which is why
   * `RELATED_COMPONENTS` and `RELATED_APPS` are gone: `/related-components` and
   * `/related-apps` are registered for no entity type, and the route an area
   * really serves is `/areas/{area_id}/components` (`COMPONENTS` below).
   *
   * What is checked, by what, and what is not:
   *
   * - Every enumerator has an arm in `capability_to_name` - enforced by the
   *   **compiler**, `-Werror=switch-enum`. Adding one here without an arm does
   *   not build.
   * - No arm resolves to the `"unknown"` placeholder or an empty segment -
   *   `CapabilityBuilderTest.NamesEveryEnumerator`. The compiler cannot see
   *   this: an arm that returns the placeholder is a handled enumerator.
   * - Every enumerator a handler list actually uses reaches a route that does
   *   not 404 - `test_openapi_contract::test_every_advertised_collection_is_served`,
   *   which follows each `href` against a live gateway.
   *
   * Not checked: that every enumerator is listed by some entity type. The four
   * lists are function-local in `discovery_handlers.cpp`, so an enumerator no
   * list uses is dead rather than wrong, and nothing here will say so. Pinning
   * it would mean declaring the per-type membership a second time, which is the
   * duplicated-fact problem this file already has three copies of.
   */
  enum class Capability {
    DATA,                  ///< Entity has data endpoints
    DATA_CATEGORIES,       ///< Entity has a data-categories endpoint (answers 501)
    DATA_GROUPS,           ///< Entity has a data-groups endpoint (answers 501)
    OPERATIONS,            ///< Entity has operations (services/actions)
    CONFIGURATIONS,        ///< Entity has configurations (parameters)
    FAULTS,                ///< Entity has fault management
    FAULT_TRIGGERS,        ///< Entity has fault-trigger threshold rules (apps only)
    SUBAREAS,              ///< Entity has child areas (areas only)
    SUBCOMPONENTS,         ///< Entity has child components (components only)
    COMPONENTS,            ///< Entity has member components (areas only)
    CONTAINS,              ///< Entity contains other entities (areas->components)
    HOSTS,                 ///< Entity has host apps (functions/components)
    DEPENDS_ON,            ///< Entity has dependencies (components only)
    IS_LOCATED_ON,         ///< Entity has parent component (apps only)
    BELONGS_TO,            ///< Entity has parent area (apps only)
    LOGS,                  ///< Entity has application log entries (components and apps)
    BULK_DATA,             ///< Entity has bulk data endpoints (rosbags)
    CYCLIC_SUBSCRIPTIONS,  ///< Entity has cyclic subscription endpoints
    LOCKS,                 ///< Entity has lock endpoints (components and apps only)
    SCRIPTS,               ///< Entity has diagnostic script endpoints
    TRIGGERS,              ///< Entity has trigger endpoints (x-medkit extension)
    STATUS                 ///< Entity has lifecycle status endpoint (components and apps only)
  };

  /**
   * @brief Build the capabilities array for an entity.
   *
   * @param entity_type The entity type (e.g., "areas", "components", "apps", "functions")
   * @param entity_id The entity identifier
   * @param capabilities Vector of capability types to include
   * @return One dto::EntityCapability per requested capability, in order
   */
  static std::vector<dto::EntityCapability> build_capabilities(const std::string & entity_type,
                                                               const std::string & entity_id,
                                                               const std::vector<Capability> & capabilities);

  /**
   * @brief Convert capability enum to string name.
   *
   * @param cap The capability enum value
   * @return String name for the capability (e.g., "data", "operations")
   */
  static std::string capability_to_name(Capability cap);

  /**
   * @brief Convert capability enum to URL path segment.
   *
   * @param cap The capability enum value
   * @return URL path segment for the capability (e.g., "data", "operations")
   */
  static std::string capability_to_path(Capability cap);
};

/**
 * @brief Fluent builder for HATEOAS _links objects.
 *
 * Provides a fluent API for constructing _links JSON objects.
 *
 * @example
 * LinksBuilder links;
 * auto json = links.self("/components/my-comp")
 *                  .parent("/areas/powertrain")
 *                  .collection("/components")
 *                  .add("custom", "/custom/link")
 *                  .build();
 *
 * @verifies REQ_API_002 HATEOAS links
 */
class LinksBuilder {
 public:
  /**
   * @brief Construct a new LinksBuilder.
   */
  LinksBuilder() = default;

  /**
   * @brief Set the self link.
   *
   * @param href The href for the self link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & self(const std::string & href);

  /**
   * @brief Set the parent link.
   *
   * @param href The href for the parent link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & parent(const std::string & href);

  /**
   * @brief Set the collection link.
   *
   * @param href The href for the collection link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & collection(const std::string & href);

  /**
   * @brief Add a custom link.
   *
   * @param rel The relation name
   * @param href The href for the link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & add(const std::string & rel, const std::string & href);

  /**
   * @brief Build the final _links JSON object.
   *
   * @return JSON object containing all configured links
   */
  nlohmann::json build() const;

 private:
  nlohmann::json links_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
