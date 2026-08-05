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
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entity_capability.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// Shared entity-detail members
// =============================================================================

/// Prose published for every ``_links`` member. The object stays untyped
/// because its value type is a union: every relation `LinksBuilder` writes is a
/// path string, but `depends-on` is written straight into the built object as an
/// array of paths, by the app handler and the function handler both.
/// `SchemaWriter` walks `dto_fields`, so it has no descriptor for a map whose
/// values differ in type - naming the relations and their value shapes in prose
/// is what a client can actually act on, and beats publishing `{}` with no
/// explanation.
///
/// The per-relation conditions, read off `discovery_handlers.cpp`: `self` and
/// `collection` are unconditional; `parent` needs a `parent_area_id` (areas) or
/// a `parent_component_id` (components) and is emitted for no other type;
/// `area` needs `comp.area`; `is-located-on` and `belongs-to` need
/// `app.component_id`; `depends-on` needs a non-empty `depends_on` list.
inline constexpr std::string_view kLinksDescription =
    "HATEOAS relation map. Relations emitted today: self and collection (all entity types), parent "
    "(areas and components, when the entity has a parent), area (components), is-located-on and "
    "belongs-to (apps) - each an absolute path - and depends-on (apps and functions), an array of "
    "absolute paths.";

// =============================================================================
// Area DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// AreaListItem - shape emitted per item in handle_list_areas "items" array.
//
// Wire keys:  id, name, href, type, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct AreaListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitArea> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AreaListItem> =
    std::make_tuple(field("id", &AreaListItem::id), field("name", &AreaListItem::name),
                    field("href", &AreaListItem::href), field_enum("type", &AreaListItem::type, kEntityTypeValues),
                    field("description", &AreaListItem::description), field("tags", &AreaListItem::tags),
                    field("x-medkit", &AreaListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<AreaListItem> = "AreaListItem";

// -----------------------------------------------------------------------------
// AreaDetail - shape emitted by handle_get_area.
//
// Wire keys:
//   id, name, description?, tags?,
//   subareas, components, contains, data, data-categories, data-groups,
//   operations, configurations, faults, logs, bulk-data, triggers,
//   capabilities (array of EntityCapability),
//   _links (open relation map - see kLinksDescription),
//   x-medkit
// -----------------------------------------------------------------------------
struct AreaDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  // Resource collection URI fields (always present in detail response)
  std::string subareas;
  std::string components;
  std::string contains;
  std::string data;
  std::string data_categories;  // wire key: "data-categories"
  std::string data_groups;      // wire key: "data-groups"
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string logs;
  std::string bulk_data;  // wire key: "bulk-data"
  std::string triggers;
  // Free-form fields
  std::optional<std::vector<EntityCapability>> capabilities;
  std::optional<nlohmann::json> links;  // wire key: "_links" - see kLinksDescription
  std::optional<XMedkitArea> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AreaDetail> =
    std::make_tuple(field("id", &AreaDetail::id), field("name", &AreaDetail::name),
                    field_enum("type", &AreaDetail::type, kEntityTypeValues),
                    field("description", &AreaDetail::description), field("tags", &AreaDetail::tags),
                    field("subareas", &AreaDetail::subareas), field("components", &AreaDetail::components),
                    field("contains", &AreaDetail::contains), field("data", &AreaDetail::data),
                    field("data-categories", &AreaDetail::data_categories),
                    field("data-groups", &AreaDetail::data_groups), field("operations", &AreaDetail::operations),
                    field("configurations", &AreaDetail::configurations), field("faults", &AreaDetail::faults),
                    field("logs", &AreaDetail::logs), field("bulk-data", &AreaDetail::bulk_data),
                    field("triggers", &AreaDetail::triggers), field("capabilities", &AreaDetail::capabilities),
                    field("_links", &AreaDetail::links, kLinksDescription), field("x-medkit", &AreaDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<AreaDetail> = "AreaDetail";

// =============================================================================
// Component DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// ComponentListItem - shape emitted per item in handle_list_components "items".
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct ComponentListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitComponent> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ComponentListItem> =
    std::make_tuple(field("id", &ComponentListItem::id), field("name", &ComponentListItem::name),
                    field("href", &ComponentListItem::href),
                    field_enum("type", &ComponentListItem::type, kEntityTypeValues),
                    field("description", &ComponentListItem::description), field("tags", &ComponentListItem::tags),
                    field("x-medkit", &ComponentListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<ComponentListItem> = "ComponentListItem";

// -----------------------------------------------------------------------------
// ComponentDetail - shape emitted by handle_get_component.
//
// Wire keys:
//   id, name, description?, tags?,
//   status, data, data-categories, data-groups, operations, configurations,
//   faults, subcomponents, hosts, logs, bulk-data, cyclic-subscriptions,
//   triggers,
//   scripts? (conditional on script backend), locks? (conditional on locking),
//   depends-on? (conditional),
//   belongs-to? (conditional on area), is-located-on? (not present here - app only),
//   capabilities (array of EntityCapability),
//   _links (open relation map - see kLinksDescription),
//   x-medkit
// -----------------------------------------------------------------------------
struct ComponentDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string status;
  std::string data;
  std::string data_categories;  // wire key: "data-categories"
  std::string data_groups;      // wire key: "data-groups"
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string subcomponents;
  std::string hosts;
  std::string logs;
  std::string bulk_data;             // wire key: "bulk-data"
  std::string cyclic_subscriptions;  // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Conditional URI fields
  std::optional<std::string> scripts;     // present only with script backend
  std::optional<std::string> locks;       // present only when locking is enabled
  std::optional<std::string> depends_on;  // wire key: "depends-on"
  std::optional<std::string> belongs_to;  // wire key: "belongs-to"
  // Free-form fields
  std::optional<std::vector<EntityCapability>> capabilities;
  std::optional<nlohmann::json> links;       // wire key: "_links" - see kLinksDescription
  std::optional<XMedkitComponent> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ComponentDetail> = std::make_tuple(
    field("id", &ComponentDetail::id), field("name", &ComponentDetail::name),
    field_enum("type", &ComponentDetail::type, kEntityTypeValues), field("description", &ComponentDetail::description),
    field("tags", &ComponentDetail::tags), field("status", &ComponentDetail::status),
    field("data", &ComponentDetail::data), field("data-categories", &ComponentDetail::data_categories),
    field("data-groups", &ComponentDetail::data_groups), field("operations", &ComponentDetail::operations),
    field("configurations", &ComponentDetail::configurations), field("faults", &ComponentDetail::faults),
    field("subcomponents", &ComponentDetail::subcomponents), field("hosts", &ComponentDetail::hosts),
    field("logs", &ComponentDetail::logs), field("bulk-data", &ComponentDetail::bulk_data),
    field("cyclic-subscriptions", &ComponentDetail::cyclic_subscriptions),
    field("triggers", &ComponentDetail::triggers), field("scripts", &ComponentDetail::scripts),
    field("locks", &ComponentDetail::locks), field("depends-on", &ComponentDetail::depends_on),
    field("belongs-to", &ComponentDetail::belongs_to), field("capabilities", &ComponentDetail::capabilities),
    field("_links", &ComponentDetail::links, kLinksDescription), field("x-medkit", &ComponentDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<ComponentDetail> = "ComponentDetail";

// =============================================================================
// App DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// AppListItem - shape emitted per item in handle_list_apps "items" array.
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct AppListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitApp> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AppListItem> =
    std::make_tuple(field("id", &AppListItem::id), field("name", &AppListItem::name), field("href", &AppListItem::href),
                    field_enum("type", &AppListItem::type, kEntityTypeValues),
                    field("description", &AppListItem::description), field("tags", &AppListItem::tags),
                    field("x-medkit", &AppListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<AppListItem> = "AppListItem";

// -----------------------------------------------------------------------------
// AppDetail - shape emitted by handle_get_app.
//
// Wire keys:
//   id, name, description?, translation_id?, tags?,
//   status, data, data-categories, data-groups, operations, configurations,
//   fault-triggers, faults, logs, bulk-data, cyclic-subscriptions, triggers,
//   scripts? (conditional on script backend),
//   locks? (conditional on locking being enabled),
//   is-located-on? (conditional on component_id),
//   belongs-to? (conditional on component_id),
//   depends-on? (conditional on depends_on list),
//   capabilities (array of EntityCapability),
//   _links (open relation map - see kLinksDescription),
//   x-medkit
// -----------------------------------------------------------------------------
struct AppDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::string> translation_id;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string status;
  std::string data;
  std::string data_categories;  // wire key: "data-categories"
  std::string data_groups;      // wire key: "data-groups"
  std::string operations;
  std::string configurations;
  std::string fault_triggers;  // wire key: "fault-triggers" (apps only)
  std::string faults;
  std::string logs;
  std::string bulk_data;             // wire key: "bulk-data"
  std::string cyclic_subscriptions;  // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Conditional URI fields
  std::optional<std::string> scripts;        // present only with script backend
  std::optional<std::string> locks;          // present only when locking is enabled
  std::optional<std::string> is_located_on;  // wire key: "is-located-on"
  std::optional<std::string> belongs_to;     // wire key: "belongs-to"
  std::optional<std::string> depends_on;     // wire key: "depends-on"
  // Free-form fields
  std::optional<std::vector<EntityCapability>> capabilities;
  std::optional<nlohmann::json> links;  // wire key: "_links" - see kLinksDescription
  std::optional<XMedkitApp> x_medkit;   // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AppDetail> = std::make_tuple(
    field("id", &AppDetail::id), field("name", &AppDetail::name),
    field_enum("type", &AppDetail::type, kEntityTypeValues), field("description", &AppDetail::description),
    field("translation_id", &AppDetail::translation_id), field("tags", &AppDetail::tags),
    field("status", &AppDetail::status), field("data", &AppDetail::data),
    field("data-categories", &AppDetail::data_categories), field("data-groups", &AppDetail::data_groups),
    field("operations", &AppDetail::operations), field("configurations", &AppDetail::configurations),
    field("fault-triggers", &AppDetail::fault_triggers), field("faults", &AppDetail::faults),
    field("logs", &AppDetail::logs), field("bulk-data", &AppDetail::bulk_data),
    field("cyclic-subscriptions", &AppDetail::cyclic_subscriptions), field("triggers", &AppDetail::triggers),
    field("scripts", &AppDetail::scripts), field("locks", &AppDetail::locks),
    field("is-located-on", &AppDetail::is_located_on), field("belongs-to", &AppDetail::belongs_to),
    field("depends-on", &AppDetail::depends_on), field("capabilities", &AppDetail::capabilities),
    field("_links", &AppDetail::links, kLinksDescription), field("x-medkit", &AppDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<AppDetail> = "AppDetail";

// =============================================================================
// Function DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// FunctionListItem - shape emitted per item in handle_list_functions "items".
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct FunctionListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitFunction> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FunctionListItem> =
    std::make_tuple(field("id", &FunctionListItem::id), field("name", &FunctionListItem::name),
                    field("href", &FunctionListItem::href),
                    field_enum("type", &FunctionListItem::type, kEntityTypeValues),
                    field("description", &FunctionListItem::description), field("tags", &FunctionListItem::tags),
                    field("x-medkit", &FunctionListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<FunctionListItem> = "FunctionListItem";

// -----------------------------------------------------------------------------
// FunctionDetail - shape emitted by handle_get_function.
//
// Wire keys:
//   id, name, description?, translation_id?, tags?,
//   hosts, data, data-categories, data-groups, operations, configurations,
//   faults, logs, bulk-data, x-medkit-graph?, cyclic-subscriptions, triggers,
//   capabilities (array of EntityCapability),
//   _links (open relation map - see kLinksDescription),
//   x-medkit
// -----------------------------------------------------------------------------
struct FunctionDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::string> translation_id;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string hosts;
  std::string data;
  std::string data_categories;  // wire key: "data-categories"
  std::string data_groups;      // wire key: "data-groups"
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string logs;
  std::string bulk_data;  // wire key: "bulk-data"
  // Optional, unlike the collections above it: `x-medkit-graph` is served by
  // the graph-provider plugin, not by the gateway, so on a gateway without
  // that plugin loaded there is no route behind the URI. Emitted only when
  // the entity's capability list says a plugin serves it - a link to a 404 is
  // worse than no link.
  std::optional<std::string> x_medkit_graph;  // wire key: "x-medkit-graph"
  std::string cyclic_subscriptions;           // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Free-form fields
  std::optional<std::vector<EntityCapability>> capabilities;
  std::optional<nlohmann::json> links;      // wire key: "_links" - see kLinksDescription
  std::optional<XMedkitFunction> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FunctionDetail> = std::make_tuple(
    field("id", &FunctionDetail::id), field("name", &FunctionDetail::name),
    field_enum("type", &FunctionDetail::type, kEntityTypeValues), field("description", &FunctionDetail::description),
    field("translation_id", &FunctionDetail::translation_id), field("tags", &FunctionDetail::tags),
    field("hosts", &FunctionDetail::hosts), field("data", &FunctionDetail::data),
    field("data-categories", &FunctionDetail::data_categories), field("data-groups", &FunctionDetail::data_groups),
    field("operations", &FunctionDetail::operations), field("configurations", &FunctionDetail::configurations),
    field("faults", &FunctionDetail::faults), field("logs", &FunctionDetail::logs),
    field("bulk-data", &FunctionDetail::bulk_data), field("x-medkit-graph", &FunctionDetail::x_medkit_graph),
    field("cyclic-subscriptions", &FunctionDetail::cyclic_subscriptions), field("triggers", &FunctionDetail::triggers),
    field("capabilities", &FunctionDetail::capabilities), field("_links", &FunctionDetail::links, kLinksDescription),
    field("x-medkit", &FunctionDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<FunctionDetail> = "FunctionDetail";

// =============================================================================
// Collection<T, XMedkit> wrapper
// =============================================================================

/// Generic collection wrapper used for every entity list response.
///
/// The "items" array element type T is one of the *ListItem DTOs above.
/// The second template parameter selects the x-medkit payload type; it
/// defaults to the generic XMedkitCollection (total_count + contributors +
/// fan-out observability fields), but per-domain endpoints can plug in a
/// richer typed payload (e.g. FaultListXMedkit, ConfigListXMedkit,
/// DataListXMedkit, LogListXMedkit) without needing a separate top-level
/// struct.
///
/// The optional "links" member carries the free-form "_links" object emitted
/// by sub-collection handlers (subareas, contains, hosts, depends-on, etc.).
///
/// dto_name<Collection<T, XMedkit>>: the default (XMedkit = XMedkitCollection)
/// is specialized per item type below ("AreaList", "ComponentList", ...). Non-
/// default XMedkit instantiations must be co-located with their domain header
/// and registered in AllDtos.
template <class T, class XMedkit = XMedkitCollection>
struct Collection {
  std::vector<T> items;
  std::optional<XMedkit> x_medkit;      // wire key: "x-medkit"
  std::optional<nlohmann::json> links;  // wire key: "_links"
};

template <class T, class XMedkit>
inline constexpr auto dto_fields<Collection<T, XMedkit>> =
    std::make_tuple(field("items", &Collection<T, XMedkit>::items),
                    field("x-medkit", &Collection<T, XMedkit>::x_medkit),
                    field("_links", &Collection<T, XMedkit>::links));

// dto_name per concrete instantiation (no runtime string concatenation):
template <>
inline constexpr std::string_view dto_name<Collection<AreaListItem>> = "AreaList";

template <>
inline constexpr std::string_view dto_name<Collection<ComponentListItem>> = "ComponentList";

template <>
inline constexpr std::string_view dto_name<Collection<AppListItem>> = "AppList";

template <>
inline constexpr std::string_view dto_name<Collection<FunctionListItem>> = "FunctionList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
