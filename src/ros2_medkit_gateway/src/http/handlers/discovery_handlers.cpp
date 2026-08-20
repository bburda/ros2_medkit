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

#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <variant>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/capability_builder.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Emit the x-medkit `external` flag only when the entity is effectively
/// external (tri-state true). Explicit false and unset both leave the field
/// absent, so the "absence == not external" wire contract holds. Centralised
/// here so no call site can leak `external: false` by assigning the raw
/// optional; every route that carries a Component/App x-medkit calls this (#516).
void set_x_medkit_external(dto::XMedkitComponent & x_medkit, const std::optional<bool> & external) {
  if (external.value_or(false)) {
    x_medkit.external = true;
  }
}
void set_x_medkit_external(dto::XMedkitApp & x_medkit, const std::optional<bool> & external) {
  if (external.value_or(false)) {
    x_medkit.external = true;
  }
}

/// Emit the x-medkit `available` flag only when the entity cannot be reached,
/// so an absent field means reachable. Carried by the entities a request can be
/// addressed to; a grouping entity is a view over members and has no
/// availability of its own.
void set_x_medkit_available(dto::XMedkitComponent & x_medkit, bool available) {
  if (!available) {
    x_medkit.available = false;
  }
}

/// Check if a capability name is already present in the capabilities array
bool has_capability(const json & capabilities, const std::string & name) {
  for (const auto & cap : capabilities) {
    if (cap.contains("name") && cap["name"] == name) {
      return true;
    }
  }
  return false;
}

/// Append plugin-registered capabilities to a capabilities JSON array
void append_plugin_capabilities(json & capabilities, const std::string & entity_type_path,
                                const std::string & entity_id, SovdEntityType entity_type, const GatewayNode * node) {
  auto * pmgr = node->get_plugin_manager();
  if (!pmgr) {
    return;
  }

  std::string href_prefix;
  href_prefix.reserve(64);
  href_prefix.append("/api/v1/").append(entity_type_path).append("/").append(entity_id).append("/");

  // Auto-add standard capabilities based on registered providers
  if (pmgr->get_data_provider_for_entity(entity_id) && !has_capability(capabilities, "data")) {
    capabilities.push_back({{"name", "data"}, {"href", href_prefix + "data"}});
  }
  if (pmgr->get_operation_provider_for_entity(entity_id) && !has_capability(capabilities, "operations")) {
    capabilities.push_back({{"name", "operations"}, {"href", href_prefix + "operations"}});
  }
  if (pmgr->get_fault_provider_for_entity(entity_id) && !has_capability(capabilities, "faults")) {
    capabilities.push_back({{"name", "faults"}, {"href", href_prefix + "faults"}});
  }

  // Plugin-registered custom capabilities (via PluginContext)
  auto * ctx = pmgr->get_context();
  if (!ctx) {
    return;
  }

  // Type-level capabilities (registered for all entities of this type)
  for (const auto & cap_name : ctx->get_type_capabilities(entity_type)) {
    if (!has_capability(capabilities, cap_name)) {
      capabilities.push_back({{"name", cap_name}, {"href", href_prefix + cap_name}});
    }
  }

  // Entity-specific capabilities
  for (const auto & cap_name : ctx->get_entity_capabilities(entity_id)) {
    if (!has_capability(capabilities, cap_name)) {
      capabilities.push_back({{"name", cap_name}, {"href", href_prefix + cap_name}});
    }
  }
}

/// Drop base capabilities whose routes cannot serve a plugin-owned entity.
/// The /data and /operations handlers dispatch plugin entities to their
/// plugin's Data/OperationProvider and 404 by contract when none is
/// registered, so advertising those capabilities is a dead link every SOVD
/// client (and the UI) follows into a guaranteed 404.
void prune_plugin_unserved_capabilities(std::vector<CapabilityBuilder::Capability> & caps,
                                        const std::string & entity_id, const GatewayNode * node) {
  auto * pmgr = node ? node->get_plugin_manager() : nullptr;
  if (!pmgr || !pmgr->get_entity_owner(entity_id)) {
    return;  // native entity: the core managers serve these collections
  }
  auto drop = [&caps](CapabilityBuilder::Capability cap) {
    caps.erase(std::remove(caps.begin(), caps.end(), cap), caps.end());
  };
  if (!pmgr->get_data_provider_for_entity(entity_id) && !pmgr->has_entity_data_route(entity_id)) {
    // Only prune when NEITHER serving path exists: data_handlers now falls
    // back to the plugin's own x-plc-data route, so an entity with that route
    // serves /data even without a DataProvider.
    drop(CapabilityBuilder::Capability::DATA);
  }
  if (!pmgr->get_operation_provider_for_entity(entity_id)) {
    drop(CapabilityBuilder::Capability::OPERATIONS);
  }
}

/// Build the ErrorInfo for a "$entity not found" 404 with the per-entity id
/// param the legacy handlers emitted (e.g. {"area_id": "..."}).
ErrorInfo make_not_found_error(const char * entity_label, const std::string & id_param_name,
                               const std::string & entity_id) {
  ErrorInfo err;
  err.code = ERR_ENTITY_NOT_FOUND;
  err.message = std::string(entity_label) + " not found";
  err.http_status = 404;
  err.params = json{{id_param_name, entity_id}};
  return err;
}

/// Build the ErrorInfo for an "Invalid <entity> ID" 400 with the same body
/// shape as the legacy handlers' `validate_entity_id` failure path.
ErrorInfo make_invalid_id_error(const char * entity_label, const std::string & id_param_name,
                                const std::string & entity_id, const std::string & details) {
  ErrorInfo err;
  err.code = ERR_INVALID_PARAMETER;
  err.message = std::string("Invalid ") + entity_label + " ID";
  err.http_status = 400;
  err.params = json{{"details", details}, {id_param_name, entity_id}};
  return err;
}

ErrorInfo make_internal_error(const char * where, const std::exception & e) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  ErrorInfo err;
  err.code = ERR_INTERNAL_ERROR;
  err.message = "Internal server error";
  err.http_status = 500;
  err.params = json{{"details", e.what()}};
  return err;
}

ErrorInfo make_internal_error_no_details(const char * where, const std::exception & e) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  ErrorInfo err;
  err.code = ERR_INTERNAL_ERROR;
  err.message = "Internal server error";
  err.http_status = 500;
  return err;
}

ErrorInfo make_invalid_request_error() {
  ErrorInfo err;
  err.code = ERR_INVALID_REQUEST;
  err.message = "Invalid request";
  err.http_status = 400;
  return err;
}

/// Read a positional path parameter ("0" -> first regex capture group). On
/// failure the validator's typed ErrorInfo is returned as-is.
tl::expected<std::string, ErrorInfo> read_path_param(const http::TypedRequest & req) {
  // The legacy handlers test for `req.matches.size() < 2` and return
  // ERR_INVALID_REQUEST/400. TypedRequest::path_param returns an
  // ERR_INVALID_PARAMETER/400 in that scenario which is different on the
  // wire. Preserve the legacy shape so integration tests do not flip.
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_invalid_request_error());
}

}  // namespace

// =============================================================================
// Area handlers
// =============================================================================

http::Result<dto::Collection<dto::AreaListItem>> DiscoveryHandlers::get_areas(const http::TypedRequest & req) {
  (void)req;
  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto areas = cache.get_areas();

    dto::Collection<dto::AreaListItem> response;
    for (const auto & area : areas) {
      // Subareas (with parent_area_id) are only visible via
      // GET /areas/{id}/subareas, not in the top-level list.
      if (!area.parent_area_id.empty()) {
        continue;
      }

      dto::AreaListItem item;
      item.id = area.id;
      item.name = area.name.empty() ? area.id : area.name;
      item.href = "/api/v1/areas/" + area.id;
      item.type = "area";

      if (!area.description.empty()) {
        item.description = area.description;
      }
      if (!area.tags.empty()) {
        item.tags = area.tags;
      }

      if (!area.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = area.namespace_path;
        dto::XMedkitArea ext;
        ext.ros2 = ros2;
        item.x_medkit = ext;
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error_no_details("get_areas", e));
  }
}

http::Result<dto::AreaDetail> DiscoveryHandlers::get_area(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string area_id = *id_result;

    // Validate entity and forward to peer if remote (aggregation support).
    auto entity_result = ctx_.validate_entity_for_route(req, area_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Local entity - look up full object from cache for detail response.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      return tl::unexpected(make_not_found_error("Area", "area_id", area_id));
    }

    const auto & area = *area_opt;

    dto::AreaDetail detail;
    detail.id = area.id;
    detail.name = area.name.empty() ? area.id : area.name;
    detail.type = "area";

    if (!area.description.empty()) {
      detail.description = area.description;
    }
    if (!area.tags.empty()) {
      detail.tags = area.tags;
    }

    std::string base_uri = "/api/v1/areas/" + area.id;
    detail.subareas = base_uri + "/subareas";
    detail.components = base_uri + "/components";
    detail.contains = base_uri + "/contains";
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.triggers = base_uri + "/triggers";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::SUBAREAS, Cap::CONTAINS, Cap::DATA,      Cap::OPERATIONS, Cap::CONFIGURATIONS,
                             Cap::FAULTS,   Cap::LOGS,     Cap::BULK_DATA, Cap::TRIGGERS};
    prune_plugin_unserved_capabilities(caps, area.id, ctx_.node());
    auto area_caps = CapabilityBuilder::build_capabilities("areas", area.id, caps);
    append_plugin_capabilities(area_caps, "areas", area.id, SovdEntityType::AREA, ctx_.node());
    detail.capabilities = area_caps;

    LinksBuilder links;
    links.self("/api/v1/areas/" + area.id).collection("/api/v1/areas");
    if (!area.parent_area_id.empty()) {
      links.parent("/api/v1/areas/" + area.parent_area_id);
    }
    detail.links = links.build();

    dto::XMedkitArea x_medkit_area;
    if (!area.namespace_path.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.ns = area.namespace_path;
      x_medkit_area.ros2 = ros2;
    }
    if (!area.parent_area_id.empty()) {
      x_medkit_area.parent_area_id = area.parent_area_id;
    }
    if (!area.contributors.empty()) {
      x_medkit_area.contributors = sorted_contributors(area.contributors);
    }
    detail.x_medkit = x_medkit_area;

    return detail;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_area", e));
  }
}

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_area_components(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string area_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      return tl::unexpected(make_invalid_id_error("area", "area_id", area_id, validation_result.error()));
    }

    const auto & cache = ctx_.node()->get_thread_safe_cache();

    if (!cache.has_area(area_id)) {
      return tl::unexpected(make_not_found_error("Area", "area_id", area_id));
    }

    const auto components = cache.get_components();
    dto::Collection<dto::ComponentListItem> response;
    for (const auto & component : components) {
      if (component.area == area_id) {
        dto::ComponentListItem item;
        item.id = component.id;
        item.name = component.name.empty() ? component.id : component.name;
        item.href = "/api/v1/components/" + component.id;
        item.type = "component";

        if (!component.description.empty()) {
          item.description = component.description;
        }

        dto::XMedkitComponent x_medkit_comp;
        set_x_medkit_available(x_medkit_comp, component.available);
        if (!component.source.empty()) {
          x_medkit_comp.source = component.source;
        }
        if (!component.namespace_path.empty()) {
          dto::XMedkitRos2 ros2;
          ros2.ns = component.namespace_path;
          x_medkit_comp.ros2 = ros2;
        }
        set_x_medkit_external(x_medkit_comp, component.external);
        item.x_medkit = x_medkit_comp;

        response.items.push_back(std::move(item));
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error_no_details("get_area_components", e));
  }
}

http::Result<dto::Collection<dto::AreaListItem>> DiscoveryHandlers::get_subareas(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string area_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      return tl::unexpected(make_invalid_id_error("area", "area_id", area_id, validation_result.error()));
    }

    // Cache-first lookup: EntityCache has merged entities from peers.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      return tl::unexpected(make_not_found_error("Area", "area_id", area_id));
    }

    // Use cache relationship index for subarea IDs, then look up each.
    auto subarea_ids = cache.get_subareas(area_id);

    dto::Collection<dto::AreaListItem> response;
    for (const auto & sub_id : subarea_ids) {
      auto subarea_opt = cache.get_area(sub_id);
      if (!subarea_opt) {
        continue;
      }
      const auto & subarea = *subarea_opt;

      dto::AreaListItem item;
      item.id = subarea.id;
      item.name = subarea.name.empty() ? subarea.id : subarea.name;
      item.href = "/api/v1/areas/" + subarea.id;
      item.type = "area";

      if (!subarea.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = subarea.namespace_path;
        dto::XMedkitArea x_medkit_area;
        x_medkit_area.ros2 = ros2;
        item.x_medkit = x_medkit_area;
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/subareas";
    links["parent"] = "/api/v1/areas/" + area_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_subareas", e));
  }
}

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_area_contains(const http::TypedRequest & req) {
  // @verifies REQ_INTEROP_006
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string area_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      return tl::unexpected(make_invalid_id_error("area", "area_id", area_id, validation_result.error()));
    }

    // Cache-first lookup: EntityCache has merged entities from peers.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      return tl::unexpected(make_not_found_error("Area", "area_id", area_id));
    }

    // Recursively collect components from this area and all descendant subareas
    // (mirrors ManifestManager::get_components_for_area behavior).
    std::vector<std::string> area_queue = {area_id};
    std::set<std::string> visited_areas;
    std::vector<std::string> all_comp_ids;

    while (!area_queue.empty()) {
      auto current_area = area_queue.back();
      area_queue.pop_back();
      if (!visited_areas.insert(current_area).second) {
        continue;
      }

      auto comp_ids = cache.get_components_for_area(current_area);
      all_comp_ids.insert(all_comp_ids.end(), comp_ids.begin(), comp_ids.end());

      auto sub_ids = cache.get_subareas(current_area);
      area_queue.insert(area_queue.end(), sub_ids.begin(), sub_ids.end());
    }

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & comp_id : all_comp_ids) {
      auto comp_opt = cache.get_component(comp_id);
      if (!comp_opt) {
        continue;
      }
      const auto & comp = *comp_opt;

      dto::ComponentListItem item;
      item.id = comp.id;
      item.name = comp.name.empty() ? comp.id : comp.name;
      item.href = "/api/v1/components/" + comp.id;
      item.type = "component";

      dto::XMedkitComponent x_medkit_comp;
      set_x_medkit_available(x_medkit_comp, comp.available);
      if (!comp.source.empty()) {
        x_medkit_comp.source = comp.source;
      }
      if (!comp.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = comp.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      set_x_medkit_external(x_medkit_comp, comp.external);
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/contains";
    links["area"] = "/api/v1/areas/" + area_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_area_contains", e));
  }
}

// =============================================================================
// Component handlers
// =============================================================================

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_components(const http::TypedRequest & req) {
  (void)req;
  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto components = cache.get_components();

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & component : components) {
      // Subcomponents (with parent_component_id) are only visible via
      // GET /components/{id}/subcomponents, not in the top-level list.
      if (!component.parent_component_id.empty()) {
        continue;
      }

      dto::ComponentListItem item;
      item.id = component.id;
      item.name = component.name.empty() ? component.id : component.name;
      item.href = "/api/v1/components/" + component.id;
      item.type = "component";

      if (!component.description.empty()) {
        item.description = component.description;
      }
      if (!component.tags.empty()) {
        item.tags = component.tags;
      }

      dto::XMedkitComponent x_medkit_comp;
      set_x_medkit_available(x_medkit_comp, component.available);
      if (!component.source.empty()) {
        x_medkit_comp.source = component.source;
      }
      if (!component.fqn.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.node = component.fqn;
        if (!component.namespace_path.empty()) {
          ros2.ns = component.namespace_path;
        }
        x_medkit_comp.ros2 = ros2;
      } else if (!component.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = component.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      if (!component.identity.empty()) {
        x_medkit_comp.identity = component.identity.to_json();
      }
      set_x_medkit_external(x_medkit_comp, component.external);
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error_no_details("get_components", e));
  }
}

http::Result<dto::ComponentDetail> DiscoveryHandlers::get_component(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string component_id = *id_result;

    // Validate entity and forward to peer if remote (aggregation support).
    auto entity_result = ctx_.validate_entity_for_route(req, component_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Local entity - look up full object from cache for detail response.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      return tl::unexpected(make_not_found_error("Component", "component_id", component_id));
    }

    const auto & comp = *comp_opt;

    dto::ComponentDetail detail;
    detail.id = comp.id;
    detail.name = comp.name.empty() ? comp.id : comp.name;
    detail.type = "component";

    if (!comp.description.empty()) {
      detail.description = comp.description;
    }
    if (!comp.tags.empty()) {
      detail.tags = comp.tags;
    }

    std::string base = "/api/v1/components/" + comp.id;
    detail.status = base + "/status";
    detail.data = base + "/data";
    detail.operations = base + "/operations";
    detail.configurations = base + "/configurations";
    detail.faults = base + "/faults";
    detail.subcomponents = base + "/subcomponents";
    detail.hosts = base + "/hosts";
    detail.logs = base + "/logs";
    detail.bulk_data = base + "/bulk-data";
    detail.cyclic_subscriptions = base + "/cyclic-subscriptions";
    detail.triggers = base + "/triggers";

    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      detail.scripts = base + "/scripts";
    }

    if (!comp.depends_on.empty()) {
      detail.depends_on = base + "/depends-on";
    }

    if (!comp.area.empty()) {
      detail.belongs_to = "/api/v1/areas/" + comp.area;
    }

    LinksBuilder links;
    links.self(base).collection("/api/v1/components");
    if (!comp.area.empty()) {
      links.add("area", "/api/v1/areas/" + comp.area);
    }
    if (!comp.parent_component_id.empty()) {
      links.parent("/api/v1/components/" + comp.parent_component_id);
    }
    detail.links = links.build();

    dto::XMedkitComponent x_medkit_comp;
    set_x_medkit_available(x_medkit_comp, comp.available);
    if (!comp.source.empty()) {
      x_medkit_comp.source = comp.source;
    }
    if (!comp.fqn.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.node = comp.fqn;
      if (!comp.namespace_path.empty()) {
        ros2.ns = comp.namespace_path;
      }
      x_medkit_comp.ros2 = ros2;
    } else if (!comp.namespace_path.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.ns = comp.namespace_path;
      x_medkit_comp.ros2 = ros2;
    }
    if (!comp.type.empty()) {
      x_medkit_comp.type = comp.type;
    }
    if (!comp.parent_component_id.empty()) {
      x_medkit_comp.parent_component_id = comp.parent_component_id;
    }
    if (!comp.depends_on.empty()) {
      x_medkit_comp.depends_on = comp.depends_on;
    }
    if (!comp.area.empty()) {
      x_medkit_comp.area = comp.area;
    }
    if (!comp.variant.empty()) {
      x_medkit_comp.variant = comp.variant;
    }
    if (!comp.description.empty()) {
      x_medkit_comp.description = comp.description;
    }
    if (!comp.contributors.empty()) {
      x_medkit_comp.contributors = sorted_contributors(comp.contributors);
    }
    if (!comp.identity.empty()) {
      x_medkit_comp.identity = comp.identity.to_json();
    }
    set_x_medkit_external(x_medkit_comp, comp.external);

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {
        Cap::STATUS,        Cap::DATA,  Cap::OPERATIONS, Cap::CONFIGURATIONS,       Cap::FAULTS,  Cap::LOGS,
        Cap::SUBCOMPONENTS, Cap::HOSTS, Cap::BULK_DATA,  Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      caps.push_back(Cap::SCRIPTS);
    }
    if (ctx_.node() && ctx_.node()->get_lock_manager()) {
      caps.push_back(Cap::LOCKS);
    }
    if (!comp.depends_on.empty()) {
      caps.push_back(Cap::DEPENDS_ON);
    }
    prune_plugin_unserved_capabilities(caps, comp.id, ctx_.node());
    auto comp_caps = CapabilityBuilder::build_capabilities("components", comp.id, caps);
    append_plugin_capabilities(comp_caps, "components", comp.id, SovdEntityType::COMPONENT, ctx_.node());
    // Capabilities at root level (SOVD standard) and in x-medkit (vendor extension for tools
    // that only read x-medkit). Apps don't duplicate because they have no vendor extensions block.
    detail.capabilities = comp_caps;
    x_medkit_comp.capabilities = comp_caps;
    detail.x_medkit = x_medkit_comp;

    return detail;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_component", e));
  }
}

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_subcomponents(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string component_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      return tl::unexpected(
          make_invalid_id_error("component", "component_id", component_id, validation_result.error()));
    }

    // Cache-first lookup: EntityCache has merged entities from peers.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      return tl::unexpected(make_not_found_error("Component", "component_id", component_id));
    }

    // Cache has no get_subcomponents(), so filter from all components.
    const auto all_components = cache.get_components();

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & sub : all_components) {
      if (sub.parent_component_id != component_id) {
        continue;
      }

      dto::ComponentListItem item;
      item.id = sub.id;
      item.name = sub.name.empty() ? sub.id : sub.name;
      item.href = "/api/v1/components/" + sub.id;
      item.type = "component";

      dto::XMedkitComponent x_medkit_comp;
      set_x_medkit_available(x_medkit_comp, sub.available);
      if (!sub.source.empty()) {
        x_medkit_comp.source = sub.source;
      }
      if (!sub.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = sub.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      set_x_medkit_external(x_medkit_comp, sub.external);
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/subcomponents";
    links["parent"] = "/api/v1/components/" + component_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_subcomponents", e));
  }
}

http::Result<dto::Collection<dto::AppListItem>> DiscoveryHandlers::get_component_hosts(const http::TypedRequest & req) {
  // @verifies REQ_INTEROP_007
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string component_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      return tl::unexpected(
          make_invalid_id_error("component", "component_id", component_id, validation_result.error()));
    }

    // Cache-first lookup: EntityCache has merged entities from peers.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      return tl::unexpected(make_not_found_error("Component", "component_id", component_id));
    }

    // Use cache relationship index for app IDs, then look up each.
    auto app_ids = cache.get_apps_for_component(component_id);

    dto::Collection<dto::AppListItem> response;
    for (const auto & aid : app_ids) {
      auto app_opt = cache.get_app(aid);
      if (!app_opt) {
        continue;
      }
      const auto & app = *app_opt;

      dto::AppListItem item;
      item.id = app.id;
      item.name = app.name.empty() ? app.id : app.name;
      item.href = "/api/v1/apps/" + app.id;
      item.type = "app";

      dto::XMedkitApp x_medkit_app;
      x_medkit_app.is_online = app.is_online;
      // Emitted only when false: an absent field means reachable.
      if (!app.available) {
        x_medkit_app.available = false;
      }
      if (!app.source.empty()) {
        x_medkit_app.source = app.source;
      }
      if (app.bound_fqn) {
        dto::XMedkitRos2 ros2;
        ros2.node = *app.bound_fqn;
        x_medkit_app.ros2 = ros2;
      }
      set_x_medkit_external(x_medkit_app, app.external);
      item.x_medkit = x_medkit_app;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/hosts";
    links["component"] = "/api/v1/components/" + component_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_component_hosts", e));
  }
}

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_component_depends_on(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string component_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      return tl::unexpected(
          make_invalid_id_error("component", "component_id", component_id, validation_result.error()));
    }

    // Cache-first lookup: EntityCache has merged entities from peers.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      return tl::unexpected(make_not_found_error("Component", "component_id", component_id));
    }

    const auto & comp = *comp_opt;

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & dep_id : comp.depends_on) {
      dto::ComponentListItem item;
      item.id = dep_id;
      item.href = "/api/v1/components/" + dep_id;
      item.type = "component";

      auto dep_opt = cache.get_component(dep_id);
      if (dep_opt) {
        item.name = dep_opt->name.empty() ? dep_id : dep_opt->name;

        dto::XMedkitComponent x_medkit_comp;
        set_x_medkit_available(x_medkit_comp, dep_opt->available);
        if (!dep_opt->source.empty()) {
          x_medkit_comp.source = dep_opt->source;
        }
        set_x_medkit_external(x_medkit_comp, dep_opt->external);
        item.x_medkit = x_medkit_comp;
      } else {
        item.name = dep_id;
        dto::XMedkitComponent x_medkit_comp;
        x_medkit_comp.missing = true;
        item.x_medkit = x_medkit_comp;
        RCLCPP_WARN(HandlerContext::logger(), "Component '%s' declares dependency on unknown component '%s'",
                    component_id.c_str(), dep_id.c_str());
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/depends-on";
    links["component"] = "/api/v1/components/" + component_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_component_depends_on", e));
  }
}

// =============================================================================
// App handlers
// =============================================================================

http::Result<dto::Collection<dto::AppListItem>> DiscoveryHandlers::get_apps(const http::TypedRequest & req) {
  (void)req;
  try {
    // Use ThreadSafeEntityCache for consistent discovery (avoids race with data endpoints).
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto apps = cache.get_apps();

    dto::Collection<dto::AppListItem> response;
    for (const auto & app : apps) {
      dto::AppListItem item;
      item.id = app.id;
      item.name = app.name.empty() ? app.id : app.name;
      item.href = "/api/v1/apps/" + app.id;
      item.type = "app";

      if (!app.description.empty()) {
        item.description = app.description;
      }
      if (!app.tags.empty()) {
        item.tags = app.tags;
      }

      dto::XMedkitApp x_medkit_app;
      if (!app.source.empty()) {
        x_medkit_app.source = app.source;
      }
      x_medkit_app.is_online = app.is_online;
      // Emitted only when false: an absent field means reachable.
      if (!app.available) {
        x_medkit_app.available = false;
      }
      if (!app.component_id.empty()) {
        x_medkit_app.component_id = app.component_id;
      }
      if (app.bound_fqn) {
        dto::XMedkitRos2 ros2;
        ros2.node = *app.bound_fqn;
        x_medkit_app.ros2 = ros2;
      }
      set_x_medkit_external(x_medkit_app, app.external);
      item.x_medkit = x_medkit_app;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_apps", e));
  }
}

http::Result<dto::AppDetail> DiscoveryHandlers::get_app(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string app_id = *id_result;

    // Validate entity and forward to peer if remote (aggregation support).
    auto entity_result = ctx_.validate_entity_for_route(req, app_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Local entity - look up full object from cache for detail response.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto app_opt = cache.get_app(app_id);
    if (!app_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      app_opt = discovery->get_app(app_id);
    }

    if (!app_opt) {
      return tl::unexpected(make_not_found_error("App", "app_id", app_id));
    }

    const auto & app = *app_opt;

    dto::AppDetail detail;
    detail.id = app.id;
    detail.name = app.name;
    detail.type = "app";

    if (!app.description.empty()) {
      detail.description = app.description;
    }
    if (!app.translation_id.empty()) {
      detail.translation_id = app.translation_id;
    }
    if (!app.tags.empty()) {
      detail.tags = app.tags;
    }

    std::string base_uri = "/api/v1/apps/" + app.id;
    detail.status = base_uri + "/status";
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.cyclic_subscriptions = base_uri + "/cyclic-subscriptions";
    detail.triggers = base_uri + "/triggers";

    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      detail.scripts = base_uri + "/scripts";
    }

    if (!app.component_id.empty()) {
      detail.is_located_on = "/api/v1/components/" + app.component_id;
      detail.belongs_to = base_uri + "/belongs-to";
    }

    if (!app.depends_on.empty()) {
      detail.depends_on = base_uri + "/depends-on";
    }

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::STATUS, Cap::DATA,      Cap::OPERATIONS,           Cap::CONFIGURATIONS, Cap::FAULTS,
                             Cap::LOGS,   Cap::BULK_DATA, Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    // Relationship endpoints are gated the same way as the top-level URI keys
    // above so the three advertising surfaces (top-level URIs, `_links`,
    // `capabilities` array) describe the same set of available collections.
    if (!app.component_id.empty()) {
      caps.push_back(Cap::IS_LOCATED_ON);
      caps.push_back(Cap::BELONGS_TO);
    }
    if (!app.depends_on.empty()) {
      caps.push_back(Cap::DEPENDS_ON);
    }
    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      caps.push_back(Cap::SCRIPTS);
    }
    if (ctx_.node() && ctx_.node()->get_lock_manager()) {
      caps.push_back(Cap::LOCKS);
    }
    prune_plugin_unserved_capabilities(caps, app.id, ctx_.node());
    auto app_caps = CapabilityBuilder::build_capabilities("apps", app.id, caps);
    append_plugin_capabilities(app_caps, "apps", app.id, SovdEntityType::APP, ctx_.node());
    detail.capabilities = app_caps;

    LinksBuilder links;
    links.self("/api/v1/apps/" + app.id).collection("/api/v1/apps");
    if (!app.component_id.empty()) {
      links.add("is-located-on", "/api/v1/components/" + app.component_id);
      links.add("belongs-to", base_uri + "/belongs-to");
    }
    auto links_json = links.build();

    if (!app.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : app.depends_on) {
        depends_links.push_back("/api/v1/apps/" + dep_id);
      }
      links_json["depends-on"] = depends_links;
    }
    detail.links = links_json;

    dto::XMedkitApp x_medkit_app;
    if (!app.source.empty()) {
      x_medkit_app.source = app.source;
    }
    x_medkit_app.is_online = app.is_online;
    // Emitted only when false: an absent field means reachable.
    if (!app.available) {
      x_medkit_app.available = false;
    }
    if (app.bound_fqn) {
      dto::XMedkitRos2 ros2;
      ros2.node = *app.bound_fqn;
      x_medkit_app.ros2 = ros2;
    }
    if (!app.component_id.empty()) {
      x_medkit_app.component_id = app.component_id;
    }
    if (!app.contributors.empty()) {
      x_medkit_app.contributors = sorted_contributors(app.contributors);
    }
    set_x_medkit_external(x_medkit_app, app.external);
    detail.x_medkit = x_medkit_app;

    return detail;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_app", e));
  }
}

http::Result<dto::Collection<dto::AppListItem>> DiscoveryHandlers::get_app_depends_on(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string app_id = *id_result;

    // validate_entity_for_route forwards the request to the owning peer when
    // the app is remote (aggregation setup) - validate_entity_id alone would
    // resolve locally only and return 404 for remote apps.
    auto entity_result = ctx_.validate_entity_for_route(req, app_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Atomic snapshot keeps the app + every dependency resolved in the same
    // cache generation; a writer refresh between per-dependency lookups could
    // otherwise yield a mix from N different generations.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_dependencies(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app) {
        snapshot.dependencies.reserve(snapshot.app->depends_on.size());
        for (const auto & dep_id : snapshot.app->depends_on) {
          snapshot.dependencies.emplace_back(dep_id, discovery->get_app(dep_id));
        }
      }
    }

    if (!snapshot.app) {
      return tl::unexpected(make_not_found_error("App", "app_id", app_id));
    }

    dto::Collection<dto::AppListItem> response;
    for (const auto & [dep_id, dep_opt] : snapshot.dependencies) {
      dto::AppListItem item;
      item.id = dep_id;
      item.href = "/api/v1/apps/" + dep_id;
      item.type = "app";

      if (dep_opt) {
        item.name = dep_opt->name.empty() ? dep_id : dep_opt->name;

        dto::XMedkitApp x_medkit_app;
        if (!dep_opt->source.empty()) {
          x_medkit_app.source = dep_opt->source;
        }
        x_medkit_app.is_online = dep_opt->is_online;
        // Emitted only when false: an absent field means reachable.
        if (!dep_opt->available) {
          x_medkit_app.available = false;
        }
        set_x_medkit_external(x_medkit_app, dep_opt->external);
        item.x_medkit = x_medkit_app;
      } else {
        item.name = dep_id;
        dto::XMedkitApp x_medkit_app;
        x_medkit_app.missing = true;
        item.x_medkit = x_medkit_app;
        RCLCPP_WARN(HandlerContext::logger(), "App '%s' declares dependency on unknown app '%s'", app_id.c_str(),
                    dep_id.c_str());
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/depends-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_app_depends_on", e));
  }
}

http::Result<dto::Collection<dto::AreaListItem>> DiscoveryHandlers::get_app_belongs_to(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string app_id = *id_result;

    // Forward to peer if app is remote (aggregation setup); locally resolved
    // entity falls through to the cache/discovery lookup below.
    auto entity_result = ctx_.validate_entity_for_route(req, app_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Atomic snapshot avoids a mixed-generation view of App -> Component ->
    // Area across concurrent cache refreshes. Fall back to the discovery
    // manager (no atomic guarantee, but rare path) when the app is not in
    // the cache.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_links(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app && !snapshot.app->component_id.empty()) {
        snapshot.component = discovery->get_component(snapshot.app->component_id);
        if (snapshot.component && !snapshot.component->area.empty()) {
          snapshot.area = discovery->get_area(snapshot.component->area);
        }
      }
    }

    if (!snapshot.app) {
      return tl::unexpected(make_not_found_error("App", "app_id", app_id));
    }

    dto::Collection<dto::AreaListItem> response;
    const auto & app = *snapshot.app;

    if (!app.component_id.empty()) {
      if (!snapshot.component) {
        // Mirror get_app_is_located_on: surface broken parent reference with
        // x-medkit.missing=true so HATEOAS clients can distinguish 'no parent
        // area' from 'manifest broken, component gone'.
        dto::AreaListItem item;
        item.id = "";
        item.name = "<unknown area>";
        item.href = "";
        item.type = "area";
        dto::XMedkitArea x_medkit_area;
        x_medkit_area.missing = true;
        x_medkit_area.unresolved_component = app.component_id;
        item.x_medkit = x_medkit_area;
        response.items.push_back(std::move(item));
        RCLCPP_WARN(HandlerContext::logger(), "App '%s' belongs-to unresolvable: parent component '%s' is unknown",
                    app_id.c_str(), app.component_id.c_str());
      } else if (!snapshot.component->area.empty()) {
        const auto & area_id_ref = snapshot.component->area;
        dto::AreaListItem item;
        item.id = area_id_ref;
        item.href = "/api/v1/areas/" + area_id_ref;
        item.type = "area";

        if (snapshot.area) {
          item.name = snapshot.area->name.empty() ? area_id_ref : snapshot.area->name;
        } else {
          item.name = area_id_ref;
          dto::XMedkitArea x_medkit_area;
          x_medkit_area.missing = true;
          item.x_medkit = x_medkit_area;
          RCLCPP_WARN(HandlerContext::logger(), "App '%s' belongs to unknown area '%s' (via component '%s')",
                      app_id.c_str(), area_id_ref.c_str(), app.component_id.c_str());
        }
        response.items.push_back(std::move(item));
      }
      // If component resolves but has no area_id assigned, items stays empty -
      // that is a legitimate manifest configuration (component without parent
      // area), not a broken reference.
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/belongs-to";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_app_belongs_to", e));
  }
}

http::Result<dto::Collection<dto::ComponentListItem>>
DiscoveryHandlers::get_app_is_located_on(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string app_id = *id_result;

    // Forward to peer if app is remote (aggregation setup).
    auto entity_result = ctx_.validate_entity_for_route(req, app_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Atomic snapshot keeps app -> component consistent across a concurrent
    // cache refresh. Fall back to discovery manager when the app is not in
    // the cache.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_links(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app && !snapshot.app->component_id.empty()) {
        snapshot.component = discovery->get_component(snapshot.app->component_id);
      }
    }

    if (!snapshot.app) {
      return tl::unexpected(make_not_found_error("App", "app_id", app_id));
    }

    dto::Collection<dto::ComponentListItem> response;
    const auto & app = *snapshot.app;

    if (!app.component_id.empty()) {
      if (snapshot.component) {
        dto::ComponentListItem item;
        item.id = snapshot.component->id;
        item.name = snapshot.component->name.empty() ? snapshot.component->id : snapshot.component->name;
        item.href = "/api/v1/components/" + snapshot.component->id;
        item.type = "component";
        response.items.push_back(std::move(item));
      } else {
        dto::ComponentListItem item;
        item.id = app.component_id;
        item.name = app.component_id;
        item.href = "/api/v1/components/" + app.component_id;
        item.type = "component";

        dto::XMedkitComponent x_medkit_comp;
        x_medkit_comp.missing = true;
        item.x_medkit = x_medkit_comp;
        response.items.push_back(std::move(item));

        RCLCPP_WARN(HandlerContext::logger(), "App '%s' references unknown component '%s'", app_id.c_str(),
                    app.component_id.c_str());
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/is-located-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_app_is_located_on", e));
  }
}

// =============================================================================
// Function handlers
// =============================================================================

http::Result<dto::Collection<dto::FunctionListItem>> DiscoveryHandlers::get_functions(const http::TypedRequest & req) {
  (void)req;
  try {
    // Use ThreadSafeEntityCache for consistent discovery (avoids race with data endpoints).
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto functions = cache.get_functions();

    dto::Collection<dto::FunctionListItem> response;
    for (const auto & func : functions) {
      dto::FunctionListItem item;
      item.id = func.id;
      item.name = func.name.empty() ? func.id : func.name;
      item.href = "/api/v1/functions/" + func.id;
      item.type = "function";

      if (!func.description.empty()) {
        item.description = func.description;
      }
      if (!func.tags.empty()) {
        item.tags = func.tags;
      }

      dto::XMedkitFunction x_medkit_func;
      if (!func.source.empty()) {
        x_medkit_func.source = func.source;
      }
      item.x_medkit = x_medkit_func;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_functions", e));
  }
}

http::Result<dto::FunctionDetail> DiscoveryHandlers::get_function(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string function_id = *id_result;

    // Validate entity and forward to peer if remote (aggregation support).
    auto entity_result = ctx_.validate_entity_for_route(req, function_id);
    if (!entity_result) {
      return tl::unexpected(flatten_validator_error(entity_result.error()));
    }

    // Local entity - look up full object from cache for detail response.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func_opt = cache.get_function(function_id);
    if (!func_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      func_opt = discovery->get_function(function_id);
    }

    if (!func_opt) {
      return tl::unexpected(make_not_found_error("Function", "function_id", function_id));
    }

    const auto & func = *func_opt;

    dto::FunctionDetail detail;
    detail.id = func.id;
    detail.name = func.name.empty() ? func.id : func.name;
    detail.type = "function";

    if (!func.description.empty()) {
      detail.description = func.description;
    }
    if (!func.translation_id.empty()) {
      detail.translation_id = func.translation_id;
    }
    if (!func.tags.empty()) {
      detail.tags = func.tags;
    }

    std::string base_uri = "/api/v1/functions/" + func.id;
    detail.hosts = base_uri + "/hosts";
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.x_medkit_graph = base_uri + "/x-medkit-graph";
    detail.cyclic_subscriptions = base_uri + "/cyclic-subscriptions";
    detail.triggers = base_uri + "/triggers";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::HOSTS, Cap::DATA,      Cap::OPERATIONS,           Cap::CONFIGURATIONS, Cap::FAULTS,
                             Cap::LOGS,  Cap::BULK_DATA, Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    prune_plugin_unserved_capabilities(caps, func.id, ctx_.node());
    auto func_caps = CapabilityBuilder::build_capabilities("functions", func.id, caps);
    append_plugin_capabilities(func_caps, "functions", func.id, SovdEntityType::FUNCTION, ctx_.node());
    detail.capabilities = func_caps;

    LinksBuilder links;
    links.self("/api/v1/functions/" + func.id).collection("/api/v1/functions");
    auto links_json = links.build();

    if (!func.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : func.depends_on) {
        depends_links.push_back("/api/v1/functions/" + dep_id);
      }
      links_json["depends-on"] = depends_links;
    }
    detail.links = links_json;

    dto::XMedkitFunction x_medkit_func;
    if (!func.source.empty()) {
      x_medkit_func.source = func.source;
    }
    if (!func.hosts.empty()) {
      x_medkit_func.hosts = func.hosts;
    }
    if (!func.description.empty()) {
      x_medkit_func.description = func.description;
    }
    if (!func.contributors.empty()) {
      x_medkit_func.contributors = sorted_contributors(func.contributors);
    }
    detail.x_medkit = x_medkit_func;

    return detail;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_function", e));
  }
}

http::Result<dto::Collection<dto::AppListItem>> DiscoveryHandlers::get_function_hosts(const http::TypedRequest & req) {
  try {
    auto id_result = read_path_param(req);
    if (!id_result) {
      return tl::unexpected(id_result.error());
    }
    const std::string function_id = *id_result;

    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      return tl::unexpected(make_invalid_id_error("function", "function_id", function_id, validation_result.error()));
    }

    // Read from cache (has merged entities from aggregation with combined hosts).
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func_opt = cache.get_function(function_id);
    if (!func_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      func_opt = discovery->get_function(function_id);
    }

    if (!func_opt) {
      return tl::unexpected(make_not_found_error("Function", "function_id", function_id));
    }

    // Use the Function's hosts list directly (includes merged hosts from all peers).
    const auto & host_ids = func_opt->hosts;

    dto::Collection<dto::AppListItem> response;
    for (const auto & app_id : host_ids) {
      auto app_opt = cache.get_app(app_id);
      if (app_opt) {
        dto::AppListItem item;
        item.id = app_opt->id;
        item.name = app_opt->name.empty() ? app_opt->id : app_opt->name;
        item.href = "/api/v1/apps/" + app_opt->id;
        item.type = "app";

        dto::XMedkitApp x_medkit_app;
        x_medkit_app.is_online = app_opt->is_online;
        // Emitted only when false: an absent field means reachable.
        if (!app_opt->available) {
          x_medkit_app.available = false;
        }
        if (!app_opt->source.empty()) {
          x_medkit_app.source = app_opt->source;
        }
        if (app_opt->bound_fqn) {
          dto::XMedkitRos2 ros2;
          ros2.node = *app_opt->bound_fqn;
          x_medkit_app.ros2 = ros2;
        }
        set_x_medkit_external(x_medkit_app, app_opt->external);
        item.x_medkit = x_medkit_app;

        response.items.push_back(std::move(item));
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/functions/" + function_id + "/hosts";
    links["function"] = "/api/v1/functions/" + function_id;
    response.links = links;

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_function_hosts", e));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
