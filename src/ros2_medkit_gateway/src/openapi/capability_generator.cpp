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

#include "capability_generator.hpp"

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "openapi_spec_builder.hpp"
#include "path_builder.hpp"

#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/member_qualified_id.hpp"
#include "ros2_medkit_gateway/core/http/operation_item_id.hpp"
#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/openapi/document_checks.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/version.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

CapabilityGenerator::CapabilityGenerator(handlers::HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr,
                                         const RouteRegistry * route_registry)
  : ctx_(ctx), node_(node), plugin_mgr_(plugin_mgr), route_registry_(route_registry), schema_builder_() {
}

// TODO(#272): Fix TOCTOU race - use compare-and-swap when storing cached specs
// TODO(#273): Use cache.snapshot() for consistent reads across multiple queries
std::optional<nlohmann::json> CapabilityGenerator::generate(const std::string & base_path) const {
  auto cache_key = get_cache_key(base_path);
  auto cached = lookup_cache(cache_key);
  if (cached.has_value()) {
    return cached;
  }

  auto result = generate_impl(base_path);
  if (result.has_value()) {
    store_cache(cache_key, *result);
  }
  return result;
}

std::optional<nlohmann::json> CapabilityGenerator::generate_impl(const std::string & base_path) const {
  auto resolved = PathResolver::resolve(base_path);

  switch (resolved.category) {
    case PathCategory::kRoot:
      return generate_root();

    case PathCategory::kEntityCollection:
      return generate_entity_collection(resolved);

    case PathCategory::kSpecificEntity:
      if (!validate_entity_hierarchy(resolved)) {
        return std::nullopt;
      }
      return generate_specific_entity(resolved);

    case PathCategory::kResourceCollection:
      if (!validate_entity_hierarchy(resolved)) {
        return std::nullopt;
      }
      return generate_resource_collection(resolved);

    case PathCategory::kSpecificResource:
      if (!validate_entity_hierarchy(resolved)) {
        return std::nullopt;
      }
      return generate_specific_resource(resolved);

    case PathCategory::kUnresolved: {
      auto plugin_spec = generate_plugin_docs(base_path);
      if (!plugin_spec.empty()) {
        return plugin_spec;
      }
      return std::nullopt;
    }
    case PathCategory::kError:
      return std::nullopt;
  }
  return std::nullopt;
}

// -----------------------------------------------------------------------------
// Root spec - server-level endpoints
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_root() const {
  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway", kGatewayVersion)
      .description(
          "SOVD-compatible REST API for ROS 2 diagnostics and control. "
          "See https://selfpatch.github.io/ros2_medkit/ for documentation.")
      .contact("selfpatch.ai", "https://selfpatch.ai")
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .tags({
          {"Server", "Gateway health, metadata, and version info"},
          {"Discovery", "Entity discovery and hierarchy navigation"},
          {"Data", "Read and write ROS 2 topic data"},
          {"Operations", "Execute ROS 2 service and action operations"},
          {"Configuration", "Read and write ROS 2 node parameters"},
          {"Faults", "Fault management and diagnostics"},
          {"Logs", "Application log access and configuration"},
          {"Bulk Data", "Large file downloads (rosbags, snapshots)"},
          {"Subscriptions", "Cyclic data subscriptions and event streaming"},
          {"Triggers", "Event-driven condition monitoring and notifications"},
          {"FaultTriggers", "Threshold rules on discovered data points that raise and auto-clear faults"},
          {"Locking", "Entity lock management for exclusive access"},
          {"Scripts", "Diagnostic script upload, execution, and management"},
          {"Updates", "Software update management"},
          {"Lifecycle", "Entity status and lifecycle control (start, restart, shutdown)"},
          {"Authentication", "JWT-based authentication"},
      });

  // Use route registry as single source of truth for paths when available
  if (route_registry_) {
    builder.add_paths(route_registry_->to_openapi_paths());
  }

  const auto & auth_config = ctx_.auth_config();
  if (auth_config.enabled) {
    builder.security_scheme("bearerAuth", {{"type", "http"}, {"scheme", "bearer"}, {"bearerFormat", "JWT"}});
  }

  // Register named schemas in components/schemas for $ref usage.
  // Generated clients use these as named types (e.g., FaultDetail, Lock, Trigger).
  nlohmann::json named_schemas;
  for (const auto & [name, schema] : SchemaBuilder::component_schemas()) {
    named_schemas[name] = schema;
  }
  builder.add_schemas(named_schemas);

  auto document = builder.build();

  // Every named schema has to be reachable from some operation, or a generated
  // client ships a type it can never receive. Only the assembled document knows
  // both halves of that question, which is why the check runs here and not in
  // `RouteRegistry::validate_completeness()`.
  //
  // Warned, not asserted: this is a Release build, and a document that ships a
  // few dead types is still a usable document - refusing to serve it would turn
  // a documentation defect into an outage.
  // `test_openapi_contract::test_no_unreachable_schemas` is what turns the
  // suite red; this line is what a developer sees without running it.
  const auto orphans = unreachable_schemas(document);
  if (!orphans.empty()) {
    std::string names;
    for (const auto & name : orphans) {
      names += (names.empty() ? "" : ", ") + name;
    }
    RCLCPP_WARN(handlers::HandlerContext::logger(),
                "OpenAPI document ships %zu schema(s) no operation can reach: %s. Bind each to the route "
                "that returns it, or drop it from dto::AllDtos.",
                orphans.size(), names.c_str());
  }

  return document;
}

// -----------------------------------------------------------------------------
// Entity collection spec (e.g., /areas, /components)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_entity_collection(const ResolvedPath & resolved) const {
  PathBuilder path_builder(schema_builder_, ctx_.auth_config().enabled);
  nlohmann::json paths;

  // Build parent path prefix from parent chain using concrete entity IDs
  // (this spec is scoped to a specific entity, not a generic template)
  std::string prefix;
  for (const auto & parent : resolved.parent_chain) {
    prefix += "/" + parent.entity_type + "/" + parent.entity_id;
  }

  // Collection listing path
  std::string collection_path = prefix + "/" + resolved.entity_type;
  paths[collection_path] = path_builder.build_entity_collection(resolved.entity_type);

  // Detail path for individual entity
  // Derive singular for path parameter
  std::string singular = resolved.entity_type;
  if (!singular.empty() && singular.back() == 's') {
    singular.pop_back();
  }
  std::string detail_path = collection_path + "/{" + singular + "_id}";
  paths[detail_path] = path_builder.build_entity_detail(resolved.entity_type);

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - " + resolved.entity_type, kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .add_paths(paths);

  return builder.build();
}

// -----------------------------------------------------------------------------
// Specific entity spec (e.g., /apps/my_app)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_specific_entity(const ResolvedPath & resolved) const {
  PathBuilder path_builder(schema_builder_, ctx_.auth_config().enabled);
  nlohmann::json paths;

  // Build entity path prefix
  std::string entity_path;
  for (const auto & parent : resolved.parent_chain) {
    entity_path += "/" + parent.entity_type + "/" + parent.entity_id;
  }
  entity_path += "/" + resolved.entity_type + "/" + resolved.entity_id;

  // Entity detail endpoint
  paths[entity_path] = path_builder.build_entity_detail(resolved.entity_type, false);

  // Add resource collection paths based on entity capabilities
  auto sovd_type = entity_type_from_keyword(resolved.entity_type);
  add_resource_collection_paths(paths, entity_path, resolved.entity_id, sovd_type);

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - " + resolved.entity_id, kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .add_paths(paths);

  return builder.build();
}

// -----------------------------------------------------------------------------
// Resource collection spec (e.g., /apps/my_app/data)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_resource_collection(const ResolvedPath & resolved) const {
  auto sovd_type_check = entity_type_from_keyword(resolved.entity_type);
  if (sovd_type_check == SovdEntityType::UNKNOWN) {
    return build_base_spec();
  }

  PathBuilder path_builder(schema_builder_, ctx_.auth_config().enabled);
  nlohmann::json paths;

  // Build entity path
  std::string entity_path;
  for (const auto & parent : resolved.parent_chain) {
    entity_path += "/" + parent.entity_type + "/" + parent.entity_id;
  }
  entity_path += "/" + resolved.entity_type + "/" + resolved.entity_id;

  std::string collection_path = entity_path + "/" + resolved.resource_collection;
  const auto & cache = node_.get_thread_safe_cache();

  // The documented id of an item has to be the id the collection emits, or the
  // spec describes a request the gateway then refuses. Both use the same rule:
  // an id that more than one item carries is qualified with its owning member,
  // an id that names one thing is left bare. What the generator cannot see is
  // a peer contributing a second item under the same id - it reads the local
  // cache only, and there is no peer view here to consult.
  const auto qualified_item_id = [](const std::string & item_id, size_t provider_count,
                                    const std::string & owner) -> std::string {
    if (provider_count < 2 || owner.empty()) {
      return item_id;
    }
    return http::make_member_qualified_id(owner, item_id);
  };

  if (resolved.resource_collection == "data") {
    auto data = cache.get_entity_data(resolved.entity_id);
    paths[collection_path] = path_builder.build_data_collection(entity_path, data.topics);
    // Add individual data item paths
    for (const auto & topic : data.topics) {
      std::string item_path = collection_path + "/" + topic.name;
      paths[item_path] = path_builder.build_data_item(entity_path, topic);
    }
  } else if (resolved.resource_collection == "operations") {
    auto ops = cache.get_app_operations(resolved.entity_id);
    // Try component/area/function-level aggregation if app-level is empty
    auto sovd_type = entity_type_from_keyword(resolved.entity_type);
    if (ops.empty() && sovd_type == SovdEntityType::COMPONENT) {
      ops = cache.get_component_operations(resolved.entity_id);
    } else if (ops.empty() && sovd_type == SovdEntityType::AREA) {
      ops = cache.get_area_operations(resolved.entity_id);
    } else if (ops.empty() && sovd_type == SovdEntityType::FUNCTION) {
      ops = cache.get_function_operations(resolved.entity_id);
    }
    paths[collection_path] = path_builder.build_operations_collection(entity_path, ops);

    // A path key that two operations produce documents one of them and drops
    // the other, so the id here has to be the id the collection emits - both
    // halves of it. `short_name_counts` decides the member half; a short name
    // one provider carries twice takes the ROS path as its item half, because
    // the member half is that same provider for both copies.
    std::unordered_map<std::string, size_t> short_name_counts;
    for (const auto & svc : ops.services) {
      ++short_name_counts[svc.name];
    }
    for (const auto & action : ops.actions) {
      ++short_name_counts[action.name];
    }
    const auto owner_of = [&ops](const std::string & full_path) -> std::string {
      auto owner = ops.owner_by_path.find(full_path);
      return owner != ops.owner_by_path.end() ? owner->second : std::string{};
    };
    const auto addressed_by_path = http::operation_paths_addressed_by_path(ops);
    const auto operation_id_of = [&](const std::string & name, const std::string & full_path) {
      return qualified_item_id(http::operation_item_half(name, full_path, addressed_by_path), short_name_counts[name],
                               owner_of(full_path));
    };

    for (const auto & svc : ops.services) {
      std::string item_path = collection_path + "/" + operation_id_of(svc.name, svc.full_path);
      paths[item_path] = path_builder.build_operation_item(entity_path, svc);
    }
    for (const auto & action : ops.actions) {
      std::string item_path = collection_path + "/" + operation_id_of(action.name, action.full_path);
      paths[item_path] = path_builder.build_operation_item(entity_path, action);
    }
  } else if (resolved.resource_collection == "configurations") {
    paths[collection_path] = path_builder.build_configurations_collection(entity_path);
  } else if (resolved.resource_collection == "faults") {
    paths[collection_path] = path_builder.build_faults_collection(entity_path);
  } else if (resolved.resource_collection == "logs") {
    paths[collection_path] = path_builder.build_logs_collection(entity_path);
    // Also add log configuration sub-endpoint
    add_log_configuration_path(paths, collection_path, entity_path);
  } else if (resolved.resource_collection == "bulk-data") {
    paths[collection_path] = path_builder.build_bulk_data_collection(entity_path);
  } else if (resolved.resource_collection == "cyclic-subscriptions") {
    paths[collection_path] = path_builder.build_cyclic_subscriptions_collection(entity_path);
  } else {
    // Unsupported resource collection - just note it exists with a generic path
    nlohmann::json generic_path;
    nlohmann::json get_op;
    get_op["summary"] = "List " + resolved.resource_collection + " for " + resolved.entity_id;
    get_op["responses"]["200"]["description"] = "Successful response";
    generic_path["get"] = std::move(get_op);
    paths[collection_path] = std::move(generic_path);
  }

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - " + resolved.entity_id + "/" + resolved.resource_collection, kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .add_paths(paths);

  return builder.build();
}

// -----------------------------------------------------------------------------
// Specific resource spec (e.g., /apps/my_app/data/temperature)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_specific_resource(const ResolvedPath & resolved) const {
  auto sovd_type_check = entity_type_from_keyword(resolved.entity_type);
  if (sovd_type_check == SovdEntityType::UNKNOWN) {
    return build_base_spec();
  }

  PathBuilder path_builder(schema_builder_, ctx_.auth_config().enabled);
  nlohmann::json paths;

  // Build full path
  std::string entity_path;
  for (const auto & parent : resolved.parent_chain) {
    entity_path += "/" + parent.entity_type + "/" + parent.entity_id;
  }
  entity_path += "/" + resolved.entity_type + "/" + resolved.entity_id;

  std::string resource_path = entity_path + "/" + resolved.resource_collection + "/" + resolved.resource_id;
  const auto & cache = node_.get_thread_safe_cache();

  if (resolved.resource_collection == "data") {
    // Look up specific topic data
    auto data = cache.get_entity_data(resolved.entity_id);
    bool found = false;
    for (const auto & topic : data.topics) {
      if (topic.name == resolved.resource_id) {
        paths[resource_path] = path_builder.build_data_item(entity_path, topic);
        found = true;
        break;
      }
    }
    if (!found) {
      // Topic not found in cache, generate a generic data path
      TopicData generic_topic;
      generic_topic.name = resolved.resource_id;
      generic_topic.type = "";
      generic_topic.direction = "publish";
      paths[resource_path] = path_builder.build_data_item(entity_path, generic_topic);
    }
  } else if (resolved.resource_collection == "operations") {
    // Look up specific operation
    auto ops = cache.get_app_operations(resolved.entity_id);
    auto sovd_type = entity_type_from_keyword(resolved.entity_type);
    if (ops.empty() && sovd_type == SovdEntityType::COMPONENT) {
      ops = cache.get_component_operations(resolved.entity_id);
    } else if (ops.empty() && sovd_type == SovdEntityType::AREA) {
      ops = cache.get_area_operations(resolved.entity_id);
    } else if (ops.empty() && sovd_type == SovdEntityType::FUNCTION) {
      ops = cache.get_function_operations(resolved.entity_id);
    }

    // The requested id may name its owning member, which is how the collection
    // addresses an operation short name that more than one member exposes.
    // Matching the bare half alone would document one member's operation under
    // the other member's id.
    auto requested = http::parse_member_qualified_id(resolved.resource_id, ops.is_aggregated);
    const auto owned_by_requested = [&ops, &requested](const std::string & full_path) {
      if (!requested.has_member) {
        return true;
      }
      auto owner = ops.owner_by_path.find(full_path);
      return owner != ops.owner_by_path.end() && owner->second == requested.member_id;
    };

    bool found = false;
    for (const auto & svc : ops.services) {
      if (svc.name == requested.item_id && owned_by_requested(svc.full_path)) {
        paths[resource_path] = path_builder.build_operation_item(entity_path, svc);
        found = true;
        break;
      }
    }
    if (!found) {
      for (const auto & action : ops.actions) {
        if (action.name == requested.item_id && owned_by_requested(action.full_path)) {
          paths[resource_path] = path_builder.build_operation_item(entity_path, action);
          found = true;
          break;
        }
      }
    }
    if (!found) {
      // Operation not found - generate generic
      ServiceInfo generic_svc;
      generic_svc.name = resolved.resource_id;
      generic_svc.type = "";
      paths[resource_path] = path_builder.build_operation_item(entity_path, generic_svc);
    }
  } else if (resolved.resource_collection == "faults") {
    paths[resource_path] = path_builder.build_faults_collection(entity_path);
  } else {
    // Generic resource path
    nlohmann::json generic_path;
    nlohmann::json get_op;
    get_op["summary"] = "Get " + resolved.resource_id;
    get_op["responses"]["200"]["description"] = "Successful response";
    generic_path["get"] = std::move(get_op);
    paths[resource_path] = std::move(generic_path);
  }

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - " + resolved.resource_id, kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .add_paths(paths);

  return builder.build();
}

// -----------------------------------------------------------------------------
// Plugin route docs
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_plugin_docs(const std::string & path) const {
  if (!plugin_mgr_) {
    return {};
  }

  auto descriptions = plugin_mgr_->collect_route_descriptions();
  nlohmann::json matching_paths = nlohmann::json::object();

  for (const auto & desc : descriptions) {
    auto paths_json = desc.to_json();  // CapabilityGenerator is friend
    for (auto & [key, value] : paths_json.items()) {
      if (key == path || key.find(path + "/") == 0) {
        matching_paths[key] = value;
      }
    }
  }

  if (matching_paths.empty()) {
    return {};
  }

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - Plugin", kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .add_paths(matching_paths);
  return builder.build();
}

// -----------------------------------------------------------------------------
// Entity hierarchy validation
// -----------------------------------------------------------------------------

bool CapabilityGenerator::validate_entity_hierarchy(const ResolvedPath & resolved) const {
  const auto & cache = node_.get_thread_safe_cache();

  // Validate the main entity exists
  if (!resolved.entity_id.empty()) {
    auto entity_type = entity_type_from_keyword(resolved.entity_type);
    switch (entity_type) {
      case SovdEntityType::AREA:
        if (!cache.has_area(resolved.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::COMPONENT:
        if (!cache.has_component(resolved.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::APP:
        if (!cache.has_app(resolved.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::FUNCTION:
        if (!cache.has_function(resolved.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::SERVER:
      case SovdEntityType::UNKNOWN:
      default:
        return false;
    }
  }

  // Validate each parent in the chain
  for (const auto & parent : resolved.parent_chain) {
    auto parent_type = entity_type_from_keyword(parent.entity_type);
    switch (parent_type) {
      case SovdEntityType::AREA:
        if (!cache.has_area(parent.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::COMPONENT:
        if (!cache.has_component(parent.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::APP:
        if (!cache.has_app(parent.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::FUNCTION:
        if (!cache.has_function(parent.entity_id)) {
          return false;
        }
        break;
      case SovdEntityType::SERVER:
      case SovdEntityType::UNKNOWN:
      default:
        return false;
    }
  }

  // Validate parent-child relationships
  // Walk the chain: each parent must actually contain the next entity
  if (!resolved.parent_chain.empty()) {
    for (size_t i = 0; i < resolved.parent_chain.size(); ++i) {
      const auto & parent = resolved.parent_chain[i];
      std::string child_id;
      std::string child_type_keyword;

      if (i + 1 < resolved.parent_chain.size()) {
        child_id = resolved.parent_chain[i + 1].entity_id;
        child_type_keyword = resolved.parent_chain[i + 1].entity_type;
      } else {
        child_id = resolved.entity_id;
        child_type_keyword = resolved.entity_type;
      }

      if (child_id.empty()) {
        continue;
      }

      auto parent_sovd_type = entity_type_from_keyword(parent.entity_type);
      auto child_sovd_type = entity_type_from_keyword(child_type_keyword);

      // Verify the parent-child relationship
      if (parent_sovd_type == SovdEntityType::AREA && child_sovd_type == SovdEntityType::COMPONENT) {
        auto children = cache.get_components_for_area(parent.entity_id);
        bool found = false;
        for (const auto & c : children) {
          if (c == child_id) {
            found = true;
            break;
          }
        }
        if (!found) {
          return false;
        }
      } else if (parent_sovd_type == SovdEntityType::AREA && child_sovd_type == SovdEntityType::AREA) {
        // Subarea relationship
        auto children = cache.get_subareas(parent.entity_id);
        bool found = false;
        for (const auto & c : children) {
          if (c == child_id) {
            found = true;
            break;
          }
        }
        if (!found) {
          return false;
        }
      } else if (parent_sovd_type == SovdEntityType::COMPONENT && child_sovd_type == SovdEntityType::APP) {
        auto children = cache.get_apps_for_component(parent.entity_id);
        bool found = false;
        for (const auto & c : children) {
          if (c == child_id) {
            found = true;
            break;
          }
        }
        if (!found) {
          return false;
        }
      }
      // Other relationships (function->apps etc.) are less strictly hierarchical
    }
  }

  return true;
}

// -----------------------------------------------------------------------------
// Helper methods
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::build_base_spec() const {
  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway", kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server");

  const auto & auth_config = ctx_.auth_config();
  if (auth_config.enabled) {
    builder.security_scheme("bearerAuth", {{"type", "http"}, {"scheme", "bearer"}, {"bearerFormat", "JWT"}});
  }

  return builder.build();
}

std::string CapabilityGenerator::build_server_url() const {
  // Read host/port independently so a missing host doesn't clobber a valid port
  std::string host = "localhost";
  int port = 8080;
  try {
    host = node_.get_parameter("server.host").as_string();
  } catch (...) {
  }
  try {
    port = static_cast<int>(node_.get_parameter("server.port").as_int());
  } catch (...) {
  }

  // Use localhost for display if bound to all interfaces
  if (host == "0.0.0.0") {
    host = "localhost";
  }

  // Determine protocol based on TLS config
  std::string protocol = "http";
  if (ctx_.tls_config().enabled) {
    protocol = "https";
  }

  return protocol + "://" + host + ":" + std::to_string(port) + API_BASE_PATH;
}

SovdEntityType CapabilityGenerator::entity_type_from_keyword(const std::string & keyword) {
  if (keyword == "areas" || keyword == "subareas") {
    return SovdEntityType::AREA;
  }
  if (keyword == "components" || keyword == "subcomponents") {
    return SovdEntityType::COMPONENT;
  }
  if (keyword == "apps") {
    return SovdEntityType::APP;
  }
  if (keyword == "functions") {
    return SovdEntityType::FUNCTION;
  }
  return SovdEntityType::UNKNOWN;
}

void CapabilityGenerator::add_log_configuration_path(nlohmann::json & paths, const std::string & logs_path,
                                                     const std::string & entity_path) {
  nlohmann::json config_path_item;

  nlohmann::json config_get;
  config_get["tags"] = nlohmann::json::array({"Logs"});
  config_get["summary"] = "Get log configuration for " + entity_path;
  config_get["description"] = "Returns the current log level configuration.";
  config_get["responses"]["200"]["description"] = "Current log configuration";
  config_get["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("LogConfiguration");
  config_path_item["get"] = std::move(config_get);

  nlohmann::json config_put;
  config_put["tags"] = nlohmann::json::array({"Logs"});
  config_put["summary"] = "Update log configuration for " + entity_path;
  config_put["description"] = "Update the log level configuration.";
  config_put["requestBody"]["required"] = true;
  config_put["requestBody"]["content"]["application/json"]["schema"] = SchemaBuilder::ref("LogConfiguration");
  config_put["responses"]["204"]["description"] = "Log configuration updated";
  config_path_item["put"] = std::move(config_put);

  paths[logs_path + "/configuration"] = std::move(config_path_item);
}

void CapabilityGenerator::add_resource_collection_paths(nlohmann::json & paths, const std::string & entity_path,
                                                        const std::string & entity_id,
                                                        ros2_medkit_gateway::SovdEntityType entity_type) const {
  if (entity_type == SovdEntityType::UNKNOWN) {
    return;
  }
  PathBuilder path_builder(schema_builder_, ctx_.auth_config().enabled);
  auto caps = EntityCapabilities::for_type(entity_type);
  const auto & cache = node_.get_thread_safe_cache();

  for (const auto & col : caps.collections()) {
    std::string col_path = entity_path + "/" + to_path_segment(col);

    switch (col) {
      case ResourceCollection::DATA: {
        auto data = cache.get_entity_data(entity_id);
        paths[col_path] = path_builder.build_data_collection(entity_path, data.topics);
        break;
      }
      case ResourceCollection::OPERATIONS: {
        AggregatedOperations ops;
        switch (entity_type) {
          case SovdEntityType::APP:
            ops = cache.get_app_operations(entity_id);
            break;
          case SovdEntityType::COMPONENT:
            ops = cache.get_component_operations(entity_id);
            break;
          case SovdEntityType::AREA:
            ops = cache.get_area_operations(entity_id);
            break;
          case SovdEntityType::FUNCTION:
            ops = cache.get_function_operations(entity_id);
            break;
          case SovdEntityType::SERVER:
          case SovdEntityType::UNKNOWN:
          default:
            break;
        }
        paths[col_path] = path_builder.build_operations_collection(entity_path, ops);
        break;
      }
      case ResourceCollection::CONFIGURATIONS:
        paths[col_path] = path_builder.build_configurations_collection(entity_path);
        break;
      case ResourceCollection::FAULTS:
        paths[col_path] = path_builder.build_faults_collection(entity_path);
        break;
      case ResourceCollection::BULK_DATA:
        paths[col_path] = path_builder.build_bulk_data_collection(entity_path);
        break;
      case ResourceCollection::CYCLIC_SUBSCRIPTIONS:
        paths[col_path] = path_builder.build_cyclic_subscriptions_collection(entity_path);
        break;
      case ResourceCollection::LOGS:
        paths[col_path] = path_builder.build_logs_collection(entity_path);
        add_log_configuration_path(paths, col_path, entity_path);
        break;
      case ResourceCollection::DATA_LISTS:
      case ResourceCollection::LOCKS:
      case ResourceCollection::MODES:
      case ResourceCollection::COMMUNICATION_LOGS:
      case ResourceCollection::TRIGGERS:
      case ResourceCollection::SCRIPTS:
      case ResourceCollection::UPDATES:
      default:
        // For other collections we don't have specific builders, add generic listing
        {
          nlohmann::json generic_path;
          nlohmann::json get_op;
          get_op["summary"] = "List " + to_string(col) + " for " + entity_id;
          get_op["responses"]["200"]["description"] = "Successful response";
          generic_path["get"] = std::move(get_op);
          paths[col_path] = std::move(generic_path);
        }
        break;
    }
  }
}

// -----------------------------------------------------------------------------
// Cache helpers
// -----------------------------------------------------------------------------

std::string CapabilityGenerator::get_cache_key(const std::string & path) const {
  auto & cache = node_.get_thread_safe_cache();
  auto generation = cache.generation();
  {
    std::unique_lock lock(cache_mutex_);
    if (generation != cached_generation_) {
      spec_cache_.clear();
      cached_generation_ = generation;
    }
  }
  return std::to_string(generation) + ":" + path;
}

std::optional<nlohmann::json> CapabilityGenerator::lookup_cache(const std::string & key) const {
  std::shared_lock lock(cache_mutex_);
  auto it = spec_cache_.find(key);
  if (it != spec_cache_.end()) {
    return it->second;
  }
  return std::nullopt;
}

void CapabilityGenerator::store_cache(const std::string & key, const nlohmann::json & spec) const {
  std::unique_lock lock(cache_mutex_);
  if (spec_cache_.size() >= kMaxCacheSize) {
    spec_cache_.clear();  // Simple eviction: clear all when full
  }
  spec_cache_[key] = spec;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
