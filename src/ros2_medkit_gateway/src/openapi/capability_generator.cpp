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

#include <algorithm>
#include <cctype>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "openapi_spec_builder.hpp"
#include "path_builder.hpp"

#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/member_qualified_id.hpp"
#include "ros2_medkit_gateway/core/http/operation_item_id.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/openapi/document_checks.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/version.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

namespace {

/// The one bearer-token scheme any document that mentions security refers to.
///
/// The description is part of the definition because the name alone says
/// nothing about what a *given* gateway does with a token, and the document is
/// served by that gateway. It describes the state this document is already in
/// rather than the settings that produced it: every operation states its own
/// requirement, and the 401/403 an operation declares is one it can answer.
nlohmann::json bearer_scheme() {
  return nlohmann::json{{"type", "http"},
                        {"scheme", "bearer"},
                        {"bearerFormat", "JWT"},
                        {"description",
                         "JWT bearer token. Every operation states its own requirement, and it is the requirement "
                         "this gateway enforces as configured: a scope names the role the permission table grants "
                         "for that operation, and the empty requirement (`security: []`) marks an operation "
                         "reachable with no token at all. The `Unauthorized` and `Forbidden` responses appear on "
                         "exactly the operations the authentication middleware can refuse. Both follow "
                         "`auth.enabled` together with `auth.require_auth_for`: under `write` the GETs carry the "
                         "empty requirement and no refusal statuses, under `none` so does every operation, and with "
                         "`auth.enabled` off the document drops per-operation requirements and the document-level "
                         "one alike. Re-read the document after changing either setting - it is generated per "
                         "gateway, not per release."}};
}

/// The five verbs an OpenAPI Path Item Object can carry that this gateway
/// registers. Anything else under a path item (`parameters`, `summary`,
/// vendor extensions) is not an operation and must not be rewritten as one.
bool is_operation_key(const std::string & key) {
  return key == "get" || key == "post" || key == "put" || key == "patch" || key == "delete";
}

/// An OpenAPI method key as the request line spells it. The auth policies
/// compare against upper-case literals; a document spells its methods in
/// lower case.
std::string to_upper(std::string method) {
  for (char & c : method) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return method;
}

/// Call `visit(METHOD, path, operation)` for every operation in an assembled
/// document. Walks whatever `paths` contains, so it does not care which
/// producer wrote an operation - registry, plugin fold, or the concrete items
/// a `<entity-path>/docs` sub-document projects.
template <typename Visitor>
void for_each_operation(nlohmann::json & document, Visitor && visit) {
  auto paths = document.find("paths");
  if (paths == document.end() || !paths->is_object()) {
    return;
  }
  for (auto path_it = paths->begin(); path_it != paths->end(); ++path_it) {
    if (!path_it.value().is_object()) {
      continue;
    }
    for (auto op_it = path_it.value().begin(); op_it != path_it.value().end(); ++op_it) {
      if (!op_it.value().is_object() || !is_operation_key(op_it.key())) {
        continue;
      }
      visit(to_upper(op_it.key()), path_it.key(), op_it.value());
    }
  }
}

}  // namespace

CapabilityGenerator::CapabilityGenerator(handlers::HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr,
                                         const RouteRegistry * route_registry, DocsCacheBounds bounds)
  : ctx_(ctx)
  , node_(node)
  , plugin_mgr_(plugin_mgr)
  , route_registry_(route_registry)
  , schema_builder_()
  , bounds_(bounds) {
}

// TODO(#272): Fix TOCTOU race - use compare-and-swap when storing cached specs
// TODO(#273): Use cache.snapshot() for consistent reads across multiple queries
std::optional<std::string> CapabilityGenerator::generate_serialized(const std::string & base_path) const {
  auto cache_key = get_cache_key(base_path);
  auto cached = lookup_cache(cache_key);
  if (cached.has_value()) {
    return cached;
  }

  auto result = generate_impl(base_path);
  if (!result.has_value()) {
    return std::nullopt;
  }

  // `dump(2)` is exactly what `http::detail::write_json_body` applies to a
  // document, so serializing here rather than at the writer leaves the bytes
  // on the wire unchanged - and lets the cache hold them instead of a DOM.
  auto document = result->dump(2);
  store_cache(cache_key, document);
  return document;
}

std::optional<nlohmann::json> CapabilityGenerator::generate(const std::string & base_path) const {
  auto document = generate_serialized(base_path);
  if (!document.has_value()) {
    return std::nullopt;
  }

  // Non-throwing parse: handlers in this gateway report failure by value
  // rather than by exception. The input is text this class just produced with
  // `dump(2)`, so a parse failure would mean nlohmann cannot read back its own
  // output; nullopt is then the only honest answer available here.
  auto parsed = nlohmann::json::parse(*document, /*cb=*/nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded()) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<nlohmann::json> CapabilityGenerator::generate_impl(const std::string & base_path) const {
  auto document = build_document(base_path);

  // Applied once, here, over the finished document rather than inside any one
  // producer - and that placement is the point.
  //
  // The rule is a property of the *gateway*, not of where an operation came
  // from: this document is served from `/docs` by a running gateway and
  // describes it. Two producers emit a requirement - `RouteEntry::
  // requires_role` through `RouteRegistry::to_openapi_paths()`, and a plugin's
  // `OperationDesc::requires_role` through the fold - and both reach every
  // `<entity-path>/docs` sub-document as well, because those are a projection
  // of the same two. Putting the rule in either producer would have meant a
  // second copy in the other. Here it sits after all of them, so a new one
  // inherits it.
  if (document.has_value()) {
    project_security_onto_enforcement(*document);
  }
  return document;
}

void CapabilityGenerator::project_security_onto_enforcement(nlohmann::json & document) const {
  // Two configurations, two different corrections, because the document-level
  // requirement differs between them.
  //
  // With `auth.enabled` off the middleware admits every caller and the
  // document carries no document-level `security` (`openapi_spec_builder`
  // only adds one when auth is on), so an operation that publishes nothing
  // already reads as "no token needed" - erasing is enough, and is what a
  // reader of the scheme description expects.
  if (!ctx_.auth_config().enabled) {
    for_each_operation(document, [](const std::string &, const std::string &, nlohmann::json & operation) {
      operation.erase("security");
    });
    return;
  }

  // With it on the document *does* carry `security: [{bearerAuth: []}]`, and
  // an operation that publishes nothing inherits it - so erasing here would
  // say the opposite of the intent. OpenAPI's override for "reachable with no
  // token" is the explicit empty list, which is also exactly what
  // `RouteEntry::public_route()` already emits for `/auth/*`; writing it on
  // every operation the policy admits puts those routes and the ones a
  // permissive `require_auth_for` opens on the same footing.
  const auto * auth_manager = ctx_.auth_manager();
  if (auth_manager == nullptr) {
    return;
  }
  for_each_operation(document,
                     [auth_manager](const std::string & method, const std::string & path, nlohmann::json & operation) {
                       // `requires_authentication` is the middleware's own predicate, called on
                       // the same request line a client would send: the path keys here are
                       // relative to the API base, and the concrete ids a sub-document publishes
                       // substitute for the templates without changing what the *reachable*
                       // policies match - all three key on the method and the `/api/v1/auth/`
                       // prefix, neither of which a substitution touches.
                       //
                       // `ConfigurableAuthRequirementPolicy` matches whole path patterns and
                       // would care, but nothing constructs it: `AuthManager` builds its policy
                       // from `require_auth_for`, which has exactly three values. That is also
                       // what makes `test_auth_policy_contract`'s three gateways a *complete*
                       // sweep of the policy input space rather than a sample of it.
                       if (!auth_manager->requires_authentication(method, std::string(API_BASE_PATH) + path)) {
                         operation["security"] = nlohmann::json::array();
                       }
                     });
}

std::optional<nlohmann::json> CapabilityGenerator::build_document(const std::string & base_path) const {
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
  std::vector<TagInfo> tags{
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
      {"PlannedStops", "Windows during which faults are expected, declared by an operator"},
      {"FaultTriggers", "Threshold rules on discovered data points that raise and auto-clear faults"},
      {"Locking", "Entity lock management for exclusive access"},
      {"Scripts", "Diagnostic script upload, execution, and management"},
      {"Updates", "Software update management"},
      {"Lifecycle", "Entity status and lifecycle control (start, restart, shutdown)"},
      {"Authentication", "JWT-based authentication"},
  };

  const auto registry_paths = route_registry_ ? route_registry_->to_openapi_paths() : nlohmann::json::object();
  const auto extension_paths = plugin_paths();

  // A plugin picks its own tag, so the tag list cannot be a literal and stay
  // complete - `test_health` and `test_openapi_contract` both fail on a tag an
  // operation uses without the document declaring it. Collected from the
  // operations themselves so the invariant holds for whatever a plugin
  // chooses, rather than for the one tag the in-tree plugin happens to use.
  for (const auto & [path, item] : extension_paths.items()) {
    for (const auto & [method, operation] : item.items()) {
      if (!operation.is_object()) {
        continue;
      }
      // Walked rather than looked up with `find()`: GCC inlines the iterator
      // dereference into a -Wnull-dereference false positive, the same one
      // `route_registry.cpp` documents around its parameter scan, and the
      // build treats it as an error.
      for (const auto & [op_key, op_value] : operation.items()) {
        if (op_key != "tags" || !op_value.is_array()) {
          continue;
        }
        for (const auto & tag : op_value) {
          if (!tag.is_string()) {
            continue;
          }
          const auto & name = tag.get_ref<const std::string &>();
          const bool already_declared = std::any_of(tags.begin(), tags.end(), [&name](const TagInfo & declared) {
            return declared.name == name;
          });
          if (!already_declared) {
            tags.push_back({name, "Vendor extension resources served by a gateway plugin"});
          }
        }
      }
    }
  }

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway", kGatewayVersion)
      .description(
          "SOVD-compatible REST API for ROS 2 diagnostics and control. "
          "See https://selfpatch.github.io/ros2_medkit/ for documentation.")
      .contact("selfpatch.ai", "https://selfpatch.ai")
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      .tags(tags)
      .add_paths(registry_paths);

  // Plugin-served routes are mounted on the same HTTP server as the
  // registry's, so a client that cannot see them here has no other way to
  // learn they exist. Merged after the registry's paths and never over them:
  // a plugin pattern that shadows a gateway route is a routing defect, and
  // silently replacing the gateway's description of that path would hide it.
  for (const auto & [path, item] : extension_paths.items()) {
    if (registry_paths.contains(path)) {
      RCLCPP_WARN(handlers::HandlerContext::logger(),
                  "Plugin describes path '%s', which the gateway already documents; keeping the gateway's "
                  "description. The plugin's route is still mounted - check it is not shadowing a gateway route.",
                  path.c_str());
      continue;
    }
    builder.add_paths(nlohmann::json{{path, item}});
  }

  // Registered whether or not authentication is on. With it on, this is the
  // scheme the per-operation requirements name - an operation cannot reference
  // a scheme the document does not define. With it off there are no such
  // requirements at all (`generate_impl` strips them from the assembled
  // document), and the definition is kept anyway because a definition nothing
  // references asserts nothing about this gateway - it only tells a reader
  // what `bearerAuth` would mean.
  //
  // The *document-level* requirement - "every request needs a token" - is the
  // part that would be untrue with `auth.enabled` off, so that stays gated.
  builder.security_scheme("bearerAuth", bearer_scheme(), ctx_.auth_config().enabled);

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
// Sub-documents - a projection of the document the gateway serves
//
// Every `<entity-path>/docs` document below is a slice of `served_paths()`,
// with the ids the caller named substituted into the path templates. Nothing
// here describes a route a second time: what a sub-document says about an
// operation is what the root document says about it, minus the templating the
// caller has already resolved.
//
// The exception is a data item and an operation item, whose payload schema
// comes from the ROS type in the entity cache - a fact no registration holds.
// `add_cache_derived_items` is the whole of that exception.
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::served_paths() const {
  // Bound to named locals. Both calls return by value, and iterating `.items()`
  // on the temporary walks a destroyed object.
  const nlohmann::json registry_paths =
      route_registry_ ? route_registry_->to_openapi_paths() : nlohmann::json::object();
  const nlohmann::json extension_paths = plugin_paths();

  nlohmann::json served = registry_paths;
  for (const auto & [path, item] : extension_paths.items()) {
    if (served.contains(path)) {
      continue;  // `generate_root` warns about the shadowing; once is enough.
    }
    served[path] = item;
  }
  return served;
}

nlohmann::json CapabilityGenerator::project(const nlohmann::json & served, const std::string & template_prefix,
                                            const std::vector<PathBinding> & bindings) {
  const nlohmann::json subtree = paths_under(served, template_prefix);

  nlohmann::json bound = nlohmann::json::object();
  for (const auto & [key, item] : subtree.items()) {
    std::string path = key;
    nlohmann::json path_item = item;
    for (const auto & [parameter, value] : bindings) {
      const std::string placeholder = "{" + parameter + "}";
      const auto pos = path.find(placeholder);
      if (pos == std::string::npos) {
        continue;
      }
      path.replace(pos, placeholder.size(), value);
      strip_entity_path_parameter(path_item, parameter);
    }
    bound[path] = std::move(path_item);
  }
  return bound;
}

nlohmann::json CapabilityGenerator::build_subtree_document(const std::string & title,
                                                           const nlohmann::json & paths) const {
  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - " + title, kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      // The projected operations carry whatever `security` their registration
      // declared, so the scheme they name has to be defined here as well - an
      // operation cannot reference a scheme its document does not have. With
      // `auth.enabled` off `generate_impl` removes those requirements from the
      // finished document, and the definition stays for the same reason it does
      // in the root document: it asserts nothing about this gateway.
      .security_scheme("bearerAuth", bearer_scheme(), ctx_.auth_config().enabled)
      .add_paths(paths);

  // Only the schemas this slice reaches. The root document carries all of
  // `dto::AllDtos`; an entity page that did the same would ship every DTO the
  // gateway knows on every request, and one that carried none - which is what
  // these documents used to do - publishes `$ref`s no client can resolve.
  nlohmann::json named_schemas = nlohmann::json::object();
  for (const auto & [name, schema] : referenced_schemas(paths, SchemaBuilder::component_schemas())) {
    named_schemas[name] = schema;
  }
  builder.add_schemas(named_schemas);

  return builder.build();
}

std::pair<std::string, std::vector<CapabilityGenerator::PathBinding>>
CapabilityGenerator::entity_template(const ResolvedPath & resolved, bool include_self) {
  std::string prefix;
  std::vector<PathBinding> bindings;

  auto append = [&prefix, &bindings](const std::string & entity_type, const std::string & entity_id) {
    // "areas" -> "area_id", "subcomponents" -> "subcomponent_id": the same
    // singular-plus-`_id` shape every entity route in `rest_server.cpp` is
    // registered under.
    std::string parameter = entity_type;
    if (!parameter.empty() && parameter.back() == 's') {
      parameter.pop_back();
    }
    parameter += "_id";
    prefix += "/" + entity_type + "/{" + parameter + "}";
    bindings.push_back({parameter, entity_id});
  };

  for (const auto & parent : resolved.parent_chain) {
    append(parent.entity_type, parent.entity_id);
  }
  if (include_self) {
    append(resolved.entity_type, resolved.entity_id);
  } else {
    prefix += "/" + resolved.entity_type;
  }
  return {prefix, bindings};
}

std::string CapabilityGenerator::concrete_path(const std::string & template_prefix,
                                               const std::vector<PathBinding> & bindings) {
  std::string path = template_prefix;
  for (const auto & [parameter, value] : bindings) {
    const std::string placeholder = "{" + parameter + "}";
    const auto pos = path.find(placeholder);
    if (pos != std::string::npos) {
      path.replace(pos, placeholder.size(), value);
    }
  }
  return path;
}

std::optional<std::string> CapabilityGenerator::single_parameter_segment_under(const nlohmann::json & served,
                                                                               const std::string & prefix) {
  if (!served.is_object()) {
    return std::nullopt;
  }
  std::optional<std::string> found;
  for (auto entry = served.begin(); entry != served.end(); ++entry) {
    const std::string & key = entry.key();
    if (key.size() <= prefix.size() + 1 || key.compare(0, prefix.size(), prefix) != 0 || key[prefix.size()] != '/') {
      continue;
    }
    const std::string segment = key.substr(prefix.size() + 1);
    if (segment.find('/') != std::string::npos || segment.front() != '{' || segment.back() != '}') {
      continue;
    }
    const std::string parameter = segment.substr(1, segment.size() - 2);
    if (found.has_value() && *found != parameter) {
      return std::nullopt;  // ambiguous - see the header comment
    }
    found = parameter;
  }
  return found;
}

// -----------------------------------------------------------------------------
// Entity collection spec (e.g., /areas, /components)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_entity_collection(const ResolvedPath & resolved) const {
  const auto [prefix, bindings] = entity_template(resolved, false);
  const nlohmann::json served = served_paths();
  return build_subtree_document(resolved.entity_type, project(served, prefix, bindings));
}

// -----------------------------------------------------------------------------
// Specific entity spec (e.g., /apps/my_app)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_specific_entity(const ResolvedPath & resolved) const {
  const auto [prefix, bindings] = entity_template(resolved, true);
  const nlohmann::json served = served_paths();
  return build_subtree_document(resolved.entity_id, project(served, prefix, bindings));
}

// -----------------------------------------------------------------------------
// Resource collection spec (e.g., /apps/my_app/data)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_resource_collection(const ResolvedPath & resolved) const {
  auto [prefix, bindings] = entity_template(resolved, true);
  const std::string entity_path = concrete_path(prefix, bindings);
  prefix += "/" + resolved.resource_collection;

  const nlohmann::json served = served_paths();
  auto paths = project(served, prefix, bindings);
  add_cache_derived_items(paths, resolved, entity_path);

  return build_subtree_document(resolved.entity_id + "/" + resolved.resource_collection, paths);
}

// -----------------------------------------------------------------------------
// Specific resource spec (e.g., /apps/my_app/data/temperature)
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::generate_specific_resource(const ResolvedPath & resolved) const {
  auto [prefix, bindings] = entity_template(resolved, true);
  const std::string entity_path = concrete_path(prefix, bindings);
  const std::string collection_prefix = prefix + "/" + resolved.resource_collection;

  const nlohmann::json served = served_paths();

  // Which template the item sits under is read from the registry, not
  // tabulated: the item route is the one key that extends the collection by
  // exactly one segment and whose segment is a whole `{param}`. A collection
  // whose item segment is a literal - `/logs/configuration` - has none, and
  // the literal is the prefix.
  const auto item_parameter = single_parameter_segment_under(served, collection_prefix);
  if (item_parameter.has_value()) {
    prefix = collection_prefix + "/{" + *item_parameter + "}";
    bindings.push_back({*item_parameter, resolved.resource_id});
  } else {
    prefix = collection_prefix + "/" + resolved.resource_id;
  }

  auto paths = project(served, prefix, bindings);
  add_cache_derived_items(paths, resolved, entity_path);

  return build_subtree_document(resolved.resource_id, paths);
}

bool CapabilityGenerator::adopt_projected_framework(nlohmann::json & item, const nlohmann::json & paths,
                                                    const std::string & sibling_key) {
  // A concrete item and the templated route it sits beside are the *same
  // route* - `/apps/x/data/temperature` is served by the handler registered at
  // `/apps/{app_id}/data/{data_id}`. Only the payload schema differs, because
  // only the payload schema depends on which topic was named.
  //
  // So everything that is not the payload is taken from the projected sibling
  // rather than rebuilt here. That is what closes four ways these items used to
  // contradict the document they sit in: no `security` at all where the sibling
  // published `viewer`/`operator`, no 416 where the sibling declared one, no
  // lock 409 on the write, and 401/403 carrying an inline SOVD `GenericError`
  // schema where the middleware answers the RFC 6749 shape the `Unauthorized`
  // and `Forbidden` components describe.
  //
  // Inheriting beats restating: the sibling already tracks `auth.require_auth_for`,
  // `locking.enabled`, rate limiting and aggregation, so a concrete item cannot
  // drift from the gateway's configuration without the projection drifting too.
  // Returns false when the sibling is not there, and the caller must then leave
  // the projection alone rather than write the un-enriched item over it. The
  // first version returned void on a miss and the caller wrote regardless,
  // which is how `<entity>/operations/<op>/docs` came to publish an operation
  // with no `security` and no 416 on a gateway that answers 401 for it: at
  // specific-resource scope `project()` has already substituted the item id
  // into the path *key*, so the templated key this used to look for does not
  // exist there. The key is now supplied by the caller, which knows which scope
  // it is in, and a miss is a refusal rather than a silent pass-through.
  const auto sibling = paths.find(sibling_key);
  if (sibling == paths.end() || !sibling->is_object()) {
    return false;
  }
  for (auto method_it = item.begin(); method_it != item.end(); ++method_it) {
    if (!method_it.value().is_object()) {
      continue;  // `x-sovd-*` path-item extensions
    }
    const auto projected = sibling->find(method_it.key());
    if (projected == sibling->end() || !projected->is_object()) {
      // The path item is there but this verb is not, so this method has no
      // contract to inherit and would go out un-enriched. Refusing the whole
      // item is the same answer as a missing path item, for the same reason.
      return false;
    }
    auto & operation = method_it.value();

    const auto security = projected->find("security");
    if (security != projected->end()) {
      operation["security"] = *security;
    }

    // `lock_guarded()` declares the marker, the `X-Client-Id` parameter and the
    // 409 as one contract (`route_registry.cpp`). Inheriting the 409 alone
    // published a lock conflict with nothing saying who gets refused, which is
    // the split that call exists to prevent.
    //
    // Path parameters are dropped: at collection scope the sibling still
    // carries its own unbound id (`{data_id}`, `{operation_id}`), and a
    // concrete path has no placeholder for it. Header parameters describe the
    // request either way and come across.
    const auto lock_marker = projected->find("x-medkit-lock-guarded");
    if (lock_marker != projected->end()) {
      operation["x-medkit-lock-guarded"] = *lock_marker;
    }
    const auto parameters = projected->find("parameters");
    if (parameters != projected->end() && parameters->is_array()) {
      nlohmann::json inherited = nlohmann::json::array();
      for (const auto & parameter : *parameters) {
        const auto in = parameter.find("in");
        if (in != parameter.end() && *in == "path") {
          continue;
        }
        inherited.push_back(parameter);
      }
      if (!inherited.empty()) {
        operation["parameters"] = std::move(inherited);
      }
    }

    // A request body the item did not build - which is every one where the ROS
    // type is unknown, i.e. all of them today. The sibling's named
    // `$ref: DataWriteRequest` says the same thing better than an envelope
    // whose `data` member is `x-medkit-schema-unavailable`.
    const auto request_body = projected->find("requestBody");
    if (request_body != projected->end() && !operation.contains("requestBody")) {
      operation["requestBody"] = *request_body;
    }

    // Non-2xx always comes from the sibling, so the inline error bodies give way
    // to the shared components. 2xx is filled in only where the item built none
    // - and it builds none, because the gateway envelopes every response
    // (`DataValue`, `OperationDetail`) rather than returning the bare message.
    // A locally built 2xx would be a second, contradictory answer for one route.
    const auto responses = projected->find("responses");
    if (responses == projected->end() || !responses->is_object()) {
      continue;
    }
    for (auto resp_it = responses->begin(); resp_it != responses->end(); ++resp_it) {
      const bool success = !resp_it.key().empty() && resp_it.key()[0] == '2';
      if (success && operation["responses"].contains(resp_it.key())) {
        continue;
      }
      operation["responses"][resp_it.key()] = resp_it.value();
    }
  }
  return true;
}

void CapabilityGenerator::add_cache_derived_items(nlohmann::json & paths, const ResolvedPath & resolved,
                                                  const std::string & entity_path) const {
  const std::string collection_path = entity_path + "/" + resolved.resource_collection;
  const auto & cache = node_.get_thread_safe_cache();
  const PathBuilder path_builder(schema_builder_);
  const bool one_item = !resolved.resource_id.empty();

  // Where the projected operation for these items lives, which differs by
  // scope and is the whole reason this is computed here rather than guessed
  // inside the adopter:
  //
  //  * collection scope - `project()` bound the entity id and nothing else, so
  //    the item route is still templated: `/apps/x/data/{data_id}`.
  //  * specific-resource scope - it bound the item id too, so the projection
  //    already sits at the concrete key: `/apps/x/data/parameter_events`.
  //
  // At specific-resource scope the item is therefore written *at that same
  // key*, replacing a projection with the enriched version of itself.
  // The item parameter's name is read from the served routes, the same way
  // `generate_specific_resource` reads it, rather than spelled out here. Two
  // spellings of one fact meant renaming `{data_id}` in the registry would have
  // silently discarded every built item - and until this round no test would
  // have noticed.
  const auto [template_prefix, template_bindings] = entity_template(resolved, true);
  (void)template_bindings;
  const auto item_parameter =
      single_parameter_segment_under(served_paths(), template_prefix + "/" + resolved.resource_collection);
  const std::string sibling_key = one_item ? collection_path + "/" + resolved.resource_id
                                           : collection_path + "/{" + item_parameter.value_or("") + "}";
  if (!one_item && !item_parameter.has_value()) {
    return;  // No templated item route to inherit from; nothing here can be honest.
  }

  // A resource id reaches here with its leading `/` already stripped by
  // `PathResolver`, while a ROS topic name keeps one - `parameter_events`
  // against `/parameter_events`. Comparing them raw matched nothing, so at
  // specific-resource scope no data item was built at all and the payload
  // schema, the entire reason these items exist, was absent from exactly the
  // document a client asks for when it wants one topic.
  auto names_match = [](const std::string & lhs, const std::string & rhs) {
    const auto strip = [](const std::string & value) {
      return !value.empty() && value.front() == '/' ? value.substr(1) : value;
    };
    return strip(lhs) == strip(rhs);
  };

  // Enrich first, write only on success. A built item that could not take the
  // framework half from its sibling is strictly worse than the projection it
  // would replace: the projection is a truthful account of the route minus the
  // payload schema, the raw item is neither.
  auto emit = [&](const std::string & key, nlohmann::json item) {
    if (adopt_projected_framework(item, paths, sibling_key)) {
      paths[key] = std::move(item);
    }
  };

  // A data point or operation the cache does not know needs nothing here: the
  // projection already published the item route's own description at that key,
  // which is a truthful account of what the gateway will do with the request.
  // Only a resource the cache *does* know gains anything, and what it gains is
  // the ROS type.
  if (resolved.resource_collection == "data") {
    auto data = cache.get_entity_data(resolved.entity_id);
    for (const auto & topic : data.topics) {
      if (one_item && !names_match(topic.name, resolved.resource_id)) {
        continue;
      }
      emit(one_item ? sibling_key : collection_path + "/" + topic.name,
           path_builder.build_data_item(entity_path, topic));
    }
    return;
  }

  if (resolved.resource_collection != "operations") {
    return;
  }

  // Operations aggregate upward: an app owns its services directly, while a
  // component, area or function collects the ones its apps expose.
  auto ops = cache.get_app_operations(resolved.entity_id);
  if (ops.empty()) {
    switch (entity_type_from_keyword(resolved.entity_type)) {
      case SovdEntityType::COMPONENT:
        ops = cache.get_component_operations(resolved.entity_id);
        break;
      case SovdEntityType::AREA:
        ops = cache.get_area_operations(resolved.entity_id);
        break;
      case SovdEntityType::FUNCTION:
        ops = cache.get_function_operations(resolved.entity_id);
        break;
      case SovdEntityType::APP:
      case SovdEntityType::SERVER:
      case SovdEntityType::UNKNOWN:
      default:
        break;
    }
  }

  // The id documented for an item has to be the id the collection emits, or the
  // document describes a request the gateway then refuses. `operation_item_ids`
  // is the one place that rule lives, so this producer and the collection
  // cannot drift apart, and it also reports the ids no caller can reach.
  const auto item_ids = http::operation_item_ids(ops);
  std::size_t next_index = 0;

  // A specific-resource request may name its owning member, which is how the
  // collection addresses a short name that more than one member exposes.
  // Matching the bare half alone would document one member's operation under
  // the other member's id.
  const auto requested = http::parse_member_qualified_id(resolved.resource_id, ops.is_aggregated);
  const auto owned_by_requested = [&ops, &requested](const std::string & full_path) {
    if (!requested.has_member) {
      return true;
    }
    auto owner = ops.owner_by_path.find(full_path);
    return owner != ops.owner_by_path.end() && owner->second == requested.member_id;
  };

  // Consumed in the order `operation_item_ids` filled it: services, then
  // actions, matching the two loops below.
  const auto emit_operation = [&](const auto & op) {
    const std::string item_id = item_ids.ids[next_index++];
    if (one_item && !(names_match(op.name, requested.item_id) && owned_by_requested(op.full_path))) {
      return;
    }
    // An id that names two operations answers 400, and the second write would
    // silently replace the first, so no concrete item is published for it.
    if (item_ids.ambiguous(item_id)) {
      return;
    }
    emit(one_item ? sibling_key : collection_path + "/" + item_id, path_builder.build_operation_item(entity_path, op));
  };
  for (const auto & svc : ops.services) {
    emit_operation(svc);
  }
  for (const auto & action : ops.actions) {
    emit_operation(action);
  }
}

// -----------------------------------------------------------------------------
// Plugin route docs
// -----------------------------------------------------------------------------

nlohmann::json CapabilityGenerator::plugin_paths() const {
  if (!plugin_mgr_) {
    return nlohmann::json::object();
  }

  // A plugin's declared role is carried through unchanged here. Whether the
  // gateway is in a configuration that honours it is not a question about the
  // fold, so it is not answered in the fold - `generate_impl` rewrites every
  // per-operation requirement once, over the finished document, for whatever
  // producer wrote it.
  const auto * auth_manager = ctx_.auth_manager();
  nlohmann::json paths = nlohmann::json::object();
  for (const auto & desc : plugin_mgr_->collect_route_descriptions()) {
    auto paths_json = desc.to_json();  // CapabilityGenerator is friend
    for (auto & [key, item] : paths_json.items()) {
      for (auto & [method, operation] : item.items()) {
        if (!operation.is_object()) {
          continue;
        }
        // First-wins, like `RouteRegistry`'s `add_response_ref`: a status the
        // plugin described itself keeps the plugin's description. Everything
        // stamped below goes through it.
        auto add_response_ref = [&operation](const std::string & code, const std::string & component) {
          auto & responses = operation["responses"];
          if (!responses.contains(code)) {
            responses[code] = nlohmann::json{{"$ref", "#/components/responses/" + component}};
          }
        };
        // What separates a plugin operation from a gateway one, and the
        // reason it has to be visible in the document rather than inferred
        // from the path: a plugin route is mounted straight onto the HTTP
        // server by `PluginManager::register_routes`, not through the
        // `RouteRegistry`. Everything the registry attaches at mount time -
        // most of all the emitted-status recorder that
        // `test_openapi_error_coverage` reads - therefore never sees it.
        operation["x-medkit-plugin-served"] = true;

        // 416 is answered by cpp-httplib before routing, so it reaches a
        // plugin route for exactly the same reason it reaches a registry
        // one (see the `add_response_ref("416", ...)` comment in
        // `route_registry.cpp`). Stamped here rather than left to the
        // plugin: it is a fact about the HTTP server the plugin is mounted
        // on, not about the plugin.
        add_response_ref("416", "GenericError");

        // The auth middleware is pre-routing too, and the argument for 416 is
        // the argument for these: `PluginManager::register_routes` mounts a
        // plugin route on the same `httplib::Server` the middleware's
        // pre-routing handler runs on, so an anonymous caller meets the
        // middleware before the plugin's handler. Measured on the graph
        // provider's `GET /functions/{function_id}/x-medkit-graph` under
        // `require_auth_for: all`: no token 401, a viewer token 403, an admin
        // token 200, and the document used to declare none of it.
        //
        // Keyed on the same predicate the registry uses, so a plugin route and
        // a registry route on one gateway cannot disagree about whether the
        // middleware answers them. Under a policy that admits the operation
        // nothing is stamped, and `project_security_onto_enforcement` gives it
        // the empty requirement to match.
        if (auth_manager != nullptr &&
            auth_manager->requires_authentication(to_upper(method), std::string(API_BASE_PATH) + key)) {
          add_response_ref("401", "Unauthorized");
          add_response_ref("403", "Forbidden");
        }
      }
      paths[key] = item;
    }
  }
  return paths;
}

nlohmann::json CapabilityGenerator::generate_plugin_docs(const std::string & path) const {
  nlohmann::json matching_paths = nlohmann::json::object();

  auto all_paths = plugin_paths();
  for (auto & [key, value] : all_paths.items()) {
    if (key == path || key.find(path + "/") == 0) {
      matching_paths[key] = value;
    }
  }

  if (matching_paths.empty()) {
    return {};
  }

  OpenApiSpecBuilder builder;
  builder.info("ROS 2 Medkit Gateway - Plugin", kGatewayVersion)
      .sovd_version(kSovdVersion)
      .server(build_server_url(), "Gateway server")
      // Without the definition, the per-operation `security` a plugin
      // declares would name a scheme this sub-document does not have.
      .security_scheme("bearerAuth", bearer_scheme(), ctx_.auth_config().enabled)
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

// -----------------------------------------------------------------------------
// Cache helpers
// -----------------------------------------------------------------------------

void CapabilityGenerator::clear_cache_locked() const {
  spec_cache_.clear();
  cache_bytes_ = 0;
}

size_t CapabilityGenerator::cache_entry_count() const {
  std::shared_lock lock(cache_mutex_);
  return spec_cache_.size();
}

size_t CapabilityGenerator::cache_byte_size() const {
  std::shared_lock lock(cache_mutex_);
  return cache_bytes_;
}

std::string CapabilityGenerator::get_cache_key(const std::string & path) const {
  auto & cache = node_.get_thread_safe_cache();
  auto generation = cache.generation();
  {
    std::unique_lock lock(cache_mutex_);
    if (generation != cached_generation_) {
      clear_cache_locked();
      cached_generation_ = generation;
    }
  }
  return std::to_string(generation) + ":" + path;
}

std::optional<std::string> CapabilityGenerator::lookup_cache(const std::string & key) const {
  std::shared_lock lock(cache_mutex_);
  auto it = spec_cache_.find(key);
  if (it != spec_cache_.end()) {
    return it->second;
  }
  return std::nullopt;
}

void CapabilityGenerator::store_cache(const std::string & key, const std::string & document) const {
  const size_t entry_bytes = key.size() + document.size();
  // A document that cannot fit the budget by itself is served but not cached;
  // storing it would put the cache over its bound for as long as it is held.
  if (entry_bytes > bounds_.max_bytes) {
    return;
  }

  std::unique_lock lock(cache_mutex_);

  // Replacing an existing key: swap the accounting rather than adding to it.
  // Two threads can miss on the same key concurrently (TODO(#272)), so this
  // path is reachable and double-counting here would leak budget until the
  // next generation change.
  auto it = spec_cache_.find(key);
  if (it != spec_cache_.end()) {
    cache_bytes_ -= it->first.size() + it->second.size();
    it->second = document;
    cache_bytes_ += entry_bytes;
    return;
  }

  if (spec_cache_.size() >= bounds_.max_entries || cache_bytes_ + entry_bytes > bounds_.max_bytes) {
    clear_cache_locked();  // Simple eviction: clear all when full
  }
  spec_cache_.emplace(key, document);
  cache_bytes_ += entry_bytes;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
