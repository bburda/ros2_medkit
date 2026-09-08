// Copyright 2025-2026 bburda
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

#include "ros2_medkit_gateway/core/http/handlers/health_handlers.hpp"

#include <chrono>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/warning_codes.hpp"
#include "ros2_medkit_gateway/core/version.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

#include "../../openapi/route_registry.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

ErrorInfo make_internal_error(const char * where, const std::exception & e) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  ErrorInfo err;
  err.code = ERR_INTERNAL_ERROR;
  err.message = "Internal server error";
  err.http_status = 500;
  return err;
}

}  // namespace

namespace {

/// True when authentication is on and this request did not present a token
/// this gateway accepts.
///
/// Reachable two ways: under `require_auth_for: write`, where every GET is
/// open, and on a route an operator listed in `auth.public_routes`. In both an
/// anonymous caller reaches this handler, and the full body is more than the
/// probe asked for: the linking warnings name entities and ROS node FQNs, and
/// the entity cache reports how many apps, areas and components this gateway
/// sees. So an anonymous caller gets liveness and nothing else, flagged as cut
/// down, and an authenticated one gets the whole document.
bool is_anonymous(const HandlerContext & ctx, const http::TypedRequest & req) {
  if (!ctx.auth_config().enabled) {
    return false;  // Nothing is anonymous when nothing is authenticated.
  }
  auto * manager = ctx.auth_manager();
  if (manager == nullptr) {
    return true;  // Fail closed: cannot verify, so do not disclose.
  }
  auto header = req.header("Authorization");
  if (!header) {
    return true;
  }
  auto token = AuthMiddleware::extract_bearer_token(*header);
  if (!token) {
    return true;
  }
  return !manager->validate_token(*token).valid;
}

}  // namespace

http::Result<dto::Health> HealthHandlers::get_health(const http::TypedRequest & req) {
  try {
    dto::Health response;
    response.status = "healthy";
    response.timestamp = std::chrono::system_clock::now().time_since_epoch().count();

    // Liveness and nothing else for an anonymous caller. Returned before any
    // of the sections below are built, so a section added later is private by
    // default rather than public until someone remembers to think about it.
    // The flag is what keeps the empty `warnings` below from reading as
    // "nothing is wrong here" to a monitor that never presented a credential.
    if (is_anonymous(ctx_, req)) {
      response.x_medkit_reduced = true;
      return response;
    }

    // Operator-actionable warnings the gateway flags without taking itself
    // offline. Collected across every subsystem that can produce one, so the
    // array and its schema version are part of the /health contract whether or
    // not aggregation is configured.
    std::vector<dto::HealthWarning> warnings;

    // Add discovery info
    auto * dm = ctx_.node() ? ctx_.node()->get_discovery_manager() : nullptr;
    if (dm) {
      dto::HealthDiscovery discovery;
      discovery.mode = discovery_mode_to_string(dm->get_mode());
      discovery.strategy = dm->get_strategy_name();

      auto report = dm->get_merge_report();
      if (report) {
        discovery.pipeline = report->to_json();
      }

      auto linking = dm->get_linking_result();
      if (linking) {
        const auto manifest_config = dm->get_manifest_config();
        const auto policy = manifest_config.unmanifested_nodes;

        dto::HealthDiscoveryLinking linking_dto;
        linking_dto.linked_count = static_cast<int64_t>(linking->node_to_app.size());
        linking_dto.orphan_count = static_cast<int64_t>(linking->orphan_nodes.size());
        linking_dto.binding_conflicts = static_cast<int64_t>(linking->binding_conflicts);
        linking_dto.unmanifested_policy = discovery::ManifestConfig::policy_to_string(policy);
        if (!linking->warnings.empty()) {
          linking_dto.warnings = linking->warnings;
        }
        discovery.linking = linking_dto;

        // The 'error' policy reports, it does not stop the process: a
        // documentation-level setting must not be able to take a running
        // gateway offline, and a permanently unhealthy status would disable
        // every deployment gate that keys on a healthy -> unhealthy edge.
        if (linking->has_errors(policy)) {
          dto::HealthWarning warning;
          warning.code = WARN_UNMANIFESTED_NODES;
          warning.message =
              std::to_string(linking->orphan_nodes.size()) +
              " running ROS node(s) are not declared in the manifest while unmanifested_nodes is set to 'error' "
              "(listed in ros_node_fqns). The gateway keeps serving. Declare them in the manifest, or relax the "
              "policy to 'warn' or 'ignore'.";
          // The subjects of this warning are ROS nodes, so they go in
          // ros_node_fqns and entity_ids stays empty. Not a formality: the
          // orphan list is taken from the UNFILTERED runtime app list
          // (runtime_layer.cpp), before gap-fill, the namespace filters and
          // the policy filter, so some of these nodes have no SOVD entity at
          // all - and for those that do, the App id is derived from the bare
          // node name and only becomes namespace-qualified when some other
          // node collides with it (ros2_runtime_introspection.cpp), so it can
          // change between two polls while the FQN cannot.
          warning.ros_node_fqns = linking->orphan_nodes;
          warnings.push_back(std::move(warning));
        }
      }

      response.discovery = std::move(discovery);
    }

    // Surface subscription-executor and data-provider stats via x-medkit-*
    // vendor extensions. Atomic reads only; safe to serve from /health even
    // under load. External monitors (k8s liveness, systemd watchdog, Docker
    // HEALTHCHECK) can trigger on `x-medkit-subscription-executor.degraded`.
    if (ctx_.node()) {
      if (auto * tdp = ctx_.node()->get_topic_data_provider()) {
        auto x = tdp->x_medkit_stats();
        if (x.contains("x-medkit-data-provider")) {
          response.x_medkit_data_provider = x["x-medkit-data-provider"];
        }
        if (x.contains("x-medkit-subscription-executor")) {
          response.x_medkit_subscription_executor = x["x-medkit-subscription-executor"];
        }
      }

      // Surface entity cache live counts and capacity via x-medkit-entity-cache.
      // Enables external monitors and operators to verify the cache is sized
      // correctly (grew == true signals that entity_cache.capacity should be raised).
      const auto stats = ctx_.node()->get_thread_safe_cache().get_stats();
      response.x_medkit_entity_cache =
          json{{"capacity", stats.capacity}, {"areas", stats.area_count},         {"components", stats.component_count},
               {"apps", stats.app_count},    {"functions", stats.function_count}, {"generation", stats.generation},
               {"grew", stats.grew}};
    }

    // How much of the SSE client cap is in use. The cap is small - two by
    // default - and on an aggregating deployment a peer's slots are consumed
    // by the aggregator's fault-stream relay rather than by anyone an operator
    // can see. A 503 on /faults/stream is otherwise unexplainable from outside.
    if (sse_tracker_) {
      response.x_medkit_sse =
          json{{"connected_clients", sse_tracker_->connected_clients()}, {"max_clients", sse_tracker_->max_clients()}};
    }

    // Add peer status when aggregation is active
    if (auto * agg = ctx_.aggregation_manager()) {
      response.peers = agg->get_peer_status();

      // Aggregation warnings are only collectable when there is an aggregation
      // manager; the array itself is not conditional on one.
      for (const auto & w : agg->get_leaf_warnings()) {
        std::string peers_list;
        for (size_t i = 0; i < w.peer_names.size(); ++i) {
          if (i > 0u) {
            peers_list += ", ";
          }
          peers_list += w.peer_names[i];
        }
        dto::HealthWarning warning;
        warning.code = WARN_LEAF_ID_COLLISION;
        warning.message = "Component '" + w.entity_id + "' is announced by multiple peers (" + peers_list +
                          "); routing falls back to last-writer-wins which is non-deterministic. Resolve by "
                          "renaming the Component on one side or by modelling it as a hierarchical parent "
                          "(declare a child Component with parentComponentId='" +
                          w.entity_id + "' on the owning peer).";
        // A Component id, so it belongs in entity_ids and this code names no
        // ROS nodes - but the id came verbatim from a remote peer and nothing
        // on that path validated it. entity_ids promises every element is
        // addressable, so an id a client could not put in a URL is withheld
        // rather than published: the message above still names the component,
        // so only the machine-readable link is lost, and that link would not
        // have worked.
        if (auto valid = validate_entity_id(w.entity_id); valid) {
          warning.entity_ids = {w.entity_id};
        } else {
          RCLCPP_WARN(HandlerContext::logger(),
                      "Peer(s) [%s] announced Component id '%s', which is not a valid entity id (%s); omitting it "
                      "from the '%s' warning's entity_ids",
                      peers_list.c_str(), w.entity_id.c_str(), valid.error().c_str(), WARN_LEAF_ID_COLLISION);
        }
        warning.peer_names = w.peer_names;
        warnings.push_back(std::move(warning));
      }
    }

    // Contract version for the warnings array. Increment whenever a code is
    // added or the shape of a warning object changes so typed clients (MCP,
    // Web UI, Foxglove) can feature-detect. Emitted unconditionally: keying it
    // on capabilities.aggregation used to hide the whole warnings channel from
    // every gateway that does not aggregate.
    // Keep in sync with docs/api/warning_codes.rst "Schema versioning".
    response.warning_schema_version = static_cast<int64_t>(kWarningSchemaVersion);
    response.warnings = std::move(warnings);

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_health", e));
  }
}

http::Result<dto::RootOverview> HealthHandlers::get_root(const http::TypedRequest & req) {
  (void)req;  // Unused parameter
  try {
    // Generate endpoint list from route registry (single source of truth)
    std::vector<std::string> endpoints;
    if (route_registry_) {
      auto ep_list = route_registry_->to_endpoint_list(API_BASE_PATH);
      for (auto & ep : ep_list) {
        endpoints.push_back(std::move(ep));
      }
    }

    // Plugin routes are mounted straight onto the HTTP server by
    // `PluginManager::register_routes`, never through the registry, so the loop
    // above cannot see them. Without this they were the one thing `/docs`
    // documented that the root did not advertise - the opposite direction to
    // the `hidden()` routes, which the root advertises and `/docs` omits.
    //
    // Both directions are correct once the two lists are read for what they
    // are: this one says what is *mounted* (see the Swagger UI note below), and
    // the document says what is *documented*. A `hidden()` route is mounted and
    // answers 405; a plugin route is mounted and is documented. Pinned by
    // `test_openapi_contract.test.py::test_the_root_list_and_the_document_agree`.
    if (ctx_.node() && ctx_.node()->get_plugin_manager()) {
      const std::string api_base_path{API_BASE_PATH};
      for (const auto & desc : ctx_.node()->get_plugin_manager()->collect_route_descriptions()) {
        for (const auto & [method, path] : desc.endpoints()) {
          // Built in place rather than by `+` chaining, matching
          // `RouteRegistry::to_endpoint_list` above: the chain allocates a
          // throwaway temporary per operator.
          std::string endpoint;
          endpoint.reserve(method.size() + api_base_path.size() + path.size() + 1);
          endpoint += method;
          endpoint += ' ';
          endpoint += api_base_path;
          endpoint += path;
          endpoints.push_back(std::move(endpoint));
        }
      }
    }

#ifdef ENABLE_SWAGGER_UI
    // The two `/docs` routes are in the registry, so `to_endpoint_list` above
    // already lists them; a hand-written entry here would list each twice, and
    // under a second spelling of the path parameter at that. Swagger UI is
    // still mounted straight onto the server, so it is still added by hand -
    // and it is the only endpoint `docs.enabled` now hides from this list. The
    // `/docs` routes stay listed when the capability is off, because they stay
    // mounted and answer 501: this list says what is mounted, not what is on.
    bool docs_enabled = true;
    if (ctx_.node()) {
      try {
        docs_enabled = ctx_.node()->get_parameter("docs.enabled").as_bool();
      } catch (...) {
        // Parameter may not be declared - default to true
      }
    }
    if (docs_enabled) {
      endpoints.push_back("GET " + std::string(API_BASE_PATH) + "/swagger-ui");
    }
#endif

    const auto & auth_config = ctx_.auth_config();
    const auto & tls_config = ctx_.tls_config();

    dto::RootCapabilities capabilities;
    capabilities.discovery = true;
    capabilities.data_access = true;
    capabilities.operations = true;
    capabilities.async_actions = true;
    capabilities.configurations = true;
    capabilities.faults = true;
    capabilities.logs = true;
    capabilities.bulk_data = true;
    capabilities.cyclic_subscriptions = true;
    capabilities.locking = ctx_.node() && ctx_.node()->get_lock_manager() != nullptr;
    capabilities.triggers = ctx_.node() && ctx_.node()->get_trigger_manager() != nullptr;
    capabilities.updates = ctx_.node() && ctx_.node()->get_update_manager() != nullptr;
    capabilities.authentication = auth_config.enabled;
    capabilities.tls = tls_config.enabled;
    capabilities.scripts =
        ctx_.node() && ctx_.node()->get_script_manager() != nullptr && ctx_.node()->get_script_manager()->has_backend();
    capabilities.aggregation = ctx_.aggregation_manager() != nullptr;
    capabilities.vendor_extensions =
        ctx_.node() && ctx_.node()->get_plugin_manager() && ctx_.node()->get_plugin_manager()->has_plugins();

    dto::RootOverview response;
    response.name = "ROS 2 Medkit Gateway";
    response.version = kGatewayVersion;
    response.api_base = API_BASE_PATH;
    response.endpoints = std::move(endpoints);
    response.capabilities = capabilities;

    // Add auth info if enabled
    if (auth_config.enabled) {
      dto::RootAuth auth;
      auth.enabled = true;
      auth.algorithm = algorithm_to_string(auth_config.jwt_algorithm);
      auth.require_auth_for = auth_config.require_auth_for == AuthRequirement::NONE    ? "none"
                              : auth_config.require_auth_for == AuthRequirement::WRITE ? "write"
                                                                                       : "all";
      response.auth = std::move(auth);
    }

    // Add TLS info if enabled
    if (tls_config.enabled) {
      dto::RootTls tls;
      tls.enabled = true;
      tls.min_version = tls_config.min_version;
      // TODO(future): Add mutual_tls when implemented
      response.tls = std::move(tls);
    }

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_root", e));
  }
}

http::Result<dto::VersionInfo> HealthHandlers::get_version_info(const http::TypedRequest & req) {
  try {
    // SOVD 7.4.1 compliant response format
    dto::VersionInfoVendor vendor;
    vendor.version = kGatewayVersion;
    vendor.name = "ros2_medkit";

    dto::VersionInfoEntry entry;
    entry.version = kSovdVersion;    // SOVD standard version
    entry.base_uri = API_BASE_PATH;  // Version-specific base URI
    entry.vendor_info = std::move(vendor);

    dto::VersionInfo response;
    response.items.push_back(std::move(entry));

    // Fan-out aggregation: merge items from peers and collect x-medkit metadata.
    // merge_peer_items operates on the raw cpp-httplib request; use the
    // framework-internal escape hatch deliberately (handlers do not own that
    // helper's signature yet - typed fan-out for non-collection endpoints lands
    // in a later commit of this PR).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
    json response_json = dto::JsonWriter<dto::VersionInfo>::write(response);
    dto::XMedkitVersionInfo ext_dto;
    json ext_json = json::object();
    merge_peer_items(ctx_.aggregation_manager(), raw_req, response_json, ext_json);
    if (!ext_json.empty()) {
      if (ext_json.contains("partial")) {
        ext_dto.partial = ext_json["partial"].get<bool>();
      }
      if (ext_json.contains("failed_peers")) {
        ext_dto.failed_peers = ext_json["failed_peers"].get<std::vector<std::string>>();
      }
      if (ext_json.contains("peer_failures") && ext_json["peer_failures"].is_array()) {
        std::vector<dto::PeerFailure> failures;
        for (const auto & failure_entry : ext_json["peer_failures"]) {
          failures.push_back(dto::PeerFailure{failure_entry.value("peer", ""), failure_entry.value("reason", "")});
        }
        ext_dto.peer_failures = std::move(failures);
      }
    }

    // Re-parse merged items back into the DTO and attach x-medkit if present
    if (response_json.contains("items") && response_json["items"].is_array()) {
      response.items.clear();
      for (const auto & item : response_json["items"]) {
        auto parsed = dto::JsonReader<dto::VersionInfoEntry>::read(item);
        if (parsed.has_value()) {
          response.items.push_back(std::move(*parsed));
        }
      }
    }
    if (ext_dto.partial.has_value() || ext_dto.failed_peers.has_value() || ext_dto.peer_failures.has_value()) {
      response.x_medkit = std::move(ext_dto);
    }

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_version_info", e));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
