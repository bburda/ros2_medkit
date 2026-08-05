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

#include "ros2_medkit_gateway/core/http/handlers/config_handlers.hpp"

#include <algorithm>
#include <future>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/parameter_error_classification.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

// =============================================================================
// Constants
// =============================================================================

/// Maximum length for aggregated parameter IDs.
///
/// Aggregated parameter IDs use the format `app_id:param_name`, so the max
/// length is:
///   - app_id    : up to 256 characters (entity ID limit)
///   - separator : 1 character (`:`)
///   - param_name: up to 256 characters (ROS 2 parameter name limit)
constexpr size_t kMaxAggregatedParamIdLength = 512;

// =============================================================================
// Helper structures and free functions
// =============================================================================

/// Read the positional entity-id capture group from the typed request. cpp-
/// httplib only invokes the route when the regex matches, so the `nullopt`
/// branch is effectively unreachable; we surface it as 400 invalid-request to
/// match the legacy handlers' explicit `req.matches.size() < 2` guard.
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the second positional capture group (the parameter id) with the same
/// "missing capture is treated as 400 invalid-request" semantics as
/// `read_entity_id`.
tl::expected<std::string, ErrorInfo> read_param_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Parsed parameter ID with optional app prefix.
struct ParsedParamId {
  std::string app_id;      ///< Target app ID (empty if not prefixed)
  std::string param_name;  ///< Parameter name
  bool has_prefix{false};  ///< Whether the ID had an app prefix
};

/// Split `param_id` into member and parameter halves at its first colon, when
/// the half before that colon names a member of this entity.
///
/// An entity id is restricted to alphanumerics, underscore and hyphen, so it
/// can never contain a colon and the first one is the only candidate separator.
/// Whether it IS a separator is decided by the entity's member set, because
/// that is the set the member half draws its meaning from. A count of the nodes
/// this gateway resolves for the entity cannot decide it: a member another
/// gateway runs reports no ROS binding here and so contributes no node, which
/// makes the count a measure of how much of the entity is local and not of how
/// its ids are formed. An entity whose members all belong to peers resolves no
/// node at all, and one that resolves a single node can still have peer-owned
/// members alongside it.
///
/// A prefix that names no member is part of the parameter name. That is what
/// keeps a parameter whose own name contains a colon addressable, and it is why
/// the entity's own id is not read as a member half: it separates nothing, and
/// accepting it would give a parameter a second id nothing offers.
///
/// A member half followed by an empty parameter name addresses no parameter.
/// Carried further it would name the member's configurations COLLECTION - the
/// route one path segment shorter - and hand a list back to a caller that asked
/// for one value, so it is left unsplit and misses as an ordinary name.
ParsedParamId address_parameter(const ThreadSafeEntityCache & cache, const EntityInfo & entity,
                                const std::string & param_id) {
  ParsedParamId result;
  result.param_name = param_id;

  const auto colon_pos = param_id.find(':');
  if (colon_pos == std::string::npos || colon_pos == 0 || colon_pos + 1 == param_id.size()) {
    return result;
  }

  std::string candidate = param_id.substr(0, colon_pos);
  if (candidate == entity.id) {
    return result;
  }

  const auto members = cache.get_members(entity.sovd_type(), entity.id);
  if (std::find(members.begin(), members.end(), candidate) == members.end()) {
    return result;
  }

  result.app_id = std::move(candidate);
  result.param_name = param_id.substr(colon_pos + 1);
  result.has_prefix = true;
  return result;
}

/// Find node info for a specific app ID in aggregated configurations.
const NodeConfigInfo * find_node_for_app(const std::vector<NodeConfigInfo> & nodes, const std::string & app_id) {
  for (const auto & node : nodes) {
    if (node.app_id == app_id) {
      return &node;
    }
  }
  return nullptr;
}

/// Settle where a member-qualified configuration id is served, and hand the
/// request over when that is another gateway. Reads, writes and resets share it
/// because a parameter has exactly one owning node whatever the method is.
///
/// A parameter is owned by a ROS node, and the node behind a member another
/// gateway runs is on a graph this one cannot see: the local walk records no
/// FQN for it, so every method resolved here refuses an id that names it. The
/// member, however, is an entity with an owner, and on that owner the parameter
/// is addressed by its bare name under the member's own App or Component route -
/// nothing over there is aggregating, so nothing over there qualifies the id.
///
/// Returns the answer the handler must return - including the sentinel that says
/// the owning peer has already committed the wire - or nullopt when this gateway
/// serves the parameter itself, which is every unprefixed id and every member
/// this gateway owns. An unprefixed id carries no member half, and an empty
/// member id is the case `dispatch_to_member` answers with kServeLocally.
std::optional<ErrorInfo> dispatch_configuration(const HandlerContext & ctx, const http::TypedRequest & req,
                                                const std::string & entity_id, const std::string & param_id,
                                                const ParsedParamId & parsed) {
  auto dispatch = ctx.dispatch_to_member(req, parsed.app_id, "configurations/" + parsed.param_name,
                                         json{{"entity_id", entity_id}, {"id", param_id}});
  if (!dispatch) {
    return dispatch.error();
  }
  if (*dispatch == MemberDispatch::kForwarded) {
    return HandlerContext::forwarded_sentinel_error();
  }
  return std::nullopt;
}

/// Build a typed `ErrorInfo` for a failed parameter operation. Mirrors the
/// legacy `send_parameter_error` helper's wire shape (params: details +
/// entity_id + id, message: "Failed to <op> parameter").
ErrorInfo make_parameter_error(const ParameterResult & result, const std::string & operation_name,
                               const std::string & entity_id, const std::string & param_id) {
  auto err = classify_parameter_error(result);
  std::string message = "Failed to " + operation_name + " parameter";
  return make_error(err.status_code, err.error_code, std::move(message),
                    json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
}

/// Build a typed `ErrorInfo` for the GET path when a parameter is missing on
/// a specific node. Legacy uses message "Parameter not found" for 404 and
/// "Failed to get parameter" otherwise; we preserve that branching here so
/// the wire shape is byte-identical.
ErrorInfo make_get_parameter_error(const ParameterResult & result, const std::string & entity_id,
                                   const std::string & param_id) {
  auto err = classify_parameter_error(result);
  std::string message = err.status_code == 404 ? "Parameter not found" : "Failed to get parameter";
  return make_error(err.status_code, err.error_code, std::move(message),
                    json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
}

/// Build the `ConfigurationReadValue` DTO and accompanying `ConfigValueXMedkit`
/// payload for a successful get/set on a specific node. Shared between the
/// happy paths of `get_configuration` and `set_configuration`.
dto::ConfigurationReadValue make_read_value(const std::string & entity_id, const std::string & node_fqn,
                                            const std::string & param_id, const std::optional<std::string> & source_app,
                                            const json & param_data) {
  dto::ConfigValueXMedkit xm;
  xm.ros2 = dto::XMedkitRos2{};
  xm.ros2->node = node_fqn;
  xm.entity_id = entity_id;
  xm.source = "runtime";
  xm.parameter = param_data;
  if (source_app.has_value()) {
    xm.source_app = *source_app;
  }

  dto::ConfigurationReadValue resp;
  resp.id = param_id;
  resp.data = param_data.contains("value") ? param_data["value"] : param_data;
  resp.x_medkit = std::move(xm);
  return resp;
}

/// Build a single `ConfigurationMetaData` item for the list response.
dto::ConfigurationMetaData make_meta_item(const json & param, const NodeConfigInfo & node_info, bool is_aggregated) {
  dto::ConfigurationMetaData meta;
  std::string param_name = param.value("name", "");

  // Unique ID for aggregated configs is `app_id:param_name` so callers can
  // disambiguate same-named parameters across nodes hosted by the same SOVD
  // entity. Non-aggregated entities just use the bare parameter name.
  meta.id = is_aggregated ? (node_info.app_id + ":" + param_name) : param_name;
  meta.name = param_name;
  meta.type = "parameter";

  if (is_aggregated) {
    dto::ConfigXMedkitItem item_xm;
    item_xm.source = node_info.app_id;
    meta.x_medkit = std::move(item_xm);
  }
  return meta;
}

/// Single node-query result used by the parallel list-parameters fan-out.
struct NodeQueryResult {
  NodeConfigInfo node_info;
  ParameterResult result;
};

/// Run `ConfigurationManager::list_parameters` in parallel against every
/// node in `agg_configs.nodes` and return the per-node results. The order
/// matches `agg_configs.nodes` to keep deterministic per-test output.
std::vector<NodeQueryResult> query_nodes_in_parallel(ConfigurationManager * config_mgr,
                                                     const AggregatedConfigurations & agg_configs) {
  std::vector<std::future<NodeQueryResult>> futures;
  futures.reserve(agg_configs.nodes.size());

  for (const auto & node_info : agg_configs.nodes) {
    futures.push_back(std::async(std::launch::async, [config_mgr, node_info]() {
      NodeQueryResult query_result;
      query_result.node_info = node_info;
      query_result.result = config_mgr->list_parameters(node_info.node_fqn);
      return query_result;
    }));
  }

  std::vector<NodeQueryResult> results;
  results.reserve(futures.size());
  for (auto & future : futures) {
    results.push_back(future.get());
  }
  return results;
}

/// Fold the partial/failed_peers/peer_dropped_items observability fields
/// from a `FanOutResult<ConfigurationMetaData>` into the typed
/// `ConfigListXMedkit`. Mirrors the legacy behaviour where
/// `merge_peer_items` injected `partial` and `failed_peers` keys into the
/// x-medkit JSON; here the typed equivalent is a direct field assignment
/// plus dropped-item enrichment from the typed reader.
void apply_fan_out_observability(dto::ConfigListXMedkit & xm,
                                 const FanOutResult<dto::ConfigurationMetaData> & fan_out) {
  if (fan_out.partial) {
    xm.partial = true;
    xm.failed_peers = fan_out.failed_peers;
    xm.peer_failures = fan_out.peer_failures;
  }
  if (!fan_out.dropped_items.empty()) {
    xm.peer_dropped_items = fan_out.dropped_items;
  }
}

/// Severity rank for order-independent folding: node-unavailable (503) is the
/// most actionable verdict (retry), then other server errors, then non-404
/// client errors; NOT_FOUND ranks lowest so it can never mask a real failure.
int severity_rank(int status_code) {
  if (status_code == 404) {
    return 0;
  }
  if (status_code == 503) {
    return 3;
  }
  if (status_code >= 500) {
    return 2;
  }
  return 1;  // non-404 4xx
}

/// Folds per-node parameter lookup failures so the error surfaced when no
/// node succeeds does not depend on node iteration order. Failures are
/// ranked by `severity_rank`; the highest rank folded so far is kept, and
/// within a rank the first failure folded wins.
class ParameterErrorAccumulator {
 public:
  /// Fold one failed ParameterResult.
  void add(const ParameterResult & result) {
    auto err = classify_parameter_error(result);
    const int rank = severity_rank(err.status_code);
    if (rank > 0) {
      all_not_found_ = false;
    }
    // Strictly-greater keeps the first failure folded at the winning rank, so
    // the surfaced status never depends on node iteration order.
    if (rank > worst_rank_) {
      worst_ = result;
      classification_ = std::move(err);
      worst_rank_ = rank;
    }
  }

  /// True when every folded failure classified as 404.
  bool all_not_found() const {
    return all_not_found_;
  }

  /// The failure to surface: the highest-severity failure folded so far.
  const ParameterResult & worst() const {
    return worst_;
  }

  /// Classification of worst().
  const ParameterErrorClassification & classification() const {
    return classification_;
  }

 private:
  ParameterResult worst_;
  ParameterErrorClassification classification_;
  int worst_rank_ = -1;
  bool all_not_found_ = true;
};

}  // namespace

// ===========================================================================
// GET /{entity-path}/configurations
// ===========================================================================

http::Result<dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>>
ConfigHandlers::list_configurations(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit> response;
  dto::ConfigListXMedkit xm;
  xm.entity_id = entity_id;
  xm.source = "runtime";
  xm.aggregation_level = agg_configs.aggregation_level;
  xm.is_aggregated = agg_configs.is_aggregated;

  json all_parameters = json::array();
  std::vector<std::string> queried_nodes;

  if (!agg_configs.nodes.empty()) {
    auto * config_mgr = ctx_.node()->get_configuration_manager();
    const auto node_results = query_nodes_in_parallel(config_mgr, agg_configs);

    bool any_success = false;
    ParameterErrorAccumulator errors;            // worst failure to surface if no node answers
    std::vector<std::string> unavailable_nodes;  // 503-class: node down/unresponsive
    std::vector<std::string> failed_nodes;       // other per-node failures

    for (const auto & qr : node_results) {
      const auto & node_info = qr.node_info;
      const auto & result = qr.result;

      if (result.success) {
        any_success = true;
        queried_nodes.push_back(node_info.node_fqn);

        if (result.data.is_array()) {
          for (const auto & param : result.data) {
            response.items.push_back(make_meta_item(param, node_info, agg_configs.is_aggregated));

            // Also track the full parameter details (with x-medkit source/node)
            // inside the x-medkit `parameters` array. Free-form JSON because the
            // ConfigurationManager produces vendor-specific raw output here.
            json param_with_source = param;
            dto::ConfigXMedkitItem param_xm;
            param_xm.source = node_info.app_id;
            param_xm.node = node_info.node_fqn;
            param_with_source["x-medkit"] = dto::JsonWriter<dto::ConfigXMedkitItem>::write(param_xm);
            all_parameters.push_back(std::move(param_with_source));
          }
        }
      } else {
        errors.add(result);
        // A per-node failure never discards the nodes that answered. A 503-class
        // failure (node down/unresponsive) is an availability gap named in
        // `unavailable_nodes`; any other failure is named in `failed_nodes`.
        // Both keep the answering nodes' parameters and flag the list partial.
        if (classify_parameter_error(result).status_code == 503) {
          unavailable_nodes.push_back(node_info.node_fqn);
        } else {
          failed_nodes.push_back(node_info.node_fqn);
        }
      }
    }

    if (!any_success) {
      // No backing node answered. Surface the worst failure order-independently
      // (`severity_rank`: 503 outranks other 5xx), so a mixed down + internal
      // outage returns 503 to match GET /{entity}/configurations/{param}; name
      // every failed node so the outage is diagnosable.
      const auto & cls = errors.classification();
      json details{{"details", errors.worst().error_message}, {"entity_id", entity_id}};
      if (!unavailable_nodes.empty()) {
        details["unavailable_nodes"] = unavailable_nodes;
      }
      if (!failed_nodes.empty()) {
        details["failed_nodes"] = failed_nodes;
      }
      return tl::unexpected(
          make_error(cls.status_code, cls.error_code, "Failed to list parameters from any node", std::move(details)));
    }

    xm.parameters = all_parameters;
    if (!agg_configs.source_ids.empty()) {
      xm.source_ids = agg_configs.source_ids;
    }
    if (!queried_nodes.empty()) {
      xm.queried_nodes = queried_nodes;
    }
    // Some (but not all) backing nodes failed. Mirror the peer fan-out honesty
    // convention (partial + failed_peers) for local nodes: keep the answering
    // nodes' parameters, flag the response partial, and name what failed -
    // 503-class nodes in `unavailable_nodes`, other failures in `failed_nodes` -
    // instead of silently returning a shrunken 200. Keeps the list surface
    // consistent with GET /{entity}/configurations/{param}, which returns the
    // same 503 for a backing-node outage.
    if (!unavailable_nodes.empty() || !failed_nodes.empty()) {
      xm.partial = true;
    }
    if (!unavailable_nodes.empty()) {
      xm.unavailable_nodes = std::move(unavailable_nodes);
    }
    if (!failed_nodes.empty()) {
      xm.failed_nodes = std::move(failed_nodes);
    }
  }

  // Typed fan-out for collection endpoints. Replaces the legacy raw-JSON
  // `merge_peer_items` mutator: peer items come back as parsed
  // `dto::ConfigurationMetaData` values via `JsonReader<ConfigurationMetaData>`,
  // malformed peer items are surfaced as `dropped_items` (folded into
  // xm.peer_dropped_items below), and partial/failed_peers go into the typed
  // xm fields. `fan_out_collection` still operates on the raw cpp-httplib
  // request (path + headers) - the typed-request raw escape hatch is used
  // deliberately here; a later commit will accept the typed request directly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto fan_out = fan_out_collection<dto::ConfigurationMetaData>(ctx_.aggregation_manager(), raw_req);
  for (auto & item : fan_out.items) {
    response.items.push_back(std::move(item));
  }
  apply_fan_out_observability(xm, fan_out);

  response.x_medkit = std::move(xm);
  return response;
}

// ===========================================================================
// GET /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<dto::ConfigurationReadValue> ConfigHandlers::get_configuration(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  // Bounded before anything is parsed: the id may carry a member half, so the
  // cap covers an entity id, the separator and a parameter name.
  if (param_id.empty() || param_id.length() > kMaxAggregatedParamIdLength) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                     json{{"details", "Parameter ID is empty or too long"}}));
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);
  auto parsed = address_parameter(cache, entity, param_id);

  // Ownership is settled before the local node set is consulted, because the
  // node set describes this gateway's own graph: an entity all of whose members
  // belong to peers resolves none, and refusing on that count would refuse the
  // one id the entity can actually answer.
  if (auto answered = dispatch_configuration(ctx_, req, entity_id, param_id, parsed)) {
    return tl::unexpected(*answered);
  }

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();

  // The id named a member this gateway owns; read the parameter from that
  // member's own node rather than probing every node the entity has.
  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->get_parameter(node_info->node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_get_parameter_error(result, entity_id, param_id));
    }
    return make_read_value(entity_id, node_info->node_fqn, param_id, parsed.app_id, result.data);
  }

  // For non-aggregated entities (or aggregated entities without a prefix) we
  // probe every backing node for the parameter and return the first success.
  // Failures fold through ParameterErrorAccumulator, which ranks them by
  // severity (503 > other 5xx > non-404 4xx > 404), so the surfaced error is
  // the same regardless of node iteration order.
  ParameterErrorAccumulator errors;

  for (const auto & node_info : agg_configs.nodes) {
    auto result = config_mgr->get_parameter(node_info.node_fqn, parsed.param_name);
    if (result.success) {
      std::optional<std::string> source_app;
      if (agg_configs.is_aggregated) {
        source_app = node_info.app_id;
      }
      return make_read_value(entity_id, node_info.node_fqn, parsed.param_name, source_app, result.data);
    }

    errors.add(result);
  }

  if (errors.all_not_found()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Parameter not found",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  // Some node reported a non-404 (e.g. unavailable); surface that.
  const auto & err = errors.classification();
  return tl::unexpected(
      make_error(err.status_code, err.error_code, "Failed to get parameter from any node",
                 json{{"details", errors.worst().error_message}, {"entity_id", entity_id}, {"id", param_id}}));
}

// ===========================================================================
// PUT /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<dto::ConfigurationReadValue> ConfigHandlers::set_configuration(const http::TypedRequest & req,
                                                                            dto::ConfigurationWriteRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  // Validate entity_id format first so that pathological IDs get a 400
  // before we touch the cache.
  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                     json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  if (param_id.empty() || param_id.length() > kMaxAggregatedParamIdLength) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                     json{{"details", "Parameter ID is empty or too long"}}));
  }

  // Both `data` and `value` are optional at the DTO level so legacy clients
  // sending `{"value": ...}` are not rejected by the typed body parser. We
  // enforce "at least one present" here and prefer `data` over `value` to
  // match the legacy handler's behaviour bit-for-bit.
  json config_value;
  if (body.data.has_value()) {
    config_value = *body.data;
  } else if (body.value.has_value()) {
    config_value = *body.value;
  } else {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Request body must contain a 'data' field"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);
  auto parsed = address_parameter(cache, entity, param_id);

  // Helper: turn a successful set into the typed response. The wire shape
  // matches the legacy handler exactly - the response id is the original
  // request param_id (which includes the app_id: prefix if any), so callers
  // who issued `PUT .../foo:bar` see `"id": "foo:bar"` come back.
  auto on_success = [&](const std::string & node_fqn, const std::optional<std::string> & source_app,
                        const json & param_data) -> dto::ConfigurationReadValue {
    return make_read_value(entity_id, node_fqn, param_id, source_app, param_data);
  };

  // Ownership is settled before the local node set is consulted, for the same
  // reason as on the read: the node set counts this gateway's own nodes, and a
  // member another gateway runs contributes none of them.
  if (auto answered = dispatch_configuration(ctx_, req, entity_id, param_id, parsed)) {
    return tl::unexpected(*answered);
  }

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();

  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->set_parameter(node_info->node_fqn, parsed.param_name, config_value);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "set", entity_id, param_id));
    }
    return on_success(node_info->node_fqn, parsed.app_id, result.data);
  }

  // Non-aggregated entity: target the single backing node.
  if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
    const auto & node_info = agg_configs.nodes[0];
    auto result = config_mgr->set_parameter(node_info.node_fqn, parsed.param_name, config_value);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "set", entity_id, param_id));
    }
    // Non-aggregated case never carries source_app; keep the wire shape
    // legacy-compatible by passing nullopt.
    return on_success(node_info.node_fqn, std::nullopt, result.data);
  }

  // Aggregated entity but no prefix: ambiguous, reject.
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Aggregated configuration requires app_id prefix",
                                   json{{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                        {"entity_id", entity_id},
                                        {"id", param_id}}));
}

// ===========================================================================
// DELETE /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<http::NoContent> ConfigHandlers::delete_configuration(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  // Same bound, and in the same position, as `set_configuration`: validate the
  // caller's own inputs before consulting the lock, because a malformed id is
  // the client's bug rather than a contended resource. `read_param_id` checks
  // only that the capture is present, so without this DELETE was the one verb
  // in the family accepting an unbounded parameter id - and the OpenAPI
  // document publishes `maxLength: 512` on every route carrying `{config_id}`,
  // this one included.
  if (param_id.empty() || param_id.length() > kMaxAggregatedParamIdLength) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                     json{{"details", "Parameter ID is empty or too long"}}));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);
  auto parsed = address_parameter(cache, entity, param_id);

  // Ownership is settled before the local node set is consulted, for the same
  // reason as on the read: the node set counts this gateway's own nodes, and a
  // member another gateway runs contributes none of them.
  if (auto answered = dispatch_configuration(ctx_, req, entity_id, param_id, parsed)) {
    return tl::unexpected(*answered);
  }

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();

  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->reset_parameter(node_info->node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "reset", entity_id, param_id));
    }
    return http::NoContent{};
  }

  if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
    const auto & node_info = agg_configs.nodes[0];
    auto result = config_mgr->reset_parameter(node_info.node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "reset", entity_id, param_id));
    }
    return http::NoContent{};
  }

  // Aggregated configs without a prefix - we cannot resolve a single target node.
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Aggregated configuration requires app_id prefix",
                                   json{{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                        {"entity_id", entity_id},
                                        {"id", param_id}}));
}

// ===========================================================================
// DELETE /{entity-path}/configurations - reset all (204 success / 207 partial)
// ===========================================================================

http::Result<std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>>
ConfigHandlers::delete_all_configurations(const http::TypedRequest & req) {
  using ResultVariant = std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>;

  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  auto * config_mgr = ctx_.node()->get_configuration_manager();
  bool all_reset = true;
  dto::ConfigurationDeleteMultiStatus multi_status;
  multi_status.entity_id = entity_id;

  for (const auto & node_info : agg_configs.nodes) {
    auto result = config_mgr->reset_all_parameters(node_info.node_fqn);

    dto::ConfigurationDeleteResultItem entry;
    entry.node = node_info.node_fqn;
    entry.app_id = node_info.app_id;
    entry.success = result.success;
    if (!result.success) {
      all_reset = false;
      entry.error = result.error_message;
    }
    // Carried on the failing entry as well as the succeeding one: a partial
    // reset names the parameters it could not restore, and dropping that leaves
    // the caller a verdict with nothing to act on.
    if (result.data.is_object() || result.data.is_array()) {
      entry.details = result.data;
    }
    multi_status.results.push_back(std::move(entry));
  }

  // Members whose node another gateway runs. This route resets by calling the
  // parameter services on this gateway's own ROS graph, and there is no such
  // service for them here - so they are not reset, however the local half went.
  // They are named rather than omitted because the entity's configurations
  // collection LISTS their parameters: a caller who resets the entity and is
  // answered 204 has been told the parameters it can see were restored, and for
  // these they were not. A member with no parameters at all is a different case
  // and is not reported - nothing was left undone for it.
  //
  // A member routed to a peer never contributes a local node: an id a peer
  // announces that this gateway also declares is renamed `<peer>__<id>` by the
  // merge, so the loop above and this one cannot name the same member.
  if (auto * agg = ctx_.aggregation_manager(); agg != nullptr) {
    for (const auto & member : cache.get_members(entity.sovd_type(), entity_id)) {
      // Apps only: a parameter belongs to a ROS node, an App is what carries
      // one, and a peer-owned Component's Apps are members in their own right.
      // Naming the Component too would report the same gap twice.
      if (!cache.get_app(member)) {
        continue;
      }
      auto peer = agg->find_peer_for_entity(member);
      if (!peer) {
        continue;
      }

      std::string reason = "Not reset here: '";
      reason += member;
      reason += "' is owned by gateway '";
      reason += *peer;
      reason += "'. Reset it on that gateway, through its own /apps/";
      reason += member;
      reason += "/configurations route.";

      dto::ConfigurationDeleteResultItem entry;
      entry.app_id = member;
      entry.success = false;
      entry.error = std::move(reason);
      all_reset = false;
      multi_status.results.push_back(std::move(entry));
    }
  }

  if (all_reset) {
    // Every member this entity has was reset - including the case where it has
    // none at all, which SOVD answers as a success with no content.
    return ResultVariant{http::NoContent{}};
  }
  return ResultVariant{std::move(multi_status)};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
