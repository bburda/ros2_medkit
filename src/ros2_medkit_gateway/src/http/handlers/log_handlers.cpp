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

#include "ros2_medkit_gateway/core/http/handlers/log_handlers.hpp"

#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

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

/// Parse a JSON array of LogEntry-shaped objects (the wire shape produced by
/// `LogManager::entry_to_json`) into a typed `std::vector<dto::LogEntry>`.
/// Items that fail validation are dropped silently - they would also be
/// dropped by the legacy raw-JSON path when the consumer rehydrates them, and
/// the local writer is the only producer of this shape.
std::vector<dto::LogEntry> parse_local_items(const json & arr) {
  std::vector<dto::LogEntry> out;
  if (!arr.is_array()) {
    return out;
  }
  out.reserve(arr.size());
  for (const auto & item : arr) {
    auto parsed = dto::JsonReader<dto::LogEntry>::read(item);
    if (parsed.has_value()) {
      out.push_back(std::move(parsed.value()));
    }
  }
  return out;
}

/// Fold the partial/failed_peers/peer_dropped_items observability fields from
/// a `FanOutResult<LogEntry>` into the typed `LogListXMedkit`. Mirrors the
/// legacy behaviour where `merge_peer_items` injected `partial` and
/// `failed_peers` keys into the x-medkit JSON; here the typed equivalent is a
/// direct field assignment plus dropped-item enrichment from the typed
/// reader.
void apply_fan_out_observability(dto::LogListXMedkit & xm, const FanOutResult<dto::LogEntry> & fan_out) {
  if (fan_out.partial) {
    xm.partial = true;
    xm.failed_peers = fan_out.failed_peers;
    xm.peer_failures = fan_out.peer_failures;
  }
  if (!fan_out.dropped_items.empty()) {
    xm.peer_dropped_items = fan_out.dropped_items;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// GET /{entity-path}/logs
// ---------------------------------------------------------------------------

http::Result<dto::Collection<dto::LogEntry, dto::LogListXMedkit>>
LogHandlers::get_logs(const http::TypedRequest & req) {
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

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "LogManager not available"));
  }

  // Validate optional query parameters (shared across all entity types)
  const auto q = req.query<dto::LogQuery>();
  const std::string min_severity = q.severity.value_or(std::string{});
  const std::string context_filter = q.context.value_or(std::string{});

  if (!min_severity.empty() && !LogManager::is_valid_severity(min_severity)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Invalid severity value: must be one of debug, info, warning, error, fatal"));
  }

  static constexpr size_t kMaxContextFilterLen = 256;
  if (context_filter.size() > kMaxContextFilterLen) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "context filter exceeds maximum length of 256"));
  }

  dto::Collection<dto::LogEntry, dto::LogListXMedkit> response;
  dto::LogListXMedkit xm;
  bool xm_used = false;

  // Local query: one shared scope resolution for every entity type. Exact
  // sources follow the fault-scope rule (external app -> bare id, external
  // component owns its own id, areas recurse subareas, functions follow
  // component hosts); COMPONENT and AREA additionally merge the entity.fqn
  // namespace-prefix query, so a component whose apps are all external keeps
  // the ROS node entries under its namespace.
  const auto & cache = ctx_.node()->get_thread_safe_cache();
  std::set<std::string> resolved;
  auto local_logs = logs::query_entity_logs(*log_mgr, cache, entity.sovd_type(), entity_id, entity.fqn, min_severity,
                                            context_filter, &resolved);
  if (!local_logs) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, local_logs.error()));
  }
  response.items = parse_local_items(*local_logs);

  if (entity.type == EntityType::FUNCTION) {
    xm.entity_id = entity_id;
    xm.aggregation_level = "function";
    xm.aggregated = true;
    xm_used = true;
    if (!resolved.empty()) {
      xm.host_count = static_cast<int64_t>(resolved.size());
      xm.aggregation_sources = std::vector<std::string>(resolved.begin(), resolved.end());
    }
  } else if (entity.type == EntityType::AREA) {
    xm.entity_id = entity_id;
    xm.aggregation_level = "area";
    xm.aggregated = true;
    xm_used = true;
    if (!resolved.empty()) {
      xm.component_count = static_cast<int64_t>(cache.get_components_for_area(entity_id).size());
      xm.app_count = static_cast<int64_t>(resolved.size());
      xm.aggregation_sources = std::vector<std::string>(resolved.begin(), resolved.end());
    }
  } else if (entity.type == EntityType::COMPONENT) {
    xm.entity_id = entity_id;
    xm.aggregation_level = "component";
    xm.aggregated = true;
    xm_used = true;
    if (!resolved.empty()) {
      xm.app_count = static_cast<int64_t>(resolved.size());
      xm.aggregation_sources = std::vector<std::string>(resolved.begin(), resolved.end());
    }
  }
  // APP: x-medkit only appears when fan-out below produces observability
  // output; tracked via xm_used (still false here).

  // Typed fan-out for collection endpoints. Replaces the legacy raw-JSON
  // `merge_peer_items` mutator: peer items come back as parsed `dto::LogEntry`
  // values via `JsonReader<LogEntry>`, malformed peer items are surfaced as
  // `dropped_items` (folded into xm.peer_dropped_items below), and
  // partial/failed_peers go into the typed xm fields. fan_out_collection still
  // operates on the raw cpp-httplib request (path + headers) - the typed-
  // request raw escape hatch is used deliberately here; a later commit will
  // accept the typed request directly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto fan_out = fan_out_collection<dto::LogEntry>(ctx_.aggregation_manager(), raw_req);
  for (auto & item : fan_out.items) {
    response.items.push_back(std::move(item));
  }
  apply_fan_out_observability(xm, fan_out);
  if (fan_out.partial || !fan_out.dropped_items.empty()) {
    xm_used = true;
  }

  if (xm_used) {
    response.x_medkit = std::move(xm);
  }
  return response;
}

// ---------------------------------------------------------------------------
// GET /{entity-path}/logs/configuration
// ---------------------------------------------------------------------------

http::Result<dto::LogConfiguration> LogHandlers::get_logs_configuration(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "LogManager not available"));
  }

  auto cfg = log_mgr->get_config(entity_id);
  if (!cfg) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, cfg.error()));
  }

  dto::LogConfiguration response;
  response.severity_filter = cfg->severity_filter;
  response.max_entries = static_cast<int64_t>(cfg->max_entries);
  return response;
}

// ---------------------------------------------------------------------------
// PUT /{entity-path}/logs/configuration
// ---------------------------------------------------------------------------

http::Result<http::NoContent> LogHandlers::put_logs_configuration(const http::TypedRequest & req,
                                                                  dto::LogConfiguration body) {
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

  // Check lock access for logs (typed validator returns ErrorInfo directly).
  if (auto lock_err = ctx_.validate_lock_access(req, entity, "logs"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "LogManager not available"));
  }

  std::optional<std::string> severity_filter;
  std::optional<size_t> max_entries;

  if (body.severity_filter.has_value()) {
    severity_filter = body.severity_filter;
  }

  static constexpr int64_t kMaxEntriesCap = 10000;
  if (body.max_entries.has_value()) {
    const int64_t val = *body.max_entries;
    if (val <= 0) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "max_entries must be greater than 0"));
    }
    if (val > kMaxEntriesCap) {
      return tl::unexpected(
          make_error(400, ERR_INVALID_PARAMETER, "max_entries exceeds maximum allowed value of 10000"));
    }
    max_entries = static_cast<size_t>(val);
  }

  const auto err = log_mgr->update_config(entity_id, severity_filter, max_entries);
  if (!err.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, err));
  }

  return http::NoContent{};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
