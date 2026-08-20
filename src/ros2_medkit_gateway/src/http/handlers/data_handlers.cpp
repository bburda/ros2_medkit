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

#include "ros2_medkit_gateway/core/http/handlers/data_handlers.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/exceptions.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/member_qualified_id.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"
#include "ros2_medkit_serialization/type_introspection.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

// =============================================================================
// Helper free functions
// =============================================================================

/// Sanitize a plugin-supplied error into the standard `x-medkit-plugin-error`
/// shape: clamp HTTP status to [400, 599] and truncate message at 512 chars.
ErrorInfo make_plugin_error(int http_status, const std::string & message, json extra_params = {}) {
  static constexpr size_t kMaxMessageLength = 512;
  int status = http_status < 400 ? 400 : (http_status > 599 ? 599 : http_status);
  std::string msg = message.size() > kMaxMessageLength ? message.substr(0, kMaxMessageLength) + "..." : message;
  return make_error(status, ERR_PLUGIN_ERROR, std::move(msg), std::move(extra_params));
}

/// Map a `DataProviderErrorInfo` (from the typed plugin ABI) into the SOVD
/// `x-medkit-plugin-error` wire shape via `make_plugin_error`.
ErrorInfo make_provider_error(const DataProviderErrorInfo & info, const std::string & entity_id) {
  return make_plugin_error(info.http_status, info.message, json{{"entity_id", entity_id}});
}

/// Read the first positional capture group (entity_id) with the same "missing
/// capture is treated as 400 invalid-request" semantics as the other migrated
/// handlers (matches the legacy `req.matches.size() < N` guard).
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the second positional capture group (topic_id; cpp-httplib already
/// percent-decodes the value).
tl::expected<std::string, ErrorInfo> read_topic_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// A data item is addressed by its full ROS topic path; the leading slash is
/// optional on the wire because the route captures a percent-decoded segment.
std::string to_full_topic_path(const std::string & topic_name) {
  if (topic_name.empty() || topic_name.front() == '/') {
    return topic_name;
  }
  return "/" + topic_name;
}

/// What one addressed data item resolves to: the ROS topic to act on, and the
/// id to echo back to the caller.
struct AddressedDataItem {
  std::string full_topic_path;
  std::string item_id;
};

/// Resolve the id in the route against the entity, for reads and writes alike.
///
/// A qualified id is answered exactly, because the member set and what each
/// member contributes are both known here: an id naming an unknown member, or
/// an item that member does not provide, is a miss. Without that check the
/// gateway samples the local graph, finds nothing, and returns 200 with an
/// empty body and status `metadata_only` - a typo reported as success.
///
/// A ROS topic name cannot contain a colon, so one in the id can only be the
/// member separator. Building the member set is not free, so the cache is only
/// consulted for an id that carries one; a bare id addresses a topic path,
/// which names one topic on its own and keeps its existing behaviour.
tl::expected<AddressedDataItem, ErrorInfo>
address_data_item(const ThreadSafeEntityCache & cache, const std::string & entity_id, const std::string & topic_name) {
  AddressedDataItem addressed;
  addressed.full_topic_path = to_full_topic_path(topic_name);
  addressed.item_id = addressed.full_topic_path;

  if (topic_name.find(':') == std::string::npos) {
    return addressed;
  }

  auto aggregated = cache.get_entity_data(entity_id);
  auto parsed = http::parse_member_qualified_id(topic_name, aggregated.is_aggregated);
  if (!parsed.has_member) {
    return addressed;
  }

  if (std::find(aggregated.source_ids.begin(), aggregated.source_ids.end(), parsed.member_id) ==
      aggregated.source_ids.end()) {
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Member not found in entity",
                   json{{"entity_id", entity_id}, {"id", topic_name}, {"member_id", parsed.member_id}}));
  }

  // A member retained while its gateway is silent stays addressable and says
  // why it cannot answer, rather than falling through to a sample of the local
  // graph that comes back empty and reads as success.
  if (auto app = cache.get_app(parsed.member_id); app && !app->available) {
    return tl::make_unexpected(
        make_error(504, ERR_NOT_RESPONDING, "Member '" + parsed.member_id + "' is not available",
                   json{{"details",
                         "The gateway contributing this member is not answering; it is retained from its "
                         "last known declaration"},
                        {"entity_id", entity_id},
                        {"id", topic_name},
                        {"member_id", parsed.member_id}}));
  }

  addressed.full_topic_path = to_full_topic_path(parsed.item_id);
  auto owners = aggregated.owners_by_topic.find(addressed.full_topic_path);
  if (owners == aggregated.owners_by_topic.end() ||
      std::find(owners->second.begin(), owners->second.end(), parsed.member_id) == owners->second.end()) {
    return tl::make_unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Data item not provided by member",
                                          json{{"entity_id", entity_id},
                                               {"id", topic_name},
                                               {"member_id", parsed.member_id},
                                               {"topic_name", addressed.full_topic_path}}));
  }

  addressed.item_id = http::make_member_qualified_id(parsed.member_id, addressed.full_topic_path);
  return addressed;
}

/// Build the typed x-medkit per-item payload for the list endpoint.
dto::XMedkitDataItem build_list_item_xmedkit(const std::string & topic_name, const std::string & direction,
                                             const std::string & topic_type,
                                             ros2_medkit_serialization::TypeIntrospection * type_introspection) {
  dto::XMedkitDataItem item_xm;
  dto::XMedkitRos2 ros2_meta;
  ros2_meta.topic = topic_name;
  ros2_meta.direction = direction;
  if (!topic_type.empty()) {
    ros2_meta.type = topic_type;
    try {
      auto type_info = type_introspection->get_type_info(topic_type);
      json type_info_obj;
      type_info_obj["schema"] = type_info.schema;
      type_info_obj["default_value"] = type_info.default_value;
      item_xm.type_info = type_info_obj;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", topic_name.c_str(),
                   e.what());
    }
  }
  item_xm.ros2 = ros2_meta;
  return item_xm;
}

/// Fold typed fan-out observability (partial / failed_peers / dropped items)
/// into the list-level x-medkit. Mirrors the legacy `merge_peer_items`
/// post-processing path.
void apply_fan_out_observability(dto::DataListXMedkit & xm, const FanOutResult<dto::DataItem> & fan_out) {
  if (fan_out.partial) {
    xm.partial = true;
    xm.failed_peers = fan_out.failed_peers;
  }
  if (!fan_out.dropped_items.empty()) {
    xm.peer_dropped_items = fan_out.dropped_items;
  }
}

/// Build a `DataValue` whose `content` matches the wire shape produced by the
/// legacy `handle_get_data_item` (top-level `id`, `data`, `x-medkit`).
/// `item_id` is the id the caller addressed - a qualified id when it named a
/// member, the topic path otherwise - and is echoed so a caller can keep using
/// the id it read out of the collection. `full_topic_path` is the ROS topic
/// that was actually sampled.
dto::DataValue build_read_response(const std::string & item_id, const std::string & full_topic_path,
                                   const TopicSampleResult & sample, const std::string & entity_id,
                                   ros2_medkit_serialization::TypeIntrospection * type_introspection) {
  json response;
  response["id"] = item_id;
  if (sample.has_data && sample.data) {
    response["data"] = *sample.data;
  } else {
    response["data"] = json::object();
  }

  dto::XMedkitDataItem xm;
  dto::XMedkitRos2 ros2_meta;
  ros2_meta.topic = full_topic_path;
  xm.entity_id = entity_id;
  if (!sample.message_type.empty()) {
    ros2_meta.type = sample.message_type;
    try {
      auto type_info = type_introspection->get_type_info(sample.message_type);
      json type_info_obj;
      type_info_obj["schema"] = type_info.schema;
      type_info_obj["default_value"] = type_info.default_value;
      xm.type_info = type_info_obj;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", full_topic_path.c_str(),
                   e.what());
    }
  }
  xm.ros2 = ros2_meta;
  xm.timestamp = sample.timestamp_ns;
  xm.publisher_count = static_cast<std::int64_t>(sample.publisher_count);
  xm.subscriber_count = static_cast<std::int64_t>(sample.subscriber_count);
  xm.status = json(sample.has_data ? "data" : "metadata_only");
  response["x-medkit"] = dto::JsonWriter<dto::XMedkitDataItem>::write(xm);

  dto::DataValue value;
  value.content = std::move(response);
  return value;
}

/// Build a `DataValue` whose `content` matches the wire shape produced by the
/// legacy `handle_put_data_item` write echo (top-level `id`, `data`,
/// `x-medkit`).
dto::DataValue build_write_response(const std::string & item_id, const std::string & full_topic_path,
                                    const std::string & msg_type, const std::string & entity_id, const json & data,
                                    const json & publish_result) {
  json response;
  response["id"] = item_id;
  response["data"] = data;  // Echo back the written data

  dto::XMedkitDataItem xm;
  dto::XMedkitRos2 ros2_meta;
  ros2_meta.topic = full_topic_path;
  ros2_meta.type = msg_type;
  xm.ros2 = ros2_meta;
  xm.entity_id = entity_id;
  if (publish_result.contains("status")) {
    xm.status = publish_result["status"];
  }
  if (publish_result.contains("publisher_created") && publish_result["publisher_created"].is_boolean()) {
    xm.publisher_created = publish_result["publisher_created"].get<bool>();
  }
  response["x-medkit"] = dto::JsonWriter<dto::XMedkitDataItem>::write(xm);

  dto::DataValue value;
  value.content = std::move(response);
  return value;
}

}  // namespace

// =============================================================================
// GET /{entity}/data - list data items
// =============================================================================

http::Result<dto::DataListResult> DataHandlers::list_data(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Delegate to plugin DataProvider for plugin-owned entities.
  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
    if (data_prov == nullptr) {
      // PLC bridges serve live values through their own x-plc-data route
      // instead of a DataProvider - dispatch it in-process (same path the
      // freeze-frame capture uses) so /data works for them too. The vendor
      // shape differs from the DataProvider one (name = data name, no id, no
      // category), and SOVD requires id on every item - normalize here so
      // /data items always carry id/name/category regardless of which plugin
      // path produced them; vendor extras pass through untouched.
      if (auto via_route = pmgr ? pmgr->fetch_entity_data_via_route(entity_id) : std::nullopt) {
        if (via_route->contains("items") && (*via_route)["items"].is_array()) {
          for (auto & item : (*via_route)["items"]) {
            if (!item.is_object()) {
              continue;
            }
            if (!item.contains("id") && item.contains("name")) {
              item["id"] = item["name"];
            }
            if (!item.contains("category")) {
              item["category"] = "currentData";
            }
          }
        }
        return dto::DataListResult{std::move(*via_route)};
      }
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "No data provider for plugin entity '" + entity_id + "'"));
    }
    try {
      auto result = data_prov->list_data(entity_id);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id));
      }
      // The plugin returns an opaque `DataListResult`. Pass it through verbatim:
      // the per-item shape is decided by the plugin (the in-tree OPC-UA plugin
      // adds value/unit/data_type/writable), so re-parsing it into the typed
      // `Collection<DataItem>` would silently drop every field outside
      // {id,name,category} (JsonReader is lenient). The opaque envelope is the
      // contract for exactly this case.
      return std::move(*result);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  try {
    // Use unified cache method to get aggregated data.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto aggregated = cache.get_entity_data(entity_id);

    // Get data access manager for type introspection.
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    dto::Collection<dto::DataItem, dto::DataListXMedkit> response;
    for (const auto & topic : aggregated.topics) {
      dto::DataItem di;
      di.id = topic.name;
      di.name = topic.name;
      di.category = "currentData";
      const std::string topic_type = cache.get_topic_type(topic.name);
      di.x_medkit = build_list_item_xmedkit(topic.name, topic.direction, topic_type, type_introspection);
      // Attribute the item to the members that contribute it. Only a grouping
      // has members to name; on a leaf the entity is the sole contributor and
      // the field stays absent rather than repeating the entity id.
      if (auto owners = aggregated.owners_by_topic.find(topic.name);
          owners != aggregated.owners_by_topic.end() && aggregated.is_aggregated) {
        di.x_medkit->member_ids = owners->second;
      }
      response.items.push_back(std::move(di));
    }

    // Typed fan-out for the data list. Replaces the legacy raw-JSON
    // `merge_peer_items` mutator: peer items come back as parsed
    // `dto::DataItem` values via `JsonReader<DataItem>`, malformed peer items
    // are surfaced as `dropped_items` (folded into xm.peer_dropped_items
    // below), and partial/failed_peers go into the typed xm fields.
    // `fan_out_collection` still operates on the raw cpp-httplib request
    // (path + headers) - the typed-request raw escape hatch is used
    // deliberately here; a later commit will accept the typed request
    // directly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
    auto fan_out = fan_out_collection<dto::DataItem>(ctx_.aggregation_manager(), raw_req);
    for (auto & item : fan_out.items) {
      response.items.push_back(std::move(item));
    }

    // A topic path names one topic however many members publish and subscribe
    // to it - those merge into a single item, whose contributors are already
    // named in member_ids - so nothing here is qualified in the ordinary case.
    // Two gateways each contributing an item under one path is the case that
    // is genuinely ambiguous, and it is the case this catches.
    http::qualify_ambiguous_ids(response.items, [](const dto::DataItem & item) {
      return item.x_medkit.has_value() && item.x_medkit->member_ids.has_value() ? &*item.x_medkit->member_ids : nullptr;
    });

    dto::DataListXMedkit xm;
    xm.entity_id = entity_id;
    if (aggregated.is_aggregated) {
      xm.aggregated = true;
      xm.aggregation_sources = aggregated.source_ids;
      xm.aggregation_level = aggregated.aggregation_level;
    }
    xm.total_count = response.items.size();
    apply_fan_out_observability(xm, fan_out);
    response.x_medkit = std::move(xm);
    // Serialize the typed runtime collection into the opaque envelope. The wire
    // shape ({items:[...], x-medkit:{...}}) is byte-identical to the previous
    // typed return; only the published OpenAPI schema for this route is now the
    // opaque DataListResult (consistent with the fault list route).
    return dto::DataListResult{dto::JsonWriter<dto::Collection<dto::DataItem, dto::DataListXMedkit>>::write(response)};
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in list_data for entity '%s': %s", entity_id.c_str(), e.what());
    return tl::make_unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to retrieve entity data",
                                          json{{"details", e.what()}, {"entity_id", entity_id}}));
  }
}

// =============================================================================
// GET /{entity}/data/{topic} - read data item
// =============================================================================

http::Result<dto::DataValue> DataHandlers::get_data_item(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto topic_result = read_topic_id(req);
  if (!topic_result) {
    return tl::make_unexpected(topic_result.error());
  }
  const std::string topic_name = *topic_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Delegate to plugin DataProvider for plugin-owned entities.
  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
    if (data_prov == nullptr) {
      // Same x-plc-data route fallback as list_data above, single-item form.
      if (auto via_route = pmgr ? pmgr->fetch_entity_data_via_route(entity_id, topic_name) : std::nullopt) {
        return dto::DataValue{std::move(*via_route)};
      }
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "No data provider for plugin entity '" + entity_id + "'"));
    }
    try {
      auto result = data_prov->read_data(entity_id, topic_name);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id));
      }
      return std::move(*result);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  try {
    auto addressed = address_data_item(ctx_.node()->get_thread_safe_cache(), entity_id, topic_name);
    if (!addressed) {
      return tl::make_unexpected(addressed.error());
    }
    const std::string & full_topic_path = addressed->full_topic_path;

    // Sampling goes through the pool-backed TopicDataProvider (issue #375 race
    // fix). The provider is configured in main() before serving traffic.
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    const auto timeout_sec = data_access_mgr->get_topic_sample_timeout();
    auto * provider = data_access_mgr->get_topic_data_provider();
    if (provider == nullptr) {
      return tl::make_unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Topic sampling is not configured"));
    }
    const auto timeout_ms = std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout_sec, 0.0) * 1000.0)};
    auto r = provider->sample(full_topic_path, timeout_ms);
    if (!r) {
      // Propagate the provider's ErrorInfo verbatim: http_status, code (SOVD
      // constant or x-medkit-*), message, and structured params. Collapsing
      // all errors to metadata-only (previous behaviour) would mask 503s
      // (gateway shutdown, cold-wait cap) and 500s (subscribe failed) as
      // successful metadata-only responses, which breaks retry-on-5xx clients.
      const auto & err = r.error();
      json params = err.params;
      params["entity_id"] = entity_id;
      params["topic_name"] = topic_name;
      const std::string code = err.code.empty() ? std::string{ERR_INTERNAL_ERROR} : err.code;
      const int status = (err.http_status >= 400 && err.http_status < 600) ? err.http_status : 500;
      return tl::make_unexpected(make_error(status, code, err.message, std::move(params)));
    }

    auto type_introspection = data_access_mgr->get_type_introspection();
    return build_read_response(addressed->item_id, full_topic_path, *r, entity_id, type_introspection);
  } catch (const TopicNotAvailableException & e) {
    RCLCPP_DEBUG(HandlerContext::logger(), "Topic not available for entity '%s', topic '%s': %s", entity_id.c_str(),
                 topic_name.c_str(), e.what());
    return tl::make_unexpected(make_error(404, ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE, "Topic not found",
                                          json{{"entity_id", entity_id}, {"topic_name", topic_name}}));
  } catch (const ProviderErrorException & e) {
    // Provider returned a non-404 ErrorInfo via DAM (shutdown, subscribe
    // failed, pool saturation). Preserve the original http_status / code so
    // 5xx surfaces to retry-on-5xx clients instead of looking like 404.
    const auto & info = e.info();
    json params = info.params;
    params["entity_id"] = entity_id;
    params["topic_name"] = topic_name;
    const std::string code = info.code.empty() ? std::string{ERR_INTERNAL_ERROR} : info.code;
    const int status = (info.http_status >= 400 && info.http_status < 600) ? info.http_status : 500;
    RCLCPP_WARN(HandlerContext::logger(), "Provider error for entity '%s', topic '%s': %s [%d]", entity_id.c_str(),
                topic_name.c_str(), info.message.c_str(), info.http_status);
    return tl::make_unexpected(make_error(status, code, info.message, std::move(params)));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in get_data_item for entity '%s', topic '%s': %s", entity_id.c_str(),
                 topic_name.c_str(), e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to retrieve topic data",
                   json{{"details", e.what()}, {"entity_id", entity_id}, {"topic_name", topic_name}}));
  }
}

// =============================================================================
// PUT /{entity}/data/{topic} - write data item
// =============================================================================

http::Result<dto::DataValue> DataHandlers::put_data_item(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::make_unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto topic_result = read_topic_id(req);
  if (!topic_result) {
    return tl::make_unexpected(topic_result.error());
  }
  const std::string topic_name = *topic_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::make_unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto entity_info = *entity_result;

  // Lock access for data (before plugin delegation - locks apply to all entities).
  if (auto lock_err = ctx_.validate_lock_access(req, entity_info, "data"); !lock_err) {
    return tl::make_unexpected(lock_err.error());
  }

  // Body is parsed manually because plugin-owned entities accept free-form
  // JSON (e.g. UDS sends a bare hex-encoded string), while the ROS path
  // requires the strict `DataWriteRequest` shape. Auto-binding
  // `DataWriteRequest` at the framework level would break plugin
  // compatibility. The typed-request raw escape hatch is used deliberately
  // here; a later commit will expose `body_raw()` on the typed request.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const std::string & raw_body = req.raw_for_framework().body;
#pragma GCC diagnostic pop

  // Delegate to plugin DataProvider for plugin-owned entities.
  if (entity_info.is_plugin) {
    auto * pmgr = ctx_.node()->get_plugin_manager();
    auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
    if (data_prov == nullptr) {
      return tl::make_unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "No data provider for plugin entity '" + entity_id + "'"));
    }
    json value;
    if (!raw_body.empty()) {
      value = json::parse(raw_body, nullptr, false);
      if (value.is_discarded()) {
        return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid JSON body"));
      }
    }
    try {
      auto result = data_prov->write_data(entity_id, topic_name, value);
      if (!result) {
        return tl::make_unexpected(make_provider_error(result.error(), entity_id));
      }
      dto::DataValue data_value;
      data_value.content = std::move(result->content);
      return data_value;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                   e.what());
      return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
    } catch (...) {
      RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                   entity_id.c_str());
      return tl::make_unexpected(
          make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
    }
  }

  // ROS path: enforce the strict `DataWriteRequest` shape via JsonReader.
  // Matches the legacy `parse_body<DataWriteRequest>` semantics byte-for-byte:
  // - Malformed JSON renders as 400 ERR_INVALID_REQUEST with message
  //   "Request body is not valid JSON".
  // - Field-validation failure renders as 400 ERR_INVALID_REQUEST with the
  //   joined field-error string as the message (so integration tests that
  //   grep for "data" / "type" in the message keep passing).
  json body_json;
  try {
    body_json = json::parse(raw_body);
  } catch (const json::parse_error &) {
    return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, "Request body is not valid JSON"));
  }
  auto parsed = dto::JsonReader<dto::DataWriteRequest>::read(body_json);
  if (!parsed) {
    std::string detail;
    for (const auto & err : parsed.error()) {
      if (!detail.empty()) {
        detail += "; ";
      }
      detail += err.field + ": " + err.message;
    }
    return tl::make_unexpected(make_error(400, ERR_INVALID_REQUEST, std::move(detail)));
  }
  dto::DataWriteRequest body = std::move(*parsed);

  try {
    const std::string & msg_type = body.type;
    const json & data = body.data;

    // Validate message type format (e.g., std_msgs/msg/Float32).
    auto slash_count = static_cast<size_t>(std::count(msg_type.begin(), msg_type.end(), '/'));
    size_t msg_pos = msg_type.find("/msg/");
    bool valid_format =
        (slash_count == 2) && (msg_pos != std::string::npos) && (msg_pos > 0) && (msg_pos + 5 < msg_type.length());
    if (!valid_format) {
      return tl::make_unexpected(
          make_error(400, ERR_INVALID_PARAMETER, "Invalid message type format",
                     json{{"details", "Message type should be in format: package/msg/Type"}, {"type", msg_type}}));
    }

    // A write addresses the same item a read does, so it resolves the same way.
    auto addressed = address_data_item(ctx_.node()->get_thread_safe_cache(), entity_id, topic_name);
    if (!addressed) {
      return tl::make_unexpected(addressed.error());
    }
    const std::string & full_topic_path = addressed->full_topic_path;

    // Publish data using DataAccessManager.
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    json publish_result = data_access_mgr->publish_to_topic(full_topic_path, msg_type, data);
    return build_write_response(addressed->item_id, full_topic_path, msg_type, entity_id, data, publish_result);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in put_data_item for entity '%s', topic '%s': %s", entity_id.c_str(),
                 topic_name.c_str(), e.what());
    return tl::make_unexpected(
        make_error(500, ERR_INTERNAL_ERROR, "Failed to publish to topic",
                   json{{"details", e.what()}, {"entity_id", entity_id}, {"topic_name", topic_name}}));
  }
}

// =============================================================================
// GET /{entity}/data-categories - 501 Not Implemented
// =============================================================================

http::Result<dto::DataValue> DataHandlers::data_categories(const http::TypedRequest & /*req*/) {
  return tl::make_unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Data categories are not implemented for ROS 2",
                                        json{{"feature", "data-categories"}}));
}

// =============================================================================
// GET /{entity}/data-groups - 501 Not Implemented
// =============================================================================

http::Result<dto::DataValue> DataHandlers::data_groups(const http::TypedRequest & /*req*/) {
  return tl::make_unexpected(make_error(501, ERR_NOT_IMPLEMENTED, "Data groups are not implemented for ROS 2",
                                        json{{"feature", "data-groups"}}));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
