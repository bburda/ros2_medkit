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

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"

namespace ros2_medkit_gateway {

/// Log a "peer item failed to parse" warning. Defined in src/http/ so the
/// core/ header layer stays free of rclcpp includes (gateway_core_purity
/// invariant). Templates in this header forward through this function rather
/// than calling RCLCPP_WARN directly.
void log_peer_drop_warning(const char * dto_name, const char * reason);

inline std::string url_encode_param(const std::string & value) {
  std::string result;
  result.reserve(value.size());
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (const auto ch : value) {
    auto c = static_cast<unsigned char>(ch);
    // Locale-independent RFC 3986 unreserved character check (ASCII-only).
    // std::isalnum is locale-dependent and could pass bytes >= 0x80.
    bool unreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
                      c == '_' || c == '.' || c == '~';
    if (unreserved) {
      result += static_cast<char>(c);
    } else {
      result += '%';
      result += kHex[static_cast<size_t>(c >> 4u)];
      result += kHex[static_cast<size_t>(c & 0x0Fu)];
    }
  }
  return result;
}

// Extract the entity id from a per-entity collection path like
// `/api/v1/components/<id>/logs` or `/api/v1/apps/<id>/data/<sub>`.
// Matching is intentionally permissive: any path containing
// `/components|apps|areas|functions/<id>/<anything>` is treated as
// per-entity, regardless of what the segment after `<id>` is. This keeps
// future per-entity sub-endpoints eligible for target-filtered fan-out
// without requiring a whitelist bump every time.
//
// Returns std::nullopt for:
//   - global endpoints (`/api/v1/faults`, `/api/v1/health`, ...),
//   - entity-detail endpoints (`/api/v1/components/<id>` with no trailing
//     `/...` - these go through the forwarding path, not fan-out),
//   - malformed paths with an empty entity segment (`/components//logs`).
inline std::optional<std::string> extract_entity_id_for_fan_out(const std::string & path) {
  static constexpr std::array<std::string_view, 4> kEntityCollections = {"/components/", "/apps/", "/areas/",
                                                                         "/functions/"};
  for (const auto & prefix : kEntityCollections) {
    auto start = path.find(prefix);
    if (start == std::string::npos) {
      continue;
    }
    start += prefix.size();
    auto end = path.find('/', start);
    if (end == std::string::npos) {
      // Entity detail endpoint like `/components/<id>` (no trailing collection).
      // These are routed to peers via a different mechanism; not a fan-out case.
      return std::nullopt;
    }
    std::string id = path.substr(start, end - start);
    if (id.empty()) {
      return std::nullopt;
    }
    return id;
  }
  return std::nullopt;
}

inline std::string build_fan_out_path(const httplib::Request & req) {
  if (req.params.empty()) {
    return req.path;
  }
  std::string path = req.path;
  char sep = '?';
  for (const auto & [key, value] : req.params) {
    path += sep;
    path += url_encode_param(key);
    path += '=';
    path += url_encode_param(value);
    sep = '&';
  }
  return path;
}

/// Fan-out GET to peer gateways and merge their items into `result["items"]`.
/// Aggregation metadata (partial, failed_peers) is written into `ext_json`
/// (a plain JSON object that callers fold into the response x-medkit block).
inline void merge_peer_items(AggregationManager * agg, const httplib::Request & req, nlohmann::json & result,
                             nlohmann::json & ext_json) {
  if (agg == nullptr) {
    return;
  }
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    return;
  }
  if (agg->healthy_peer_count() == 0) {
    return;
  }
  std::optional<std::vector<std::string>> contributors_buffer;
  const std::vector<std::string> * target_peers = nullptr;
  if (auto entity_id = extract_entity_id_for_fan_out(req.path); entity_id.has_value()) {
    contributors_buffer = agg->get_peer_contributors(*entity_id);
    if (contributors_buffer->empty()) {
      return;
    }
    target_peers = &contributors_buffer.value();
  }
  auto fan_path = build_fan_out_path(req);
  auto fan_result = agg->fan_out_get(fan_path, req.get_header_value("Authorization"), target_peers);
  if (fan_result.merged_items.is_array() && !fan_result.merged_items.empty()) {
    if (!result.contains("items") || !result["items"].is_array()) {
      result["items"] = nlohmann::json::array();
    }
    for (const auto & item : fan_result.merged_items) {
      if (item.is_object()) {
        result["items"].push_back(item);
      }
    }
  }
  if (fan_result.is_partial) {
    ext_json["partial"] = true;
    ext_json["failed_peers"] = fan_result.failed_peers;
  }
}

/// Typed fan-out result for collection endpoints. Mirrors merge_peer_items
/// but returns parsed `T` items (via dto::JsonReader<T>) plus typed observability
/// records for items that failed validation, instead of mutating raw JSON in
/// place.
template <class T>
struct FanOutResult {
  /// Typed peer items that successfully parsed as `T`.
  std::vector<T> items;
  /// Name of the peer that sent `items[i]`, same order and length as `items`.
  ///
  /// A peer names its own members, and a member this gateway renamed on
  /// collision (`<peer>__<id>`) is called something else here, so an item can
  /// only be re-addressed to the leaf that owns it once the sending peer is
  /// known. Kept parallel rather than folded into `T` because `T` is the wire
  /// DTO and provenance is not part of the wire shape.
  std::vector<std::string> item_peers;
  /// True if at least one targeted peer failed.
  bool partial{false};
  /// Names of peers that failed (matches AggregationManager::FanOutResult.failed_peers).
  std::vector<std::string> failed_peers;
  /// Per-item drop records for peer items that failed JsonReader<T> validation.
  /// Surfaces "invisible drift" - malformed peer items used to disappear silently;
  /// now callers can fold these into x-medkit.peer_dropped_items for observability.
  std::vector<dto::DroppedItem> dropped_items;
};

/// Typed fan-out for collection endpoints. Replacement for merge_peer_items
/// that returns parsed `T` items instead of mutating a raw JSON document.
///
/// Behavior parity with merge_peer_items:
///   - null `agg`, `X-Medkit-No-Fan-Out` header, or zero healthy peers all
///     short-circuit to an empty result (no fan-out attempted).
///   - per-entity paths consult the routing/contributor tables to target only
///     the peers that host the entity; global paths fan out to all healthy peers.
///   - peer failures are surfaced via `partial` + `failed_peers`.
///
/// New behavior:
///   - peer items are parsed via `dto::JsonReader<T>` rather than copied as
///     raw JSON. Items that fail validation are dropped from `items` and
///     recorded in `dropped_items` with the JsonReader error message plus a
///     best-effort `source_id`. A WARN is logged for each drop.
///   - every item that parsed carries the peer it came from, in `item_peers`.
template <class T>
inline FanOutResult<T> fan_out_collection(AggregationManager * agg, const httplib::Request & req) {
  FanOutResult<T> result;
  if (agg == nullptr) {
    return result;
  }
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    return result;
  }
  if (agg->healthy_peer_count() == 0) {
    return result;
  }

  std::optional<std::vector<std::string>> contributors_buffer;
  const std::vector<std::string> * target_peers = nullptr;
  if (auto entity_id = extract_entity_id_for_fan_out(req.path); entity_id.has_value()) {
    contributors_buffer = agg->get_peer_contributors(*entity_id);
    if (contributors_buffer->empty()) {
      return result;
    }
    target_peers = &contributors_buffer.value();
  }

  auto fan_path = build_fan_out_path(req);
  auto fan_result = agg->fan_out_get(fan_path, req.get_header_value("Authorization"), target_peers);

  if (fan_result.merged_items.is_array()) {
    for (size_t index = 0; index < fan_result.merged_items.size(); ++index) {
      const auto & item = fan_result.merged_items[index];
      // `item_peers` is built alongside `merged_items` and is the same length;
      // the bound is checked so a shorter list degrades to an unattributed item
      // rather than reading past the end.
      const std::string peer_name = index < fan_result.item_peers.size() ? fan_result.item_peers[index] : std::string{};
      if (!item.is_object()) {
        continue;
      }
      auto parsed = dto::JsonReader<T>::read(item);
      if (parsed.has_value()) {
        result.items.push_back(std::move(parsed.value()));
        result.item_peers.push_back(peer_name);
        continue;
      }
      dto::DroppedItem dropped;
      // The wire key is a peer URL and what the fan-out carries is a peer name,
      // so the two are not interchangeable. Left empty rather than filled with
      // the wrong kind of identifier.
      dropped.peer = "";
      // Best-effort source_id: scan a small set of common id keys.
      static constexpr std::array<std::string_view, 5> kIdKeys = {"id", "name", "fault_id", "data_id", "operation_id"};
      for (const auto & key : kIdKeys) {
        const std::string key_str(key);
        auto it = item.find(key_str);
        if (it != item.end() && it->is_string()) {
          dropped.source_id = it->get<std::string>();
          break;
        }
      }
      std::string reason;
      for (const auto & err : parsed.error()) {
        if (!reason.empty()) {
          reason += "; ";
        }
        reason += err.field;
        reason += ": ";
        reason += err.message;
      }
      log_peer_drop_warning(std::string(dto::dto_name<T>).c_str(), reason.c_str());
      dropped.reason = std::move(reason);
      result.dropped_items.push_back(std::move(dropped));
    }
  }
  result.partial = fan_result.is_partial;
  result.failed_peers = fan_result.failed_peers;
  return result;
}

}  // namespace ros2_medkit_gateway
