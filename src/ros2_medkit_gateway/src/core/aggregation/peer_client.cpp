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

#include "ros2_medkit_gateway/core/aggregation/peer_client.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iterator>
#include <set>
#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Vendor error code for peer unavailable (connection failure)
constexpr const char * ERR_X_MEDKIT_PEER_UNAVAILABLE = "x-medkit-peer-unavailable";

/// API prefix for SOVD endpoints
constexpr const char * API_PREFIX = "/api/v1";

/// Maximum response body size to accept from peers (10MB)
constexpr size_t MAX_PEER_RESPONSE_SIZE = 10 * 1024 * 1024;

/// Maximum number of entities per collection (areas, components, apps, functions)
/// before detail fetches are skipped to prevent excessive HTTP requests.
constexpr size_t MAX_ENTITIES_PER_COLLECTION = 1000;

/**
 * @brief Percent-encode a query parameter key or value (RFC 3986)
 *
 * Unreserved characters (A-Z, a-z, 0-9, '-', '.', '_', '~') pass through;
 * everything else is encoded as %XX. This avoids depending on
 * httplib::detail internals.
 */
std::string encode_query_param(const std::string & value) {
  std::string result;
  result.reserve(value.size());
  for (char ch : value) {
    auto c = static_cast<unsigned char>(ch);
    if (std::isalnum(c) || c == '-' || c == '.' || c == '_' || c == '~') {
      result += static_cast<char>(c);
    } else {
      static const char hex[] = "0123456789ABCDEF";
      result += '%';
      result += hex[c >> 4];
      result += hex[c & 0x0F];
    }
  }
  return result;
}

/**
 * @brief Reconstruct path with query string from httplib request
 *
 * httplib::Request::path does not include the query string; query parameters
 * are stored separately in req.params. This helper reconstructs the full
 * request target (path + "?key=val&...") so forwarded requests preserve
 * filtering/pagination parameters.
 */
std::string path_with_query(const httplib::Request & req) {
  if (req.params.empty()) {
    return req.path;
  }
  std::string result = req.path + "?";
  bool first = true;
  for (const auto & param : req.params) {
    if (!first) {
      result += "&";
    }
    result += encode_query_param(param.first);
    result += "=";
    result += encode_query_param(param.second);
    first = false;
  }
  return result;
}

/**
 * @brief Validate an entity ID for safe use in URL paths
 *
 * Rejects IDs with path traversal characters (/, ..), null bytes, or other
 * characters that could be used for SSRF or path injection. Matches the same
 * rules as HandlerContext::validate_entity_id: alphanumeric + underscore + hyphen,
 * max 256 chars.
 */
bool is_valid_entity_id(const std::string & id) {
  if (id.empty() || id.size() > 256) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
  });
}

/**
 * @brief Build a SOVD GenericError JSON body
 */
nlohmann::json make_error_body(const std::string & error_code, const std::string & message,
                               const std::string & vendor_code = "") {
  nlohmann::json body;
  if (!vendor_code.empty()) {
    body["error_code"] = ERR_VENDOR_ERROR;
    body["vendor_code"] = vendor_code;
  } else {
    body["error_code"] = error_code;
  }
  body["message"] = message;
  return body;
}

/**
 * @brief Parse an entity collection from a JSON response
 *
 * Expects the response to contain an "items" array. Each item is parsed
 * by the provided parser function.
 */
template <typename T, typename Parser>
std::vector<T> parse_collection(const nlohmann::json & response_json, Parser parser) {
  std::vector<T> result;
  if (response_json.contains("items") && response_json["items"].is_array()) {
    for (const auto & item : response_json["items"]) {
      result.push_back(parser(item));
    }
  }
  return result;
}

/**
 * @brief Parse an Area from JSON
 */
Area parse_area(const nlohmann::json & j) {
  Area area;
  area.id = j.value("id", "");
  area.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    area.namespace_path = xm.value("namespace", "");
    area.description = xm.value("description", "");
    area.source = xm.value("source", "");
  }
  if (j.contains("translationId")) {
    area.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    area.tags = j["tags"].get<std::vector<std::string>>();
  }
  return area;
}

/**
 * @brief Parse a Component from JSON
 */
Component parse_component(const nlohmann::json & j) {
  Component comp;
  comp.id = j.value("id", "");
  comp.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    comp.namespace_path = xm.value("namespace", "");
    comp.fqn = xm.value("fqn", "");
    comp.area = xm.value("area", "");
    comp.source = xm.value("source", "");
    comp.description = xm.value("description", "");
    comp.variant = xm.value("variant", "");
    comp.parent_component_id = xm.value("parentComponentId", "");
    if (xm.contains("dependsOn") && xm["dependsOn"].is_array()) {
      comp.depends_on = xm["dependsOn"].get<std::vector<std::string>>();
    }
    // Asset-identity nameplate (with per-field provenance). Emitted by peers under
    // x-medkit.identity only when populated; parse it back so identity survives
    // aggregation instead of being silently dropped.
    if (xm.contains("identity") && xm["identity"].is_object()) {
      comp.identity = AssetIdentity::from_json(xm["identity"]);
    }
    // A peer emits `available` only to say false, so absence is the peer
    // stating the entity is reachable and the default has to be true. A
    // Component carries no second signal - an App has `is_online` - so this is
    // the only way an unreachable one stays unreachable past another hop.
    comp.available = xm.value("available", true);
  }
  if (j.contains("translationId")) {
    comp.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    comp.tags = j["tags"].get<std::vector<std::string>>();
  }
  return comp;
}

/**
 * @brief Extract the component ID from an ``is-located-on`` URI.
 *
 * SOVD exposes the app-to-component binding as a standard relationship
 * URI (ISO 17978-3, §7.6). Peers may emit it as either an absolute URL
 * (``http://host:port/api/v1/components/{id}``) or a path-only reference
 * (``/api/v1/components/{id}``). Extract the trailing segment after the
 * ``/components/`` marker and validate it with ``is_valid_entity_id`` so
 * that a malformed or hostile peer URI cannot smuggle path traversal or
 * percent-encoded junk into the aggregator's entity cache.
 * Returns an empty string if the URI contains no marker or if the
 * extracted segment fails validation.
 */
std::string component_id_from_located_on(const std::string & uri) {
  static const std::string kMarker = "/components/";
  auto pos = uri.rfind(kMarker);
  if (pos == std::string::npos) {
    return "";
  }
  auto id_start = pos + kMarker.size();
  if (id_start >= uri.size()) {
    return "";
  }
  auto id_end = uri.find_first_of("/?#", id_start);
  auto candidate = uri.substr(id_start, id_end - id_start);
  return is_valid_entity_id(candidate) ? candidate : std::string{};
}

/**
 * @brief Read a peer's `/operations` collection into an App's service/action lists.
 *
 * The wire id is not used as the name: on the peer it is already qualified when
 * that peer saw a collision among its own members, and the qualifier is the
 * peer's business, not ours. `name` is always the bare short name, which is what
 * ambiguity is keyed on, and the full ROS path comes from x-medkit.ros2 so two
 * same-named operations stay distinguishable.
 */
void parse_operations_into(const nlohmann::json & j, App & app) {
  if (!j.contains("items") || !j["items"].is_array()) {
    return;
  }
  for (const auto & item : j["items"]) {
    if (!item.is_object()) {
      continue;
    }
    const std::string name = item.value("name", "");
    if (name.empty()) {
      continue;
    }
    std::string full_path;
    std::string type;
    bool is_action = item.value("asynchronous_execution", false);
    if (item.contains("x-medkit") && item["x-medkit"].is_object()) {
      const auto & xm = item["x-medkit"];
      if (xm.contains("ros2") && xm["ros2"].is_object()) {
        const auto & ros2 = xm["ros2"];
        type = ros2.value("type", "");
        full_path = ros2.value("service", "");
        if (full_path.empty()) {
          full_path = ros2.value("action", "");
          is_action = is_action || !full_path.empty();
        }
        const std::string kind = ros2.value("kind", "");
        if (!kind.empty()) {
          is_action = kind == "action";
        }
      }
    }
    if (full_path.empty()) {
      continue;
    }
    if (is_action) {
      app.actions.push_back(ActionInfo{name, full_path, type, std::nullopt});
    } else {
      app.services.push_back(ServiceInfo{name, full_path, type, std::nullopt});
    }
  }
}

/**
 * @brief Read a peer's data collection into the App's topics.
 *
 * A data item names its ROS topic by full path in ``x-medkit.ros2.topic``. An
 * item without one describes something that is not a topic - a plugin's data
 * point, for instance - and carries nothing this gateway can address as one, so
 * it is skipped rather than recorded under its wire id.
 *
 * ``direction`` decides which side of the topic the App is on, and only the
 * three values a gateway emits are accepted. An item that says anything else
 * leaves the App unattributed for that topic instead of being recorded as a
 * publisher it may not be: the ownership built out of these lists is what a
 * bare item id is dispatched by, and a guess there sends a read to a gateway
 * that does not have the topic.
 */
void parse_data_items_into(const nlohmann::json & j, App & app) {
  if (!j.contains("items") || !j["items"].is_array()) {
    return;
  }
  for (const auto & item : j["items"]) {
    if (!item.is_object() || !item.contains("x-medkit") || !item["x-medkit"].is_object()) {
      continue;
    }
    const auto & xm = item["x-medkit"];
    if (!xm.contains("ros2") || !xm["ros2"].is_object()) {
      continue;
    }
    const std::string topic = xm["ros2"].value("topic", "");
    const std::string direction = xm["ros2"].value("direction", "");
    if (topic.empty()) {
      continue;
    }
    if (direction == "publish" || direction == "both") {
      app.topics.publishes.push_back(topic);
    }
    if (direction == "subscribe" || direction == "both") {
      app.topics.subscribes.push_back(topic);
    }
  }
}

/**
 * @brief Parse an App from JSON.
 *
 * The app-to-component binding is recovered from SOVD's standard
 * ``is-located-on`` relationship (body field or ``_links`` entry),
 * which every SOVD-compliant peer emits. Vendor fallback:
 * ``x-medkit.component_id`` (gateway's own extension).
 */
App parse_app(const nlohmann::json & j) {
  App app;
  app.id = j.value("id", "");
  app.name = j.value("name", "");
  app.description = j.value("description", "");

  // SOVD-standard: is-located-on relationship (ISO 17978-3, §7.6)
  if (j.contains("is-located-on") && j["is-located-on"].is_string()) {
    app.component_id = component_id_from_located_on(j["is-located-on"].get<std::string>());
  }
  if (app.component_id.empty() && j.contains("_links") && j["_links"].is_object()) {
    const auto & links = j["_links"];
    if (links.contains("is-located-on") && links["is-located-on"].is_string()) {
      app.component_id = component_id_from_located_on(links["is-located-on"].get<std::string>());
    } else if (links.contains("is-located-on") && links["is-located-on"].is_object()) {
      // HAL+JSON object form: {"href": "/api/v1/components/ecu-a"}
      auto href = links["is-located-on"].value("href", "");
      if (!href.empty()) {
        app.component_id = component_id_from_located_on(href);
      }
    }
  }

  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    // Vendor fallback: gateway emits x-medkit.component_id (snake_case) in
    // discovery_handlers.cpp. Only used if the SOVD standard is-located-on
    // field is absent. Validated for the same reasons as
    // component_id_from_located_on - the value is peer-provided.
    if (app.component_id.empty()) {
      auto candidate = xm.value("component_id", "");
      if (is_valid_entity_id(candidate)) {
        app.component_id = candidate;
      }
    }
    app.source = xm.value("source", "");
    app.is_online = xm.value("is_online", false);
    // Emitted only to say false, so absence means reachable. Read back for the
    // same reason `is_online` is: a leaf that went quiet several hops away is
    // described by the gateway that still holds its declaration, and that
    // description is the only account of it this gateway can get.
    app.available = xm.value("available", true);
    if (app.description.empty()) {
      app.description = xm.value("description", "");
    }
  }
  if (j.contains("translationId")) {
    app.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    app.tags = j["tags"].get<std::vector<std::string>>();
  }
  return app;
}

/**
 * @brief Parse a Function from JSON
 */
Function parse_function(const nlohmann::json & j) {
  Function func;
  func.id = j.value("id", "");
  func.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    func.source = xm.value("source", "");
    func.description = xm.value("description", "");
    if (xm.contains("hosts") && xm["hosts"].is_array()) {
      func.hosts = xm["hosts"].get<std::vector<std::string>>();
    }
  }
  if (j.contains("translationId")) {
    func.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    func.tags = j["tags"].get<std::vector<std::string>>();
  }
  return func;
}

/**
 * @brief What a non-200 on a sub-request means, which depends on the route.
 */
enum class RouteKind {
  /// A top-level collection: ``/areas``, ``/components``, ``/apps``,
  /// ``/functions``. Every peer serves these, so nothing but 200 describes one.
  kCollection,
  /// A nested collection: ``/areas/{id}/subareas``, ``/components/{id}/subcomponents``,
  /// ``/apps/{id}/operations``. A gateway old enough not to have the route answers
  /// 404, and aggregation has to keep working across that version boundary, so a
  /// 404 here means "not offered" rather than "could not be read".
  ///
  /// It also carries ``504 not-responding``, and for the same reason the detail
  /// of an addressable entity does: the route hangs off an entity, and a peer
  /// holding a declaration for a gateway that went quiet answers 504 on every
  /// route of that entity, this one included. Read as a hole in the picture it
  /// would abort the whole fetch, and one unreachable member anywhere behind a
  /// peer would freeze this gateway's view of everything that peer holds.
  kNestedCollection,
  /// The detail of an entity that carries availability of its own (a Component).
  /// ``504 not-responding`` is the peer describing that entity as unreachable -
  /// a statement about the entity, which an aggregating peer makes whenever it
  /// is itself holding a declaration for a gateway that went quiet. It is
  /// carried, not read as a hole in the picture.
  kAddressableDetail,
  /// The detail of a grouping entity (a Function). It has no availability of its
  /// own to carry, and its members are named nowhere else, so nothing but 200
  /// describes it.
  kGroupingDetail,
};

/**
 * @brief What one sub-request issued while describing a peer produced.
 */
struct SubResponse {
  enum class Kind {
    kBody,               ///< `body` holds the parsed response
    kRouteAbsent,        ///< the peer does not offer this route; carry on without it
    kEntityUnreachable,  ///< the peer named this entity and says it cannot be reached
    kIncomplete,         ///< part of the picture could not be read; `error` says why
  };

  Kind kind{Kind::kIncomplete};
  nlohmann::json body;
  std::string error;
};

/**
 * @brief True for a SOVD error body carrying the ``not-responding`` code.
 */
bool says_not_responding(const std::string & body) {
  if (body.size() > MAX_PEER_RESPONSE_SIZE) {
    return false;
  }
  auto parsed = nlohmann::json::parse(body, nullptr, false);
  return !parsed.is_discarded() && parsed.is_object() && parsed.value("error_code", "") == ERR_NOT_RESPONDING;
}

/**
 * @brief Classify one sub-request of a peer fetch.
 *
 * @param result httplib result for the call
 * @param peer_name Peer the call was made against, for the error text
 * @param path Route that was called, for the error text
 * @param kind What a non-200 means on this route
 */
SubResponse read_sub_response(const httplib::Result & result, const std::string & peer_name, const std::string & path,
                              RouteKind kind) {
  SubResponse out;
  if (!result) {
    out.error = "Failed to connect to peer '" + peer_name + "' for " + path;
    return out;
  }
  if (result->status == 404 && kind == RouteKind::kNestedCollection) {
    out.kind = SubResponse::Kind::kRouteAbsent;
    return out;
  }
  if (result->status == 504 && says_not_responding(result->body) &&
      (kind == RouteKind::kAddressableDetail || kind == RouteKind::kNestedCollection)) {
    out.kind = SubResponse::Kind::kEntityUnreachable;
    return out;
  }
  if (result->status != 200) {
    out.error = "Peer '" + peer_name + "' returned status " + std::to_string(result->status) + " for " + path;
    return out;
  }
  if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
    out.error = "Response from peer '" + peer_name + "' for " + path + " exceeds size limit";
    return out;
  }
  auto parsed = nlohmann::json::parse(result->body, nullptr, false);
  if (parsed.is_discarded()) {
    out.error = "Invalid JSON from peer '" + peer_name + "' for " + path;
    return out;
  }
  out.kind = SubResponse::Kind::kBody;
  out.body = std::move(parsed);
  return out;
}

}  // namespace

namespace {

/// Name and value of the budget an error was measured against, for a message a
/// client can act on. The READ budget differs by call: a forward spends
/// `aggregation.forward_timeout_ms`, a fan-out spends `aggregation.timeout_ms`,
/// so the caller names its own rather than have this guess.
std::string describe_budget(PeerBudget budget, const PeerTimeouts & timeouts, const char * read_key, int read_ms) {
  switch (budget) {
    case PeerBudget::kConnect:
      return "aggregation.timeout_ms (" + std::to_string(timeouts.connect_ms) + "ms, connect)";
    case PeerBudget::kRead:
      return std::string(read_key) + " (" + std::to_string(read_ms) + "ms, read)";
    case PeerBudget::kWrite:
      return "aggregation.write_timeout_ms (" + std::to_string(timeouts.write_ms) + "ms, write)";
    case PeerBudget::kNone:
      break;
  }
  return "its budget";
}

}  // namespace

PeerBudget peer_budget_for(httplib::Error error) {
  switch (error) {
    case httplib::Error::ConnectionTimeout:
      return PeerBudget::kConnect;
    case httplib::Error::Read:
      return PeerBudget::kRead;
    case httplib::Error::Write:
      return PeerBudget::kWrite;
    case httplib::Error::Success:
    case httplib::Error::Unknown:
    case httplib::Error::Connection:
    case httplib::Error::BindIPAddress:
    case httplib::Error::ExceedRedirectCount:
    case httplib::Error::Canceled:
    case httplib::Error::SSLConnection:
    case httplib::Error::SSLLoadingCerts:
    case httplib::Error::SSLServerVerification:
    case httplib::Error::UnsupportedMultipartBoundaryChars:
    case httplib::Error::Compression:
    case httplib::Error::ProxyConnection:
    case httplib::Error::SSLPeerCouldBeClosed_:
      return PeerBudget::kNone;
  }
  return PeerBudget::kNone;
}

PeerFailureKind classify_peer_error(httplib::Error error, std::chrono::milliseconds elapsed,
                                    std::chrono::milliseconds read_budget, std::chrono::milliseconds write_budget) {
  // Every enumerator listed rather than a default, because -Wswitch-enum is an
  // error here: a cpp-httplib upgrade that adds a failure mode has to be
  // classified deliberately instead of falling into "unreachable" unnoticed.
  switch (error) {
    case httplib::Error::ConnectionTimeout:
      return PeerFailureKind::kTimeout;
    case httplib::Error::Canceled:
      return PeerFailureKind::kCanceled;
    case httplib::Error::Read:
      // The budget is always set on these clients, so a call that consumed it
      // ran out of time. One that came back sooner broke for another reason -
      // most often the peer process dying with the answer half sent - and
      // reporting that as a timeout would tell an operator to wait for a peer
      // that is gone.
      //
      // `elapsed` covers connect plus read, because cpp-httplib reports no
      // separate connect-completion time and its socket-options hook runs
      // BEFORE the connect. A slow handshake therefore counts towards the read
      // budget. The connect budget is capped well below the read budgets (see
      // AggregationConfig::peer_timeouts) so that window is bounded by the cap
      // rather than by the whole metadata budget, and a handshake slower than
      // the cap fails as ConnectionTimeout instead, which is attributed
      // correctly. Removing the window entirely needs the vendored client to
      // expose a post-connect callback.
      return elapsed >= read_budget ? PeerFailureKind::kTimeout : PeerFailureKind::kUnreachable;
    case httplib::Error::Write:
      // Against the WRITE budget, which is its own and smaller than the read
      // budget by default. Measured against the read budget, a write that ran
      // out at 5 s inside a 15 s forward budget looks like a peer that died,
      // and the one key whose whole purpose is bounding a slow upload could
      // never report its own expiry.
      return elapsed >= write_budget ? PeerFailureKind::kTimeout : PeerFailureKind::kUnreachable;
    case httplib::Error::Success:
      // Not reachable through the callers, which classify only a null result,
      // but the enumerator exists and "no failure" is the only honest answer.
      return PeerFailureKind::kNone;
    case httplib::Error::Unknown:
    case httplib::Error::Connection:
    case httplib::Error::BindIPAddress:
    case httplib::Error::ExceedRedirectCount:
    case httplib::Error::SSLConnection:
    case httplib::Error::SSLLoadingCerts:
    case httplib::Error::SSLServerVerification:
    case httplib::Error::UnsupportedMultipartBoundaryChars:
    case httplib::Error::Compression:
    case httplib::Error::ProxyConnection:
    case httplib::Error::SSLPeerCouldBeClosed_:
      return PeerFailureKind::kUnreachable;
  }
  return PeerFailureKind::kUnreachable;
}

const char * peer_failure_reason(PeerFailureKind kind) {
  switch (kind) {
    case PeerFailureKind::kNone:
      return "none";
    case PeerFailureKind::kTimeout:
      return "timeout";
    case PeerFailureKind::kUnreachable:
      return "unreachable";
    case PeerFailureKind::kCanceled:
      return "canceled";
    case PeerFailureKind::kErrorStatus:
      return "error-status";
    case PeerFailureKind::kTooLarge:
      return "too-large";
    case PeerFailureKind::kInvalidResponse:
      return "invalid-response";
  }
  return "unreachable";
}

PeerClient::PeerClient(const std::string & url, const std::string & name, PeerTimeouts timeouts, bool forward_auth)
  : url_(url), name_(name), timeouts_(timeouts), forward_auth_(forward_auth) {
}

PeerClient::PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth)
  : PeerClient(url, name, PeerTimeouts::uniform(timeout_ms), forward_auth) {
}

const std::string & PeerClient::url() const {
  return url_;
}

const std::string & PeerClient::name() const {
  return name_;
}

bool PeerClient::is_healthy() const {
  return healthy_.load();
}

void PeerClient::ensure_client() {
  if (!client_) {
    client_ = std::make_unique<httplib::Client>(url_);
    // Cap the health-check timeouts at 1s regardless of the configured
    // forward timeout. Reasoning:
    //   * cpp-httplib's Client::stop() called concurrently with an
    //     in-flight Get() / Post() unlocks an internal mutex from a
    //     thread that did not lock it; glibc + pthread debug mode
    //     (Ubuntu Noble) treats that as fatal mutex misuse and aborts
    //     with SIGABRT (TestShutdown.test_exit_codes saw exit code -6).
    //     So we cannot use Client::stop() to interrupt the health check.
    //   * Without an interrupt, the worst-case shutdown delay equals
    //     the health-check read timeout. A 5s nominal timeout becomes
    //     ~25s under TSan, exceeding the launch_test SIGINT->SIGKILL
    //     grace and producing an exit code -9.
    //   * Health checks should be fast by their nature; an unresponsive
    //     peer is "unhealthy" whether we wait 1s or 5s. Capping at 1s
    //     bounds shutdown delay to ~5s under TSan, well within the 15s
    //     grace window, without affecting forward semantics (which still
    //     use the configured timeout via ScopedClient).
    int health_timeout_ms = std::min(timeouts_.metadata_read_ms, 1000);
    client_->set_connection_timeout(std::min(timeouts_.connect_ms, health_timeout_ms) / 1000,
                                    (std::min(timeouts_.connect_ms, health_timeout_ms) % 1000) * 1000);
    client_->set_read_timeout(health_timeout_ms / 1000, (health_timeout_ms % 1000) * 1000);
    client_->set_write_timeout(timeouts_.write_ms / 1000, (timeouts_.write_ms % 1000) * 1000);
    // Note: cpp-httplib Client does not expose set_payload_max_length (server-only).
    // Response size is enforced post-download in forward_request() and
    // forward_and_get_json() via MAX_PEER_RESPONSE_SIZE body length checks.
    // The read timeout provides a secondary defense against slow-drip attacks.
  }
}

void PeerClient::check_health() {
  std::lock_guard<std::mutex> lock(client_mutex_);
  ensure_client();
  auto result = client_->Get(std::string(API_PREFIX) + "/health");
  healthy_.store(result && result->status == 200);
}

tl::expected<PeerEntities, std::string> PeerClient::fetch_entities() {
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return tl::unexpected<std::string>("Peer '" + name_ + "': fetch_entities skipped, shutdown in progress");
  }
  // Use a dedicated client for this long-running operation (4 sequential HTTP
  // requests, up to 8s with 2s timeout) to avoid blocking health checks and
  // forwarding on the shared client_mutex_. ScopedClient registers with the
  // active-client registry so shutdown() can stop() the in-flight call.
  ScopedClient scoped(*this, url_, timeouts_.metadata_read_ms);
  auto & cli = *scoped;

  PeerEntities entities;
  const std::string peer_source = "peer:" + name_;

  // A route the peer does not offer is a property of the peer, not of the
  // entity that happened to hit it first, so it is recorded once however many
  // entities ask for it.
  auto note_absent_route = [&entities](const std::string & route) {
    if (std::find(entities.absent_routes.begin(), entities.absent_routes.end(), route) ==
        entities.absent_routes.end()) {
      entities.absent_routes.push_back(route);
    }
  };

  // Fetch areas
  {
    auto response =
        read_sub_response(cli.Get(std::string(API_PREFIX) + "/areas"), name_, "/areas", RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    entities.areas = parse_collection<Area>(response.body, parse_area);
    // Validate entity IDs and enforce per-collection limit
    entities.areas.erase(std::remove_if(entities.areas.begin(), entities.areas.end(),
                                        [](const Area & a) {
                                          return !is_valid_entity_id(a.id);
                                        }),
                         entities.areas.end());
    if (entities.areas.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(entities.areas.size()) +
                                         " areas (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & area : entities.areas) {
      area.declared_source = area.source;
      area.source = peer_source;
    }

    // Fetch subareas for each top-level area (list endpoint filters them out).
    // Collect into a separate vector first to avoid push_back during iteration
    // (which can invalidate references if the vector reallocates).
    std::vector<Area> all_subareas;
    for (const auto & area : entities.areas) {
      const std::string route = "/areas/" + area.id + "/subareas";
      auto sub =
          read_sub_response(cli.Get(std::string(API_PREFIX) + route), name_, route, RouteKind::kNestedCollection);
      if (sub.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(sub.error);
      }
      if (sub.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/areas/{id}/subareas");
        continue;
      }
      if (sub.kind == SubResponse::Kind::kEntityUnreachable) {
        // The peer holds this Area's id but the gateway contributing it is
        // silent, so its members are out of reach. That is one Area's worth of
        // detail missing, not a failed read of the peer.
        continue;
      }
      auto subareas = parse_collection<Area>(sub.body, parse_area);
      for (auto & subarea : subareas) {
        if (!is_valid_entity_id(subarea.id)) {
          continue;
        }
        subarea.declared_source = subarea.source;
        subarea.source = peer_source;
        all_subareas.push_back(std::move(subarea));
      }
    }
    entities.areas.insert(entities.areas.end(), std::make_move_iterator(all_subareas.begin()),
                          std::make_move_iterator(all_subareas.end()));
  }

  // Fetch components (list then detail per entity for full relationship data)
  {
    auto response = read_sub_response(cli.Get(std::string(API_PREFIX) + "/components"), name_, "/components",
                                      RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    // Parse IDs from list, then fetch detail per entity for relationships
    auto comp_list = parse_collection<Component>(response.body, parse_component);
    // Validate entity IDs and enforce per-collection limit
    comp_list.erase(std::remove_if(comp_list.begin(), comp_list.end(),
                                   [](const Component & c) {
                                     return !is_valid_entity_id(c.id);
                                   }),
                    comp_list.end());
    if (comp_list.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(comp_list.size()) +
                                         " components (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    // The detail response carries the relationships (parent, dependencies,
    // identity) the list omits, so a Component built from the list alone is a
    // Component asserted to have none.
    for (auto & comp : comp_list) {
      const std::string route = "/components/" + comp.id;
      auto detail =
          read_sub_response(cli.Get(std::string(API_PREFIX) + route), name_, route, RouteKind::kAddressableDetail);
      if (detail.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(detail.error);
      }
      if (detail.kind == SubResponse::Kind::kBody) {
        comp = parse_component(detail.body);
      } else {
        // The peer holds this Component's id but cannot describe it: whoever
        // contributes it has gone quiet. What the list gave is everything the
        // peer can still say, and the availability is what it is saying.
        comp.available = false;
      }
      comp.declared_source = comp.source;
      comp.source = peer_source;
    }
    // Fetch subcomponents for each top-level component (list endpoint filters them out).
    // Collect into a separate vector first to avoid push_back during iteration
    // (which can invalidate references if the vector reallocates).
    std::vector<Component> all_subcomps;
    for (const auto & comp : comp_list) {
      const std::string route = "/components/" + comp.id + "/subcomponents";
      auto sub =
          read_sub_response(cli.Get(std::string(API_PREFIX) + route), name_, route, RouteKind::kNestedCollection);
      if (sub.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(sub.error);
      }
      if (sub.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/components/{id}/subcomponents");
        continue;
      }
      if (sub.kind == SubResponse::Kind::kEntityUnreachable) {
        // Same as for subareas: the parent is retained and unreachable, so what
        // it contains cannot be read. The parent itself already carries that.
        continue;
      }
      auto subcomps = parse_collection<Component>(sub.body, parse_component);
      for (auto & subcomp : subcomps) {
        if (!is_valid_entity_id(subcomp.id)) {
          continue;
        }
        // Fetch detail for each subcomponent to get full relationships
        const std::string detail_route = "/components/" + subcomp.id;
        auto detail = read_sub_response(cli.Get(std::string(API_PREFIX) + detail_route), name_, detail_route,
                                        RouteKind::kAddressableDetail);
        if (detail.kind == SubResponse::Kind::kIncomplete) {
          return tl::unexpected<std::string>(detail.error);
        }
        if (detail.kind == SubResponse::Kind::kBody) {
          subcomp = parse_component(detail.body);
        } else {
          subcomp.available = false;
          // The route this id came from is itself the statement that `comp` is
          // its parent, so the tree keeps its shape even though the entity's
          // own description is out of reach.
          subcomp.parent_component_id = comp.id;
        }
        subcomp.declared_source = subcomp.source;
        subcomp.source = peer_source;
        all_subcomps.push_back(std::move(subcomp));
      }
    }
    comp_list.insert(comp_list.end(), std::make_move_iterator(all_subcomps.begin()),
                     std::make_move_iterator(all_subcomps.end()));

    entities.components = std::move(comp_list);
  }

  // Fetch apps
  {
    auto response =
        read_sub_response(cli.Get(std::string(API_PREFIX) + "/apps"), name_, "/apps", RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    entities.apps = parse_collection<App>(response.body, parse_app);
    // Validate entity IDs and enforce per-collection limit
    entities.apps.erase(std::remove_if(entities.apps.begin(), entities.apps.end(),
                                       [](const App & a) {
                                         return !is_valid_entity_id(a.id);
                                       }),
                        entities.apps.end());
    if (entities.apps.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(entities.apps.size()) +
                                         " apps (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & app : entities.apps) {
      app.declared_source = app.source;
      app.source = peer_source;
    }
    // Filter ROS 2 internal nodes (underscore prefix convention) at source.
    // These are noise nodes like _param_client_node that should never appear
    // as SOVD entities.
    entities.apps.erase(std::remove_if(entities.apps.begin(), entities.apps.end(),
                                       [](const App & app) {
                                         return !app.id.empty() && app.id[0] == '_';
                                       }),
                        entities.apps.end());

    // Fetch each app's operations and data. Neither is ever declared in a
    // manifest - both are discovered from the ROS graph - so the only record of
    // what a peer's app exposes is what the peer reports. Without the
    // operations the aggregator cannot tell that two members share an operation
    // short name except by asking at request time, and an answer that depends
    // on who is reachable is not an answer a client can rely on. Without the
    // topics it holds no record of which member owns one, so an item id that
    // names no member - the form a topic with a single provider is listed under
    // - cannot be routed to the gateway that has the topic.
    //
    // `X-Medkit-No-Fan-Out` keeps the peer from re-asking ITS peers: each
    // gateway reports what it holds, and the hop that owns the entity is the
    // hop that answers for it. It is also what makes this terminate.
    //
    // A route that is absent or answers for an unreachable entity is skipped on
    // its own, never for the app: the two collections are read independently
    // and one missing must not cost the other.
    for (auto & app : entities.apps) {
      const httplib::Headers no_fan_out{{"X-Medkit-No-Fan-Out", "1"}};

      const std::string ops_route = "/apps/" + app.id + "/operations";
      auto ops = read_sub_response(cli.Get(std::string(API_PREFIX) + ops_route, no_fan_out), name_, ops_route,
                                   RouteKind::kNestedCollection);
      if (ops.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(ops.error);
      }
      if (ops.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/apps/{id}/operations");
      } else if (ops.kind == SubResponse::Kind::kBody) {
        parse_operations_into(ops.body, app);
      }
      // kEntityUnreachable: the App is retained and its gateway is silent, so
      // the peer answers for it rather than proxying. It keeps what the peer
      // already reported; there is nothing further to read.

      const std::string data_route = "/apps/" + app.id + "/data";
      auto data = read_sub_response(cli.Get(std::string(API_PREFIX) + data_route, no_fan_out), name_, data_route,
                                    RouteKind::kNestedCollection);
      if (data.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(data.error);
      }
      if (data.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/apps/{id}/data");
      } else if (data.kind == SubResponse::Kind::kBody) {
        parse_data_items_into(data.body, app);
      }
    }
  }

  // Fetch functions (list then detail per entity for hosts data)
  {
    auto response =
        read_sub_response(cli.Get(std::string(API_PREFIX) + "/functions"), name_, "/functions", RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    // Parse IDs from list, then fetch detail per entity for hosts
    auto func_list = parse_collection<Function>(response.body, parse_function);
    // Validate entity IDs and enforce per-collection limit
    func_list.erase(std::remove_if(func_list.begin(), func_list.end(),
                                   [](const Function & f) {
                                     return !is_valid_entity_id(f.id);
                                   }),
                    func_list.end());
    if (func_list.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(func_list.size()) +
                                         " functions (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    // A Function's hosts live only in its detail response: the list carries
    // none. A Function built from the list alone is a Function asserted to
    // group nothing, which is a statement the peer never made.
    for (auto & func : func_list) {
      const std::string route = "/functions/" + func.id;
      auto detail =
          read_sub_response(cli.Get(std::string(API_PREFIX) + route), name_, route, RouteKind::kGroupingDetail);
      if (detail.kind != SubResponse::Kind::kBody) {
        return tl::unexpected<std::string>(detail.error);
      }
      func = parse_function(detail.body);
      func.declared_source = func.source;
      func.source = peer_source;
    }
    entities.functions = std::move(func_list);
  }

  return entities;
}

void PeerClient::forward_request(const httplib::Request & req, httplib::Response & res) {
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    res.status = 503;
    auto error_body =
        make_error_body(ERR_VENDOR_ERROR, "Gateway shutting down; peer forward refused", ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }
  // Create a dedicated client per forwarding call to avoid holding client_mutex_
  // during potentially long I/O operations. The shared client_ is reserved for
  // short health checks only. ScopedClient registers with the active-client
  // registry so shutdown() can stop() the in-flight call.
  //
  // The forward budget, not the metadata one: what comes back is whatever the
  // peer had to do to answer, which for a synchronous operation is that
  // gateway's own service budget and for a large resource is the transfer.
  ScopedClient scoped(*this, url_, timeouts_.forward_read_ms);
  auto & cli = *scoped;

  httplib::Headers headers;
  // Forward Authorization header only when explicitly enabled (forward_auth).
  // Default is off to prevent token leakage to untrusted/mDNS-discovered peers.
  if (forward_auth_ && req.has_header("Authorization")) {
    headers.emplace("Authorization", req.get_header_value("Authorization"));
  }
  // Propagate fan-out suppression header to prevent recursive loops when
  // a forwarded request bounces back to the origin via bidirectional peering.
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    headers.emplace("X-Medkit-No-Fan-Out", "1");
  }
  // The client's identity, which is what a lock is held against. The peer owns
  // the entity, so the peer holds the lock and judges every request against the
  // name it recorded; a request that arrives anonymous is a different caller
  // than the one that took the lock, whoever sent it. It is not a credential -
  // authority travels in Authorization, which is forwarded only when the
  // deployment says the peer is trusted with it.
  if (req.has_header("X-Client-Id")) {
    headers.emplace("X-Client-Id", req.get_header_value("X-Client-Id"));
  }

  httplib::Result result{nullptr, httplib::Error::Unknown};
  const std::string path = path_with_query(req);
  const std::string content_type = req.get_header_value("Content-Type");

  const auto started_at = std::chrono::steady_clock::now();

  if (req.method == "GET") {
    result = cli.Get(path, headers);
  } else if (req.method == "POST") {
    result = cli.Post(path, headers, req.body, content_type);
  } else if (req.method == "PUT") {
    result = cli.Put(path, headers, req.body, content_type);
  } else if (req.method == "DELETE") {
    result = cli.Delete(path, headers);
  } else if (req.method == "PATCH") {
    result = cli.Patch(path, headers, req.body, content_type);
  }

  if (!result) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at);
    const auto kind = classify_peer_error(result.error(), elapsed, std::chrono::milliseconds(timeouts_.forward_read_ms),
                                          std::chrono::milliseconds(timeouts_.write_ms));
    if (kind == PeerFailureKind::kTimeout) {
      // The peer is answering someone; this gateway stopped waiting. Saying it
      // is unavailable describes a different system state and points an
      // operator at the wrong box.
      //
      // The message names the budget that actually expired. A connect timeout
      // is bounded by aggregation.timeout_ms, not by the forward budget the
      // call was hoping to spend, and telling an operator to raise the forward
      // budget would not move a peer that never accepted the connection.
      const auto expired = describe_budget(peer_budget_for(result.error()), timeouts_, "aggregation.forward_timeout_ms",
                                           timeouts_.forward_read_ms);
      res.status = 504;
      auto error_body =
          make_error_body(ERR_NOT_RESPONDING, "Peer '" + name_ + "' at " + url_ + " did not answer within " + expired +
                                                  ". The request may still be running there.");
      res.set_content(error_body.dump(), "application/json");
      return;
    }
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR,
                                      "Peer '" + name_ + "' at " + url_ + " is unavailable (" +
                                          std::string(peer_failure_reason(kind)) + ")",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR, "Response from peer '" + name_ + "' exceeds size limit",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Copy response from peer - only forward safe headers.
  // The x-medkit header allowlist must match headers the gateway actually produces.
  // Currently the only x-medkit HTTP header is X-Medkit-Local-Only (fault_handlers.cpp).
  // Update this list when adding new x-medkit HTTP response headers.
  //
  // `location` is what a 201 or 202 hands the client to address the resource it
  // just created. The peer builds it from the request it received, so it names
  // the member's own route - a path this gateway resolves to the same member,
  // and the only address of a resource that lives on the other side. Dropping it
  // leaves the client a status with nothing to follow.
  static const std::set<std::string> allowed_headers = {"content-type",  "etag",     "cache-control",
                                                        "last-modified", "location", "x-medkit-local-only"};

  res.status = result->status;
  res.body = result->body;
  for (const auto & header : result->headers) {
    std::string lower_name = header.first;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), [](unsigned char c) {
      return std::tolower(c);
    });
    if (allowed_headers.count(lower_name) > 0) {
      res.set_header(header.first, header.second);
    }
  }
}

tl::expected<nlohmann::json, PeerError> PeerClient::forward_and_get_json(const std::string & method,
                                                                         const std::string & path,
                                                                         const std::string & auth_header,
                                                                         const httplib::Headers & extra_headers) {
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return tl::unexpected(PeerError{PeerFailureKind::kCanceled,
                                    "Peer '" + name_ + "': forward_and_get_json skipped, shutdown in progress"});
  }
  // Create a dedicated client per call to avoid holding client_mutex_ during I/O.
  // The shared client_ is reserved for short health checks only. ScopedClient
  // registers with the active-client registry so shutdown() can stop() the
  // in-flight call.
  //
  // The metadata budget: a fan-out runs against every peer at once with a
  // client waiting on the slowest of them, so this is bounded by what a
  // listing is worth, not by what one peer's work might cost.
  ScopedClient scoped(*this, url_, timeouts_.metadata_read_ms);
  auto & cli = *scoped;

  httplib::Headers headers;
  // Only forward auth header when forward_auth is enabled
  if (forward_auth_ && !auth_header.empty()) {
    headers.emplace("Authorization", auth_header);
  }
  for (const auto & [key, value] : extra_headers) {
    headers.emplace(key, value);
  }

  // Accumulate body with streaming size enforcement via ContentReceiver.
  // This prevents a malicious peer from pushing hundreds of MB before the
  // post-download check. The receiver returns false to abort the download
  // as soon as MAX_PEER_RESPONSE_SIZE is exceeded.
  std::string accumulated_body;
  bool size_exceeded = false;
  int response_status = 0;

  auto content_receiver = [&](const char * data, size_t data_length) -> bool {
    if (accumulated_body.size() + data_length > MAX_PEER_RESPONSE_SIZE) {
      size_exceeded = true;
      return false;  // Abort download
    }
    accumulated_body.append(data, data_length);
    return true;
  };

  httplib::Result result{nullptr, httplib::Error::Unknown};

  const auto started_at = std::chrono::steady_clock::now();

  if (method == "GET") {
    result = cli.Get(
        path, headers,
        [&response_status](const httplib::Response & resp) -> bool {
          response_status = resp.status;
          return true;  // Continue to receive body
        },
        content_receiver);
  } else if (method == "POST") {
    result = cli.Post(path, headers, "", "application/json");
  } else if (method == "PUT") {
    result = cli.Put(path, headers, "", "application/json");
  } else if (method == "DELETE") {
    result = cli.Delete(path, headers);
  }

  // For non-GET methods, fall back to post-download size check since
  // cpp-httplib does not offer ContentReceiver overloads for all methods.
  bool used_streaming = (method == "GET");

  // The size guard aborts the download from inside the content receiver, which
  // cpp-httplib surfaces as a cancelled read. Checked before the transport
  // error so an oversized body is reported as one rather than as a broken
  // connection.
  auto transport_failure = [&]() {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at);
    const auto kind =
        classify_peer_error(result.error(), elapsed, std::chrono::milliseconds(timeouts_.metadata_read_ms),
                            std::chrono::milliseconds(timeouts_.write_ms));
    std::string message = kind == PeerFailureKind::kTimeout
                              ? "Peer '" + name_ + "' at " + url_ + " did not answer " + method + " " + path +
                                    " within " +
                                    describe_budget(peer_budget_for(result.error()), timeouts_,
                                                    "aggregation.timeout_ms", timeouts_.metadata_read_ms)
                              : "Failed to reach peer '" + name_ + "' at " + url_ + " for " + method + " " + path;
    return PeerError{kind, std::move(message)};
  };

  if (used_streaming) {
    if (size_exceeded) {
      return tl::unexpected(
          PeerError{PeerFailureKind::kTooLarge,
                    "Response from peer '" + name_ + "' exceeds size limit for " + method + " " + path});
    }
    if (!result) {
      return tl::unexpected(transport_failure());
    }
    if (response_status < 200 || response_status >= 300) {
      return tl::unexpected(PeerError{PeerFailureKind::kErrorStatus, "Peer '" + name_ + "' returned status " +
                                                                         std::to_string(response_status) + " for " +
                                                                         method + " " + path});
    }
  } else {
    if (!result) {
      return tl::unexpected(transport_failure());
    }
    if (result->status < 200 || result->status >= 300) {
      return tl::unexpected(PeerError{PeerFailureKind::kErrorStatus, "Peer '" + name_ + "' returned status " +
                                                                         std::to_string(result->status) + " for " +
                                                                         method + " " + path});
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected(
          PeerError{PeerFailureKind::kTooLarge,
                    "Response from peer '" + name_ + "' exceeds size limit for " + method + " " + path});
    }
    accumulated_body = std::move(result->body);
  }

  auto parsed = nlohmann::json::parse(accumulated_body, nullptr, false);
  if (parsed.is_discarded()) {
    return tl::unexpected(PeerError{PeerFailureKind::kInvalidResponse,
                                    "Invalid JSON response from peer '" + name_ + "' for " + method + " " + path});
  }

  return parsed;
}

void PeerClient::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  // Snapshot active clients under the registry mutex so we do not hold it
  // while calling stop() (each stop() may block momentarily on the socket
  // close). Worker threads remove themselves on unwind.
  //
  // We deliberately do NOT stop the lazily-created health-check client_
  // here. That client is exclusively used inside check_health() which
  // holds client_mutex_ for the duration of a single short Get();
  // calling stop() on it from another thread while check_health holds
  // the same socket internals races with cpp-httplib's own socket_mutex_
  // and TSan flags it as "unlock of an unlocked mutex (or by a wrong
  // thread)". The health check is short-lived (single GET, short
  // timeout) so blocking on it is bounded; the multi-second hang we
  // are fixing came from in-flight forward_request / fetch_entities
  // calls, which use ScopedClient and ARE in to_stop.
  std::vector<httplib::Client *> to_stop;
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    to_stop.reserve(active_clients_.size());
    for (auto * cli : active_clients_) {
      to_stop.push_back(cli);
    }
  }
  for (auto * cli : to_stop) {
    cli->stop();
  }
}

void PeerClient::register_active(httplib::Client * cli) {
  std::lock_guard<std::mutex> lock(active_mutex_);
  active_clients_.insert(cli);
}

void PeerClient::unregister_active(httplib::Client * cli) {
  std::lock_guard<std::mutex> lock(active_mutex_);
  active_clients_.erase(cli);
}

PeerClient::ScopedClient::ScopedClient(PeerClient & owner, const std::string & url, int read_timeout_ms)
  : owner_(owner), cli_(url) {
  const int connect_ms = owner.timeouts_.connect_ms;
  const int write_ms = owner.timeouts_.write_ms;
  cli_.set_connection_timeout(connect_ms / 1000, (connect_ms % 1000) * 1000);
  cli_.set_read_timeout(read_timeout_ms / 1000, (read_timeout_ms % 1000) * 1000);
  // Without this the write budget is cpp-httplib's own 5 s default and follows
  // no configured value at all, so a large request body to a slow peer is cut
  // off however high the read budget is set.
  cli_.set_write_timeout(write_ms / 1000, (write_ms % 1000) * 1000);
  owner_.register_active(&cli_);
  // If shutdown landed between the timeout setup and registration, pre-stop
  // the client so the first Get/Post returns Error::Canceled immediately.
  if (owner_.shutdown_requested_.load(std::memory_order_acquire)) {
    cli_.stop();
  }
}

PeerClient::ScopedClient::~ScopedClient() {
  owner_.unregister_active(&cli_);
}

}  // namespace ros2_medkit_gateway
