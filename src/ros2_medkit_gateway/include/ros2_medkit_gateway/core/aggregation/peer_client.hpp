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

#include <httplib.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Collection of entities fetched from a peer gateway
 */
struct PeerEntities {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;

  /// Nested collection routes this peer does not offer, as route templates
  /// (e.g. ``/components/{id}/subcomponents``), each recorded once.
  ///
  /// A peer running an older gateway answers 404 for a route that did not
  /// exist yet. That is a version boundary, not a read failure, so the fetch
  /// carries on without those members and names the routes here for the
  /// caller to report.
  std::vector<std::string> absent_routes;
};

/**
 * @brief HTTP client for communicating with a peer gateway instance
 *
 * PeerClient wraps cpp-httplib to provide typed access to a peer gateway's
 * REST API. It supports health checking, entity fetching, transparent request
 * forwarding (proxy), and JSON fan-out queries.
 *
 * Thread safety: The healthy_ flag is atomic. Client creation is lazy and
 * guarded by a mutex. All public methods are safe to call from any thread.
 */
class PeerClient {
 public:
  /**
   * @brief Construct a PeerClient for a peer gateway
   * @param url Base URL of the peer (e.g., "http://localhost:8081")
   * @param name Human-readable peer name (e.g., "subsystem_b")
   * @param timeout_ms Connection and read timeout in milliseconds
   * @param forward_auth Whether to forward Authorization headers to this peer
   */
  PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth = false);

  /// Get the peer base URL
  const std::string & url() const;

  /// Get the peer name
  const std::string & name() const;

  /// Check if the peer was healthy at last health check
  bool is_healthy() const;

  /**
   * @brief Perform a health check against the peer
   *
   * GETs /api/v1/health on the peer. Sets the internal healthy_ flag
   * based on whether a 200 response was received.
   */
  void check_health();

  /**
   * @brief Fetch all entity collections from the peer
   *
   * GETs /api/v1/areas, /api/v1/components, /api/v1/apps, /api/v1/functions
   * and parses the items[] arrays, then the nested routes that carry structure
   * the lists omit: subareas, subcomponents, per-entity detail (the only source
   * of a Component's relationships and a Function's hosts) and per-app
   * operations. Each entity's source is set to "peer:<name>".
   *
   * The result describes the peer or it does not. Any sub-request that could
   * not be read - a dead connection, a non-200 the route has no other meaning
   * for, an oversized body, unparsable JSON - fails the whole fetch, because a
   * picture missing a branch is indistinguishable on the wire from a peer that
   * does not have that branch. Two statuses do carry a meaning of their own and
   * are read rather than failed:
   *
   * - 404 on a nested collection route: the peer predates the route. Those
   *   members are omitted and the route is named in
   *   PeerEntities::absent_routes.
   * - `504 not-responding` on a Component detail: the peer holds that id and
   *   says whoever contributes it has gone quiet, which is what an aggregating
   *   peer answers for a declaration it is retaining. The Component is kept as
   *   its list named it, marked unavailable.
   *
   * @return PeerEntities on success, error message on failure
   */
  tl::expected<PeerEntities, std::string> fetch_entities();

  /**
   * @brief Forward an HTTP request transparently to the peer (proxy)
   *
   * Copies method, path, body, and Content-Type from the incoming request.
   * Forwards the Authorization header only if forward_auth is enabled.
   * Copies the peer's response status, headers, and body back to the
   * outgoing response.
   *
   * On connection failure, returns 502 with x-medkit-peer-unavailable error.
   *
   * @param req Incoming HTTP request to forward
   * @param res Outgoing HTTP response to populate
   */
  void forward_request(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Forward a request and parse the JSON response
   *
   * Used for fan-out merge scenarios where the aggregator needs to
   * combine JSON results from multiple peers.
   *
   * @param method HTTP method (e.g., "GET")
   * @param path Request path (e.g., "/api/v1/components/abc/data")
   * @param auth_header Authorization header value (empty to omit)
   * @param extra_headers Additional headers to include in the request
   *   (e.g., X-Medkit-No-Fan-Out to prevent recursive fan-out loops)
   * @return Parsed JSON on success, error message on failure
   */
  tl::expected<nlohmann::json, std::string> forward_and_get_json(const std::string & method, const std::string & path,
                                                                 const std::string & auth_header = "",
                                                                 const httplib::Headers & extra_headers = {});

  /**
   * @brief Cancel every in-flight HTTP call against this peer.
   *
   * Sets a shutdown flag (so subsequent forwards return 503 immediately
   * without dialing the peer) and invokes ``stop()`` on every active
   * ``httplib::Client`` registered by an in-flight forward / fetch / health
   * call. This unblocks worker threads sitting in ``cli.Get/Post/...`` so
   * gateway shutdown does not have to wait out the full peer read timeout.
   * Idempotent. Safe to call from any thread.
   */
  void shutdown();

 private:
  /**
   * @brief Ensure the underlying HTTP client exists (lazy initialization)
   *
   * Must be called under client_mutex_. Creates the client if it doesn't exist.
   */
  void ensure_client();

  /// RAII helper for per-call clients: registers itself with active_clients_
  /// on construction and unregisters on destruction. shutdown() iterates
  /// active_clients_ and calls stop() on each so blocked I/O unwinds.
  class ScopedClient {
   public:
    ScopedClient(PeerClient & owner, const std::string & url, int timeout_ms);
    ~ScopedClient();
    ScopedClient(const ScopedClient &) = delete;
    ScopedClient & operator=(const ScopedClient &) = delete;
    ScopedClient(ScopedClient &&) = delete;
    ScopedClient & operator=(ScopedClient &&) = delete;

    httplib::Client & operator*() {
      return cli_;
    }
    httplib::Client * operator->() {
      return &cli_;
    }

   private:
    PeerClient & owner_;
    httplib::Client cli_;
  };

  void register_active(httplib::Client * cli);
  void unregister_active(httplib::Client * cli);

  std::string url_;
  std::string name_;
  int timeout_ms_;
  bool forward_auth_;
  std::atomic<bool> healthy_{false};
  std::atomic<bool> shutdown_requested_{false};

  std::mutex client_mutex_;
  std::unique_ptr<httplib::Client> client_;

  std::mutex active_mutex_;
  std::unordered_set<httplib::Client *> active_clients_;
};

}  // namespace ros2_medkit_gateway
