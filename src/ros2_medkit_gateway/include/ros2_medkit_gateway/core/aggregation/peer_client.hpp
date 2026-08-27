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

#include <chrono>

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
 * @brief Why a call to a peer produced no usable answer.
 *
 * `httplib::Result::operator bool()` is a null check, so a branch on the result
 * alone renders a peer that ran out of time and a peer that refused the
 * connection identically. They are not the same event: the first says the peer
 * is working and this gateway stopped waiting, the second says nothing
 * answered, and an operator reading "unavailable" about a peer that is serving
 * the same request in five seconds is being told something false.
 */
enum class PeerFailureKind : uint8_t {
  kNone,             ///< No failure.
  kTimeout,          ///< A connect, read or write budget ran out. The peer may still be working.
  kUnreachable,      ///< Nothing answered: refused, no route, DNS or TLS failure.
  kCanceled,         ///< This gateway stopped the call, which happens during its own shutdown.
  kErrorStatus,      ///< The peer answered, with a status outside 2xx.
  kTooLarge,         ///< The body passed the peer-response size limit.
  kInvalidResponse,  ///< The peer answered with something that is not the JSON the route promises.
};

/// A peer call that failed, and why. Kept together because every caller that
/// reports the message also has to decide a status from the kind, and the two
/// were previously separated by the error being a bare string.
struct PeerError {
  PeerFailureKind kind{PeerFailureKind::kUnreachable};
  std::string message;
};

/// Classify a cpp-httplib transport error.
///
/// cpp-httplib reports both "the read budget ran out" and "the connection
/// broke while the answer was arriving" as `Error::Read`, so the error alone
/// cannot tell a peer that is still working from one that died mid-answer.
/// `elapsed` against `budget` is what separates them: a call that came back
/// inside its budget did not run out of it, whatever else went wrong.
/// @param read_budget Budget for the read this call was making.
/// @param write_budget Budget for pushing the request body.
/// A write that ran out compares against the WRITE budget, which is its own and
/// is smaller than the read budget by default; comparing it against the read
/// budget classifies every write timeout as unreachable.
PeerFailureKind classify_peer_error(httplib::Error error, std::chrono::milliseconds elapsed,
                                    std::chrono::milliseconds read_budget, std::chrono::milliseconds write_budget);

/// Which budget a failure was measured against, so a message can name the one
/// that actually expired. A connect timeout is bounded by the connect budget,
/// not by the read budget the call was hoping to use.
enum class PeerBudget : uint8_t { kNone, kConnect, kRead, kWrite };

/// The budget `classify_peer_error` measured this error against.
PeerBudget peer_budget_for(httplib::Error error);

/// Stable wire token for a failure kind, as carried in
/// ``x-medkit.peer_failures[].reason``.
const char * peer_failure_reason(PeerFailureKind kind);

/**
 * @brief The budgets one peer request runs under.
 *
 * Four numbers rather than one, because they bound different things and a
 * single value can only be right for one of them at a time. A metadata read
 * that is given a real operation's budget stalls a discovery pass behind a
 * dead peer; a real operation that is given a metadata read's budget is cut
 * off while the peer is still working on it.
 */
struct PeerTimeouts {
  /// Connect budget for every call. A peer that will not complete a TCP
  /// handshake inside this is out of reach whatever the request asks for, so
  /// this stays short even where the read budget is long.
  int connect_ms{2000};

  /// Read budget for the peer's description of itself: discovery, health, and
  /// the collection fan-out a client is waiting on across every peer at once.
  int metadata_read_ms{2000};

  /// Read budget for a forwarded request, which carries the peer's own work.
  /// A synchronous service call on the far side runs against that gateway's
  /// `service_call_timeout_sec`, so a budget below it cannot ever
  /// see the answer; a large resource needs the transfer on top of that.
  int forward_read_ms{15000};

  /// Write budget for every call. cpp-httplib defaults this to 5 s.
  int write_ms{5000};

  /// Every budget set to the same value - the pre-split shape, for a caller
  /// that is not testing the split.
  static PeerTimeouts uniform(int ms) {
    return PeerTimeouts{ms, ms, ms, ms};
  }
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
   * @param timeouts Per-kind budgets; see PeerTimeouts
   * @param forward_auth Whether to forward Authorization headers to this peer
   */
  PeerClient(const std::string & url, const std::string & name, PeerTimeouts timeouts, bool forward_auth = false,
             std::string peer_auth_header = "");

  /// Convenience overload giving every budget the same value.
  PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth = false,
             std::string peer_auth_header = "");

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
   * @return Parsed JSON on success, a classified PeerError on failure. The
   *   kind is what lets a fanned-out collection say WHY a peer contributed
   *   nothing; the caller used to receive a bare string and drop it.
   */
  tl::expected<nlohmann::json, PeerError> forward_and_get_json(const std::string & method, const std::string & path,
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
    /// @param read_timeout_ms Read budget for this call. Connect and write
    /// come from the owner's PeerTimeouts, which do not vary per call kind.
    ScopedClient(PeerClient & owner, const std::string & url, int read_timeout_ms);
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
  PeerTimeouts timeouts_;
  bool forward_auth_;
  /// What this gateway presents as Authorization when it is acting for itself
  /// rather than relaying a client's request: the health check, and any
  /// forward whose caller sent no credential to pass on. Empty for none.
  std::string peer_auth_header_;
  std::atomic<bool> healthy_{false};
  std::atomic<bool> shutdown_requested_{false};

  std::mutex client_mutex_;
  std::unique_ptr<httplib::Client> client_;

  std::mutex active_mutex_;
  std::unordered_set<httplib::Client *> active_clients_;
};

}  // namespace ros2_medkit_gateway
