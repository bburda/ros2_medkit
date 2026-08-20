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

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/classification.hpp"
#include "ros2_medkit_gateway/core/aggregation/peer_client.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Configuration for peer aggregation
 */
struct AggregationConfig {
  bool enabled{false};
  int timeout_ms{2000};
  bool announce{false};
  bool discover{false};
  std::string mdns_service{"_medkit._tcp.local"};

  /// Forward the client's Authorization header to peer gateways.
  /// Default: false (safe default - prevents token leakage to untrusted peers).
  bool forward_auth{false};

  /// Require TLS (https://) for all peer URLs.
  /// When true, peers with http:// URLs are rejected (logged as ERROR and skipped).
  /// When false, http:// peers produce a WARN at startup about cleartext communication.
  bool require_tls{false};

  /// URL scheme for mDNS-discovered peer URLs ("http" or "https").
  /// Default: "http". Set to "https" when peers use TLS.
  std::string peer_scheme{"http"};

  /// Maximum number of peers that can be added via mDNS discovery.
  /// Prevents unbounded growth of the peer list from rogue mDNS announcements.
  /// Static peers do not count against this limit.
  /// Default: 50.
  size_t max_discovered_peers{50};

  struct PeerConfig {
    std::string url;
    std::string name;
  };
  std::vector<PeerConfig> peers;
};

/**
 * @brief Coordinator for peer aggregation
 *
 * Manages PeerClients, health monitoring, routing table, entity merging,
 * and fan-out logic. Thread-safe: shared_mutex protects peers_ and
 * routing_table_. Readers use shared lock, writers use exclusive lock.
 */
class AggregationManager {
 public:
  /**
   * @brief Result of fetching and merging entities from all healthy peers
   *
   * Contains merged entity vectors and a routing table mapping remote entity
   * IDs to the peer name that owns them.
   */
  struct MergedPeerResult {
    std::vector<Area> areas;
    std::vector<Component> components;
    std::vector<App> apps;
    std::vector<Function> functions;
    std::unordered_map<std::string, std::string> routing_table;
    /// Multi-peer leaf-Component collisions detected during merge. Surfaced
    /// to operators via /health.warnings; runtime falls back to last-writer
    /// for routing.
    std::vector<LeafCollisionWarning> leaf_warnings;
  };

  /**
   * @brief Result of a fan-out GET across all healthy peers
   */
  struct FanOutResult {
    nlohmann::json merged_items;            ///< Merged "items" array from all peers
    bool is_partial{false};                 ///< True if some peers failed
    std::vector<std::string> failed_peers;  ///< Names of peers that failed
  };

  /**
   * @brief Construct an AggregationManager from config
   *
   * Creates a PeerClient for each statically configured peer. Validates
   * TLS requirements and logs warnings for cleartext peer URLs.
   *
   * @param config Aggregation configuration
   * @param logger Optional logger for TLS warnings (pass nullptr to suppress)
   */
  explicit AggregationManager(const AggregationConfig & config, rclcpp::Logger * logger = nullptr);

  /**
   * @brief Cancel every in-flight HTTP call to every peer.
   *
   * Calls ``PeerClient::shutdown()`` on every known peer (static + discovered).
   * Each peer stops its active ``httplib::Client`` instances so worker threads
   * blocked in cli.Get/Post return immediately with ``Error::Canceled`` instead
   * of waiting out the full peer read timeout. Idempotent; safe to call from
   * any thread. Called from ``GatewayNode::~GatewayNode`` before the REST
   * server stop so worker threads can drain promptly even under sanitizer
   * instrumentation, where the per-call read timeout otherwise exceeds the
   * launch_test SIGINT->SIGKILL grace window.
   */
  void shutdown();

  /// Get the number of known peers (static + discovered)
  size_t peer_count() const;

  /**
   * @brief Add a dynamically discovered peer
   *
   * Thread-safe. If a peer with the given name already exists, this is a no-op.
   *
   * @param url Base URL of the peer
   * @param name Human-readable peer name
   */
  void add_discovered_peer(const std::string & url, const std::string & name);

  /**
   * @brief Remove a dynamically discovered peer by name
   *
   * Thread-safe. If the peer is not found, this is a no-op.
   *
   * @param name Peer name to remove
   */
  void remove_discovered_peer(const std::string & name);

  /**
   * @brief Check health of all peers
   *
   * Calls check_health() on each PeerClient.
   */
  void check_all_health();

  /**
   * @brief Get count of currently healthy peers
   * @return Number of peers that report healthy status
   */
  size_t healthy_peer_count() const;

  /**
   * @brief Fetch entities from all healthy peers and merge them
   * @return Merged PeerEntities from all reachable peers
   */
  PeerEntities fetch_all_peer_entities();

  /**
   * @brief Fetch entities from all healthy peers, merge with local entities, and build routing table
   *
   * Snapshots peer shared_ptrs under lock, releases before network I/O. The
   * shared_ptr copies keep PeerClients alive even if remove_discovered_peer()
   * runs concurrently. Uses EntityMerger per-peer so that collision-prefixed
   * IDs are correctly tracked in the routing table.
   *
   * @param local_areas Local areas to merge with
   * @param local_components Local components to merge with
   * @param local_apps Local apps to merge with
   * @param local_functions Local functions to merge with
   * @param max_entities_per_peer Maximum total entities accepted from a single peer
   * @param logger Optional logger for warnings (pass nullptr to suppress)
   * @return MergedPeerResult with merged entity vectors and routing table
   */
  MergedPeerResult
  fetch_and_merge_peer_entities(const std::vector<Area> & local_areas, const std::vector<Component> & local_components,
                                const std::vector<App> & local_apps, const std::vector<Function> & local_functions,
                                size_t max_entities_per_peer = 10000, rclcpp::Logger * logger = nullptr);

  /**
   * @brief Update the routing table (entity_id -> peer_name)
   * @param table New routing table to replace the current one
   */
  void update_routing_table(const std::unordered_map<std::string, std::string> & table);

  /**
   * @brief Publish the latest leaf-collision warnings (replaces previous list).
   *
   * Warnings describe deployment-level anomalies where more than one peer
   * announced the same leaf Component ID. Exposed on /health.warnings so
   * operators can notice without tailing logs. Thread-safe.
   */
  void set_leaf_warnings(std::vector<LeafCollisionWarning> warnings);

  /**
   * @brief Snapshot the current leaf-collision warnings.
   * @return Copy of the warning list (may be empty). Thread-safe.
   */
  std::vector<LeafCollisionWarning> get_leaf_warnings() const;

  /**
   * @brief Look up which peer owns a given entity
   * @param entity_id Entity ID to look up
   * @return Peer name if entity is remote, std::nullopt if local or unknown
   */
  std::optional<std::string> find_peer_for_entity(const std::string & entity_id) const;

  /**
   * @brief Replace the map of per-entity peer contributors.
   *
   * Covers both routed leaf entities (entity fully hosted by a peer - also
   * tracked via the routing table) and merged/hierarchical entities (served
   * locally with contributions from one or more peers - e.g. a Component
   * aggregating subcomponents from multiple gateways, or an Area/Function
   * with an ID shared across peers).
   *
   * Fan-out helpers consult this map to target per-entity collection
   * requests only at the peers that actually host / contribute to a given
   * entity, avoiding spurious 404s from non-contributing peers.
   *
   * @param contributors New map: entity id -> list of peer names that
   *        contribute to it. Entries with empty lists are discarded.
   */
  void update_peer_contributors(std::unordered_map<std::string, std::vector<std::string>> contributors);

  /**
   * @brief Check whether an entity id has at least one peer contributor.
   *
   * Returns true when the entity is either routed to a peer (remote leaf) or
   * served locally but aggregated from peers (hierarchical / merged). Returns
   * false for local-only entities and for unknown IDs. Thread-safe.
   */
  bool has_peer_contributors(const std::string & entity_id) const;

  /**
   * @brief List the peers that host / contribute to a given entity.
   *
   * Combines information from the routing table (routed leaves) and the
   * per-entity contributors map (merged / hierarchical entities). Duplicates
   * are removed while preserving first-seen order. Returns an empty vector
   * for local-only entities and unknown IDs. Thread-safe.
   */
  std::vector<std::string> get_peer_contributors(const std::string & entity_id) const;

  /**
   * @brief Get the URL for a known peer by name
   * @param peer_name Name of the peer
   * @return URL if found, empty string otherwise
   */
  std::string get_peer_url(const std::string & peer_name) const;

  /**
   * @brief Forward an HTTP request to a specific peer
   *
   * Finds the PeerClient by name and calls forward_request().
   * If the peer is not found, sends a 502 error response.
   *
   * @param peer_name Name of the target peer
   * @param req Incoming HTTP request to forward
   * @param res Outgoing HTTP response to populate
   */
  void forward_request(const std::string & peer_name, const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Fan-out a GET request to healthy peers in parallel.
   *
   * Sends GET requests concurrently via std::async, merges the "items"
   * arrays from their responses. Returns partial results if some peers fail.
   *
   * @param path Request path (e.g., "/api/v1/components")
   * @param auth_header Authorization header value (empty to omit)
   * @param target_peers When non-null, only peers whose name is in this
   *        list AND are currently healthy are queried. A non-null but empty
   *        list means "no matching peers" and produces an empty result with
   *        no fan-out. When null (default), all healthy peers are queried,
   *        preserving the pre-filtering behavior for global endpoints.
   * @return FanOutResult with merged items and failure info.
   */
  FanOutResult fan_out_get(const std::string & path, const std::string & auth_header,
                           const std::vector<std::string> * target_peers = nullptr);

  /**
   * @brief Get peer status for /health endpoint
   * @return JSON array of peer objects with name, url, status
   */
  nlohmann::json get_peer_status() const;

 private:
  AggregationConfig config_;
  rclcpp::Logger logger_;
  size_t static_peer_count_{0};  ///< Number of statically configured peers (not subject to max_discovered_peers)
  /// Record what `peer_name` declared, so it can be replayed while that peer is
  /// silent. Stored already marked unavailable - it is only ever read back on
  /// the path where the peer did not answer.
  void remember_declaration(const std::string & peer_name, const PeerEntities & entities);

  /// The retained declaration for `peer_name`, empty if none was ever recorded.
  PeerEntities replay_declaration(const std::string & peer_name) const;

  mutable std::shared_mutex mutex_;  // Declared before data it protects (destruction order)
  std::vector<std::shared_ptr<PeerClient>> peers_;
  std::unordered_map<std::string, std::string> routing_table_;
  /// Last known manifest-declared entities per peer name, replayed while that
  /// peer is not answering.
  ///
  /// A tree that changes shape because a link went down cannot be reasoned
  /// about: the same request gets a different answer depending on who happens
  /// to be reachable. What a peer DECLARED is a property of its configuration
  /// and does not stop being true when it stops replying, so it is retained and
  /// marked unavailable. What a peer merely DISCOVERED at runtime is a property
  /// of a live graph this gateway can no longer observe, so it is allowed to
  /// disappear.
  std::unordered_map<std::string, PeerEntities> retained_peer_entities_;
  std::unordered_map<std::string, std::vector<std::string>> peer_contributors_by_entity_;
  std::vector<LeafCollisionWarning> leaf_warnings_;

  /**
   * @brief Find a peer by name (caller must hold lock)
   * @param name Peer name to search for
   * @return Raw pointer to PeerClient, or nullptr if not found
   */
  PeerClient * find_peer(const std::string & name) const;

  /**
   * @brief Find a peer by name and return a shared_ptr (caller must hold lock)
   *
   * Returns a shared_ptr copy that keeps the PeerClient alive after the caller
   * releases the lock, enabling lock-free network I/O.
   *
   * @param name Peer name to search for
   * @return shared_ptr to PeerClient, or nullptr if not found
   */
  std::shared_ptr<PeerClient> find_peer_shared(const std::string & name) const;
};

}  // namespace ros2_medkit_gateway
