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

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"

#include <algorithm>
#include <future>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>

#include <rclcpp/logging.hpp>

#include "ros2_medkit_gateway/core/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {

namespace {

/**
 * @brief Extract the host portion from a URL
 *
 * Handles both regular hosts and IPv6 bracket notation.
 * Returns empty string if the URL cannot be parsed.
 */
std::string extract_host(const std::string & url) {
  // Find the start of the host (after "://")
  auto scheme_end = url.find("://");
  if (scheme_end == std::string::npos) {
    return "";
  }
  size_t host_start = scheme_end + 3;
  if (host_start >= url.size()) {
    return "";
  }

  // IPv6 bracket notation: [::1] or [fe80::1%eth0]
  if (url[host_start] == '[') {
    auto bracket_end = url.find(']', host_start);
    if (bracket_end == std::string::npos) {
      return "";
    }
    return url.substr(host_start + 1, bracket_end - host_start - 1);
  }

  // Regular host: find end at ':', '/', or end of string
  size_t host_end = url.find_first_of(":/", host_start);
  if (host_end == std::string::npos) {
    host_end = url.size();
  }
  return url.substr(host_start, host_end - host_start);
}

/**
 * @brief Check if a resolved address is loopback, link-local, or unspecified
 *
 * Returns true if the address should be blocked for mDNS-discovered peers.
 */
bool is_blocked_address(const struct sockaddr * addr) {
  if (addr->sa_family == AF_INET) {
    const auto * sin = reinterpret_cast<const struct sockaddr_in *>(addr);
    uint32_t ip = ntohl(sin->sin_addr.s_addr);
    // 127.0.0.0/8 - loopback
    if ((ip >> 24) == 127) {
      return true;
    }
    // 0.0.0.0 - unspecified (binds all interfaces, self-referential)
    if (ip == 0) {
      return true;
    }
    // 169.254.0.0/16 - link-local (includes cloud metadata 169.254.169.254)
    if ((ip >> 16) == 0xA9FE) {
      return true;
    }
    return false;
  }

  if (addr->sa_family == AF_INET6) {
    const auto * sin6 = reinterpret_cast<const struct sockaddr_in6 *>(addr);
    // ::1 - IPv6 loopback
    if (IN6_IS_ADDR_LOOPBACK(&sin6->sin6_addr)) {
      return true;
    }
    // :: - IPv6 unspecified
    if (IN6_IS_ADDR_UNSPECIFIED(&sin6->sin6_addr)) {
      return true;
    }
    // fe80::/10 - IPv6 link-local
    if (IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr)) {
      return true;
    }
    // ::ffff:127.x.x.x - IPv4-mapped loopback
    // ::ffff:0.0.0.0 - IPv4-mapped unspecified
    // ::ffff:169.254.x.x - IPv4-mapped link-local
    if (IN6_IS_ADDR_V4MAPPED(&sin6->sin6_addr)) {
      const auto * bytes = sin6->sin6_addr.s6_addr;
      // Last 4 bytes are the IPv4 address
      uint32_t ip = (static_cast<uint32_t>(bytes[12]) << 24) | (static_cast<uint32_t>(bytes[13]) << 16) |
                    (static_cast<uint32_t>(bytes[14]) << 8) | static_cast<uint32_t>(bytes[15]);
      if ((ip >> 24) == 127 || ip == 0 || (ip >> 16) == 0xA9FE) {
        return true;
      }
    }
    return false;
  }

  // Unknown address family - block by default
  return true;
}

/**
 * @brief Validate a peer URL discovered via mDNS
 *
 * Rejects URLs that don't use HTTP(S), point to cloud metadata endpoints,
 * or resolve to loopback/link-local/unspecified addresses. Uses getaddrinfo()
 * to resolve the host, which catches IPv4-mapped IPv6 loopback (::ffff:127.0.0.1),
 * expanded IPv6 loopback (0:0:0:0:0:0:0:1), 0.0.0.0, [::], and other bypass
 * variants that substring matching would miss.
 *
 * Known TOCTOU: DNS is resolved here at validation time, but PeerClient resolves
 * again when connecting. A DNS rebinding attack could pass validation with a safe
 * IP and then resolve to a blocked IP on connection. Mitigation: the 2s read
 * timeout on PeerClient limits exposure, and IP pinning would break legitimate
 * DNS failover scenarios. This is inherent to DNS-based validation.
 */
bool is_valid_peer_url(const std::string & url) {
  // Must start with http:// or https://
  if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0) {
    return false;
  }

  // Block well-known cloud metadata hostnames regardless of resolution
  if (url.find("metadata.google") != std::string::npos) {
    return false;
  }

  std::string host = extract_host(url);
  if (host.empty()) {
    return false;
  }

  // Resolve the host to check the actual address
  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICHOST;  // Try numeric first (fast, no DNS)

  struct addrinfo * result = nullptr;
  int ret = getaddrinfo(host.c_str(), nullptr, &hints, &result);

  if (ret != 0) {
    // Not a numeric address - try DNS resolution
    hints.ai_flags = 0;
    ret = getaddrinfo(host.c_str(), nullptr, &hints, &result);
    if (ret != 0) {
      // Cannot resolve - reject
      return false;
    }
  }

  // Check all resolved addresses - block if ANY resolves to a blocked range
  bool blocked = false;
  for (struct addrinfo * rp = result; rp != nullptr; rp = rp->ai_next) {
    if (is_blocked_address(rp->ai_addr)) {
      blocked = true;
      break;
    }
  }

  freeaddrinfo(result);
  return !blocked;
}

}  // namespace

AggregationManager::AggregationManager(const AggregationConfig & config, rclcpp::Logger * logger)
  : config_(config), logger_(logger ? *logger : rclcpp::get_logger("aggregation_manager")) {
  for (const auto & peer_cfg : config_.peers) {
    // Validate scheme for static peers (http/https only).
    // Unlike mDNS peers, loopback addresses are valid for static config
    // (e.g., testing or same-host deployments).
    if (peer_cfg.url.rfind("http://", 0) != 0 && peer_cfg.url.rfind("https://", 0) != 0) {
      continue;
    }

    // TLS enforcement: reject http:// peers when require_tls is true,
    // warn about cleartext when require_tls is false.
    if (peer_cfg.url.rfind("http://", 0) == 0) {
      if (config_.require_tls) {
        if (logger) {
          RCLCPP_ERROR(*logger, "Aggregation: skipping peer '%s' at %s - require_tls is enabled but URL uses http://",
                       peer_cfg.name.c_str(), peer_cfg.url.c_str());
        }
        continue;
      }
      if (logger) {
        RCLCPP_WARN(*logger, "Aggregation: peer '%s' at %s uses cleartext HTTP - consider using HTTPS",
                    peer_cfg.name.c_str(), peer_cfg.url.c_str());
      }
    }

    peers_.push_back(
        std::make_shared<PeerClient>(peer_cfg.url, peer_cfg.name, config_.peer_timeouts(), config_.forward_auth));
  }
  static_peer_count_ = peers_.size();
}

size_t AggregationManager::peer_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return peers_.size();
}

void AggregationManager::shutdown() {
  // Snapshot the peer list under the shared mutex so we do not hold it
  // while each peer's PeerClient::shutdown() iterates its active-client
  // set. Holding the aggregation mutex through that path could deadlock
  // with add/remove_discovered_peer calls still racing through.
  std::vector<std::shared_ptr<PeerClient>> peers_snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    peers_snapshot.reserve(peers_.size());
    for (auto & peer : peers_) {
      peers_snapshot.push_back(peer);
    }
  }
  for (auto & peer : peers_snapshot) {
    peer->shutdown();
  }
}

void AggregationManager::add_discovered_peer(const std::string & url, const std::string & name) {
  // Validate mDNS-discovered peer URLs (static config peers bypass this check)
  if (!is_valid_peer_url(url)) {
    return;
  }

  // TLS enforcement for discovered peers
  if (url.rfind("http://", 0) == 0 && config_.require_tls) {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Do not add if a peer with this name already exists
  if (find_peer(name) != nullptr) {
    return;
  }

  // Enforce max_discovered_peers limit (static peers do not count)
  size_t discovered_count = peers_.size() - static_peer_count_;
  if (discovered_count >= config_.max_discovered_peers) {
    RCLCPP_WARN(logger_,
                "Aggregation: max_discovered_peers limit (%zu) reached, "
                "rejecting peer '%s' at %s",
                config_.max_discovered_peers, name.c_str(), url.c_str());
    return;
  }

  peers_.push_back(std::make_shared<PeerClient>(url, name, config_.peer_timeouts(), config_.forward_auth));
}

void AggregationManager::remove_discovered_peer(const std::string & name) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Only iterate discovered peers [static_peer_count_, end) to protect
  // static peers from removal by mDNS goodbye messages or other dynamic events.
  auto discovered_begin = peers_.begin() + static_cast<ptrdiff_t>(static_peer_count_);
  auto it = std::remove_if(discovered_begin, peers_.end(), [&name](const std::shared_ptr<PeerClient> & peer) {
    return peer->name() == name;
  });
  peers_.erase(it, peers_.end());
}

void AggregationManager::check_all_health() {
  // Snapshot shared_ptrs under lock, then release before I/O.
  // shared_ptr copies keep PeerClients alive even if remove_discovered_peer()
  // erases them from peers_ during health checks.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    snapshot.reserve(peers_.size());
    for (const auto & peer : peers_) {
      snapshot.push_back(peer);
    }
  }

  // Health checks run in parallel via std::async to reduce worst-case latency
  // from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::vector<std::future<void>> futures;
  futures.reserve(snapshot.size());
  for (auto & peer : snapshot) {
    futures.push_back(std::async(std::launch::async, [peer]() {
      peer->check_health();
    }));
  }
  for (auto & f : futures) {
    f.get();
  }
}

size_t AggregationManager::healthy_peer_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  size_t count = 0;
  for (const auto & peer : peers_) {
    if (peer->is_healthy()) {
      ++count;
    }
  }
  return count;
}

namespace {

/// True for an entity a peer said it had DECLARED, rather than discovered from
/// its live ROS graph. Only a declaration outlives the link that reported it:
/// runtime discovery describes a graph this gateway can no longer observe once
/// the peer stops answering, so it is not ours to keep asserting.
template <class Entity>
bool is_declared(const Entity & entity) {
  return entity.declared_source == "manifest";
}

/// Availability belongs to an entity a request can be addressed to. A grouping
/// entity has none of its own: it is a view over its members, and the members
/// carry the flag, so retaining one says nothing about what can be reached.
void mark_unreachable(Area & /*area*/) {
}
void mark_unreachable(Function & /*function*/) {
}
void mark_unreachable(App & app) {
  app.available = false;
  // A retained App is not observably running: the graph it was bound to is on
  // the other side of a link that is down. `is_online` is the field every
  // consumer already reads for that, so it carries the news rather than a
  // second flag they would each have to learn.
  app.is_online = false;
}
void mark_unreachable(Component & component) {
  component.available = false;
}

/// The declared entities of `src`, as the peer reported them.
template <class Entity>
std::vector<Entity> declared_only(const std::vector<Entity> & src) {
  std::vector<Entity> kept;
  for (const auto & entity : src) {
    if (is_declared(entity)) {
      kept.push_back(entity);
    }
  }
  return kept;
}

/// Mark every addressable entity of a replayed declaration unreachable.
///
/// The marking is one-way: it only ever sets the flag false, never back to
/// true. So an entity a peer itself reported as unreachable keeps that from its
/// own account, and where the marking does apply over a peer's `available` it
/// is because the peer stopped answering - which is the stronger statement,
/// since everything it holds sits behind the link that is down.
template <class Entity>
void mark_all_unreachable(std::vector<Entity> & entities) {
  for (auto & entity : entities) {
    mark_unreachable(entity);
  }
}

}  // namespace

void AggregationManager::remember_declaration(const std::string & peer_name, const PeerEntities & entities) {
  PeerEntities declared;
  declared.areas = declared_only(entities.areas);
  declared.components = declared_only(entities.components);
  declared.apps = declared_only(entities.apps);
  declared.functions = declared_only(entities.functions);

  std::unique_lock<std::shared_mutex> lock(mutex_);
  retained_peer_entities_[peer_name] = std::move(declared);
}

PeerEntities AggregationManager::replay_declaration(const std::string & peer_name, Reachability reachability) const {
  PeerEntities replayed;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = retained_peer_entities_.find(peer_name);
    if (it == retained_peer_entities_.end()) {
      return replayed;
    }
    replayed = it->second;
  }

  if (reachability == Reachability::kUnreachable) {
    mark_all_unreachable(replayed.areas);
    mark_all_unreachable(replayed.components);
    mark_all_unreachable(replayed.apps);
    mark_all_unreachable(replayed.functions);
  }
  return replayed;
}

AggregationManager::MergedPeerResult AggregationManager::fetch_and_merge_peer_entities(
    const std::vector<Area> & local_areas, const std::vector<Component> & local_components,
    const std::vector<App> & local_apps, const std::vector<Function> & local_functions, size_t max_entities_per_peer,
    rclcpp::Logger * logger) {
  MergedPeerResult merged;
  merged.areas = local_areas;
  merged.components = local_components;
  merged.apps = local_apps;
  merged.functions = local_functions;

  // Seed aggregation provenance: every locally-sourced entity contributes
  // "local" to its own contributors list. EntityMerger will append
  // "peer:<name>" entries as peer contributions are folded in.
  auto seed_local = [](auto & entities) {
    for (auto & e : entities) {
      if (e.contributors.empty()) {
        e.contributors.emplace_back("local");
      }
    }
  };
  seed_local(merged.areas);
  seed_local(merged.components);
  seed_local(merged.apps);
  seed_local(merged.functions);

  // Snapshot healthy peers under lock, release before network I/O.
  // shared_ptr copies keep PeerClients alive even if remove_discovered_peer()
  // erases them from peers_ concurrently.
  std::vector<std::shared_ptr<PeerClient>> snapshot;

  // A peer this cycle could not describe, and whether it can still be reached.
  struct UnreadPeer {
    std::string name;
    Reachability reachability{Reachability::kUnreachable};
    std::string reason;
  };
  std::vector<UnreadPeer> unread_peers;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto & peer : peers_) {
      if (peer->is_healthy()) {
        snapshot.push_back(peer);
      } else {
        unread_peers.push_back({peer->name(), Reachability::kUnreachable, "not answering"});
      }
    }
  }

  // Per-peer fetch result collected by each async task
  struct PeerFetchResult {
    std::string peer_name;
    bool success{false};
    std::string error;
    /// Reachability as of the health check taken after a failed fetch, which is
    /// what separates a peer that died mid-refresh from one that is answering
    /// but could not be read in full.
    Reachability reachability{Reachability::kReachable};
    PeerEntities entities;
  };

  // Fan out fetch_entities() in parallel via std::async to reduce worst-case
  // latency from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::vector<std::future<PeerFetchResult>> futures;
  futures.reserve(snapshot.size());
  for (auto & peer : snapshot) {
    futures.push_back(std::async(std::launch::async, [peer, max_entities_per_peer]() {
      PeerFetchResult pfr;
      pfr.peer_name = peer->name();

      // Re-checking health only on the failure path answers the one question the
      // caller needs and costs nothing on the path that succeeded: the flag from
      // before the fetch cannot tell a peer that has since died from one that is
      // still there and merely could not be read.
      auto fetch_failed = [&pfr, &peer](std::string reason) {
        pfr.success = false;
        pfr.error = std::move(reason);
        peer->check_health();
        pfr.reachability = peer->is_healthy() ? Reachability::kReachable : Reachability::kUnreachable;
        return pfr;
      };

      auto result = peer->fetch_entities();
      if (!result.has_value()) {
        return fetch_failed(result.error());
      }

      size_t total = result->areas.size() + result->components.size() + result->apps.size() + result->functions.size();
      if (total > max_entities_per_peer) {
        return fetch_failed("returned " + std::to_string(total) + " entities (max " +
                            std::to_string(max_entities_per_peer) + "), skipping");
      }

      pfr.success = true;
      pfr.entities = std::move(result.value());
      return pfr;
    }));
  }

  // Per-peer Component IDs each peer claimed, fed into classify_component_routing
  // after the loop so that hierarchical-vs-leaf classification sees the full
  // merged Component set (including subcomponents from other peers).
  std::vector<PeerClaim> peer_component_claims;

  // Collect results and merge sequentially (merge order must be deterministic)
  std::vector<PeerFetchResult> to_merge;
  to_merge.reserve(futures.size() + unread_peers.size());
  for (auto & f : futures) {
    auto pfr = f.get();
    if (!pfr.success) {
      // The last complete declaration is not replaced by a picture with holes
      // in it: `remember_declaration` overwrites, so recording a partial fetch
      // would outlive the cycle that failed and be replayed as the peer's own
      // account of itself.
      if (logger) {
        RCLCPP_WARN(*logger, "Failed to fetch entities from peer '%s': %s", pfr.peer_name.c_str(), pfr.error.c_str());
      }
      unread_peers.push_back({pfr.peer_name, pfr.reachability,
                              pfr.reachability == Reachability::kReachable ? "refresh incomplete" : "not answering"});
      continue;
    }
    if (logger && !pfr.entities.absent_routes.empty()) {
      std::string routes;
      for (const auto & route : pfr.entities.absent_routes) {
        routes += routes.empty() ? route : ", " + route;
      }
      RCLCPP_WARN(*logger, "Peer '%s' does not expose %s; aggregating without the members those routes carry",
                  pfr.peer_name.c_str(), routes.c_str());
    }
    remember_declaration(pfr.peer_name, pfr.entities);
    to_merge.push_back(std::move(pfr));
  }

  // A peer this cycle could not describe still contributes what it declared.
  // Merged after the peers that did answer, so a live declaration always wins
  // over a retained one carrying the same id.
  for (const auto & peer : unread_peers) {
    auto retained = replay_declaration(peer.name, peer.reachability);
    if (retained.areas.empty() && retained.components.empty() && retained.apps.empty() && retained.functions.empty()) {
      continue;
    }
    if (logger) {
      const size_t count =
          retained.areas.size() + retained.components.size() + retained.apps.size() + retained.functions.size();
      if (peer.reachability == Reachability::kUnreachable) {
        RCLCPP_INFO(*logger, "Peer '%s' %s; retaining %zu declared entities as unavailable", peer.name.c_str(),
                    peer.reason.c_str(), count);
      } else {
        RCLCPP_WARN(*logger, "Peer '%s' %s; it is still reachable, so its %zu declared entities stand as last read",
                    peer.name.c_str(), peer.reason.c_str(), count);
      }
    }
    PeerFetchResult replayed;
    replayed.peer_name = peer.name;
    replayed.success = true;
    replayed.entities = std::move(retained);
    to_merge.push_back(std::move(replayed));
  }

  for (auto & pfr : to_merge) {
    PeerClaim claim;
    claim.peer_name = pfr.peer_name;
    for (const auto & c : pfr.entities.components) {
      claim.claimed_entity_ids.insert(c.id);
    }
    peer_component_claims.push_back(std::move(claim));

    EntityMerger merger(pfr.peer_name);
    merged.areas = merger.merge_areas(merged.areas, pfr.entities.areas);
    merged.components = merger.merge_components(merged.components, pfr.entities.components);
    // Merge apps BEFORE functions so that collision-prefixed App IDs are known
    // when we need to remap Function hosts below.
    merged.apps = merger.merge_apps(merged.apps, pfr.entities.apps);

    // Build a map of original remote App IDs to their (possibly prefixed) final IDs.
    // When merge_apps prefixes a remote App (e.g., "camera_driver" -> "peer_b__camera_driver"),
    // Function hosts referencing the original ID need to be updated to the prefixed ID.
    std::unordered_map<std::string, std::string> app_id_remap;
    for (const auto & [routed_id, peer_name_val] : merger.get_routing_table()) {
      // Check if this is a prefixed App ID (contains SEPARATOR)
      std::string sep = EntityMerger::SEPARATOR;
      auto sep_pos = routed_id.find(sep);
      if (sep_pos != std::string::npos) {
        std::string original_id = routed_id.substr(sep_pos + sep.size());
        app_id_remap[original_id] = routed_id;
      }
    }

    // Remap Function hosts from original remote App IDs to their prefixed IDs
    // before merging functions.
    for (auto & func : pfr.entities.functions) {
      for (auto & host : func.hosts) {
        auto remap_it = app_id_remap.find(host);
        if (remap_it != app_id_remap.end()) {
          host = remap_it->second;
        }
      }
    }

    merged.functions = merger.merge_functions(merged.functions, pfr.entities.functions);

    for (const auto & [id, name] : merger.get_routing_table()) {
      merged.routing_table[id] = name;
      // Log collision-prefixed entities to help operators diagnose naming conflicts
      if (id.find(EntityMerger::SEPARATOR) != std::string::npos) {
        if (logger) {
          RCLCPP_WARN(*logger, "Entity ID collision: '%s' prefixed for peer '%s'", id.c_str(), name.c_str());
        }
      }
    }
  }

  // Reclassify Component routing entries against the fully-merged Component
  // set. Hierarchical parents (Components referenced as parent_component_id by
  // another Component) are removed from the routing table so they are served
  // locally with the merged view, mirroring Areas and Functions. Leaves keep
  // routing to the owning peer. Multi-peer leaf collisions surface as warnings.
  std::unordered_set<std::string> all_component_ids;
  all_component_ids.reserve(merged.components.size());
  for (const auto & c : merged.components) {
    all_component_ids.insert(c.id);
  }
  for (auto it = merged.routing_table.begin(); it != merged.routing_table.end();) {
    if (all_component_ids.count(it->first) > 0u) {
      it = merged.routing_table.erase(it);
    } else {
      ++it;
    }
  }
  auto classified = classify_component_routing(merged.components, peer_component_claims);
  for (auto & [id, name] : classified.routing_table) {
    merged.routing_table[id] = std::move(name);
  }
  merged.leaf_warnings = std::move(classified.leaf_warnings);

  if (logger) {
    for (const auto & msg : classified.malformed_parent_warnings) {
      RCLCPP_WARN(*logger, "Aggregation: %s", msg.c_str());
    }
    for (const auto & w : merged.leaf_warnings) {
      std::string peers_str;
      for (size_t i = 0; i < w.peer_names.size(); ++i) {
        if (i > 0u) {
          peers_str += ", ";
        }
        peers_str += w.peer_names[i];
      }
      RCLCPP_WARN(*logger, "Leaf Component '%s' announced by multiple peers: %s. Routing uses last-writer-wins.",
                  w.entity_id.c_str(), peers_str.c_str());
    }
  }

  return merged;
}

void AggregationManager::update_routing_table(const std::unordered_map<std::string, std::string> & table) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  routing_table_ = table;
}

void AggregationManager::set_leaf_warnings(std::vector<LeafCollisionWarning> warnings) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  leaf_warnings_ = std::move(warnings);
}

std::vector<LeafCollisionWarning> AggregationManager::get_leaf_warnings() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return leaf_warnings_;
}

std::optional<std::string> AggregationManager::find_peer_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  auto it = routing_table_.find(entity_id);
  if (it != routing_table_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::string AggregationManager::local_member_id(const std::string & peer_name, const std::string & member_id) const {
  if (peer_name.empty() || member_id.empty()) {
    return member_id;
  }
  const std::string prefixed = peer_name + EntityMerger::SEPARATOR + member_id;
  std::shared_lock<std::shared_mutex> lock(mutex_);
  auto it = routing_table_.find(prefixed);
  // The routing entry has to belong to THIS peer. Another peer can own an
  // entity whose id happens to read like this peer's prefix, and answering with
  // it would re-address the item to a gateway that never sent it.
  return (it != routing_table_.end() && it->second == peer_name) ? prefixed : member_id;
}

void AggregationManager::update_peer_contributors(
    std::unordered_map<std::string, std::vector<std::string>> contributors) {
  // Drop entries with empty contributor lists - they would be indistinguishable
  // from "local-only" lookups and only waste space.
  for (auto it = contributors.begin(); it != contributors.end();) {
    if (it->second.empty()) {
      it = contributors.erase(it);
    } else {
      ++it;
    }
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  peer_contributors_by_entity_ = std::move(contributors);
}

bool AggregationManager::has_peer_contributors(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (routing_table_.count(entity_id) > 0u) {
    return true;
  }
  auto it = peer_contributors_by_entity_.find(entity_id);
  return it != peer_contributors_by_entity_.end() && !it->second.empty();
}

std::vector<std::string> AggregationManager::get_peer_contributors(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> result;
  // Routed-leaf entry wins ordering: it always reflects the authoritative
  // owner of the entity's runtime state.
  auto rt_it = routing_table_.find(entity_id);
  if (rt_it != routing_table_.end()) {
    result.push_back(rt_it->second);
  }
  auto pc_it = peer_contributors_by_entity_.find(entity_id);
  if (pc_it != peer_contributors_by_entity_.end()) {
    for (const auto & name : pc_it->second) {
      if (std::find(result.begin(), result.end(), name) == result.end()) {
        result.push_back(name);
      }
    }
  }
  return result;
}

std::string AggregationManager::get_peer_url(const std::string & peer_name) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const auto * peer = find_peer(peer_name);
  if (peer != nullptr) {
    return peer->url();
  }
  return "";
}

void AggregationManager::forward_request(const std::string & peer_name, const httplib::Request & req,
                                         httplib::Response & res) {
  forward_request(peer_name, req, res, req.path);
}

void AggregationManager::forward_request(const std::string & peer_name, const httplib::Request & req,
                                         httplib::Response & res, const std::string & target_path) {
  // Find peer under lock, take shared_ptr copy for lifetime safety, then release
  // before network I/O. The shared_ptr keeps the PeerClient alive even if
  // remove_discovered_peer() erases it from peers_ concurrently.
  std::shared_ptr<PeerClient> peer;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    peer = find_peer_shared(peer_name);
  }

  if (!peer) {
    res.status = 502;
    nlohmann::json error_body;
    error_body["error_code"] = ERR_VENDOR_ERROR;
    error_body["vendor_code"] = "x-medkit-peer-unavailable";
    error_body["message"] = "Peer '" + peer_name + "' is not known to this gateway";
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Validate forwarded path - only allow SOVD API paths to prevent SSRF
  // to internal peer endpoints (e.g., /metrics, /debug, /admin). The path is
  // checked whether it came from the client or was built by a caller from
  // client-supplied ids; both are equally untrusted.
  if (target_path.rfind("/api/v1/", 0) != 0) {
    res.status = 400;
    nlohmann::json error_body;
    error_body["error_code"] = ERR_INVALID_REQUEST;
    error_body["message"] = "Forwarded request path must start with /api/v1/";
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Strip peer prefix from entity ID in the path if present.
  // When entity ID collision causes renaming (e.g., camera_driver -> peer_b__camera_driver),
  // the peer only knows the entity by its original ID (camera_driver), so we must strip
  // the prefix before forwarding.
  // Anchor to path segment boundary: the prefix must appear right after '/' to avoid
  // false matches inside other path segments (e.g., "v1" matching "/api/v1/").
  std::string forwarded_path = target_path;
  std::string prefix = peer_name + EntityMerger::SEPARATOR;
  auto prefix_pos = forwarded_path.find(prefix);
  if (prefix_pos != std::string::npos && prefix_pos > 0 && forwarded_path[prefix_pos - 1] == '/') {
    forwarded_path.erase(prefix_pos, prefix.size());
  }

  httplib::Request modified_req = req;
  modified_req.path = forwarded_path;

  peer->forward_request(modified_req, res);
}

AggregationManager::FanOutResult AggregationManager::fan_out_get(const std::string & path,
                                                                 const std::string & auth_header,
                                                                 const std::vector<std::string> * target_peers) {
  // Per-peer result collected by each async task
  struct PeerResult {
    std::string peer_name;
    bool success{false};
    /// Wire token for why this peer failed, from `peer_failure_reason`. Empty
    /// on success. Built here rather than discarded, which is what left every
    /// fan-out failure indistinguishable on the wire.
    std::string failure_reason;
    nlohmann::json items = nlohmann::json::array();
  };

  // A non-null, empty target list means "no peers match" - skip fan-out
  // entirely so callers don't see spurious partial flags. Keep the
  // merged_items invariant (always a JSON array, never null) for downstream
  // .is_array() checks.
  if (target_peers != nullptr && target_peers->empty()) {
    FanOutResult empty_result;
    empty_result.merged_items = nlohmann::json::array();
    return empty_result;
  }

  // Snapshot healthy peers under lock, release before network I/O.
  // When target_peers is non-null, restrict the snapshot to matching peers.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto & peer : peers_) {
      if (!peer->is_healthy()) {
        continue;
      }
      if (target_peers != nullptr &&
          std::find(target_peers->begin(), target_peers->end(), peer->name()) == target_peers->end()) {
        continue;
      }
      snapshot.push_back(peer);
    }
  }

  // Fan out GET requests in parallel via std::async to reduce worst-case
  // latency from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::vector<std::future<PeerResult>> futures;
  futures.reserve(snapshot.size());
  for (auto & peer : snapshot) {
    futures.push_back(std::async(std::launch::async, [peer, path, auth_header]() {
      PeerResult pr;
      pr.peer_name = peer->name();

      auto result = peer->forward_and_get_json("GET", path, auth_header, {{"X-Medkit-No-Fan-Out", "1"}});
      if (!result.has_value()) {
        pr.success = false;
        pr.failure_reason = peer_failure_reason(result.error().kind);
        return pr;
      }

      pr.success = true;
      const auto & response_json = result.value();
      if (response_json.contains("items") && response_json["items"].is_array()) {
        pr.items = response_json["items"];
      }
      return pr;
    }));
  }

  // Merge results from all peers
  FanOutResult fan_out_result;
  fan_out_result.merged_items = nlohmann::json::array();
  for (auto & f : futures) {
    auto pr = f.get();
    if (!pr.success) {
      fan_out_result.is_partial = true;
      fan_out_result.peer_failures.push_back(dto::PeerFailure{pr.peer_name, std::move(pr.failure_reason)});
      fan_out_result.failed_peers.push_back(std::move(pr.peer_name));
      continue;
    }
    for (auto & item : pr.items) {
      fan_out_result.merged_items.push_back(std::move(item));
      fan_out_result.item_peers.push_back(pr.peer_name);
    }
  }

  return fan_out_result;
}

nlohmann::json AggregationManager::get_peer_status() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  nlohmann::json status_array = nlohmann::json::array();
  for (const auto & peer : peers_) {
    nlohmann::json peer_obj;
    peer_obj["name"] = peer->name();
    peer_obj["url"] = peer->url();
    peer_obj["status"] = peer->is_healthy() ? "online" : "offline";
    status_array.push_back(peer_obj);
  }
  return status_array;
}

std::vector<PeerEndpoint> AggregationManager::healthy_peer_endpoints() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<PeerEndpoint> endpoints;
  for (const auto & peer : peers_) {
    if (peer->is_healthy()) {
      endpoints.push_back(PeerEndpoint{peer->name(), peer->url()});
    }
  }
  return endpoints;
}

PeerClient * AggregationManager::find_peer(const std::string & name) const {
  for (const auto & peer : peers_) {
    if (peer->name() == name) {
      return peer.get();
    }
  }
  return nullptr;
}

std::shared_ptr<PeerClient> AggregationManager::find_peer_shared(const std::string & name) const {
  for (const auto & peer : peers_) {
    if (peer->name() == name) {
      return peer;
    }
  }
  return nullptr;
}

}  // namespace ros2_medkit_gateway
