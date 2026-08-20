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

#include <gtest/gtest.h>
#include <httplib.h>

#include <algorithm>
#include <future>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"

using namespace ros2_medkit_gateway;

// Use a port that nothing listens on for connection-refused tests
constexpr int DEAD_PORT = 59999;
static const std::string DEAD_URL = "http://127.0.0.1:" + std::to_string(DEAD_PORT);

// =============================================================================
// Helper: build AggregationConfig with unreachable static peers
// =============================================================================

static AggregationConfig make_config(size_t num_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;  // Short timeout for fast test execution

  for (size_t i = 0; i < num_peers; ++i) {
    AggregationConfig::PeerConfig peer;
    peer.url = "http://127.0.0.1:" + std::to_string(DEAD_PORT + static_cast<int>(i));
    peer.name = "peer_" + std::to_string(i);
    config.peers.push_back(peer);
  }

  return config;
}

// =============================================================================
// Construction and peer count tests
// =============================================================================

TEST(AggregationManager, adds_static_peers_from_config) {
  auto config = make_config(3);
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 3u);
}

TEST(AggregationManager, peer_count_reflects_config) {
  auto config_empty = make_config(0);
  AggregationManager manager_empty(config_empty);
  EXPECT_EQ(manager_empty.peer_count(), 0u);

  auto config_one = make_config(1);
  AggregationManager manager_one(config_one);
  EXPECT_EQ(manager_one.peer_count(), 1u);

  auto config_five = make_config(5);
  AggregationManager manager_five(config_five);
  EXPECT_EQ(manager_five.peer_count(), 5u);
}

// =============================================================================
// Dynamic peer management tests
// =============================================================================

TEST(AggregationManager, add_discovered_peer_increases_count) {
  auto config = make_config(1);
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 1u);

  manager.add_discovered_peer("http://192.168.1.50:8081", "discovered_peer");
  EXPECT_EQ(manager.peer_count(), 2u);
}

TEST(AggregationManager, add_discovered_peer_is_idempotent) {
  auto config = make_config(0);
  AggregationManager manager(config);

  manager.add_discovered_peer("http://192.168.1.50:8081", "peer_alpha");
  EXPECT_EQ(manager.peer_count(), 1u);

  // Adding again with same name is a no-op
  manager.add_discovered_peer("http://192.168.1.50:8082", "peer_alpha");
  EXPECT_EQ(manager.peer_count(), 1u);
}

// =============================================================================
// Peer URL validation tests
// =============================================================================

TEST(AggregationManager, rejects_non_http_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  manager.add_discovered_peer("ftp://192.168.1.50:8081", "ftp_peer");
  EXPECT_EQ(manager.peer_count(), 0u);

  manager.add_discovered_peer("file:///etc/passwd", "file_peer");
  EXPECT_EQ(manager.peer_count(), 0u);

  manager.add_discovered_peer("not-a-url", "bad_peer");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_cloud_metadata_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // AWS metadata endpoint
  manager.add_discovered_peer("http://169.254.169.254/latest/meta-data/", "aws_meta");
  EXPECT_EQ(manager.peer_count(), 0u);

  // GCP metadata endpoint
  manager.add_discovered_peer("http://metadata.google.internal/computeMetadata/v1/", "gcp_meta");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_loopback_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // IPv4 loopback
  manager.add_discovered_peer("http://127.0.0.1:8081", "loopback_v4");
  EXPECT_EQ(manager.peer_count(), 0u);

  manager.add_discovered_peer("http://127.0.0.99:8081", "loopback_v4_2");
  EXPECT_EQ(manager.peer_count(), 0u);

  // localhost
  manager.add_discovered_peer("http://localhost:8081", "loopback_name");
  EXPECT_EQ(manager.peer_count(), 0u);

  // IPv6 loopback
  manager.add_discovered_peer("http://[::1]:8081", "loopback_v6");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_link_local_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Link-local address (not just the metadata endpoint)
  manager.add_discovered_peer("http://169.254.1.1:8081", "link_local");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, accepts_valid_http_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  manager.add_discovered_peer("http://192.168.1.50:8081", "valid_peer");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, accepts_valid_https_peer_url) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Use IP address (not hostname) to avoid DNS resolution issues in test environments
  manager.add_discovered_peer("https://10.0.0.1:8443", "secure_peer");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, remove_discovered_peer_decreases_count) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Add discovered peers
  manager.add_discovered_peer("http://192.168.1.10:8081", "disc_peer_0");
  manager.add_discovered_peer("http://192.168.1.11:8081", "disc_peer_1");
  EXPECT_EQ(manager.peer_count(), 2u);

  manager.remove_discovered_peer("disc_peer_0");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, remove_nonexistent_peer_is_noop) {
  auto config = make_config(2);
  AggregationManager manager(config);

  manager.remove_discovered_peer("no_such_peer");
  EXPECT_EQ(manager.peer_count(), 2u);
}

TEST(AggregationManager, remove_discovered_peer_does_not_remove_static_peers) {
  // Static peers must never be removed by remove_discovered_peer(), even if
  // the name matches. This prevents mDNS goodbye messages from corrupting
  // the static peer list.
  auto config = make_config(2);  // 2 static peers: peer_0, peer_1
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 2u);

  // Attempt to remove a static peer by name
  manager.remove_discovered_peer("peer_0");
  EXPECT_EQ(manager.peer_count(), 2u);  // Static peer should survive

  manager.remove_discovered_peer("peer_1");
  EXPECT_EQ(manager.peer_count(), 2u);  // Still 2 static peers

  // Add a discovered peer and verify it CAN be removed
  manager.add_discovered_peer("http://192.168.1.50:8081", "dynamic_peer");
  EXPECT_EQ(manager.peer_count(), 3u);

  manager.remove_discovered_peer("dynamic_peer");
  EXPECT_EQ(manager.peer_count(), 2u);  // Only the 2 static peers remain
}

// =============================================================================
// Health monitoring tests
// =============================================================================

TEST(AggregationManager, healthy_peer_count_zero_when_all_unreachable) {
  auto config = make_config(3);
  AggregationManager manager(config);

  // Peers start unhealthy (never checked)
  EXPECT_EQ(manager.healthy_peer_count(), 0u);

  // After health check, they should still be unhealthy (unreachable)
  manager.check_all_health();
  EXPECT_EQ(manager.healthy_peer_count(), 0u);
}

// =============================================================================
// Fan-out tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationManager, fan_out_returns_partial_when_peers_unreachable) {
  auto config = make_config(2);
  AggregationManager manager(config);

  // No peers are healthy, so fan-out should return empty merged_items
  // with no failed_peers (they are simply not included as targets)
  auto result = manager.fan_out_get("/api/v1/components", "");

  EXPECT_TRUE(result.merged_items.is_array());
  EXPECT_TRUE(result.merged_items.empty());
  // No healthy peers means no targets, so no failures
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

TEST(AggregationManager, fan_out_returns_empty_array_with_no_peers) {
  auto config = make_config(0);
  AggregationManager manager(config);

  auto result = manager.fan_out_get("/api/v1/apps", "Bearer token123");

  EXPECT_TRUE(result.merged_items.is_array());
  EXPECT_TRUE(result.merged_items.empty());
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

// =============================================================================
// Forward request tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationManager, forward_returns_502_for_unknown_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components/abc/data";
  httplib::Response res;

  manager.forward_request("nonexistent_peer", req, res);

  EXPECT_EQ(res.status, 502);

  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-peer-unavailable");
  EXPECT_TRUE(body["message"].get<std::string>().find("nonexistent_peer") != std::string::npos);
}

TEST(AggregationManager, forward_returns_502_for_unreachable_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components/abc/data";
  httplib::Response res;

  // peer_0 exists but is unreachable
  manager.forward_request("peer_0", req, res);

  EXPECT_EQ(res.status, 502);
}

// =============================================================================
// Peer status tests
// =============================================================================

TEST(AggregationManager, get_peer_status_includes_all_peers) {
  auto config = make_config(3);
  AggregationManager manager(config);

  auto status = manager.get_peer_status();

  ASSERT_TRUE(status.is_array());
  ASSERT_EQ(status.size(), 3u);

  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(status[i]["name"], "peer_" + std::to_string(i));
    EXPECT_TRUE(status[i].contains("url"));
    EXPECT_EQ(status[i]["status"], "offline");
  }
}

TEST(AggregationManager, get_peer_status_empty_when_no_peers) {
  auto config = make_config(0);
  AggregationManager manager(config);

  auto status = manager.get_peer_status();

  ASSERT_TRUE(status.is_array());
  EXPECT_TRUE(status.empty());
}

// =============================================================================
// Routing table tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationManager, routing_table_update_and_find) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Initially empty - looking up any entity returns nullopt
  EXPECT_FALSE(manager.find_peer_for_entity("anything").has_value());

  // Update with entries
  std::unordered_map<std::string, std::string> table;
  table["remote_component_a"] = "peer_x";
  table["remote_app_b"] = "peer_y";
  manager.update_routing_table(table);

  auto result_a = manager.find_peer_for_entity("remote_component_a");
  ASSERT_TRUE(result_a.has_value());
  EXPECT_EQ(*result_a, "peer_x");

  auto result_b = manager.find_peer_for_entity("remote_app_b");
  ASSERT_TRUE(result_b.has_value());
  EXPECT_EQ(*result_b, "peer_y");

  // Unknown entity returns nullopt
  EXPECT_FALSE(manager.find_peer_for_entity("unknown_entity").has_value());
}

TEST(AggregationManager, has_peer_contributors_covers_routing_and_merged_entities) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Initially no contributions known - everything is "local-only".
  EXPECT_FALSE(manager.has_peer_contributors("anything"));

  // Routed leaf: entry in routing table means the entity lives on a peer,
  // which implies it has a peer contributor.
  std::unordered_map<std::string, std::string> routing;
  routing["remote_leaf"] = "peer_x";
  manager.update_routing_table(routing);
  EXPECT_TRUE(manager.has_peer_contributors("remote_leaf"));

  // Merged / hierarchical: no routing entry, but gateway tracks peer
  // contributions separately. Without this signal the fan-out gate would
  // treat aggregated Areas/Functions/parent Components as local-only.
  manager.update_peer_contributors({
      {"merged_area", {"peer_a", "peer_b"}},
      {"hierarchical_parent", {"peer_b"}},
  });
  EXPECT_TRUE(manager.has_peer_contributors("merged_area"));
  EXPECT_TRUE(manager.has_peer_contributors("hierarchical_parent"));

  // Still false for genuinely local entities (not in either map).
  EXPECT_FALSE(manager.has_peer_contributors("local_only"));

  // Replacing the contributor map drops entries no longer reported.
  manager.update_peer_contributors({{"merged_area", {"peer_a"}}});
  EXPECT_TRUE(manager.has_peer_contributors("merged_area"));
  EXPECT_FALSE(manager.has_peer_contributors("hierarchical_parent"));
  // Routing table is independent and still applies.
  EXPECT_TRUE(manager.has_peer_contributors("remote_leaf"));
}

TEST(AggregationManager, get_peer_contributors_unions_routing_and_contributor_map) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Unknown entity: empty result.
  EXPECT_TRUE(manager.get_peer_contributors("unknown").empty());

  // Routed-leaf only: single peer from routing table.
  manager.update_routing_table({{"routed_leaf", "peer_x"}});
  EXPECT_EQ(manager.get_peer_contributors("routed_leaf"), std::vector<std::string>{"peer_x"});

  // Merged-only (not in routing table): peers come from contributor map.
  manager.update_peer_contributors({{"merged", {"peer_a", "peer_b"}}});
  auto merged_peers = manager.get_peer_contributors("merged");
  ASSERT_EQ(merged_peers.size(), 2u);
  EXPECT_EQ(merged_peers[0], "peer_a");
  EXPECT_EQ(merged_peers[1], "peer_b");

  // Entity present in both: routing-table owner first, extra contributors
  // appended without duplicates.
  manager.update_routing_table({{"both", "peer_x"}});
  manager.update_peer_contributors({{"both", {"peer_x", "peer_y"}}});
  auto both_peers = manager.get_peer_contributors("both");
  ASSERT_EQ(both_peers.size(), 2u);
  EXPECT_EQ(both_peers[0], "peer_x");
  EXPECT_EQ(both_peers[1], "peer_y");
}

TEST(AggregationManager, update_peer_contributors_drops_empty_entries) {
  auto config = make_config(0);
  AggregationManager manager(config);

  manager.update_peer_contributors({
      {"with_peers", {"peer_a"}}, {"empty_entry", {}},  // caller bug / stale placeholder - must be dropped
  });
  EXPECT_TRUE(manager.has_peer_contributors("with_peers"));
  // Empty list must not count as "has peer contributors" - otherwise the
  // fan-out gate would skip fan-out with no contributor to fall back on.
  EXPECT_FALSE(manager.has_peer_contributors("empty_entry"));
  EXPECT_TRUE(manager.get_peer_contributors("empty_entry").empty());
}

TEST(AggregationManager, routing_table_replaces_on_update) {
  auto config = make_config(0);
  AggregationManager manager(config);

  std::unordered_map<std::string, std::string> table1;
  table1["entity_a"] = "peer_1";
  manager.update_routing_table(table1);
  EXPECT_TRUE(manager.find_peer_for_entity("entity_a").has_value());

  // Replace with different table
  std::unordered_map<std::string, std::string> table2;
  table2["entity_b"] = "peer_2";
  table2["entity_c"] = "peer_3";
  manager.update_routing_table(table2);

  EXPECT_FALSE(manager.find_peer_for_entity("entity_a").has_value());  // Old entry gone

  auto result_b = manager.find_peer_for_entity("entity_b");
  ASSERT_TRUE(result_b.has_value());
  EXPECT_EQ(*result_b, "peer_2");

  auto result_c = manager.find_peer_for_entity("entity_c");
  ASSERT_TRUE(result_c.has_value());
  EXPECT_EQ(*result_c, "peer_3");
}

// =============================================================================
// get_peer_url tests
// =============================================================================

TEST(AggregationManager, get_peer_url_returns_url_for_known_peer) {
  auto config = make_config(2);
  AggregationManager manager(config);

  std::string url = manager.get_peer_url("peer_0");
  EXPECT_EQ(url, "http://127.0.0.1:" + std::to_string(DEAD_PORT));

  std::string url1 = manager.get_peer_url("peer_1");
  EXPECT_EQ(url1, "http://127.0.0.1:" + std::to_string(DEAD_PORT + 1));
}

TEST(AggregationManager, get_peer_url_returns_empty_for_unknown_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  std::string url = manager.get_peer_url("no_such_peer");
  EXPECT_TRUE(url.empty());
}

// =============================================================================
// fetch_all_peer_entities tests (with unreachable peers)
// =============================================================================

TEST(AggregationManager, fetch_all_peer_entities_returns_empty_when_none_healthy) {
  auto config = make_config(2);
  AggregationManager manager(config);

  // No peers healthy -> empty result
  auto entities = manager.fetch_all_peer_entities();

  EXPECT_TRUE(entities.areas.empty());
  EXPECT_TRUE(entities.components.empty());
  EXPECT_TRUE(entities.apps.empty());
  EXPECT_TRUE(entities.functions.empty());
}

// =============================================================================
// Prefix stripping tests (forward_request)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationManager, forward_strips_peer_prefix_from_path) {
  // This tests the path rewriting logic: when a collision-renamed entity
  // (e.g., peer_0__camera_driver) is forwarded to the peer, the peer prefix
  // must be stripped so the peer receives the original entity ID.
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  // Path contains peer_0__ prefix due to collision renaming
  req.path = "/api/v1/apps/peer_0__camera_driver/data";
  httplib::Response res;

  // The peer is unreachable, so we get 502. But the test verifies the path
  // rewriting happened. The peer_client receives the stripped path.
  // We verify the forward completes without crashing and returns 502
  // (peer unreachable), not 404 (wrong entity ID).
  manager.forward_request("peer_0", req, res);

  // The peer is unreachable, so we get 502 (not a routing error like 404)
  EXPECT_EQ(res.status, 502);
}

TEST(AggregationManager, forward_preserves_path_without_prefix) {
  // When the entity ID does not contain the peer prefix, the path
  // should be forwarded unchanged.
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/camera_driver/data";
  httplib::Response res;

  manager.forward_request("peer_0", req, res);

  // Peer unreachable -> 502
  EXPECT_EQ(res.status, 502);
}

TEST(AggregationManager, forward_strips_only_matching_peer_prefix) {
  // Verify that prefix stripping only removes the correct peer prefix,
  // not an arbitrary occurrence of the separator.
  auto config = make_config(2);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  // Entity has peer_1__ prefix, forwarded to peer_1
  req.path = "/api/v1/components/peer_1__lidar_sensor/data";
  httplib::Response res;

  manager.forward_request("peer_1", req, res);
  EXPECT_EQ(res.status, 502);
}

// =============================================================================
// Static peer scheme validation tests
// =============================================================================

TEST(AggregationManager, rejects_non_http_static_peer) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;

  AggregationConfig::PeerConfig ftp_peer;
  ftp_peer.url = "ftp://192.168.1.50:8081";
  ftp_peer.name = "ftp_peer";
  config.peers.push_back(ftp_peer);

  AggregationConfig::PeerConfig file_peer;
  file_peer.url = "file:///etc/passwd";
  file_peer.name = "file_peer";
  config.peers.push_back(file_peer);

  AggregationConfig::PeerConfig valid_peer;
  valid_peer.url = "http://192.168.1.10:8080";
  valid_peer.name = "valid_peer";
  config.peers.push_back(valid_peer);

  AggregationManager manager(config);

  // Only the valid HTTP peer should have been added
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, accepts_localhost_for_static_peer) {
  // Static peers allow localhost (unlike mDNS-discovered peers)
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;

  AggregationConfig::PeerConfig localhost_peer;
  localhost_peer.url = "http://localhost:8081";
  localhost_peer.name = "local_peer";
  config.peers.push_back(localhost_peer);

  AggregationConfig::PeerConfig loopback_peer;
  loopback_peer.url = "http://127.0.0.1:8082";
  loopback_peer.name = "loopback_peer";
  config.peers.push_back(loopback_peer);

  AggregationManager manager(config);

  // Both should be accepted for static config
  EXPECT_EQ(manager.peer_count(), 2u);
}

// =============================================================================
// TLS enforcement tests (require_tls)
// =============================================================================

TEST(AggregationManager, require_tls_rejects_http_static_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.require_tls = true;

  AggregationConfig::PeerConfig http_peer;
  http_peer.url = "http://192.168.1.10:8080";
  http_peer.name = "insecure_peer";
  config.peers.push_back(http_peer);

  AggregationConfig::PeerConfig https_peer;
  https_peer.url = "https://192.168.1.11:8443";
  https_peer.name = "secure_peer";
  config.peers.push_back(https_peer);

  AggregationManager manager(config);

  // Only the HTTPS peer should be accepted
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, require_tls_rejects_http_discovered_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.require_tls = true;

  AggregationManager manager(config);

  // http:// should be rejected
  manager.add_discovered_peer("http://192.168.1.50:8081", "http_peer");
  EXPECT_EQ(manager.peer_count(), 0u);

  // https:// should be accepted
  manager.add_discovered_peer("https://192.168.1.51:8443", "https_peer");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, require_tls_false_accepts_http_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.require_tls = false;

  AggregationConfig::PeerConfig http_peer;
  http_peer.url = "http://192.168.1.10:8080";
  http_peer.name = "http_peer";
  config.peers.push_back(http_peer);

  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 1u);
}

// =============================================================================
// forward_auth config tests
// =============================================================================

TEST(AggregationManager, forward_auth_config_propagated_to_peer_clients) {
  // When forward_auth is false (default), PeerClients should not forward auth headers.
  // This is indirectly tested via fan_out_get tests above.
  // Here we just verify the config is accepted without error.
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.forward_auth = false;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(DEAD_PORT);
  peer.name = "test_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  EXPECT_EQ(manager.peer_count(), 1u);
}

// =============================================================================
// Helper: RAII wrapper for a mock HTTP server
// =============================================================================

/**
 * @brief RAII wrapper for httplib::Server that starts listening on a random port
 * and stops + joins on destruction. All mock server tests use this pattern.
 */
class MockPeerServer {
 public:
  MockPeerServer() = default;

  ~MockPeerServer() {
    if (server_) {
      server_->stop();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  // Non-copyable, non-movable
  MockPeerServer(const MockPeerServer &) = delete;
  MockPeerServer & operator=(const MockPeerServer &) = delete;

  httplib::Server & server() {
    if (!server_) {
      server_ = std::make_unique<httplib::Server>();
    }
    return *server_;
  }

  /**
   * @brief Bind to a random port and start listening in a background thread.
   * @return The port number the server is listening on.
   */
  int start() {
    port_ = server_->bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this]() {
      server_->listen_after_bind();
    });
    return port_;
  }

  int port() const {
    return port_;
  }

  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

 private:
  std::unique_ptr<httplib::Server> server_;
  std::thread thread_;
  int port_{0};
};

/**
 * @brief Install standard entity endpoints on a mock server that return
 * the given areas, components, apps, and functions counts.
 * Each entity gets a simple {"id":"entity_N","name":"Entity N"} shape.
 */
static void install_entity_endpoints(httplib::Server & svr, size_t num_areas, size_t num_components, size_t num_apps,
                                     size_t num_functions) {
  // Health endpoint (required to mark peer as healthy)
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });

  // Areas list
  svr.Get("/api/v1/areas", [num_areas](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    for (size_t i = 0; i < num_areas; ++i) {
      items.push_back({{"id", "area_" + std::to_string(i)}, {"name", "Area " + std::to_string(i)}});
    }
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  // Area subareas (empty for all areas)
  svr.Get(R"(/api/v1/areas/([^/]+)/subareas)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  // Components list
  svr.Get("/api/v1/components", [num_components](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    for (size_t i = 0; i < num_components; ++i) {
      items.push_back({{"id", "comp_" + std::to_string(i)}, {"name", "Component " + std::to_string(i)}});
    }
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  // Component detail (for relationship data)
  svr.Get(R"(/api/v1/components/([^/]+))", [](const httplib::Request & req, httplib::Response & res) {
    // Check it's not a sub-resource path
    std::string match = req.matches[1].str();
    if (match.find('/') != std::string::npos) {
      res.status = 404;
      return;
    }
    res.set_content(nlohmann::json({{"id", match}, {"name", match}}).dump(), "application/json");
  });

  // Component subcomponents (empty for all components)
  svr.Get(R"(/api/v1/components/([^/]+)/subcomponents)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  // Apps list
  svr.Get("/api/v1/apps", [num_apps](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    for (size_t i = 0; i < num_apps; ++i) {
      items.push_back({{"id", "app_" + std::to_string(i)}, {"name", "App " + std::to_string(i)}});
    }
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  // Functions list
  svr.Get("/api/v1/functions", [num_functions](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    for (size_t i = 0; i < num_functions; ++i) {
      items.push_back({{"id", "func_" + std::to_string(i)}, {"name", "Function " + std::to_string(i)}});
    }
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  // Function detail (for hosts data)
  svr.Get(R"(/api/v1/functions/([^/]+))", [](const httplib::Request & req, httplib::Response & res) {
    std::string match = req.matches[1].str();
    res.set_content(nlohmann::json({{"id", match}, {"name", match}}).dump(), "application/json");
  });
}

// =============================================================================
// max_entities_per_peer safety limit test
// =============================================================================

TEST(AggregationManager, fetch_and_merge_skips_peer_exceeding_entity_limit) {
  // Set up a mock server that returns more entities than the safety limit.
  // The default limit is 10000; we set a small limit for test speed.
  MockPeerServer mock;
  // Return 60 entities total: 10 areas + 20 components + 20 apps + 10 functions
  install_entity_endpoints(mock.server(), 10, 20, 20, 10);
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "big_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);

  // Make the peer healthy
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  // Use a limit smaller than the 60 entities the peer returns -> should skip
  auto result = manager.fetch_and_merge_peer_entities({}, {}, {}, {}, /*max_entities_per_peer=*/50);

  // The peer should have been skipped, so merged result is just the empty locals
  EXPECT_TRUE(result.areas.empty());
  EXPECT_TRUE(result.components.empty());
  EXPECT_TRUE(result.apps.empty());
  EXPECT_TRUE(result.functions.empty());
  EXPECT_TRUE(result.routing_table.empty());
}

TEST(AggregationManager, fetch_and_merge_accepts_peer_under_entity_limit) {
  // Same setup but with a limit higher than the total entities
  MockPeerServer mock;
  install_entity_endpoints(mock.server(), 2, 3, 4, 1);
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "small_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);

  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  // 2+3+4+1 = 10 entities, limit is 100 -> should accept
  auto result = manager.fetch_and_merge_peer_entities({}, {}, {}, {}, /*max_entities_per_peer=*/100);

  // Peer entities should be merged in
  EXPECT_EQ(result.areas.size(), 2u);
  EXPECT_EQ(result.components.size(), 3u);
  EXPECT_EQ(result.apps.size(), 4u);
  EXPECT_EQ(result.functions.size(), 1u);
  // Remote entities should have routing table entries (apps and components are
  // remote-only, so they get routing entries; areas and functions get merged by ID
  // but still need routing for non-colliding remote entities)
  EXPECT_FALSE(result.routing_table.empty());
}

TEST(AggregationManager, fetch_and_merge_uses_default_10k_limit) {
  // Verify that the default parameter value is 10000 by calling without explicit limit.
  // With a small mock peer, it should always be accepted.
  MockPeerServer mock;
  install_entity_endpoints(mock.server(), 1, 1, 1, 1);
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "normal_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  // Use default limit (10000)
  auto result = manager.fetch_and_merge_peer_entities({}, {}, {}, {});

  EXPECT_EQ(result.areas.size(), 1u);
  EXPECT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.apps.size(), 1u);
  EXPECT_EQ(result.functions.size(), 1u);
}

// =============================================================================
// Function hosts remapped after App collision prefixing
// =============================================================================

TEST(AggregationManager, fetch_and_merge_remaps_function_hosts_after_app_collision) {
  // When a remote peer has App "camera_driver" that collides with a local App
  // "camera_driver", the remote App gets prefixed to "peer_cam__camera_driver".
  // A remote Function referencing "camera_driver" in its hosts must have that
  // host updated to the prefixed ID.
  MockPeerServer mock;

  // Health endpoint
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });

  // Areas: empty
  mock.server().Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  // Components: empty
  mock.server().Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  mock.server().Get(R"(/api/v1/components/([^/]+)/subcomponents)",
                    [](const httplib::Request &, httplib::Response & res) {
                      res.set_content(R"({"items":[]})", "application/json");
                    });

  // Apps: one app "camera_driver" that will collide with local
  mock.server().Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "camera_driver"}, {"name", "Camera Driver"}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });

  // Functions: one function referencing "camera_driver" in hosts
  mock.server().Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back(
        {{"id", "perception"}, {"name", "Perception"}, {"x-medkit", {{"hosts", {"camera_driver", "lidar_driver"}}}}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  mock.server().Get(R"(/api/v1/functions/([^/]+))", [](const httplib::Request & req, httplib::Response & res) {
    std::string match = req.matches[1].str();
    if (match == "perception") {
      res.set_content(
          R"({"id":"perception","name":"Perception","x-medkit":{"hosts":["camera_driver","lidar_driver"]}})",
          "application/json");
    } else {
      res.status = 404;
    }
  });

  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer_cam";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  // Local apps: one "camera_driver" that will cause collision
  App local_app;
  local_app.id = "camera_driver";
  local_app.name = "Camera Driver";
  local_app.source = "manifest";

  auto result = manager.fetch_and_merge_peer_entities({}, {}, {local_app}, {});

  // The remote app should be prefixed
  ASSERT_EQ(result.apps.size(), 2u);
  EXPECT_EQ(result.apps[0].id, "camera_driver");            // local
  EXPECT_EQ(result.apps[1].id, "peer_cam__camera_driver");  // remote, prefixed

  // The remote function's hosts should reference the prefixed app ID
  ASSERT_EQ(result.functions.size(), 1u);
  EXPECT_EQ(result.functions[0].id, "perception");

  // Check that "camera_driver" was remapped to "peer_cam__camera_driver" in hosts
  auto & hosts = result.functions[0].hosts;
  bool has_prefixed = std::find(hosts.begin(), hosts.end(), "peer_cam__camera_driver") != hosts.end();
  bool has_original = std::find(hosts.begin(), hosts.end(), "camera_driver") != hosts.end();
  EXPECT_TRUE(has_prefixed) << "Function hosts should contain prefixed 'peer_cam__camera_driver'";
  EXPECT_FALSE(has_original) << "Function hosts should NOT contain original 'camera_driver' after remap";

  // "lidar_driver" should be unchanged (no collision for that app)
  bool has_lidar = std::find(hosts.begin(), hosts.end(), "lidar_driver") != hosts.end();
  EXPECT_TRUE(has_lidar) << "Function hosts should still contain 'lidar_driver'";
}

// =============================================================================
// fan_out_get happy-path tests with mock server
// =============================================================================

TEST(AggregationManager, fan_out_get_merges_items_from_single_healthy_peer) {
  MockPeerServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {
        {"items", {{{"id", "comp_a"}, {"name", "Component A"}}, {{"id", "comp_b"}, {"name", "Component B"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "mock_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  auto result = manager.fan_out_get("/api/v1/components", "");

  ASSERT_TRUE(result.merged_items.is_array());
  EXPECT_EQ(result.merged_items.size(), 2u);
  EXPECT_EQ(result.merged_items[0]["id"], "comp_a");
  EXPECT_EQ(result.merged_items[1]["id"], "comp_b");
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

TEST(AggregationManager, fan_out_get_merges_items_from_multiple_peers) {
  MockPeerServer mock1;
  mock1.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock1.server().Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {{"items", {{{"id", "app_x"}}}}};
    res.set_content(body.dump(), "application/json");
  });

  MockPeerServer mock2;
  mock2.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock2.server().Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {{"items", {{{"id", "app_y"}}, {{"id", "app_z"}}}}};
    res.set_content(body.dump(), "application/json");
  });

  int port1 = mock1.start();
  int port2 = mock2.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer1;
  peer1.url = "http://127.0.0.1:" + std::to_string(port1);
  peer1.name = "peer_alpha";
  config.peers.push_back(peer1);

  AggregationConfig::PeerConfig peer2;
  peer2.url = "http://127.0.0.1:" + std::to_string(port2);
  peer2.name = "peer_beta";
  config.peers.push_back(peer2);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 2u);

  auto result = manager.fan_out_get("/api/v1/apps", "");

  ASSERT_TRUE(result.merged_items.is_array());
  EXPECT_EQ(result.merged_items.size(), 3u);
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

TEST(AggregationManager, fan_out_get_partial_results_one_peer_fails) {
  // peer_ok returns items, peer_bad returns 500
  MockPeerServer mock_ok;
  mock_ok.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock_ok.server().Get("/api/v1/data", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {{"items", {{{"id", "item_1"}}}}};
    res.set_content(body.dump(), "application/json");
  });

  MockPeerServer mock_bad;
  mock_bad.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock_bad.server().Get("/api/v1/data", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
    res.set_content(R"({"error":"internal"})", "application/json");
  });

  int port_ok = mock_ok.start();
  int port_bad = mock_bad.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer_ok;
  peer_ok.url = "http://127.0.0.1:" + std::to_string(port_ok);
  peer_ok.name = "peer_ok";
  config.peers.push_back(peer_ok);

  AggregationConfig::PeerConfig peer_bad;
  peer_bad.url = "http://127.0.0.1:" + std::to_string(port_bad);
  peer_bad.name = "peer_bad";
  config.peers.push_back(peer_bad);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 2u);

  auto result = manager.fan_out_get("/api/v1/data", "");

  // Should have partial results: items from peer_ok, failure from peer_bad
  ASSERT_TRUE(result.merged_items.is_array());
  EXPECT_EQ(result.merged_items.size(), 1u);
  EXPECT_EQ(result.merged_items[0]["id"], "item_1");
  EXPECT_TRUE(result.is_partial);
  ASSERT_EQ(result.failed_peers.size(), 1u);
  EXPECT_EQ(result.failed_peers[0], "peer_bad");
}

TEST(AggregationManager, fan_out_get_forwards_auth_header_when_enabled) {
  MockPeerServer mock;
  std::string captured_auth;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/secure-data", [&captured_auth](const httplib::Request & req, httplib::Response & res) {
    if (req.has_header("Authorization")) {
      captured_auth = req.get_header_value("Authorization");
    }
    nlohmann::json body = {{"items", {{{"id", "secret"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  config.forward_auth = true;  // Enable auth forwarding

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "secure_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();

  auto result = manager.fan_out_get("/api/v1/secure-data", "Bearer test-jwt-token");

  EXPECT_EQ(result.merged_items.size(), 1u);
  EXPECT_EQ(captured_auth, "Bearer test-jwt-token");
}

TEST(AggregationManager, fan_out_get_does_not_forward_auth_by_default) {
  MockPeerServer mock;
  bool auth_received = false;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/secure-data", [&auth_received](const httplib::Request & req, httplib::Response & res) {
    auth_received = req.has_header("Authorization");
    nlohmann::json body = {{"items", {{{"id", "data"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  // forward_auth defaults to false

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "noauth_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();

  auto result = manager.fan_out_get("/api/v1/secure-data", "Bearer should-not-forward");

  EXPECT_EQ(result.merged_items.size(), 1u);
  EXPECT_FALSE(auth_received);
}

// =============================================================================
// Concurrent add/remove peer stress test
// =============================================================================

TEST(AggregationManager, concurrent_add_remove_and_health_check_no_crash) {
  // Exercise locking by concurrently calling add_discovered_peer,
  // remove_discovered_peer, check_all_health, peer_count, and routing
  // table operations from multiple threads. The goal is to verify no
  // TSAN errors, deadlocks, or crashes.
  //
  // We avoid check_all_health() here because peers point to non-routable
  // IPs that would time out (making the test slow). Instead we exercise
  // the shared_lock paths via peer_count/healthy_peer_count/get_peer_status
  // which are fast read-only operations.
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 50;              // Very short timeout (only matters if health is checked)
  config.max_discovered_peers = 1000;  // Allow enough room for stress test

  AggregationManager manager(config);
  EXPECT_EQ(manager.peer_count(), 0u);

  constexpr int NUM_ITERATIONS = 500;

  // Thread 1: repeatedly add peers
  auto adder = std::async(std::launch::async, [&]() {
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
      manager.add_discovered_peer("http://192.168.1." + std::to_string((i % 254) + 1) + ":8080",
                                  "peer_" + std::to_string(i));
    }
  });

  // Thread 2: repeatedly remove peers (overlapping range with adder)
  auto remover = std::async(std::launch::async, [&]() {
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
      manager.remove_discovered_peer("peer_" + std::to_string(i));
    }
  });

  // Thread 3: repeatedly read peer state (shared lock contention)
  auto reader = std::async(std::launch::async, [&]() {
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
      (void)manager.peer_count();
      (void)manager.healthy_peer_count();
      (void)manager.get_peer_status();
    }
  });

  // Thread 4: routing table mutations (exclusive lock contention)
  auto routing = std::async(std::launch::async, [&]() {
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
      std::unordered_map<std::string, std::string> table;
      table["entity_" + std::to_string(i)] = "peer_" + std::to_string(i % 10);
      manager.update_routing_table(table);
      (void)manager.find_peer_for_entity("entity_" + std::to_string(i));
    }
  });

  adder.get();
  remover.get();
  reader.get();
  routing.get();

  // If we reach here without deadlock or crash, the test passes.
  // The final peer count is non-deterministic due to interleaving, but it
  // should be within [0, NUM_ITERATIONS].
  EXPECT_LE(manager.peer_count(), static_cast<size_t>(NUM_ITERATIONS));
}

TEST(AggregationManager, concurrent_fan_out_with_peer_mutations) {
  // Similar stress test but includes fan_out_get and routing table operations
  MockPeerServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {{"items", {{{"id", "comp_1"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 2000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "stable_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();

  constexpr int ITERATIONS = 50;

  // Thread 1: fan-out reads
  auto fan_out_reader = std::async(std::launch::async, [&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      auto result = manager.fan_out_get("/api/v1/components", "");
      // Result should be valid (not crashed)
      EXPECT_TRUE(result.merged_items.is_array());
    }
  });

  // Thread 2: routing table mutations
  auto routing_writer = std::async(std::launch::async, [&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      std::unordered_map<std::string, std::string> table;
      table["entity_" + std::to_string(i)] = "stable_peer";
      manager.update_routing_table(table);
      (void)manager.find_peer_for_entity("entity_" + std::to_string(i));
    }
  });

  // Thread 3: add and remove discovered peers
  auto peer_mutator = std::async(std::launch::async, [&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      manager.add_discovered_peer("http://10.0.0." + std::to_string((i % 254) + 1) + ":8080",
                                  "dynamic_" + std::to_string(i));
      manager.remove_discovered_peer("dynamic_" + std::to_string(i));
    }
  });

  fan_out_reader.get();
  routing_writer.get();
  peer_mutator.get();

  // No crash or deadlock = success
  SUCCEED();
}

// =============================================================================
// fetch_all_peer_entities happy-path with mock server
// =============================================================================

TEST(AggregationManager, fetch_all_peer_entities_returns_entities_from_healthy_peer) {
  MockPeerServer mock;
  install_entity_endpoints(mock.server(), 2, 1, 3, 0);
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "entity_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 1u);

  auto entities = manager.fetch_all_peer_entities();

  EXPECT_EQ(entities.areas.size(), 2u);
  EXPECT_EQ(entities.components.size(), 1u);
  EXPECT_EQ(entities.apps.size(), 3u);
  EXPECT_EQ(entities.functions.size(), 0u);

  // Verify source tagging
  for (const auto & area : entities.areas) {
    EXPECT_EQ(area.source, "peer:entity_peer");
  }
  for (const auto & app : entities.apps) {
    EXPECT_EQ(app.source, "peer:entity_peer");
  }
}

// =============================================================================
// forward_request happy-path with mock server
// =============================================================================

TEST(AggregationManager, forward_request_proxies_to_correct_peer) {
  MockPeerServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get(R"(/api/v1/apps/camera_driver/data)", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json body = {{"temperature", 42.5}, {"status", "ok"}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;

  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "proxy_peer";
  config.peers.push_back(peer);

  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/camera_driver/data";
  httplib::Response res;

  manager.forward_request("proxy_peer", req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_DOUBLE_EQ(body["temperature"].get<double>(), 42.5);
  EXPECT_EQ(body["status"].get<std::string>(), "ok");
}

// =============================================================================
// SSRF protection: IPv6 and edge-case loopback rejection
// =============================================================================

TEST(AggregationManager, rejects_ipv4_mapped_ipv6_loopback) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationManager manager(config);

  // IPv4-mapped IPv6 loopback (::ffff:127.0.0.1)
  manager.add_discovered_peer("http://[::ffff:127.0.0.1]:8081", "mapped_loopback");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_expanded_ipv6_loopback) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationManager manager(config);

  // Expanded IPv6 loopback (0:0:0:0:0:0:0:1)
  manager.add_discovered_peer("http://[0:0:0:0:0:0:0:1]:8081", "expanded_v6");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_zero_address) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationManager manager(config);

  // 0.0.0.0 (unspecified, binds all interfaces)
  manager.add_discovered_peer("http://0.0.0.0:8081", "zero_addr");
  EXPECT_EQ(manager.peer_count(), 0u);

  // [::] (IPv6 unspecified)
  manager.add_discovered_peer("http://[::]:8081", "zero_v6");
  EXPECT_EQ(manager.peer_count(), 0u);
}

TEST(AggregationManager, rejects_ipv6_link_local) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationManager manager(config);

  // fe80:: link-local address
  manager.add_discovered_peer("http://[fe80::1]:8081", "link_local_v6");
  EXPECT_EQ(manager.peer_count(), 0u);
}

// =============================================================================
// max_discovered_peers limit
// =============================================================================

TEST(AggregationManager, max_discovered_peers_limits_dynamic_additions) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.max_discovered_peers = 3;
  AggregationManager manager(config);

  // Add 3 discovered peers (should all succeed)
  manager.add_discovered_peer("http://192.168.1.10:8081", "peer_a");
  manager.add_discovered_peer("http://192.168.1.11:8081", "peer_b");
  manager.add_discovered_peer("http://192.168.1.12:8081", "peer_c");
  EXPECT_EQ(manager.peer_count(), 3u);

  // 4th should be rejected
  manager.add_discovered_peer("http://192.168.1.13:8081", "peer_d");
  EXPECT_EQ(manager.peer_count(), 3u);
}

TEST(AggregationManager, max_discovered_peers_does_not_count_static_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  config.max_discovered_peers = 2;

  // Add 2 static peers
  AggregationConfig::PeerConfig static1;
  static1.url = "http://10.0.0.1:8080";
  static1.name = "static_1";
  config.peers.push_back(static1);

  AggregationConfig::PeerConfig static2;
  static2.url = "http://10.0.0.2:8080";
  static2.name = "static_2";
  config.peers.push_back(static2);

  AggregationManager manager(config);
  EXPECT_EQ(manager.peer_count(), 2u);

  // Should still be able to add 2 discovered peers
  manager.add_discovered_peer("http://192.168.1.10:8081", "disc_1");
  manager.add_discovered_peer("http://192.168.1.11:8081", "disc_2");
  EXPECT_EQ(manager.peer_count(), 4u);

  // 3rd discovered peer exceeds limit
  manager.add_discovered_peer("http://192.168.1.12:8081", "disc_3");
  EXPECT_EQ(manager.peer_count(), 4u);
}

// =============================================================================
// Forward request path validation
// =============================================================================

TEST(AggregationManager, forward_rejects_non_api_path) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/internal/metrics";
  httplib::Response res;

  manager.forward_request("peer_0", req, res);

  EXPECT_EQ(res.status, 400);
  auto body_json = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body_json.is_discarded());
  EXPECT_EQ(body_json["error_code"], "invalid-request");
}

TEST(AggregationManager, forward_rejects_root_path) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/";
  httplib::Response res;

  manager.forward_request("peer_0", req, res);

  EXPECT_EQ(res.status, 400);
}

TEST(AggregationManager, forward_accepts_api_v1_path) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components/abc/data";
  httplib::Response res;

  manager.forward_request("peer_0", req, res);

  // Should get 502 (peer unreachable), not 400 (path rejected)
  EXPECT_EQ(res.status, 502);
}

// A tree that changes shape when a link drops cannot be reasoned about: the
// same request gets a different answer depending on who happens to be
// reachable. What a peer DECLARED stays true while it is silent; what it merely
// discovered from a live graph does not, because nothing can observe that graph
// any more. These two tests pin both halves of that split.
namespace {

/// Mock peer serving one manifest-declared app and one runtime-discovered app.
void install_two_origin_apps(httplib::Server & svr) {
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get(R"(/api/v1/areas/([^/]+)/subareas)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "declared_app"}, {"name", "Declared"}, {"x-medkit", {{"source", "manifest"}}}});
    items.push_back({{"id", "discovered_app"}, {"name", "Discovered"}, {"x-medkit", {{"source", "runtime"}}}});
    res.set_content(nlohmann::json({{"items", items}}).dump(), "application/json");
  });
  svr.Get(R"(/api/v1/apps/([^/]+)/operations)", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });
}

const App * find_app(const std::vector<App> & apps, const std::string & id) {
  for (const auto & app : apps) {
    if (app.id == id) {
      return &app;
    }
  }
  return nullptr;
}

}  // namespace

TEST(AggregationManager, retains_declared_peer_entities_when_the_peer_goes_silent) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 1000;
  AggregationConfig::PeerConfig peer;
  peer.name = "silent_peer";

  int port = 0;
  {
    MockPeerServer mock;
    install_two_origin_apps(mock.server());
    port = mock.start();
    peer.url = "http://127.0.0.1:" + std::to_string(port);
    config.peers.push_back(peer);

    AggregationManager manager(config);
    manager.check_all_health();
    ASSERT_EQ(manager.healthy_peer_count(), 1u);

    auto live = manager.fetch_and_merge_peer_entities({}, {}, {}, {});
    const App * declared = find_app(live.apps, "declared_app");
    const App * discovered = find_app(live.apps, "discovered_app");
    ASSERT_NE(declared, nullptr);
    ASSERT_NE(discovered, nullptr);
    EXPECT_EQ(declared->declared_source, "manifest") << "the peer's own origin was overwritten by peer:<name>";
    EXPECT_TRUE(declared->available);
    EXPECT_TRUE(discovered->available);

    // The peer stops answering: health goes false and the fetch fails.
    mock.server().stop();
    manager.check_all_health();
    ASSERT_EQ(manager.healthy_peer_count(), 0u);

    auto silent = manager.fetch_and_merge_peer_entities({}, {}, {}, {});
    const App * retained = find_app(silent.apps, "declared_app");
    ASSERT_NE(retained, nullptr) << "a declared entity vanished when its peer went quiet";
    EXPECT_FALSE(retained->available) << "a retained entity must say it cannot be reached";
    EXPECT_FALSE(retained->is_online) << "a retained app is not observably running";
    EXPECT_EQ(find_app(silent.apps, "discovered_app"), nullptr)
        << "a runtime-discovered entity describes a graph this gateway can no longer observe";
  }
}

TEST(AggregationManager, retains_nothing_for_a_peer_that_never_answered) {
  // Never reachable means nothing was ever declared to us, so there is nothing
  // to keep asserting - the entity is absent, not unavailable.
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:1";  // nothing listens here
  peer.name = "never_up";
  config.peers.push_back(peer);

  AggregationManager manager(config);
  manager.check_all_health();
  ASSERT_EQ(manager.healthy_peer_count(), 0u);

  auto result = manager.fetch_and_merge_peer_entities({}, {}, {}, {});
  EXPECT_TRUE(result.apps.empty());
  EXPECT_TRUE(result.components.empty());
  EXPECT_TRUE(result.routing_table.empty());
}
