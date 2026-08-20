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

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"

using namespace ros2_medkit_gateway;  // NOLINT(google-build-using-namespace)
using nlohmann::json;

namespace {

// Tiny inline DTO used as the item type for fan_out_collection<T> tests.
// Two fields: required `id` and required `name`, both strings.
struct FanOutTestItem {
  std::string id;
  std::string name;
};

}  // namespace

template <>
[[maybe_unused]] inline constexpr auto ros2_medkit_gateway::dto::dto_fields<FanOutTestItem> =
    std::make_tuple(ros2_medkit_gateway::dto::field("id", &FanOutTestItem::id),
                    ros2_medkit_gateway::dto::field("name", &FanOutTestItem::name));

template <>
[[maybe_unused]] inline constexpr std::string_view ros2_medkit_gateway::dto::dto_name<FanOutTestItem> =
    "FanOutTestItem";

// =============================================================================
// url_encode_param
// =============================================================================

TEST(FanOutHelpers, url_encode_param_empty_string) {
  EXPECT_EQ(url_encode_param(""), "");
}

TEST(FanOutHelpers, url_encode_param_unreserved_chars_pass_through) {
  EXPECT_EQ(url_encode_param("abcXYZ019"), "abcXYZ019");
  EXPECT_EQ(url_encode_param("-_.~"), "-_.~");
}

TEST(FanOutHelpers, url_encode_param_encodes_space) {
  EXPECT_EQ(url_encode_param("hello world"), "hello%20world");
}

TEST(FanOutHelpers, url_encode_param_encodes_special_chars) {
  EXPECT_EQ(url_encode_param("a=b&c"), "a%3Db%26c");
  EXPECT_EQ(url_encode_param("100%"), "100%25");
  EXPECT_EQ(url_encode_param("foo+bar"), "foo%2Bbar");
  EXPECT_EQ(url_encode_param("/path/to"), "%2Fpath%2Fto");
}

TEST(FanOutHelpers, url_encode_param_encodes_unicode_bytes) {
  // UTF-8 for U+00E9 (e-acute) is 0xC3 0xA9
  std::string input = "\xC3\xA9";
  EXPECT_EQ(url_encode_param(input), "%C3%A9");
}

// =============================================================================
// build_fan_out_path
// =============================================================================

TEST(FanOutHelpers, build_fan_out_path_no_params) {
  httplib::Request req;
  req.path = "/api/v1/apps/temp_sensor/logs";
  EXPECT_EQ(build_fan_out_path(req), "/api/v1/apps/temp_sensor/logs");
}

TEST(FanOutHelpers, build_fan_out_path_single_param) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("severity", "error");
  EXPECT_EQ(build_fan_out_path(req), "/api/v1/apps/sensor/logs?severity=error");
}

TEST(FanOutHelpers, build_fan_out_path_multiple_params) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("context", "my.logger");
  req.params.emplace("severity", "warning");
  auto result = build_fan_out_path(req);
  // multimap iteration order is sorted by key
  EXPECT_TRUE(result.find("context=my.logger") != std::string::npos);
  EXPECT_TRUE(result.find("severity=warning") != std::string::npos);
  EXPECT_EQ(result[req.path.size()], '?');
}

TEST(FanOutHelpers, build_fan_out_path_encodes_special_values) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("context", "my logger/test");
  auto result = build_fan_out_path(req);
  EXPECT_TRUE(result.find("context=my%20logger%2Ftest") != std::string::npos);
}

TEST(FanOutHelpers, build_fan_out_path_encodes_keys) {
  httplib::Request req;
  req.path = "/api/v1/test";
  req.params.emplace("a=b", "val");
  auto result = build_fan_out_path(req);
  EXPECT_TRUE(result.find("a%3Db=val") != std::string::npos);
}

// =============================================================================
// merge_peer_items
// =============================================================================

TEST(FanOutHelpers, merge_peer_items_null_aggregation_manager_is_noop) {
  httplib::Request req;
  req.path = "/api/v1/test";
  json result;
  result["items"] = json::array({{"id", "local1"}});
  json ext;

  merge_peer_items(nullptr, req, result, ext);

  EXPECT_EQ(result["items"].size(), 1u);
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_skips_when_no_fan_out_header_set) {
  httplib::Request req;
  req.path = "/api/v1/test";
  req.headers.emplace("X-Medkit-No-Fan-Out", "1");
  json result;
  result["items"] = json::array();
  json ext;

  // Even with a non-null pointer, should skip due to header.
  // We pass a bogus pointer since it should never be dereferenced.
  auto * fake_agg = reinterpret_cast<AggregationManager *>(0x1);  // NOLINT
  merge_peer_items(fake_agg, req, result, ext);

  EXPECT_EQ(result["items"].size(), 0u);
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_null_agg_does_not_touch_result) {
  httplib::Request req;
  req.path = "/api/v1/test";
  json result;
  // No "items" key at all
  json ext;

  merge_peer_items(nullptr, req, result, ext);

  // null agg = no-op, result should be untouched
  EXPECT_FALSE(result.contains("items"));
}

// =============================================================================
// merge_peer_items with real AggregationManager + mock server
// =============================================================================

namespace {

class MockServer {
 public:
  ~MockServer() {
    if (server_) {
      server_->stop();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  MockServer(const MockServer &) = delete;
  MockServer & operator=(const MockServer &) = delete;
  MockServer(MockServer &&) = delete;
  MockServer & operator=(MockServer &&) = delete;
  MockServer() = default;

  httplib::Server & server() {
    if (!server_) {
      server_ = std::make_unique<httplib::Server>();
    }
    return *server_;
  }

  int start() {
    int port = server().bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this]() {
      server_->listen_after_bind();
    });
    // stop() only interrupts a server that is already listening.
    server_->wait_until_ready();
    return port;
  }

 private:
  std::unique_ptr<httplib::Server> server_;
  std::thread thread_;
};

}  // namespace

TEST(FanOutHelpers, merge_peer_items_appends_peer_items_to_result) {
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/functions/f1/logs", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_log_1"}}, {{"id", "peer_log_2"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "test_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();
  // Routing table maps the entity in the path to the peer; without this the
  // fan-out helper treats the entity as local-only and skips fan-out.
  agg.update_routing_table({{"f1", "test_peer"}});

  httplib::Request req;
  req.path = "/api/v1/functions/f1/logs";
  json result;
  result["items"] = json::array({{{"id", "local_log_1"}}});
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 3u);
  EXPECT_EQ(result["items"][0]["id"], "local_log_1");
  EXPECT_EQ(result["items"][1]["id"], "peer_log_1");
  EXPECT_EQ(result["items"][2]["id"], "peer_log_2");
  EXPECT_TRUE(ext.empty());  // no partial failure
}

TEST(FanOutHelpers, merge_peer_items_sets_partial_on_peer_failure) {
  // Mock server responds to health but has no handler for the actual
  // resource path, so the peer returns 404 (non-2xx = failure).
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 2000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "failing_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_routing_table({{"f1", "failing_peer"}});

  httplib::Request req;
  req.path = "/api/v1/functions/f1/logs";
  json result;
  result["items"] = json::array();
  json ext;

  merge_peer_items(&agg, req, result, ext);

  EXPECT_EQ(result["items"].size(), 0u);
  EXPECT_FALSE(ext.empty());
  EXPECT_TRUE(ext.value("partial", false));
  ASSERT_TRUE(ext.contains("failed_peers"));
  EXPECT_EQ(ext["failed_peers"].size(), 1u);
  EXPECT_EQ(ext["failed_peers"][0], "failing_peer");
}

TEST(FanOutHelpers, merge_peer_items_creates_items_when_missing_and_peer_has_data) {
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/apps/a/data", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_item"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_routing_table({{"a", "peer"}});

  httplib::Request req;
  req.path = "/api/v1/apps/a/data";
  json result;
  // No "items" key - merge_peer_items should create it
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_TRUE(result.contains("items"));
  ASSERT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "peer_item");
}

// =============================================================================
// extract_entity_id_for_fan_out
// =============================================================================

TEST(FanOutHelpers, extract_entity_id_components_collection) {
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/components/ecu-primary/logs"), "ecu-primary");
}

TEST(FanOutHelpers, extract_entity_id_apps_with_subpath) {
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/apps/helloApp/data/%2Fhello%2Fheartbeat"), "helloApp");
}

TEST(FanOutHelpers, extract_entity_id_areas_and_functions) {
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/areas/vehicle/faults"), "vehicle");
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/functions/perception/logs"), "perception");
}

TEST(FanOutHelpers, extract_entity_id_global_endpoints_return_nullopt) {
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/faults").has_value());
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/health").has_value());
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/").has_value());
}

TEST(FanOutHelpers, extract_entity_id_detail_endpoint_returns_nullopt) {
  // `/components/<id>` (no trailing collection suffix) is routed to the owning
  // peer via the forwarding path, not fan-out. The helper returns nullopt so
  // the caller falls through to the current global behavior.
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/components/ecu-primary").has_value());
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/apps/helloApp").has_value());
}

TEST(FanOutHelpers, extract_entity_id_trailing_slash_still_matches) {
  // A trailing `/` right after the collection segment is a valid per-entity
  // path (some clients or proxies append it). The next '/' after the id
  // still terminates the id, so extraction succeeds.
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/components/ecu-a/logs/"), "ecu-a");
  EXPECT_EQ(extract_entity_id_for_fan_out("/api/v1/apps/sensor/data/"), "sensor");
}

TEST(FanOutHelpers, extract_entity_id_empty_segment_returns_nullopt) {
  // Malformed path with an empty entity id (`//`) must not be treated as a
  // per-entity request - otherwise the helper would return an empty string
  // and the fan-out gate would look it up in the routing table.
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/components//logs").has_value());
  EXPECT_FALSE(extract_entity_id_for_fan_out("/api/v1/apps//data").has_value());
}

TEST(FanOutHelpers, extract_entity_id_ignores_query_string_because_httplib_splits_it) {
  // httplib splits `?...` off req.path before handlers see it, so the helper
  // never needs to strip query strings itself. The synthetic test request
  // (with query baked into req.path) also resolves correctly thanks to the
  // `/` boundary between the entity id and the collection segment.
  httplib::Request req;
  req.path = "/api/v1/apps/helloApp/data";  // what httplib would present
  EXPECT_EQ(extract_entity_id_for_fan_out(req.path), "helloApp");
}

// =============================================================================
// merge_peer_items - skip fan-out for local-only entities
// =============================================================================

TEST(FanOutHelpers, merge_peer_items_skips_fanout_when_entity_is_local_only) {
  // A local-only entity has no peer contributors: it is absent from both the
  // routing table and the peer-contributed-id set. Peers do not host it so
  // fan-out would always 404 and produce spurious `failed_peers`. Regression
  // test for the component-logs bug.
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/components/local-only-comp/logs", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 2000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "other_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_routing_table({{"something-else", "other_peer"}});

  httplib::Request req;
  req.path = "/api/v1/components/local-only-comp/logs";
  json result;
  result["items"] = json::array({{{"id", "local_log"}}});
  json ext;

  merge_peer_items(&agg, req, result, ext);

  EXPECT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "local_log");
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_fans_out_for_merged_entity_without_routing_entry) {
  // Merged/hierarchical entities (parent Components aggregated across peers,
  // or Areas/Functions with IDs shared across gateways) are served locally
  // and therefore have no routing-table entry, but peers still host runtime
  // state backing them. The helper must fan out in this case; it identifies
  // these entities via update_peer_contributors().
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/areas/root/logs", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_log"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();
  // "root" is not routed (merged Area) but was contributed by a peer.
  agg.update_peer_contributors({{"root", {"peer"}}});

  httplib::Request req;
  req.path = "/api/v1/areas/root/logs";
  json result;
  result["items"] = json::array({{{"id", "local_log"}}});
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 2u);
  EXPECT_EQ(result["items"][0]["id"], "local_log");
  EXPECT_EQ(result["items"][1]["id"], "peer_log");
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_fans_out_only_to_routed_leaf_owner) {
  // Routed leaf: three healthy peers configured, but only peer_owner hosts
  // the entity. peer_a and peer_c have no handler for the collection path
  // and would return 404 if asked. With target-filtered fan-out they are
  // never queried - confirmed by the absence of partial/failed_peers.
  MockServer owner;
  owner.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  owner.server().Get("/api/v1/apps/temp_sensor/logs", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "owner_log"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int owner_port = owner.start();

  MockServer other_a;
  other_a.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  int a_port = other_a.start();

  MockServer other_c;
  other_c.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  int c_port = other_c.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  for (const auto & [port, name] :
       std::vector<std::pair<int, std::string>>{{a_port, "peer_a"}, {owner_port, "peer_owner"}, {c_port, "peer_c"}}) {
    AggregationConfig::PeerConfig p;
    p.url = "http://127.0.0.1:" + std::to_string(port);
    p.name = name;
    config.peers.push_back(p);
  }

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_routing_table({{"temp_sensor", "peer_owner"}});

  httplib::Request req;
  req.path = "/api/v1/apps/temp_sensor/logs";
  json result;
  result["items"] = json::array();
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "owner_log");
  EXPECT_TRUE(ext.empty()) << "peer_a and peer_c must not be asked";
}

TEST(FanOutHelpers, merge_peer_items_fans_out_only_to_listed_contributors) {
  // Merged Area with contributors peer_a and peer_c but NOT peer_b. peer_b
  // has no handler and would return 404 if asked. Target-filter fan-out
  // must skip peer_b entirely; the result must merge only peer_a + peer_c
  // items with no partial flag.
  MockServer peer_a;
  peer_a.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  peer_a.server().Get("/api/v1/areas/vehicle/faults", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "a_fault"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int a_port = peer_a.start();

  MockServer peer_b;  // NOT a contributor - must not be queried.
  peer_b.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  int b_port = peer_b.start();

  MockServer peer_c;
  peer_c.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  peer_c.server().Get("/api/v1/areas/vehicle/faults", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "c_fault"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int c_port = peer_c.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  for (const auto & [port, name] :
       std::vector<std::pair<int, std::string>>{{a_port, "peer_a"}, {b_port, "peer_b"}, {c_port, "peer_c"}}) {
    AggregationConfig::PeerConfig p;
    p.url = "http://127.0.0.1:" + std::to_string(port);
    p.name = name;
    config.peers.push_back(p);
  }

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_peer_contributors({{"vehicle", {"peer_a", "peer_c"}}});

  httplib::Request req;
  req.path = "/api/v1/areas/vehicle/faults";
  json result;
  result["items"] = json::array();
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 2u);
  // Order is not guaranteed - std::async parallelism. Check presence.
  std::set<std::string> ids;
  for (const auto & item : result["items"]) {
    ids.insert(item["id"].get<std::string>());
  }
  EXPECT_EQ(ids, (std::set<std::string>{"a_fault", "c_fault"}));
  EXPECT_TRUE(ext.empty()) << "peer_b must not be asked, no partial should be set";
}

TEST(FanOutHelpers, merge_peer_items_partial_only_when_contributor_fails) {
  // Two contributor peers; one responds successfully, the other returns 500.
  // The result must be partial with failed_peers listing ONLY the failing
  // contributor - peers that were not queried (non-contributors) must not
  // appear in failed_peers.
  MockServer ok_peer;
  ok_peer.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  ok_peer.server().Get("/api/v1/components/cluster/logs", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "ok_log"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int ok_port = ok_peer.start();

  MockServer broken_peer;
  broken_peer.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  broken_peer.server().Get("/api/v1/components/cluster/logs", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
  });
  int broken_port = broken_peer.start();

  MockServer bystander;  // Not a contributor - must not end up in failed_peers.
  bystander.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 200;
  });
  int bystander_port = bystander.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  for (const auto & [port, name] : std::vector<std::pair<int, std::string>>{
           {ok_port, "peer_ok"}, {broken_port, "peer_broken"}, {bystander_port, "peer_bystander"}}) {
    AggregationConfig::PeerConfig p;
    p.url = "http://127.0.0.1:" + std::to_string(port);
    p.name = name;
    config.peers.push_back(p);
  }

  AggregationManager agg(config);
  agg.check_all_health();
  agg.update_peer_contributors({{"cluster", {"peer_ok", "peer_broken"}}});

  httplib::Request req;
  req.path = "/api/v1/components/cluster/logs";
  json result;
  result["items"] = json::array();
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "ok_log");
  ASSERT_FALSE(ext.empty());
  EXPECT_TRUE(ext.value("partial", false));
  ASSERT_TRUE(ext.contains("failed_peers"));
  ASSERT_EQ(ext["failed_peers"].size(), 1u);
  EXPECT_EQ(ext["failed_peers"][0], "peer_broken")
      << "peer_bystander was not a contributor and must not appear in failed_peers";
}

TEST(FanOutHelpers, merge_peer_items_fans_out_for_global_endpoints_without_entity) {
  // Paths with no entity id (like /faults, /health) keep fan-out-to-all.
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/faults", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_fault"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/faults";
  json result;
  result["items"] = json::array();
  json ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "peer_fault");
}

// =============================================================================
// fan_out_collection<T> - typed replacement for merge_peer_items
// =============================================================================

TEST(FanOutHelpers, fan_out_collection_null_agg_returns_empty) {
  httplib::Request req;
  req.path = "/api/v1/faults";
  auto result = fan_out_collection<FanOutTestItem>(nullptr, req);
  EXPECT_TRUE(result.items.empty());
  EXPECT_FALSE(result.partial);
  EXPECT_TRUE(result.failed_peers.empty());
  EXPECT_TRUE(result.dropped_items.empty());
}

TEST(FanOutHelpers, fan_out_collection_no_fan_out_header_returns_empty) {
  httplib::Request req;
  req.path = "/api/v1/faults";
  req.headers.emplace("X-Medkit-No-Fan-Out", "1");
  // Bogus pointer; must never be dereferenced because header short-circuits.
  auto * fake_agg = reinterpret_cast<AggregationManager *>(0x1);  // NOLINT
  auto result = fan_out_collection<FanOutTestItem>(fake_agg, req);
  EXPECT_TRUE(result.items.empty());
  EXPECT_FALSE(result.partial);
  EXPECT_TRUE(result.failed_peers.empty());
  EXPECT_TRUE(result.dropped_items.empty());
}

TEST(FanOutHelpers, fan_out_collection_no_healthy_peers_returns_empty) {
  // Aggregation configured but no peers -> healthy_peer_count() == 0.
  AggregationConfig config;
  config.enabled = true;
  AggregationManager agg(config);

  httplib::Request req;
  req.path = "/api/v1/faults";
  auto result = fan_out_collection<FanOutTestItem>(&agg, req);
  EXPECT_TRUE(result.items.empty());
  EXPECT_FALSE(result.partial);
  EXPECT_TRUE(result.failed_peers.empty());
  EXPECT_TRUE(result.dropped_items.empty());
}

TEST(FanOutHelpers, fan_out_collection_parses_typed_peer_items) {
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/faults", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "f1"}, {"name", "alpha"}}, {{"id", "f2"}, {"name", "beta"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/faults";
  auto result = fan_out_collection<FanOutTestItem>(&agg, req);

  ASSERT_EQ(result.items.size(), 2u);
  // Order is preserved by the merge step.
  EXPECT_EQ(result.items[0].id, "f1");
  EXPECT_EQ(result.items[0].name, "alpha");
  EXPECT_EQ(result.items[1].id, "f2");
  EXPECT_EQ(result.items[1].name, "beta");
  EXPECT_FALSE(result.partial);
  EXPECT_TRUE(result.failed_peers.empty());
  EXPECT_TRUE(result.dropped_items.empty());
}

TEST(FanOutHelpers, fan_out_collection_drops_malformed_items_into_dropped_items) {
  // One valid item (id + name) and one malformed item (missing required `name`)
  // come from the same peer response. The malformed entry must end up in
  // `dropped_items` with a non-empty reason and the best-effort source_id; the
  // valid entry must end up in `items` unaffected.
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/faults", [](const httplib::Request &, httplib::Response & res) {
    json body = {
        {"items", {{{"id", "good"}, {"name", "ok"}}, {{"id", "bad_no_name"}}}}};  // second item missing required `name`
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/faults";
  auto result = fan_out_collection<FanOutTestItem>(&agg, req);

  ASSERT_EQ(result.items.size(), 1u);
  EXPECT_EQ(result.items[0].id, "good");
  EXPECT_EQ(result.items[0].name, "ok");

  ASSERT_EQ(result.dropped_items.size(), 1u);
  const auto & dropped = result.dropped_items[0];
  // Per-peer attribution not threaded through yet; documented limitation.
  EXPECT_EQ(dropped.peer, "");
  // source_id is a best-effort lookup of common id keys on the raw JSON.
  EXPECT_EQ(dropped.source_id, "bad_no_name");
  // Reason aggregates JsonReader field errors. The exact text comes from
  // JsonReader; we only check it is non-empty and references the missing field.
  EXPECT_FALSE(dropped.reason.empty());
  EXPECT_NE(dropped.reason.find("name"), std::string::npos);

  EXPECT_FALSE(result.partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

TEST(FanOutHelpers, fan_out_collection_sets_partial_on_peer_failure) {
  // Peer healthy at /health but returns 500 on the target path -> failure.
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/faults", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 2000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "failing_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/faults";
  auto result = fan_out_collection<FanOutTestItem>(&agg, req);

  EXPECT_TRUE(result.items.empty());
  EXPECT_TRUE(result.dropped_items.empty());
  EXPECT_TRUE(result.partial);
  ASSERT_EQ(result.failed_peers.size(), 1u);
  EXPECT_EQ(result.failed_peers[0], "failing_peer");
}
