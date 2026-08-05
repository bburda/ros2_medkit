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

#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lock_handlers.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/dto/locks.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/alternate_status.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::ERR_INVALID_PARAMETER;
using ros2_medkit_gateway::ERR_INVALID_REQUEST;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::LockConfig;
using ros2_medkit_gateway::LockManager;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LockHandlers;
using ros2_medkit_gateway::http::TypedRequest;

namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

namespace {

// Manifest with components and apps for entity validation
const char * kManifestYaml = R"(
manifest_version: "1.0"
metadata:
  name: "lock-handlers-test"
  version: "1.0.0"
components:
  - id: "ecu1"
    name: "ECU 1"
    namespace: "/ecu1"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "ecu1"
)";

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }
  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

// PR-403 commit 18: typed handlers take TypedRequest, which wraps an
// httplib::Request. Populate the request's `path` + `matches` via
// std::regex_match so the typed handler's `path_param("N")` returns the
// Nth capture group exactly the way the production routing layer does.
void prime_request(httplib::Request & req, const std::string & path, const std::string & pattern) {
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
}

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_lock_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    return {};
  }
  close(fd);
  std::ofstream out(path_template);
  if (!out) {
    std::remove(path_template);
    return {};
  }
  out << contents;
  out.close();
  return path_template;
}

}  // namespace

// ============================================================================
// Static tests (no GatewayNode required)
// ============================================================================

TEST(LockHandlersStaticTest, LockingDisabledReturns501) {
  CorsConfig cors;
  AuthConfig auth;
  TlsConfig tls;
  HandlerContext ctx(nullptr, cors, auth, tls, nullptr);
  LockHandlers handlers(ctx, nullptr);

  httplib::Request req;
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;

  auto result = handlers.post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, "not-implemented");
}

// ============================================================================
// Full handler tests (with GatewayNode for entity validation)
// ============================================================================

class LockHandlersTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {"test_lock_handlers",
                                     "--ros-args",
                                     "-p",
                                     "server.port:=" + std::to_string(suite_server_port_),
                                     "-p",
                                     "refresh_interval_ms:=60000",
                                     "-p",
                                     "locking.enabled:=true",
                                     "-p",
                                     "locking.default_max_expiration:=3600"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }

    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    suite_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Write manifest and initialize discovery
    manifest_path_ = write_temp_manifest(kManifestYaml);
    ASSERT_FALSE(manifest_path_.empty());

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(suite_node_->get_discovery_manager()->initialize(config));

    auto areas = suite_node_->get_discovery_manager()->discover_areas();
    auto components = suite_node_->get_discovery_manager()->discover_components();
    auto apps = suite_node_->get_discovery_manager()->discover_apps();
    auto functions = suite_node_->get_discovery_manager()->discover_functions();

    // Set up apps with FQNs
    for (auto & app : apps) {
      if (app.id == "planner") {
        app.bound_fqn = "/ecu1/planner";
      }
    }

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    // Get the LockManager from the node
    lock_manager_ = suite_node_->get_lock_manager();
    ASSERT_NE(lock_manager_, nullptr) << "LockManager should be created when locking.enabled=true";

    ctx_ = std::make_unique<HandlerContext>(suite_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<LockHandlers>(*ctx_, lock_manager_);
  }

  void TearDown() override {
    handlers_.reset();
    ctx_.reset();

    // Clean up any locks from previous test
    if (lock_manager_) {
      // Release locks by acquiring and releasing for known entities
      for (const auto & eid : {"ecu1", "planner"}) {
        auto lock = lock_manager_->get_lock(eid);
        if (lock) {
          // Force-acquire to take ownership, then release
          (void)lock_manager_->acquire(eid, "__cleanup__", {}, 1, true);
          (void)lock_manager_->release(eid, "__cleanup__");
        }
      }
    }

    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  // Helper: prime a POST /api/v1/components/{id}/locks request
  void make_component_locks_request(httplib::Request & req, const std::string & component_id) {
    prime_request(req, "/api/v1/components/" + component_id + "/locks", R"(/api/v1/components/([^/]+)/locks)");
  }

  // Helper: prime a POST /api/v1/apps/{id}/locks request
  void make_app_locks_request(httplib::Request & req, const std::string & app_id) {
    prime_request(req, "/api/v1/apps/" + app_id + "/locks", R"(/api/v1/apps/([^/]+)/locks)");
  }

  // Helper: prime a GET/PUT/DELETE /api/v1/components/{id}/locks/{lock_id} request
  void make_component_lock_item_request(httplib::Request & req, const std::string & component_id,
                                        const std::string & lock_id) {
    prime_request(req, "/api/v1/components/" + component_id + "/locks/" + lock_id,
                  R"(/api/v1/components/([^/]+)/locks/([^/]+))");
  }

  // Helper: prime a GET/PUT/DELETE /api/v1/apps/{id}/locks/{lock_id} request
  void make_app_lock_item_request(httplib::Request & req, const std::string & app_id, const std::string & lock_id) {
    prime_request(req, "/api/v1/apps/" + app_id + "/locks/" + lock_id, R"(/api/v1/apps/([^/]+)/locks/([^/]+))");
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  LockManager * lock_manager_ = nullptr;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<LockHandlers> handlers_;
  std::string manifest_path_;
};

// ============================================================================
// POST acquire lock
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockOnComponentReturns201) {
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_TRUE(result.has_value());
  // 201 is declared by the Created<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result->first)>::value, 201);
  EXPECT_FALSE(result->second.status_override.has_value());
  // Location header set to <request-path>/<lock-id>
  ASSERT_FALSE(result->second.headers.empty());
  EXPECT_EQ(result->second.headers[0].first, "Location");

  const auto & lock = result->first.value;
  EXPECT_FALSE(lock.id.empty());
  EXPECT_TRUE(lock.owned);
  EXPECT_FALSE(lock.lock_expiration.empty());
  EXPECT_TRUE(lock.lock_expiration.find('T') != std::string::npos);
  EXPECT_TRUE(lock.lock_expiration.find('Z') != std::string::npos);
  // No scopes specified - field should be absent
  EXPECT_FALSE(lock.scopes.has_value());

  // Confirm Location header value is consistent with the lock id.
  EXPECT_EQ(result->second.headers[0].second, req.path + "/" + lock.id);
}

TEST_F(LockHandlersTest, AcquireLockOnAppReturns201) {
  httplib::Request req;
  make_app_locks_request(req, "planner");
  req.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 600;
  body.scopes = std::vector<std::string>{"configurations"};

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_TRUE(result.has_value());
  // 201 is declared by the Created<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result->first)>::value, 201);
  EXPECT_FALSE(result->second.status_override.has_value());

  const auto & lock = result->first.value;
  EXPECT_FALSE(lock.id.empty());
  EXPECT_TRUE(lock.owned);
  ASSERT_TRUE(lock.scopes.has_value());
  ASSERT_EQ(lock.scopes->size(), 1u);
  EXPECT_EQ((*lock.scopes)[0], "configurations");
  EXPECT_FALSE(lock.lock_expiration.empty());
}

TEST_F(LockHandlersTest, AcquireLockWithoutClientIdReturns400) {
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  // No X-Client-Id header
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

TEST_F(LockHandlersTest, AcquireLockWithInvalidScopeReturns400) {
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;
  body.scopes = std::vector<std::string>{"invalid_scope"};

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

TEST_F(LockHandlersTest, AcquireLockWithMissingExpirationParsedAsZeroReturns400) {
  // The legacy test sent body `{"scopes": ["configurations"]}` and relied on
  // ctx_.parse_body<AcquireLockRequest> emitting ERR_INVALID_REQUEST when the
  // required `lock_expiration` field was missing. With typed router commit 18
  // the body is parsed by the framework BEFORE the handler runs, so missing-
  // -field errors no longer reach the handler under test. The handler-side
  // check for `lock_expiration <= 0` is still exercised here by emitting a
  // body with the field defaulted to zero (matching the framework's behavior
  // on missing fields for required ints).
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;  // lock_expiration defaults to 0

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

TEST_F(LockHandlersTest, AcquireLockWithZeroExpirationReturns400) {
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 0;

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

TEST_F(LockHandlersTest, AcquireLockAlreadyLockedReturns409) {
  // First acquire succeeds
  httplib::Request req1;
  make_component_locks_request(req1, "ecu1");
  req1.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req1(req1);
  dto::AcquireLockRequest body1;
  body1.lock_expiration = 300;

  auto res1 = handlers_->post_lock(typed_req1, body1);
  ASSERT_TRUE(res1.has_value());

  // Second acquire by different client should fail
  httplib::Request req2;
  make_component_locks_request(req2, "ecu1");
  req2.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req2(req2);
  dto::AcquireLockRequest body2;
  body2.lock_expiration = 300;

  auto res2 = handlers_->post_lock(typed_req2, body2);
  ASSERT_FALSE(res2.has_value());
  EXPECT_EQ(res2.error().http_status, 409);
  EXPECT_EQ(res2.error().code, ERR_INVALID_REQUEST);
}

TEST_F(LockHandlersTest, AcquireLockOnNonexistentEntityReturns404) {
  httplib::Request req;
  make_component_locks_request(req, "nonexistent_component");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, "entity-not-found");
}

TEST_F(LockHandlersTest, AcquireLockBodyParsingIsFrameworkLevel) {
  // The legacy test sent `req.body = "not json"` and expected
  // handle_acquire_lock to emit a 400. With the typed router, malformed JSON
  // is rejected by the framework's JsonReader BEFORE the handler runs, so
  // the body-level malformed-JSON case is now covered by typed-router-level
  // tests (see test_typed_route_registry). Here we just confirm the handler
  // path still rejects a body with `lock_expiration=0` (after the framework
  // has successfully parsed the body), since that is the only check the
  // handler itself owns now.
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 0;

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// ============================================================================
// GET list locks
// ============================================================================

TEST_F(LockHandlersTest, ListLocksReturnsItemsArray) {
  // Acquire a lock first
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  acquire_body.scopes = std::vector<std::string>{"configurations"};
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());

  // List locks
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);

  auto result = handlers_->get_locks(typed_req);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->items.size(), 1u);

  const auto & lock_item = result->items[0];
  EXPECT_FALSE(lock_item.id.empty());
  EXPECT_TRUE(lock_item.owned);
  ASSERT_TRUE(lock_item.scopes.has_value());
  ASSERT_EQ(lock_item.scopes->size(), 1u);
  EXPECT_EQ((*lock_item.scopes)[0], "configurations");
  // Verify lock_expiration is ISO 8601
  EXPECT_TRUE(lock_item.lock_expiration.find('T') != std::string::npos);
  EXPECT_TRUE(lock_item.lock_expiration.find('Z') != std::string::npos);
}

TEST_F(LockHandlersTest, ListLocksOwnedFieldReflectsClient) {
  // Acquire a lock
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());

  // List as same client - should be owned
  httplib::Request req1;
  make_component_locks_request(req1, "ecu1");
  req1.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req1(req1);
  auto res1 = handlers_->get_locks(typed_req1);
  ASSERT_TRUE(res1.has_value());
  ASSERT_EQ(res1->items.size(), 1u);
  EXPECT_TRUE(res1->items[0].owned);

  // List as different client - should not be owned
  httplib::Request req2;
  make_component_locks_request(req2, "ecu1");
  req2.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req2(req2);
  auto res2 = handlers_->get_locks(typed_req2);
  ASSERT_TRUE(res2.has_value());
  ASSERT_EQ(res2->items.size(), 1u);
  EXPECT_FALSE(res2->items[0].owned);

  // List without client id - should not be owned
  httplib::Request req3;
  make_component_locks_request(req3, "ecu1");
  TypedRequest typed_req3(req3);
  auto res3 = handlers_->get_locks(typed_req3);
  ASSERT_TRUE(res3.has_value());
  ASSERT_EQ(res3->items.size(), 1u);
  EXPECT_FALSE(res3->items[0].owned);
}

TEST_F(LockHandlersTest, ListLocksEmptyWhenNoLocks) {
  httplib::Request req;
  make_component_locks_request(req, "ecu1");
  TypedRequest typed_req(req);

  auto result = handlers_->get_locks(typed_req);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->items.empty());
}

// ============================================================================
// GET single lock
// ============================================================================

TEST_F(LockHandlersTest, GetLockReturns200) {
  // Acquire a lock
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Get the lock
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);

  auto result = handlers_->get_lock(typed_req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, lock_id);
  EXPECT_TRUE(result->owned);
  EXPECT_FALSE(result->lock_expiration.empty());
}

TEST_F(LockHandlersTest, GetLockNonExistentReturns404) {
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", "lock_nonexistent");
  TypedRequest typed_req(req);

  auto result = handlers_->get_lock(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, "resource-not-found");
}

// ============================================================================
// PUT extend lock
// ============================================================================

TEST_F(LockHandlersTest, ExtendLockReturns204) {
  // Acquire a lock
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Extend
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::ExtendLockRequest body;
  body.lock_expiration = 600;

  auto result = handlers_->put_lock(typed_req, body);
  EXPECT_TRUE(result.has_value()) << "extend lock should succeed";
}

TEST_F(LockHandlersTest, ExtendLockNotOwnerReturns403) {
  // Acquire a lock as client_a
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Try to extend as client_b - should fail with 403
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  req.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req(req);
  dto::ExtendLockRequest body;
  body.lock_expiration = 600;

  auto result = handlers_->put_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 403);
  EXPECT_EQ(result.error().code, "forbidden");
}

TEST_F(LockHandlersTest, ExtendLockWithoutClientIdReturns400) {
  // Acquire a lock
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // No X-Client-Id header
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  TypedRequest typed_req(req);
  dto::ExtendLockRequest body;
  body.lock_expiration = 600;

  auto result = handlers_->put_lock(typed_req, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

// ============================================================================
// DELETE release lock
// ============================================================================

TEST_F(LockHandlersTest, ReleaseLockReturns204) {
  // Acquire a lock
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Release
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);

  auto result = handlers_->del_lock(typed_req);
  EXPECT_TRUE(result.has_value());

  // Verify lock is gone
  auto check = lock_manager_->get_lock("ecu1");
  EXPECT_FALSE(check.has_value());
}

TEST_F(LockHandlersTest, ReleaseLockNotOwnerReturns403) {
  // Acquire a lock as client_a
  httplib::Request acquire_req;
  make_component_locks_request(acquire_req, "ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Try to release as client_b - should fail with 403
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", lock_id);
  req.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req(req);

  auto result = handlers_->del_lock(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 403);
  EXPECT_EQ(result.error().code, "forbidden");
}

TEST_F(LockHandlersTest, ReleaseLockNonexistentReturns404) {
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", "lock_nonexistent");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);

  auto result = handlers_->del_lock(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, "resource-not-found");
}

TEST_F(LockHandlersTest, ReleaseLockWithoutClientIdReturns400) {
  httplib::Request req;
  make_component_lock_item_request(req, "ecu1", "some_lock");
  // No X-Client-Id
  TypedRequest typed_req(req);

  auto result = handlers_->del_lock(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_INVALID_PARAMETER);
}

// ============================================================================
// App path tests (dual-path validation)
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockOnAppPathReturns201) {
  httplib::Request req;
  make_app_locks_request(req, "planner");
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::AcquireLockRequest body;
  body.lock_expiration = 300;
  body.scopes = std::vector<std::string>{"operations", "configurations"};

  auto result = handlers_->post_lock(typed_req, body);
  ASSERT_TRUE(result.has_value());
  // 201 is declared by the Created<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result->first)>::value, 201);
  EXPECT_FALSE(result->second.status_override.has_value());
  const auto & lock = result->first.value;
  EXPECT_TRUE(lock.owned);
  ASSERT_TRUE(lock.scopes.has_value());
  EXPECT_EQ(lock.scopes->size(), 2u);
}

TEST_F(LockHandlersTest, ListLocksOnAppPathReturns200) {
  // Acquire first
  httplib::Request acquire_req;
  make_app_locks_request(acquire_req, "planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());

  // List
  httplib::Request req;
  make_app_locks_request(req, "planner");
  TypedRequest typed_req(req);

  auto result = handlers_->get_locks(typed_req);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->items.size(), 1u);
}

TEST_F(LockHandlersTest, GetLockOnAppPathReturns200) {
  // Acquire
  httplib::Request acquire_req;
  make_app_locks_request(acquire_req, "planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Get
  httplib::Request req;
  make_app_lock_item_request(req, "planner", lock_id);
  TypedRequest typed_req(req);

  auto result = handlers_->get_lock(typed_req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, lock_id);
}

TEST_F(LockHandlersTest, ExtendLockOnAppPathReturns204) {
  // Acquire
  httplib::Request acquire_req;
  make_app_locks_request(acquire_req, "planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Extend
  httplib::Request req;
  make_app_lock_item_request(req, "planner", lock_id);
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);
  dto::ExtendLockRequest body;
  body.lock_expiration = 600;

  auto result = handlers_->put_lock(typed_req, body);
  EXPECT_TRUE(result.has_value());
}

TEST_F(LockHandlersTest, ReleaseLockOnAppPathReturns204) {
  // Acquire
  httplib::Request acquire_req;
  make_app_locks_request(acquire_req, "planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  TypedRequest acquire_typed(acquire_req);
  dto::AcquireLockRequest acquire_body;
  acquire_body.lock_expiration = 300;
  auto acquire_res = handlers_->post_lock(acquire_typed, acquire_body);
  ASSERT_TRUE(acquire_res.has_value());
  const std::string lock_id = acquire_res->first.value.id;

  // Release
  httplib::Request req;
  make_app_lock_item_request(req, "planner", lock_id);
  req.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req(req);

  auto result = handlers_->del_lock(typed_req);
  EXPECT_TRUE(result.has_value());
}

// ============================================================================
// Break lock test
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockWithBreakReplacesExisting) {
  // First acquire
  httplib::Request req1;
  make_component_locks_request(req1, "ecu1");
  req1.set_header("X-Client-Id", "client_a");
  TypedRequest typed_req1(req1);
  dto::AcquireLockRequest body1;
  body1.lock_expiration = 300;
  auto res1 = handlers_->post_lock(typed_req1, body1);
  ASSERT_TRUE(res1.has_value());

  // Break and acquire by different client
  httplib::Request req2;
  make_component_locks_request(req2, "ecu1");
  req2.set_header("X-Client-Id", "client_b");
  TypedRequest typed_req2(req2);
  dto::AcquireLockRequest body2;
  body2.lock_expiration = 600;
  body2.break_lock = true;
  auto res2 = handlers_->post_lock(typed_req2, body2);

  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(http::dto_alternate_status<decltype(res2->first)>::value, 201);
  EXPECT_FALSE(res2->second.status_override.has_value());
  EXPECT_TRUE(res2->first.value.owned);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
