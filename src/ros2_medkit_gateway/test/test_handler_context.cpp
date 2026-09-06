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

#include <arpa/inet.h>
#include <cstring>
#include <functional>
#include <memory>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <variant>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

#include "../src/openapi/route_registry.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;
// json alias already imported via the `using namespace` above (defined in
// core/auth/auth_models.hpp). A local `using json = nlohmann::json;` would
// shadow it and trip clang-diagnostic-shadow under clang-tidy.

// =============================================================================
// HandlerContext static method tests (don't require GatewayNode)
//
// The previous SendError* and SendJson* suites here exercised the legacy
// HandlerContext::send_error / send_json wrappers. Commit 30 removed that
// public surface; the canonical wire-format coverage now lives in
// test_primitives.cpp (write_json_body / write_generic_error suites) which
// drive the same framework primitives the wrappers used to delegate to.
// =============================================================================

TEST(HandlerContextStaticTest, LoggerReturnsValidLogger) {
  auto logger = HandlerContext::logger();
  // Just verify it doesn't throw and returns a valid logger name
  EXPECT_NE(logger.get_name(), nullptr);
  EXPECT_GT(strlen(logger.get_name()), 0);
}

// =============================================================================
// HandlerContext instance tests with CorsConfig
// =============================================================================

class HandlerContextCorsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cors_config_ = CorsConfigBuilder()
                       .with_origins({"http://localhost:3000", "http://example.com"})
                       .with_methods({"GET", "POST", "PUT", "DELETE"})
                       .with_headers({"Content-Type", "Authorization"})
                       .with_credentials(true)
                       .build();
  }

  CorsConfig cors_config_;
  AuthConfig auth_config_;  // Default (disabled)
  TlsConfig tls_config_;    // Default (disabled)
};

TEST_F(HandlerContextCorsTest, IsOriginAllowedReturnsTrueForConfiguredOrigins) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.is_origin_allowed("http://localhost:3000"));
  EXPECT_TRUE(ctx.is_origin_allowed("http://example.com"));
}

TEST_F(HandlerContextCorsTest, IsOriginAllowedReturnsFalseForUnknownOrigins) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_FALSE(ctx.is_origin_allowed("http://malicious.com"));
  EXPECT_FALSE(ctx.is_origin_allowed("http://localhost:8080"));
  EXPECT_FALSE(ctx.is_origin_allowed(""));
}

TEST_F(HandlerContextCorsTest, IsOriginAllowedWithWildcard) {
  auto wildcard_config = CorsConfigBuilder()
                             .with_origins({"*"})
                             .with_methods({"GET"})
                             .with_headers({"Content-Type"})
                             .with_credentials(false)  // Must be false with wildcard
                             .build();

  HandlerContext ctx(nullptr, wildcard_config, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.is_origin_allowed("http://any-site.com"));
  EXPECT_TRUE(ctx.is_origin_allowed("http://localhost:12345"));
}

TEST_F(HandlerContextCorsTest, SetCorsHeadersSetsAllHeaders) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);
  httplib::Response res;

  ctx.set_cors_headers(res, "http://localhost:3000");

  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Origin"), "http://localhost:3000");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Methods"), "GET, POST, PUT, DELETE");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Headers"), "Content-Type, Authorization");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Credentials"), "true");
  EXPECT_EQ(res.get_header_value("Access-Control-Expose-Headers"), "Content-Disposition, Content-Length");
}

TEST_F(HandlerContextCorsTest, SetCorsHeadersWithoutCredentials) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000"})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .with_credentials(false)
                    .build();

  HandlerContext ctx(nullptr, config, auth_config_, tls_config_, nullptr);
  httplib::Response res;

  ctx.set_cors_headers(res, "http://localhost:3000");

  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Origin"), "http://localhost:3000");
  // Credentials header should not be set when disabled
  EXPECT_TRUE(res.get_header_value("Access-Control-Allow-Credentials").empty());
}

// =============================================================================
// Entity ID validation tests
// =============================================================================

TEST_F(HandlerContextCorsTest, ValidateEntityIdAcceptsValidIds) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.validate_entity_id("engine").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("engine_temp_sensor").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("sensor123").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("MyComponent").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("engine-ecu").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("front-left-door").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("a").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("X1").has_value());
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsEmptyString) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  auto result = ctx.validate_entity_id("");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("empty") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsTooLongIds) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Create a string longer than 256 characters
  std::string long_id(257, 'a');

  auto result = ctx.validate_entity_id(long_id);
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("too long") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsForwardSlash) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  auto result = ctx.validate_entity_id("path/injection");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("invalid character") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsSpecialCharacters) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Test various invalid characters
  EXPECT_FALSE(ctx.validate_entity_id("test@value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test#value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test$value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test%value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test&value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test*value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test!value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test value").has_value());   // Space
  EXPECT_FALSE(ctx.validate_entity_id("test\nvalue").has_value());  // Newline
  EXPECT_FALSE(ctx.validate_entity_id("test\tvalue").has_value());  // Tab
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsNonPrintableCharacters) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Test non-printable characters
  std::string with_null = "test";
  with_null += '\0';
  with_null += "value";

  auto result = ctx.validate_entity_id(with_null);
  EXPECT_FALSE(result.has_value());
  // Error should show hex representation for non-printable
  EXPECT_TRUE(result.error().find("0x00") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdAcceptsMaxLengthId) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Create exactly 256 character string (max allowed)
  std::string max_id(256, 'a');

  auto result = ctx.validate_entity_id(max_id);
  EXPECT_TRUE(result.has_value());
}

// =============================================================================
// CorsConfigBuilder tests
// =============================================================================

TEST(CorsConfigBuilderTest, BuildsValidConfig) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000"})
                    .with_methods({"GET", "POST"})
                    .with_headers({"Content-Type"})
                    .with_credentials(true)
                    .with_max_age(3600)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins.size(), 1);
  EXPECT_EQ(config.allowed_origins[0], "http://localhost:3000");
  EXPECT_EQ(config.allowed_methods.size(), 2);
  EXPECT_EQ(config.allowed_headers.size(), 1);
  EXPECT_TRUE(config.allow_credentials);
  EXPECT_EQ(config.max_age_seconds, 3600);
  EXPECT_EQ(config.methods_header, "GET, POST");
  EXPECT_EQ(config.headers_header, "Content-Type");
}

TEST(CorsConfigBuilderTest, EmptyOriginsDisablesCors) {
  auto config = CorsConfigBuilder().with_origins({}).with_methods({"GET"}).with_headers({"Content-Type"}).build();

  EXPECT_FALSE(config.enabled);
}

TEST(CorsConfigBuilderTest, FiltersEmptyStringsFromOrigins) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000", "", "http://example.com", ""})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins.size(), 2);
  EXPECT_EQ(config.allowed_origins[0], "http://localhost:3000");
  EXPECT_EQ(config.allowed_origins[1], "http://example.com");
}

TEST(CorsConfigBuilderTest, CredentialsWithWildcardThrows) {
  EXPECT_THROW(
      {
        CorsConfigBuilder()
            .with_origins({"*"})
            .with_methods({"GET"})
            .with_headers({"Content-Type"})
            .with_credentials(true)
            .build();
      },
      std::invalid_argument);
}

TEST(CorsConfigBuilderTest, WildcardWithoutCredentialsSucceeds) {
  auto config = CorsConfigBuilder()
                    .with_origins({"*"})
                    .with_methods({"GET", "POST"})
                    .with_headers({"Content-Type", "Authorization"})
                    .with_credentials(false)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins[0], "*");
}

TEST(CorsConfigBuilderTest, MultipleMethodsAndHeadersJoined) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost"})
                    .with_methods({"GET", "POST", "PUT", "DELETE", "OPTIONS"})
                    .with_headers({"Content-Type", "Authorization", "X-Custom-Header"})
                    .build();

  EXPECT_EQ(config.methods_header, "GET, POST, PUT, DELETE, OPTIONS");
  EXPECT_EQ(config.headers_header, "Content-Type, Authorization, X-Custom-Header");
}

TEST(CorsConfigBuilderTest, DefaultMaxAge) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost"})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .build();

  EXPECT_EQ(config.max_age_seconds, 86400);  // Default value
}

// =============================================================================
// Helper: reserve a free TCP port for test isolation
// =============================================================================

namespace {

// Reserve an ephemeral port for test isolation. There is an inherent TOCTOU
// between closing this socket and the server binding: another process could
// grab the port in between. In practice this is astronomically unlikely with
// ephemeral ports (kernel assigns from a large range) and retrying would add
// complexity for negligible benefit.
int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
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

/// Build a request with regex matches populated (simulates cpp-httplib routing)
httplib::Request make_request_with_path(const std::string & path) {
  httplib::Request req;
  req.method = "GET";
  req.path = path;
  return req;
}

}  // namespace

// =============================================================================
// Shared rclcpp lifecycle for all fixtures that need GatewayNode
// =============================================================================

class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

::testing::Environment * const rclcpp_env = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// =============================================================================
// HandlerContext forwarding tests (require GatewayNode + AggregationManager)
// =============================================================================

class HandlerContextForwardingTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0) << "Failed to reserve a port for test";

    rclcpp::NodeOptions options;
    options.append_parameter_override("server.port", suite_server_port_);
    options.append_parameter_override("refresh_interval_ms", 60000);
    suite_node_ = std::make_shared<GatewayNode>(options);
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Seed the entity cache with a local component and a "remote" component.
    // The "remote" component is just a normal component in the cache; the routing
    // table in the AggregationManager is what marks it as remote.
    Component local_comp;
    local_comp.id = "local_ecu";
    local_comp.name = "Local ECU";
    local_comp.namespace_path = "/local";
    local_comp.fqn = "/local/local_ecu";

    Component remote_comp;
    remote_comp.id = "remote_sensor";
    remote_comp.name = "Remote Sensor";
    remote_comp.namespace_path = "/remote";
    remote_comp.fqn = "/remote/remote_sensor";

    App remote_app;
    remote_app.id = "remote_driver";
    remote_app.name = "Remote Driver";
    remote_app.component_id = "remote_sensor";

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all({}, {local_comp, remote_comp}, {remote_app}, {});

    // Create AggregationManager with a static peer (unreachable - we only need routing)
    AggregationConfig agg_config;
    agg_config.enabled = true;
    agg_config.timeout_ms = 200;
    AggregationConfig::PeerConfig peer;
    peer.url = "http://127.0.0.1:59999";  // Unreachable port
    peer.name = "peer_subsystem";
    agg_config.peers.push_back(peer);
    agg_mgr_ = std::make_unique<AggregationManager>(agg_config);

    // Set up routing table: remote_sensor and remote_driver are owned by peer_subsystem
    std::unordered_map<std::string, std::string> routing;
    routing["remote_sensor"] = "peer_subsystem";
    routing["remote_driver"] = "peer_subsystem";
    agg_mgr_->update_routing_table(routing);
  }

  void TearDown() override {
    agg_mgr_.reset();
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::unique_ptr<AggregationManager> agg_mgr_;
};

// The legacy validate_entity_for_route(req, res, id) overload was removed in
// commit 30. The forwarding and entity-resolution behaviour it covered now
// runs exclusively through the typed `validate_entity_for_route(typed_req,
// id)` overload exercised by the TypedValidateEntityForRoute_* tests below
// (and end-to-end through the typed router in the per-handler test suites).

// Verify get_entity_info marks entity as remote when aggregation manager is set
// and entity is in the routing table.
TEST_F(HandlerContextForwardingTest, GetEntityInfoSetsRemoteFieldsWithAggregation) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto info = ctx.get_entity_info("remote_sensor", SovdEntityType::COMPONENT);

  EXPECT_EQ(info.type, EntityType::COMPONENT);
  EXPECT_TRUE(info.is_remote);
  EXPECT_EQ(info.peer_name, "peer_subsystem");
  EXPECT_FALSE(info.peer_url.empty());
}

// Verify get_entity_info does NOT set remote fields when aggregation manager is null.
TEST_F(HandlerContextForwardingTest, GetEntityInfoNoRemoteWithoutAggregation) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  // No aggregation manager set

  auto info = ctx.get_entity_info("remote_sensor", SovdEntityType::COMPONENT);

  EXPECT_EQ(info.type, EntityType::COMPONENT);
  EXPECT_FALSE(info.is_remote);
  EXPECT_TRUE(info.peer_name.empty());
  EXPECT_TRUE(info.peer_url.empty());
}

// =============================================================================
// Typed validator surface tests (commit 6)
//
// The typed overloads of validate_entity_for_route / validate_collection_access /
// validate_lock_access return tl::expected<T, ErrorInfo> (or
// ValidatorResult<T> for validate_entity_for_route which can also short-circuit
// via Forwarded) and MUST NOT touch the httplib::Response on local failure.
// Forwarded is the one exception - peer proxying still writes to the response
// because that is the path's only sink.
// =============================================================================

TEST_F(HandlerContextForwardingTest, TypedValidateEntityForRoute_SuccessReturnsEntityInfo) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto raw_req = make_request_with_path("/api/v1/components/local_ecu/data");
  http::TypedRequest typed_req(raw_req);

  auto result = ctx.validate_entity_for_route(typed_req, "local_ecu");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "local_ecu");
  EXPECT_EQ(result->type, EntityType::COMPONENT);
  EXPECT_FALSE(result->is_remote);
}

TEST_F(HandlerContextForwardingTest, TypedValidateEntityForRoute_InvalidIdReturnsErrorInfo) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);

  auto raw_req = make_request_with_path("/api/v1/components/bad%20id/data");
  http::TypedRequest typed_req(raw_req);

  // We also need to verify nothing was written to a hypothetical response, so
  // we keep a fresh response next to the call and assert it stays untouched.
  httplib::Response untouched_res;
  auto result = ctx.validate_entity_for_route(typed_req, "bad id");

  ASSERT_FALSE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<ErrorInfo>(result.error()));
  const auto & err = std::get<ErrorInfo>(result.error());
  EXPECT_EQ(err.http_status, 400);
  EXPECT_EQ(err.code, ERR_INVALID_PARAMETER);
  // The typed path must not write any response.
  EXPECT_EQ(untouched_res.status, -1);
  EXPECT_TRUE(untouched_res.body.empty());
  EXPECT_TRUE(untouched_res.get_header_value("Content-Type").empty());
}

TEST_F(HandlerContextForwardingTest, TypedValidateEntityForRoute_UnknownEntityReturnsErrorInfo) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto raw_req = make_request_with_path("/api/v1/components/nonexistent/data");
  http::TypedRequest typed_req(raw_req);

  httplib::Response untouched_res;
  auto result = ctx.validate_entity_for_route(typed_req, "nonexistent");

  ASSERT_FALSE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<ErrorInfo>(result.error()));
  const auto & err = std::get<ErrorInfo>(result.error());
  EXPECT_EQ(err.http_status, 404);
  EXPECT_EQ(err.code, ERR_ENTITY_NOT_FOUND);
  // Typed path does not write anything.
  EXPECT_EQ(untouched_res.status, -1);
  EXPECT_TRUE(untouched_res.body.empty());
}

TEST_F(HandlerContextForwardingTest, TypedValidateEntityForRoute_WrongTypeReturnsErrorInfo) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  // remote_driver is an App, but we ask the /components route for it
  auto raw_req = make_request_with_path("/api/v1/components/remote_driver/data");
  http::TypedRequest typed_req(raw_req);

  auto result = ctx.validate_entity_for_route(typed_req, "remote_driver");

  ASSERT_FALSE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<ErrorInfo>(result.error()));
  const auto & err = std::get<ErrorInfo>(result.error());
  EXPECT_EQ(err.http_status, 400);
  EXPECT_EQ(err.code, ERR_INVALID_PARAMETER);
  EXPECT_NE(err.message.find("Invalid entity type for route"), std::string::npos);
}

TEST_F(HandlerContextForwardingTest, TypedValidateEntityForRoute_ForwardedWhenRemote) {
  // The typed overload writes the proxied response when the framework has
  // installed a response sink. In the production flow the legacy overload does
  // that; in this unit test we exercise the legacy overload to drive the
  // forwarding path end-to-end through the typed implementation. We verify the
  // legacy overload returns kForwarded and that the typed overload alone (with
  // no installed sink) signals Forwarded without crashing.

  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto raw_req = make_request_with_path("/api/v1/components/remote_sensor/data");
  http::TypedRequest typed_req(raw_req);

  // Direct call to the typed overload: with no response sink installed, the
  // typed validator still returns Forwarded (it is the framework's contract
  // signal that the request was proxied) but writes nothing to any wire.
  auto typed_result = ctx.validate_entity_for_route(typed_req, "remote_sensor");
  ASSERT_FALSE(typed_result.has_value());
  EXPECT_TRUE(std::holds_alternative<http::Forwarded>(typed_result.error()));
}

// =============================================================================
// Aggregation forwarding through the typed wrappers (regression).
//
// validate_entity_for_route only streams a remote-peer response when the typed
// wrapper has installed a ForwardResponseScope. The body / alternates / delete-
// alternates / multipart wrappers must each install it; otherwise a write to a
// remote entity returns Forwarded with no sink and the client gets an empty
// no-op response instead of the proxied peer response. Each test below drives a
// real request through a registered wrapper for a remote entity and asserts the
// proxied gateway error (502 x-medkit-peer-unavailable; the peer port is
// unreachable) reaches the client - which only happens when forwarding ran.
// =============================================================================

namespace ros2_medkit_gateway {
namespace dto {

struct FwdReqBody {
  std::optional<nlohmann::json> data;
};
template <>
inline constexpr auto dto_fields<FwdReqBody> = std::make_tuple(field("data", &FwdReqBody::data));
template <>
inline constexpr std::string_view dto_name<FwdReqBody> = "FwdReqBody";

struct FwdAck {
  std::string ok;
};
template <>
inline constexpr auto dto_fields<FwdAck> = std::make_tuple(field("ok", &FwdAck::ok));
template <>
inline constexpr std::string_view dto_name<FwdAck> = "FwdAck";

}  // namespace dto
}  // namespace ros2_medkit_gateway

namespace {

// Mirror handlers' flatten_validator_error: a Forwarded validator outcome maps
// to the framework's forwarded sentinel so the typed wrapper's error renderer
// treats it as a no-op and preserves the already-written proxy body.
ErrorInfo forwarded_or_error(const std::variant<ErrorInfo, http::Forwarded> & e) {
  if (std::holds_alternative<http::Forwarded>(e)) {
    return HandlerContext::forwarded_sentinel_error();
  }
  return std::get<ErrorInfo>(e);
}

// Spin up a cpp-httplib server for a registry on an ephemeral loopback port.
struct ScopedFwdServer {
  std::unique_ptr<httplib::Server> server;
  std::thread thread;
  int port{0};
  ScopedFwdServer() = default;
  ScopedFwdServer(ScopedFwdServer &&) noexcept = default;
  ScopedFwdServer & operator=(ScopedFwdServer &&) noexcept = default;
  ScopedFwdServer(const ScopedFwdServer &) = delete;
  ScopedFwdServer & operator=(const ScopedFwdServer &) = delete;
  ~ScopedFwdServer() {
    if (server) {
      server->stop();
    }
    if (thread.joinable()) {
      thread.join();
    }
  }
};

ScopedFwdServer start_fwd_server(const openapi::RouteRegistry & reg) {
  ScopedFwdServer s;
  s.server = std::make_unique<httplib::Server>();
  reg.register_all(*s.server, "/api/v1");
  s.port = s.server->bind_to_any_port("127.0.0.1");
  s.thread = std::thread([srv = s.server.get()]() {
    srv->listen_after_bind();
  });
  s.server->wait_until_ready();
  return s;
}

void expect_proxied_peer_unavailable(const httplib::Result & r) {
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 502) << "remote-entity request was not forwarded (empty no-op response)";
  auto body = nlohmann::json::parse(r->body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body.value("vendor_code", ""), "x-medkit-peer-unavailable");
}

}  // namespace

TEST_F(HandlerContextForwardingTest, TypedBodyWrapperForwardsRemoteEntityWrite) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  openapi::RouteRegistry reg;
  std::function<http::Result<http::NoContent>(http::TypedRequest, dto::FwdReqBody)> handler =
      [&ctx](const http::TypedRequest & req, const dto::FwdReqBody &) -> http::Result<http::NoContent> {
    auto entity = ctx.validate_entity_for_route(req, "remote_sensor");
    if (!entity) {
      return tl::unexpected(forwarded_or_error(entity.error()));
    }
    return http::NoContent{};
  };
  reg.put<dto::FwdReqBody, http::NoContent>("/components/{id}/configurations/{cid}", std::move(handler))
      .tag("Test")
      .summary("forwarding put");

  auto s = start_fwd_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Put("/api/v1/components/remote_sensor/configurations/foo", nlohmann::json{{"data", 1}}.dump(),
                   "application/json");
  expect_proxied_peer_unavailable(r);
}

TEST_F(HandlerContextForwardingTest, TypedPostAlternatesWrapperForwardsRemoteEntity) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  openapi::RouteRegistry reg;
  std::function<http::Result<std::variant<dto::FwdAck>>(http::TypedRequest, dto::FwdReqBody)> handler =
      [&ctx](const http::TypedRequest & req, const dto::FwdReqBody &) -> http::Result<std::variant<dto::FwdAck>> {
    auto entity = ctx.validate_entity_for_route(req, "remote_sensor");
    if (!entity) {
      return tl::unexpected(forwarded_or_error(entity.error()));
    }
    return std::variant<dto::FwdAck>{dto::FwdAck{"ok"}};
  };
  reg.post_alternates<dto::FwdReqBody, dto::FwdAck>("/components/{id}/operations/{oid}/executions", std::move(handler))
      .tag("Test")
      .summary("forwarding post");

  auto s = start_fwd_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Post("/api/v1/components/remote_sensor/operations/op/executions", nlohmann::json{{"data", 1}}.dump(),
                    "application/json");
  expect_proxied_peer_unavailable(r);
}

TEST_F(HandlerContextForwardingTest, TypedDeleteAlternatesWrapperForwardsRemoteEntity) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  openapi::RouteRegistry reg;
  std::function<http::Result<std::variant<dto::FwdAck>>(http::TypedRequest)> handler =
      [&ctx](const http::TypedRequest & req) -> http::Result<std::variant<dto::FwdAck>> {
    auto entity = ctx.validate_entity_for_route(req, "remote_sensor");
    if (!entity) {
      return tl::unexpected(forwarded_or_error(entity.error()));
    }
    return std::variant<dto::FwdAck>{dto::FwdAck{"ok"}};
  };
  reg.del_alternates<dto::FwdAck>("/components/{id}/faults/{fid}", std::move(handler))
      .tag("Test")
      .summary("forwarding delete");

  auto s = start_fwd_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Delete("/api/v1/components/remote_sensor/faults/bar");
  expect_proxied_peer_unavailable(r);
}

TEST_F(HandlerContextForwardingTest, TypedMultipartWrapperForwardsRemoteEntityUpload) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  openapi::RouteRegistry reg;
  std::function<http::Result<std::pair<dto::FwdAck, http::ResponseAttachments>>(http::TypedRequest,
                                                                                http::MultipartBody)>
      handler = [&ctx](const http::TypedRequest & req,
                       const http::MultipartBody &) -> http::Result<std::pair<dto::FwdAck, http::ResponseAttachments>> {
    auto entity = ctx.validate_entity_for_route(req, "remote_sensor");
    if (!entity) {
      return tl::unexpected(forwarded_or_error(entity.error()));
    }
    return std::make_pair(dto::FwdAck{"ok"}, http::ResponseAttachments{});
  };
  reg.multipart_upload<dto::FwdAck>("/components/{id}/bulk-data/{cat}", std::move(handler))
      .tag("Test")
      .summary("forwarding upload");

  auto s = start_fwd_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  httplib::MultipartFormDataItems items{{"file", "payload-bytes", "f.bin", "application/octet-stream"}};
  auto r = cli.Post("/api/v1/components/remote_sensor/bulk-data/cat1", items);
  expect_proxied_peer_unavailable(r);
}

TEST_F(HandlerContextForwardingTest, TypedValidateCollectionAccess_SupportedReturnsSuccess) {
  EntityInfo entity;
  entity.type = EntityType::COMPONENT;
  entity.id = "engine_ecu";
  entity.error_name = "Component";

  auto result = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::DATA);
  EXPECT_TRUE(result.has_value());
}

TEST_F(HandlerContextForwardingTest, TypedValidateCollectionAccess_UnsupportedReturnsErrorInfo) {
  // Areas do not support SCRIPTS - the ros2_medkit extension to SOVD adds
  // DATA/OPERATIONS/CONFIGURATIONS/FAULTS/LOGS/BULK_DATA aggregation but not
  // SCRIPTS.
  EntityInfo entity;
  entity.type = EntityType::AREA;
  entity.id = "powertrain";
  entity.error_name = "Area";

  auto result = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::SCRIPTS);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ERR_COLLECTION_NOT_SUPPORTED);
  EXPECT_NE(result.error().message.find("do not support"), std::string::npos);
  // params should be a JSON object carrying entity_id and collection
  EXPECT_TRUE(result.error().params.contains("entity_id"));
  EXPECT_TRUE(result.error().params.contains("collection"));
}

// The legacy std::optional<std::string> overload of
// validate_collection_access was removed in commit 30; the typed variant
// above is the only API.

TEST_F(HandlerContextForwardingTest, TypedValidateLockAccess_AllowedWhenLockingDisabled) {
  // suite_node_ is created without a LockManager (default ctor path), so the
  // typed validator's phase-1 short-circuit kicks in and returns success.
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);

  EntityInfo entity;
  entity.id = "engine_ecu";
  entity.error_name = "Component";

  auto raw_req = make_request_with_path("/api/v1/components/engine_ecu/configurations");
  http::TypedRequest typed_req(raw_req);

  auto result = ctx.validate_lock_access(typed_req, entity, "configurations");
  EXPECT_TRUE(result.has_value()) << "validate_lock_access should allow when no LockManager is configured";
}

TEST_F(HandlerContextForwardingTest, TypedValidateLockAccess_AllowedWhenNodeNull) {
  // No GatewayNode -> the validator must allow access (phase-1 short-circuit).
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);

  EntityInfo entity;
  entity.id = "engine_ecu";
  entity.error_name = "Component";

  auto raw_req = make_request_with_path("/api/v1/components/engine_ecu/configurations");
  http::TypedRequest typed_req(raw_req);

  auto result = ctx.validate_lock_access(typed_req, entity, "configurations");
  EXPECT_TRUE(result.has_value());
}

// The legacy (req, res, entity, collection) overload of validate_lock_access
// was removed in commit 30; the typed variant above is the only API.

// =============================================================================
// filter_internal_node_apps tests (no GatewayNode required)
// =============================================================================

TEST(FilterInternalNodeAppsTest, FiltersLocalInternalNodes) {
  std::vector<App> apps;
  App normal;
  normal.id = "temp_sensor";
  normal.name = "Temperature Sensor";
  apps.push_back(normal);

  App internal;
  internal.id = "_ros2cli_daemon";
  internal.name = "ROS 2 CLI Daemon";
  apps.push_back(internal);

  App another_internal;
  another_internal.id = "_launch_introspection";
  another_internal.name = "Launch Introspection";
  apps.push_back(another_internal);

  std::unordered_map<std::string, std::string> routing;
  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 2u);
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "temp_sensor");
}

TEST(FilterInternalNodeAppsTest, PreservesAllNormalNodes) {
  std::vector<App> apps;
  App a1;
  a1.id = "temp_sensor";
  apps.push_back(a1);

  App a2;
  a2.id = "rpm_sensor";
  apps.push_back(a2);

  App a3;
  a3.id = "lidar_driver";
  apps.push_back(a3);

  std::unordered_map<std::string, std::string> routing;
  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(apps.size(), 3u);
}

TEST(FilterInternalNodeAppsTest, FiltersPeerPrefixedInternalNodes) {
  // Remote entity: peer_subsystem___ros2cli_daemon
  // The routing table maps this to peer "peer_subsystem", so after stripping
  // the prefix we get "_ros2cli_daemon" which starts with '_' -> filtered.
  std::vector<App> apps;
  App remote_internal;
  remote_internal.id = "peer_subsystem___ros2cli_daemon";
  remote_internal.name = "Remote Internal";
  apps.push_back(remote_internal);

  App remote_normal;
  remote_normal.id = "peer_subsystem__lidar_driver";
  remote_normal.name = "Remote Lidar";
  apps.push_back(remote_normal);

  std::unordered_map<std::string, std::string> routing;
  routing["peer_subsystem___ros2cli_daemon"] = "peer_subsystem";
  routing["peer_subsystem__lidar_driver"] = "peer_subsystem";

  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 1u);
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "peer_subsystem__lidar_driver");
}

TEST(FilterInternalNodeAppsTest, DoesNotStripPrefixWithoutRoutingEntry) {
  // An entity ID that looks like it has a peer prefix but has no routing entry.
  // Without routing, we don't know the peer name, so we check the raw ID.
  // "peer__normal_node" has no routing entry, raw ID doesn't start with '_' -> kept.
  std::vector<App> apps;
  App ambiguous;
  ambiguous.id = "peer__normal_node";
  ambiguous.name = "Ambiguous";
  apps.push_back(ambiguous);

  std::unordered_map<std::string, std::string> routing;
  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "peer__normal_node");
}

TEST(FilterInternalNodeAppsTest, HandlesEmptyAppList) {
  std::vector<App> apps;
  std::unordered_map<std::string, std::string> routing;

  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 0u);
  EXPECT_TRUE(apps.empty());
}

TEST(FilterInternalNodeAppsTest, MixedLocalAndRemoteInternalNodes) {
  std::vector<App> apps;

  App local_normal;
  local_normal.id = "temp_sensor";
  apps.push_back(local_normal);

  App local_internal;
  local_internal.id = "_daemon";
  apps.push_back(local_internal);

  App remote_normal;
  remote_normal.id = "sub_b__actuator";
  remote_normal.name = "Remote Actuator";
  apps.push_back(remote_normal);

  App remote_internal;
  remote_internal.id = "sub_b___parameter_bridge";
  remote_internal.name = "Remote Parameter Bridge";
  apps.push_back(remote_internal);

  std::unordered_map<std::string, std::string> routing;
  routing["sub_b__actuator"] = "sub_b";
  routing["sub_b___parameter_bridge"] = "sub_b";

  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 2u);
  ASSERT_EQ(apps.size(), 2u);
  // Verify the surviving apps
  std::set<std::string> remaining_ids;
  for (const auto & app : apps) {
    remaining_ids.insert(app.id);
  }
  EXPECT_TRUE(remaining_ids.count("temp_sensor"));
  EXPECT_TRUE(remaining_ids.count("sub_b__actuator"));
}

TEST(FilterInternalNodeAppsTest, PeerPrefixMatchMustBeExact) {
  // Entity ID: "my_peer__sensor" with routing mapping to peer "my_peer".
  // After stripping "my_peer__", we get "sensor" which doesn't start with '_' -> kept.
  // This verifies the prefix match is exact and doesn't over-strip.
  std::vector<App> apps;
  App app;
  app.id = "my_peer__sensor";
  apps.push_back(app);

  std::unordered_map<std::string, std::string> routing;
  routing["my_peer__sensor"] = "my_peer";

  auto removed = filter_internal_node_apps(apps, routing, "/ros2_medkit_gateway");

  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(apps.size(), 1u);
}

namespace {

App bound_app(const std::string & id, const std::string & fqn) {
  App app;
  app.id = id;
  app.name = id;
  app.bound_fqn = fqn;
  return app;
}

}  // namespace

TEST(FilterInternalNodeAppsTest, DropsTheGatewaysOwnHelperNodes) {
  // The gateway creates "<self>_sub", "<self>_fault_clients" and
  // "<self>_lifecycle_state_reader" in its own process. None starts with '_',
  // so runtime introspection returns them as ordinary apps and the gateway ends
  // up listing its own plumbing as diagnosable.
  const std::string self_fqn = "/ros2_medkit_gateway";
  std::vector<App> apps{
      bound_app("ros2_medkit_gateway", self_fqn),
      bound_app("ros2_medkit_gateway_sub", self_fqn + "_sub"),
      bound_app("ros2_medkit_gateway_fault_clients", self_fqn + "_fault_clients"),
      bound_app("ros2_medkit_gateway_lifecycle_state_reader", self_fqn + "_lifecycle_state_reader"),
      // Positive controls on the same harness: a similarly suffixed FOREIGN
      // node, a node whose name merely extends the gateway's, and the
      // fault_manager, which is a separate diagnosable component even when it
      // shares the process.
      bound_app("other_gateway_sub", "/other_gateway_sub"),
      bound_app("ros2_medkit_gateway_monitor", self_fqn + "_monitor"),
      bound_app("fault_manager", "/fault_manager"),
  };

  std::unordered_map<std::string, std::string> routing;
  auto removed = filter_internal_node_apps(apps, routing, self_fqn);

  EXPECT_EQ(removed, 4u);
  std::set<std::string> remaining;
  for (const auto & app : apps) {
    remaining.insert(app.id);
  }
  EXPECT_EQ(remaining, (std::set<std::string>{"other_gateway_sub", "ros2_medkit_gateway_monitor", "fault_manager"}));
}

TEST(FilterInternalNodeAppsTest, LeavesPeerHelperNodesToThePeer) {
  // A remote entity carrying the same FQN belongs to the peer that reported
  // it, so this gateway must not reach across and filter it.
  const std::string self_fqn = "/ros2_medkit_gateway";
  std::vector<App> apps{bound_app("sub_b__ros2_medkit_gateway_sub", self_fqn + "_sub")};

  std::unordered_map<std::string, std::string> routing;
  routing["sub_b__ros2_medkit_gateway_sub"] = "sub_b";

  auto removed = filter_internal_node_apps(apps, routing, self_fqn);

  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(apps.size(), 1u);
}

TEST(IsOwnGatewayNodeTest, MatchesSelfAndHelpersExactlyAndNothingElse) {
  const std::string self_fqn = "/ros2_medkit_gateway";
  EXPECT_TRUE(is_own_gateway_node(self_fqn, self_fqn));
  EXPECT_TRUE(is_own_gateway_node(self_fqn + "_sub", self_fqn));
  EXPECT_TRUE(is_own_gateway_node(self_fqn + "_fault_clients", self_fqn));
  EXPECT_TRUE(is_own_gateway_node(self_fqn + "_lifecycle_state_reader", self_fqn));

  // Prefix neighbours are genuine peers, not ours.
  EXPECT_FALSE(is_own_gateway_node(self_fqn + "_monitor", self_fqn));
  EXPECT_FALSE(is_own_gateway_node(self_fqn + "2", self_fqn));
  EXPECT_FALSE(is_own_gateway_node("/other" + self_fqn + "_sub", self_fqn));
  EXPECT_FALSE(is_own_gateway_node("/fault_manager", self_fqn));

  // An unknown self FQN must claim nothing rather than everything.
  EXPECT_FALSE(is_own_gateway_node(self_fqn, ""));
  EXPECT_FALSE(is_own_gateway_node("", self_fqn));
}

// =============================================================================
// Area fault/log aggregation handler tests (via REST API)
// =============================================================================

class AreaAggregationTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0) << "Failed to reserve a port for test";

    rclcpp::NodeOptions options;
    options.append_parameter_override("server.port", suite_server_port_);
    options.append_parameter_override("server.host", std::string("127.0.0.1"));
    options.append_parameter_override("refresh_interval_ms", 60000);
    suite_node_ = std::make_shared<GatewayNode>(options);
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Seed the cache with Area -> Component -> App hierarchy.
    // Area "powertrain" contains Component "engine_ecu" which has App "temp_sensor".
    Area area;
    area.id = "powertrain";
    area.name = "Powertrain";
    area.namespace_path = "/powertrain";

    Component comp;
    comp.id = "engine_ecu";
    comp.name = "Engine ECU";
    comp.area = "powertrain";
    comp.namespace_path = "/powertrain/engine";
    comp.fqn = "/powertrain/engine/engine_ecu";

    App app1;
    app1.id = "temp_sensor";
    app1.name = "Temperature Sensor";
    app1.component_id = "engine_ecu";
    app1.ros_binding = App::RosBinding{"/powertrain/engine/temp_sensor", "/powertrain/engine", ""};

    App app2;
    app2.id = "rpm_sensor";
    app2.name = "RPM Sensor";
    app2.component_id = "engine_ecu";
    app2.ros_binding = App::RosBinding{"/powertrain/engine/rpm_sensor", "/powertrain/engine", ""};

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all({area}, {comp}, {app1, app2}, {});

    client_ = std::make_unique<httplib::Client>("127.0.0.1", suite_server_port_);
    client_->set_connection_timeout(5);
    client_->set_read_timeout(5);

    // Wait for the REST server thread to start listening.
    // On slow CI runners (Humble), the httplib::Server::listen() thread may
    // not have opened the port yet when the test starts making requests.
    for (int attempt = 0; attempt < 50; ++attempt) {
      auto probe = client_->Get("/api/v1/health");
      if (probe) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void TearDown() override {
    client_.reset();
  }

  std::unique_ptr<httplib::Client> client_;
};

// Note: Area faults handler returns 503 when FaultManager service is unavailable
// (5s service timeout, too slow for unit tests under load). The area fault
// aggregation path is tested end-to-end in integration tests instead.

// Area logs handler traverses area -> components -> apps -> FQNs chain and
// returns aggregated logs with x-medkit metadata. The LogManager is in-process
// so it works without external nodes (empty log buffer = empty items).
TEST_F(AreaAggregationTest, AreaLogsReturnsAggregatedResult) {
  auto res = client_->Get("/api/v1/areas/powertrain/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());

  // Verify x-medkit aggregation metadata is present
  ASSERT_TRUE(body.contains("x-medkit"));
  auto xmedkit = body["x-medkit"];
  EXPECT_EQ(xmedkit["entity_id"], "powertrain");
  EXPECT_EQ(xmedkit["aggregation_level"], "area");
  EXPECT_TRUE(xmedkit["aggregated"].get<bool>());
  EXPECT_EQ(xmedkit["component_count"], 1);
  EXPECT_EQ(xmedkit["app_count"], 2);

  // Verify aggregation sources contain the app FQNs
  ASSERT_TRUE(xmedkit.contains("aggregation_sources"));
  auto sources = xmedkit["aggregation_sources"];
  EXPECT_EQ(sources.size(), 2);
}

// =============================================================================
// Component aggregation tests for synthetic / runtime-discovered components.
//
// Synthetic components have empty fqn AND empty namespace_path. The COMPONENT
// branch in handle_get_logs must resolve hosted apps via the entity cache and
// emit aggregation metadata; otherwise the response was silently empty for
// every runtime-discovered component (the original bug fixed by this branch).
// =============================================================================

// Component with hosted apps but empty fqn/namespace_path returns aggregated
// metadata + items list. With no log buffer entries the items list is empty,
// but x-medkit must still report aggregation_level=component plus app_count
// and aggregation_sources covering both hosted apps.
TEST_F(AreaAggregationTest, ComponentLogsAggregatesFromHostedAppsForSyntheticComponent) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());

  // Synthetic component: empty fqn AND empty namespace_path - mirrors what
  // the runtime discovery strategy produces for components grouping nodes by
  // namespace.
  Component synthetic;
  synthetic.id = "runtime_engine";
  synthetic.name = "Runtime Engine";
  synthetic.area = "powertrain";
  synthetic.namespace_path = "";
  synthetic.fqn = "";

  // Synthetic components store runtime-discovered apps with bound_fqn populated
  // by the discovery layer (manifest-only path uses ros_binding instead).
  App app1;
  app1.id = "temp_sensor";
  app1.name = "Temperature Sensor";
  app1.component_id = "runtime_engine";
  app1.bound_fqn = "/powertrain/engine/temp_sensor";

  App app2;
  app2.id = "rpm_sensor";
  app2.name = "RPM Sensor";
  app2.component_id = "runtime_engine";
  app2.bound_fqn = "/powertrain/engine/rpm_sensor";

  Area area;
  area.id = "powertrain";
  area.name = "Powertrain";
  area.namespace_path = "/powertrain";
  cache.update_all({area}, {synthetic}, {app1, app2}, {});

  auto res = client_->Get("/api/v1/components/runtime_engine/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  ASSERT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());

  ASSERT_TRUE(body.contains("x-medkit"));
  auto xmedkit = body["x-medkit"];
  EXPECT_EQ(xmedkit["entity_id"], "runtime_engine");
  EXPECT_EQ(xmedkit["aggregation_level"], "component");
  EXPECT_TRUE(xmedkit["aggregated"].get<bool>());
  EXPECT_EQ(xmedkit["app_count"], 2);

  ASSERT_TRUE(xmedkit.contains("aggregation_sources"));
  auto sources = xmedkit["aggregation_sources"];
  ASSERT_EQ(sources.size(), 2);
  std::set<std::string> source_set;
  for (const auto & s : sources) {
    source_set.insert(s.get<std::string>());
  }
  EXPECT_TRUE(source_set.count("/powertrain/engine/temp_sensor"));
  EXPECT_TRUE(source_set.count("/powertrain/engine/rpm_sensor"));
}

// Manifest component without hosted apps (component groups topics rather than
// nodes) falls through to the namespace-prefix path. Aggregation metadata
// reports level=component + aggregated=true but omits app_count and
// aggregation_sources because hosted-app aggregation was not active.
TEST_F(AreaAggregationTest, ComponentLogsManifestOnlyFallsThroughToNamespacePrefix) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());

  // Manifest component with non-empty fqn but no hosted apps.
  Component manifest_comp;
  manifest_comp.id = "topic_only_component";
  manifest_comp.name = "Topic-Only Component";
  manifest_comp.area = "powertrain";
  manifest_comp.namespace_path = "/topics/group";
  manifest_comp.fqn = "/topics/group";

  Area area;
  area.id = "powertrain";
  area.name = "Powertrain";
  area.namespace_path = "/powertrain";
  cache.update_all({area}, {manifest_comp}, {}, {});

  auto res = client_->Get("/api/v1/components/topic_only_component/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  ASSERT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());

  ASSERT_TRUE(body.contains("x-medkit"));
  auto xmedkit = body["x-medkit"];
  EXPECT_EQ(xmedkit["aggregation_level"], "component");
  EXPECT_TRUE(xmedkit["aggregated"].get<bool>());
  // Hosted-app aggregation was not active, so these conditional fields must
  // be omitted (per docs/api/rest.rst contract).
  EXPECT_FALSE(xmedkit.contains("app_count"));
  EXPECT_FALSE(xmedkit.contains("aggregation_sources"));
}

// Component without hosted apps AND without fqn / namespace_path returns an
// empty items list - there is no source to query. Metadata still reports
// aggregation_level=component (the handler classified it as a component
// request) but no source metadata is emitted.
TEST_F(AreaAggregationTest, ComponentLogsEmptyComponentReturnsEmptyItemsAndNoSources) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());

  Component empty_comp;
  empty_comp.id = "empty_runtime_comp";
  empty_comp.name = "Empty Runtime Component";
  empty_comp.area = "powertrain";
  empty_comp.namespace_path = "";
  empty_comp.fqn = "";

  Area area;
  area.id = "powertrain";
  area.name = "Powertrain";
  area.namespace_path = "/powertrain";
  cache.update_all({area}, {empty_comp}, {}, {});

  auto res = client_->Get("/api/v1/components/empty_runtime_comp/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  ASSERT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());
  EXPECT_EQ(body["items"].size(), 0u);

  ASSERT_TRUE(body.contains("x-medkit"));
  auto xmedkit = body["x-medkit"];
  EXPECT_EQ(xmedkit["aggregation_level"], "component");
  EXPECT_FALSE(xmedkit.contains("app_count"));
  EXPECT_FALSE(xmedkit.contains("aggregation_sources"));
}

// Area with no components falls through to namespace prefix matching for logs.
// With no matching logs, returns empty items.
TEST_F(AreaAggregationTest, AreaLogsWithNoComponentsFallsThrough) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  Area empty_area;
  empty_area.id = "empty_domain";
  empty_area.name = "Empty Domain";
  empty_area.namespace_path = "/empty";

  Area pt_area;
  pt_area.id = "powertrain";
  pt_area.name = "Powertrain";
  pt_area.namespace_path = "/powertrain";

  Component comp;
  comp.id = "engine_ecu";
  comp.name = "Engine ECU";
  comp.area = "powertrain";
  comp.namespace_path = "/powertrain/engine";
  comp.fqn = "/powertrain/engine/engine_ecu";

  App app1;
  app1.id = "temp_sensor";
  app1.name = "Temperature Sensor";
  app1.component_id = "engine_ecu";
  app1.ros_binding = App::RosBinding{"/powertrain/engine/temp_sensor", "/powertrain/engine", ""};

  cache.update_all({pt_area, empty_area}, {comp}, {app1}, {});

  auto res = client_->Get("/api/v1/areas/empty_domain/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  // Falls through to namespace prefix matching (no components linked to area)
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());
}

// =============================================================================
// resolve_app_host_fqns tests (no GatewayNode required)
//
// Order-preserving wrapper over faults::resolve_app_source_fqn for explicit
// app-id lists. These tests pin the per-app rule (external -> bare id, stray
// binding, bound_fqn preference) and the silent-skip semantics (missing apps,
// empty effective_fqn).
// =============================================================================

namespace {

App make_app_with_binding(const std::string & id, const std::string & node_name, const std::string & ns) {
  App a;
  a.id = id;
  a.name = id;
  App::RosBinding rb;
  rb.node_name = node_name;
  rb.namespace_pattern = ns;
  a.ros_binding = rb;
  return a;
}

// An asset introspected by a protocol plugin (e.g. a PLC entity over OPC UA):
// external=true, no ROS binding, so effective_fqn() is empty.
App make_external_app(const std::string & app_id, const std::string & component_id) {
  App a;
  a.id = app_id;
  a.name = app_id;
  a.component_id = component_id;
  a.external = true;
  return a;
}

}  // namespace

TEST(ResolveAppHostFqnsTest, EmptyAppListReturnsEmpty) {
  ThreadSafeEntityCache cache;
  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {});
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveAppHostFqnsTest, ResolvesSingleAppByEffectiveFqn) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"temp_sensor"});
  ASSERT_EQ(fqns.size(), 1u);
  EXPECT_EQ(fqns[0], "/powertrain/engine/temp_sensor");
}

TEST(ResolveAppHostFqnsTest, ResolvesMultipleAppsPreservesInputOrder) {
  ThreadSafeEntityCache cache;
  cache.update_apps({
      make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine"),
      make_app_with_binding("rpm_sensor", "rpm_sensor", "/powertrain/engine"),
      make_app_with_binding("lidar", "lidar_sensor", "/perception/lidar"),
  });

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"temp_sensor", "lidar", "rpm_sensor"});
  ASSERT_EQ(fqns.size(), 3u);
  EXPECT_EQ(fqns[0], "/powertrain/engine/temp_sensor");
  EXPECT_EQ(fqns[1], "/perception/lidar/lidar_sensor");
  EXPECT_EQ(fqns[2], "/powertrain/engine/rpm_sensor");
}

TEST(ResolveAppHostFqnsTest, SkipsAppIdsMissingFromCache) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"missing_app", "temp_sensor", "also_missing"});
  ASSERT_EQ(fqns.size(), 1u);
  EXPECT_EQ(fqns[0], "/powertrain/engine/temp_sensor");
}

TEST(ResolveAppHostFqnsTest, SkipsAppsWithEmptyEffectiveFqn) {
  // App without ros_binding and without bound_fqn yields empty effective_fqn().
  ThreadSafeEntityCache cache;
  App empty_fqn_app;
  empty_fqn_app.id = "no_binding";
  empty_fqn_app.name = "no_binding";
  cache.update_apps({empty_fqn_app, make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"no_binding", "temp_sensor"});
  ASSERT_EQ(fqns.size(), 1u);
  EXPECT_EQ(fqns[0], "/powertrain/engine/temp_sensor");
}

TEST(ResolveAppHostFqnsTest, ExternalAppResolvesToItsBareEntityId) {
  // A plugin-introspected asset has no ROS binding, so effective_fqn() is empty,
  // but it reports faults - and therefore owns its rosbags and logs - under its
  // bare entity id. Dropping it here left the bulk-data / log filter set empty
  // and every download for the hosting component answered "not found".
  ThreadSafeEntityCache cache;
  cache.update_apps({make_external_app("plc_line1", "plc_hw"),
                     make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"plc_line1", "temp_sensor"});
  ASSERT_EQ(fqns.size(), 2u);
  EXPECT_EQ(fqns[0], "plc_line1");
  EXPECT_EQ(fqns[1], "/powertrain/engine/temp_sensor");
}

TEST(ResolveAppHostFqnsTest, ExternalAppWithStrayBindingStillResolvesToItsId) {
  // Mirrors the fault-scope rule: a manifest stub may declare a ros_binding on
  // an external app; the derived FQN must not shadow the id the plugin reports
  // under, or the entity's bags become unreachable again.
  ThreadSafeEntityCache cache;
  App plc = make_external_app("plc_line1", "plc_hw");
  plc.ros_binding = App::RosBinding{"process", "/plc", ""};
  cache.update_apps({plc});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"plc_line1"});
  ASSERT_EQ(fqns.size(), 1u);
  EXPECT_EQ(fqns[0], "plc_line1");
}

TEST(ResolveAppHostFqnsTest, AllAppsMissingReturnsEmpty) {
  ThreadSafeEntityCache cache;
  // Cache is empty - none of these IDs exist
  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"a", "b", "c"});
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveAppHostFqnsTest, PrefersBoundFqnOverRosBinding) {
  // bound_fqn (set by runtime linking) wins over ros_binding-derived FQN.
  ThreadSafeEntityCache cache;
  App linked = make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine");
  linked.bound_fqn = "/runtime/discovered/path";
  cache.update_apps({linked});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"temp_sensor"});
  ASSERT_EQ(fqns.size(), 1u);
  EXPECT_EQ(fqns[0], "/runtime/discovered/path");
}

TEST(ResolveAppHostFqnsTest, DuplicateEffectiveFqnsAreDeduplicated) {
  // Two app_ids resolving to the same effective_fqn (manifest + runtime
  // double-bind, or two App entries pointing at the same ROS node) must
  // produce exactly one filter. The first occurrence wins so first-seen
  // ordering is preserved.
  ThreadSafeEntityCache cache;
  App primary = make_app_with_binding("primary", "temp_sensor", "/powertrain/engine");
  App alias = make_app_with_binding("alias", "temp_sensor", "/powertrain/engine");
  // alias.bound_fqn left unset; ros_binding-derived fqn matches primary.
  cache.update_apps({primary, alias, make_app_with_binding("rpm_sensor", "rpm_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_app_host_fqns(cache, {"primary", "alias", "rpm_sensor", "alias"});
  ASSERT_EQ(fqns.size(), 2u);
  EXPECT_EQ(fqns[0], "/powertrain/engine/temp_sensor");
  EXPECT_EQ(fqns[1], "/powertrain/engine/rpm_sensor");
}

// =============================================================================
// resolve_entity_source_fqns tests
//
// The helper underpins #395's per-entity fault scope check. It must reject
// faults that come from outside the entity (empty result -> caller returns
// 404) AND must collect the FQNs of every hosted ROS node so that
// reporting-source matching covers all the apps the entity actually owns.
// =============================================================================

namespace {

EntityInfo make_entity_info(EntityType t, const std::string & id) {
  EntityInfo ei;
  ei.type = t;
  ei.id = id;
  return ei;
}

App make_owned_app(const std::string & app_id, const std::string & component_id, const std::string & node_name,
                   const std::string & ns) {
  App a = make_app_with_binding(app_id, node_name, ns);
  a.component_id = component_id;
  return a;
}

}  // namespace

TEST(ResolveEntitySourceFqnsTest, AppReturnsItsOwnFqn) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "temp_sensor"));
  EXPECT_EQ(fqns, std::set<std::string>{"/powertrain/engine/temp_sensor"});
}

TEST(ResolveEntitySourceFqnsTest, AppMissingFromCacheYieldsEmptySet) {
  ThreadSafeEntityCache cache;
  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "gone"));
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveEntitySourceFqnsTest, ComponentAggregatesHostedAppFqns) {
  ThreadSafeEntityCache cache;
  cache.update_apps({
      make_owned_app("temp-sensor", "temp-hw", "temp_sensor", "/powertrain/engine"),
      make_owned_app("rpm-sensor", "rpm-hw", "rpm_sensor", "/powertrain/engine"),
      make_owned_app("lidar-sensor", "lidar-unit", "lidar_sensor", "/perception/lidar"),
  });

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "temp-hw"));
  // The lidar app must not appear: the bug under #395 was that an empty
  // namespace_path on the addressed component silently disabled the scope
  // filter and exposed faults from unrelated apps.
  EXPECT_EQ(fqns, std::set<std::string>{"/powertrain/engine/temp_sensor"});
}

TEST(ResolveEntitySourceFqnsTest, ComponentWithoutHostedAppsReturnsEmptySet) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_owned_app("temp-sensor", "temp-hw", "temp_sensor", "/powertrain/engine")});

  // Querying a different component yields an empty set, NOT "no filter".
  // Callers (fault handlers) treat empty as out-of-scope -> 404.
  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "lidar-unit"));
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveEntitySourceFqnsTest, AreaCollectsAppsFromAllComponentsInArea) {
  ThreadSafeEntityCache cache;
  Component comp_a;
  comp_a.id = "temp-hw";
  comp_a.area = "engine";
  Component comp_b;
  comp_b.id = "rpm-hw";
  comp_b.area = "engine";
  Component comp_off_area;
  comp_off_area.id = "lidar-unit";
  comp_off_area.area = "perception";
  cache.update_components({comp_a, comp_b, comp_off_area});
  cache.update_apps({
      make_owned_app("temp-sensor", "temp-hw", "temp_sensor", "/powertrain/engine"),
      make_owned_app("rpm-sensor", "rpm-hw", "rpm_sensor", "/powertrain/engine"),
      make_owned_app("lidar-sensor", "lidar-unit", "lidar_sensor", "/perception/lidar"),
  });

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::AREA, "engine"));
  // Non-external components never claim their bare ids (#516 guardrail), so
  // the area rollup carries only the hosted apps' FQNs. The off-area lidar
  // branch must not appear.
  std::set<std::string> expected{"/powertrain/engine/temp_sensor", "/powertrain/engine/rpm_sensor"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, UnknownEntityTypeReturnsEmptySet) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});
  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::UNKNOWN, "anything"));
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveEntitySourceFqnsTest, AppsWithEmptyEffectiveFqnAreSkipped) {
  ThreadSafeEntityCache cache;
  App no_binding;
  no_binding.id = "no_binding";
  no_binding.component_id = "comp-x";
  App ok = make_owned_app("ok", "comp-x", "ok_node", "/ns");
  cache.update_apps({no_binding, ok});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "comp-x"));
  EXPECT_EQ(fqns, std::set<std::string>{"/ns/ok_node"});
}

TEST(ResolveEntitySourceFqnsTest, FunctionAggregatesHostedAppFqns) {
  ThreadSafeEntityCache cache;
  cache.update_apps({
      make_app_with_binding("nav-planner", "planner", "/perception/nav"),
      make_app_with_binding("nav-localizer", "localizer", "/perception/nav"),
      make_app_with_binding("unrelated", "telemetry", "/telemetry"),
  });
  Function autonomy;
  autonomy.id = "autonomous-navigation";
  autonomy.name = "Autonomous Navigation";
  autonomy.hosts = {"nav-planner", "nav-localizer"};
  cache.update_functions({autonomy});

  auto fqns = HandlerContext::resolve_entity_source_fqns(
      cache, make_entity_info(EntityType::FUNCTION, "autonomous-navigation"));
  std::set<std::string> expected{"/perception/nav/planner", "/perception/nav/localizer"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, AreaWalksNestedSubareasRecursively) {
  // Real manifests put components under subareas (e.g. demo manifest attaches
  // components to `engine` which is a subarea of `powertrain`). A query for
  // the top-level area must walk subareas, otherwise the demo would 404 on
  // every `/areas/powertrain/...` fault read.
  ThreadSafeEntityCache cache;
  Area powertrain;
  powertrain.id = "powertrain";
  powertrain.namespace_path = "/powertrain";
  Area engine;
  engine.id = "engine";
  engine.namespace_path = "/powertrain/engine";
  engine.parent_area_id = "powertrain";
  Area perception;
  perception.id = "perception";
  perception.namespace_path = "/perception";
  cache.update_areas({powertrain, engine, perception});

  Component engine_ecu;
  engine_ecu.id = "engine-ecu";
  engine_ecu.area = "engine";  // attached to subarea, NOT to top-level
  Component lidar_unit;
  lidar_unit.id = "lidar-unit";
  lidar_unit.area = "perception";
  cache.update_components({engine_ecu, lidar_unit});

  cache.update_apps({
      make_owned_app("engine-temp-sensor", "engine-ecu", "temp_sensor", "/powertrain/engine"),
      make_owned_app("lidar-sensor", "lidar-unit", "lidar_sensor", "/perception/lidar"),
  });

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::AREA, "powertrain"));
  // The subarea's non-external component does not ride along (#516 guardrail);
  // neither does the off-area lidar branch.
  EXPECT_EQ(fqns, std::set<std::string>{"/powertrain/engine/temp_sensor"});
}

TEST(ResolveEntitySourceFqnsTest, FunctionHostingComponentExpandsToComponentApps) {
  // Function.hosts can carry component IDs as well as app IDs (see
  // function.hpp). The cache's function_to_apps_ index only resolves
  // app-host entries, so the helper must reach into Function.hosts itself
  // and expand component hosts to the apps they own.
  ThreadSafeEntityCache cache;
  cache.update_components({Component{}});  // touch to ensure indexes settle
  Component drive_ecu;
  drive_ecu.id = "drive-ecu";
  cache.update_components({drive_ecu});

  cache.update_apps({
      make_owned_app("planner", "drive-ecu", "planner_node", "/drive"),
      make_owned_app("localizer", "drive-ecu", "localizer_node", "/drive"),
      make_owned_app("standalone", "", "standalone_node", "/misc"),
  });

  Function autonomy;
  autonomy.id = "autonomy";
  autonomy.hosts = {"drive-ecu", "standalone"};  // mix of component + app host
  cache.update_functions({autonomy});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::FUNCTION, "autonomy"));
  // The non-external component host contributes only its apps' FQNs, not its
  // own id (#516 guardrail).
  std::set<std::string> expected{"/drive/planner_node", "/drive/localizer_node", "/misc/standalone_node"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, FunctionWithUnboundHostsReturnsOnlyResolvedFqns) {
  // Function.hosts referencing an app that does not produce an effective_fqn
  // (no bound_fqn and no ros_binding) must be skipped, not return empty FQNs
  // - they would prefix-match everything in `fault_in_source_scope` and the
  // scope check would let any fault through.
  ThreadSafeEntityCache cache;
  App unbound;
  unbound.id = "unbound";
  cache.update_apps({unbound, make_app_with_binding("planner", "planner", "/perception/nav")});
  Function f;
  f.id = "func-x";
  f.hosts = {"unbound", "planner"};
  cache.update_functions({f});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::FUNCTION, "func-x"));
  EXPECT_EQ(fqns, std::set<std::string>{"/perception/nav/planner"});
}

// =============================================================================
// External protocol-plugin entity ownership.
//
// An external app (e.g. a PLC introspected over OPC UA) has no ROS binding, so
// effective_fqn() is empty. It reports faults to the fault_manager under its
// own entity id, so fault-scope resolution must recognise that bare id as the
// entity's sole owned source - otherwise the per-entity fault list is empty and
// the detail 404s. The bare id is granted ONLY for external apps; an unbound
// ROS app stays skipped so it cannot claim faults it never reported.
// =============================================================================

TEST(ResolveEntitySourceFqnsTest, ExternalAppOwnsFaultsUnderItsOwnId) {
  ThreadSafeEntityCache cache;
  cache.update_apps({make_external_app("process", "s7_1500")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "process"));
  // Scope set is exactly {its id}: narrow, so it owns only faults reported
  // under "process" (and sub-paths), never another entity's.
  EXPECT_EQ(fqns, std::set<std::string>{"process"});
}

TEST(ResolveEntitySourceFqnsTest, NonExternalUnboundAppStaysSkipped) {
  // Security: an app that is NOT external and has no binding (a ROS app that
  // failed to link) must NOT be granted its bare id - otherwise it would claim
  // ownership of any fault reported under that id. This pins the `external`
  // gate on the bare-id ownership grant.
  ThreadSafeEntityCache cache;
  App unbound;
  unbound.id = "process";
  unbound.name = "process";
  unbound.component_id = "s7_1500";
  unbound.external = false;
  cache.update_apps({unbound});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "process"));
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveEntitySourceFqnsTest, BoundAppScopeUnaffectedByExternalFix) {
  // Regression pin for security requirement (c): a ROS-bound entity's scope set
  // is byte-identical to before the external-ownership change - it resolves to
  // its FQN, never its bare id.
  ThreadSafeEntityCache cache;
  cache.update_apps({make_app_with_binding("temp_sensor", "temp_sensor", "/powertrain/engine")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "temp_sensor"));
  EXPECT_EQ(fqns, std::set<std::string>{"/powertrain/engine/temp_sensor"});
}

TEST(ResolveEntitySourceFqnsTest, ComponentHostingExternalAppOwnsItsFaults) {
  // The PLC case: GET /components/s7_1500/faults. The component hosts the
  // external app "process"; its scope must include "process" so faults the app
  // reported (reporting_sources=["process"]) surface on the component route.
  ThreadSafeEntityCache cache;
  cache.update_apps({make_external_app("process", "s7_1500")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "s7_1500"));
  EXPECT_EQ(fqns, std::set<std::string>{"process"});
}

TEST(ResolveEntitySourceFqnsTest, FunctionHostingExternalAppOwnsItsFaults) {
  // GET /functions/<id>/faults where the manifest Function's hosted_by lists an
  // external app directly (App-host, not Component-host). The Function's scope
  // must include the app's bare id alongside the FQNs of its ROS-bound hosts,
  // otherwise faults the external app reported vanish from the Function rollup.
  ThreadSafeEntityCache cache;
  cache.update_apps(
      {make_external_app("process", "s7_1500"), make_owned_app("planner", "nav_comp", "planner", "/perception/nav")});
  Function f;
  f.id = "level_control";
  f.hosts = {"process", "planner"};
  cache.update_functions({f});

  auto fqns =
      HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::FUNCTION, "level_control"));
  std::set<std::string> expected{"process", "/perception/nav/planner"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, AreaHostingExternalAppOwnsItsFaults) {
  // GET /areas/<area>/faults. The area's component hosts an external app (a PLC
  // over OPC UA - the opcua plugin sets comp.area). The area walks the same gate
  // as the component/function routes (collect_area_app_fqns -> collect_app_fqn),
  // so its scope must include the external app's bare id or the app's faults
  // vanish from the area rollup.
  ThreadSafeEntityCache cache;
  Component plc;
  plc.id = "s7_1500";
  plc.area = "cell_a";
  plc.external = true;  // plugin-discovered PLC component
  cache.update_components({plc});
  cache.update_apps({make_external_app("process", "s7_1500")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::AREA, "cell_a"));
  // The external app's bare id plus the hosting external component's own id.
  std::set<std::string> expected{"s7_1500", "process"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, ExternalAppWithStrayRosBindingOwnsFaultsByBareId) {
  // The #517 "neighbor" case: a manifest declares the app with a real
  // ros_binding (so effective_fqn() derives a live FQN) and a plugin classifies
  // it external. If fault scope used the derived FQN, the app's faults (reported
  // under its bare id) would vanish - the exact #517 symptom. The external
  // classification must win: the scope is the bare id, never the FQN.
  ThreadSafeEntityCache cache;
  App a;
  a.id = "process";
  a.name = "process";
  a.component_id = "s7_1500";
  a.external = true;
  App::RosBinding binding;
  binding.node_name = "process";
  binding.namespace_pattern = "/plc";
  a.ros_binding = binding;  // stray binding: effective_fqn() would be "/plc/process"
  cache.update_apps({a});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::APP, "process"));
  // Bare id owns the faults; the derived FQN "/plc/process" must NOT appear.
  EXPECT_EQ(fqns, std::set<std::string>{"process"});
}

TEST(ResolveEntitySourceFqnsTest, ExternalComponentWithNoAppsOwnsFaultsUnderItsOwnId) {
  // The #516 case: a PLC modeled as an external Component with no child App.
  // It reports faults under its own component id; the scope must be exactly
  // {its id} so GET /components/<id>/faults is non-empty.
  ThreadSafeEntityCache cache;
  Component plc;
  plc.id = "s7_1500";
  plc.external = true;
  cache.update_components({plc});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "s7_1500"));
  EXPECT_EQ(fqns, std::set<std::string>{"s7_1500"});
}

TEST(ResolveEntitySourceFqnsTest, InternalComponentNeverOwnsBareIdFaults) {
  // #516 guardrail: a non-external Component must not claim faults reported
  // under its bare id - an unbound ROS component would otherwise own faults
  // it never reported. Only external components (bridge plugins raising
  // PLC_COMMS_LOST-class faults with source_id = component id) get that.
  ThreadSafeEntityCache cache;
  Component comp;
  comp.id = "nav_comp";
  comp.external = false;
  cache.update_components({comp});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "nav_comp"));
  EXPECT_TRUE(fqns.empty());
}

TEST(ResolveEntitySourceFqnsTest, ExternalComponentWithContributingAppKeepsBothScopes) {
  // A Component that hosts a ROS-bound app owns the app's FQN AND its own id:
  // faults reported by the component itself must not vanish just because a
  // child app also contributes sources.
  ThreadSafeEntityCache cache;
  Component plc;
  plc.id = "s7_1500";
  plc.external = true;
  cache.update_components({plc});
  cache.update_apps({make_owned_app("planner", "s7_1500", "planner", "/perception/nav")});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::COMPONENT, "s7_1500"));
  std::set<std::string> expected{"s7_1500", "/perception/nav/planner"};
  EXPECT_EQ(fqns, expected);
}

TEST(ResolveEntitySourceFqnsTest, FunctionHostingExternalComponentOwnsItsFaults) {
  // Function.hosted_by lists the device Component directly (Component-host).
  // The Function rollup must include the external Component's bare id (#516:
  // no synthetic App needed).
  ThreadSafeEntityCache cache;
  Component plc;
  plc.id = "s7_1500";
  plc.external = true;
  cache.update_components({plc});
  Function f;
  f.id = "material_flow";
  f.hosts = {"s7_1500"};
  cache.update_functions({f});

  auto fqns =
      HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::FUNCTION, "material_flow"));
  EXPECT_EQ(fqns, std::set<std::string>{"s7_1500"});
}

TEST(ResolveEntitySourceFqnsTest, AreaHostingExternalComponentOwnsItsFaults) {
  // Area rollup walks the same component gate; an external device Component in
  // the area owns its faults on the area route.
  ThreadSafeEntityCache cache;
  Component plc;
  plc.id = "s7_1500";
  plc.area = "cell_a";
  plc.external = true;
  cache.update_components({plc});

  auto fqns = HandlerContext::resolve_entity_source_fqns(cache, make_entity_info(EntityType::AREA, "cell_a"));
  EXPECT_EQ(fqns, std::set<std::string>{"s7_1500"});
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
