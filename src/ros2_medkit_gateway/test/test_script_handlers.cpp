// Copyright 2026 Bartlomiej Burda
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

#include <cstring>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/script_handlers.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::EntityType;
using ros2_medkit_gateway::ExecutionInfo;
using ros2_medkit_gateway::ExecutionRequest;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ScriptBackendError;
using ros2_medkit_gateway::ScriptBackendErrorInfo;
using ros2_medkit_gateway::ScriptInfo;
using ros2_medkit_gateway::ScriptManager;
using ros2_medkit_gateway::ScriptProvider;
using ros2_medkit_gateway::ScriptUploadResult;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::ScriptHandlers;
namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

namespace {

// =============================================================================
// Mock script provider that returns configurable errors
// =============================================================================

class ErrorMockScriptProvider : public ScriptProvider {
 public:
  // The error to return from all operations (set per-test)
  ScriptBackendError error_code{ScriptBackendError::Internal};
  std::string error_message{"mock error"};

  // When true, operations succeed instead of returning errors
  bool succeed{false};

  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
  list_scripts(const std::string & /*entity_id*/) override {
    if (succeed) {
      return std::vector<ScriptInfo>{};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & /*entity_id*/,
                                                              const std::string & /*script_id*/) override {
    if (succeed) {
      return ScriptInfo{"s1", "Script", "desc", false, false, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & /*entity_id*/, const std::string & /*filename*/, const std::string & /*content*/,
                const std::optional<json> & /*metadata*/) override {
    if (succeed) {
      return ScriptUploadResult{"uploaded_001", "Uploaded Script"};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & /*entity_id*/,
                                                           const std::string & /*script_id*/) override {
    if (succeed) {
      return {};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> start_execution(const std::string & /*entity_id*/,
                                                                      const std::string & /*script_id*/,
                                                                      const ExecutionRequest & /*request*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001",   "running",    std::nullopt, "2026-03-18T10:00:00Z",
                           std::nullopt, std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> get_execution(const std::string & /*entity_id*/,
                                                                    const std::string & /*script_id*/,
                                                                    const std::string & /*execution_id*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001", "running", 50, "2026-03-18T10:00:00Z", std::nullopt, std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & /*entity_id*/,
                                                                        const std::string & /*script_id*/,
                                                                        const std::string & /*execution_id*/,
                                                                        const std::string & /*action*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001",   "terminated", std::nullopt, "2026-03-18T10:00:00Z", "2026-03-18T10:01:00Z",
                           std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_execution(const std::string & /*entity_id*/,
                                                              const std::string & /*script_id*/,
                                                              const std::string & /*execution_id*/) override {
    if (succeed) {
      return {};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }
};

// =============================================================================
// Helpers
// =============================================================================

// Minimal manifest that provides a component and app with SCRIPTS support
const char * kScriptTestManifest = R"(
manifest_version: "1.0"
metadata:
  name: "script-handlers-test"
  version: "1.0.0"
components:
  - id: "ecu"
    name: "ECU"
    namespace: "/vehicle"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "ecu"
)";

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_script_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    ADD_FAILURE() << "Failed to create temp manifest: " << std::strerror(errno);
    return {};
  }
  close(fd);

  std::ofstream out(path_template);
  if (!out) {
    ADD_FAILURE() << "Failed to open temp manifest for writing";
    std::remove(path_template);
    return {};
  }
  out << contents;
  out.close();
  return path_template;
}

// Build a request whose `matches` array carries the per-route capture groups.
// The typed handler reads `req.path_param("N")` (1-indexed by regex group), so
// the captures must be populated by `std::regex_match` against the per-route
// pattern - the production framework wires this through cpp-httplib's regex
// router. We replicate that wiring here so the test exercises the same typed
// surface as the framework.
httplib::Request make_list_request(const std::string & entity_type, const std::string & entity_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts";
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

httplib::Request make_script_request(const std::string & entity_type, const std::string & entity_id,
                                     const std::string & script_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts/" + script_id;
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts/([^/]+)";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

httplib::Request make_execution_request(const std::string & entity_type, const std::string & entity_id,
                                        const std::string & script_id, const std::string & execution_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts/" + script_id + "/executions/" + execution_id;
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts/([^/]+)/executions/([^/]+)";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

}  // namespace

// =============================================================================
// Test fixture - no backend (tests 501 responses with null HandlerContext)
//
// PR-403 commit 24: all 8 handlers now return `http::Result<T>`. Tests inspect
// the typed ErrorInfo directly instead of round-tripping through httplib::
// Response. The 501 short-circuit fires before any path-capture lookup, so a
// default-constructed TypedRequest is enough.
// =============================================================================

class ScriptHandlersNoBackendTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  // Null script manager -> the typed handlers short-circuit with 501.
  ScriptHandlers handlers_{ctx_, nullptr};

  static httplib::Request empty_request() {
    return httplib::Request{};
  }
};

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersNoBackendTest, UploadReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.upload_script(typed, http::MultipartBody{});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_041
TEST_F(ScriptHandlersNoBackendTest, ListReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.list_scripts(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_042
TEST_F(ScriptHandlersNoBackendTest, GetReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_script(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersNoBackendTest, DeleteReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.delete_script(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersNoBackendTest, StartExecutionReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.start_execution(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_046
TEST_F(ScriptHandlersNoBackendTest, GetExecutionReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_execution(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_047
TEST_F(ScriptHandlersNoBackendTest, ControlExecutionReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.control_execution(typed, dto::ScriptControlRequest{});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

TEST_F(ScriptHandlersNoBackendTest, DeleteExecutionReturns501WhenNoBackend) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.delete_execution(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// Also test with a ScriptManager that has no backend set
TEST_F(ScriptHandlersNoBackendTest, ScriptManagerWithoutBackendReturns501) {
  ScriptManager manager;  // has_backend() returns false
  ScriptHandlers handlers(ctx_, &manager);

  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers.list_scripts(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// =============================================================================
// Test fixture - full setup with GatewayNode, manifest entities, mock provider
// =============================================================================

class ScriptHandlersErrorMappingTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> node_;

  static void SetUpTestSuite() {
    std::vector<std::string> args = {"test_script_handlers", "--ros-args", "-p",
                                     "server.port:=0",       "-p",         "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    node_ = std::make_shared<GatewayNode>();
  }

  static void TearDownTestSuite() {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kScriptTestManifest);
    ASSERT_FALSE(manifest_path_.empty());

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

    auto areas = node_->get_discovery_manager()->discover_areas();
    auto components = node_->get_discovery_manager()->discover_components();
    auto apps = node_->get_discovery_manager()->discover_apps();
    auto functions = node_->get_discovery_manager()->discover_functions();

    auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_, auth_, tls_, nullptr);

    mock_provider_ = std::make_unique<ErrorMockScriptProvider>();
    script_manager_ = std::make_unique<ScriptManager>();
    script_manager_->set_backend(mock_provider_.get());

    handlers_ = std::make_unique<ScriptHandlers>(*ctx_, script_manager_.get());
  }

  void TearDown() override {
    handlers_.reset();
    script_manager_.reset();
    mock_provider_.reset();
    ctx_.reset();
    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  /// Helper: call get_script with entity "ecu" and trigger the mock error.
  /// Returns the typed Result<T> so each test can inspect status + code.
  http::Result<dto::ScriptMetadata> call_get_script_with_error(ScriptBackendError err, httplib::Request & req_storage) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    req_storage = make_script_request("components", "ecu", "test_script");
    http::TypedRequest typed(req_storage);
    return handlers_->get_script(typed);
  }

  /// Helper: call delete_script with entity "ecu" and trigger the mock error.
  http::Result<http::NoContent> call_delete_script_with_error(ScriptBackendError err, httplib::Request & req_storage) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    req_storage = make_script_request("components", "ecu", "test_script");
    http::TypedRequest typed(req_storage);
    return handlers_->delete_script(typed);
  }

  /// Helper: call start_execution with entity "ecu" and trigger the mock error.
  http::Result<std::pair<http::Accepted<dto::ScriptExecution>, http::ResponseAttachments>>
  call_start_execution_with_error(ScriptBackendError err, httplib::Request & req_storage) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    req_storage = make_script_request("components", "ecu", "test_script");
    req_storage.body = R"({"execution_type": "now"})";
    http::TypedRequest typed(req_storage);
    return handlers_->start_execution(typed);
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<ErrorMockScriptProvider> mock_provider_;
  std::unique_ptr<ScriptManager> script_manager_;
  std::unique_ptr<ScriptHandlers> handlers_;
  std::string manifest_path_;
};

// =============================================================================
// Error mapping: ScriptBackendError -> HTTP status code
// =============================================================================

// @verifies REQ_INTEROP_042
TEST_F(ScriptHandlersErrorMappingTest, NotFoundMapsTo404) {
  httplib::Request req;
  auto result = call_get_script_with_error(ScriptBackendError::NotFound, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersErrorMappingTest, ManagedScriptMapsTo409) {
  httplib::Request req;
  auto result = call_delete_script_with_error(ScriptBackendError::ManagedScript, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  // Vendor-specific code: the renderer in the typed router wraps it as a
  // vendor-error envelope on the wire, but at the typed surface the ErrorInfo
  // still carries the bare vendor code.
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SCRIPT_MANAGED);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, AlreadyRunningMapsTo409) {
  httplib::Request req;
  auto result = call_delete_script_with_error(ScriptBackendError::AlreadyRunning, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SCRIPT_RUNNING);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, ConcurrencyLimitMapsTo429) {
  httplib::Request req;
  auto result = call_start_execution_with_error(ScriptBackendError::ConcurrencyLimit, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 429);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SCRIPT_CONCURRENCY_LIMIT);
}

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, FileTooLargeMapsTo413) {
  // Upload handler needs multipart file field - use get_script with FileTooLarge instead
  // since the backend-error mapping is shared across all handlers.
  httplib::Request req;
  auto result = call_get_script_with_error(ScriptBackendError::FileTooLarge, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 413);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SCRIPT_FILE_TOO_LARGE);
}

TEST_F(ScriptHandlersErrorMappingTest, InternalErrorMapsTo500) {
  httplib::Request req;
  auto result = call_get_script_with_error(ScriptBackendError::Internal, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 500);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INTERNAL_ERROR);
}

// =============================================================================
// Success paths: upload -> 201, start -> 202, delete -> 204
// =============================================================================

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, UploadReturns201WithLocation) {
  mock_provider_->succeed = true;

  auto req = make_list_request("components", "ecu");
  req.set_header("Content-Type", "multipart/form-data; boundary=----WebKitFormBoundary");

  http::MultipartBody body;
  httplib::MultipartFormData file_part;
  file_part.name = "file";
  file_part.filename = "diag.py";
  file_part.content = "#!/usr/bin/env python3\nprint('hello')";
  file_part.content_type = "application/octet-stream";
  body.parts.push_back(std::move(file_part));

  http::TypedRequest typed(req);
  auto result = handlers_->upload_script(typed, body);

  ASSERT_TRUE(result.has_value());
  const auto & [created, att] = result.value();
  const auto & upload_resp = created.value;
  // 201 is declared by the Created<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result.value().first)>::value, 201);
  EXPECT_FALSE(att.status_override.has_value());

  // Location header is appended to ResponseAttachments::headers.
  bool found_location = false;
  for (const auto & [name, value] : att.headers) {
    if (name == "Location") {
      found_location = true;
      EXPECT_NE(value.find("/scripts/uploaded_001"), std::string::npos);
    }
  }
  EXPECT_TRUE(found_location);

  EXPECT_EQ(upload_resp.id, "uploaded_001");
  EXPECT_EQ(upload_resp.name, "Uploaded Script");
}

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, UploadRejectsWrongContentType) {
  mock_provider_->succeed = true;

  auto req = make_list_request("components", "ecu");
  // No Content-Type header set - should be rejected
  http::MultipartBody body;
  httplib::MultipartFormData file_part;
  file_part.name = "file";
  file_part.filename = "diag.py";
  file_part.content = "#!/usr/bin/env python3\nprint('hello')";
  file_part.content_type = "application/octet-stream";
  body.parts.push_back(std::move(file_part));

  http::TypedRequest typed(req);
  auto result = handlers_->upload_script(typed, body);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, StartExecutionReturns202WithLocation) {
  mock_provider_->succeed = true;

  auto req = make_script_request("components", "ecu", "test_script");
  req.body = R"({"execution_type": "now"})";

  http::TypedRequest typed(req);
  auto result = handlers_->start_execution(typed);

  ASSERT_TRUE(result.has_value());
  const auto & [accepted, att] = result.value();
  const auto & exec_dto = accepted.value;
  // 202 is declared by the Accepted<> return type, not by a runtime override.
  EXPECT_EQ(http::dto_alternate_status<decltype(result.value().first)>::value, 202);
  EXPECT_FALSE(att.status_override.has_value());

  bool found_location = false;
  for (const auto & [name, value] : att.headers) {
    if (name == "Location") {
      found_location = true;
      EXPECT_NE(value.find("/executions/exec_001"), std::string::npos);
    }
  }
  EXPECT_TRUE(found_location);

  EXPECT_EQ(exec_dto.id, "exec_001");
  EXPECT_EQ(exec_dto.status, "running");
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersErrorMappingTest, DeleteReturns204) {
  mock_provider_->succeed = true;

  auto req = make_script_request("components", "ecu", "test_script");
  http::TypedRequest typed(req);
  auto result = handlers_->delete_script(typed);

  // Success branch carries http::NoContent{}; the framework's del<NoContent>
  // wrapper turns this into a 204 on the wire.
  ASSERT_TRUE(result.has_value());
}

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, ListEmitsTypedHateoasLinks) {
  mock_provider_->succeed = true;

  auto req = make_list_request("components", "ecu");
  http::TypedRequest typed(req);
  auto result = handlers_->list_scripts(typed);

  ASSERT_TRUE(result.has_value());
  const auto & list = result.value();
  ASSERT_TRUE(list.links.has_value());
  EXPECT_NE(list.links->self.find("/components/ecu/scripts"), std::string::npos);
  ASSERT_TRUE(list.links->parent.has_value());
  EXPECT_NE(list.links->parent->find("/components/ecu"), std::string::npos);
}

// =============================================================================
// Adversarial input validation for is_valid_resource_id
// =============================================================================

TEST_F(ScriptHandlersErrorMappingTest, PathTraversalScriptIdRejected) {
  // cpp-httplib's `[^/]+` regex never lets `/`-bearing inputs reach this code
  // path in production; the defense-in-depth `is_valid_resource_id` check
  // exists for non-slash-but-still-malicious inputs (relative-traversal
  // sentinels like `..` or path-separator-equivalents). Use a slash-free
  // payload so the regex DOES match and the validator is exercised.
  auto req = make_script_request("components", "ecu", "..");
  http::TypedRequest typed(req);
  auto result = handlers_->get_script(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, ShellMetacharsRejected) {
  auto req = make_script_request("components", "ecu", "test;rm");
  http::TypedRequest typed(req);
  auto result = handlers_->get_script(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, OverlongScriptIdRejected) {
  auto req = make_script_request("components", "ecu", std::string(257, 'a'));
  http::TypedRequest typed(req);
  auto result = handlers_->get_script(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, ValidScriptIdWithSpecialCharsAccepted) {
  // Valid ID with underscore and hyphen passes validation (gets 404 from mock provider).
  mock_provider_->succeed = false;
  mock_provider_->error_code = ScriptBackendError::NotFound;
  mock_provider_->error_message = "not found";

  auto req = make_script_request("components", "ecu", "my-script_01");
  http::TypedRequest typed(req);
  auto result = handlers_->get_script(typed);
  ASSERT_FALSE(result.has_value());
  // Should NOT be 400 - it passes validation and reaches the provider (which returns 404).
  EXPECT_EQ(result.error().http_status, 404);
}

// =============================================================================
// Control execution validation
//
// PR-403 commit 24: the typed router parses ScriptControlRequest via
// `parse_body<TBody>` BEFORE invoking the handler, so missing/empty/invalid
// `action` is reported as a 400 by the framework's reader, not by the handler.
// These tests therefore inject the validated body directly to exercise the
// handler's own error surface (provider-side InvalidInput, etc.).
// =============================================================================

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionRejectsBogusActionFromProvider) {
  // The framework parses the typed body upstream; this test simulates a
  // provider that rejects an otherwise structurally-valid action.
  mock_provider_->succeed = false;
  mock_provider_->error_code = ScriptBackendError::InvalidInput;
  mock_provider_->error_message = "Unknown action: bogus";

  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  http::TypedRequest typed(req);
  dto::ScriptControlRequest body;
  body.action = "stop";  // would pass parse_body's enum check upstream
  auto result = handlers_->control_execution(typed, body);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionStopForwardsToProvider) {
  mock_provider_->succeed = true;

  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  http::TypedRequest typed(req);
  dto::ScriptControlRequest body;
  body.action = "stop";
  auto result = handlers_->control_execution(typed, body);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value().id, "exec_001");
  EXPECT_EQ(result.value().status, "terminated");
}

TEST_F(ScriptHandlersErrorMappingTest, NotRunningMapsTo409) {
  httplib::Request req;
  auto result = call_get_script_with_error(ScriptBackendError::NotRunning, req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SCRIPT_NOT_RUNNING);
}
