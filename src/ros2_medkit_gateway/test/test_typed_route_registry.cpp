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

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>

#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"

// -----------------------------------------------------------------------------
// Test DTOs - declared at namespace scope so dto_fields / dto_name
// specializations can live in the dto namespace.
// -----------------------------------------------------------------------------

namespace ros2_medkit_gateway {
namespace dto {

struct TypedRouteTestDto {
  std::string name;
  int count{0};
};

template <>
inline constexpr auto dto_fields<TypedRouteTestDto> =
    std::make_tuple(field("name", &TypedRouteTestDto::name), field("count", &TypedRouteTestDto::count));

template <>
inline constexpr std::string_view dto_name<TypedRouteTestDto> = "TypedRouteTestDto";

struct TypedRouteTestReq {
  std::string greeting;
};

template <>
inline constexpr auto dto_fields<TypedRouteTestReq> = std::make_tuple(field("greeting", &TypedRouteTestReq::greeting));

template <>
inline constexpr std::string_view dto_name<TypedRouteTestReq> = "TypedRouteTestReq";

struct TypedRouteAltA {
  std::string a;
};

template <>
inline constexpr auto dto_fields<TypedRouteAltA> = std::make_tuple(field("a", &TypedRouteAltA::a));
template <>
inline constexpr std::string_view dto_name<TypedRouteAltA> = "TypedRouteAltA";

struct TypedRouteAltB {
  int b{0};
};

template <>
inline constexpr auto dto_fields<TypedRouteAltB> = std::make_tuple(field("b", &TypedRouteAltB::b));
template <>
inline constexpr std::string_view dto_name<TypedRouteAltB> = "TypedRouteAltB";

}  // namespace dto

// alternate-status specialization for one of the test alts: 202 for AltA.
namespace http {
template <>
struct dto_alternate_status<dto::TypedRouteAltA> {
  static constexpr int value = 202;
};
}  // namespace http

}  // namespace ros2_medkit_gateway

using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::dto::TypedRouteAltA;
using ros2_medkit_gateway::dto::TypedRouteAltB;
using ros2_medkit_gateway::dto::TypedRouteTestDto;
using ros2_medkit_gateway::dto::TypedRouteTestReq;
using ros2_medkit_gateway::http::ResponseAttachments;
using ros2_medkit_gateway::http::Result;
using ros2_medkit_gateway::http::TypedRequest;
using ros2_medkit_gateway::openapi::ErrorRenderer;
using ros2_medkit_gateway::openapi::RouteEntry;
using ros2_medkit_gateway::openapi::RouteRegistry;

namespace {

/// Spin up a cpp-httplib server with the given registry and return the bound
/// port plus a stop hook. The server runs on a dedicated thread; the dtor
/// stops the server and joins the thread.
struct ScopedServer {
  std::unique_ptr<httplib::Server> server;
  std::thread thread;
  int port{0};

  ScopedServer() = default;
  ScopedServer(ScopedServer &&) noexcept = default;
  ScopedServer & operator=(ScopedServer &&) noexcept = default;
  ScopedServer(const ScopedServer &) = delete;
  ScopedServer & operator=(const ScopedServer &) = delete;

  ~ScopedServer() {
    if (server) {
      server->stop();
    }
    if (thread.joinable()) {
      thread.join();
    }
  }
};

ScopedServer start_server(const RouteRegistry & reg) {
  ScopedServer s;
  s.server = std::make_unique<httplib::Server>();
  reg.register_all(*s.server, "/api/v1");
  s.port = s.server->bind_to_any_port("127.0.0.1");
  s.thread = std::thread([srv = s.server.get()]() {
    srv->listen_after_bind();
  });
  s.server->wait_until_ready();
  return s;
}

}  // namespace

// =============================================================================
// 1. Typed GET round-trip
// =============================================================================

TEST(TypedRouteRegistry, TypedGetReturnsDtoBodyAnd200) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest)> handler =
      [](TypedRequest /*req*/) -> Result<TypedRouteTestDto> {
    TypedRouteTestDto dto;
    dto.name = "hello";
    dto.count = 7;
    return dto;
  };
  reg.get<TypedRouteTestDto>("/test/health", std::move(handler)).tag("Test").summary("Test endpoint");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/test/health");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  auto body = nlohmann::json::parse(r->body);
  EXPECT_EQ(body["name"], "hello");
  EXPECT_EQ(body["count"], 7);
}

// =============================================================================
// 2. Typed POST body parsing
// =============================================================================

TEST(TypedRouteRegistry, TypedPostParsesBodyAndReturnsDto) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest, TypedRouteTestReq)> handler =
      [](TypedRequest /*req*/, const TypedRouteTestReq & body) -> Result<TypedRouteTestDto> {
    TypedRouteTestDto dto;
    dto.name = body.greeting;
    dto.count = static_cast<int>(body.greeting.size());
    return dto;
  };
  reg.post<TypedRouteTestReq, TypedRouteTestDto>("/test/echo", std::move(handler)).tag("Test").summary("Echo");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);

  // Happy path.
  {
    nlohmann::json req_body{{"greeting", "hi"}};
    auto r = cli.Post("/api/v1/test/echo", req_body.dump(), "application/json");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->status, 200);
    auto body = nlohmann::json::parse(r->body);
    EXPECT_EQ(body["name"], "hi");
    EXPECT_EQ(body["count"], 2);
  }

  // Malformed JSON -> 400 invalid-request.
  {
    auto r = cli.Post("/api/v1/test/echo", "not-json", "application/json");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->status, 400);
    auto body = nlohmann::json::parse(r->body);
    EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
  }

  // Missing required field -> 400 invalid-request with field error.
  {
    auto r = cli.Post("/api/v1/test/echo", "{}", "application/json");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->status, 400);
    auto body = nlohmann::json::parse(r->body);
    EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
    ASSERT_TRUE(body.contains("parameters"));
    ASSERT_TRUE(body["parameters"].contains("fields"));
    ASSERT_TRUE(body["parameters"]["fields"].is_array());
    ASSERT_FALSE(body["parameters"]["fields"].empty());
    EXPECT_EQ(body["parameters"]["fields"][0]["field"], "greeting");
  }
}

// =============================================================================
// 3. Error renderer per-route
// =============================================================================

TEST(TypedRouteRegistry, ErrorRendererOAuth2OverridesSovdShape) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest)> handler =
      [](TypedRequest /*req*/) -> Result<TypedRouteTestDto> {
    ErrorInfo err;
    err.code = "invalid_grant";
    err.message = "credentials rejected";
    err.http_status = 400;
    return tl::make_unexpected(err);
  };
  reg.get<TypedRouteTestDto>("/test/oauth", std::move(handler))
      .tag("Test")
      .summary("OAuth-shaped error")
      .error_renderer(ErrorRenderer::kOAuth2Error);

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/test/oauth");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 400);
  auto body = nlohmann::json::parse(r->body);
  // OAuth2 shape: { error, error_description }
  EXPECT_EQ(body["error"], "invalid_grant");
  EXPECT_EQ(body["error_description"], "credentials rejected");
  // Must NOT carry the SOVD GenericError keys.
  EXPECT_FALSE(body.contains("error_code"));
  EXPECT_FALSE(body.contains("message"));
}

TEST(TypedRouteRegistry, DefaultErrorRendererIsSovdGenericError) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest)> handler =
      [](TypedRequest /*req*/) -> Result<TypedRouteTestDto> {
    ErrorInfo err;
    err.code = ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND;
    err.message = "not found";
    err.http_status = 404;
    return tl::make_unexpected(err);
  };
  reg.get<TypedRouteTestDto>("/test/notfound", std::move(handler)).tag("Test").summary("Default error");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/test/notfound");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 404);
  auto body = nlohmann::json::parse(r->body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
  EXPECT_EQ(body["message"], "not found");
  EXPECT_FALSE(body.contains("error_description"));
}

// =============================================================================
// 4. ResponseAttachments status / header overrides
// =============================================================================

TEST(TypedRouteRegistry, AttachmentsApplyStatusAndHeaders) {
  RouteRegistry reg;
  using PairT = std::pair<TypedRouteTestDto, ResponseAttachments>;
  std::function<Result<PairT>(TypedRequest, TypedRouteTestReq)> handler =
      [](TypedRequest /*req*/, const TypedRouteTestReq & body) -> Result<PairT> {
    TypedRouteTestDto dto;
    dto.name = body.greeting;
    dto.count = 1;
    ResponseAttachments att;
    att.with_status(201).with_header("Location", "/api/v1/test/items/x").with_header("X-Medkit-Trace-Id", "trace1");
    return std::make_pair(std::move(dto), std::move(att));
  };
  reg.post<TypedRouteTestReq, TypedRouteTestDto>("/test/items", std::move(handler)).tag("Test").summary("Create");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  nlohmann::json req_body{{"greeting", "n"}};
  auto r = cli.Post("/api/v1/test/items", req_body.dump(), "application/json");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 201);
  EXPECT_EQ(r->get_header_value("Location"), "/api/v1/test/items/x");
  EXPECT_EQ(r->get_header_value("X-Medkit-Trace-Id"), "trace1");
  auto body = nlohmann::json::parse(r->body);
  EXPECT_EQ(body["name"], "n");
}

// =============================================================================
// 5. Alternates dispatch via dto_alternate_status<T>
// =============================================================================

TEST(TypedRouteRegistry, PostAlternatesPicksStatusFromActiveVariant) {
  // AltA -> 202 (specialized above), AltB -> 200 (default).
  //
  // The attachments overload is not incidental here: a 202 alternate makes the
  // registry declare a `Location` header on that response, and only this
  // overload gives the handler a channel to send one. The non-attachments
  // overload rejects the combination at compile time. What this test proves is
  // unchanged - the status still comes from the active alternate, not from the
  // attachments, which carry nothing.
  RouteRegistry reg;
  using VarT = std::variant<TypedRouteAltA, TypedRouteAltB>;
  using PairT = std::pair<VarT, ros2_medkit_gateway::http::ResponseAttachments>;
  std::function<Result<PairT>(TypedRequest, TypedRouteTestReq)> handler =
      [](TypedRequest /*req*/, const TypedRouteTestReq & body) -> Result<PairT> {
    ros2_medkit_gateway::http::ResponseAttachments att;
    if (body.greeting == "a") {
      TypedRouteAltA a;
      a.a = "alt-a";
      att.with_location("/api/v1/test/alt/a");
      return PairT{VarT{a}, std::move(att)};
    }
    TypedRouteAltB b;
    b.b = 99;
    return PairT{VarT{b}, std::move(att)};
  };
  reg.post_alternates<TypedRouteTestReq, TypedRouteAltA, TypedRouteAltB>("/test/alt", std::move(handler))
      .tag("Test")
      .summary("Alternates");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);

  {
    nlohmann::json req_body{{"greeting", "a"}};
    auto r = cli.Post("/api/v1/test/alt", req_body.dump(), "application/json");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->status, 202) << "AltA must use the specialized 202 status";
    EXPECT_EQ(r->get_header_value("Location"), "/api/v1/test/alt/a");
    auto body = nlohmann::json::parse(r->body);
    EXPECT_EQ(body["a"], "alt-a");
  }
  {
    nlohmann::json req_body{{"greeting", "b"}};
    auto r = cli.Post("/api/v1/test/alt", req_body.dump(), "application/json");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->status, 200) << "AltB must use the default 200 status";
    auto body = nlohmann::json::parse(r->body);
    EXPECT_EQ(body["b"], 99);
  }
}

// =============================================================================
// 6. Schema auto-population
// =============================================================================

TEST(TypedRouteRegistry, TypedGetAutoPopulatesResponseSchemaRef) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest)> handler =
      [](TypedRequest /*req*/) -> Result<TypedRouteTestDto> {
    return TypedRouteTestDto{};
  };
  reg.get<TypedRouteTestDto>("/test/schema", std::move(handler)).tag("Test").summary("Schema check");

  auto paths = reg.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/test/schema"));
  auto & resp_200 = paths["/test/schema"]["get"]["responses"]["200"];
  ASSERT_TRUE(resp_200.contains("content"));
  ASSERT_TRUE(resp_200["content"].contains("application/json"));
  auto & schema = resp_200["content"]["application/json"]["schema"];
  ASSERT_TRUE(schema.contains("$ref"));
  EXPECT_EQ(schema["$ref"], "#/components/schemas/TypedRouteTestDto");
}

TEST(TypedRouteRegistry, TypedPostAutoPopulatesRequestBodySchemaRef) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest, TypedRouteTestReq)> handler =
      [](TypedRequest /*req*/, const TypedRouteTestReq & /*body*/) -> Result<TypedRouteTestDto> {
    return TypedRouteTestDto{};
  };
  reg.post<TypedRouteTestReq, TypedRouteTestDto>("/test/body", std::move(handler)).tag("Test").summary("Body");

  auto paths = reg.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/test/body"));
  auto & op = paths["/test/body"]["post"];
  ASSERT_TRUE(op.contains("requestBody"));
  auto & req_schema = op["requestBody"]["content"]["application/json"]["schema"];
  ASSERT_TRUE(req_schema.contains("$ref"));
  EXPECT_EQ(req_schema["$ref"], "#/components/schemas/TypedRouteTestReq");
}

// =============================================================================
// 7. Compile-time DTO check
//
// Documented contract: `reg.get<int>(path, ...)` must fail to compile because
// `int` is not a DTO. We cannot portably unit-test a compile failure, so this
// is a documentation comment only; CI compilers will reject the call site.
// =============================================================================

// =============================================================================
// Typed DELETE + NoContent
// =============================================================================

TEST(TypedRouteRegistry, TypedDeleteWithNoContentReturns204) {
  RouteRegistry reg;
  std::function<Result<ros2_medkit_gateway::http::NoContent>(TypedRequest)> handler =
      [](TypedRequest /*req*/) -> Result<ros2_medkit_gateway::http::NoContent> {
    return ros2_medkit_gateway::http::NoContent{};
  };
  reg.del<ros2_medkit_gateway::http::NoContent>("/test/item", std::move(handler)).tag("Test").summary("Delete");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Delete("/api/v1/test/item");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 204);
  EXPECT_TRUE(r->body.empty());
}

// =============================================================================
// Typed PUT round-trip
// =============================================================================

TEST(TypedRouteRegistry, TypedPutRoundTrip) {
  RouteRegistry reg;
  std::function<Result<TypedRouteTestDto>(TypedRequest, TypedRouteTestReq)> handler =
      [](TypedRequest /*req*/, const TypedRouteTestReq & body) -> Result<TypedRouteTestDto> {
    TypedRouteTestDto dto;
    dto.name = body.greeting;
    dto.count = 42;
    return dto;
  };
  reg.put<TypedRouteTestReq, TypedRouteTestDto>("/test/put", std::move(handler)).tag("Test").summary("Put");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  nlohmann::json req_body{{"greeting", "putme"}};
  auto r = cli.Put("/api/v1/test/put", req_body.dump(), "application/json");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  auto body = nlohmann::json::parse(r->body);
  EXPECT_EQ(body["name"], "putme");
  EXPECT_EQ(body["count"], 42);
}

// =============================================================================
// docs_subtree - catch-all regex, documented under a path template
// =============================================================================

TEST(TypedRouteRegistry, DocsSubtreeRegexRoutes) {
  RouteRegistry reg;
  reg.docs_subtree("/{doc_path}/docs", "/(.*)/docs$",
                   [](const httplib::Request & req, httplib::Response & res) {
                     res.status = 200;
                     res.set_content("docs:" + req.matches[1].str(), "text/plain");
                   })
      .tag("Server")
      .summary("Scoped docs");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/foo/bar/docs");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  EXPECT_EQ(r->body, "docs:foo/bar");
}

// The regex is what cpp-httplib matches; the path template is what the
// document publishes. They are separate arguments precisely so the `(.*)`
// never reaches the document as a path key - a client reading `/(.*)/docs$`
// has no way to fill it in.
TEST(TypedRouteRegistry, DocsSubtreePublishesTheTemplateNotTheRegex) {
  RouteRegistry reg;
  reg.docs_subtree("/{doc_path}/docs", "/(.*)/docs$",
                   [](const httplib::Request &, httplib::Response & res) {
                     res.status = 200;
                   })
      .tag("Server")
      .summary("Scoped docs");

  const auto paths = reg.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/{doc_path}/docs"));
  EXPECT_FALSE(paths.contains("/(.*)/docs$"));
  // The helper declares the success status, so the call site above did not -
  // and could not without hand-attaching a 2xx.
  const auto & ok = paths["/{doc_path}/docs"]["get"]["responses"]["200"];
  EXPECT_FALSE(ok["description"].get<std::string>().empty());
  EXPECT_EQ(ok["content"]["application/json"]["schema"]["type"], "object");
  // The template's parameter is published, so a generated client knows there
  // is something to substitute.
  const auto & params = paths["/{doc_path}/docs"]["get"]["parameters"];
  ASSERT_EQ(params.size(), 1U);
  EXPECT_EQ(params[0]["name"], "doc_path");
  EXPECT_EQ(params[0]["in"], "path");
}

// =============================================================================
// 7. binary_download: the Range contract the document now advertises
// =============================================================================

namespace {

// Range-capable download over a fixed in-memory payload. `provider` honours
// offset/length, which is what makes cpp-httplib's range machinery usable.
constexpr std::string_view kDownloadPayload = "0123456789abcdef";

Result<ros2_medkit_gateway::http::BinaryResponse> range_download_handler(TypedRequest /*req*/) {
  ros2_medkit_gateway::http::BinaryResponse resp;
  resp.content_type = "application/octet-stream";
  resp.filename = "payload.bin";
  resp.supports_ranges = true;
  resp.total_size = kDownloadPayload.size();
  resp.provider = [](uint64_t offset, uint64_t length, httplib::DataSink & sink) -> bool {
    sink.write(kDownloadPayload.data() + offset, static_cast<std::size_t>(length));
    return true;
  };
  return resp;
}

RouteEntry & seed_download(RouteRegistry & reg, const std::string & path) {
  std::function<Result<ros2_medkit_gateway::http::BinaryResponse>(TypedRequest)> h = &range_download_handler;
  // Deliberately the single type this fixture's handler serves, with no
  // catch-all: the production route needs `*/*` because its served set is open,
  // but a closed list here is what lets the assertions below tell an exact
  // declaration from a wildcard that would match anything.
  return reg.binary_download(path, std::move(h), {"application/octet-stream"});
}

}  // namespace

TEST(TypedRouteRegistry, BinaryDownloadSendsAcceptRangesOnAPlainGet) {
  // The document declares `Accept-Ranges` on the 200. cpp-httplib only fills it
  // in for HEAD, so if the framework ever stops setting it the document starts
  // advertising a header nobody sends.
  RouteRegistry reg;
  seed_download(reg, "/test/blob").tag("Test").summary("Download");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/test/blob");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 200);
  EXPECT_EQ(r->get_header_value("Accept-Ranges"), "bytes");
  EXPECT_NE(r->get_header_value("Content-Disposition").find("payload.bin"), std::string::npos);
  EXPECT_EQ(r->body, std::string(kDownloadPayload));
}

TEST(TypedRouteRegistry, BinaryDownloadAnswers206WithContentRangeForARangeRequest) {
  // The 206 is not the handler's doing - it never assigns res.status, so
  // cpp-httplib picks it from a non-empty req.ranges. This is the wire proof
  // behind declaring 206 and `Content-Range` in the document.
  RouteRegistry reg;
  seed_download(reg, "/test/blob").tag("Test").summary("Download");

  auto s = start_server(reg);
  httplib::Client cli("127.0.0.1", s.port);
  auto r = cli.Get("/api/v1/test/blob", {{"Range", "bytes=4-7"}});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->status, 206);
  EXPECT_EQ(r->body, "4567");
  EXPECT_EQ(r->get_header_value("Content-Range"), "bytes 4-7/16");
  EXPECT_EQ(r->get_header_value("Accept-Ranges"), "bytes");
}

TEST(TypedRouteRegistry, BinaryDownloadDeclaresBothSuccessStatusesAndMarksItself) {
  // Two 2xx codes are only legitimate because the route says why - the document
  // contract test reads `x-medkit-partial-content` to tell this apart from a
  // route declaring a status it can never return.
  RouteRegistry reg;
  seed_download(reg, "/test/blob").tag("Test").summary("Download");

  auto paths = reg.to_openapi_paths();
  auto & op = paths["/test/blob"]["get"];
  EXPECT_TRUE(op["x-medkit-partial-content"].get<bool>());
  EXPECT_FALSE(op.contains("x-medkit-alternates")) << "partial content is not variant dispatch";

  auto & responses = op["responses"];
  ASSERT_TRUE(responses.contains("200"));
  ASSERT_TRUE(responses.contains("206"));
  EXPECT_TRUE(responses["200"]["headers"].contains("Accept-Ranges"));
  EXPECT_TRUE(responses["200"]["headers"].contains("Content-Disposition"));
  EXPECT_TRUE(responses["206"]["headers"].contains("Content-Range"));
  EXPECT_FALSE(responses["206"]["description"].get<std::string>().empty());
}
