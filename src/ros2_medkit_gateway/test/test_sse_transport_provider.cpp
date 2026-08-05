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

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/sse_transport_provider.hpp"
#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"

using namespace ros2_medkit_gateway;

class SseTransportProviderTest : public ::testing::Test {
 protected:
  SubscriptionManager mgr_{10};
  std::shared_ptr<SSEClientTracker> tracker_ = std::make_shared<SSEClientTracker>(5);
  SseTransportProvider provider_{mgr_, tracker_};
};

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, ProtocolReturnsSse) {
  EXPECT_EQ(provider_.protocol(), "sse");
}

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, StartReturnsEventsUrl) {
  CyclicSubscriptionInfo info;
  info.id = "sub_001";
  info.entity_type = "apps";
  info.entity_id = "node1";

  ResourceSamplerFn sampler = [](const std::string &, const std::string &) {
    return nlohmann::json{{"value", 42}};
  };

  auto result = provider_.start(info, sampler, nullptr);
  ASSERT_TRUE(result.has_value());
  EXPECT_NE(result->find("/apps/node1/cyclic-subscriptions/sub_001/events"), std::string::npos);
}

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, StopRemovesStream) {
  CyclicSubscriptionInfo info;
  info.id = "sub_002";
  info.entity_type = "components";
  info.entity_id = "ecu1";

  ResourceSamplerFn sampler = [](const std::string &, const std::string &) {
    return nlohmann::json{};
  };

  auto result = provider_.start(info, sampler, nullptr);
  ASSERT_TRUE(result.has_value());

  // stop should not throw even if subscription doesn't exist in mgr
  provider_.stop("sub_002");

  // Calling stop again on already-removed stream should be a no-op
  provider_.stop("sub_002");
}

TEST_F(SseTransportProviderTest, NotifyUpdateIsNoOp) {
  // SSE transport re-reads each iteration, so notify_update is a no-op
  // Just verify it doesn't crash
  provider_.notify_update("nonexistent");
}

namespace {

/// Drive one `next_event` tick and return the JSON in the frame's `data:`
/// field. Writing `false` back from the sink stops the loop before its
/// interval wait, so a tick costs no wall time.
nlohmann::json first_frame_payload(http::SseStream & stream) {
  std::string written;
  httplib::DataSink sink;
  sink.write = [&written](const char * data, size_t len) {
    written.assign(data, len);
    return false;  // one frame is enough; also skips the interval wait
  };
  sink.is_writable = [] {
    return true;
  };
  sink.done = [] {};
  stream.next_event(sink);

  const auto data_pos = written.find("data: ");
  EXPECT_NE(data_pos, std::string::npos) << "frame carried no data: field: " << written;
  if (data_pos == std::string::npos) {
    return nlohmann::json::object();
  }
  return nlohmann::json::parse(written.substr(data_pos + 6));
}

}  // namespace

// A sampler failure is reported as a vendor error, and the vendor-error
// sentinel is only usable together with the code it stands in for: on its own
// `"error_code": "vendor-error"` tells a client that something vendor-specific
// went wrong and not what. Every other emitter of the sentinel pairs the two;
// this one shipped the sentinel alone, and `SubscriptionEventFrame.error`
// declares the pair.
TEST_F(SseTransportProviderTest, SamplerFailureFrameNamesTheVendorCode) {
  auto created =
      mgr_.create("node1", "apps", "/api/v1/apps/node1/data", "data", "/temperature", "sse", CyclicInterval::FAST, 60);
  ASSERT_TRUE(created.has_value()) << created.error();

  ResourceSamplerFn failing = [](const std::string &,
                                 const std::string &) -> tl::expected<nlohmann::json, std::string> {
    return tl::make_unexpected(std::string("topic has no publisher"));
  };
  ASSERT_TRUE(provider_.start(*created, failing, nullptr).has_value());

  auto stream = provider_.make_sse_stream(created->id);
  ASSERT_TRUE(stream.has_value());

  const auto payload = first_frame_payload(*stream);
  ASSERT_TRUE(payload.contains("error")) << payload.dump();
  EXPECT_FALSE(payload.contains("payload")) << "a failed sample must not also claim a value";
  const auto & error = payload.at("error");
  EXPECT_EQ(error.at("error_code"), ERR_VENDOR_ERROR);
  EXPECT_EQ(error.at("vendor_code"), ERR_X_MEDKIT_RESOURCE_SAMPLE_FAILED);
  EXPECT_EQ(error.at("message"), "topic has no publisher");
  EXPECT_TRUE(payload.contains("timestamp"));
}

// The success side of the same frame, so the test above cannot pass by the
// stream simply always erroring.
TEST_F(SseTransportProviderTest, SuccessfulSampleFrameCarriesPayloadAndNoError) {
  auto created =
      mgr_.create("node2", "apps", "/api/v1/apps/node2/data", "data", "/temperature", "sse", CyclicInterval::FAST, 60);
  ASSERT_TRUE(created.has_value()) << created.error();

  ResourceSamplerFn ok = [](const std::string &, const std::string &) {
    return nlohmann::json{{"value", 42}};
  };
  ASSERT_TRUE(provider_.start(*created, ok, nullptr).has_value());

  auto stream = provider_.make_sse_stream(created->id);
  ASSERT_TRUE(stream.has_value());

  const auto payload = first_frame_payload(*stream);
  EXPECT_FALSE(payload.contains("error")) << payload.dump();
  ASSERT_TRUE(payload.contains("payload"));
  EXPECT_EQ(payload.at("payload").at("value"), 42);
  EXPECT_TRUE(payload.contains("timestamp"));
}
