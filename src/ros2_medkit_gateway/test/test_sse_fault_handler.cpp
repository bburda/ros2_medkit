// Copyright 2026 sewon
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
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <chrono>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/handlers/sse_fault_handler.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::SSEClientTracker;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::SSEFaultHandler;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;

namespace {

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

FaultEvent make_fault_event(const std::string & event_type, const std::string & fault_code, int32_t sec,
                            uint32_t nanosec = 0) {
  FaultEvent event;
  event.event_type = event_type;
  event.timestamp.sec = sec;
  event.timestamp.nanosec = nanosec;
  event.fault.fault_code = fault_code;
  event.fault.description = "Synthetic SSE fault";
  event.fault.severity = Fault::SEVERITY_ERROR;
  event.fault.status = Fault::STATUS_CONFIRMED;
  event.fault.occurrence_count = 3;
  event.fault.reporting_sources = {"/apps/temp_sensor"};
  event.fault.first_occurred.sec = sec - 5;
  event.fault.last_occurred.sec = sec;
  event.fault.last_occurred.nanosec = nanosec;
  return event;
}

httplib::Request make_stream_request(const std::string & remote_addr, const std::string & last_event_id = "") {
  httplib::Request req;
  req.remote_addr = remote_addr;
  if (!last_event_id.empty()) {
    req.headers.emplace("Last-Event-ID", last_event_id);
  }
  return req;
}

std::string read_stream_once(httplib::Response & res, size_t writes_before_disconnect) {
  std::string output;
  size_t write_count = 0;

  httplib::DataSink sink;
  sink.write = [&](const char * data, size_t data_len) {
    output.append(data, data_len);
    write_count++;
    return write_count < writes_before_disconnect;
  };
  sink.is_writable = []() {
    return true;
  };
  sink.done = []() {};
  sink.done_with_trailer = [](const httplib::Headers &) {};

  EXPECT_TRUE(static_cast<bool>(res.content_provider_));
  EXPECT_FALSE(res.content_provider_(0, 0, sink));

  return output;
}

json parse_sse_payload(const std::string & sse_frame) {
  auto data_pos = sse_frame.find("data: ");
  if (data_pos == std::string::npos) {
    ADD_FAILURE() << "Missing SSE data field in frame: " << sse_frame;
    return json::object();
  }
  auto data_end = sse_frame.find("\n\n", data_pos);
  if (data_end == std::string::npos) {
    ADD_FAILURE() << "Missing SSE frame terminator in frame: " << sse_frame;
    return json::object();
  }
  auto payload = sse_frame.substr(data_pos + 6, data_end - (data_pos + 6));
  return json::parse(payload);
}

void release_stream(httplib::Response & res, bool success = false) {
  if (res.content_provider_resource_releaser_) {
    res.content_provider_resource_releaser_(success);
    res.content_provider_resource_releaser_ = nullptr;
  }
}

}  // namespace

class SSEFaultHandlerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    int server_port = reserve_local_port();
    ASSERT_NE(server_port, 0);

    // Per-test fault_manager namespace gives every TEST_F a unique events
    // topic (/test<N>/fault_manager/events). Without this, all tests share
    // /fault_manager/events, so a stale publisher/subscription in the same
    // DDS domain could leak messages across tests when the suite is run
    // alongside other gateway test binaries.
    const int test_id = test_counter_++;
    auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(false).parameter_overrides({
        {"server.port", server_port},
        {"fault_manager.namespace", "test" + std::to_string(test_id)},
    });

    node_ = std::make_shared<GatewayNode>(options);

    // These tests inject a known entity-cache state directly (apps / node_to_app
    // via get_thread_safe_cache()) and assert on the x-medkit context the SSE
    // handler snapshots at fault-arrival time. The gateway's own discovery
    // refresh, driven by the rclcpp graph-event timer, reconciles that cache
    // back to the live ROS graph and would wipe the injected entities. Under a
    // sanitizer's wider timing window a graph-event refresh can land between the
    // injection and the fault-event delivery, dropping x-medkit. Stop the
    // refresh drivers so the injected cache is the single source of truth.
    node_->stop_discovery_refresh_for_testing();

    CorsConfig cors_config;
    AuthConfig auth_config;
    TlsConfig tls_config;

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_config, auth_config, tls_config, nullptr);
    tracker_ = std::make_shared<SSEClientTracker>(4);
    handler_ = std::make_unique<SSEFaultHandler>(*ctx_, tracker_);

    publisher_node_ = std::make_shared<rclcpp::Node>("test_sse_fault_handler_publisher_" + std::to_string(test_id));
    fault_events_topic_ = ros2_medkit_gateway::build_fault_manager_events_topic(node_.get());
    publisher_ = publisher_node_->create_publisher<FaultEvent>(fault_events_topic_, rclcpp::QoS(100).reliable());

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);

    wait_for_subscribers();
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    executor_.reset();
    publisher_.reset();
    publisher_node_.reset();
    handler_.reset();
    ctx_.reset();
    node_.reset();
  }

  void enqueue_event(const FaultEvent & event) {
    // Wait until the handler has actually consumed this event (its
    // on_fault_event ran and snapshotted the owning entity) before returning,
    // so a test that mutates the entity cache afterwards cannot race the
    // snapshot. A fixed spin budget was too short under sanitizer slowdown.
    const uint64_t before = handler_->events_received();
    publisher_->publish(event);

    for (int i = 0; i < 500; ++i) {  // up to ~5s, generous for sanitizer runs
      executor_->spin_some();
      if (handler_->events_received() > before) {
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
    FAIL() << "Timed out waiting for the SSE handler to receive the published fault event";
  }

  void wait_for_subscribers() {
    for (int i = 0; i < 50; ++i) {
      executor_->spin_some();
      if (publisher_->get_subscription_count() >= 1u) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    FAIL() << "Timed out waiting for SSE fault handler subscription on " << fault_events_topic_;
  }

  std::shared_ptr<GatewayNode> node_;
  static inline int test_counter_ = 0;
  std::unique_ptr<HandlerContext> ctx_;
  std::shared_ptr<SSEClientTracker> tracker_;
  std::unique_ptr<SSEFaultHandler> handler_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<FaultEvent>::SharedPtr publisher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::string fault_events_topic_;
};

TEST_F(SSEFaultHandlerTest, HandleStreamSetsSseHeadersAndChunkedProvider) {
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;

  handler_->handle_stream(req, res);

  EXPECT_EQ(res.get_header_value("Content-Type"), "text/event-stream");
  EXPECT_EQ(res.get_header_value("Cache-Control"), "no-cache");
  EXPECT_EQ(res.get_header_value("Connection"), "keep-alive");
  EXPECT_EQ(res.get_header_value("X-Accel-Buffering"), "no");
  EXPECT_TRUE(res.is_chunked_content_provider_);
  EXPECT_EQ(handler_->connected_clients(), 1u);

  release_stream(res);
  EXPECT_EQ(handler_->connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, HandleStreamRejectsWhenClientLimitIsReached) {
  auto limited_tracker = std::make_shared<SSEClientTracker>(1);
  SSEFaultHandler limited_handler(*ctx_, limited_tracker);

  auto req_one = make_stream_request("127.0.0.1");
  auto req_two = make_stream_request("127.0.0.2");
  httplib::Response res_one;
  httplib::Response res_two;

  limited_handler.handle_stream(req_one, res_one);
  limited_handler.handle_stream(req_two, res_two);

  EXPECT_EQ(limited_handler.connected_clients(), 1u);
  EXPECT_EQ(res_two.status, 503);
  auto body = json::parse(res_two.body);
  EXPECT_EQ(body["error_code"], "service-unavailable");

  release_stream(res_one);
  EXPECT_EQ(limited_handler.connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, StreamReplaysBufferedEventsUsingSseFormat) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "TEMP_HIGH", 123, 456000000));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 1\n"), std::string::npos);
  EXPECT_NE(output.find("event: fault_confirmed\n"), std::string::npos);
  EXPECT_NE(output.find("data: "), std::string::npos);

  auto payload = parse_sse_payload(output);
  EXPECT_EQ(payload["event_type"], "fault_confirmed");
  EXPECT_EQ(payload["fault"]["fault_code"], "TEMP_HIGH");
  EXPECT_EQ(payload["fault"]["severity_label"], "ERROR");
  EXPECT_DOUBLE_EQ(payload["timestamp"].get<double>(), 123.456);
  EXPECT_FALSE(payload.contains("auto_cleared_codes"));  // omitted when empty

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamCarriesAutoClearedCodes) {
  // A correlation cascade lists its auto-cleared symptoms only on the root
  // cause's event; the stream payload must carry them or SSE consumers never
  // learn about the cascade.
  auto event = make_fault_event(FaultEvent::EVENT_CLEARED, "ROOT_CAUSE", 80);
  event.auto_cleared_codes = {"SYMPTOM_A", "SYMPTOM_B"};
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("auto_cleared_codes")) << payload.dump();
  EXPECT_EQ(payload["auto_cleared_codes"], (json::array({"SYMPTOM_A", "SYMPTOM_B"})));

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, LastEventIdReplaysOnlyMissedBufferedEvents) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_ONE", 100));
  enqueue_event(make_fault_event(FaultEvent::EVENT_UPDATED, "FAULT_TWO", 101));

  auto req = make_stream_request("127.0.0.1", "1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  EXPECT_NE(output.find("id: 2\n"), std::string::npos);
  EXPECT_EQ(payload["event_type"], "fault_updated");
  EXPECT_EQ(payload["fault"]["fault_code"], "FAULT_TWO");
  EXPECT_EQ(payload["timestamp"], 101.0);

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, InvalidLastEventIdFallsBackToFullReplay) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_X", 200));

  auto req = make_stream_request("127.0.0.1", "not-a-number");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 1\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_X");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, BufferedEventsAreReplayedToMultipleClients) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CLEARED, "FAULT_MULTI", 55));

  auto req_one = make_stream_request("127.0.0.1");
  auto req_two = make_stream_request("127.0.0.2");
  httplib::Response res_one;
  httplib::Response res_two;

  handler_->handle_stream(req_one, res_one);
  handler_->handle_stream(req_two, res_two);

  auto output_one = read_stream_once(res_one, 1);
  auto output_two = read_stream_once(res_two, 1);

  EXPECT_NE(output_one.find("id: 1\n"), std::string::npos);
  EXPECT_NE(output_two.find("id: 1\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output_one)["fault"]["fault_code"], "FAULT_MULTI");
  EXPECT_EQ(parse_sse_payload(output_two)["fault"]["fault_code"], "FAULT_MULTI");

  release_stream(res_one);
  release_stream(res_two);
  EXPECT_EQ(handler_->connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, BufferEvictsOldestEventWhenFull) {
  for (int i = 1; i <= 101; ++i) {
    enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_" + std::to_string(i), i));
  }

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 2\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_2");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, BufferRotationWithNoClientsIsNotADrop) {
  // Observed on a live diagnostic box: 19,000+ "events dropped ... slow or
  // disconnected clients" with zero SSE clients ever connected. Rotating a
  // replay buffer nobody is reading loses nothing and must not be reported as
  // backpressure.
  for (int i = 1; i <= 150; ++i) {
    enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_" + std::to_string(i), i));
  }

  EXPECT_EQ(handler_->connected_clients(), 0u);
  EXPECT_EQ(handler_->dropped_events(), 0u);
}

TEST_F(SSEFaultHandlerTest, LaggingClientKeepsRareFaultWhenFlapFillsBuffer) {
  // A PLC alarm flapping on a few-second cycle used to push every other fault
  // out of the 100-entry buffer, so a client behind by more than the buffer
  // never learned the standing fault was active. Superseded flap transitions
  // are evicted before the unique standing fault - but a transition is
  // history a lagging client can never recover, so every one of those
  // evictions is honest, counted loss, never "coalesced".
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);  // registers a cursor; never drained below

  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "PLC_COMMS_LOST", 1));
  for (int i = 0; i < 150; ++i) {
    const auto type = (i % 2 == 0) ? FaultEvent::EVENT_CLEARED : FaultEvent::EVENT_CONFIRMED;
    enqueue_event(make_fault_event(type, "TC3_ALARM_FLAP", 100 + i));
  }

  EXPECT_EQ(handler_->dropped_events(), 51u);   // 151 events, 100 kept, all losses counted
  EXPECT_EQ(handler_->coalesced_events(), 0u);  // transitions are never "no-loss" evictions

  auto output = read_stream_once(res, 1);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "PLC_COMMS_LOST");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, CoalescedReplayKeepsTransitionsAndEndsOnCurrentState) {
  // Convergence contract for genuine coalescing: only fault_updated entries
  // may be silently superseded. An update storm must not push out the fault's
  // confirmation, and the last replayed frame must be the latest state.
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);  // cursor open while the buffer is trimmed

  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FLAPPER", 200));
  for (int i = 1; i <= 149; ++i) {
    enqueue_event(make_fault_event(FaultEvent::EVENT_UPDATED, "FLAPPER", 200 + i));
  }
  enqueue_event(make_fault_event(FaultEvent::EVENT_CLEARED, "FLAPPER", 999));

  EXPECT_GT(handler_->coalesced_events(), 0u);
  EXPECT_EQ(handler_->dropped_events(), 0u);

  // The buffer holds exactly its cap after the overflow; disconnect on the last
  // replayed frame so the loop never reaches the keepalive wait.
  auto output = read_stream_once(res, 100);

  const auto first_frame = output.find("event: ");
  ASSERT_NE(first_frame, std::string::npos);
  auto first = parse_sse_payload(output.substr(first_frame));
  EXPECT_EQ(first["event_type"], "fault_confirmed");  // the transition survived the storm

  const auto last_frame = output.rfind("event: ");
  ASSERT_NE(last_frame, std::string::npos);
  auto payload = parse_sse_payload(output.substr(last_frame));
  EXPECT_EQ(payload["fault"]["fault_code"], "FLAPPER");
  EXPECT_EQ(payload["event_type"], "fault_cleared");
  EXPECT_DOUBLE_EQ(payload["timestamp"].get<double>(), 999.0);

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, OwedDistinctEventsLostUnderPressureAreCountedAndLogged) {
  // The counter this PR is about: a live client is owed events, the buffer
  // overflows with distinct fault codes (nothing to coalesce), so events are
  // genuinely lost - counted, attributed to the one slow client, and WARNed
  // starting from the very first loss.
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);  // cursor registered; never drained

  testing::internal::CaptureStderr();
  for (int i = 1; i <= 150; ++i) {
    enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "DISTINCT_" + std::to_string(i), i));
  }
  auto logs = testing::internal::GetCapturedStderr();

  EXPECT_EQ(handler_->dropped_events(), 50u);
  EXPECT_EQ(handler_->coalesced_events(), 0u);
  EXPECT_NE(logs.find("SSE fault event lost: 1 event(s) dropped total for 1 slow client(s)"), std::string::npos)
      << logs;

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, HugeLastEventIdIsClampedToNewestIssuedId) {
  // Last-Event-ID above the newest issued id (here: UINT64_MAX) used to make
  // the stream permanently blind and alias the delivery watermark. It must be
  // treated as "caught up": new events still flow.
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_OLD", 100));

  auto req = make_stream_request("127.0.0.1", "18446744073709551615");
  httplib::Response res;
  handler_->handle_stream(req, res);

  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_NEW", 101));

  auto output = read_stream_once(res, 1);
  EXPECT_NE(output.find("id: 2\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_NEW");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, NegativeLastEventIdFallsBackToFullReplay) {
  // std::stoull accepts "-1" and wraps it to UINT64_MAX; the parser must
  // reject it as malformed so the client gets the full replay instead of a
  // blind stream.
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_NEG", 300));

  auto req = make_stream_request("127.0.0.1", "-1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  EXPECT_NE(output.find("id: 1\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_NEG");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamSanitizesNewlinesInEventType) {
  enqueue_event(make_fault_event("fault_confirmed\nretry: 0\r\nevent: injected", "FAULT_SANITIZED", 300));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  EXPECT_NE(output.find("event: fault_confirmedretry: 0event: injected\n"), std::string::npos);
  EXPECT_EQ(output.find("\nretry: 0\r\n"), std::string::npos);
  EXPECT_EQ(output.find("\nevent: injected\n"), std::string::npos);
  EXPECT_EQ(payload["event_type"], "fault_confirmedretry: 0event: injected");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamSendsKeepaliveCommentAfterTimeout) {
  auto fast_tracker = std::make_shared<SSEClientTracker>(1);
  SSEFaultHandler fast_handler(*ctx_, fast_tracker, 10ms);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  fast_handler.handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_EQ(output, ":keepalive\n\n");

  release_stream(res);
  EXPECT_EQ(fast_handler.connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, NonPositiveKeepaliveOverrideLogsWarning) {
  testing::internal::CaptureStderr();
  {
    auto warn_tracker = std::make_shared<SSEClientTracker>(1);
    SSEFaultHandler warn_handler(*ctx_, warn_tracker, 0ms);
  }
  auto logs = testing::internal::GetCapturedStderr();

  EXPECT_NE(logs.find("Non-positive SSE keepalive override"), std::string::npos);
}

TEST_F(SSEFaultHandlerTest, StreamOmitsTheEntityHintWhenNoMatchingApp) {
  // Empty cache: reporting source ("/apps/temp_sensor") has no manifest mapping
  // and no App with id "temp_sensor" exists in the runtime cache, so the entity
  // hint is not emitted and a consumer falls back to discovery. The ``x-medkit``
  // object itself IS emitted: it also carries the planned-stop flag, which is a
  // property of the fault rather than of any entity.
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "NO_OWNER", 10));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  // Nothing to put in it: the reporting source resolves to no entity, and this
  // fixture has no fault manager, so the gateway has never read a window set and
  // must not claim the fault was unexpected. Both halves absent means the object
  // itself is absent.
  EXPECT_FALSE(payload.contains("x-medkit"));

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamEmitsXMedkitForRuntimeApp) {
  // Runtime_only fallback: synthetic App with id matching the reporting
  // source's last segment exists in the cache, so the ``x-medkit`` payload
  // extension points consumers straight at
  // /apps/temp_sensor/bulk-data/rosbags/<fault_code>.
  App app;
  app.id = "temp_sensor";
  app.name = "temp_sensor";
  app.source = "heuristic";
  app.bound_fqn = "/apps/temp_sensor";
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_apps({app});

  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "WITH_OWNER", 20));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_type"], "apps");
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "temp_sensor");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamEmitsXMedkitForManifestApp) {
  // Manifest/hybrid mode: linking populates node_to_app with the ROS FQN ->
  // manifest app id mapping. The fault's reporting source uses the ROS FQN
  // form (with leading slash), but the manifest app id can be arbitrary (here
  // "diagnostic-bridge" with a hyphen, to model the FQN/id naming mismatch
  // called out in #380).
  App app;
  app.id = "diagnostic-bridge";
  app.name = "diagnostic_bridge";
  app.source = "manifest";
  app.bound_fqn = "/bridge/diagnostic_bridge";
  std::unordered_map<std::string, std::string> node_to_app{{"/bridge/diagnostic_bridge", "diagnostic-bridge"}};
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_all({}, {}, {app}, {}, node_to_app);

  auto event = make_fault_event(FaultEvent::EVENT_CONFIRMED, "BRIDGE_FAULT", 30);
  event.fault.reporting_sources = {"/bridge/diagnostic_bridge"};
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_type"], "apps");
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "diagnostic-bridge");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamPrefersManifestMatchOverRuntimeFallback) {
  // Manifest match must win over the runtime heuristic when both could apply
  // to the same FQN. Seed both: (1) a manifest node_to_app entry mapping
  // "/bridge/diagnostic_bridge" -> "manifest-bridge", and (2) a runtime-style
  // App whose id matches the FQN's last segment ("diagnostic_bridge"). The
  // resolver tries manifest first - if anyone ever flips the ordering, the
  // x-medkit emits the runtime id and this test catches it.
  App manifest_app;
  manifest_app.id = "manifest-bridge";
  manifest_app.name = "diagnostic_bridge";
  manifest_app.source = "manifest";
  manifest_app.bound_fqn = "/bridge/diagnostic_bridge";

  App runtime_app;
  runtime_app.id = "diagnostic_bridge";
  runtime_app.name = "diagnostic_bridge";
  runtime_app.source = "heuristic";
  runtime_app.bound_fqn = "/bridge/diagnostic_bridge";

  std::unordered_map<std::string, std::string> node_to_app{{"/bridge/diagnostic_bridge", "manifest-bridge"}};
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_all({}, {}, {manifest_app, runtime_app}, {}, node_to_app);

  auto event = make_fault_event(FaultEvent::EVENT_CONFIRMED, "PRECEDENCE", 35);
  event.fault.reporting_sources = {"/bridge/diagnostic_bridge"};
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "manifest-bridge");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamSnapshotsEntityContextAtEnqueue) {
  // The core design claim of #380: entity resolution is pinned at enqueue
  // time so a discovery refresh between enqueue and stream replay cannot
  // flip the entity reported to consumers. Enqueue with a matching App in
  // the cache, then mutate the cache to remove it, then replay - the
  // x-medkit object must still reflect the snapshot.
  App app;
  app.id = "temp_sensor";
  app.name = "temp_sensor";
  app.source = "heuristic";
  app.bound_fqn = "/apps/temp_sensor";
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_apps({app});

  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "SNAPSHOTTED", 50));

  // Discovery refresh: remove the App. Without snapshot-at-enqueue, a
  // replay would now see an empty cache and omit x-medkit.
  cache.update_apps({});

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_type"], "apps");
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "temp_sensor");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamResolvesManifestFqnWithoutLeadingSlash) {
  // gateway_node's node_resolver tries both FQN forms; the linking layer may
  // store keys without the leading '/' (e.g. when the manifest declares
  // namespace-only FQNs). The handler must try the slash-stripped form so
  // /bridge/diagnostic_bridge resolves against a node_to_app key of
  // "bridge/diagnostic_bridge".
  App app;
  app.id = "diagnostic-bridge";
  app.name = "diagnostic_bridge";
  app.source = "manifest";
  app.bound_fqn = "/bridge/diagnostic_bridge";
  std::unordered_map<std::string, std::string> node_to_app{{"bridge/diagnostic_bridge", "diagnostic-bridge"}};
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_all({}, {}, {app}, {}, node_to_app);

  auto event = make_fault_event(FaultEvent::EVENT_CONFIRMED, "SLASH_STRIP", 60);
  event.fault.reporting_sources = {"/bridge/diagnostic_bridge"};
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "diagnostic-bridge");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamResolvesRuntimeCollisionRenamedApp) {
  // Runtime discovery renames collision-prone nodes to <ns_prefix>_<name>
  // (see ros2_runtime_introspection.cpp). The fallback must also try this
  // form so faults from those apps still get a usable x-medkit context.
  App app;
  app.id = "bridge_diagnostic_bridge";
  app.name = "diagnostic_bridge";
  app.source = "heuristic";
  app.bound_fqn = "/bridge/diagnostic_bridge";
  auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
  cache.update_apps({app});

  auto event = make_fault_event(FaultEvent::EVENT_CONFIRMED, "COLLISION", 70);
  event.fault.reporting_sources = {"/bridge/diagnostic_bridge"};
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  ASSERT_TRUE(payload.contains("x-medkit")) << payload.dump();
  EXPECT_EQ(payload["x-medkit"]["entity_id"], "bridge_diagnostic_bridge");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamOmitsTheEntityHintWhenReportingSourcesEmpty) {
  auto event = make_fault_event(FaultEvent::EVENT_CONFIRMED, "ORPHAN", 40);
  event.fault.reporting_sources.clear();
  enqueue_event(event);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  // "Unknown" is not "false". No fault manager has ever answered here, so the
  // frame says nothing about whether the fault was expected rather than saying
  // it was not - a consumer must not read an outage as "nothing is planned".
  if (payload.contains("x-medkit")) {
    EXPECT_FALSE(payload["x-medkit"].contains("expected"));
    EXPECT_FALSE(payload["x-medkit"].contains("planned_stop_id"));
  }

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, DisconnectReleasesTrackedClientSlot) {
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;

  handler_->handle_stream(req, res);
  EXPECT_EQ(handler_->connected_clients(), 1u);

  release_stream(res, false);

  EXPECT_EQ(handler_->connected_clients(), 0u);
}
