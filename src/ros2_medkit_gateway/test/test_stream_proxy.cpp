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

#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/stream_proxy.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Construction and lifecycle tests
// =============================================================================

TEST(StreamProxy, sse_proxy_can_be_constructed) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  // Should not throw or crash
}

TEST(StreamProxy, initially_not_connected) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  EXPECT_FALSE(proxy.is_connected());
}

TEST(StreamProxy, close_is_idempotent) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  // Calling close() multiple times without open() should not crash
  proxy.close();
  proxy.close();
  proxy.close();
  EXPECT_FALSE(proxy.is_connected());
}

// =============================================================================
// SSE parsing tests (pure function, no networking required)
// =============================================================================

// @verifies REQ_INTEROP_090
TEST(StreamProxy, parse_sse_single_event) {
  std::string raw = "data: {\"temperature\": 42}\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_a");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "{\"temperature\": 42}");
  EXPECT_EQ(events[0].event_type, "message");  // Default SSE type
  EXPECT_EQ(events[0].peer_name, "peer_a");
  EXPECT_TRUE(events[0].id.empty());
}

// @verifies REQ_INTEROP_090
TEST(StreamProxy, parse_sse_multiple_events) {
  std::string raw =
      "data: {\"temp\": 42}\n"
      "\n"
      "data: {\"temp\": 43}\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_b");

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].data, "{\"temp\": 42}");
  EXPECT_EQ(events[0].peer_name, "peer_b");
  EXPECT_EQ(events[1].data, "{\"temp\": 43}");
  EXPECT_EQ(events[1].peer_name, "peer_b");
}

TEST(StreamProxy, parse_sse_multiline_data) {
  std::string raw =
      "data: line one\n"
      "data: line two\n"
      "data: line three\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_c");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "line one\nline two\nline three");
  EXPECT_EQ(events[0].event_type, "message");
}

// @verifies REQ_INTEROP_090
TEST(StreamProxy, parse_sse_with_event_type) {
  std::string raw =
      "event: data_update\n"
      "data: {\"sensor\": \"imu\"}\n"
      "id: 123\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_d");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "data_update");
  EXPECT_EQ(events[0].data, "{\"sensor\": \"imu\"}");
  EXPECT_EQ(events[0].id, "123");
  EXPECT_EQ(events[0].peer_name, "peer_d");
}

TEST(StreamProxy, parse_sse_empty_input) {
  auto events = SSEStreamProxy::parse_sse_data("", "peer_e");
  EXPECT_TRUE(events.empty());
}

// =============================================================================
// Additional SSE parsing edge cases
// =============================================================================

TEST(StreamProxy, parse_sse_with_comments) {
  std::string raw =
      ": this is a comment\n"
      "data: hello\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_f");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "hello");
}

TEST(StreamProxy, parse_sse_event_without_trailing_blank_line) {
  // An event at end of stream without a trailing blank line should still parse
  std::string raw = "event: fault_added\ndata: {\"fault_id\": \"F001\"}\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_g");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "fault_added");
  EXPECT_EQ(events[0].data, "{\"fault_id\": \"F001\"}");
}

TEST(StreamProxy, parse_sse_mixed_events_with_and_without_types) {
  std::string raw =
      "event: fault_added\n"
      "data: {\"fault_id\": \"F001\"}\n"
      "\n"
      "data: {\"heartbeat\": true}\n"
      "\n"
      "event: fault_cleared\n"
      "data: {\"fault_id\": \"F001\"}\n"
      "id: 42\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_h");

  ASSERT_EQ(events.size(), 3u);

  EXPECT_EQ(events[0].event_type, "fault_added");
  EXPECT_EQ(events[0].data, "{\"fault_id\": \"F001\"}");
  EXPECT_TRUE(events[0].id.empty());

  EXPECT_EQ(events[1].event_type, "message");
  EXPECT_EQ(events[1].data, "{\"heartbeat\": true}");

  EXPECT_EQ(events[2].event_type, "fault_cleared");
  EXPECT_EQ(events[2].data, "{\"fault_id\": \"F001\"}");
  EXPECT_EQ(events[2].id, "42");
}

TEST(StreamProxy, parse_sse_with_crlf_line_endings) {
  std::string raw = "event: update\r\ndata: payload\r\n\r\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_i");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "update");
  EXPECT_EQ(events[0].data, "payload");
}

TEST(StreamProxy, parse_sse_data_with_colons_in_value) {
  // The SSE spec says only the first colon splits field from value
  std::string raw = "data: http://example.com:8080/path\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_j");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "http://example.com:8080/path");
}

TEST(StreamProxy, parse_sse_blank_lines_only) {
  // Only blank lines with no data should produce no events
  std::string raw = "\n\n\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_k");
  EXPECT_TRUE(events.empty());
}

TEST(StreamProxy, on_event_can_set_callback) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_l");
  bool called = false;
  proxy.on_event([&called](const StreamEvent &) {
    called = true;
  });
  // Callback is set but not invoked without open()
  EXPECT_FALSE(called);
}

// =============================================================================
// Mock server integration tests (local httplib::Server)
// =============================================================================

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace {

/// Helper to wait for httplib::Server to be ready for connections.
///
/// Running out of budget here is a failure and has to be reported as one. A
/// silent return leaves the case to go on against a server that is not
/// listening yet, and whatever it then asserts is about the wait, not about the
/// proxy.
void wait_for_server(httplib::Server & svr, int timeout_ms = 5000) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!svr.is_running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(svr.is_running()) << "the mock server was not listening after " << timeout_ms
                                << " ms, so nothing below is a test of the proxy";
}

}  // namespace

// @verifies REQ_INTEROP_090
TEST(SSEStreamProxyIntegration, receives_events_from_mock_server) {
  httplib::Server svr;

  std::atomic<bool> stop_stream{false};

  // Chunked content provider simulating a real SSE stream:
  // sends events with pauses between them, then waits until stopped.
  svr.Get("/events", [&stop_stream](const httplib::Request &, httplib::Response & res) {
    res.set_chunked_content_provider("text/event-stream", [&stop_stream](size_t offset, httplib::DataSink & sink) {
      if (offset == 0) {
        std::string event1 =
            "event: test\n"
            "data: {\"value\":42}\n"
            "\n";
        sink.write(event1.data(), event1.size());
        return true;
      }
      // Second call - send the second event
      std::string data_so_far;
      if (offset > 0 && offset < 200) {
        std::string event2 =
            "event: update\n"
            "data: {\"value\":43}\n"
            "\n";
        sink.write(event2.data(), event2.size());
        return true;
      }
      // Keep connection open until test signals stop
      if (stop_stream.load()) {
        sink.done();
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      return true;
    });
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread server_thread([&svr]() {
    svr.listen_after_bind();
  });
  wait_for_server(svr);

  SSEStreamProxy proxy("http://127.0.0.1:" + std::to_string(port), "/events", "test_peer");

  std::vector<StreamEvent> received;
  std::mutex mtx;
  std::condition_variable cv;

  proxy.on_event([&](const StreamEvent & event) {
    std::lock_guard<std::mutex> lock(mtx);
    received.push_back(event);
    cv.notify_one();
  });

  proxy.open();

  // Wait for at least two events (with timeout)
  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait_for(lock, std::chrono::seconds(5), [&]() {
      return received.size() >= 2u;
    });
  }

  stop_stream.store(true);
  proxy.close();
  svr.stop();
  server_thread.join();

  ASSERT_GE(received.size(), 2u);
  EXPECT_EQ(received[0].event_type, "test");
  EXPECT_EQ(received[0].data, "{\"value\":42}");
  EXPECT_EQ(received[0].peer_name, "test_peer");
  EXPECT_EQ(received[1].event_type, "update");
  EXPECT_EQ(received[1].data, "{\"value\":43}");
  EXPECT_EQ(received[1].peer_name, "test_peer");
}

TEST(SSEStreamProxyIntegration, close_terminates_reader_thread) {
  httplib::Server svr;

  // Chunked provider that streams indefinitely until client disconnects
  svr.Get("/events", [](const httplib::Request &, httplib::Response & res) {
    res.set_chunked_content_provider("text/event-stream", [](size_t /*offset*/, httplib::DataSink & sink) {
      std::string event = "data: heartbeat\n\n";
      sink.write(event.data(), event.size());
      // Sleep to simulate a long-lived stream
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return true;
    });
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread server_thread([&svr]() {
    svr.listen_after_bind();
  });
  wait_for_server(svr);

  SSEStreamProxy proxy("http://127.0.0.1:" + std::to_string(port), "/events", "long_stream_peer");

  std::atomic<int> event_count{0};
  proxy.on_event([&](const StreamEvent &) {
    event_count.fetch_add(1);
  });

  proxy.open();

  // Wait for at least one event to confirm the connection is live
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (event_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Even if we got zero events (e.g. timing), close() must not hang
  auto close_start = std::chrono::steady_clock::now();
  proxy.close();
  auto close_duration = std::chrono::steady_clock::now() - close_start;

  // Reader thread should have joined within a reasonable time
  EXPECT_LT(close_duration, std::chrono::seconds(5));
  EXPECT_FALSE(proxy.is_connected());

  svr.stop();
  server_thread.join();

  // Verify that at least one event was received (confirms streaming worked)
  EXPECT_GT(event_count.load(), 0);
}

TEST(SSEStreamProxyIntegration, buffer_overflow_disconnects) {
  httplib::Server svr;

  // Server sends >1MB without any event boundary (\n\n)
  svr.Get("/events", [](const httplib::Request &, httplib::Response & res) {
    res.set_chunked_content_provider("text/event-stream", [](size_t /*offset*/, httplib::DataSink & sink) {
      // Send 8KB chunks of data without a double-newline boundary
      std::string chunk(8192, 'x');
      sink.write(chunk.data(), chunk.size());
      return true;
    });
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread server_thread([&svr]() {
    svr.listen_after_bind();
  });
  wait_for_server(svr);

  SSEStreamProxy proxy("http://127.0.0.1:" + std::to_string(port), "/events", "overflow_peer");

  std::atomic<int> event_count{0};
  proxy.on_event([&](const StreamEvent &) {
    event_count.fetch_add(1);
  });

  proxy.open();

  // Wait for the proxy to disconnect due to buffer overflow
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (proxy.is_connected() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  proxy.close();
  svr.stop();
  server_thread.join();

  // No valid events should have been delivered since there were no boundaries
  EXPECT_EQ(event_count.load(), 0);
}
