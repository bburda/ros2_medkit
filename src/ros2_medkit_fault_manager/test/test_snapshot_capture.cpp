// Copyright 2026 mfaferek93
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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::SnapshotCapture;
using ros2_medkit_fault_manager::SnapshotConfig;

class SnapshotCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_snapshot_capture_node");
    storage_ = std::make_unique<InMemoryFaultStorage>();
  }

  void TearDown() override {
    node_.reset();
    storage_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<InMemoryFaultStorage> storage_;
};

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorRequiresValidNode) {
  SnapshotConfig config;
  EXPECT_THROW(SnapshotCapture(nullptr, storage_.get(), config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorRequiresValidStorage) {
  SnapshotConfig config;
  EXPECT_THROW(SnapshotCapture(node_.get(), nullptr, config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorSucceedsWithValidParams) {
  SnapshotConfig config;
  config.enabled = true;
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, DisabledCaptureSkipsProcessing) {
  SnapshotConfig config;
  config.enabled = false;
  config.default_topics = {"/test_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("TEST_FAULT");

  // No snapshots should be stored when disabled
  auto snapshots = storage_->get_snapshots("TEST_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, IsEnabledReturnsConfigState) {
  SnapshotConfig config;
  config.enabled = true;
  SnapshotCapture capture_enabled(node_.get(), storage_.get(), config);
  EXPECT_TRUE(capture_enabled.is_enabled());

  config.enabled = false;
  SnapshotCapture capture_disabled(node_.get(), storage_.get(), config);
  EXPECT_FALSE(capture_disabled.is_enabled());
}

// Topic Resolution Priority Tests

class TopicResolutionTest : public SnapshotCaptureTest {
 protected:
  // Helper to access resolved topics through capture behavior
  // Since resolve_topics is private, we test through the capture interface
};

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, FaultSpecificHasHighestPriority) {
  SnapshotConfig config;
  config.enabled = true;
  config.fault_specific["MOTOR_OVERHEAT"] = {"/motor/specific_topic"};
  config.patterns["MOTOR_.*"] = {"/motor/pattern_topic"};
  config.default_topics = {"/default_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Verify config is stored correctly
  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.fault_specific.at("MOTOR_OVERHEAT").size(), 1u);
  EXPECT_EQ(stored_config.fault_specific.at("MOTOR_OVERHEAT")[0], "/motor/specific_topic");
}

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, PatternMatchUsedWhenNoFaultSpecific) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["SENSOR_.*"] = {"/sensor/pattern_topic"};
  config.default_topics = {"/default_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Pattern should be compiled
  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.patterns.size(), 1u);
}

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, DefaultTopicsUsedAsFallback) {
  SnapshotConfig config;
  config.enabled = true;
  config.default_topics = {"/default1", "/default2"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.default_topics.size(), 2u);
}

// Regex Pattern Tests

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, InvalidRegexPatternIsSkipped) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["[invalid(regex"] = {"/topic1"};  // Invalid regex
  config.patterns["VALID_.*"] = {"/topic2"};        // Valid regex

  // Should not throw, invalid pattern is logged and skipped
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, MultipleValidPatternsCompiled) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["MOTOR_.*"] = {"/motor/topic"};
  config.patterns["SENSOR_[0-9]+"] = {"/sensor/topic"};
  config.patterns["^ERROR_"] = {"/error/topic"};

  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// Configuration Tests

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConfigAccessorReturnsCorrectValues) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.timeout_sec = 2.5;
  config.max_message_size = 32768;

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  const auto & stored = capture.config();
  EXPECT_TRUE(stored.enabled);
  EXPECT_TRUE(stored.background_capture);
  EXPECT_DOUBLE_EQ(stored.timeout_sec, 2.5);
  EXPECT_EQ(stored.max_message_size, 32768u);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, EmptyConfigurationHandledGracefully) {
  SnapshotConfig config;
  config.enabled = true;
  // No topics configured at all

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("ANY_FAULT");

  // Should not crash, just log that no topics configured
  auto snapshots = storage_->get_snapshots("ANY_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// Background capture initialization test
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, BackgroundCaptureInitializesSubscriptions) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.default_topics = {"/nonexistent_topic"};  // Topic doesn't exist

  // Should not throw even if topics don't exist
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// On-demand capture with non-existent topic
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, OnDemandCaptureHandlesNonExistentTopic) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 0.1;  // Short timeout for test
  config.default_topics = {"/nonexistent_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("TEST_FAULT");

  // Should timeout gracefully, no snapshot stored
  auto snapshots = storage_->get_snapshots("TEST_FAULT");
  EXPECT_TRUE(snapshots.empty());

  // A configured capture that sampled nothing still records an empty {} frame.
  auto frame = storage_->get_freeze_frame("TEST_FAULT");
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->data, "{}");
}

// Freeze-frame end-to-end tests

// Stops and joins a publisher thread on scope exit, so an assertion failure
// mid-test never destroys a joinable std::thread (which would std::terminate).
struct ScopedPublisherThread {
  std::atomic<bool> stop{false};
  std::thread thread;

  explicit ScopedPublisherThread(std::function<void(std::atomic<bool> &)> body)
    : thread([this, body = std::move(body)]() {
      body(stop);
    }) {
  }

  ~ScopedPublisherThread() {
    stop.store(true);
    if (thread.joinable()) {
      thread.join();
    }
  }
};

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, CaptureWritesFreezeFrameFromConfiguredTopic) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/pressure", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["PLC_PRESSURE_HIGH"] = {"/plc/pressure"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Publish continuously so the on-demand one-shot subscription catches a value.
  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 42.5;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  // Wait for the publisher to be visible to the capture node before capturing.
  auto start = std::chrono::steady_clock::now();
  while (node_->count_publishers("/plc/pressure") == 0 &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(node_->count_publishers("/plc/pressure"), 0u);

  capture.capture("PLC_PRESSURE_HIGH");

  auto frame = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  ASSERT_TRUE(parsed.contains("/plc/pressure"));
  EXPECT_DOUBLE_EQ(parsed["/plc/pressure"]["data"].get<double>(), 42.5);
}

// Regression for the empty-recapture overwrite: a re-confirm while the source
// publishers are down must not clobber a previously retained non-empty frame.
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, EmptyRecaptureKeepsRetainedFreezeFrame) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/flow", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["PLC_FLOW_LOW"] = {"/plc/flow"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  {
    ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
      while (!stop.load()) {
        std_msgs::msg::Float64 msg;
        msg.data = 7.25;
        pub->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });

    auto start = std::chrono::steady_clock::now();
    while (node_->count_publishers("/plc/flow") == 0 &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node_->count_publishers("/plc/flow"), 0u);

    capture.capture("PLC_FLOW_LOW");
  }

  auto frame = storage_->get_freeze_frame("PLC_FLOW_LOW");
  ASSERT_TRUE(frame.has_value());
  ASSERT_NE(frame->data, "{}");

  // Take the publisher down and wait until the graph reflects it, then re-capture.
  pub.reset();
  auto start = std::chrono::steady_clock::now();
  while (node_->count_publishers("/plc/flow") > 0 &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(node_->count_publishers("/plc/flow"), 0u);

  capture.capture("PLC_FLOW_LOW");

  auto retained = storage_->get_freeze_frame("PLC_FLOW_LOW");
  ASSERT_TRUE(retained.has_value());
  auto parsed = nlohmann::json::parse(retained->data);
  ASSERT_TRUE(parsed.contains("/plc/flow"));
  EXPECT_DOUBLE_EQ(parsed["/plc/flow"]["data"].get<double>(), 7.25);
}

// Exercises the background-capture cache path into the freeze-frame (cached JSON
// string parsed back into a structured value).
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, BackgroundCaptureCachesFreezeFrame) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/temperature", rclcpp::QoS(10));

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 91.5;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  // The publisher already exists on this node, so the topic type is resolvable
  // when the constructor sets up the background subscription.
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.fault_specific["PLC_TEMP_HIGH"] = {"/plc/temperature"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Spin so the background subscription caches a message, then capture from cache.
  // Until the cache fills, capture records an empty first-run {} frame; retry.
  bool got_frame = false;
  auto start = std::chrono::steady_clock::now();
  while (!got_frame && std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    rclcpp::spin_some(node_);
    capture.capture("PLC_TEMP_HIGH");
    auto frame = storage_->get_freeze_frame("PLC_TEMP_HIGH");
    got_frame = frame.has_value() && frame->data != "{}";
    if (!got_frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(got_frame);

  auto frame = storage_->get_freeze_frame("PLC_TEMP_HIGH");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  ASSERT_TRUE(parsed.contains("/plc/temperature"));
  EXPECT_DOUBLE_EQ(parsed["/plc/temperature"]["data"].get<double>(), 91.5);

  // The cache path also stores a per-topic snapshot.
  auto snapshots = storage_->get_snapshots("PLC_TEMP_HIGH");
  ASSERT_FALSE(snapshots.empty());
  EXPECT_EQ(snapshots.back().topic, "/plc/temperature");
  EXPECT_EQ(snapshots.back().message_type, "std_msgs/msg/Float64");
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, UnconfiguredFaultWritesNoFreezeFrame) {
  SnapshotConfig config;
  config.enabled = true;
  config.fault_specific["OTHER_FAULT"] = {"/plc/pressure"};
  // No default_topics: an unrelated fault code resolves to an empty capture set.
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  capture.capture("UNMAPPED_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("UNMAPPED_FAULT").has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, DisabledCaptureWritesNoFreezeFrame) {
  SnapshotConfig config;
  config.enabled = false;
  config.default_topics = {"/plc/pressure"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  capture.capture("ANY_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("ANY_FAULT").has_value());
}

// Entity-default (zero-config) capture tests. The reporting source is this
// test node's own FQN, so resolve_entity_topics finds the topics it publishes.

class EntityDefaultCaptureTest : public SnapshotCaptureTest {
 protected:
  /// Store a CONFIRMED fault whose reporting source is this test node.
  void store_fault_from_this_node(const std::string & fault_code) {
    ros2_medkit_fault_manager::DebounceConfig debounce;  // threshold -1: first FAILED confirms
    storage_->report_fault_event(fault_code, 0 /*EVENT_FAILED*/, ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                                 "test fault", node_->get_fully_qualified_name(), rclcpp::Clock().now(), debounce);
  }

  /// Wait until this node's publisher on @p topic is visible on the graph.
  void wait_for_publisher(const std::string & topic) {
    auto start = std::chrono::steady_clock::now();
    while (node_->count_publishers(topic) == 0 && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node_->count_publishers(topic), 0u);
  }
};

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, EntityDefaultCapturesSourceNodeTopics) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/own_metric", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  // No fault_specific / patterns / default_topics: only the entity-default
  // fallback can produce a capture set.
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  store_fault_from_this_node("ENTITY_FAULT");

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 13.5;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  wait_for_publisher("/entity/own_metric");

  capture.capture("ENTITY_FAULT");

  auto frame = storage_->get_freeze_frame("ENTITY_FAULT");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  ASSERT_TRUE(parsed.contains("/entity/own_metric"));
  EXPECT_DOUBLE_EQ(parsed["/entity/own_metric"]["data"].get<double>(), 13.5);
  // Per-node noise topics never enter the entity capture set.
  EXPECT_FALSE(parsed.contains("/rosout"));
  EXPECT_FALSE(parsed.contains("/parameter_events"));
}

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, EntityDefaultDisabledWritesNoFreezeFrame) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/opted_out", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.entity_default = false;
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  store_fault_from_this_node("OPTED_OUT_FAULT");
  wait_for_publisher("/entity/opted_out");

  capture.capture("OPTED_OUT_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("OPTED_OUT_FAULT").has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, ExplicitConfigWinsOverEntityDefault) {
  auto configured_pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/configured", rclcpp::QoS(10));
  auto own_pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/own_other", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["CONFIGURED_FAULT"] = {"/entity/configured"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  store_fault_from_this_node("CONFIGURED_FAULT");

  ScopedPublisherThread pub_thread([&configured_pub, &own_pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 1.0;
      configured_pub->publish(msg);
      own_pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  wait_for_publisher("/entity/configured");

  capture.capture("CONFIGURED_FAULT");

  auto frame = storage_->get_freeze_frame("CONFIGURED_FAULT");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  // Only the explicitly configured topic is captured, never the node's other topics.
  EXPECT_TRUE(parsed.contains("/entity/configured"));
  EXPECT_FALSE(parsed.contains("/entity/own_other"));
}

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, ExplicitEmptyConfigEntryOptsOutOfEntityDefault) {
  // A present-but-empty fault_specific or pattern entry is an explicit
  // "capture nothing for this code" and must not fall through to the
  // entity-default capture of the node's own topics.
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/own_optout", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["OPTED_OUT_SPECIFIC"] = {};
  config.patterns["^OPTED_OUT_PATTERN_.*"] = {};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  store_fault_from_this_node("OPTED_OUT_SPECIFIC");
  store_fault_from_this_node("OPTED_OUT_PATTERN_X");

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 7.0;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  wait_for_publisher("/entity/own_optout");

  capture.capture("OPTED_OUT_SPECIFIC");
  capture.capture("OPTED_OUT_PATTERN_X");

  EXPECT_FALSE(storage_->get_freeze_frame("OPTED_OUT_SPECIFIC").has_value());
  EXPECT_FALSE(storage_->get_freeze_frame("OPTED_OUT_PATTERN_X").has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, UnresolvableSourceWritesNoRow) {
  SnapshotConfig config;
  config.enabled = true;
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Plugin-style bare entity id: not a live node, resolves to no topics.
  ros2_medkit_fault_manager::DebounceConfig debounce;
  storage_->report_fault_event("PLC_FAULT", 0 /*EVENT_FAILED*/, ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                               "plc fault", "beckhoff_plc_app", rclcpp::Clock().now(), debounce);

  capture.capture("PLC_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("PLC_FAULT").has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(EntityDefaultCaptureTest, BarePluginIdNeverMatchesSameNamedNode) {
  // A bare plugin entity id that happens to equal a live root-namespace node's
  // name (this test node) must not be parsed as (name, ns="/") and capture
  // that unrelated node's topics.
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/entity/bare_id_metric", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  ros2_medkit_fault_manager::DebounceConfig debounce;
  storage_->report_fault_event("BARE_ID_FAULT", 0 /*EVENT_FAILED*/, ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                               "plugin fault", node_->get_name(), rclcpp::Clock().now(), debounce);

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 7.0;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  wait_for_publisher("/entity/bare_id_metric");

  capture.capture("BARE_ID_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("BARE_ID_FAULT").has_value());
}

namespace {

/// Block until the capture node can see a publisher on the topic; the on-demand
/// subscription samples nothing before discovery completes.
void await_publisher(const std::shared_ptr<rclcpp::Node> & node, const std::string & topic) {
  const auto start = std::chrono::steady_clock::now();
  while (node->count_publishers(topic) == 0 && std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(node->count_publishers(topic), 0u);
}

/// Put the fault in the store as CONFIRMED, the way a real confirmation would.
void confirm_fault(ros2_medkit_fault_manager::FaultStorage * storage, const std::string & code) {
  rclcpp::Clock clock;
  storage->report_fault_event(code, ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED,
                              ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR, "test", "/node", clock.now(),
                              ros2_medkit_fault_manager::DebounceConfig{});
}

}  // namespace

// A restart must not hand out capture ids below the ones already stored: eviction
// protects the HIGHEST id, so a counter starting at zero makes the capture just
// written the first one dropped, and get_snapshots returns the stale set first.
TEST_F(SnapshotCaptureTest, CaptureIdContinuesFromWhatStorageAlreadyHolds) {
  storage_->set_max_snapshots_per_fault(0);

  // Rows from an earlier process, under a different fault: the counter is global.
  ros2_medkit_fault_manager::SnapshotData earlier;
  earlier.fault_code = "SOMETHING_ELSE";
  earlier.topic = "/old";
  earlier.message_type = "std_msgs/msg/Float64";
  earlier.data = R"({"data": 1.0})";
  earlier.captured_at_ns = 1;
  earlier.capture_id = 41;
  storage_->store_snapshots({earlier});

  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/seeded", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["SEEDED_FAULT"] = {"/plc/seeded"};
  // Constructed AFTER the rows exist, exactly as a fault manager starting on a
  // populated database would be.
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 3.0;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  await_publisher(node_, "/plc/seeded");

  confirm_fault(storage_.get(), "SEEDED_FAULT");
  capture.capture("SEEDED_FAULT");

  const auto rows = storage_->get_snapshots("SEEDED_FAULT");
  ASSERT_FALSE(rows.empty());
  EXPECT_GT(rows.front().capture_id, 41) << "the new capture outranks everything already stored";
}

// An acknowledgement can land while a capture is still sampling: clear_fault deletes
// the fault's rows, and a batch written afterwards would put them back.
TEST_F(SnapshotCaptureTest, ACaptureFinishingAfterAcknowledgementIsNotStored) {
  storage_->set_max_snapshots_per_fault(0);
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/acked", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["ACKED_FAULT"] = {"/plc/acked"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 5.0;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  await_publisher(node_, "/plc/acked");

  confirm_fault(storage_.get(), "ACKED_FAULT");
  ASSERT_TRUE(storage_->clear_fault("ACKED_FAULT"));

  capture.capture("ACKED_FAULT");

  EXPECT_TRUE(storage_->get_snapshots("ACKED_FAULT").empty())
      << "acknowledgement promised these were gone; the batch must not resurrect them";
}

// ...unless the operator asked for snapshots to survive acknowledgement, in which
// case no such promise was made and the readings are worth keeping.
TEST_F(SnapshotCaptureTest, ACaptureFinishingAfterAcknowledgementIsStoredWhenEvidenceIsRetained) {
  storage_->set_max_snapshots_per_fault(0);
  storage_->set_retain_snapshots_on_clear(true);
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/retained", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["RETAINED_FAULT"] = {"/plc/retained"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 9.0;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  await_publisher(node_, "/plc/retained");

  confirm_fault(storage_.get(), "RETAINED_FAULT");
  ASSERT_TRUE(storage_->clear_fault("RETAINED_FAULT"));

  capture.capture("RETAINED_FAULT");

  EXPECT_FALSE(storage_->get_snapshots("RETAINED_FAULT").empty());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
