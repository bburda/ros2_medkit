// Copyright 2025 mfaferek93
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

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/rosbag_capture.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::RosbagCapture;
using ros2_medkit_fault_manager::RosbagConfig;
using ros2_medkit_fault_manager::SnapshotConfig;

class RosbagCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_rosbag_capture_node");
    storage_ = std::make_unique<InMemoryFaultStorage>();

    // Create temp directory for test bags
    temp_dir_ = std::filesystem::temp_directory_path() / "rosbag_capture_test";
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override {
    node_.reset();
    storage_.reset();
    rclcpp::shutdown();

    // Clean up temp directory
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
  }

  RosbagConfig create_rosbag_config(bool enabled = true) {
    RosbagConfig config;
    config.enabled = enabled;
    config.duration_sec = 2.0;
    config.duration_after_sec = 0.5;
    config.topics = "all";
    config.format = "mcap";
    config.storage_path = temp_dir_.string();
    config.max_bag_size_mb = 10;
    config.max_total_storage_mb = 50;
    config.auto_cleanup = true;
    return config;
  }

  SnapshotConfig create_snapshot_config() {
    SnapshotConfig config;
    config.enabled = true;
    config.default_topics = {"/test_topic"};
    return config;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<InMemoryFaultStorage> storage_;
  std::filesystem::path temp_dir_;
};

// Constructor tests

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorRequiresValidNode) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_THROW(RosbagCapture(nullptr, storage_.get(), rosbag_config, snapshot_config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorRequiresValidStorage) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_THROW(RosbagCapture(node_.get(), nullptr, rosbag_config, snapshot_config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorSucceedsWithValidParams) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorWithDisabledRosbag) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorFallsBackOnUnknownFormat) {
  // An unknown/unavailable format must NOT terminate the node; it degrades to
  // sqlite3 (always shipped with rosbag2) and capture stays enabled.
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "invalid_format";
  auto snapshot_config = create_snapshot_config();
  std::shared_ptr<RosbagCapture> rb;
  EXPECT_NO_THROW(rb = std::make_shared<RosbagCapture>(node_.get(), storage_.get(), rosbag_config, snapshot_config));
  ASSERT_NE(rb, nullptr);
  EXPECT_TRUE(rb->is_enabled());
  EXPECT_EQ(rb->config().format, "sqlite3");
}

// The crash-safety branches below force the storage probe via an injected double,
// because mcap is installed in CI so the real probe cannot reach them there.

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConfiguredFormatUnavailableFallsBackToSqlite3) {
  // A known format (mcap) whose plugin is unavailable degrades to sqlite3 and
  // capture stays enabled - the exact scenario the crash-safety change exists for.
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "mcap";
  auto snapshot_config = create_snapshot_config();
  RosbagCapture::StorageProbeFn probe = [](const std::string & f) -> std::optional<std::string> {
    if (f == "mcap") {
      return std::string("simulated: mcap plugin not found");
    }
    return std::nullopt;  // sqlite3 usable
  };
  std::shared_ptr<RosbagCapture> rb;
  EXPECT_NO_THROW(
      rb = std::make_shared<RosbagCapture>(node_.get(), storage_.get(), rosbag_config, snapshot_config, probe));
  ASSERT_NE(rb, nullptr);
  EXPECT_TRUE(rb->is_enabled());
  EXPECT_EQ(rb->config().format, "sqlite3");
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, NoUsableBackendDisablesCaptureWithoutCrashing) {
  // When neither the configured format nor sqlite3 is usable, capture self-disables
  // and the node keeps running (no throw out of the constructor).
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "mcap";
  auto snapshot_config = create_snapshot_config();
  RosbagCapture::StorageProbeFn probe = [](const std::string &) -> std::optional<std::string> {
    return std::string("simulated: backend unavailable");
  };
  std::shared_ptr<RosbagCapture> rb;
  EXPECT_NO_THROW(
      rb = std::make_shared<RosbagCapture>(node_.get(), storage_.get(), rosbag_config, snapshot_config, probe));
  ASSERT_NE(rb, nullptr);
  EXPECT_FALSE(rb->is_enabled());
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, Sqlite3BaselineUnavailableDisablesCapture) {
  // When the always-shipped sqlite3 baseline itself fails to load, capture
  // self-disables rather than crashing the node.
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "sqlite3";
  auto snapshot_config = create_snapshot_config();
  RosbagCapture::StorageProbeFn probe = [](const std::string &) -> std::optional<std::string> {
    return std::string("simulated: rosbag2 base install broken");
  };
  std::shared_ptr<RosbagCapture> rb;
  EXPECT_NO_THROW(
      rb = std::make_shared<RosbagCapture>(node_.get(), storage_.get(), rosbag_config, snapshot_config, probe));
  ASSERT_NE(rb, nullptr);
  EXPECT_FALSE(rb->is_enabled());
}

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, ConstructorAcceptsMcapFormat) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "mcap";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// State management tests

// @verifies REQ_INTEROP_088
TEST_F(RosbagCaptureTest, IsEnabledReturnsConfigState) {
  auto rosbag_config = create_rosbag_config(true);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture_enabled(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_TRUE(capture_enabled.is_enabled());

  rosbag_config.enabled = false;
  RosbagCapture capture_disabled(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_FALSE(capture_disabled.is_enabled());
}

TEST_F(RosbagCaptureTest, AutoStartsWhenNotLazy) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = false;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start=false, capture auto-starts on construction
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StartMakesRunning) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StopMakesNotRunning) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
  capture.stop();
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureTest, DoubleStartIsIdempotent) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
  capture.start();  // Second start should not throw
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StopWithoutStartIsIdempotent) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_NO_THROW(capture.stop());  // Should not throw when already stopped
  EXPECT_FALSE(capture.is_running());
}

// Lazy start tests

TEST_F(RosbagCaptureTest, LazyStartDoesNotRunImmediately) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = true;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start, capture is not running until a fault triggers it
  // The start() would typically be called internally on fault
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureTest, NonLazyAutoStartsOnConstruction) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = false;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start=false, auto-starts immediately on construction
  EXPECT_TRUE(capture.is_running());
}

// Topic configuration tests

TEST_F(RosbagCaptureTest, AllTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ConfigTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "config";
  auto snapshot_config = create_snapshot_config();
  snapshot_config.default_topics = {"/topic1", "/topic2"};
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ExplicitTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "explicit";
  rosbag_config.include_topics = {"/topic1", "/topic2"};
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ExcludeTopicsRespected) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  rosbag_config.exclude_topics = {"/rosout", "/parameter_events"};
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, EntityTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "entity";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, QosMatchDisabledFallsBackToSensorData) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  rosbag_config.qos_match = false;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, SensorTopicsExcludedByDefaultInBroadMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  rosbag_config.exclude_sensor_topics = true;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST(RosbagHighBandwidthTopicTest, MatchesSensorStreamsButNotLookalikes) {
  // High-bandwidth sensor streams are classified as such.
  EXPECT_TRUE(RosbagCapture::is_high_bandwidth_topic("/camera/image_raw"));
  EXPECT_TRUE(RosbagCapture::is_high_bandwidth_topic("/image"));
  EXPECT_TRUE(RosbagCapture::is_high_bandwidth_topic("/points"));
  EXPECT_TRUE(RosbagCapture::is_high_bandwidth_topic("/camera/depth/points"));
  EXPECT_TRUE(RosbagCapture::is_high_bandwidth_topic("/camera/image_raw/compressed"));

  // Low-bandwidth lookalikes that merely contain the word must NOT be excluded.
  EXPECT_FALSE(RosbagCapture::is_high_bandwidth_topic("/waypoints"));
  EXPECT_FALSE(RosbagCapture::is_high_bandwidth_topic("/setpoints"));
  EXPECT_FALSE(RosbagCapture::is_high_bandwidth_topic("/keypoints"));
  EXPECT_FALSE(RosbagCapture::is_high_bandwidth_topic("/joint_states"));
  EXPECT_FALSE(RosbagCapture::is_high_bandwidth_topic("/cmd_vel"));
}

// Fault lifecycle tests

TEST_F(RosbagCaptureTest, OnFaultPrefailedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_prefailed("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultConfirmedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_confirmed("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultClearedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_cleared("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultPrefailedStartsLazyCapture) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = true;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_FALSE(capture.is_running());
  capture.on_fault_prefailed("TEST_FAULT");
  EXPECT_TRUE(capture.is_running());
}

// Storage path tests

TEST_F(RosbagCaptureTest, DefaultStoragePathUsed) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.storage_path = "";  // Empty = use default
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, CustomStoragePathAccepted) {
  auto rosbag_config = create_rosbag_config();
  auto custom_path = temp_dir_ / "custom";
  rosbag_config.storage_path = custom_path.string();
  auto snapshot_config = create_snapshot_config();
  // Storage path creation happens when bag is written, not on construction
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Format tests

TEST_F(RosbagCaptureTest, McapFormatAccepted) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "mcap";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Duration configuration tests

TEST_F(RosbagCaptureTest, ZeroDurationHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.0;
  rosbag_config.duration_after_sec = 0.0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, NegativeDurationClamped) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = -1.0;
  rosbag_config.duration_after_sec = -1.0;
  auto snapshot_config = create_snapshot_config();
  // Should not throw, negative values should be handled gracefully
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Size limits tests

TEST_F(RosbagCaptureTest, ZeroMaxBagSizeHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.max_bag_size_mb = 0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ZeroMaxTotalStorageHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.max_total_storage_mb = 0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Quota accounting for shared bags: a burst of correlated faults confirming in
// one post-roll window produces one recording with one row per fault, so the
// quota must count it once and eviction must free it once.

TEST_F(RosbagCaptureTest, SharedBagTotalTakesTheLargestRowPerPath) {
  // Rows of one recording can transiently disagree on size while it is being
  // finalised. Undercounting there would pass the quota while a bigger bag sits
  // on disk, so the total takes the largest row - same as the SQLite MAX().
  // Two bags with opposite code orderings, because rows iterate by fault code:
  // whichever way the map runs, last-write-wins gets one of them wrong.
  auto store = [this](const char * code, const char * path, size_t bytes) {
    ros2_medkit_fault_manager::RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = path;
    info.format = "mcap";
    info.duration_sec = 5.0;
    info.size_bytes = bytes;
    info.created_at_ns = 1000;
    storage_->store_rosbag_file(info);
  };

  store("AAA_BIG", "/tmp/shared_bag_a", 4096);
  store("ZZZ_SMALL", "/tmp/shared_bag_a", 512);
  store("BBB_SMALL", "/tmp/shared_bag_b", 256);
  store("YYY_BIG", "/tmp/shared_bag_b", 2048);

  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 4096u + 2048u);
}

TEST_F(RosbagCaptureTest, EvictionFreesASharedBagOnceAndKeepsTheQuotaHonest) {
  // Deleting a single row of a shared bag leaves the directory on disk (a sibling
  // still points at it), so crediting its bytes per row would satisfy the quota
  // on paper only and stop the eviction while storage is still over budget.
  const auto shared_path = (temp_dir_ / "shared_bag").string();
  const auto solo_path = (temp_dir_ / "solo_bag").string();
  std::filesystem::create_directories(shared_path);
  std::filesystem::create_directories(solo_path);

  ros2_medkit_fault_manager::RosbagFileInfo shared;
  shared.file_path = shared_path;
  shared.format = "mcap";
  shared.duration_sec = 5.0;
  shared.size_bytes = 1000;
  shared.created_at_ns = 1000;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    shared.fault_code = code;
    storage_->store_rosbag_file(shared);
  }

  ros2_medkit_fault_manager::RosbagFileInfo solo;
  solo.fault_code = "UNRELATED";
  solo.file_path = solo_path;
  solo.format = "mcap";
  solo.duration_sec = 5.0;
  solo.size_bytes = 600;
  solo.created_at_ns = 2000;
  storage_->store_rosbag_file(solo);

  ASSERT_EQ(storage_->get_total_rosbag_storage_bytes(), 1600u);

  auto evicted = RosbagCapture::evict_bags_over_quota(storage_.get(), 1000);

  // The oldest bag goes as a unit, taking all three of its rows with it, and the
  // freed bytes match what actually left the disk.
  EXPECT_EQ(evicted, std::vector<std::string>{shared_path});
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 600u);
  EXPECT_FALSE(std::filesystem::exists(shared_path));
  EXPECT_FALSE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());
  EXPECT_FALSE(storage_->get_rosbag_file("CORRELATED_A").has_value());
  EXPECT_FALSE(storage_->get_rosbag_file("CORRELATED_B").has_value());

  // The newer bag fits under the quota and must survive.
  EXPECT_TRUE(storage_->get_rosbag_file("UNRELATED").has_value());
  EXPECT_TRUE(std::filesystem::exists(solo_path));
}

TEST_F(RosbagCaptureTest, EvictionKeepsGoingWhenTheSharedBagIsNotEnough) {
  // Freeing the shared bag still leaves storage over budget here. The running
  // total must stay truthful (no size_t underflow from per-row subtraction)
  // and the eviction must move on to the next bag.
  const auto shared_path = (temp_dir_ / "shared_bag").string();
  const auto solo_path = (temp_dir_ / "solo_bag").string();

  ros2_medkit_fault_manager::RosbagFileInfo shared;
  shared.file_path = shared_path;
  shared.format = "mcap";
  shared.duration_sec = 5.0;
  shared.size_bytes = 1000;
  shared.created_at_ns = 1000;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A"}) {
    shared.fault_code = code;
    storage_->store_rosbag_file(shared);
  }

  ros2_medkit_fault_manager::RosbagFileInfo solo;
  solo.fault_code = "UNRELATED";
  solo.file_path = solo_path;
  solo.format = "mcap";
  solo.duration_sec = 5.0;
  solo.size_bytes = 600;
  solo.created_at_ns = 2000;
  storage_->store_rosbag_file(solo);

  auto evicted = RosbagCapture::evict_bags_over_quota(storage_.get(), 500);

  EXPECT_EQ(evicted, (std::vector<std::string>{shared_path, solo_path}));
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 0u);
}

// Integration tests (simplified without actual message publishing)

class RosbagCaptureIntegrationTest : public RosbagCaptureTest {
 protected:
  void spin_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Publish on a test topic until the capture has buffered something, so a
  /// confirmation produces a real bag instead of the "buffer empty" no-op.
  void fill_buffer(const std::string & topic) {
    auto pub = node_->create_publisher<std_msgs::msg::String>(topic, 10);
    std_msgs::msg::String msg;
    msg.data = "payload";
    // The capture subscribes via a 500 ms discovery timer, so publish across
    // several ticks rather than assuming the subscription exists immediately.
    for (int i = 0; i < 40; ++i) {
      pub->publish(msg);
      spin_for(std::chrono::milliseconds(50));
    }
  }
};

TEST_F(RosbagCaptureIntegrationTest, FullFaultLifecycleWithNoMessages) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.5;
  rosbag_config.duration_after_sec = 0.2;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  // Start capture
  capture.start();
  EXPECT_TRUE(capture.is_running());

  // Simulate fault lifecycle
  capture.on_fault_prefailed("TEST_FAULT_001");

  // Let some time pass
  spin_for(std::chrono::milliseconds(100));

  capture.on_fault_confirmed("TEST_FAULT_001");

  // Wait for post-fault timer
  spin_for(std::chrono::milliseconds(300));

  // Clear the fault
  capture.on_fault_cleared("TEST_FAULT_001");

  // Stop capture
  capture.stop();
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureIntegrationTest, MultipleFaultsHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.5;
  rosbag_config.duration_after_sec = 0.1;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Multiple faults in sequence
  capture.on_fault_prefailed("FAULT_A");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_prefailed("FAULT_B");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_confirmed("FAULT_A");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_confirmed("FAULT_B");
  spin_for(std::chrono::milliseconds(200));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, FaultConfirmedDuringPostRollGetsTheSameBag) {
  // Correlated faults arrive in a burst. The one that confirms while the first
  // fault's post-roll is still running used to be skipped outright and ended up
  // with no black box at all; it must now resolve to the burst's recording.
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 10.0;
  rosbag_config.duration_after_sec = 1.0;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();
  fill_buffer("/rosbag_burst_probe");

  capture.on_fault_confirmed("ROOT_CAUSE");
  capture.on_fault_confirmed("CORRELATED");

  // Let the post-fault timer fire and finalise the recording.
  spin_for(std::chrono::milliseconds(1600));

  auto root = storage_->get_rosbag_file("ROOT_CAUSE");
  ASSERT_TRUE(root.has_value()) << "the first fault must produce a bag";
  auto correlated = storage_->get_rosbag_file("CORRELATED");
  ASSERT_TRUE(correlated.has_value()) << "the fault confirmed during the post-roll lost its recording";
  EXPECT_EQ(correlated->file_path, root->file_path);
  EXPECT_TRUE(std::filesystem::exists(root->file_path));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, ClearingOneFaultOfABurstKeepsTheSharedBag) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 10.0;
  rosbag_config.duration_after_sec = 1.0;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();
  fill_buffer("/rosbag_burst_cleanup_probe");

  capture.on_fault_confirmed("ROOT_CAUSE");
  capture.on_fault_confirmed("CORRELATED");
  spin_for(std::chrono::milliseconds(1600));

  auto root = storage_->get_rosbag_file("ROOT_CAUSE");
  ASSERT_TRUE(root.has_value());
  ASSERT_TRUE(storage_->get_rosbag_file("CORRELATED").has_value());
  const std::string bag_path = root->file_path;

  // Auto-cleanup of one fault must not pull the bag out from under its sibling.
  capture.on_fault_cleared("CORRELATED");
  EXPECT_TRUE(std::filesystem::exists(bag_path));
  EXPECT_TRUE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());

  // The last reference going away does delete it.
  capture.on_fault_cleared("ROOT_CAUSE");
  EXPECT_FALSE(std::filesystem::exists(bag_path));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, StopDuringPostRollFinalisesTheRecording) {
  // Shutting down (or restarting) mid post-roll must not strand the recording:
  // the writer has to close, the bag has to get its metadata row, and the
  // recording state has to clear so the next confirmation opens its own bag
  // instead of attaching to a recording whose timer is already gone.
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 10.0;
  rosbag_config.duration_after_sec = 30.0;  // long enough that stop() lands mid post-roll
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();
  fill_buffer("/rosbag_stop_probe");

  capture.on_fault_confirmed("BEFORE_STOP");
  capture.stop();

  auto before = storage_->get_rosbag_file("BEFORE_STOP");
  ASSERT_TRUE(before.has_value()) << "a post-roll cut short by stop() must still leave a usable bag";

  // Restart and confirm again: the new fault must get its own recording.
  capture.start();
  fill_buffer("/rosbag_stop_probe_2");

  capture.on_fault_confirmed("AFTER_RESTART");
  capture.stop();

  auto after = storage_->get_rosbag_file("AFTER_RESTART");
  ASSERT_TRUE(after.has_value()) << "confirmation after restart was swallowed by stale recording state";
  EXPECT_NE(after->file_path, before->file_path);
}

TEST_F(RosbagCaptureIntegrationTest, FaultClearedBeforeConfirmed) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Prefail then clear (fault didn't confirm)
  capture.on_fault_prefailed("CLEARED_EARLY");
  spin_for(std::chrono::milliseconds(50));
  capture.on_fault_cleared("CLEARED_EARLY");

  // Should not crash
  spin_for(std::chrono::milliseconds(100));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, ConfirmedWithoutPrefailed) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Direct confirm (edge case)
  capture.on_fault_confirmed("DIRECT_CONFIRM");
  spin_for(std::chrono::milliseconds(200));

  capture.stop();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
