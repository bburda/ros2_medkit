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
#include <cmath>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

/// No planned-stop windows declared: every fault in these cases is a surprise,
/// which is what a gateway with an empty planned_stops table serves.
static const std::vector<ros2_medkit_gateway::faults::PlannedStopWindow> kNoPlannedStops;

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::DataProvider;
using ros2_medkit_gateway::DataProviderError;
using ros2_medkit_gateway::DataProviderErrorInfo;
using ros2_medkit_gateway::EntityFreezeFrameCapture;
using ros2_medkit_gateway::handlers::FaultHandlers;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;

namespace {

/// Fake plugin DataProvider serving PLC-like values for one entity. The
/// temperature is settable so tests can prove a re-confirm re-samples.
class FakePlcDataProvider : public DataProvider {
 public:
  explicit FakePlcDataProvider(std::string entity_id) : entity_id_(std::move(entity_id)) {
  }

  void set_temperature(double value) {
    temperature_.store(value);
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    json items = json::array();
    items.push_back({{"id", "temperature"}, {"name", "Temperature"}, {"value", temperature_.load()}});
    items.push_back({{"id", "pressure"}, {"name", "Pressure"}, {"value", 3.2}});
    return ros2_medkit_gateway::dto::DataListResult{json{{"items", std::move(items)}}};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
  std::atomic<double> temperature_{42.5};
};

/// Fake DataProvider serving fixed list_data content (for gate tests).
class StaticContentDataProvider : public DataProvider {
 public:
  StaticContentDataProvider(std::string entity_id, json content)
    : entity_id_(std::move(entity_id)), content_(std::move(content)) {
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    return ros2_medkit_gateway::dto::DataListResult{content_};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
  json content_;
};

FaultEvent make_confirmed_event(const std::string & fault_code, const std::vector<std::string> & sources) {
  FaultEvent event;
  event.event_type = FaultEvent::EVENT_CONFIRMED;
  event.fault.fault_code = fault_code;
  event.fault.severity = Fault::SEVERITY_ERROR;
  event.fault.status = Fault::STATUS_CONFIRMED;
  event.fault.reporting_sources = sources;
  return event;
}

class EntityFreezeFrameCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("entity_freeze_frame_test_node");
    // Publisher lives on a node the main executor never spins (see
    // Ros2SubscriptionSlotTest): publishers mutate their node's rcutils_hash_map
    // under the hood, which TSan otherwise flags against the spun node.
    publisher_node_ = std::make_shared<rclcpp::Node>("entity_freeze_frame_test_publisher");
    publisher_ = publisher_node_->create_publisher<FaultEvent>("/fault_manager/events", rclcpp::QoS(100).reliable());
    provider_ = std::make_unique<FakePlcDataProvider>("plc_app");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    // The capture creates its fault-events subscription on this executor's
    // dedicated _sub node, off the main node (issue #375).
    sub_exec_ = std::make_unique<Ros2SubscriptionExecutor>(node_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    sub_exec_.reset();
    executor_.reset();
    publisher_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  /// Block until the capture's subscription has matched our publisher, so a
  /// reliable-QoS publish is not dropped by a late-joining subscriber.
  bool wait_for_match() {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (publisher_->get_subscription_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  /// Publish an event repeatedly until the capture holds frames (or timeout).
  /// The callback runs on the subscription executor worker, not this thread.
  bool publish_and_wait(EntityFreezeFrameCapture & capture, const FaultEvent & event) {
    if (!wait_for_match()) {
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      publisher_->publish(event);
      if (!capture.frames_for(event.fault.fault_code).empty()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<FaultEvent>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::unique_ptr<FakePlcDataProvider> provider_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ConfirmedPluginFaultCapturesEntityValues) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_OVERPRESSURE", {"plc_app"})));

  auto frames = capture.frames_for("PLC_OVERPRESSURE");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);
  EXPECT_DOUBLE_EQ(frames[0].values.value("pressure", 0.0), 3.2);
  EXPECT_GT(frames[0].captured_at_ns, 0);
  EXPECT_FALSE(frames[0].startup_catchup);  // live confirm edge, not catch-up
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonPluginSourceCapturesNothing) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [](const std::string &) -> DataProvider * {
    return nullptr;  // ROS FQN sources are never plugin-owned
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("ROS_FAULT", {"/powertrain/engine/temp_sensor"});
  // Deliver repeatedly so the callback demonstrably ran and filtered it.
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("ROS_FAULT").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonConfirmedEventsAreIgnored) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_UPDATED_ONLY", {"plc_app"});
  event.event_type = FaultEvent::EVENT_UPDATED;
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_UPDATED_ONLY").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, StandingFaultsAreFramedWithoutAnyEvent) {
  // The startup case: the fault confirmed before this object subscribed, so no
  // event will ever arrive for it. The lister supplies it instead.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 7.0}}})}};
      },
      256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {{"PLC_STANDING", {"route_plc_app"}}};
      });

  // No publish at all - the frame must appear from the catch-up alone.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (capture.frames_for("PLC_STANDING").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  const auto frames = capture.frames_for("PLC_STANDING");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 7.0);
  EXPECT_TRUE(frames[0].startup_catchup);  // consumers can tell catch-up frames apart
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, QueuedLiveConfirmDedupesStandingCatchUp) {
  // Both paths carry the same fault code and the confirm is already queued
  // when the catch-up snapshots the queue: the catch-up must skip that code
  // entirely (one plugin read per confirm, no double capture) and the drain
  // loop's live frame is the one that lands.
  std::atomic<bool> published{false};
  std::atomic<bool> standing_sampled{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&standing_sampled](const std::string & entity_id) -> std::optional<json> {
        double level = 0.0;
        if (entity_id == "route_live_app") {
          level = 99.0;
        } else if (entity_id == "route_standing_app") {
          level = 1.0;
          standing_sampled.store(true);
        } else {
          return std::nullopt;
        }
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level}}})}};
      },
      256,
      [&published](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        // Hold the catch-up until the live confirm has reached the queue so
        // the standing entry and the queued event genuinely overlap.
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!published.load() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(10ms);
        }
        std::this_thread::sleep_for(500ms);  // let the event reach the queue
        return {{"PLC_BOTH_PATHS", {"route_standing_app"}}};
      });

  ASSERT_TRUE(wait_for_match());
  const auto live = make_confirmed_event("PLC_BOTH_PATHS", {"route_live_app"});
  publisher_->publish(live);
  published.store(true);

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  std::vector<EntityFreezeFrameCapture::Frame> frames;
  while (std::chrono::steady_clock::now() < deadline) {
    frames = capture.frames_for("PLC_BOTH_PATHS");
    if (!frames.empty()) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  // The queued confirm dedupes the catch-up: the standing entity was never
  // sampled and the only frame is the live one.
  EXPECT_FALSE(standing_sampled.load());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_live_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 99.0);
  EXPECT_FALSE(frames[0].startup_catchup);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ThrowingListerDoesNotKillCaptureThread) {
  // The lister runs on the capture thread's entry path: an escaping exception
  // there is std::terminate. It must be swallowed, and the drain loop must
  // still capture live confirms afterwards.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, 256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        throw std::runtime_error("fault services exploded");
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_AFTER_THROW", {"plc_app"})));
  EXPECT_FALSE(capture.frames_for("PLC_AFTER_THROW").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DestructionAbortsBlockedLister) {
  // A lister stuck waiting for the fault services must not hold the
  // destructor's join for its full timeout: should_abort turns true on stop.
  std::atomic<bool> entered{false};
  std::atomic<bool> abort_seen{false};
  auto capture = std::make_unique<EntityFreezeFrameCapture>(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [&](const std::function<bool()> & should_abort) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        entered.store(true);
        while (!should_abort()) {  // stands in for the bounded service wait
          std::this_thread::sleep_for(10ms);
        }
        abort_seen.store(true);
        return {};
      });

  const auto entry_deadline = std::chrono::steady_clock::now() + 15s;
  while (!entered.load() && std::chrono::steady_clock::now() < entry_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(entered.load());

  const auto start = std::chrono::steady_clock::now();
  capture.reset();
  EXPECT_TRUE(abort_seen.load());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 5s);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ListerRunsOnlyAfterEventsPublisherMatched) {
  // ~/events is reliable but volatile on both ends: a confirm published after
  // the standing snapshot but before the reader matches would be missed by
  // both paths. The catch-up must therefore wait for a matched publisher
  // before calling the lister.
  publisher_.reset();                  // no events publisher exists yet
  std::this_thread::sleep_for(250ms);  // let the graph forget it
  std::atomic<bool> publisher_created{false};
  std::atomic<bool> lister_ran{false};
  std::atomic<bool> saw_publisher{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [&](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        saw_publisher.store(publisher_created.load());
        lister_ran.store(true);
        return {};
      });

  // Ample time for an unordered catch-up to have run before any publisher.
  std::this_thread::sleep_for(500ms);
  publisher_created.store(true);
  publisher_ = publisher_node_->create_publisher<FaultEvent>("/fault_manager/events", rclcpp::QoS(100).reliable());

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!lister_ran.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(lister_ran.load());
  EXPECT_TRUE(saw_publisher.load());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, CatchUpCapsStoredFramesAtMaxFaults) {
  // max_faults = 2. The no-data fault must not consume the cap (nothing was
  // stored for it), and the fault past the cap must be skipped instead of
  // FIFO-evicting the catch-up's own earliest frame.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id == "route_no_data_app") {
          return json{{"connected", false}};  // dead row: no frame stored
        }
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 7.0}}})}};
      },
      /*max_faults=*/2,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {{"PLC_CAP_NO_DATA", {"route_no_data_app"}},
                {"PLC_CAP_A", {"route_a"}},
                {"PLC_CAP_B", {"route_b"}},
                {"PLC_CAP_C", {"route_c"}}};
      });

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (capture.frames_for("PLC_CAP_B").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // would be enough for an uncapped C capture
  EXPECT_TRUE(capture.frames_for("PLC_CAP_NO_DATA").empty());
  EXPECT_FALSE(capture.frames_for("PLC_CAP_A").empty());  // not evicted by C
  EXPECT_FALSE(capture.frames_for("PLC_CAP_B").empty());
  EXPECT_TRUE(capture.frames_for("PLC_CAP_C").empty());  // past the cap
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, FramesRetainedAcrossClearAndOverwrittenOnReconfirm) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_CYCLING", {"plc_app"})));
  ASSERT_DOUBLE_EQ(capture.frames_for("PLC_CYCLING")[0].values.value("temperature", 0.0), 42.5);

  // Clearing retains the confirmed-state record, mirroring the fault_manager's
  // freeze-frame retention across clear_fault.
  auto cleared = make_confirmed_event("PLC_CYCLING", {"plc_app"});
  cleared.event_type = FaultEvent::EVENT_CLEARED;
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(cleared);
    std::this_thread::sleep_for(10ms);
  }
  auto retained = capture.frames_for("PLC_CYCLING");
  ASSERT_EQ(retained.size(), 1u);
  EXPECT_DOUBLE_EQ(retained[0].values.value("temperature", 0.0), 42.5);

  // The next confirm re-samples the plugin, replacing the retained frame, so
  // a new occurrence never serves the previous incident's values past confirm.
  provider_->set_temperature(99.0);
  const auto reconfirm = make_confirmed_event("PLC_CYCLING", {"plc_app"});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool overwritten = false;
  while (std::chrono::steady_clock::now() < deadline && !overwritten) {
    publisher_->publish(reconfirm);
    std::this_thread::sleep_for(20ms);
    auto latest = capture.frames_for("PLC_CYCLING");
    overwritten = !latest.empty() && std::abs(latest[0].values.value("temperature", 0.0) - 99.0) < 1e-9;
  }
  EXPECT_TRUE(overwritten);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, RouteFallbackCapturesWhenPluginHasNoDataProvider) {
  // Mirror the commercial PLC bridges: no DataProvider, values only through
  // the plugin's x-plc-data route (dispatched in-process by the fetcher).
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id != "route_plc_app") {
          return std::nullopt;
        }
        return json{{"connected", true},
                    {"items", json::array({{{"name", "level"}, {"value", 87.5}, {"unit", "mm"}},
                                           {{"name", "alarm"}, {"value", true}}})}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_ROUTE_LEVEL_HIGH", {"route_plc_app"})));

  auto frames = capture.frames_for("PLC_ROUTE_LEVEL_HIGH");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 87.5);
  EXPECT_EQ(frames[0].values.value("alarm", false), true);
  EXPECT_GT(frames[0].captured_at_ns, 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, RouteFallbackSkipsDisconnectedPlcWithNoValues) {
  // A disconnected PLC with nothing cached must not freeze-frame a row of
  // nulls. One that still serves its last known values does get a frame - see
  // DisconnectedEntityWithLastKnownValuesIsCaptured.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", false}, {"items", json::array({{{"name", "level"}, {"value", nullptr}}})}};
      });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_DISCONNECTED", {"route_plc_app"});
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_DISCONNECTED").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DisconnectedEntityWithLastKnownValuesIsCaptured) {
  // The loss-of-comms case: the bridge reports the link down and still serves
  // the values it last read. That is what the fault is about, so it is frozen,
  // with the payload's link flag and own timestamp carried as provenance.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", false},
                    {"items", json::array({{{"name", "level"}, {"value", 42.0}}})},
                    {"timestamp", 1234567890}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST", {"route_plc_app"})));

  const auto frames = capture.frames_for("PLC_COMMS_LOST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].values["level"], 42.0);
  ASSERT_TRUE(frames[0].connected.has_value());
  EXPECT_FALSE(*frames[0].connected);
  EXPECT_EQ(frames[0].source_timestamp, 1234567890);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DataProviderWinsOverRouteFallback) {
  std::atomic<bool> fetcher_called{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      [&fetcher_called](const std::string &) -> std::optional<json> {
        fetcher_called = true;
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 1.0}}})}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_PROVIDER_FIRST", {"plc_app"})));

  auto frames = capture.frames_for("PLC_PROVIDER_FIRST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);  // provider values, not route
  EXPECT_FALSE(fetcher_called.load());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DataProviderWithoutLiveValuesCapturesNothing) {
  // Same "no row = nothing captured" invariant as the route path: an empty
  // items array (cold poll cache) or all-null values must not freeze-frame a
  // row of {} / nulls, connected or not.
  StaticContentDataProvider cold_provider("cold_app", json{{"items", json::array()}});
  StaticContentDataProvider null_provider("null_app",
                                          json{{"connected", false}, {"items", json::array({{{"id", "level"}}})}});
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [&](const std::string & entity_id) -> DataProvider * {
    if (entity_id == "cold_app") {
      return &cold_provider;
    }
    if (entity_id == "null_app") {
      return &null_provider;
    }
    return nullptr;
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_NO_LIVE_DATA", {"cold_app", "null_app"});
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_NO_LIVE_DATA").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DisconnectedDataProviderWithLastKnownValuesIsCaptured) {
  // DataProvider flavour of the loss-of-comms case: list_data flags the link
  // down but still serves the last known values - captured, with the link
  // state on the frame.
  StaticContentDataProvider down_provider(
      "down_app", json{{"connected", false}, {"items", json::array({{{"id", "level"}, {"value", 5.5}}})}});
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [&](const std::string & entity_id) -> DataProvider * {
    return entity_id == "down_app" ? &down_provider : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_PROVIDER_COMMS_LOST", {"down_app"})));

  const auto frames = capture.frames_for("PLC_PROVIDER_COMMS_LOST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].values["level"], 5.5);
  ASSERT_TRUE(frames[0].connected.has_value());
  EXPECT_FALSE(*frames[0].connected);
  EXPECT_TRUE(frames[0].source_timestamp.is_null());  // provider content has no timestamp field
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, OldestFaultEvictedPastMaxFaults) {
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, /*max_faults=*/2);

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_A", {"plc_app"})));
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_B", {"plc_app"})));
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_C", {"plc_app"})));

  EXPECT_TRUE(capture.frames_for("PLC_EVICT_A").empty());  // FIFO-evicted
  EXPECT_FALSE(capture.frames_for("PLC_EVICT_B").empty());
  EXPECT_FALSE(capture.frames_for("PLC_EVICT_C").empty());
}

TEST(ContentHasLiveData, GatesOnItemsNotOnTheLinkFlag) {
  using Capture = EntityFreezeFrameCapture;
  EXPECT_TRUE(Capture::content_has_live_data(
      json{{"connected", true}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // No "connected" field: items alone are enough (plugin-defined shape).
  EXPECT_TRUE(Capture::content_has_live_data(json{{"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // Disconnected but still serving its last known values: this is the
  // loss-of-comms case, and those values are the point of the frame.
  EXPECT_TRUE(Capture::content_has_live_data(
      json{{"connected", false}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  EXPECT_FALSE(Capture::content_has_live_data(json{{"connected", true}, {"items", json::array()}}));
  EXPECT_FALSE(Capture::content_has_live_data(json{{"connected", true}}));
  EXPECT_FALSE(Capture::content_has_live_data(json::array()));
}

TEST(ContentReportsDisconnected, TriggerPathGateDiscriminatesFromFreezeFramePath) {
  using Capture = EntityFreezeFrameCapture;
  // The loss-of-comms payload: link down, last known values still served.
  const json down_with_values{{"connected", false}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}};
  // The freeze-frame path (content_has_live_data alone) captures it...
  EXPECT_TRUE(Capture::content_has_live_data(down_with_values));
  // ...while the trigger value fetcher (gateway_node) also requires the link
  // up, so it yields nullopt and threshold rules hold state on the outage
  // instead of firing on a frozen number.
  EXPECT_TRUE(Capture::content_reports_disconnected(down_with_values));
  EXPECT_FALSE(Capture::content_reports_disconnected(
      json{{"connected", true}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // No flag (DataProvider list_data shape) or a non-boolean flag: not down.
  EXPECT_FALSE(Capture::content_reports_disconnected(json{{"items", json::array()}}));
  EXPECT_FALSE(Capture::content_reports_disconnected(json{{"connected", "no"}}));
  EXPECT_FALSE(Capture::content_reports_disconnected(json::array()));
}

TEST(ValuesHaveData, RejectsEmptyAndAllNull) {
  using Capture = EntityFreezeFrameCapture;
  EXPECT_TRUE(Capture::values_have_data(json{{"a", 1}}));
  EXPECT_TRUE(Capture::values_have_data(json{{"a", nullptr}, {"b", 0.0}}));
  EXPECT_FALSE(Capture::values_have_data(json::object()));
  EXPECT_FALSE(Capture::values_have_data(json{{"a", nullptr}, {"b", nullptr}}));
  EXPECT_FALSE(Capture::values_have_data(json(nullptr)));
}

TEST(ValuesFromListContent, BuildsCompactDictFromItems) {
  json content = {{"items", json::array({{{"id", "a"}, {"value", 1}},
                                         {{"name", "b"}, {"value", "on"}},  // falls back to "name" key
                                         {{"id", "c"}}})}};                 // no value -> null
  auto values = EntityFreezeFrameCapture::values_from_list_content(content);
  EXPECT_EQ(values["a"], 1);
  EXPECT_EQ(values["b"], "on");
  EXPECT_TRUE(values["c"].is_null());
}

TEST(ValuesFromListContent, NonItemsShapeKeptVerbatim) {
  json content = {{"status", "ok"}, {"raw", 7}};
  EXPECT_EQ(EntityFreezeFrameCapture::values_from_list_content(content), content);
}

TEST(ValuesFromListContent, NonStringIdOrNameNeverThrows) {
  // json::value() throws type_error.302 on present-but-non-string keys;
  // plugin content is untrusted, so the builder must be total instead.
  json content = {{"items", json::array({{{"id", 42}, {"name", "b"}, {"value", 2}},  // int id -> name fallback
                                         {{"id", 7}, {"value", 1}},                  // no usable key -> skipped
                                         {{"name", true}}})}};                       // no usable key -> skipped
  json values;
  ASSERT_NO_THROW(values = EntityFreezeFrameCapture::values_from_list_content(content));
  EXPECT_EQ(values, json({{"b", 2}}));
}

TEST(StandingFaultsFromListReply, ParsesWellFormedReply) {
  const json data = {{"faults", json::array({json{{"fault_code", "F1"}, {"reporting_sources", json::array({"a", "b"})}},
                                             json{{"fault_code", "F2"}, {"reporting_sources", json::array()}}})},
                     {"count", 2}};
  const auto standing = EntityFreezeFrameCapture::standing_faults_from_list_reply(data);
  ASSERT_TRUE(standing.has_value());
  ASSERT_EQ(standing->size(), 2u);
  EXPECT_EQ((*standing)[0].fault_code, "F1");
  EXPECT_EQ((*standing)[0].reporting_sources, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ((*standing)[1].fault_code, "F2");
  EXPECT_TRUE((*standing)[1].reporting_sources.empty());
}

TEST(StandingFaultsFromListReply, RepliesNotShapedLikeListFaultsYieldNullopt) {
  // nullopt (vs empty vector) is what lets the caller warn on a malformed or
  // renamed reply instead of silently disabling the catch-up.
  using Capture = EntityFreezeFrameCapture;
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json::array()).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json(3)).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json(nullptr)).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json::object()).has_value());  // no "faults" key
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json{{"faults", "nope"}}).has_value());
}

TEST(StandingFaultsFromListReply, MalformedItemsAreSkippedNeverThrownOn) {
  const json data = {
      {"faults",
       json::array({"not_an_object", json{{"reporting_sources", json::array({"a"})}},     // missing fault_code
                    json{{"fault_code", 42}, {"reporting_sources", json::array({"a"})}},  // non-string fault_code
                    json{{"fault_code", "NO_SOURCES"}},                                   // missing sources
                    json{{"fault_code", "BAD_SOURCES"}, {"reporting_sources", "x"}},      // non-array sources
                    json{{"fault_code", "MIXED"}, {"reporting_sources", json::array({"ok", 7, true, "also_ok"})}}})}};
  std::optional<std::vector<EntityFreezeFrameCapture::StandingFault>> standing;
  ASSERT_NO_THROW(standing = EntityFreezeFrameCapture::standing_faults_from_list_reply(data));
  ASSERT_TRUE(standing.has_value());
  ASSERT_EQ(standing->size(), 1u);
  EXPECT_EQ((*standing)[0].fault_code, "MIXED");
  EXPECT_EQ((*standing)[0].reporting_sources, (std::vector<std::string>{"ok", "also_ok"}));
}

TEST(MergeEntityFreezeFrames, AppendsWhenNoConfiguredFreezeFrame) {
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  const auto & snap = merged["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "plc_app");
  EXPECT_EQ(json::parse(snap["data"].get<std::string>())["temperature"], 42.5);
  EXPECT_EQ(snap["captured_at_ns"], 1234);
  EXPECT_FALSE(snap.contains("capture_origin"));  // confirm-edge frames carry no marker
}

TEST(MergeEntityFreezeFrames, StartupCatchUpFrameCarriesCaptureOrigin) {
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;
  frame.startup_catchup = true;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_EQ(merged["snapshots"][0]["capture_origin"], "startup");
}

TEST(MergeEntityFreezeFrames, ExplicitConfigWins) {
  // A fault_manager-captured freeze-frame (explicit snapshot config) must
  // suppress the zero-config entity frames.
  json env_data = {{"snapshots", json::array({{{"type", "freeze_frame"},
                                               {"snapshot_type", "freeze_frame"},
                                               {"name", "scan"},
                                               {"data", "{}"},
                                               {"topic", "/scan"},
                                               {"message_type", "sensor_msgs/msg/LaserScan"},
                                               {"captured_at_ns", 1}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_EQ(merged["snapshots"][0]["name"], "scan");
}

TEST(MergeEntityFreezeFrames, RosbagSnapshotDoesNotSuppressEntityFrames) {
  // Only a freeze-frame counts as "explicit config captured": a rosbag-only
  // environment still gets the entity frame appended.
  json env_data = {
      {"snapshots",
       json::array({{{"type", "rosbag"}, {"snapshot_type", "rosbag"}, {"name", "rosbag_X"}, {"fault_code", "X"}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 2u);
  EXPECT_EQ(merged["snapshots"][1]["name"], "plc_app");
}

TEST(MergeEntityFreezeFrames, NoFramesNoChange) {
  json env_data = {{"snapshots", json::array()}};
  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {});
  EXPECT_EQ(merged, env_data);
}

TEST(MergeEntityFreezeFrames, CarriesLinkStateAndSourceTimestampWhenKnown) {
  // Loss-of-comms provenance: the payload's link flag and own timestamp ride
  // the snapshot entry through to the wire x-medkit block - and only when the
  // plugin actually reported them.
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame down_frame;
  down_frame.entity_id = "plc_app";
  down_frame.values = {{"level", 42.0}};
  down_frame.captured_at_ns = 1234;
  down_frame.connected = false;
  down_frame.source_timestamp = 1234567890;
  EntityFreezeFrameCapture::Frame bare_frame;
  bare_frame.entity_id = "other_app";
  bare_frame.values = {{"temperature", 42.5}};
  bare_frame.captured_at_ns = 5678;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {down_frame, bare_frame});
  ASSERT_EQ(merged["snapshots"].size(), 2u);
  EXPECT_EQ(merged["snapshots"][0]["connected"], false);
  EXPECT_EQ(merged["snapshots"][0]["source_timestamp"], 1234567890);
  EXPECT_FALSE(merged["snapshots"][1].contains("connected"));
  EXPECT_FALSE(merged["snapshots"][1].contains("source_timestamp"));

  // And onto the wire: build_sovd_fault_response surfaces them in x-medkit.
  const auto detail = FaultHandlers::build_sovd_fault_response(
      json{{"fault_code", "PLC_COMMS_LOST"}, {"status", "CONFIRMED"}}, merged, "/apps/plc_app", kNoPlannedStops);
  ASSERT_TRUE(detail.environment_data.snapshots.has_value());
  const auto & wire = *detail.environment_data.snapshots;
  ASSERT_EQ(wire.size(), 2u);
  EXPECT_EQ(wire[0]["x-medkit"]["connected"], false);
  EXPECT_EQ(wire[0]["x-medkit"]["source_timestamp"], 1234567890);
  EXPECT_FALSE(wire[1]["x-medkit"].contains("connected"));
  EXPECT_FALSE(wire[1]["x-medkit"].contains("source_timestamp"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
