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
//
// Drives the REAL mechanism: a real publisher on a topic-name typo and a real subscriber
// on the intended target topic, both LaserScan, over real DDS - and the detector reads the
// live endpoint counts via gateway_node->get_topic_names_and_types() +
// count_publishers()/count_subscribers() - never hand-built TopicEndpointCounts (that is
// test_orphan_policy.cpp's job) - and raises/clears through a real ReportFault service
// round-trip to a fake fault_manager. NOT a full-gateway e2e (that is
// test/e2e/test_orphan_e2e.test.py); this proves the detector end to end within its own
// scope.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"

using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::graph_fault_codes::kOrphan;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in orphan_detector.cpp but self-registers via
// REGISTER_DETECTOR, which runs when that .cpp is linked into this test. Pull an
// instance from the registry (no production factory needed).
std::unique_ptr<ros2_medkit_graph_watchdog::Detector> make_orphan() {
  for (auto & d : ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all()) {
    if (d->id() == "orphan") {
      return std::move(d);
    }
  }
  return nullptr;
}
// Aggregated fault source: no Component in the test snapshot, so graph_source_id()
// falls back to this literal (see aggregated_fault.hpp).
constexpr const char * kGraphSource = "graph_watchdog";
}  // namespace

class OrphanIntegrationTest : public ::testing::Test {
 protected:
  // Must run before the FIRST test's fixture is constructed - see the identical
  // rationale in test_qos_mismatch_integration.cpp.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    gateway_ = std::make_shared<rclcpp::Node>("orphan_it_gateway");
    sink_ = std::make_shared<rclcpp::Node>("orphan_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        [this](const std::shared_ptr<ReportFault::Request> & req, const std::shared_ptr<ReportFault::Response> & resp) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            received_.push_back(*req);
          }
          resp->accepted = true;  // ReportFault.srv response field is `bool accepted`
        });
    client_ = gateway_->create_client<ReportFault>("/fault_manager/report_fault");
    exec_.add_node(gateway_);
    exec_.add_node(sink_);
    spin_ = std::thread([this]() {
      exec_.spin();
    });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override {
    exec_.cancel();
    if (spin_.joinable()) {
      spin_.join();
    }
    exec_.remove_node(gateway_);
    exec_.remove_node(sink_);
    client_.reset();
    srv_.reset();
    sink_.reset();
    typo_pub_.reset();
    target_sub_.reset();
    debug_pub_.reset();
    typo_pub_node_.reset();
    target_sub_node_.reset();
    debug_pub_node_.reset();
    peer_sub_.reset();
    peer_sub_node_.reset();
    gateway_.reset();
  }

  DetectorContext make_ctx(DetectorMode mode) {
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.node_mutex = &node_mutex_;
    ctx.mode = mode;
    ctx.gate = nullptr;  // ungated
    ctx.fault_client = client_;
    ctx.snapshot = &snapshot_;  // empty -> aggregated source falls back to kGraphSource
    return ctx;
  }

  // Endpoint discovery (rmw participant builtin-topic data) is populated by the
  // middleware's own background discovery, not by executor spinning - so these helper
  // nodes are never added to exec_. They just need to stay alive for the DDS entities to
  // remain visible to the gateway node.
  static rclcpp::Publisher<LaserScan>::SharedPtr
  make_publisher(rclcpp::Node::SharedPtr & node_out, const std::string & node_name, const std::string & topic) {
    node_out = std::make_shared<rclcpp::Node>(node_name);
    return node_out->create_publisher<LaserScan>(topic, rclcpp::QoS(10));
  }

  static rclcpp::Subscription<LaserScan>::SharedPtr
  make_subscription(rclcpp::Node::SharedPtr & node_out, const std::string & node_name, const std::string & topic) {
    node_out = std::make_shared<rclcpp::Node>(node_name);
    return node_out->create_subscription<LaserScan>(topic, rclcpp::QoS(10), [](LaserScan::UniquePtr) {});
  }

  // How many matching faults are currently in the log (for absence checks + snapshots).
  std::size_t count_faults(const std::string & source_id, uint8_t event_type) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == kOrphan && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  // True if any recorded FAILED for `source_id` carries a description containing EVERY needle.
  bool any_failed_desc_contains(const std::string & source_id, const std::vector<std::string> & needles) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id != source_id || r.fault_code != kOrphan || r.event_type != ReportFault::Request::EVENT_FAILED) {
        continue;
      }
      bool all = true;
      for (const auto & needle : needles) {
        if (r.description.find(needle) == std::string::npos) {
          all = false;
          break;
        }
      }
      if (all) {
        return true;
      }
    }
    return false;
  }

  // Tick until a NEW matching fault appears beyond `baseline_count` (arrived AFTER the
  // trigger), or timeout. 130 iterations at 100ms is a 13s window, comfortably over the
  // >=5s real DDS endpoint-discovery window pub/sub visibility needs.
  bool poll_for_new(const std::string & source_id, uint8_t event_type, std::size_t baseline_count,
                    ros2_medkit_graph_watchdog::Detector & det, DetectorContext & ctx) {
    for (int i = 0; i < 130; ++i) {
      det.tick(ctx);
      std::this_thread::sleep_for(100ms);
      if (count_faults(source_id, event_type) > baseline_count) {
        return true;
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr gateway_, sink_;
  rclcpp::Service<ReportFault>::SharedPtr srv_;
  rclcpp::Client<ReportFault>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread spin_;
  std::mutex mtx_, node_mutex_;
  std::vector<ReportFault::Request> received_;
  IntrospectionInput snapshot_;  // owned, empty entity snapshot (no Component -> literal source)

  rclcpp::Node::SharedPtr typo_pub_node_, target_sub_node_, debug_pub_node_, peer_sub_node_;
  rclcpp::Publisher<LaserScan>::SharedPtr typo_pub_, debug_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr target_sub_, peer_sub_;
};

// The full story in one test: a pub-only typo (/scam) + sub-only intended target (/scan),
// same LaserScan type, raises the aggregate naming BOTH endpoints; a concurrent lone
// pub-only /debug topic (no counterpart at all) never appears in any raised description
// (positive control - proves the detector discriminates a real near-miss pair from "just
// some topic with only one side"); destroying the typo publisher (never "add a publisher
// on /scan" - that would leave /scam orphaned forever) clears the aggregate.
TEST_F(OrphanIntegrationTest, TypoPairRaisesNamesBothDebugNeverNamedThenClearsOnRemoval) {
  typo_pub_ = make_publisher(typo_pub_node_, "orphan_it_typo_pub", "/scam");
  target_sub_ = make_subscription(target_sub_node_, "orphan_it_target_sub", "/scan");
  debug_pub_ = make_publisher(debug_pub_node_, "orphan_it_debug_pub", "/debug");

  auto det = make_orphan();
  ASSERT_TRUE(det) << "orphan did not self-register - REGISTER_DETECTOR did not run";
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/scam"}));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/scan"}));
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"/debug"}))
      << "/debug (a lone topic with no counterpart) must never be named in an aggregated raise";

  // Clear: DESTROY the /scam typo publisher (not "add a publisher on /scan" - that would
  // leave /scam permanently orphaned). Once /scam is gone, the near-miss pair vanishes.
  typo_pub_.reset();
  typo_pub_node_.reset();

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// A restart is not a typo. Any node that publishes one name and subscribes a near-identical
// one of the same type - a CAN/serial bridge on /can_rx + /can_tx, any relay that
// republishes a one-character variant - leaves EXACTLY the pub-only/sub-only shape this
// detector matches, for as long as its peer is down. Before the persistence gate that was a
// CONFIRMED ERROR on the first sweep (fault_manager ships confirmation_threshold -1), with a
// "reconcile the two names" instruction attached to something that is not misnamed at all.
TEST_F(OrphanIntegrationTest, PeerRestartWindowDoesNotRaise) {
  // Bridge side only: publishes /can_rx, subscribes /can_tx. Its peer is down.
  typo_pub_ = make_publisher(typo_pub_node_, "orphan_it_bridge_pub", "/can_rx");
  target_sub_ = make_subscription(target_sub_node_, "orphan_it_bridge_sub", "/can_tx");

  auto det = make_orphan();
  ASSERT_TRUE(det);
  det->configure({{"grace", 10}});  // the shipped default, stated explicitly
  auto ctx = make_ctx(DetectorMode::Raise);

  for (int i = 0; i < 6; ++i) {  // peer down, but for fewer ticks than grace
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a peer that is merely restarting must not be reported as a name typo";

  // Peer returns: both topics are two-sided again, so the near-miss pair stops matching.
  debug_pub_ = make_publisher(debug_pub_node_, "orphan_it_peer_pub", "/can_tx");
  peer_sub_ = make_subscription(peer_sub_node_, "orphan_it_peer_sub", "/can_rx");
  for (int i = 0; i < 25; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the streak must reset when the pair stops matching, never accumulate across the gap";
}

// The persistence gate must not cost detection: a genuine typo is static, so it is still
// there N ticks later and still gets reported.
TEST_F(OrphanIntegrationTest, StaticTypoStillRaisesOnceGraceElapses) {
  typo_pub_ = make_publisher(typo_pub_node_, "orphan_it_static_pub", "/scam2");
  target_sub_ = make_subscription(target_sub_node_, "orphan_it_static_sub", "/scan2");

  auto det = make_orphan();
  ASSERT_TRUE(det);
  det->configure({{"grace", 2}});
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, before, *det, ctx));
  // Worded as a candidate, not a conclusion: the same shape is produced by a departed peer,
  // so telling the operator to rename one of the two would be a wrong remediation.
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"possible topic-name mismatch"}));
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"likely a remap typo"}));
}

// A heuristic over topic names cannot be right for every graph, so an operator needs a way to
// silence one finding without turning the detector off. Same escape hatch the other detectors
// offer. Exact match only: allowlisting one name must not silence a similarly-named topic.
TEST_F(OrphanIntegrationTest, AllowlistedTopicIsNeverReported) {
  typo_pub_ = make_publisher(typo_pub_node_, "orphan_it_allow_pub", "/scam3");
  target_sub_ = make_subscription(target_sub_node_, "orphan_it_allow_sub", "/scan3");

  auto det = make_orphan();
  ASSERT_TRUE(det);
  det->configure({{"grace", 0}, {"allowlist", {"/scam3"}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), before)
      << "an allowlisted topic must never be reported, however long the pair holds";
}
