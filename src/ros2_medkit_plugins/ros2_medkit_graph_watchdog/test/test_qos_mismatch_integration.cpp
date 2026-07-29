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
// Drives the REAL mechanism: real nodes publish/subscribe on real DDS topics with
// deliberately mismatched QoS, and the detector reads the live RESOLVED profiles via
// gateway_node->get_publishers_info_by_topic()/get_subscriptions_info_by_topic() -
// never hand-built rmw_qos_profile_t values (that is test_qos_policy.cpp's job) - and
// raises/clears through a real ReportFault service round-trip to a fake fault_manager.
// NOT a full-gateway e2e (that is test/e2e/test_qos_e2e.test.py); this proves the
// detector end to end within its own scope.
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
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"

using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::graph_fault_codes::kQosMismatch;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using StringMsg = std_msgs::msg::String;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in qos_mismatch_detector.cpp but self-registers via
// REGISTER_DETECTOR, which runs when that .cpp is linked into this test. Pull an
// instance from the registry (no production factory needed).
std::unique_ptr<ros2_medkit_graph_watchdog::Detector> make_qos_mismatch() {
  for (auto & d : ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all()) {
    if (d->id() == "qos_mismatch") {
      return std::move(d);
    }
  }
  return nullptr;
}
// Aggregated fault source: no Component in the test snapshot, so graph_source_id()
// falls back to this literal (see aggregated_fault.hpp).
constexpr const char * kGraphSource = "graph_watchdog";
}  // namespace

class QosMismatchIntegrationTest : public ::testing::Test {
 protected:
  // Must run before the FIRST test's fixture is constructed - see the identical
  // rationale in test_param_drift_integration.cpp.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    gateway_ = std::make_shared<rclcpp::Node>("qm_it_gateway");
    sink_ = std::make_shared<rclcpp::Node>("qm_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        [this](const std::shared_ptr<ReportFault::Request> req, std::shared_ptr<ReportFault::Response> resp) {
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
    probe_pub_.reset();
    probe_sub_.reset();
    ok_pub_.reset();
    ok_sub_.reset();
    probe_pub_node_.reset();
    probe_sub_node_.reset();
    ok_pub_node_.reset();
    ok_sub_node_.reset();
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

  // Pub/sub QoS endpoint discovery (rmw participant builtin-topic data) is populated by
  // the middleware's own background discovery, not by executor spinning - so these
  // helper nodes are never added to exec_. They just need to stay alive for the DDS
  // entities (and thus their advertised QoS) to remain visible to the gateway node.
  static rclcpp::Publisher<StringMsg>::SharedPtr make_publisher(rclcpp::Node::SharedPtr & node_out,
                                                                const std::string & node_name,
                                                                const std::string & topic, const rclcpp::QoS & qos) {
    node_out = std::make_shared<rclcpp::Node>(node_name);
    return node_out->create_publisher<StringMsg>(topic, qos);
  }

  static rclcpp::Subscription<StringMsg>::SharedPtr make_subscription(rclcpp::Node::SharedPtr & node_out,
                                                                      const std::string & node_name,
                                                                      const std::string & topic,
                                                                      const rclcpp::QoS & qos) {
    node_out = std::make_shared<rclcpp::Node>(node_name);
    return node_out->create_subscription<StringMsg>(topic, qos, [](StringMsg::UniquePtr) {});
  }

  // How many matching faults are currently in the log (for absence checks + snapshots).
  std::size_t count_faults(const std::string & source_id, uint8_t event_type) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == kQosMismatch && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  // True if any recorded FAILED for `source_id` carries a description containing EVERY needle.
  bool any_failed_desc_contains(const std::string & source_id, const std::vector<std::string> & needles) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id != source_id || r.fault_code != kQosMismatch ||
          r.event_type != ReportFault::Request::EVENT_FAILED) {
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
  // >=5s real DDS endpoint-discovery window pub/sub QoS visibility needs.
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

  rclcpp::Node::SharedPtr probe_pub_node_, probe_sub_node_, ok_pub_node_, ok_sub_node_;
  rclcpp::Publisher<StringMsg>::SharedPtr probe_pub_, ok_pub_;
  rclcpp::Subscription<StringMsg>::SharedPtr probe_sub_, ok_sub_;
};

// The full story in one test: a mismatched pair raises the aggregate naming its topic; a
// concurrent RxO-compatible-but-different pair on another topic never appears in any
// raised description (proves the detector discriminates, not just "QoS differs somewhere
// in the graph"); fixing the mismatched pair's QoS clears the aggregate.
TEST_F(QosMismatchIntegrationTest, MismatchRaisesNamesTopicOkNeverNamedThenClearsOnFix) {
  // /probe: BEST_EFFORT pub + RELIABLE sub -> RxO-incompatible.
  probe_pub_ = make_publisher(probe_pub_node_, "qm_it_probe_pub", "/probe", rclcpp::QoS(10).best_effort());
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_probe_sub", "/probe", rclcpp::QoS(10).reliable());

  // /ok: RELIABLE pub + BEST_EFFORT sub -> RxO-compatible despite differing QoS.
  ok_pub_ = make_publisher(ok_pub_node_, "qm_it_ok_pub", "/ok", rclcpp::QoS(10).reliable());
  ok_sub_ = make_subscription(ok_sub_node_, "qm_it_ok_sub", "/ok", rclcpp::QoS(10).best_effort());

  auto det = make_qos_mismatch();
  ASSERT_TRUE(det) << "qos_mismatch did not self-register - REGISTER_DETECTOR did not run";
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/probe"}));
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"/ok"}))
      << "/ok (RxO-compatible-but-different QoS) must never be named in an aggregated raise";

  // Fix: replace the RELIABLE /probe sub with a BEST_EFFORT one (distinct node name -
  // avoids racing the old node's DDS teardown) -> the mismatch resolves.
  probe_sub_.reset();
  probe_sub_node_.reset();
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_probe_sub2", "/probe", rclcpp::QoS(10).best_effort());

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// The fault has to say WHICH subscriber is starved. Without the name, an operator reading
// "reliability: publisher BEST_EFFORT vs subscriber RELIABLE" on a topic with a dozen
// subscribers still has to ssh in and run `ros2 topic info -v` - the step this removes.
TEST_F(QosMismatchIntegrationTest, DescriptionNamesTheStarvedSubscriber) {
  probe_pub_ = make_publisher(probe_pub_node_, "qm_it_named_pub", "/probe_named", rclcpp::QoS(10).best_effort());
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_named_sub", "/probe_named", rclcpp::QoS(10).reliable());

  auto det = make_qos_mismatch();
  ASSERT_TRUE(det);
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, before, *det, ctx));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/probe_named", "qm_it_named_sub"}))
      << "the aggregated description must name the starved subscriber, not just its topic";
}

// A subscriber that still receives from another publisher is NOT starved, but the pair DDS
// refused is still dropping one producer's data with nothing reporting it. That is the
// silent fault this detector exists for, so it is reported - one level down.
TEST_F(QosMismatchIntegrationTest, PartialIncompatibilityRaisesAtWarnNotError) {
  // /probe_partial: one BEST_EFFORT and one RELIABLE publisher, RELIABLE subscriber.
  // The subscriber matches the RELIABLE publisher and never matches the other one.
  probe_pub_ = make_publisher(probe_pub_node_, "qm_it_part_bad", "/probe_partial", rclcpp::QoS(10).best_effort());
  ok_pub_ = make_publisher(ok_pub_node_, "qm_it_part_good", "/probe_partial", rclcpp::QoS(10).reliable());
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_part_sub", "/probe_partial", rclcpp::QoS(10).reliable());

  auto det = make_qos_mismatch();
  ASSERT_TRUE(det);
  auto ctx = make_ctx(DetectorMode::Raise);

  const auto before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, before, *det, ctx))
      << "a publisher whose data DDS discards must not be silent just because another one works";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/probe_partial", "qm_it_part_sub"}));

  std::lock_guard<std::mutex> lk(mtx_);
  bool saw_warn = false;
  for (const auto & r : received_) {
    if (r.fault_code == kQosMismatch && r.event_type == ReportFault::Request::EVENT_FAILED) {
      EXPECT_EQ(r.severity, ros2_medkit_msgs::msg::Fault::SEVERITY_WARN)
          << "nothing here is fully starved, so the sweep's worst finding is a degraded topic";
      saw_warn = true;
    }
  }
  EXPECT_TRUE(saw_warn);
}

// An operator's own tool is not a robot fault: rviz2 subscribes RELIABLE and never
// negotiates down to a BEST_EFFORT sensor publisher. Before the allowlist the only lever
// was turning the whole detector off.
TEST_F(QosMismatchIntegrationTest, AllowlistedTopicIsNeverReported) {
  probe_pub_ = make_publisher(probe_pub_node_, "qm_it_allow_pub", "/probe_allow", rclcpp::QoS(10).best_effort());
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_allow_sub", "/probe_allow", rclcpp::QoS(10).reliable());

  auto det = make_qos_mismatch();
  ASSERT_TRUE(det);
  det->configure({{"allowlist", nlohmann::json::array({"/probe_allow"})}});
  auto ctx = make_ctx(DetectorMode::Raise);

  for (int i = 0; i < 40; ++i) {  // 4s, well past the >=5s-window helper's own raise latency
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"/probe_allow"}));
}

// One sweep used to raise, and the shipped fault_manager confirms on the first report and
// pulls a rosbag capture with it. Endpoint discovery is not atomic, so a transient
// half-discovered pair looked exactly like starvation.
TEST_F(QosMismatchIntegrationTest, GraceDefersTheRaise) {
  probe_pub_ = make_publisher(probe_pub_node_, "qm_it_grace_pub", "/probe_grace", rclcpp::QoS(10).best_effort());
  probe_sub_ = make_subscription(probe_sub_node_, "qm_it_grace_sub", "/probe_grace", rclcpp::QoS(10).reliable());

  auto det = make_qos_mismatch();
  ASSERT_TRUE(det);
  // Large enough that no run of this test can tick past it before the assertion below.
  det->configure({{"grace", 10000}});
  auto ctx = make_ctx(DetectorMode::Raise);

  for (int i = 0; i < 60; ++i) {  // 6s, past the endpoint-discovery window the other tests need
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a topic must stay affected for grace+1 consecutive ticks before it is reported";
}
