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
// Drives the REAL mechanism: a real NodeDeathDetector (pulled from the registry, exactly
// as test_qos_mismatch_integration.cpp and test_lifecycle_expectation_integration.cpp do
// for their own detectors) against hand-built IntrospectionInput snapshots and a REAL
// ReliabilityGate, raising/clearing through a real ReportFault service round-trip to a
// fake fault_manager. NOT a full-gateway e2e; this proves the detector + suppression
// framework end to end within their own scope.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::Detector;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::DetectorRegistry;
using ros2_medkit_graph_watchdog::PresenceOwnership;
using ros2_medkit_graph_watchdog::ReliabilityGate;
using ros2_medkit_graph_watchdog::graph_fault_codes::kNodeDisappeared;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in node_death_detector.cpp but self-registers via
// REGISTER_DETECTOR, which runs when that .cpp is linked into this test.
std::unique_ptr<Detector> make_node_death() {
  for (auto & d : DetectorRegistry::instance().create_all()) {
    if (d->id() == "node_death") {
      return std::move(d);
    }
  }
  return nullptr;
}

// Aggregated fault source: no Component in the test snapshot, so graph_source_id() falls
// back to this literal (see aggregated_fault.hpp).
constexpr const char * kGraphSource = "graph_watchdog";

/// Captures rcutils log output for as long as it is alive and restores the console handler
/// on every exit path.
class LogCapture {
 public:
  LogCapture() {
    active().store(this);
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }
  ~LogCapture() {
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
    active().store(nullptr);
  }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;
  LogCapture(LogCapture &&) = delete;
  LogCapture & operator=(LogCapture &&) = delete;

  int count(const std::string & needle) const {
    std::lock_guard<std::mutex> lk(mutex_);
    int n = 0;
    for (const auto & line : lines_) {
      if (line.find(needle) != std::string::npos) {
        ++n;
      }
    }
    return n;
  }

 private:
  static std::atomic<LogCapture *> & active() {
    static std::atomic<LogCapture *> current{nullptr};
    return current;
  }

  static void handler(const rcutils_log_location_t * /*location*/, int /*severity*/, const char * /*name*/,
                      rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args) {
    char buf[1024];
    va_list copy;
    va_copy(copy, *args);
    // The format string and args arrive from the logging call site through the handler
    // signature - there is no literal to write here.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    vsnprintf(buf, sizeof(buf), format, copy);
#pragma GCC diagnostic pop
    va_end(copy);
    LogCapture * capture = active().load();
    if (capture == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lk(capture->mutex_);
    capture->lines_.emplace_back(buf);
  }

  mutable std::mutex mutex_;
  std::vector<std::string> lines_;
};
}  // namespace

class NodeDeathIntegrationTest : public ::testing::Test {
 protected:
  // The ROS plumbing (both nodes, the executor, its spin thread, the fake service and the
  // client) is built ONCE for this whole binary, in SetUpTestSuite() - not per TEST_F, the
  // way every sibling integration test in this package builds it. None of this file's ~27
  // cases needs its OWN node identity: what makes a case independent is its own detector
  // instance and its own config, both already fresh per test, and the fixture's own
  // received_/snapshot_ state, cleared in SetUp() below. A full rclcpp::Node + executor +
  // OS thread create/destroy cycle is comparatively expensive and asynchronous on the DDS
  // side, and every sibling integration test in this package happens to run few enough
  // cases, each spending multiple SECONDS of its own on polling loops, that the cost never
  // shows up. This file's cases run in single-digit milliseconds with nothing to wait on,
  // so cycling that same plumbing ~27 times over in a few hundred milliseconds is a rate no
  // other test in this package produces - and doing so measurably stalled this binary
  // indefinitely at an arbitrary case, confirmed to move to a different case (never a fixed
  // one) across repeated runs of the identical binary with nothing else on the machine.
  // Building it once removes the cycling entirely rather than merely slowing it down.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    gateway_ = std::make_shared<rclcpp::Node>("nd_it_gateway");
    sink_ = std::make_shared<rclcpp::Node>("nd_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        [](const std::shared_ptr<ReportFault::Request> & req, const std::shared_ptr<ReportFault::Response> & resp) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            received_.push_back(*req);
          }
          resp->accepted = true;  // ReportFault.srv response field is `bool accepted`
        });
    client_ = gateway_->create_client<ReportFault>("/fault_manager/report_fault");
    // Constructed here, not as a default-initialized static member: MultiThreadedExecutor's
    // constructor needs a valid rclcpp context (it creates its own guard condition), and a
    // static member's default initializer runs at static-initialization time - before
    // main(), so before rclcpp::init() above ever has a chance to run.
    exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    exec_->add_node(gateway_);
    exec_->add_node(sink_);
    spin_ = std::thread([]() {
      exec_->spin();
    });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  static void TearDownTestSuite() {
    exec_->cancel();
    if (spin_.joinable()) {
      spin_.join();
    }
    exec_->remove_node(gateway_);
    exec_->remove_node(sink_);
    exec_.reset();
    client_.reset();
    srv_.reset();
    sink_.reset();
    gateway_.reset();
  }

  // Per-test isolation for the shared plumbing above: a stale response delivered late from
  // a PREVIOUS case must never be read as evidence by the NEXT one.
  void SetUp() override {
    std::lock_guard<std::mutex> lk(mtx_);
    received_.clear();
  }

  DetectorContext make_ctx(ReliabilityGate * gate) {
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.node_mutex = &node_mutex_;
    ctx.mode = DetectorMode::Raise;
    ctx.gate = gate;
    ctx.fault_client = client_;
    ctx.snapshot = &snapshot_;
    return ctx;
  }

  static App app_of(const std::string & id, bool online = true, const std::string & source = "runtime") {
    App a;
    a.id = id;
    a.is_online = online;
    a.source = source;
    a.bound_fqn = "/" + id;
    return a;
  }

  /// A permanently-present, never-under-test App any case whose subject departs must still
  /// carry in its snapshot alongside the departure.
  ///
  /// A live gateway's entity snapshot is never actually empty just because one node exits -
  /// the graph_watchdog App itself, the fault_manager's own node and whatever else is
  /// running are still in it. ReliabilityGate::update() reads that as a real signal: an
  /// EMPTY snapshot re-arms the global bringup grace (graph_seen_ = false), documented and
  /// deliberate for a genuine full-stack restart. The aggregated fault's own source_id
  /// ("graph_watchdog", see graph_source_id()) is never itself an app in any test snapshot,
  /// so it is always evaluated through that SAME global grace inside raise_fault() - and a
  /// snapshot that drops to truly empty the instant the app under test departs would
  /// silently re-gate every raise this file checks for, for a reason that has nothing to do
  /// with node_death or any suppressor. Real gateways never produce that snapshot shape, so
  /// no test here should either.
  static App anchor_app() {
    return app_of("anchor");
  }

  void set_apps(const std::vector<App> & apps) {
    snapshot_.apps = apps;
  }

  std::size_t count_faults(const std::string & source_id, uint8_t event_type) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == kNodeDisappeared && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  bool any_failed_desc_contains(const std::string & source_id, const std::vector<std::string> & needles) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id != source_id || r.fault_code != kNodeDisappeared ||
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

  /// Configure a fresh node_death, tick it once with an empty snapshot against a real
  /// node, and report whether the captured log carries `needle` - the shared shape every
  /// C1/C2/C3 case below needs.
  bool configure_warns(const nlohmann::json & config, const std::string & needle) {
    auto det = make_node_death();
    if (!det) {
      return false;
    }
    det->configure(config);
    LogCapture capture;
    IntrospectionInput empty;
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.snapshot = &empty;
    det->tick(ctx);
    return capture.count(needle) > 0;
  }

  // Suite-scoped ROS plumbing (see SetUpTestSuite()'s own doc for why this is not
  // per-fixture-instance). `inline` (C++17): each is defined here, once, with no separate
  // out-of-class definition. The executor's thread count is explicit, not the default (0):
  // MultiThreadedExecutor's own header documents 0 as "the number of cpu cores found
  // (minimum of 2)" - this fixture's actual concurrency need is at most two (the fake
  // service callback and the client's own response callback).
  static inline rclcpp::Node::SharedPtr gateway_, sink_;
  static inline rclcpp::Service<ReportFault>::SharedPtr srv_;
  static inline rclcpp::Client<ReportFault>::SharedPtr client_;
  // unique_ptr, not a by-value member: see SetUpTestSuite()'s own note on why this cannot
  // be constructed via a default member initializer.
  static inline std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  static inline std::thread spin_;
  static inline std::mutex mtx_;  // guards received_ (both the service callback and every reader below)
  static inline std::vector<ReportFault::Request> received_;

  // Per-test state: fresh for every TEST_F, unlike the plumbing above.
  std::mutex node_mutex_;
  IntrospectionInput snapshot_;
};

// =============================================================================================
// N9: peer-aggregated apps are never tracked.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, N9_PeerAggregatedAppsAreNeverTracked) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  // No anchor app here: this test's own assertions are all negative
  // (tracked_count_for_test()==0, no FAILED report) and so are unaffected by whether
  // ReliabilityGate's global bringup grace re-arms on an empty snapshot - see anchor_app()'s
  // own doc for why a case that checks a POSITIVE raise needs one and this one does not.
  set_apps({app_of("peer_app", /*online=*/true, /*source=*/"peer:robot2")});
  for (int i = 1; i <= 5; ++i) {
    gate.update(snapshot_, static_cast<std::uint64_t>(i));
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 0u) << "a peer app must never enter the tracked map";

  set_apps({});  // peer app "departs" - if it had ever been tracked, this would report it dead
  for (int i = 6; i <= 12; ++i) {
    gate.update(snapshot_, static_cast<std::uint64_t>(i));
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u);
}

TEST_F(NodeDeathIntegrationTest, IsOnlineFalseIsNeverTracked) {
  // The manifest/hybrid case the class doc describes: an App present in the snapshot but
  // not (yet) online must never be armed, however many sweeps see it.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("manifest_app", /*online=*/false)});
  for (int i = 1; i <= 10; ++i) {
    gate.update(snapshot_, static_cast<std::uint64_t>(i));
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u);
}

// =============================================================================================
// status_json() exposes tracked_count on GET /x-medkit-watchdog (detectors.node_death) - the
// field the ros2cli_ignored e2e row reads to prove tracked state does not grow across renamed-
// node churn cycles. Without this override the field is simply absent (Detector's own default),
// which that row's own comment treats as a condition to skip the comparison under rather than
// fail on - so this is required for that row to check anything at all.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, StatusJsonExposesTrackedCountMatchingTheTestSeam) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  // Before any tick: present from construction, not only after the first sweep.
  auto status = det->status_json();
  ASSERT_TRUE(status.is_object());
  ASSERT_TRUE(status.contains("tracked_count"));
  EXPECT_EQ(status.at("tracked_count").get<std::size_t>(), 0u);

  set_apps({app_of("a"), app_of("b")});
  gate.update(snapshot_, 1);
  det->tick(ctx);

  status = det->status_json();
  EXPECT_EQ(status.at("tracked_count").get<std::size_t>(), 2u);
  EXPECT_EQ(status.at("tracked_count").get<std::size_t>(), det->tracked_count_for_test());
}

TEST_F(NodeDeathIntegrationTest, StatusJsonTrackedCountStaysStableAcrossNeverArmingChurn) {
  // Mirrors the e2e row's own claim: N churn cycles of never-armed apps (present for one
  // tick each, then gone) must never move tracked_count - the SAME property N11 pins
  // through tracked_count_for_test(), checked here through the PUBLIC status route instead.
  ReliabilityGate gate(/*warmup_cycles=*/3, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("survivor")});
  for (std::uint64_t t = 1; t <= 4; ++t) {  // 4 ticks: armed at t=4 (4-1>=3)
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  const auto before = det->status_json().at("tracked_count").get<std::size_t>();
  ASSERT_EQ(before, 1u);

  for (int cycle = 0; cycle < 5; ++cycle) {
    set_apps({app_of("survivor"), app_of("churn_" + std::to_string(cycle))});
    gate.update(snapshot_, static_cast<std::uint64_t>(5 + cycle));
    det->tick(ctx);
  }

  const auto after = det->status_json().at("tracked_count").get<std::size_t>();
  EXPECT_EQ(before, after) << "five renamed-node cycles must not move tracked_count";
}

// =============================================================================================
// N11: the tracked map stays bounded under churn, and shrinks once a durable veto reclaims
// it (also corroborating S5's "a durable veto may reclaim bookkeeping" through the real
// detector, not just the pure tracker - test_node_liveness_tracker.cpp's own
// NodeLivenessTrackerPrune cases prove both halves of S5 precisely, at the framework level).
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, N11_TrackedCountStaysBoundedUnderChurnAndShrinksAfterReclaim) {
  ReliabilityGate gate(/*warmup_cycles=*/3, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000},  // floor(3000ms) == 0, so miss_grace=0 is legal
                  {"miss_grace", 0},
                  {"prune_grace", 2},
                  {"allowlist", nlohmann::json::array({"/victim"})},
                  {"suppress", nlohmann::json::array({"allowlist"})}});
  auto ctx = make_ctx(&gate);

  std::size_t max_seen = 0;
  std::uint64_t tick = 0;
  const auto run_tick = [&](const std::vector<App> & apps) {
    ++tick;
    set_apps(apps);
    gate.update(snapshot_, tick);
    det->tick(ctx);
    max_seen = std::max(max_seen, det->tracked_count_for_test());
  };

  // Arm the two survivors. is_armed() requires (tick - first_seen) >= warmup_cycles, so
  // first_seen's own tick (1) plus warmup_cycles(3) MORE ticks - 4 calls, not 3 - before the
  // gate considers them armed (see WarmupTracker::is_armed()).
  for (int i = 0; i < 4; ++i) {
    run_tick({app_of("alive_a"), app_of("alive_b")});
  }
  ASSERT_EQ(det->tracked_count_for_test(), 2u);

  // 100 ticks of churn. alive_a/alive_b stay present throughout. "victim" is present for
  // the first 4 ticks (enough to arm under warmup_cycles=3: present at ticks 5-8, armed at
  // tick 8 where 8-5>=3), then permanently gone. Every tick also carries ONE uniquely-named
  // "noise" app present for exactly that one tick alone and never again - well under
  // warmup_cycles, so none of the ~100 distinct noise apps can ever arm.
  for (int i = 0; i < 100; ++i) {
    std::vector<App> apps{app_of("alive_a"), app_of("alive_b"), app_of("noise_" + std::to_string(i))};
    if (i < 4) {
      apps.push_back(app_of("victim"));
    }
    run_tick(apps);
  }

  EXPECT_LE(max_seen, 3u) << "noise apps (never armed) and the victim's own eventual reclaim "
                             "must never push tracked_count past the 3 entities that were ever "
                             "genuinely armed";
  EXPECT_GE(max_seen, 3u) << "the victim must have been tracked at some point, or this test "
                             "would trivially pass by never tracking it at all";

  // prune_ticks = max(prune_grace=2, miss_grace(0)+1=1) = 2. The victim, durably suppressed
  // by the allowlist on every tick it is dead, must be reclaimed well before the churn loop
  // above even finished - confirmed here with a few more clean ticks.
  for (int i = 0; i < 10; ++i) {
    run_tick({app_of("alive_a"), app_of("alive_b")});
  }
  EXPECT_EQ(det->tracked_count_for_test(), 2u) << "the allowlisted, durably-suppressed victim must have been reclaimed";
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"/victim"}))
      << "an allowlisted death must never be reported in the first place";
}

// N11's own churn above never varies scale past NodeLivenessTracker's tracked_key_cap - its
// noise apps stay well under warmup_cycles, so none of them are ever armed at all, and the
// one entry that IS tracked and reclaimed (/victim) is reclaimed via prune()'s
// suppression-streak path, not the cap. An unsuppressed death has NO other reclaim path -
// prune() requires a durable suppressor - so this row exists specifically to prove the cap
// itself engages end to end through configure(), not merely inside the tracker in isolation
// (see test_node_liveness_tracker.cpp's own NodeLivenessTrackerCap suite for that).
TEST_F(NodeDeathIntegrationTest, N11_TrackedNodeCapBoundsUnsuppressedChurnAndKeepsTheFaultRaised) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}, {"tracked_node_cap", 3}});
  auto ctx = make_ctx(&gate);

  std::size_t max_seen = 0;
  std::uint64_t tick = 0;
  const auto run_tick = [&](const std::vector<App> & apps) {
    ++tick;
    set_apps(apps);
    gate.update(snapshot_, tick);
    det->tick(ctx);
    max_seen = std::max(max_seen, det->tracked_count_for_test());
  };

  // 20 distinct, unsuppressed identities - no allowlist, no suppress configured. Each is
  // armed on its own tick (PRESENT, so it never competes for the departed cap that tick),
  // then given a tick fully absent, where it matures instantly (miss_grace=0) and becomes a
  // departed-cap candidate. warmup_cycles=0 makes the fully-empty ticks harmless - the
  // global bringup grace re-arms on the very next non-empty tick regardless.
  for (int i = 0; i < 20; ++i) {
    run_tick({app_of("churn_" + std::to_string(i))});
    run_tick({});
  }

  // kCap + 1, not kCap: a present/armed key is never refused a slot or evicted for one (see
  // node_liveness_tracker.hpp's class doc) - only the DEPARTED subset is bounded. This
  // churn's own admission tick therefore transiently coexists with a departed set already
  // at the cap: kCap matured entries plus the one brand-new present arrival, until THAT one
  // departs on the very next tick and the departed set's own collapse brings it back to
  // kCap. Without the cap this would grow past 20, unbounded for the life of the process.
  constexpr std::size_t kCap = 3;
  EXPECT_LE(max_seen, kCap + 1) << "tracked_node_cap must actually engage under scale past it";
  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"more node(s) disappeared"}))
      << "identities the cap forced out of individual tracking must still count as content, "
         "or a death the cap collapsed would silently heal instead of staying raised";
}

// =============================================================================================
// Cap redesign regressions. Both sit PAST tracked_node_cap, in the two directions a cap that
// bounds the wrong thing gets wrong: a graph LARGER than the cap (every death must still be
// reported), and departed-set CHURN below miss_grace (capacity pressure must never fabricate a
// death). See node_liveness_tracker.hpp's own class doc for the ruling these pin.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, CapC1_GraphLargerThanTheDefaultCapStillReportsADeathAmongItsPresentMajority) {
  // Reproduces the sequence that exposes the defect: at the shipped default tracked_node_cap
  // (512), 513 armed nodes used to leave only one survivor once make_room() evicted every
  // present entry to admit the newcomer - and evicting a present entry can permanently lose
  // any death landing in the eviction window, because only ONLINE nodes re-enter `armed`.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});  // default tracked_node_cap (512)
  auto ctx = make_ctx(&gate);

  // 513 armed, present nodes - one more than the default cap - lexically ordered zero-padded
  // ids so the very first admitted is also lexically first, reproducing the same eviction
  // order the defect above depends on.
  std::vector<App> apps;
  apps.reserve(514);
  for (int i = 0; i < 513; ++i) {
    std::string idx = std::to_string(i);
    idx.insert(idx.begin(), 3 - idx.size(), '0');
    apps.push_back(app_of("n" + idx));
  }
  apps.push_back(anchor_app());
  set_apps(apps);
  gate.update(snapshot_, 1);
  det->tick(ctx);
  ASSERT_EQ(det->tracked_count_for_test(), 514u)
      << "every armed/present node must be tracked - none refused for capacity, however many "
         "there are";

  // Kill only the first one; every other node (512 of them) plus the anchor stay present.
  apps.erase(apps.begin());
  set_apps(apps);
  gate.update(snapshot_, 2);
  det->tick(ctx);  // misses(1) > miss_grace(0): dead

  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"n000"}))
      << "the one node that actually died among 513 present ones must still be reported - a "
         "cap that evicts present entries to make room can lose exactly this death";
}

TEST_F(NodeDeathIntegrationTest, CapC2_ChurnUnderCapPressureNeverFabricatesADeathBeforeMissGrace) {
  // Reproduces the sequence that exposes the defect: at cap(2) with three entries carrying
  // only one below-grace miss each, a make_room() that collapsed every non-idle entry used
  // to fold all three into the collapsed count, raising GRAPH_NODE_DISAPPEARED before any
  // of them had actually crossed miss_grace, and unhealably (their identities were erased).
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 2}, {"tracked_node_cap", 2}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("a"), app_of("b"), app_of("c"), anchor_app()});
  gate.update(snapshot_, 1);
  det->tick(ctx);  // arms a, b, c

  set_apps({anchor_app()});  // a, b, c all depart on the same tick
  gate.update(snapshot_, 2);
  det->tick(ctx);  // each misses 1 of 2 (miss_grace) - below grace, but the departed COUNT
                   // (3) exceeds tracked_node_cap(2): the exact capacity pressure that used
                   // to force make_room() to collapse entries before grace.

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "none of a/b/c has crossed miss_grace yet - capacity pressure alone must never "
         "report any of them dead early";
  EXPECT_EQ(det->tracked_count_for_test(), 4u)
      << "the anchor (idle) plus all three of a/b/c must still be individually tracked - none "
         "had matured, so none was an eligible collapse candidate despite the departed count "
         "exceeding the cap";

  // "/a" returns, relieving the pressure before anything matured; "/b" and "/c" stay gone and
  // now genuinely cross miss_grace - proving the pressure above did not silently erase either
  // identity (which would make it un-reportable, since only ONLINE nodes re-enter `armed`).
  set_apps({app_of("a"), anchor_app()});
  gate.update(snapshot_, 3);
  det->tick(ctx);  // b, c miss 2 == miss_grace, not yet; a is present again (idle)
  gate.update(snapshot_, 4);
  det->tick(ctx);  // b, c miss 3 > miss_grace: genuinely, individually dead - the departed
                   // count (2) no longer exceeds the cap(2), so nothing is collapsed

  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/b"}));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/c"}));
  EXPECT_EQ(det->tracked_count_for_test(), 4u) << "anchor and a (idle), b and c (departed) - none collapsed";
}

TEST_F(NodeDeathIntegrationTest, CapCollapsedDeathClearsOnceTheGraphRecoversAndTheDetectorIsReconfigured) {
  // A cap-forced collapse is a confirmed death that can no longer be named individually - it
  // still keeps GRAPH_NODE_DISAPPEARED raised, by design (collapsed_dead_count_ is monotone
  // within one tracker lifetime: the identities are gone, so there is no way to tell which of
  // possibly-several collapsed departures came back). What was untested is whether such a
  // fault can EVER clear again once the graph genuinely recovers - proven here through the
  // same reconfigure path ReconfigureWhileAbsentDoesNotWithholdAnEarnedClearOnceTheNodeReturns
  // already proves for an ordinary (uncollapsed) outstanding fault: a live reconfigure rebuilds
  // tracker_ (and so collapsed_dead_count_) from scratch while preserving ever_raised_, so a
  // clean re-observation of an all-present graph clears it.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}, {"tracked_node_cap", 1}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("a"), app_of("b"), anchor_app()});
  gate.update(snapshot_, 1);
  det->tick(ctx);  // arms a, b

  set_apps({anchor_app()});  // a, b both depart
  gate.update(snapshot_, 2);
  det->tick(ctx);  // both miss 1 > miss_grace(0): matured; departed count(2) > cap(1): collapsed

  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(det->tracked_count_for_test(), 1u)
      << "precondition: both identities collapsed - only the anchor (idle, uncapped) remains";
  ASSERT_TRUE(any_failed_desc_contains(kGraphSource, {"more node(s) disappeared"}))
      << "precondition: the collapse must itself have been reported as a genuine raise";

  set_apps({app_of("a"), app_of("b"), anchor_app()});  // the graph recovers: both return
  gate.update(snapshot_, 3);
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "merely returning must NOT by itself clear a collapsed death - collapsed_dead_count_ "
         "stays raised until an operator actually acts on the cap-saturation warning";

  // The operator's actual remedy: reconfigure (raising tracked_node_cap, say - the cap value
  // itself does not matter here, only that configure() rebuilds tracker_ from scratch).
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}, {"tracked_node_cap", 16}});
  gate.update(snapshot_, 4);  // a, b are present in this same, unchanged snapshot
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "once reconfigured, a clean re-observation of an all-present graph must clear the "
         "fault - ever_raised_ survives reconfigure (see ReconfigureWhileAbsentDoesNot... "
         "above) precisely so this recovery path is not permanently stuck";
}

// =============================================================================================
// C1: config sweep - miss_grace / prune_grace endpoints and degenerate values,
// tick_interval_ms edge values, the prune_grace clamp, and the 3000ms floor's own boundary.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, C1_MissGraceAcceptsTheLowerBoundZero) {
  // tick_interval_ms=3000 -> floor is 0, so miss_grace=0 needs no bump.
  EXPECT_FALSE(configure_warns({{"tick_interval_ms", 3000}, {"miss_grace", 0}}, "'miss_grace'"));
}

TEST_F(NodeDeathIntegrationTest, C1_MissGraceAcceptsTheUpperBoundThirtySixHundred) {
  EXPECT_FALSE(configure_warns({{"tick_interval_ms", 3000}, {"miss_grace", 3600}}, "'miss_grace' must"));
}

TEST_F(NodeDeathIntegrationTest, C1_MissGraceRejectsNegativeOne) {
  EXPECT_TRUE(configure_warns({{"miss_grace", -1}}, "'miss_grace' must be an integer"));
}

TEST_F(NodeDeathIntegrationTest, C1_MissGraceRejectsOnePastTheUpperBound) {
  EXPECT_TRUE(configure_warns({{"miss_grace", 3601}}, "'miss_grace' must be an integer"));
}

TEST_F(NodeDeathIntegrationTest, C1_PruneGraceAcceptsTheLowerBoundZero) {
  EXPECT_FALSE(configure_warns({{"prune_grace", 0}}, "'prune_grace' must"));
}

TEST_F(NodeDeathIntegrationTest, C1_PruneGraceAcceptsTheUpperBoundThirtySixHundred) {
  EXPECT_FALSE(configure_warns({{"prune_grace", 3600}}, "'prune_grace' must"));
}

TEST_F(NodeDeathIntegrationTest, C1_PruneGraceRejectsNegativeOne) {
  EXPECT_TRUE(configure_warns({{"prune_grace", -1}}, "'prune_grace' must be an integer"));
}

TEST_F(NodeDeathIntegrationTest, C1_PruneGraceRejectsOnePastTheUpperBound) {
  EXPECT_TRUE(configure_warns({{"prune_grace", 3601}}, "'prune_grace' must be an integer"));
}

TEST_F(NodeDeathIntegrationTest, C1_TickIntervalMsZeroIsRejectedAndWarns) {
  EXPECT_TRUE(configure_warns({{"tick_interval_ms", 0}}, "'tick_interval_ms'"));
}

TEST_F(NodeDeathIntegrationTest, C1_TickIntervalMsNegativeIsRejectedAndWarns) {
  EXPECT_TRUE(configure_warns({{"tick_interval_ms", -200}}, "'tick_interval_ms'"));
}

TEST_F(NodeDeathIntegrationTest, C1_TickIntervalMsOneIsAcceptedAndFloorsMissGraceHeavily) {
  // At a 1ms tick the floor is (3000+1-1)/1 - 1 = 2999 ticks - the default miss_grace(2) is
  // nowhere close, so this must warn about the bump rather than accept the default quietly.
  EXPECT_TRUE(configure_warns({{"tick_interval_ms", 1}}, "'miss_grace'"));
}

TEST(MinNodeDeathMissGrace, DoesNotOverflowAtTickIntervalMsDocumentedValidCeiling) {
  // tick_interval_ms's own documented-valid ceiling (node_death_detector.cpp's configure():
  // "must be a positive integer up to INT_MAX") - kMinNodeDeathWindowMs + tick_interval_ms
  // overflows a 32-bit int here before the division ever runs unless computed in a wider
  // type, undefined behaviour at a value the config contract explicitly accepts. The
  // returned floor must still be small and non-negative, not a wraparound artifact.
  const int floor = ros2_medkit_graph_watchdog::min_node_death_miss_grace(std::numeric_limits<int>::max());
  EXPECT_GE(floor, 0);
  EXPECT_LT(floor, 10) << "at a multi-billion-ms tick, kMinNodeDeathWindowMs is already spanned "
                          "by a single tick, so the floor must be tiny";
}

TEST_F(NodeDeathIntegrationTest, C1_MissGraceExactlyAtTheFloorIsAcceptedWithoutWarning) {
  // At the 1000ms default tick the floor is (3000+999)/1000 - 1 = 2, exactly the default -
  // the row either side of the floor this case and the next one pin.
  EXPECT_FALSE(configure_warns({{"tick_interval_ms", 1000}, {"miss_grace", 2}}, "'miss_grace'"));
}

TEST_F(NodeDeathIntegrationTest, C1_MissGraceOneBelowTheFloorWarnsAndIsBumped) {
  EXPECT_TRUE(configure_warns({{"tick_interval_ms", 1000}, {"miss_grace", 1}}, "'miss_grace'"));
}

TEST_F(NodeDeathIntegrationTest, C1_PruneGraceBelowMissGracePlusOneIsSilentlyClampedUp) {
  // The clamp is not a rejection (no warning is owed for it - prune_grace=0 is itself a
  // perfectly legal value), so this is a BEHAVIOURAL proof, in two halves: an allowlisted,
  // durably dead key must not be reclaimed before miss_grace + 1 = 3 consecutive suppressed
  // ticks, even though prune_grace=0 was configured (which would reclaim after just 1 if
  // unclamped) - AND it must actually BE reclaimed once that clamped horizon passes, or this
  // row could not tell the claimed clamp from prune() being broken entirely (neither would
  // move tracked_count off 1 at the first-eligible tick either).
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000},
                  {"miss_grace", 2},
                  {"prune_grace", 0},
                  {"allowlist", nlohmann::json::array({"/x"})},
                  {"suppress", nlohmann::json::array({"allowlist"})}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("x")});
  gate.update(snapshot_, 1);
  det->tick(ctx);  // arm
  set_apps({});
  for (std::uint64_t t = 2; t <= 4; ++t) {  // misses 1, 2, 3 (dead once misses > 2)
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  ASSERT_EQ(det->tracked_count_for_test(), 1u) << "the clamp must not have reclaimed it yet "
                                                  "at the very tick it first became eligible";

  // prune_ticks = max(prune_grace=0, miss_grace(2)+1=3) = 3. Suppression is first evaluated
  // at t=4 (streak 1); reclaim once the streak exceeds 3, i.e. streak 4 at t=7. Ticked to 9
  // for margin.
  for (std::uint64_t t = 5; t <= 9; ++t) {
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 0u)
      << "past the clamped horizon the durably-suppressed key must actually be reclaimed - a "
         "clamp that silently became 'never prune' would leave this at 1 forever";
}

// =============================================================================================
// X2: the 3000ms floor makes two otherwise-identical configurations behave DIFFERENTLY, not
// merely both eventually raise.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, X2_TheFloorMakesAConfiguredMissGraceBehaveDifferentlyFromOneAboveIt) {
  const int tick_interval_ms = 200;  // floor here is (3000+199)/200 - 1 = 14 ticks

  auto det_below = make_node_death();
  ASSERT_TRUE(det_below);
  det_below->configure({{"tick_interval_ms", tick_interval_ms}, {"miss_grace", 1}});  // bumped to 14

  auto det_above = make_node_death();
  ASSERT_TRUE(det_above);
  det_above->configure({{"tick_interval_ms", tick_interval_ms}, {"miss_grace", 20}});  // already above 14

  ReliabilityGate gate_below(0, gateway_.get(), &node_mutex_);
  ReliabilityGate gate_above(0, gateway_.get(), &node_mutex_);
  IntrospectionInput snap_below, snap_above;

  DetectorContext ctx_below;
  ctx_below.gateway_node = gateway_.get();
  ctx_below.node_mutex = &node_mutex_;
  ctx_below.mode = DetectorMode::Raise;
  ctx_below.gate = &gate_below;
  ctx_below.fault_client = client_;
  ctx_below.snapshot = &snap_below;
  DetectorContext ctx_above = ctx_below;
  ctx_above.gate = &gate_above;
  ctx_above.snapshot = &snap_above;

  snap_below.apps = {app_of("floor_below_node"), anchor_app()};
  snap_above.apps = {app_of("floor_above_node"), anchor_app()};
  gate_below.update(snap_below, 1);
  det_below->tick(ctx_below);
  gate_above.update(snap_above, 1);
  det_above->tick(ctx_above);

  snap_below.apps = {anchor_app()};
  snap_above.apps = {anchor_app()};

  int below_raised_at = -1;
  int above_raised_at = -1;
  for (int tick = 2; tick <= 25 && (below_raised_at < 0 || above_raised_at < 0); ++tick) {
    gate_below.update(snap_below, static_cast<std::uint64_t>(tick));
    det_below->tick(ctx_below);
    gate_above.update(snap_above, static_cast<std::uint64_t>(tick));
    det_above->tick(ctx_above);
    std::this_thread::sleep_for(5ms);
    if (below_raised_at < 0 && any_failed_desc_contains(kGraphSource, {"floor_below_node"})) {
      below_raised_at = tick;
    }
    if (above_raised_at < 0 && any_failed_desc_contains(kGraphSource, {"floor_above_node"})) {
      above_raised_at = tick;
    }
  }

  ASSERT_GT(below_raised_at, 0) << "the floor-bumped detector never reported the death at all";
  ASSERT_GT(above_raised_at, 0) << "the already-above-floor detector never reported the death at all";
  // Absent from tick 2 onward: reported once misses (tick - 1) exceeds the EFFECTIVE
  // miss_grace, i.e. at tick = effective_miss_grace + 2.
  EXPECT_EQ(below_raised_at, 16) << "the floor must have raised the effective miss_grace to "
                                    "14, not left it at the configured 1";
  EXPECT_EQ(above_raised_at, 22) << "20 is already above the floor and must be used as-is";
  EXPECT_NE(below_raised_at, above_raised_at)
      << "the two configurations must behave DIFFERENTLY, not merely both eventually raise";
}

// =============================================================================================
// C2: malformed allowlist/suppress warn; an allowlist that `suppress` does not name has no
// effect and says so at WARN severity - naming an entry on the allowlist is not, by itself,
// a request to suppress it.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, C2_AllowlistWrongTopLevelTypeWarns) {
  EXPECT_TRUE(configure_warns({{"allowlist", "not-an-array"}}, "'allowlist' must be an array"));
}

TEST_F(NodeDeathIntegrationTest, C2_AllowlistBadElementWarns) {
  EXPECT_TRUE(configure_warns({{"allowlist", nlohmann::json::array({42, "/ok"})}}, "'allowlist' entries must"));
}

TEST_F(NodeDeathIntegrationTest, C2_SuppressWrongTopLevelTypeWarns) {
  EXPECT_TRUE(configure_warns({{"suppress", "not-an-array"}}, "'suppress' must be an array"));
}

TEST_F(NodeDeathIntegrationTest, C2_SuppressBadElementWarns) {
  EXPECT_TRUE(configure_warns({{"suppress", nlohmann::json::array({42, "allowlist"})}}, "'suppress' entries must"));
}

TEST_F(NodeDeathIntegrationTest, C2_UnknownSuppressEntryWarnsNamingIt) {
  EXPECT_TRUE(configure_warns({{"suppress", nlohmann::json::array({"bogus_mechanism"})}}, "bogus_mechanism"));
}

TEST_F(NodeDeathIntegrationTest, C2_AllowlistNotNamedInSuppressWarnsAndDoesNotSuppress) {
  EXPECT_TRUE(configure_warns({{"allowlist", nlohmann::json::array({"/x"})}}, "inert"));

  // Behavioural half of the above: an allowlist entry that `suppress` does not name must
  // never actually suppress, whatever the ported source did.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}, {"allowlist", nlohmann::json::array({"/x"})}});
  auto ctx = make_ctx(&gate);
  set_apps({app_of("x"), anchor_app()});
  gate.update(snapshot_, 1);
  det->tick(ctx);
  set_apps({anchor_app()});
  for (std::uint64_t t = 2; t <= 3; ++t) {
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  std::this_thread::sleep_for(100ms);
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/x"}))
      << "an allowlist that suppress does not name must not suppress by itself";
}

TEST_F(NodeDeathIntegrationTest, C2_TrackedNodeCapBelowRangeWarnsAndKeepsTheDefault) {
  EXPECT_TRUE(configure_warns({{"tracked_node_cap", 0}}, "'tracked_node_cap' must be an integer in 1.."));
}

TEST_F(NodeDeathIntegrationTest, C2_TrackedNodeCapAboveRangeWarnsAndKeepsTheDefault) {
  EXPECT_TRUE(configure_warns({{"tracked_node_cap", 16385}}, "'tracked_node_cap' must be an integer in 1.."));
}

TEST_F(NodeDeathIntegrationTest, C2_TrackedNodeCapFarAboveIntMaxIsRejectedNotWrapped) {
  // The wide-then-validate pattern every other numeric key here uses: get<int>() on this
  // value would truncate it to 0 before any range check ever ran, and 0 would then read as
  // "below range" only by accident - the real risk is a value that truncates to something
  // INSIDE 1..16384 and is silently accepted as a completely different cap than what was
  // configured.
  EXPECT_TRUE(configure_warns({{"tracked_node_cap", 4294967296LL}}, "'tracked_node_cap' must be an integer in 1.."));
}

// =============================================================================================
// C3: an unknown config key warns naming it.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, C3_UnknownConfigKeyWarnsNamingIt) {
  EXPECT_TRUE(configure_warns({{"miz_grace", 5}}, "miz_grace"));
}

// =============================================================================================
// The plugin's GraphWatchdogPlugin::compute_departed_retention_ticks() sizes the gate's
// departed-lifecycle retention window for exactly this pair of properties. Both tests
// construct the gate with the SAME value that function would compute for this config (a
// 200ms tick with an unconfigured, floor-bumped miss_grace and prune_grace=3), proving the
// formula is sufficient rather than merely asserting it in a comment.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, CleanShutdownDepartureAtMissGraceBoundaryIsSuppressed) {
  const int tick_interval_ms = 200;
  const int floored_miss_grace = 14;  // detector_config_keys::min_node_death_miss_grace(200)
  const int prune_ticks = 15;         // max(prune_grace=3, floored_miss_grace+1=15)
  const int retention_ticks = prune_ticks + floored_miss_grace + 1;  // 30, the plugin's own formula

  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_, retention_ticks);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure(
      {{"tick_interval_ms", tick_interval_ms}, {"prune_grace", 3}, {"suppress", nlohmann::json::array({"lifecycle"})}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("clean"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms it (present, warmup_cycles=0)
  gate.set_departed_lifecycle_state_for_test("/clean", "finalized", /*saw_transition=*/true,
                                             /*error_terminated=*/false);
  set_apps({anchor_app()});

  // Absent from tick 1: dead once misses(tick) > 14, i.e. at tick 15 - the boundary where
  // node_death evaluates suppression against this key for the very first time.
  for (std::uint64_t t = 1; t <= 15; ++t) {
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a clean departure must not be reported even at the very first tick suppression is "
         "evaluated for it";
}

TEST_F(NodeDeathIntegrationTest, CleanShutdownDepartureIsReclaimedNotReRaisedPastRetention) {
  const int tick_interval_ms = 200;
  const int floored_miss_grace = 14;
  const int prune_ticks = 15;
  const int retention_ticks = prune_ticks + floored_miss_grace + 1;  // 30

  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_, retention_ticks);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure(
      {{"tick_interval_ms", tick_interval_ms}, {"prune_grace", 3}, {"suppress", nlohmann::json::array({"lifecycle"})}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("clean"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);
  gate.set_departed_lifecycle_state_for_test("/clean", "finalized", /*saw_transition=*/true,
                                             /*error_terminated=*/false);
  set_apps({anchor_app()});

  // Boundary at tick 15 (streak starts there), reclaimed once the streak exceeds
  // prune_ticks(15), i.e. streak 16 at tick 15+15=30. Ticked to 35 for margin.
  for (std::uint64_t t = 1; t <= 35; ++t) {
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  std::this_thread::sleep_for(50ms);
  // 1, not 0: anchor_app() stays present and armed for the whole run and is never pruned -
  // only "/clean" is ever eligible for reclaim.
  EXPECT_EQ(det->tracked_count_for_test(), 1u)
      << "a permanently clean departure must be RECLAIMED (forgotten), not merely stay "
         "suppressed forever";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the departed-lifecycle label must survive at least until the reclaim tick - if the "
         "gate's retention window under-runs node_death's own reclaim tick, the label expires "
         "first and this cleanly-shut-down node gets re-raised instead of silently reclaimed";
}

// =============================================================================================
// AllowlistSuppressor's id form: a bare-name collision gives two nodes the same leaf, so an
// allowlist entry written in the collision-prefixed id shape must suppress only the node that
// id actually names - never its bare-name sibling, which is reachable only through the exact
// FQN or bare-leaf forms proven directly on AllowlistSuppressor in test_allowlist_suppressor.cpp.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, AllowlistIdFormSuppressesOnlyTheCollisionPrefixedNode) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  // The id form specifically: neither node's bare leaf ("calibration") nor either full fqn
  // is on this list, so a suppression must be reaching the durable, id-shaped entry via
  // AllowlistSuppressor::allows() - the id captured while each node was still present (see
  // node_death_detector.cpp's tick()) - not via suppresses()'s own exact/bare-leaf checks.
  det->configure({{"tick_interval_ms", 3000},  // floor(3000ms) == 0, so miss_grace=0 is legal
                  {"miss_grace", 0},
                  {"allowlist", nlohmann::json::array({"coll_a_calibration"})},
                  {"suppress", nlohmann::json::array({"allowlist"})}});
  auto ctx = make_ctx(&gate);

  // Two nodes sharing the bare leaf "calibration" in different namespaces - the shape
  // ros2_runtime_introspection.cpp's own collision rule produces: same-bare-name nodes each
  // get a namespace-prefixed id, while their fqn (namespace + bare leaf) stays exactly what
  // it always was. Built by hand rather than via app_of(), which only ever derives bound_fqn
  // from id itself and so cannot produce an id that DIFFERS from the fqn's own leaf.
  App node_a;
  node_a.id = "coll_a_calibration";
  node_a.bound_fqn = "/coll_a/calibration";
  node_a.is_online = true;
  node_a.source = "runtime";
  App node_b;
  node_b.id = "coll_b_calibration";
  node_b.bound_fqn = "/coll_b/calibration";
  node_b.is_online = true;
  node_b.source = "runtime";

  set_apps({node_a, node_b, anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms both (present, warmup_cycles=0) and captures each one's id verdict

  set_apps({anchor_app()});  // both depart on the same tick
  gate.update(snapshot_, 1);
  det->tick(ctx);  // misses(1) > miss_grace(0): both would be dead absent suppression

  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, {"/coll_b/calibration"}))
      << "the sibling the allowlist entry does not name must still be reported dead - "
         "otherwise this test cannot tell a working id-match from a suppressor that "
         "swallows everything";
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, {"/coll_a/calibration"}))
      << "an allowlist entry naming the collision-prefixed id must suppress exactly the node "
         "that id names";
}

// =============================================================================================
// The ungated-clear guard (ever_raised_): ctx.clear_fault() carries no reliability-gate check
// of its own, so this detector may only clear GRAPH_NODE_DISAPPEARED once it has itself
// genuinely raised it at least once - never merely because SOMETHING, anything, is tracked.
// =============================================================================================

TEST_F(NodeDeathIntegrationTest, UngatedClearStaysSuppressedWhileAnUnrelatedNodeArmsAlone) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});  // floor(3000ms) == 0
  auto ctx = make_ctx(&gate);

  // "other" arms and stays present for every tick - it never dies, so report.dead is never
  // non-empty because of it. It stands in for anything elsewhere in the graph (the gateway's
  // own App among real candidates) that becomes tracked without being what a stored,
  // outstanding fault is actually about - tracker_.tracked_count() going non-zero here must
  // not be read as "safe to clear".
  for (std::uint64_t t = 0; t < 10; ++t) {
    set_apps({app_of("other"), anchor_app()});
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  ASSERT_GT(det->tracked_count_for_test(), 0u)
      << "precondition this test needs: something IS tracked, without this detector having "
         "ever raised anything";

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "a fresh instance that has never itself raised GRAPH_NODE_DISAPPEARED must never "
         "clear it either, however many unrelated entities become tracked";
}

// The fourth quadrant of the restart matrix, and a LIMITATION rather than a property worth
// having: gateway restarts AND the node returns. The fresh instance sees the node present,
// armed and not missing, so report.dead is empty, ever_raised_ stays false, and no PASSED is
// ever emitted - a fault CONFIRMED before the restart therefore stays CONFIRMED, and with the
// sqlite backend it survives every reboot until an operator acknowledges it.
//
// Why it is not simply fixed here: the guard exists because a fresh instance cannot tell "the
// node came back" from "I never saw that node". Answering that needs the KEYS the outstanding
// record names, and they are not reachable. /fault_manager/list_faults would tell this detector
// that a GRAPH_NODE_DISAPPEARED record is outstanding and that it is among its reporting
// sources, but not which nodes it names: fault_manager aggregates one record per fault_code,
// and this detector merges every dead key into that one record's description, capped at
// AggregatedFault::kMaxDescriptionChars with the remainder collapsed into a count. Acting on
// the record's mere existence would clear it for a node that is still dead and simply never
// re-observed, which is the ungated heal the sibling tests above forbid.
//
// So this pins the behaviour rather than asserting a fix: it is what the README's restart
// section documents, and it must not change silently.
TEST_F(NodeDeathIntegrationTest, RestartedInstanceNeverClearsAFaultItDidNotRaiseEvenAsTheNodeReturns) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  // A brand-new detector object is exactly what a gateway restart produces - see configure()'s
  // own note on why ever_raised_ needs no reset. Nothing here ever kills "victim": this
  // instance's whole life is the healthy graph a returned node presents.
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  for (std::uint64_t t = 0; t < 20; ++t) {
    set_apps({app_of("victim"), anchor_app()});
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  ASSERT_GT(det->tracked_count_for_test(), 0u)
      << "the returned node must be tracked and healthy, or this test is not about the quadrant "
         "where the node came back";

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "a fresh instance emitted PASSED for a fault it never raised - it cannot know which "
         "node the stored record names, so this clear would be just as wrong for a node that "
         "never came back";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "nothing died here, so nothing may be reported either";
}

// What re-admits a key the detector has just handed back, and why the handover does not hold
// without this: a dying managed node loses its lifecycle SERVICES from the snapshot, and a node
// with no managed record reads as a plain node - which is kEarned. So the very act of dying
// erases the measurement that disowned the node, and the key walks straight back in on the tick
// before it departs.
//
// The sequence is the production one, driven here where it can be made exact: provisional
// admission, a late non-active label, the release, then a sweep that still carries the App but
// no longer carries its get_state service, then the departure.
TEST_F(NodeDeathIntegrationTest, AReleasedKeyIsNotReAdmittedWhenItsLifecycleRecordVanishes) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});  // floor(3000ms) == 0
  auto ctx = make_ctx(&gate);

  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 0);
  // Injected AFTER the update: a service-less app is dropped from the watcher by every update,
  // so an entry staged before it would not survive to be read.
  gate.set_lifecycle_state_for_test("victim", "", 0);
  ASSERT_EQ(gate.presence_ownership("victim"), PresenceOwnership::kProvisional)
      << "the key has to be admitted on PROVISIONAL grounds, or there is nothing here to hand "
         "back and the sequence below tests nothing";
  det->tick(ctx);  // admits it

  gate.set_lifecycle_state_for_test("victim", "inactive", 0);
  ASSERT_EQ(gate.presence_ownership("victim"), PresenceOwnership::kDisowned);
  det->tick(ctx);  // hands it back

  // The node begins to die. Its services leave the snapshot while the App is still in it, so
  // the watcher drops the record - and the gate, seeing no lifecycle at all, answers kEarned.
  gate.update(snapshot_, 1);
  ASSERT_FALSE(gate.lifecycle_state_of("victim").has_value())
      << "the watcher must have dropped the record here, or this test is not driving the sweep "
         "that erases the disowning measurement";
  ASSERT_TRUE(gate.allows_raise("victim"));
  det->tick(ctx);

  set_apps({anchor_app()});  // and now the App itself goes
  gate.update(snapshot_, 2);
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // misses(1) > miss_grace(0)

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the key was handed back to lifecycle_expectation and then reported dead anyway - "
         "losing a lifecycle record is not a measurement that the node became this detector's, "
         "it is the loss of the one that said it was not";
}

TEST_F(NodeDeathIntegrationTest, ClearFlowsOnceThisInstanceHasGenuinelyRaised) {
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms "victim"

  set_apps({anchor_app()});  // victim departs
  gate.update(snapshot_, 1);
  det->tick(ctx);  // misses(1) > miss_grace(0): genuinely dead, genuinely raised

  std::this_thread::sleep_for(50ms);
  ASSERT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "precondition: this instance must have genuinely raised before the recovery below "
         "can prove anything about clearing";

  set_apps({app_of("victim"), anchor_app()});  // victim returns
  gate.update(snapshot_, 2);
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "a genuine recovery after this instance has itself raised must still clear - the "
         "ungated-clear guard must suppress an UNEARNED clear, not an earned one";
}

TEST_F(NodeDeathIntegrationTest, EverRaisedTracksDeliveryNotIntentSoAnUndeliveredRaiseNeverEarnsAClear) {
  // ever_raised_ must be set from emit_ordered()'s own return (did raise_fault() actually
  // call async_send_request()), not from report.dead's mere non-emptiness. Advisory mode is
  // the simplest of several ways raise_fault() can decline to send despite a non-empty
  // report (mode_emits, no client, empty source_id, the reliability gate, and a
  // not-yet-ready service all take the identical silent-decline path) - any one of them
  // proves the same gap, since ever_raised_ has no way to tell them apart from a genuine
  // send without reading raise_fault()'s own return value.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});
  auto ctx = make_ctx(&gate);
  ctx.mode = DetectorMode::Advisory;  // every raise_fault()/clear_fault() call declines to send

  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms "victim"

  set_apps({anchor_app()});  // victim departs
  gate.update(snapshot_, 1);
  det->tick(ctx);  // report.dead is non-empty, but Advisory mode declines to send it

  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "precondition: Advisory mode must have genuinely suppressed the raise";

  // Switch to Raise mode and let victim return. If ever_raised_ had already been set from
  // report.dead's mere non-emptiness on the tick above (the bug this test pins), it would
  // read true here and this recovery would emit an unearned PASSED for an occurrence the
  // fault manager never heard about in the first place.
  ctx.mode = DetectorMode::Raise;
  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 2);
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "no FAILED for this occurrence ever reached the wire, so no PASSED may either";
}

TEST_F(NodeDeathIntegrationTest, AdvisoryModeStillObservesAndAccumulatesMissesWithoutEmittingAnything) {
  // README's own contract: "advisory = observe, do not push". The sibling test above proves
  // the "do not push" half; this proves the "observe" half is not a no-op masquerading as
  // one - a detector that skipped tracker_.update() entirely in Advisory mode would ALSO
  // send nothing, and would be indistinguishable from a correct one by that test alone.
  //
  // Proof: accumulate misses PAST miss_grace while in Advisory (so the node is genuinely
  // dead from the tracker's own point of view, just never reported), then switch to Raise
  // mode WITHOUT letting the node return. A detector that truly kept observing raises on
  // this very first Raise-mode tick, because the miss count already crossed the threshold
  // during the Advisory window; one that had paused observation would need miss_grace + 1
  // MORE ticks from here to reach the same threshold from zero.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 2}});
  auto ctx = make_ctx(&gate);
  ctx.mode = DetectorMode::Advisory;

  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms "victim"

  set_apps({anchor_app()});                 // victim departs
  for (std::uint64_t t = 1; t <= 3; ++t) {  // misses 1, 2, 3 (> miss_grace(2)): genuinely dead
    gate.update(snapshot_, t);
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 2u)
      << "Advisory mode must still track victim and anchor exactly as Raise mode would";

  std::this_thread::sleep_for(50ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "precondition: Advisory mode must have genuinely suppressed the raise so far";

  ctx.mode = DetectorMode::Raise;
  gate.update(snapshot_, 4);  // victim still absent - no recovery here
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the very first Raise-mode tick must already raise, proving the miss count had "
         "already crossed miss_grace DURING the Advisory window rather than starting fresh";
}

TEST_F(NodeDeathIntegrationTest, ReconfigureWhileAbsentDoesNotWithholdAnEarnedClearOnceTheNodeReturns) {
  // ever_raised_ is a fact about this PROCESS's history, not about the currently-loaded
  // config - configure() rebuilds tracker_ fresh (an operator editing the allowlist, say,
  // with a death still outstanding and absent), and if ever_raised_ reset along with it, the
  // freshly-rebuilt tracker could never re-create evidence for a node it has not itself
  // re-observed, leaving the standing fault able to get neither a further FAILED nor a
  // PASSED for the rest of the process's life once the node genuinely returns.
  ReliabilityGate gate(/*warmup_cycles=*/0, gateway_.get(), &node_mutex_);
  auto det = make_node_death();
  ASSERT_TRUE(det);
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});
  auto ctx = make_ctx(&gate);

  set_apps({app_of("victim"), anchor_app()});
  gate.update(snapshot_, 0);
  det->tick(ctx);  // arms "victim"

  set_apps({anchor_app()});  // victim departs
  gate.update(snapshot_, 1);
  det->tick(ctx);  // genuinely raised

  std::this_thread::sleep_for(50ms);
  ASSERT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "precondition: this instance must have genuinely raised before the reconfigure below";

  // Reconfigure WHILE the node is still absent - tracker_ is rebuilt empty, and this
  // (otherwise identical) config has never itself observed "victim" at all.
  det->configure({{"tick_interval_ms", 3000}, {"miss_grace", 0}});

  set_apps({app_of("victim"), anchor_app()});  // victim returns
  gate.update(snapshot_, 2);
  det->tick(ctx);

  std::this_thread::sleep_for(50ms);
  EXPECT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), 0u)
      << "a genuine recovery after a live reconfigure must still clear - the standing fault "
         "must not be stuck unable to ever heal for the rest of the process's life";
}
