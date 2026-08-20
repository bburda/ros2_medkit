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
// Drives the REAL mechanism: the detector's own LifecycleExpectationTracker state, a
// snapshot-driven presence diff (via set_apps()), and a REAL ReliabilityGate to arm the
// global bringup grace and to feed the detector its lifecycle labels via
// ReliabilityGate::lifecycle_state_of() - raising/clearing through a real ReportFault
// service round-trip to a fake fault_manager. NOT a full-gateway e2e; this proves the
// detector end to end within its own scope (the e2e tier proves the real
// lifecycle-transition pipeline).
//
// Two ordering rules are load-bearing here, or the test silently proves nothing:
//   1. Arm the global gate FIRST. The aggregated raise goes out under
//      source_id="graph_watchdog", unknown to the WarmupTracker, so allows_raise() takes
//      the global-bringup path and stays suppressed until a non-empty gate.update() has
//      run and the global grace has elapsed: gate.update(snapshot, warmup) then
//      gate.update(snapshot, warmup + 3).
//   2. Inject the lifecycle label AFTER the last gate.update(), and never call
//      gate.update() again. LifecycleWatcher::update() rebuilds its tracked set from
//      snapshot.apps SERVICES and ERASES any id it cannot find a get-state path for; the
//      fixture's apps carry no services, so any gate.update() after
//      set_lifecycle_state_for_test() wipes the injected label (state_of() then returns
//      nullopt = no expectation = no raise).
//
// Alongside the fixture, plain configure()-level TESTs pin the config contract (unknown
// keys, validation warnings, the prune clamp): this binary links the detector .cpp, so
// REGISTER_DETECTOR is in-image and the registry hands out instances without any ROS
// scaffolding.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"  // kDefaultAbsenceGrace
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::kDefaultAbsenceGrace;
using ros2_medkit_graph_watchdog::ReliabilityGate;
using ros2_medkit_graph_watchdog::graph_fault_codes::kNodeInactive;
using ros2_medkit_graph_watchdog::graph_fault_codes::kNodeNotManaged;
using ros2_medkit_graph_watchdog::graph_fault_codes::kNodeUnreadable;
using Fault = ros2_medkit_msgs::msg::Fault;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in lifecycle_expectation_detector.cpp but
// self-registers via REGISTER_DETECTOR, which runs when that .cpp is linked into this
// test. Pull an instance from the registry (no production factory needed).
std::unique_ptr<ros2_medkit_graph_watchdog::Detector> make_lifecycle_expectation() {
  for (auto & d : ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all()) {
    if (d->id() == "lifecycle_expectation") {
      return std::move(d);
    }
  }
  return nullptr;
}
// Aggregated fault source: no Component in the test snapshot, so graph_source_id()
// falls back to this literal (see aggregated_fault.hpp).
constexpr const char * kGraphSource = "graph_watchdog";
constexpr int kWarmupCycles = 3;
// Mirrors the detector's kUnmeasuredHoldTicks (file-local there by design): consecutive
// matched-but-never-read ticks a required node holds back GRAPH_NODE_INACTIVE's clear
// before that hold releases - either into silence (not-managed) or into its own
// GRAPH_NODE_UNREADABLE report (unreadable), with no further bound past this point.
constexpr int kHoldTicks = 60;

/// Captures rcutils log output for as long as it is alive and restores the console handler on every
/// exit path. Leaving the process-global handler installed would swallow the log output of every
/// later case in this binary, so the restore cannot be left to the success path.
///
/// One implementation for the whole file. The rcutils handler CONTRACT hands over a caller-supplied
/// format string and a va_list - it is not a choice the handler makes - so `vsnprintf` here is the
/// one place in this file that cannot use a literal format.
///
/// The handler is a plain C function pointer with no user-data slot, so the live capture is reached
/// through a file-static. It is written by the test thread and read by whichever thread logs,
/// hence the atomic.
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

  /// How many captured lines contain `needle`.
  int count(const std::string & needle) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(std::count_if(lines_.begin(), lines_.end(), [&needle](const std::string & line) {
      return line.find(needle) != std::string::npos;
    }));
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
    // The format string arrives from the logging call site through the handler signature, so there
    // is no literal to write here and no other way to reach the arguments than the va_list. The
    // build runs -Werror=format=2; GCC exempts va_list-taking formatters from -Wformat-nonliteral,
    // clang does not, so the CI clang-tidy job fails on this line alone. A pragma rather than a
    // lint-suppression comment, because the diagnostic is raised as an ERROR and clang-tidy does
    // not honour suppression comments for those. Scoped to the single call.
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

/// Adapts a body that wants the request and the response by reference to the shape rclcpp requires.
///
/// create_service deduces the callback by comparing its argument tuple against
/// `std::function<void(std::shared_ptr<Request>, std::shared_ptr<Response>)>` EXACTLY, so a handler
/// cannot declare those parameters by reference however little it needs a copy of them. Stating
/// that shape once here keeps it out of the fake service below (and keeps the by-value shared_ptr
/// copies out of clang-tidy's sight - see test_param_drift_integration.cpp's identical adapter).
template <typename ServiceT, typename BodyT>
std::function<void(std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
service_callback(BodyT body) {
  return [body = std::move(body)](std::shared_ptr<typename ServiceT::Request> req,
                                  std::shared_ptr<typename ServiceT::Response> resp) {
    body(*req, *resp);
  };
}

/// Snapshot builder for the configure()-level TESTs below (the fixture has its own).
IntrospectionInput snapshot_of(const std::vector<std::string> & ids) {
  IntrospectionInput snapshot;
  for (const auto & id : ids) {
    App a;
    a.id = id;
    a.bound_fqn = "/" + id;
    a.is_online = true;  // App defaults it to FALSE; every app here models a node that is running
    snapshot.apps.push_back(a);
  }
  return snapshot;
}

/// One app under an EXPLICIT (id, fqn), for the identity-churn sweep below: every
/// respawn arrives as a brand-new fqn under a namespace, matched by the same bare entry
/// through its leaf.
IntrospectionInput namespaced_snapshot_of(const std::string & ns, const std::string & leaf) {
  IntrospectionInput snapshot;
  App a;
  a.id = ns + "_" + leaf;
  a.bound_fqn = "/" + ns + "/" + leaf;
  a.is_online = true;  // see snapshot_of() - the default is FALSE, which models a different node
  snapshot.apps.push_back(a);
  return snapshot;
}

/// How many CONFIRMED-inactive "filler" nodes, all named `<prefix>0000`, `<prefix>0001`, ...
/// (fixed 4-digit zero-padded suffix, so every one is the SAME length), are needed to exceed
/// AggregatedFault::kMaxDescriptionChars on their own - i.e. the smallest batch that would
/// already fill the description cap without any help from a later-crossing node. Derived from
/// the REAL detail-building code via a throwaway tracker instance (not a hand-rebuilt formula):
/// this is what R11's ordering fix has to survive against, since a batch this size sorts before
/// any node whose id starts with a later letter and, before this slice, would have silently
/// evicted it forever. Fills `ids_out` with those ids and returns the count.
std::size_t fill_count_past_cap(const std::string & prefix, std::vector<std::string> & ids_out) {
  ros2_medkit_graph_watchdog::LifecycleExpectationTracker probe({prefix + "0000"}, /*grace=*/0);
  const std::string sample_fqn = "/" + prefix + "0000";
  auto probe_report =
      probe.update({ros2_medkit_graph_watchdog::LifecycleMatch{prefix + "0000", sample_fqn, std::string("inactive")}});
  const std::size_t entry_len = probe_report.affected.at(sample_fqn).size();
  constexpr std::size_t kJoinSep = 2;  // "; " - AggregatedFault::describe_ordered's join separator
  const std::size_t cap = ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars;
  std::size_t count = 1;
  while (count * entry_len + (count - 1) * kJoinSep <= cap) {
    ++count;
  }
  ids_out.clear();
  for (std::size_t i = 0; i < count; ++i) {
    std::string suffix = std::to_string(i);
    suffix.insert(0, 4 - std::min<std::size_t>(4, suffix.size()), '0');  // 4-digit zero pad
    ids_out.push_back(prefix + suffix);
  }
  return count;
}
}  // namespace

class LifecycleExpectationIntegrationTest : public ::testing::Test {
 protected:
  // Must run before the FIRST test's fixture is constructed - see the identical
  // rationale in test_param_drift_integration.cpp.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    gateway_ = std::make_shared<rclcpp::Node>("le_it_gateway");
    sink_ = std::make_shared<rclcpp::Node>("le_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        service_callback<ReportFault>([this](ReportFault::Request & req, ReportFault::Response & resp) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            received_.push_back(req);
          }
          resp.accepted = true;  // ReportFault.srv response field is `bool accepted`
        }));
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
    gateway_.reset();
  }

  DetectorContext make_ctx(DetectorMode mode, ReliabilityGate * gate) {
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.node_mutex = &node_mutex_;
    ctx.mode = mode;
    ctx.gate = gate;  // the detector reads ctx.gate->lifecycle_state_of() - must be real, not null
    ctx.fault_client = client_;
    ctx.snapshot = &snapshot_;  // tick() early-returns without a snapshot; each test seeds set_apps()
    return ctx;
  }

  // (Re)populate the owned entity snapshot the detector reads each tick - the
  // fixture's presence-diff seam. components stays empty, so the aggregated fault's
  // source_id falls back to the literal "graph_watchdog".
  void set_apps(const std::vector<std::string> & ids) {
    snapshot_.apps.clear();
    for (const auto & id : ids) {
      App a;
      a.id = id;
      a.bound_fqn = "/" + id;
      // App::is_online defaults to FALSE, which is the manifest-declared-but-not-running
      // shape - a node node_death skips entirely. Every app built here models a node that
      // IS running, and the detector's own `armed` fact depends on the difference, so it is
      // set explicitly rather than inherited. set_offline_app() stages the other one.
      a.is_online = true;
      snapshot_.apps.push_back(a);
    }
  }

  // The manifest shape the builders above deliberately do not produce: an app the gateway
  // carries with no running node behind it. node_death skips such an app outright, so
  // nothing about it can ever be reported by the presence class.
  void set_offline_app(const std::string & id) {
    snapshot_.apps.clear();
    App a;
    a.id = id;
    a.bound_fqn = "/" + id;
    a.is_online = false;
    snapshot_.apps.push_back(a);
  }

  // Seed apps with EXPLICIT (id, fqn) pairs - to reproduce App::id instability, where the
  // gateway gives a bare-name-colliding node a namespaced id while its fqn stays stable.
  void set_apps_raw(const std::vector<std::pair<std::string, std::string>> & id_fqns) {
    snapshot_.apps.clear();
    for (const auto & [id, fqn] : id_fqns) {
      App a;
      a.id = id;
      a.bound_fqn = fqn;
      a.is_online = true;  // see set_apps() for why this is explicit
      snapshot_.apps.push_back(a);
    }
  }

  // Seed the snapshot with ONE managed lifecycle app under an EXPLICIT (id, fqn): the
  // GetState + ChangeState services are what make the gate's internal lifecycle tracking
  // keep the id across gate.update() calls (the service-less apps above are erased by
  // every update - ordering rule 2). No live node answers at `fqn`, so seeds fail into
  // the unknown (empty) label.
  void set_managed_app(const std::string & id, const std::string & fqn) {
    snapshot_.apps.clear();
    ros2_medkit_gateway::ServiceInfo get_state;
    get_state.full_path = fqn + "/get_state";
    get_state.type = "lifecycle_msgs/srv/GetState";
    ros2_medkit_gateway::ServiceInfo change_state;
    change_state.full_path = fqn + "/change_state";
    change_state.type = "lifecycle_msgs/srv/ChangeState";
    App a;
    a.id = id;
    a.bound_fqn = fqn;
    a.services = {get_state, change_state};
    a.is_online = true;  // see set_apps() for why this is explicit
    snapshot_.apps.push_back(a);
  }

  // Arm the global bringup grace the aggregated "graph_watchdog" source needs (see the
  // file-level ordering-rule doc comment, rule 1). Must be the LAST gate.update() call
  // in a test before any set_lifecycle_state_for_test() injection (rule 2).
  void arm_global_grace(ReliabilityGate & gate) {
    gate.update(snapshot_, 5);
    gate.update(snapshot_, 5 + kWarmupCycles);
  }

  // How many matching faults are currently in the log (for absence checks + snapshots).
  // `code` defaults to kNodeInactive since most of this fixture's tests are about that
  // fault; a test exercising the sibling GRAPH_NODE_UNREADABLE record passes it explicitly.
  std::size_t count_faults(const std::string & source_id, uint8_t event_type,
                           const std::string & code = kNodeInactive) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == code && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  // EVERY request the fake fault_manager has received, of ANY source/code/event kind.
  // The zero-config claim is "the fault_manager hears nothing at all", so the instrument
  // has to count everything, not just the matching FAILED events.
  std::size_t count_all_requests() {
    std::lock_guard<std::mutex> lk(mtx_);
    return received_.size();
  }

  // Description of the LAST matching FAILED request (empty if none arrived).
  std::string last_failed_description(const std::string & source_id, const std::string & code = kNodeInactive) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::string desc;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == code && r.event_type == ReportFault::Request::EVENT_FAILED) {
        desc = r.description;
      }
    }
    return desc;
  }

  // True if any recorded FAILED for `source_id` carries a description mentioning `needle`.
  bool any_failed_desc_contains(const std::string & source_id, const std::string & needle,
                                const std::string & code = kNodeInactive) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id != source_id || r.fault_code != code || r.event_type != ReportFault::Request::EVENT_FAILED) {
        continue;
      }
      if (r.description.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  // Severity of the LAST matching FAILED request. nullopt if none arrived - a missing
  // raise must fail the assertion that reads this, not silently compare against 0.
  std::optional<uint8_t> last_failed_severity(const std::string & source_id, const std::string & code = kNodeInactive) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::optional<uint8_t> severity;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == code && r.event_type == ReportFault::Request::EVENT_FAILED) {
        severity = r.severity;
      }
    }
    return severity;
  }

  // Tick until a NEW matching fault appears beyond `baseline_count` (arrived AFTER the
  // trigger), or timeout. NEVER calls gate.update() - only the detector is ticked, per
  // ordering rule 2 (a further gate.update() would erase the injected lifecycle label).
  bool poll_for_new(const std::string & source_id, uint8_t event_type, std::size_t baseline_count,
                    ros2_medkit_graph_watchdog::Detector & det, DetectorContext & ctx,
                    const std::string & code = kNodeInactive) {
    for (int i = 0; i < 130; ++i) {
      det.tick(ctx);
      std::this_thread::sleep_for(50ms);
      if (count_faults(source_id, event_type, code) > baseline_count) {
        return true;
      }
    }
    return false;
  }

  // Wait (WITHOUT ticking) until at least `target` matching requests have arrived - for
  // the exactly-N assertions, where further ticks would legitimately emit more.
  bool wait_for_count(const std::string & source_id, uint8_t event_type, std::size_t target,
                      const std::string & code = kNodeInactive) {
    for (int i = 0; i < 150; ++i) {
      if (count_faults(source_id, event_type, code) >= target) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return count_faults(source_id, event_type, code) >= target;
  }

  rclcpp::Node::SharedPtr gateway_, sink_;
  rclcpp::Service<ReportFault>::SharedPtr srv_;
  rclcpp::Client<ReportFault>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread spin_;
  std::mutex mtx_, node_mutex_;
  std::vector<ReportFault::Request> received_;
  IntrospectionInput snapshot_;  // owned entity snapshot the detector reads each tick
};

// The whole point of splitting the unreadable case into its own fault code: a confirmed
// violation heals while an UNREADABLE sibling is present and STAYS present (never read,
// for the whole test). GRAPH_NODE_INACTIVE must clear on "a" healing regardless of what
// "b" is doing - and GRAPH_NODE_UNREADABLE, which is entirely about "b", must not care
// that "a" healed either. "b" is run well past its own hold first, so under the OLD
// shared-record design it would already be poisoning the one record both codes used to
// share; under this design it is its own fault and never touches GRAPH_NODE_INACTIVE at
// all once past the hold.
TEST_F(LifecycleExpectationIntegrationTest, ConfirmedNodeHealsWhileUnreadableSiblingStaysPresentClearsInactiveOnly) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // confirmed violation
  gate.set_lifecycle_state_for_test("b", "");          // unreadable, never read for the rest of the test
  ASSERT_TRUE(gate.lifecycle_state_of("b").has_value() && gate.lifecycle_state_of("b")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeInactive))
      << "\"a\" never raised, so nothing below can be attributed to it healing";

  // Run "b" well past its own hold, so it is already reported as its own
  // GRAPH_NODE_UNREADABLE fault before "a" heals.
  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeUnreadable))
      << "\"b\"'s unreadable hold never converted into its own record, so nothing below is tested";
  // Baseline captured AFTER "b" has already converted to content: from this point on,
  // GRAPH_NODE_UNREADABLE is raising every tick (never clearing) because "b" stays
  // present and unread - the level-triggered PASSED stream that ran during the hold,
  // before "b" had anything to report, is over.
  const auto unreadable_passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);

  // "a" heals. GRAPH_NODE_INACTIVE must clear PROMPTLY - not after "b" is eventually read,
  // and not after any content-window delay - because "b" no longer influences it at all.
  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeInactive);
  bool cleared = false;
  for (int i = 0; i < 20 && !cleared; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
    cleared = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeInactive) > passed_before;
  }
  EXPECT_TRUE(cleared) << "GRAPH_NODE_INACTIVE did not clear once its only confirmed violation healed, even "
                          "though the only other required node was merely unreadable, never confirmed";

  // GRAPH_NODE_UNREADABLE, meanwhile, is entirely unaffected by "a" healing: "b" is still
  // unread, so its own fault must still be raised and must not have cleared even once.
  EXPECT_GT(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "GRAPH_NODE_UNREADABLE never raised at all";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeUnreadable))
      << "GRAPH_NODE_UNREADABLE stopped naming \"b\" once the unrelated \"a\" healed - the two faults must "
         "be independent";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable), unreadable_passed_before)
      << "GRAPH_NODE_UNREADABLE cleared merely because a DIFFERENT required node (\"a\") healed";
}

// The OTHER clear direction, left untested by the test above: it proves an INACTIVE
// clear leaves UNREADABLE alone, not that an UNREADABLE clear leaves a concurrently
// raised INACTIVE alone. Wiring GRAPH_NODE_UNREADABLE's clear to also clear
// GRAPH_NODE_INACTIVE would pass every other test in this file today - nothing else
// reads a PASSED for kNodeInactive across a sibling's clear - since "a" here is never
// healed at all.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableSiblingClearingDoesNotTouchAConcurrentlyRaisedInactive) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // confirmed violation, NEVER healed in this test
  gate.set_lifecycle_state_for_test("b", "");          // unreadable, will be read below
  ASSERT_TRUE(gate.lifecycle_state_of("b").has_value() && gate.lifecycle_state_of("b")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeInactive))
      << "\"a\" never raised, so nothing below can be attributed to it being left alone";
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "\"b\"'s unreadable hold never converted into GRAPH_NODE_UNREADABLE, so nothing below "
         "isolates the clear direction under test";

  const auto inactive_passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeInactive);

  // "b" is genuinely read: GRAPH_NODE_UNREADABLE clears.
  gate.set_lifecycle_state_for_test("b", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx, kNodeUnreadable))
      << "\"b\" being read never cleared GRAPH_NODE_UNREADABLE, so nothing below proves anything";

  // GRAPH_NODE_INACTIVE, meanwhile, must be entirely undisturbed: "a" is still confirmed
  // inactive and was never touched.
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeInactive), inactive_passed_before)
      << "GRAPH_NODE_UNREADABLE clearing also cleared GRAPH_NODE_INACTIVE for an unrelated node - "
         "the two faults are not independent in this direction";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a", kNodeInactive))
      << "GRAPH_NODE_INACTIVE stopped naming \"a\" once the unrelated \"b\" cleared";
}

// Retitled honestly rather than left where its position in this file implied more than it
// checks: this pattern (a real "inactive" read alternating with unread ticks) passes
// unchanged against the design that preceded the two-clock model, because a real "inactive"
// read always resets the unmeasured clock and the violation streak alone accumulates across
// the unread gaps. So it pins the STREAK-ACCUMULATION rule - that only a genuine "active"
// read resets the streak - and nothing about the cause-blind unmeasured clock. The pairs
// that do discriminate the current model live in
// AlternatingUnreadableAndNotManagedRaisesOneOfTheTwoUnmeasuredCodes and
// InactiveAlternatingWithNotManagedAcrossAbsenceGapsRaisesInactive below.
TEST_F(LifecycleExpectationIntegrationTest, MeasuredInactiveSeparatedByUnreadTicksRaisesInactiveViaTheStreak) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  constexpr int kGrace = 5;
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Alternate: one genuine "inactive" read, then a run of unread ticks LONGER than
  // kDefaultAbsenceGrace (3, fixed in the tracker) - otherwise a single unread run alone
  // could never have spent the OLD absence-borrowing budget in one go, and this test
  // would not distinguish the fix from the bug. Each unread run still stays far under
  // GRAPH_NODE_UNREADABLE's own 60-tick hold (kHoldTicks), so this pattern can only ever
  // be reported through GRAPH_NODE_INACTIVE's streak accumulating across the gaps, never
  // through its sibling maturing. Neither det->tick() nor this loop ever calls
  // gate.update() (ordering rule 2), so each set_lifecycle_state_for_test() injection
  // below sticks until the next one overwrites it.
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive);
  bool raised = false;
  for (int cycle = 0; cycle < 20 && !raised; ++cycle) {
    gate.set_lifecycle_state_for_test("a", "inactive");
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
    gate.set_lifecycle_state_for_test("a", "");
    for (int i = 0; i < kDefaultAbsenceGrace + 2; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(5ms);
    }
    raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive) > failed_before;
  }
  EXPECT_TRUE(raised) << "a node alternating between measured-inactive and unreadable never raised "
                         "GRAPH_NODE_INACTIVE - the streak kept being erased by the intervening unread ticks, "
                         "which is exactly what left it invisible to both this code and GRAPH_NODE_UNREADABLE";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "GRAPH_NODE_UNREADABLE also fired for this pattern - the alternation was not short enough "
         "to isolate GRAPH_NODE_INACTIVE's streak-accumulation fix from its sibling's own hold";
}

// The alternation this slice's redesign closes, driven through the REAL gate rather than
// the injection seam (which can only ever SET a value, never remove tracking - the seam
// cannot produce a genuine nullopt for a previously-tracked fqn). Toggling whether "a"
// carries GetState/ChangeState services drives the two unmeasured causes directly:
// present -> LifecycleWatcher tracks it and seeds "" (UNREADABLE, no live responder);
// absent -> the tracked entry is dropped (NOT-MANAGED, lifecycle_state_of() -> nullopt).
// Before this redesign, this exact 80-tick alternation left BOTH GRAPH_NODE_UNREADABLE and
// (before this slice added it) whatever code NOT-MANAGED might have had permanently silent -
// watched directly against the pre-redesign detector, in the same shape as this test, before
// this slice began. Now the two causes share one cause-blind clock, so the alternation
// matures it regardless of which cause is live on any given tick.
TEST_F(LifecycleExpectationIntegrationTest, AlternatingUnreadableAndNotManagedRaisesOneOfTheTwoUnmeasuredCodes) {
  set_apps({});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  std::uint64_t tick_counter = 5000;
  bool raised = false;
  for (int cycle = 0; cycle < 40 && !raised; ++cycle) {
    // UNREADABLE leg: services present, no live GetState responder -> seeds "".
    set_managed_app("a", "/a");
    gate.update(snapshot_, ++tick_counter);
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);

    // NOT-MANAGED leg: services gone -> the tracked entry is dropped -> nullopt.
    set_apps_raw({{"a", "/a"}});
    gate.update(snapshot_, ++tick_counter);
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);

    raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable) > 0 ||
             count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged) > 0;
  }
  EXPECT_TRUE(raised) << "a node alternating between unreadable and not-managed never raised EITHER "
                         "unmeasured code, even past 80 total ticks - the shared clock this redesign "
                         "adds did not close the alternation it exists to close";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised for a node that was never confirmed non-active, only unmeasured";
}

// The full story: "a" present, armed via a REAL wired gate (arming the global bringup
// grace the aggregated "graph_watchdog" source needs), then injected as
// lifecycle-inactive (AFTER the last gate.update(), per ordering rule 2) - ticking the
// detector past grace raises GRAPH_NODE_INACTIVE naming "a"; driving "a" to "active"
// clears it.
TEST_F(LifecycleExpectationIntegrationTest, RequiredNodeStuckInactivePastGraceRaisesThenActiveClears) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Inject the label AFTER the last gate.update() (rule 2) - never call gate.update()
  // again from here on.
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "a"));

  // "a" reaches active -> the aggregate clears.
  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// Positive control: a required node that is active from the moment it is observed must
// never raise, however many ticks pass.
TEST_F(LifecycleExpectationIntegrationTest, RequiredNodeActiveFromArmingNeverRaises) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a required node reported active from the start was flagged GRAPH_NODE_INACTIVE";
}

// Absence control: once "a" vanishes from the snapshot entirely, GRAPH_NODE_INACTIVE must
// STAY raised. The operator declared the node must be active, the detector measured that it
// was not, and the node then leaving the graph answers nothing - so its clear must not flow.
// This is the deliberate behaviour change: the design this replaces cleared here, which is
// also what let a node that vanishes periodically shed the evidence against it. Absence is
// driven via set_apps({}), NEVER gate.update({}) (which would reset the gate's own
// graph_seen_ global-bringup marker, see the file-level ordering-rule doc comment).
TEST_F(LifecycleExpectationIntegrationTest, AbsenceAfterRaiseKeepsTheFaultAndSaysTheNodeIsGone) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // "a" vanishes from the graph entirely (a crash, not a lifecycle transition).
  set_apps({});
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kDefaultAbsenceGrace + 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "GRAPH_NODE_INACTIVE cleared once the required node vanished - the node was measured "
         "not-active and nothing has said otherwise since, so leaving the graph must not heal it";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "has since left the graph"))
      << "the fault kept describing a node as merely inactive after it left the graph, which sends "
         "the operator looking for a node that is not there";
}

// This slice's fix, at the tier that proves the detector's real OUTPUT rather than the
// tracker's internal report: a node the detector has already reported must not emit a
// spurious PASSED on its very first absent tick. Companion to
// AbsenceAfterRaiseClearsNotNodeDeathsDomain above (which proves the OPPOSITE and must
// stay true: sustained absence past the grace DOES clear) - here the node returns well
// before the absence grace elapses, so no PASSED may reach the fake service at any
// point during the blink.
TEST_F(LifecycleExpectationIntegrationTest, NoPassedReachesTheServiceWhileTheNodeIsInsideTheAbsenceGrace) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // "a" blinks out of the snapshot for a few ticks - well inside kDefaultAbsenceGrace -
  // and stays absent for the whole check below. Not one PASSED may reach the fake
  // service across the blink.
  set_apps({});
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kDefaultAbsenceGrace; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a node the detector already reported was cleared on a blink well inside the "
         "absence grace - the withheld-clear guard's pending leg did not hold it";
}

// R10 pin, at the tier that proves the SERVICE saw no churn. The re-raise on return is
// CORRECT and must be preserved: before this slice's fix the node cleared on its very
// first absent tick and then re-raised on return - a raise/clear/raise churn. With the
// fix there is no clear in between, so across the whole blink-and-return sequence the
// fake service must never see a PASSED (the emitter is level-triggered and re-sends
// FAILED on every tick the condition holds regardless of a blink, so a recurring FAILED
// count is expected and not itself evidence of churn - only an interleaved PASSED is).
TEST_F(LifecycleExpectationIntegrationTest, ReRaiseOnReturnProducesNoClearChurnThroughTheRealGate) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);

  // A brief blink, well inside the absence grace, then back - still inactive throughout.
  set_apps({});
  for (int i = 0; i < 2; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  set_apps({"a"});
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the blink must never have produced a clear - the streak survived it with no fresh "
         "grace to re-earn, so there is nothing here for a re-raise to follow";
}

// Change: config changes while a node's absence-grace hold counters are live. A
// reconfigure rebuilds the tracker (the old streak is gone), but the withheld-clear
// guard's OTHER leg - the fresh tracker has not matched the entry yet - must pick the
// hold back up without a gap, so no PASSED may leak across the reconfigure boundary
// either.
TEST_F(LifecycleExpectationIntegrationTest, ReconfigureDuringAnAbsenceHoldNeverLeaksAPassed) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  const nlohmann::json config{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}};
  det->configure(config);
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // "a" blinks out of the snapshot, and the detector is reconfigured mid-blink.
  set_apps({});
  det->tick(ctx);
  std::this_thread::sleep_for(20ms);
  det->configure(config);  // rebuilds the tracker: the old streak is gone

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a reconfigure mid-blink must not spuriously heal a fault that was never re-confirmed healthy";
}

// Regression: App::id is recomputed each sweep and gets namespaced on a same-bare-name
// collision, so a bare-name require_active entry must still match a live node via its stable
// fqn leaf - otherwise the check silently stops on a multi-robot graph. Here the app carries a
// namespaced id ("robot1_a") but a stable fqn "/robot1/a"; require_active: ["a"] (bare) must
// match it by leaf and still enforce the expectation.
TEST_F(LifecycleExpectationIntegrationTest, BareNameRequireActiveMatchesNamespacedAppByLeaf) {
  set_apps_raw({{"robot1_a", "/robot1/a"}});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Lifecycle state is keyed by the node's current App::id ("robot1_a"); the require_active
  // entry "a" matches it via the fqn leaf.
  gate.set_lifecycle_state_for_test("robot1_a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a bare-name require_active entry stopped matching a node whose App::id was namespaced by a "
         "collision - the check must match via the stable fqn leaf instead of silently stopping";
}

// A FULL-FQN entry must match through the fqn arm (id != fqn), not by luck of the App::id.
// The app carries an id that equals neither the entry nor its leaf, so only the fqn
// comparison can bind them.
TEST_F(LifecycleExpectationIntegrationTest, FullFqnRequireActiveMatchesByFqnWhenIdDiffers) {
  set_apps_raw({{"nsb", "/ns/b"}});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"/ns/b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("nsb", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a full-FQN require_active entry did not match an app whose id differs from its fqn";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/ns/b"));
}

// The shipped default: an empty detector config checks nothing and CONTACTS nothing. The
// instrument counts every request of any kind, so a stray clear-spam (or any other
// chatter) fails this, not just a wrong raise. The inactive label makes the claim sharp:
// even with a would-be-offending node in view, an unconfigured detector stays silent.
TEST_F(LifecycleExpectationIntegrationTest, EmptyConfigNeverContactsTheFaultManager) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json::object());
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms);  // let any in-flight request land before counting
  EXPECT_EQ(count_all_requests(), 0u)
      << "an unconfigured lifecycle_expectation sent the fault_manager a request; the zero-config "
         "default must emit neither raises nor clears";
}

// Same claim for the explicit `require_active: []` spelling of "nothing configured".
TEST_F(LifecycleExpectationIntegrationTest, ExplicitEmptyRequireActiveNeverContactsTheFaultManager) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array()}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_all_requests(), 0u) << "require_active: [] must behave exactly like no config at all";
}

// An entry that matches a present node with NO tracked lifecycle state is probably a typo
// (or a plain node). The warning must reach the log after kUnmanagedWarnTicks consecutive
// such ticks, exactly once per entry per configuration - and a reconfigure that removes
// and re-adds the entry must warn AGAIN, because that is exactly when the operator wants
// to hear the entry still names nothing managed.
TEST_F(LifecycleExpectationIntegrationTest, UnmanagedEntryWarnsOncePerConfiguration) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  const nlohmann::json config{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}};
  det->configure(config);
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // Deliberately NO set_lifecycle_state_for_test: "a" is matched but has no tracked
  // lifecycle state, which is the unmanaged case under test.

  const std::string needle = "require_active entry 'a' has matched a present node";
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "expected the unmanaged-entry warning exactly once over ten ticks "
                                     "(0 = never surfaced, >1 = the once-per-entry guard is gone)";

  // Remove the entry, then re-add it: the warn latch is scoped to the CURRENT config, so
  // the re-added entry must warn again.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array()}});
  det->configure(config);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 2) << "an entry removed and re-added by reconfigure() never warned again - "
                                     "the latch must reset with the config";
}

// Withheld-clear guard, the never-measured half (see the detector's withheld-clear design
// note). A gateway restart re-instantiates the detector with everything unread; the fault
// it raised BEFORE the restart is still in the store. Emitting the level-triggered clear
// before a single label has been read would spuriously HEAL a still-real inactive fault -
// so through the bounded hold window the detector must emit NOTHING, and the clear must
// flow only after a label is actually read.
TEST_F(LifecycleExpectationIntegrationTest, RestartStandInWithholdsTheClearUntilALabelIsRead) {
  set_apps({"a"});

  // Phase 1 (pre-restart): a configured + labeled run raises the prior fault.
  ReliabilityGate gate1(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate1);
  auto det1 = make_lifecycle_expectation();
  ASSERT_TRUE(det1);
  det1->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx1 = make_ctx(DetectorMode::Raise, &gate1);
  gate1.set_lifecycle_state_for_test("a", "inactive");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det1, ctx1))
      << "phase 1 never raised, so phase 2 would prove nothing";
  std::this_thread::sleep_for(200ms);  // let phase-1 in-flight requests land before baselining

  // Phase 2 (restart stand-in): fresh gate, fresh detector, NO labels read yet.
  ReliabilityGate gate2(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate2);
  auto det2 = make_lifecycle_expectation();
  ASSERT_TRUE(det2);
  det2->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx2 = make_ctx(DetectorMode::Raise, &gate2);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kHoldTicks; ++i) {
    det2->tick(ctx2);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);  // any withheld-but-actually-sent clear must have landed by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a freshly restarted detector cleared GRAPH_NODE_INACTIVE before reading a single lifecycle "
         "label - the restart spuriously heals a still-real fault";

  // The label is finally read (still-inactive would re-raise; active releases the clear).
  gate2.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det2, ctx2))
      << "once the label IS read and healthy, the clear must flow";
}

// Withheld-clear guard, the typo unblock: an entry that matches a node whose label is
// NEVER read - an unmanaged node, or a typo that happens to match - may hold the clear
// back only for the bounded hold window. Past it the clear must flow, or one such entry
// blocks healing forever.
TEST_F(LifecycleExpectationIntegrationTest, NeverReadNodePastTheHoldBoundStopsBlockingTheClear) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // NO label injection, ever: "a" stays unread for the whole test.

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear stayed withheld past the bounded hold - an unmanaged or typo'd entry must not "
         "block healing forever";
}

// Withheld-clear guard, the window that actually matters: a restart whose labels seed
// FAST. The plugin seeds lifecycle labels BEFORE it ticks the detectors, so on a
// responsive stack the first tick of a fresh detector already reads the real label -
// "unread" is the rare case, not the common one. What IS fresh after a restart is the
// violation streak, and while it climbs back through grace the tracker reports nothing
// affected. Emitting the level-triggered clear there asserts "nothing is stuck" about a
// node this detector's own last read found stuck, and heals the fault the restart was
// supposed to preserve.
TEST_F(LifecycleExpectationIntegrationTest, RestartStandInWithholdsTheClearWhileTheNodeReadsNotActiveBelowGrace) {
  set_apps({"a"});

  // Phase 1 (pre-restart): a configured + labeled run raises the prior fault.
  ReliabilityGate gate1(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate1);
  auto det1 = make_lifecycle_expectation();
  ASSERT_TRUE(det1);
  det1->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx1 = make_ctx(DetectorMode::Raise, &gate1);
  gate1.set_lifecycle_state_for_test("a", "inactive");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det1, ctx1))
      << "phase 1 never raised, so phase 2 would prove nothing";
  std::this_thread::sleep_for(200ms);  // let phase-1 in-flight requests land before baselining

  // Phase 2 (restart stand-in): fresh gate, fresh detector - but the label is seeded
  // BEFORE the first tick, and the node is still stuck exactly where it was.
  ReliabilityGate gate2(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate2);
  auto det2 = make_lifecycle_expectation();
  ASSERT_TRUE(det2);
  constexpr int kGrace = 4;
  det2->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kGrace}});
  auto ctx2 = make_ctx(DetectorMode::Raise, &gate2);
  gate2.set_lifecycle_state_for_test("a", "inactive");

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < kGrace; ++i) {  // the whole below-grace window, every tick measured stuck
    det2->tick(ctx2);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // any emitted request must have landed by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the restarted detector cleared GRAPH_NODE_INACTIVE while its own reads said the node was "
         "still not active - a young streak is not health";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the raise must still wait for grace; only the clear is withheld";

  // Past grace the raise resumes, so the hold is not a silence that never ends.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det2, ctx2))
      << "the streak never reached grace + 1, so the still-real fault was never re-raised";
}

// The grace contract, through the REAL config path: "grace" is the number of consecutive
// not-active ticks a required NODE tolerates. An operator combining the two documented
// require_active forms (bare name = fleet-wide, full FQN = pin one robot) has one node
// named by two entries, and the detector pushes one match per (entry, node) - so counting
// per match raises after ceil((grace+1)/2) ticks instead of grace+1, a transient false
// positive on a node that is simply still coming up.
TEST_F(LifecycleExpectationIntegrationTest, TwoEntriesMatchingOneNodeDoNotHalveTheConfiguredGrace) {
  set_apps({"a"});  // App::id "a", fqn "/a" - matched by BOTH entries below
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  constexpr int kGrace = 4;
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "/a"})}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < kGrace; ++i) {  // exactly the tolerated window
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the node was reported inactive INSIDE its grace because two require_active entries named "
         "it - the streak must advance once per node per tick, not once per match";

  // And the expectation is still enforced once the streak really does pass grace.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the raise never came at all - de-duplicating the matches must not disable the check";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a"));
}

// The churn story the absence grace exists for, driven through the detector: a stuck node
// blinks out of the snapshot and comes back with the label LifecycleWatcher seeds when its
// GetState has not answered yet (""). Treating that re-seed tick as a healthy read restarts
// the violation count, so a node that blinks once every few ticks never accumulates
// grace + 1 consecutive stuck ticks and is never reported at all.
TEST_F(LifecycleExpectationIntegrationTest, BlinkAndUnreadReseedDoNotRestartTheViolationCount) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 3}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Tick-exact on purpose. "It raises eventually" is true whether or not the streak
  // survives - a reset only delays the raise - so an unbounded poll would pass against a
  // detector that restarts the count on every blink. The claim is that the streak reaches
  // grace + 1 on THIS tick, so the run is counted out and nothing ticks afterwards.
  gate.set_lifecycle_state_for_test("a", "inactive");
  det->tick(ctx);  // stuck 1
  det->tick(ctx);  // stuck 2

  set_apps({});    // blink: out of the snapshot for one tick
  det->tick(ctx);  // absent 1 - within the absence grace, streak preserved

  set_apps({"a"});                             // back, but the re-seed has not answered yet
  gate.set_lifecycle_state_for_test("a", "");  // exactly what a missed GetState seed leaves
  det->tick(ctx);                              // unread: absence-like, streak preserved
  gate.set_lifecycle_state_for_test("a", "inactive");
  det->tick(ctx);  // stuck 3 == grace, not past it yet

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // stuck 4 > grace - the raise is due on this tick and no later
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1))
      << "a node stuck through a blink and an unread re-seed was not reported on the tick its streak "
         "passed grace - the blink or the re-seed restarted its violation count, which is how a stuck "
         "node on a churning graph stays silent forever";
}

// Guard bookkeeping is per NODE, not per (entry, node) pair. An operator combining the
// documented bare-name and pinned-FQN forms has one node named by two entries; burning
// its not-managed hold twice a tick halves the documented bound and lets the clear out
// at half the horizon the design promises.
//
// C2: named "Unread" before this correction, but "a" carries no lifecycle services
// (set_apps, not set_managed_app) and nothing ever injects a label for it, so
// lifecycle_state_of("a") is nullopt for the whole test - this pins the NOT-MANAGED cause
// of the shared unmeasured clock, not the UNREADABLE one. See
// TwoEntriesMatchingOneNodeDoNotHalveTheUnreadableHold below for the genuine
// optional("") case.
TEST_F(LifecycleExpectationIntegrationTest, TwoEntriesMatchingOneNodeDoNotHalveTheNotManagedHold) {
  set_apps({"a"});  // App::id "a", fqn "/a" - matched by BOTH entries below
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "/a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // NO label injection, ever: "/a" stays nullopt (NOT-MANAGED), so only the hold can
  // release the clear.
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "\"a\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kHoldTicks / 2 + 5; ++i) {  // comfortably past a halved hold, short of the real one
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the not-managed hold released at half its documented length because two entries named "
         "the same node - the guard must count per node, not per match";

  // The bound itself still applies: past it, the clear flows.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the hold never released at all - it must stay bounded";
}

// The genuine optional("") twin of the test above: "a" IS tracked (injected once, via
// the seam, which writes straight into the gate's internal tracked map regardless of
// services - see set_managed_app's doc comment for the alternative, real-seeding path),
// but its label is deliberately left "" for the whole test, so BOTH entries matching it
// share ONE unmeasured clock, not two.
TEST_F(LifecycleExpectationIntegrationTest, TwoEntriesMatchingOneNodeDoNotHalveTheUnreadableHold) {
  set_apps({"a"});  // App::id "a", fqn "/a" - matched by BOTH entries below
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "/a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  for (int i = 0; i < kHoldTicks / 2 + 5; ++i) {  // comfortably past a halved hold, short of the real one
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), failed_before)
      << "the unreadable hold converted to content at half its documented length because two "
         "entries named the same node - the guard must count per node, not per match";

  // The bound itself still applies: past it, the hold converts into a report.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx, kNodeUnreadable))
      << "the hold never converted at all - it must stay bounded";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a", kNodeUnreadable));
}

// Mixed set, direction 1: node "a" was raised and has now healed, while sibling "b" has
// never been measured. Within the hold the clear waits - a run in which "b" was never
// read is not a run in which "b" was found healthy.
//
// C2: named "Unread" before this correction, but "b" carries no lifecycle services and
// is never injected, so lifecycle_state_of("b") is nullopt for the whole test - this
// pins the NOT-MANAGED leg, not the unreadable one. See
// MixedSetClearWaitsForTheUnreadableSiblingWithinTheHold below for the genuine
// optional("") case.
TEST_F(LifecycleExpectationIntegrationTest, MixedSetClearWaitsForTheNotManagedSiblingWithinTheHold) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" is never read in this test
  ASSERT_FALSE(gate.lifecycle_state_of("b").has_value())
      << "\"b\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "the raise never happened, so the heal leg would prove nothing";

  gate.set_lifecycle_state_for_test("a", "active");  // the raised node heals
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 20; ++i) {  // well inside "b"'s hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear flowed while a required sibling had never been measured once";

  gate.set_lifecycle_state_for_test("b", "active");  // now everything really is measured healthy
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "once every required node has been read healthy the clear must flow";
}

// The genuine optional("") twin of the test above: "b" IS tracked (via the injection
// seam) but its label is deliberately left "" for the whole test, so it exercises the
// unmeasured clock's UNREADABLE cause, not its NOT-MANAGED one.
TEST_F(LifecycleExpectationIntegrationTest, MixedSetClearWaitsForTheUnreadableSiblingWithinTheHold) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  gate.set_lifecycle_state_for_test("b", "");  // unreadable, never read in this test
  ASSERT_TRUE(gate.lifecycle_state_of("b").has_value() && gate.lifecycle_state_of("b")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "the raise never happened, so the heal leg would prove nothing";

  gate.set_lifecycle_state_for_test("a", "active");  // the raised node heals
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 20; ++i) {  // well inside "b"'s hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear flowed while a required sibling had never been measured once";

  gate.set_lifecycle_state_for_test("b", "active");  // now everything really is measured healthy
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "once every required node has been read healthy the clear must flow";
}

// Mixed set, direction 2: the hold is a bound on the WAIT, not a promise that every node
// was measured. "b"'s budget burns on every matched tick, including the ticks a raise
// about "a" was flowing, so a raise that outlives the hold leaves the clear free to flow
// the moment "a" heals - deliberately, or one unmanaged sibling would block healing for
// as long as the raise lasted.
//
// C2: named "Unread" before this correction, but "b" is genuinely nullopt (NOT-MANAGED)
// here, same as the pair above - see MixedSetClearWaitsForTheUnreadableSiblingWithinTheHold
// above for the corresponding UNREADABLE direction. GRAPH_NODE_INACTIVE's own clear behaves
// identically either way: once "b"'s unmeasured clock matures - whichever cause it matured
// under - ownership passes to "b"'s own fault code and GRAPH_NODE_INACTIVE has nothing left
// to hold for it, so "a" healing is free to clear GRAPH_NODE_INACTIVE regardless. What DOES
// differ is "b"'s own content: it converts into GRAPH_NODE_NOT_MANAGED here (asserted
// below), the sibling code to GRAPH_NODE_UNREADABLE - both stay reported with no further
// bound once matured (see UnreadableNodeStaysReportedWithNoExpiryAsLongAsItStaysUnreadable).
TEST_F(LifecycleExpectationIntegrationTest, MixedSetClearFlowsOnceTheNotManagedSiblingsHoldIsSpent) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" is never read in this test
  ASSERT_FALSE(gate.lifecycle_state_of("b").has_value())
      << "\"b\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";
  for (int i = 0; i < kHoldTicks + 5; ++i) {  // the raise outlives "b"'s hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1))
      << "the raise never happened, so the heal leg would prove nothing";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeNotManaged))
      << "\"b\"'s not-managed clock never converted into its own record, so the clear below is not "
         "proving what this test claims - the withhold releasing via ownership transfer, not silence";

  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never came: past its hold a not-managed sibling must stop blocking, or one "
         "unmanaged entry would keep a healed fault raised for as long as the raise lasted";
}

// R7's underlying concern, restated under the current (post-redesign) model, which has no
// "a label was read" latch of its own to go stale: a process that dies and respawns under
// the same name must not have its fresh, unread incarnation's unmeasured clock exempted by
// anything the DEAD incarnation did. Here there is nothing to exempt it WITH - a healthy
// read resets the unmeasured clock to zero (LifecycleExpectationTracker's own rule), so a
// respawn that comes back unreadable starts a clock at zero exactly like a first-contact
// unreadable node would, and only a REAL read - never the dead incarnation's - resets it
// again. This is what makes the sticky-`""` failure mode R7 originally closed structurally
// impossible now, rather than something a second bit has to keep correctly synchronized.
TEST_F(LifecycleExpectationIntegrationTest, RespawnComingBackUnreadableStartsAFreshUnmeasuredClockNotADeadLatch) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // resets both clocks
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the healthy baseline never cleared, so nothing below can be attributed to the respawn";

  set_apps({});  // the process dies
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  // Respawned under the same fqn with an unseeded label ("" is what LifecycleWatcher
  // seeds for a tracked node whose GetState has not answered yet).
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a respawn that came back unreadable was cleared on the strength of the DEAD "
         "incarnation's earlier read - the fresh incarnation's own unmeasured clock must "
         "start from zero and be honored on its own, or the sticky-\"\" failure mode R7 "
         "exists to close would just reappear on every respawn";

  // And it is not stuck in silence forever either: once the fresh incarnation is
  // actually read, the clear flows normally, same as any first-contact node.
  gate.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never flowed once the respawned incarnation was genuinely read active";
}

// The other half of the same story: the surviving latch does NOT leave a respawned node
// unguarded, because the streak the respawn starts is itself a hold. A node that comes
// back stuck must not have its still-real fault healed while its streak climbs to grace.
TEST_F(LifecycleExpectationIntegrationTest, RespawnedNodeStuckBelowGraceStillWithholdsTheClear) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  constexpr int kGrace = 5;
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // read once: the latch is set
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx));

  set_apps({});  // the process dies, past the absence grace
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);

  // It comes back - stuck. The latch is still set from the dead incarnation, so only the
  // streak can hold the clear back.
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kGrace; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a node that respawned still stuck had its fault cleared while the detector's own reads "
         "said it was not active";
}

// R25, second finding: a re-contact earns the same protection as a first contact. "a"
// stays present in the GRAPH the whole time (this is not the presence/absence case,
// already covered by AbsenceAfterRaiseClearsNotNodeDeathsDomain), but a REAL gate.update()
// call drops its LifecycleWatcher-tracked entry once its GetState/ChangeState services
// are gone from the snapshot passed to it - exactly what happens when the underlying
// process restarts and its services have not been rediscovered yet (see the class doc on
// lifecycle_watcher.cpp's drop loop). lifecycle_state_of("a") then reads nullopt, and
// GRAPH_NODE_INACTIVE's already-CONFIRMED violation must not be cleared on the strength
// of that alone - it might be coming back stuck.
TEST_F(LifecycleExpectationIntegrationTest, RestartReadingNullOptWhileServicesAreRediscoveredDoesNotClearALiveStreak) {
  set_managed_app("a", "/a");
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Inject AFTER the last gate.update() (ordering rule 2): confirmed violation.
  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "\"a\" never raised, so nothing below proves a live streak survived the restart";

  // Restart stand-in: "a" stays present in ctx.snapshot->apps (never removed via
  // set_apps({})) but loses its GetState/ChangeState services for this one gate.update()
  // call - the new incarnation has not advertised them yet. That is what makes
  // LifecycleWatcher drop the tracked entry, so lifecycle_state_of("a") reads nullopt
  // from here on, exactly as a live rediscovery window would.
  set_apps_raw({{"a", "/a"}});           // present, no services
  gate.update(snapshot_, /*tick=*/999);  // drops the tracked entry
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "the restart stand-in did not actually produce nullopt - this test would not be exercising "
         "the rediscovery window";

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kDefaultAbsenceGrace; ++i) {  // well inside the rediscovery window
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a live GRAPH_NODE_INACTIVE cleared while its node was restarting and reading nullopt with "
         "its services not yet rediscovered - it might be coming back stuck";

  // Not held open-ended either: once the fresh incarnation is genuinely read active, the
  // clear flows normally, same as any first-contact node.
  gate.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never flowed once the respawned incarnation was genuinely read active";
}

// R25, first finding: a node must earn a fresh unmeasured-clock hold across a REAL
// absence-crossing restart. "a" leaves ctx.snapshot->apps entirely and stays away past
// kDefaultAbsenceGrace - the tracker's own absence loop resets both clocks to zero on
// that crossing - then returns reading nullopt (a REAL gate.update() drops the cached
// "active" label, same restart stand-in as the test above). The observable signal:
// AggregatedFault's emit_ordered has no dedup of its own (see its class doc), so EVERY
// tick left unheld re-sends a clear - a clock that resumed from wherever it was before
// the restart, instead of starting fresh at zero, would let the level-triggered clear
// keep firing on every tick after the return.
TEST_F(LifecycleExpectationIntegrationTest, ReturningNodeAfterARealRestartEarnsAFreshHoldInsteadOfResumingTheOldClock) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // read once: both clocks reset
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the healthy baseline never cleared, so nothing below can be attributed to the restart";

  // A REAL restart: "a" leaves the graph entirely, past the absence grace - not a blink.
  set_apps({});
  for (int i = 0; i < kDefaultAbsenceGrace + 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }

  // It returns, but its GetState/ChangeState services have not been rediscovered - same
  // restart stand-in as the test above: a REAL gate.update() drops the dead incarnation's
  // cached "active" label, so lifecycle_state_of("a") now reads nullopt.
  set_apps_raw({{"a", "/a"}});
  gate.update(snapshot_, /*tick=*/999);
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "the restart stand-in did not actually produce nullopt - this test would not be exercising "
         "the return path";

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kHoldTicks - 5; ++i) {  // comfortably below the fresh hold's own bound
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a node returning after a real restart resumed the dead incarnation's unmeasured clock "
         "instead of earning a fresh hold - every tick since should have been withheld, not "
         "re-affirmed";
}

// The window BEFORE the first match, which neither the unmeasured clock nor the streak can see.
// A restarted gateway does not have the graph yet: for the first ticks after the plugin
// comes up the entity snapshot has not caught up, so a require_active entry matches
// nothing at all. Both other halves of the guard are keyed by a MATCHED node, so both are
// empty, the tracker reports nothing affected, and the level-triggered clear flows - about
// a node the detector has never once looked at. The fault the restart was supposed to
// preserve heals in that gap.
//
// Distinct from a node that VANISHES: an entry that has matched before and stops matching
// is a presence problem (pinned by AbsenceAfterRaiseClearsNotNodeDeathsDomain, which must
// stay green). This is an entry that has never matched anything since configure().
TEST_F(LifecycleExpectationIntegrationTest, ClearIsWithheldUntilARequiredEntryHasMatchedAtLeastOnce) {
  set_apps({});  // the required node has not reached the snapshot yet
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the detector asserted GRAPH_NODE_INACTIVE was healthy before it had ever matched the node "
         "it was told to check - after a restart that heals a fault that is still real";

  // Once the node is actually there and reads healthy, the clear flows.
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never flowed once the required node appeared and read active";
}

// The bound on that hold, same shape as the unread one: an entry that matches nothing
// because it is misspelt would otherwise block healing for the process lifetime.
TEST_F(LifecycleExpectationIntegrationTest, NeverMatchedEntryPastTheHoldBoundStopsBlockingTheClear) {
  set_apps({"something_else"});  // "a" matches nothing, and never will
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "a misspelt entry that can never match blocked healing forever - the never-matched hold "
         "must be bounded exactly like the unread one";
}

// A withhold is silence, and silence is indistinguishable from a detector that is working
// and finding nothing. The two reasons release on different conditions, so the log has to
// say which one is in effect - here the measured-but-below-grace one, whose hold ends when
// the node reads active or its streak passes grace, NOT after a fixed number of ticks.
TEST_F(LifecycleExpectationIntegrationTest, WithholdingForAYoungStreakSaysSoInTheLog) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  // Grace wide enough that the whole run stays inside the pending window.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 200}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");  // read every tick: only the streak can hold

  for (int i = 0; i < 30; ++i) {  // well past the 10-tick reporting horizon
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("withheld from clearing"), 1)
      << "a hold that outlives the reporting horizon must be explained exactly once per episode";
  EXPECT_EQ(log.count("measured not-active but still within grace"), 1)
      << "the log named the wrong reason: nothing here is unread, the streak is simply young";
  EXPECT_EQ(log.count("no lifecycle label read this run"), 0)
      << "the unread reason was reported for a node whose label was read on every tick";
}

// Withheld-clear guard boundary: a raise is NEVER withheld. A violation read from the
// nodes that did answer is real regardless of how many other required nodes are unread.
TEST_F(LifecycleExpectationIntegrationTest, RaiseIsNeverWithheldByAnUnreadNode) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" stays unread
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "an unread sibling node held back a raise about a node that WAS read inactive";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a"));
}

// Scale: more required nodes stuck inactive than the 480-char cap can name, PLUS one MORE
// crossing fresh on a LATER tick, after the cap is already full. This replaces the old
// TwentyFiveStuckNodesRaiseOneCappedFault, which pinned that the LEXICOGRAPHICALLY FIRST
// affected node opens the description ("node /a00 expected active..." at offset 0) with 25
// nodes crossing together in one tick. That assertion is now WRONG, on purpose: R11 makes the
// order "entered `affected` on THIS tick, first", not lexicographic, so a fresh violation must
// not be hidden behind alphabetically-earlier ones that have been stale for ticks - which is
// exactly the failure mode the old lexicographic emit() had (a later-arriving robot in a
// `require_active: [...]` fleet could be silently unnamed forever once earlier ones filled the
// cap). This test proves the fix directly: `fill_count_past_cap` derives - from the real
// detail-building code, not a hard-coded 25 - the smallest filler batch that alone exceeds the
// cap, then one MORE node crosses on a later tick with an id chosen to sort LAST among all of
// them; the old code would have dropped it from the description entirely.
TEST_F(LifecycleExpectationIntegrationTest, ANodeCrossingAfterTheCapIsFullIsNamedOverOlderEntries) {
  std::vector<std::string> ids;
  const std::size_t fill_count = fill_count_past_cap("e", ids);
  ASSERT_GT(fill_count, 0u);
  const std::string late_id = "zlate";  // sorts after every "e..." filler, lexicographically last
  ids.push_back(late_id);

  nlohmann::json require = nlohmann::json::array();
  for (const auto & id : ids) {
    require.push_back(id);
  }
  set_apps(ids);
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", 0}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  for (const auto & id : ids) {
    if (id != late_id) {
      gate.set_lifecycle_state_for_test(id, "inactive");
    }
  }
  gate.set_lifecycle_state_for_test(late_id, "active");  // healthy for now

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // ONE tick: the filler batch crosses grace=0 together; "zlate" stays healthy
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1));
  std::this_thread::sleep_for(200ms);  // a second request from the same tick would land by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before + 1)
      << "the filler batch crossing together in one tick must aggregate into exactly one FAILED";
  {
    const std::string desc = last_failed_description(kGraphSource);
    const std::string marker = "...";
    EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
        << "the aggregated description exceeded the cap";
    ASSERT_GE(desc.size(), marker.size());
    EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
        << "the filler batch alone was sized (by fill_count_past_cap) to exceed the cap, so this "
           "description must be truncated, got tail: "
        << desc.substr(desc.size() - std::min<std::size_t>(desc.size(), 20));
    EXPECT_EQ(desc.find("/zlate"), std::string::npos) << "\"zlate\" has not gone inactive yet and must "
                                                         "not be named";
  }

  // "zlate" goes inactive - the fresh crossing, on a LATER tick, after the cap is already
  // full of filler entries that all sort before it.
  gate.set_lifecycle_state_for_test(late_id, "inactive");
  const auto failed_before_fresh = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // ONE tick: "zlate" crosses grace=0 fresh; every filler is already old news
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before_fresh + 1));
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before_fresh + 1)
      << "the fresh crossing must also aggregate into exactly one FAILED";

  const std::string desc = last_failed_description(kGraphSource);
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
      << "the aggregated description exceeded the cap";
  EXPECT_EQ(desc.find("node /zlate expected active"), 0u)
      << "the freshly-crossed node must open the description, even though its fqn sorts LAST "
         "lexicographically among every entry - the old lexicographic emit() would have dropped "
         "it entirely, since the filler batch alone already exceeds the cap. Full description: "
      << desc;
}

// The other half of the same ordering rule, and the one the R11 fix above does not reach: a
// DEPARTED node and a PRESENT one crossing `grace` on the SAME tick. Absence-driven crossings
// enter `newly_affected` alongside present ones, so a single lexicographic list lets a node
// that LEFT the graph spend the description budget and truncate away the node that just
// broke - and the operator is then told about the one that is gone instead of the one that
// needs attention.
//
// A departed node can no longer cross INTO GRAPH_NODE_INACTIVE from a below-grace
// streak (see the tracker's own class doc), so "both cross on the same tick" is no longer a
// reachable shape for this fault - a departed entry only ever carries content it EARNED
// before it left. What is still reachable, and still the real question, is whether a node
// that just broke is named ahead of a pile of nodes that broke earlier and left: the departed
// batch matures FIRST while present, THEN leaves for good, THEN present_id joins and crosses
// grace fresh, on its own tick, while the batch is still absent-and-content. The batch is
// sized by fill_count_past_cap from the REAL detail builder, so its details alone exceed the
// 480-char cap - which is what makes "which one is named" a real choice rather than a
// cosmetic ordering - and the present node's id sorts LAST among them all.
TEST_F(LifecycleExpectationIntegrationTest, PresentNodeCrossingWithDepartedOnesIsNamedAheadOfThem) {
  constexpr int kGrace = 1;
  std::vector<std::string> departed_ids;
  const std::size_t departed_count = fill_count_past_cap("g", departed_ids);
  ASSERT_GT(departed_count, 1u);
  const std::string present_id = "zpresent";  // sorts after every "g..." departed node

  std::vector<std::string> all = departed_ids;
  all.push_back(present_id);
  nlohmann::json require = nlohmann::json::array();
  for (const auto & id : all) {
    require.push_back(id);
  }
  set_apps(all);  // every id known to the gate before the labels are injected
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  for (const auto & id : all) {
    gate.set_lifecycle_state_for_test(id, "inactive");
  }

  // The departed batch matures FIRST, on its own: present_id is not in the snapshot yet.
  set_apps(departed_ids);
  for (int i = 0; i < kGrace + 1; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  ASSERT_TRUE(any_failed_desc_contains(kGraphSource, departed_ids.front()))
      << "the departed batch never matured, so nothing below tests what an ALREADY-CONTENT entry "
         "does once it leaves - only what a fresh crossing does";

  // The batch leaves for good, past the absence grace. It is already matured, so absence
  // continues it unconditionally - "has since left the graph" and all.
  set_apps({});
  for (int i = 0; i < kDefaultAbsenceGrace + 2; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }

  // present_id joins and crosses grace fresh, on its own tick, while the departed batch is
  // still absent-and-content.
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  set_apps({present_id});
  for (int i = 0; i < kGrace + 1; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1))
      << "present_id never crossed grace, so this test never reached the tick it is about";
  std::this_thread::sleep_for(300ms);

  const std::string desc = last_failed_description(kGraphSource);
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars);
  ASSERT_NE(desc.find(present_id), std::string::npos)
      << "the PRESENT node that crossed on this very tick was truncated out of the description by "
         "nodes that had already left the graph - the operator is told about the departures and not "
         "about the node that just broke. Full description: "
      << desc;
  ASSERT_NE(desc.find(departed_ids.front()), std::string::npos)
      << "the departed batch left no trace at all, so the ordering claim below would be vacuous - its "
         "evidence must survive the departure, just behind present_id. Full description: "
      << desc;
  EXPECT_LT(desc.find(present_id), desc.find(departed_ids.front()))
      << "the present node is named, but behind a departed one - a departure is never more urgent "
         "than a node that is still there and has just gone bad. Full description: "
      << desc;
}

// Change over time: a required node that APPEARS mid-run (graph growth) and then sticks
// inactive must be raised - the entry was configured before the node ever existed, so
// this pins enforcement of late arrivals, not just nodes present from arming.
TEST_F(LifecycleExpectationIntegrationTest, NodeAppearingMidRunThenStuckInactiveRaises) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "active");
  gate.set_lifecycle_state_for_test("b", "inactive");  // label parked before "b" exists - harmless

  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "raised about a node that is not in the graph yet";

  // "b" joins the graph and sits inactive.
  set_apps({"a", "b"});
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a required node appearing mid-run and sticking inactive was never raised";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b"));
}

// A re-bind under the SAME App::id must not keep enforcing the OLD node's label: "k" is
// bound to one managed node, read inactive there, then the id moves to a DIFFERENT node
// whose state nothing has read yet. Whatever label survives that move is what the
// detector enforces - and an unread new binding is unknown (benign), so no raise may
// flow. Uses managed-service snapshots (not the service-less fixture apps) because the
// re-bind is expressed through gate.update() itself.
TEST_F(LifecycleExpectationIntegrationTest, RebindUnderSameAppIdDoesNotEnforceTheOldNodesLabel) {
  set_managed_app("k", "/rb_old");
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"k"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // The OLD binding's label, injected while "k" is still bound to /rb_old.
  gate.set_lifecycle_state_for_test("k", "inactive");

  // The re-bind: same App::id, different node. This gate.update() AFTER the injection is
  // deliberate (the one exception to ordering rule 2): with lifecycle services in the
  // snapshot the update keeps tracking "k", the injected label stands in for the OLD
  // binding's last read, and the NEW binding's own GetState fails into unknown - so
  // whatever label survives this update is exactly what the detector will enforce.
  set_managed_app("k", "/rb_new");
  gate.update(snapshot_, 9);

  for (int i = 0; i < 15; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // let any in-flight raise land before counting
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the OLD binding's inactive label was enforced against the re-bound node - a re-bind must "
         "re-seed the id, not keep the departed node's label";
}

// The withheld-clear guard hands a node over to the presence class the moment it is
// absent past the absence grace. Without that hand-off a never-answered node that
// crashes keeps blocking the clear until its guard entry is pruned - a whole minute at
// the shipped horizons instead of three ticks, so a GRAPH_NODE_INACTIVE about a
// DIFFERENT node takes that long to heal after the blocking node dies.
//
// C2: this pins ONLY the NOT-MANAGED cause of the shared unmeasured clock - "b" carries
// no lifecycle services and is never injected, so lifecycle_state_of("b") is nullopt for
// the whole test. The claim used to be written as if it covered "both causes" on the
// strength of sharing one absence counter; that is not the same as exercising both. See
// UnreadableNodeThatVanishesStopsBlockingTheClearAtTheAbsenceGrace below for the genuine
// optional("") case, proven separately rather than assumed.
TEST_F(LifecycleExpectationIntegrationTest, NotManagedNodeThatVanishesReleasesTheClearBySettlingNotByBeingDropped) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // "b" is matched but NEVER tracked
  ASSERT_FALSE(gate.lifecycle_state_of("b").has_value())
      << "\"b\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 5; ++i) {  // far inside "b"'s 60-tick not-managed hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear was never withheld in the first place, so the release below would prove nothing";

  // "b" crashes out of the graph. Its clock keeps climbing while it is gone, so the hold
  // does NOT end at the absence grace - it ends when the clock MATURES and "b" becomes
  // GRAPH_NODE_NOT_MANAGED's own content. The hold is released by SETTLING the node's
  // status, never by giving up on it.
  set_apps({"a"});
  for (int i = 0; i < kDefaultAbsenceGrace + 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the hold was released at the absence grace, discarding the evidence \"b\" had earned "
         "instead of continuing to count it";

  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before + 1))
      << "GRAPH_NODE_INACTIVE never cleared even after \"b\"'s clock matured and ownership passed to "
         "its own code - the withhold has no release at all";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeNotManaged))
      << "\"b\" left the graph while unmeasured and was never reported under its own code - the hold "
         "released into silence rather than into content";
}

// The genuine optional("") twin: "b" IS tracked (via the injection seam) but its label
// is left "" the whole time, exercising the UNREADABLE cause rather than NOT-MANAGED -
// proving the same release-by-settling applies to the unreadable cause too, rather than
// assuming it from the two causes sharing one clock.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableNodeThatVanishesReleasesTheClearBySettlingNotByBeingDropped) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");
  gate.set_lifecycle_state_for_test("b", "");  // "b" is matched but never answers
  ASSERT_TRUE(gate.lifecycle_state_of("b").has_value() && gate.lifecycle_state_of("b")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 5; ++i) {  // far inside "b"'s 60-tick unreadable hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear was never withheld in the first place, so the release below would prove nothing";

  // "b" crashes out of the graph, still well inside its unreadable hold. Same rule as the
  // not-managed sibling: the clock keeps climbing while it is gone.
  set_apps({"a"});
  for (int i = 0; i < kDefaultAbsenceGrace + 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the hold was released at the absence grace, discarding the evidence \"b\" had earned";

  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before + 1))
      << "GRAPH_NODE_INACTIVE never cleared even after \"b\"'s clock matured under the unreadable cause";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeUnreadable))
      << "\"b\" left the graph unread and was never reported under GRAPH_NODE_UNREADABLE - the hold "
         "released into silence rather than into content";
}

// The witness for GRAPH_NODE_UNREADABLE's OWN record across a departure, at the tier that
// sees the wire: "a" is ticked past its unreadable hold FIRST, so the record is genuine
// content (a real FAILED, not merely a withheld GRAPH_NODE_INACTIVE clear) before it
// vanishes. The record must survive the departure - the node's lifecycle promise was never
// verified, and it leaving the graph does not verify it - and the fault must say the node
// is gone rather than keep describing a graph it left.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableNodeAlreadyReportedThatVanishesKeepsItsOwnRecord) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report, so this test would not exercise an "
         "ALREADY-REPORTED node vanishing";

  // "a" vanishes, having never once been read.
  set_apps({});
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);
  for (int i = 0; i < kDefaultAbsenceGrace + 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable), passed_before)
      << "a node already reported under GRAPH_NODE_UNREADABLE cleared once it vanished - its "
         "lifecycle promise is no more verified now than it was while the node was present";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "has since left the graph", kNodeUnreadable))
      << "the record kept describing a present-but-unread node after that node left the graph";
}

// The return leg: a node whose unmeasured clock had already matured, that vanishes and then
// comes BACK still unreadable, must stay exactly where it was - one continuous fault, no
// clear on the way out and no fresh raise on the way back. A clock that absence reset would
// produce raise/clear/raise churn on the fault surface for a node whose situation ("still
// cannot be measured") never changed. `poll_for_new`'s budget covers the whole sequence.
TEST_F(LifecycleExpectationIntegrationTest, ReturningUnreadableNodeStaysReportedWithoutClearChurn) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report, so nothing below is tested";

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);

  // "a" vanishes well past the absence grace, then returns, still unreadable - the injected
  // label was never changed, so it must still read optional("").
  set_apps({});
  for (int i = 0; i < kDefaultAbsenceGrace + 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  set_apps({"a"});
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "\"a\"'s injected state did not survive the absence - this test would not be exercising "
         "the return path";
  // Let everything the ticks above put in flight land BEFORE the baseline is taken, or the
  // exact-count assertion below would be racing arrivals from the departure leg.
  std::this_thread::sleep_for(300ms);
  const auto failed_before_return = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);

  constexpr int kReturnTicks = 10;
  for (int i = 0; i < kReturnTicks; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);

  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable), passed_before)
      << "the departure-and-return produced a clear, so the operator saw GRAPH_NODE_UNREADABLE heal "
         "and re-raise for a node that was never once read";
  // The aggregate is level-triggered, so every one of those ticks emits exactly one FAILED
  // while its content is non-empty - no more. An EXTRA one on the return is the churn this
  // test is named for, and counting is the only way to see it: "a fault is present" is
  // satisfied by the ORIGINAL raise and would pass whatever the return did.
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable),
            failed_before_return + kReturnTicks)
      << "the return emitted a different number of reports than one per tick - a node coming back "
         "still unreadable must continue the one report it already had, not add a fresh raise on top";
  EXPECT_NE(last_failed_description(kGraphSource, kNodeUnreadable).find("/a"), std::string::npos)
      << "the returning node stopped being named - checked on the LAST report rather than on any, "
         "since 'some report named it' is satisfied by the original raise before the departure";
}

// A withhold is silence, so it is explained once per episode - and the episode bookkeeping
// is what makes "once" survive a hold that lives for minutes AND lets a LATER hold speak up
// again. Nothing measured the log line itself: deleting the whole reporting body, or just
// its latch, left every hold test green.
TEST_F(LifecycleExpectationIntegrationTest, WithheldClearIsExplainedOncePerEpisodeNotOncePerProcess) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  // Grace wide enough that the second episode below never leaves the pending window.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 200}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const std::string needle = "withheld from clearing";
  // Episode 1: "a" is matched and never read. The horizon is 10 ticks, so 12 crosses it.
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "a hold that outlived the 10-tick reporting horizon was never explained";
  EXPECT_EQ(log.count("no lifecycle label read this run"), 1) << "the log named the wrong reason for an unread node";

  for (int i = 0; i < 20; ++i) {  // the SAME episode, twice as long
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "the reason was repeated within one episode - it is once per episode, or "
                                     "a hold that lasts minutes fills the log with the same line";

  // The episode ends: the label reads active, everything is measured healthy, the clear flows.
  gate.set_lifecycle_state_for_test("a", "active");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the hold never released, so no second episode can start";

  // Episode 2, a different reason: the node reads not-active again and its fresh streak is
  // young, which withholds on its own.
  gate.set_lifecycle_state_for_test("a", "inactive");
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 2) << "the second withhold episode was never explained - the latch is scoped to "
                                     "the process, so an operator only ever hears about the first one";
  EXPECT_EQ(log.count("measured not-active but still within grace"), 1)
      << "the second episode named the first episode's reason";
}

// The detector's own no-match warning, at the tier that can see it: an entry that matches
// nothing at all is silent in every other path (it never reaches the violation branch, and
// the presence class never tracks a node that was never present), so this log line is the
// only signal a misspelt entry produces.
TEST_F(LifecycleExpectationIntegrationTest, EntryThatNeverMatchesAnythingIsWarnedAboutInTheLog) {
  const LogCapture log;
  set_apps({"something_else"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"typoed_name"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const std::string needle = "has matched no node at all since startup";
  for (int i = 0; i < 12; ++i) {  // the default no-match horizon is 10 ticks
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1)
      << "a require_active entry that can never match produced no fault and no log line either, which is "
         "the whole failure mode this warning exists for (0 = never fired at detector level; >1 = the "
         "once-per-entry latch is gone)";
}

// The other side of that warning: it must not be said about an entry whose node WAS there
// and left. The tracker counts CONSECUTIVE no-match ticks, so a departed node surfaces the
// same way a misspelt entry does - but for it both halves of the sentence are false, and
// the presence class (GRAPH_NODE_DISAPPEARED) owns it, exactly as the never-matched hold
// distinguishes the two cases.
TEST_F(LifecycleExpectationIntegrationTest, EntryWhoseNodeVanishedIsNotAccusedOfNeverComingUp) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");
  det->tick(ctx);  // "a" matches here, and never again

  set_apps({});
  for (int i = 0; i < 15; ++i) {  // well past the 10-tick no-match horizon
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("has matched no node at all since startup"), 0)
      << "a node that was present and then departed was reported as one that never came up - and as one "
         "the presence class cannot see, which is exactly backwards: a departed node is what "
         "GRAPH_NODE_DISAPPEARED tracks";
}

// `grace` arrives from operator YAML as a 64-bit ROS integer parameter, so a value past the
// int range reaches configure() intact and is only truncated by the narrowing read - 2^32
// becomes 0, passes a >= 0 check, and silently installs a hair-trigger that reports a node
// on its very first not-active tick. The documented contract is that anything invalid warns
// and keeps the default, so the DEFAULT (5) must be what is actually in force here.
TEST_F(LifecycleExpectationIntegrationTest, WideGraceIsRejectedInsteadOfTruncatedIntoAHairTrigger) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4294967296}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < 3; ++i) {  // a grace truncated to 0 raises on the FIRST of these
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "a grace past the int range was truncated to 0 and reported a node that had been not-active for "
         "three ticks - the default grace of 5 is what an invalid value must leave in force";

  // And the default really is in force, so the expectation is still enforced past it.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "rejecting the invalid grace disabled the check instead of falling back to the default";
}

// ---- the unreadable/not-managed split (matched-but-unread label collapsed two facts
// into one before this slice) ----

// Test-plan row 1 + the "instrument measures the claim" shape requirement: a REAL
// managed lifecycle node (GetState/ChangeState services present, via set_managed_app)
// whose seed never answers because no live node listens at its fqn - the gate's REAL
// seeding path, not the set_lifecycle_state_for_test() injection seam every other test
// here uses. The fixture's service-less set_apps() apps are never tracked at all
// (nullopt, the NOT-MANAGED case): a test that believed THAT path exercised "unreadable"
// would prove nothing, which is why this one asserts optional("") was actually measured
// before relying on it.
// A managed node whose GetState genuinely never answers is never a GRAPH_NODE_INACTIVE
// concern at all (it was never CONFIRMED non-active) - it is reported under its own
// GRAPH_NODE_UNREADABLE record instead, once the hold expires. Since "a" is the ONLY
// required node here and it is never confirmed anything, GRAPH_NODE_INACTIVE has nothing
// to say about it either way: no FAILED, ever, though its own level-triggered clear may
// eventually flow once the withhold guard's hold on it lapses - what must NEVER happen is
// GRAPH_NODE_UNREADABLE itself reporting the node healthy.
TEST_F(LifecycleExpectationIntegrationTest, ManagedNodeWhoseGetStateNeverAnswersIsReportedUnreadableNotInactive) {
  set_managed_app("a", "/a");
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);  // the two gate.update() calls also run the (failing) GetState seed

  const auto state = gate.lifecycle_state_of("a");
  ASSERT_TRUE(state.has_value()) << "\"a\" was never tracked at all - set_managed_app's services did not "
                                    "make the gate treat it as a managed lifecycle node, so this test "
                                    "would be exercising NOT-MANAGED, not UNREADABLE";
  EXPECT_TRUE(state->empty()) << "the seed unexpectedly succeeded with a real label: " << *state;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into GRAPH_NODE_UNREADABLE past its bound";
  // Baseline captured AFTER the first raise: from here on GRAPH_NODE_UNREADABLE must never
  // clear, however many more ticks pass, since "a" never gets read.
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable), passed_before)
      << "a managed node whose GetState never answers was reported healthy under GRAPH_NODE_UNREADABLE - "
         "an unreadable node must never be cleared, only reported, for as long as it stays unreadable";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised about a node that was never CONFIRMED non-active, only unread";

  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "could not be read", kNodeUnreadable));
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeUnreadable).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeUnreadable), Fault::SEVERITY_WARN)
      << "an unreadable node is an unverified promise, not a confirmed violation, and must not "
         "carry the same severity as a confirmed GRAPH_NODE_INACTIVE";
}

// ---- test-plan row 3: an unreadable node is reported for as long as it stays
// unreadable, with no window and no expiry ----

// The sibling-healing story restated with the OTHER instrument: "a" is a confirmed
// violation that heals almost immediately; "b" is unreadable and never read again for
// the rest of the test. GRAPH_NODE_INACTIVE reflects that "a" - the only OTHER required
// node - has been healthy ever since it healed (proven elsewhere by
// ConfirmedNodeHealsWhileUnreadableSiblingStaysPresentClearsInactiveOnly above); this
// test instead proves GRAPH_NODE_UNREADABLE's own content survives far longer than the
// old, now-deleted content window ever tolerated - many ticks past where it used to
// silently drop out - with no PASSED reaching the service in between.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableNodeStaysReportedWithNoExpiryAsLongAsItStaysUnreadable) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // confirmed violation, heals below
  gate.set_lifecycle_state_for_test("b", "");          // unreadable, NEVER read again in this test
  ASSERT_TRUE(gate.lifecycle_state_of("b").has_value() && gate.lifecycle_state_of("b")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "\"a\" never raised, so nothing below can be attributed to it healing";
  gate.set_lifecycle_state_for_test("a", "active");

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "\"b\"'s unreadable hold never converted into GRAPH_NODE_UNREADABLE";

  // Tick FAR past where the old, now-deleted content window (another kHoldTicks past the
  // hold) would have silently dropped "b" out of the record. "b" is still never read.
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);
  for (int i = 0; i < kHoldTicks * 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeUnreadable))
      << "an unreadable node dropped out of its own record even though it was never read - there is "
         "no expiry on this fault, only on the initial hold";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable), passed_before)
      << "GRAPH_NODE_UNREADABLE cleared while \"b\" was still, and had always been, unreadable";
}

// A genuine read while the node is being reported resets the counter entirely, and a
// LATER unreadable spell must earn a full fresh hold before it is reported again - none
// of the earlier progress may leak across the read.
TEST_F(LifecycleExpectationIntegrationTest, ReadWhileReportedUnreadableResetsAndRequiresAFreshHold) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report, so nothing below is tested";

  // Tick a while longer, then genuinely read the node.
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx, kNodeUnreadable))
      << "the read never cleared GRAPH_NODE_UNREADABLE";

  // Go unreadable again. If the earlier progress had survived the read, this would
  // convert to content again almost immediately; it must instead need a full fresh hold.
  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the re-injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  for (int i = 0; i < kHoldTicks - 5; ++i) {  // comfortably below a FRESH hold on its own
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), failed_before)
      << "the later unreadable spell converted to a report before a FRESH hold elapsed - the "
         "earlier spell's progress leaked across the intervening read";

  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx, kNodeUnreadable))
      << "the fresh spell after the read never converted to a report at all";
}

// Test-plan row 2 (and the config-sweep hold boundary at exactly kUnmeasuredHoldTicks and
// kUnmeasuredHoldTicks + 1): the hold releases INTO a GRAPH_NODE_UNREADABLE report on the
// EXACT tick, never into a GRAPH_NODE_INACTIVE clear one tick early. Driven through the
// injection seam (not set_managed_app's real, variable-latency seeding) so the boundary
// tick is pinned exactly - the same tradeoff BlinkAndUnreadReseedDoNotRestartTheViolationCount
// makes for the tracker's own tick-exact claims.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableHoldReleasesIntoAReportOnTheExactTickNotIntoAClear) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Inject AFTER the last gate.update() (ordering rule 2); never call gate.update()
  // again, so the label stays exactly "" for the rest of the test.
  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value());
  ASSERT_TRUE(gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  for (int i = 0; i < kHoldTicks; ++i) {  // exactly the tolerated hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "the unreadable hold converted into a report before its bound - the boundary must be "
         "exactly kUnmeasuredHoldTicks, not one tick early";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised about a node that was never CONFIRMED non-active, only unread";

  det->tick(ctx);  // the (kHoldTicks + 1)th tick: the exact boundary
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeUnreadable))
      << "the hold outlived its documented bound - it must release into a GRAPH_NODE_UNREADABLE "
         "report exactly one tick past kUnmeasuredHoldTicks";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "could not be read", kNodeUnreadable));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a", kNodeUnreadable));
}

// Test-plan row 4 + row 5's "never before" half: the NOT-MANAGED leg shares the unmeasured
// clock with the UNREADABLE leg (that sharing is the whole point of this slice's redesign
// - a cause-blind clock closes the alternation both used to be individually blind to), so
// past the hold it now converts into its OWN fault code (GRAPH_NODE_NOT_MANAGED) exactly
// as UNREADABLE converts into GRAPH_NODE_UNREADABLE - it no longer releases into silence.
// GRAPH_NODE_INACTIVE's clear still flows (a not-managed node was never a confirmed
// violation, whichever code ends up naming it), and GRAPH_NODE_UNREADABLE must never fire
// for it - the cause is NOT-MANAGED, never UNREADABLE.
TEST_F(LifecycleExpectationIntegrationTest,
       NotManagedEntryPastTheHoldRaisesTheNotManagedCodeNeverInactiveOrUnreadable) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // NO label injection, ever: "a" has no lifecycle services (set_apps, not
  // set_managed_app), so lifecycle_state_of("a") stays nullopt - genuinely NOT-MANAGED.
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "\"a\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";

  // Below the hold: GRAPH_NODE_NOT_MANAGED must never raise before its clock matures.
  for (int i = 0; i < kHoldTicks; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged), 0u)
      << "GRAPH_NODE_NOT_MANAGED raised before its clock crossed the hold";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeNotManaged))
      << "GRAPH_NODE_NOT_MANAGED never raised once the not-managed clock matured past its hold";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a", kNodeNotManaged));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "not a managed lifecycle node", kNodeNotManaged));
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeNotManaged).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeNotManaged), Fault::SEVERITY_WARN)
      << "a not-managed node is an unverified promise, not a confirmed violation";

  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a not-managed entry produced a GRAPH_NODE_INACTIVE FAILED - it was never a confirmed violation";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "a not-managed entry (nullopt) was reported under GRAPH_NODE_UNREADABLE, which is only for "
         "a genuinely tracked lifecycle node whose label is unread (optional(\"\"))";
}

// Shape "Change": a node read once (which resets the unmeasured clock to zero, same as
// any real measurement) that LATER goes unreadable must still eventually surface. There
// is no special-casing needed for this - the unmeasured clock simply climbs from zero
// like a first-contact node's would: a respawned incarnation whose fresh GetState never
// answers is exactly R7's "development is the common case" scenario, and nothing about
// an earlier successful read on a dead incarnation can exempt it.
TEST_F(LifecycleExpectationIntegrationTest, NodeThatWasReadThenGoesUnreadableIsEventuallyReported) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Read once: resets both clocks to zero.
  gate.set_lifecycle_state_for_test("a", "active");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the healthy baseline never cleared, so nothing below proves anything";

  // Now it goes - and stays - unreadable, as if respawned into an incarnation whose
  // GetState never answers.
  gate.set_lifecycle_state_for_test("a", "");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  for (int i = 0; i < kHoldTicks; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), failed_before)
      << "reported before the hold expired";

  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx, kNodeUnreadable))
      << "a node that was read once and later went unreadable never surfaced - an earlier "
         "successful read must not silence a later, genuinely sustained unreadable spell";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "could not be read", kNodeUnreadable));
}

// Test-plan row 5: a read releases everything - the unreadable fault clears, AND the node
// returns to ordinary handling (the tracker's own grace-based tracking), not to some
// third, permanently-exempt state. A node already reported unreadable that is FINALLY read
// must stop being reported under GRAPH_NODE_UNREADABLE and clear normally; from there, if
// it goes non-active again, GRAPH_NODE_INACTIVE must enforce it exactly like any other
// node's ordinary streak - proving the node is back in the tracker's normal care, not
// stuck in some leftover unreadable-adjacent state.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableContentClearsOnceTheNodeIsFinallyRead) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report, so the clear below would prove nothing";

  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeUnreadable);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx, kNodeUnreadable))
      << "an unreadable node that was FINALLY read active never cleared under GRAPH_NODE_UNREADABLE - "
         "content must not be a one-way latch";

  // Ordinary handling resumes: a real read (not another unread spell) that goes non-active
  // is enforced by the tracker's usual grace-based streak, ending in GRAPH_NODE_INACTIVE -
  // never GRAPH_NODE_UNREADABLE, which this node has left behind for good this run.
  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before_inactive = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  const auto unreadable_failed_before_inactive =
      count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before_inactive, *det, ctx))
      << "the node never returned to ordinary GRAPH_NODE_INACTIVE handling after its unreadable "
         "content cleared";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable),
            unreadable_failed_before_inactive)
      << "a node that read active then inactive was reported under GRAPH_NODE_UNREADABLE again, which "
         "it never re-entered";
}

// Shape "Change": reconfiguration while the unreadable clock is live. configure()
// rebuilds the tracker from scratch, so progress toward the hold must not survive a
// reconfigure - reapplying the SAME config must not accidentally convert a below-hold
// clock into a report using ticks counted under the OLD tracker instance.
TEST_F(LifecycleExpectationIntegrationTest, ReconfigureMidUnreadableHoldRestartsTheCountInsteadOfCarryingOverProgress) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  const nlohmann::json config{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}};
  det->configure(config);
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "");

  constexpr int kHalfway = kHoldTicks - 20;  // comfortably below the hold on its own
  for (int i = 0; i < kHalfway; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  det->configure(config);  // rebuilds the tracker: progress so far must not survive this

  // If the reconfigure did NOT reset the count, kHalfway + kHalfway = 2*(kHoldTicks-20)
  // > kHoldTicks would already have converted partway through this second phase. It must
  // not have: the fresh tracker's clock restarts at 0.
  for (int i = 0; i < kHalfway; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "the unreadable count survived the reconfigure - two below-hold phases summed past the "
         "bound instead of each restarting at 0";

  // And the counter is alive, not broken by the reconfigure: it still converts once IT
  // alone crosses the hold.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable counter never converted at all after the reconfigure";
}

// C6: every reconfigure test above calls configure() twice with IDENTICAL json. This one
// actually CHANGES `grace` while a streak is live: "a" accumulates 3 present-and-inactive
// ticks under grace=5 (comfortably below it), then the config shrinks grace to 1.
// configure() rebuilds the tracker from scratch (LifecycleExpectationTracker's own
// misses_ map is gone with the old instance), so the correct behaviour is that the
// 3-tick streak does NOT carry over and does NOT retroactively cross the new, smaller
// grace - a reconfigure is a fresh start, not a re-evaluation of history against a
// changed threshold. The fresh streak under the NEW grace then crosses in 2 ticks, not 5.
TEST_F(LifecycleExpectationIntegrationTest, ReconfigureShrinkingGraceMidStreakRestartsTheStreakUnderTheNewGrace) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  for (int i = 0; i < 3; ++i) {  // comfortably below grace=5
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "raised before crossing the ORIGINAL grace - the setup itself is broken";

  // Shrink grace while the 3-tick streak is live.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the reconfigure retroactively counted the old streak against the new, smaller grace - "
         "a reconfigure must be a fresh start, not a re-evaluation of history";

  // The fresh streak under grace=1 crosses within 2 ticks.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "the new (smaller) grace was never enforced after the reconfigure";
}

// C6's other half: a LIVE pending hold (a below-grace streak the withheld-clear guard is
// currently honouring) must not survive the entry that owns it being dropped from
// require_active entirely. "a" starts a below-grace streak (pending, not yet a fault);
// "b" is healthy. Before this reconfigure, GRAPH_NODE_INACTIVE has emitted nothing at
// all - the pending leg withholds even its clear, per the withheld-clear guard.
// GRAPH_NODE_UNREADABLE is unaffected by that guard (it is independent, see the class
// doc) and legitimately keeps sending its own routine clear every tick since nothing is
// unreadable here, so the instrument below is scoped to GRAPH_NODE_INACTIVE specifically,
// not every request of any kind. Dropping "a" from the config must free "b"'s
// already-healthy state to be reported, not leave the record stuck forever waiting on a
// node no longer checked.
TEST_F(LifecycleExpectationIntegrationTest, ReconfigureDroppingAnEntryWithALivePendingHoldStopsWithholdingForIt) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");  // starts a below-grace (pending) streak
  gate.set_lifecycle_state_for_test("b", "active");    // healthy from the start

  for (int i = 0; i < 3; ++i) {  // "a" pending (3 < grace 5), never crosses
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED) +
                count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED),
            0u)
      << "\"a\"'s live pending hold must withhold even GRAPH_NODE_INACTIVE's clear before the "
         "reconfigure below - if nothing was ever withheld, dropping the entry proves nothing";

  // Drop "a" from require_active entirely while its pending hold is live.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"b"})}, {"grace", 5}});
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "\"a\"'s pending hold survived being dropped from require_active - the record stayed "
         "withheld waiting on a node no longer checked, instead of reflecting that \"b\" (the "
         "only node still required) has been healthy the whole time";
}

// GRAPH_NODE_UNREADABLE's fixed severity, on its own: an unread label is an UNVERIFIED
// promise, not a CONFIRMED violation, and (being non-CRITICAL) must settle through the
// ordinary debounce path rather than bypassing it.
TEST_F(LifecycleExpectationIntegrationTest, OnlyUnreadableContentCarriesWarnSeverity) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "");

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeUnreadable).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeUnreadable), Fault::SEVERITY_WARN)
      << "GRAPH_NODE_UNREADABLE must carry WARN, not the confirmed severity";
}

// Test-plan row 2: the two faults carry different severities, ERROR and WARN, and
// NEITHER changes with the other's content - each is a fixed member (like orphan_detector
// and param_drift_detector's AggregatedFault), not a per-tick choice on one shared record
// the way this used to work before the split.
TEST_F(LifecycleExpectationIntegrationTest, InactiveAndUnreadableSeveritiesAreFixedAndIndependentOfTheOthersContent) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // confirmed violation
  gate.set_lifecycle_state_for_test("b", "");          // unreadable, sustained

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "\"a\" never raised, so nothing below can be attributed to it";
  ASSERT_TRUE(last_failed_severity(kGraphSource).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource), Fault::SEVERITY_ERROR)
      << "GRAPH_NODE_INACTIVE must carry ERROR from the moment it is raised";

  // Let "b" cross its own hold too, so it starts its own, SEPARATE GRAPH_NODE_UNREADABLE
  // record before the independence claim below is tested.
  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(any_failed_desc_contains(kGraphSource, "/b", kNodeUnreadable))
      << "\"b\"'s unreadable hold never converted, so the independence claim below is untested";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeUnreadable).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeUnreadable), Fault::SEVERITY_WARN)
      << "GRAPH_NODE_UNREADABLE must carry WARN";
  ASSERT_TRUE(last_failed_severity(kGraphSource).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource), Fault::SEVERITY_ERROR)
      << "GRAPH_NODE_INACTIVE's severity changed merely because GRAPH_NODE_UNREADABLE started "
         "reporting a sibling node - the two faults must be independent";

  // "a" heals: GRAPH_NODE_INACTIVE clears. "b" is still unreadable throughout, so
  // GRAPH_NODE_UNREADABLE keeps raising WARN, unaffected by the unrelated clear.
  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "GRAPH_NODE_INACTIVE never cleared once its only confirmed violation healed";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeUnreadable).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeUnreadable), Fault::SEVERITY_WARN)
      << "GRAPH_NODE_UNREADABLE's severity changed merely because the unrelated GRAPH_NODE_INACTIVE "
         "cleared";
}

// Scale: more unreadable nodes than the description cap can name - mirrors
// TwentyFiveStuckNodesRaiseOneCappedFault, but for content the guard ages in rather than
// content the tracker raises directly.
TEST_F(LifecycleExpectationIntegrationTest, ManyUnreadableNodesAggregateIntoOneCappedWarnFault) {
  std::vector<std::string> ids;
  nlohmann::json require = nlohmann::json::array();
  for (int i = 0; i < 25; ++i) {
    const std::string id = "a" + std::string(i < 10 ? "0" : "") + std::to_string(i);
    ids.push_back(id);
    require.push_back(id);
  }
  set_apps(ids);
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", 0}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  for (const auto & id : ids) {
    gate.set_lifecycle_state_for_test(id, "");
  }

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable);
  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1, kNodeUnreadable));
  std::this_thread::sleep_for(200ms);

  const std::string desc = last_failed_description(kGraphSource, kNodeUnreadable);
  const std::string marker = "...";
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
      << "the aggregated unreadable description exceeded the cap";
  ASSERT_GE(desc.size(), marker.size());
  EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
      << "a description this far past the cap must end in the truncation marker";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeUnreadable).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeUnreadable), Fault::SEVERITY_WARN)
      << "25 unreadable nodes must still carry WARN, not ERROR";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised about nodes that were never CONFIRMED non-active, only unread";
}

// C1/C3: the tracker's unreadable-detail builder (lifecycle_expectation_tracker.hpp,
// LifecycleExpectationTracker::update()'s matured-unreadable branch) re-applies the
// whole-detail backstop for the same reason the confirmed-violation detail does - the
// fqn and the "required by" list are graph- and config-controlled length - but nothing
// exercised it before this case. With only ONE affected node, AggregatedFault's own
// 480-char cap never engages (one entry at kMaxLifecycleDetailChars=150 is far under
// it), so a bounded description here proves the unreadable detail's OWN trim_to call
// did the work, not a coincidence of the outer cap.
TEST_F(LifecycleExpectationIntegrationTest, PathologicalFqnOnAnUnreadableNodeIsCappedByTheDetailBackstop) {
  const std::string huge_fqn = "/" + std::string(2000, 'n');
  set_apps_raw({{"a", huge_fqn}});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  // The entry must equal the fqn (or its leaf) to match, so this also pins a
  // pathologically long single "required by" entry, not just a long fqn.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({huge_fqn})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "");  // unreadable, sustained
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unreadable hold never converted into a report, so the backstop below is untested";
  const std::string desc = last_failed_description(kGraphSource, kNodeUnreadable);
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::kMaxLifecycleDetailChars)
      << "the unreadable detail's whole-detail backstop did not bound a pathological fqn/entry";
  // The fqn is 2001 chars and appears right after "node ", ahead of "expected active but
  // its lifecycle state could not be read" in the fixed message - so with a fqn this much
  // longer than kMaxLifecycleDetailChars, trim_to's head-keeping behaviour means the fixed
  // tail text CANNOT survive; asserting for it would be asserting something structurally
  // impossible, not proving the backstop works. What DOES prove it: the string starts with
  // the fixed "node " prefix (trim_to keeps the head) and ends in the "..." marker (proving
  // truncation actually fired, not that the detail happened to already fit).
  const std::string marker = "...";
  EXPECT_EQ(desc.find("node "), 0u) << "the fixed prefix did not survive at the head of the trim";
  ASSERT_GE(desc.size(), marker.size());
  EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
      << "a detail this far past the cap must end in the truncation marker, got tail: "
      << desc.substr(desc.size() - std::min<std::size_t>(desc.size(), 20));
}

// A node entering GRAPH_NODE_UNREADABLE's content because its hold just expired must
// order as NEW exactly like a tracker-confirmed violation orders for GRAPH_NODE_INACTIVE
// (ANodeCrossingAfterTheCapIsFullIsNamedOverOlderEntries above) - the operator's question
// is "what changed". Twenty-five "f..." fillers go unreadable together and fill
// GRAPH_NODE_UNREADABLE's own 480-char cap on their own (same shape and count as
// ManyUnreadableNodesAggregateIntoOneCappedWarnFault, proven there to overflow the cap for
// this exact detail string); "zunread" starts READ (active, so its own unmeasured clock
// never moves) and only goes unreadable once the filler batch has already converted -
// needing the REAL ReliabilityGate to produce a matched-but-unread label (optional("")),
// which is why this case is integration-only, unlike the label-trim cases in the tracker
// unit tests.
TEST_F(LifecycleExpectationIntegrationTest, AnUnreadableNodeAgingIntoContentOrdersAsNewLikeAConfirmedOne) {
  std::vector<std::string> ids;
  ids.reserve(25);
  for (int i = 0; i < 25; ++i) {
    ids.push_back("f" + std::string(i < 10 ? "0" : "") + std::to_string(i));
  }
  const std::string late_id = "zunread";  // sorts after every "f..." filler, lexicographically last
  ids.push_back(late_id);

  nlohmann::json require = nlohmann::json::array();
  for (const auto & id : ids) {
    require.push_back(id);
  }
  set_apps(ids);
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  for (const auto & id : ids) {
    if (id != late_id) {
      gate.set_lifecycle_state_for_test(id, "");  // unreadable from tick 1
    }
  }
  gate.set_lifecycle_state_for_test(late_id, "active");  // read and healthy, for now

  // Run the filler batch through its hold so it converts into GRAPH_NODE_UNREADABLE's
  // content together, well before "zunread" is ever touched.
  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeUnreadable));
  {
    const std::string desc = last_failed_description(kGraphSource, kNodeUnreadable);
    const std::string marker = "...";
    ASSERT_GE(desc.size(), marker.size());
    EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
        << "the filler batch alone was sized (25 unreadable nodes, proven to overflow the cap by "
           "ManyUnreadableNodesAggregateIntoOneCappedWarnFault) to exceed the cap, so this "
           "description must be truncated";
    EXPECT_EQ(desc.find("/zunread"), std::string::npos)
        << "\"zunread\" has not gone unreadable yet and must not be named";
  }

  // "zunread" goes unreadable now - the fresh crossing, on a LATER tick, after the cap is
  // already full of filler entries that all sort before it.
  gate.set_lifecycle_state_for_test(late_id, "");
  ASSERT_TRUE(gate.lifecycle_state_of(late_id).has_value() && gate.lifecycle_state_of(late_id)->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  for (int i = 0; i < kHoldTicks; ++i) {  // exactly the tolerated hold, one tick before conversion
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_FALSE(any_failed_desc_contains(kGraphSource, "/zunread", kNodeUnreadable))
      << "still inside \"zunread\"'s own hold - must not be content yet";

  det->tick(ctx);  // the exact crossing tick: converts "zunread" into content for the first time
  std::this_thread::sleep_for(300ms);
  const std::string desc = last_failed_description(kGraphSource, kNodeUnreadable);
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
      << "the aggregated description exceeded the cap";
  EXPECT_EQ(desc.find("node /zunread expected active but its lifecycle state could not be read"), 0u)
      << "the freshly-converted unreadable node must open the description, even though its fqn "
         "sorts LAST lexicographically among every entry. Full description: "
      << desc;
}

// The wire string has no witness anywhere else in this file: every helper and assertion
// above compares against the constant kNodeUnreadable, so a typo inside
// graph_fault_codes.hpp would propagate through the whole suite unnoticed - every test
// would still be comparing the (equally wrong) constant against itself.
// GRAPH_NODE_INACTIVE does not share this exposure: the Python e2e hardcodes
// "GRAPH_NODE_INACTIVE" as its own literal (`FAULT_CODE` in
// test_lifecycle_expectation_e2e.test.py), independent of this header. This is
// GRAPH_NODE_UNREADABLE's equivalent witness: the literal below is hand-typed, never
// read from kNodeUnreadable on either side.
TEST_F(LifecycleExpectationIntegrationTest, UnreadableFaultCodeOnTheWireIsTheLiteralGraphNodeUnreadable) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, "GRAPH_NODE_UNREADABLE"))
      << "no FAILED request ever carried the literal wire string \"GRAPH_NODE_UNREADABLE\" - "
         "either the raise never happened, or kNodeUnreadable no longer matches this literal";
}

// The same witness for the code this slice adds: the literal below is hand-typed, never
// read from kNodeNotManaged on either side, so a typo in graph_fault_codes.hpp would
// propagate through the whole suite unnoticed without this.
TEST_F(LifecycleExpectationIntegrationTest, NotManagedFaultCodeOnTheWireIsTheLiteralGraphNodeNotManaged) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // No injection, ever: "a" has no lifecycle services, so lifecycle_state_of("a") stays
  // nullopt - genuinely NOT-MANAGED.
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "\"a\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";

  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, "GRAPH_NODE_NOT_MANAGED"))
      << "no FAILED request ever carried the literal wire string \"GRAPH_NODE_NOT_MANAGED\" - "
         "either the raise never happened, or kNodeNotManaged no longer matches this literal";
}

// R28, through the REAL gate: a node CONFIRMED inactive (a real measurement, past grace)
// that then goes unreadable long enough to mature must stop being reported under
// GRAPH_NODE_INACTIVE entirely - not merely "not in this tick's affected map", but
// actually CLEAR - because the violation streak is RELEASED the instant the unmeasured
// clock takes ownership. Two prior review passes found the old (pre-redesign) behaviour
// wrong here: the confirmed streak used to be merely held, never released, so
// GRAPH_NODE_INACTIVE's clear stayed withheld forever once a confirmed node went
// unreadable and never came back - this is the live pin that it does not happen anymore.
TEST_F(LifecycleExpectationIntegrationTest, ConfirmedInactiveNodeGoingUnreadableReleasesInactiveAndTransfersOwnership) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeInactive))
      << "\"a\" never confirmed inactive, so nothing below proves a CONFIRMED node's ownership "
         "was ever transferred";

  // Now unreadable, sustained past the hold: the unmeasured clock matures and takes
  // ownership, releasing the violation streak.
  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";
  const auto inactive_passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED, kNodeInactive);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeUnreadable))
      << "the unmeasured clock never matured, so ownership was never transferred";
  EXPECT_TRUE(
      wait_for_count(kGraphSource, ReportFault::Request::EVENT_PASSED, inactive_passed_before + 1, kNodeInactive))
      << "GRAPH_NODE_INACTIVE did not clear once the unmeasured clock matured and took "
         "ownership - the violation streak must be RELEASED, not merely held, once another "
         "code owns the node";

  // Returning to inactive must RE-EARN grace from zero: it must NOT re-cross immediately
  // on the strength of the old, released streak.
  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive);
  det->tick(ctx);  // 1 <= grace(1): must NOT be enough on its own
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive), failed_before)
      << "the returning node re-raised GRAPH_NODE_INACTIVE on its very first inactive tick - "
         "the released streak must re-earn grace from zero, not resume where it left off";
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx, kNodeInactive))
      << "the streak never re-crossed grace at all after genuinely re-earning it";
}

// Test-plan row 2, the not-managed twin of OnlyUnreadableContentCarriesWarnSeverity: an
// unmeasured cause is an UNVERIFIED promise, not a CONFIRMED violation, whichever of the
// two it is.
TEST_F(LifecycleExpectationIntegrationTest, OnlyNotManagedContentCarriesWarnSeverity) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "\"a\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";

  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx, kNodeNotManaged))
      << "the not-managed clock never converted into a report";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeNotManaged).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeNotManaged), Fault::SEVERITY_WARN)
      << "GRAPH_NODE_NOT_MANAGED must carry WARN, not the confirmed severity";
}

// Scale + config sweep, the not-managed twin of ManyUnreadableNodesAggregateIntoOneCappedWarnFault:
// more not-managed nodes than the description cap can name aggregate into ONE capped fault.
TEST_F(LifecycleExpectationIntegrationTest, ManyNotManagedNodesAggregateIntoOneCappedWarnFault) {
  std::vector<std::string> ids;
  nlohmann::json require = nlohmann::json::array();
  for (int i = 0; i < 25; ++i) {
    const std::string id = "a" + std::string(i < 10 ? "0" : "") + std::to_string(i);
    ids.push_back(id);
    require.push_back(id);
  }
  set_apps(ids);  // plain apps, no lifecycle services: every one is genuinely NOT-MANAGED
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", 0}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged);
  for (int i = 0; i < kHoldTicks + 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1, kNodeNotManaged));
  std::this_thread::sleep_for(200ms);

  const std::string desc = last_failed_description(kGraphSource, kNodeNotManaged);
  const std::string marker = "...";
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
      << "the aggregated not-managed description exceeded the cap";
  ASSERT_GE(desc.size(), marker.size());
  EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
      << "a description this far past the cap must end in the truncation marker";
  ASSERT_TRUE(last_failed_severity(kGraphSource, kNodeNotManaged).has_value());
  EXPECT_EQ(*last_failed_severity(kGraphSource, kNodeNotManaged), Fault::SEVERITY_WARN)
      << "25 not-managed nodes must still carry WARN, not ERROR";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised about nodes that were never CONFIRMED non-active, only unmeasured";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "GRAPH_NODE_UNREADABLE raised for nodes that were NOT-MANAGED, never a genuinely tracked "
         "lifecycle node with an unread label";
}

// ---- Absence CONTINUES whatever the node was already doing, through the real gate ----
//
// The three tests below drive the shape a node in a RESTART LOOP produces on a real graph:
// present for one tick, then gone for a run of ticks, forever. That is what a crash-looping
// node looks like from the snapshot - start, crash, respawn delay, start - and it is the
// case this detector most exists to catch, so a clock that absence discards makes exactly
// that node permanently invisible. Each absence run is deliberately LONGER than
// kDefaultAbsenceGrace, i.e. past the blink tolerance, which is where the evidence used to
// be thrown away. The unit tier proves the pacing; these prove what leaves the DETECTOR.

TEST_F(LifecycleExpectationIntegrationTest, UnreadableNodeInARestartLoopIsStillReported) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "");
  ASSERT_TRUE(gate.lifecycle_state_of("a").has_value() && gate.lifecycle_state_of("a")->empty())
      << "the injection did not produce optional(\"\") - this test would not be exercising the "
         "unreadable path";

  bool raised = false;
  for (int cycle = 0; cycle < kHoldTicks + 5 && !raised; ++cycle) {
    set_apps({"a"});
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
    set_apps({});  // ABSENT: gone from the snapshot entirely, not merely unread
    for (int i = 0; i < kDefaultAbsenceGrace + 2 && !raised; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(2ms);
      raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable) > 0;
    }
    raised = raised || count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable) > 0;
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeUnreadable))
      << "a node that is unreadable whenever it is present and absent the rest of the time never "
         "raised GRAPH_NODE_UNREADABLE - every absence run past the grace discarded the evidence, "
         "so a restart loop evades this detector forever";
}

TEST_F(LifecycleExpectationIntegrationTest, NotManagedNodeInARestartLoopIsStillReported) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
      << "\"a\" was unexpectedly tracked - this test would not be exercising NOT-MANAGED";

  bool raised = false;
  for (int cycle = 0; cycle < kHoldTicks + 5 && !raised; ++cycle) {
    set_apps({"a"});
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
    set_apps({});
    for (int i = 0; i < kDefaultAbsenceGrace + 2 && !raised; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(2ms);
      raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged) > 0;
    }
    raised = raised || count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged) > 0;
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeNotManaged))
      << "a node carrying no tracked lifecycle whenever it is present, absent the rest of the time, "
         "never raised GRAPH_NODE_NOT_MANAGED";
}

// A MEASURED not-active read alternating with a NOT-MANAGED one, the two separated by
// absence runs longer than the absence grace. Driven through the REAL gate rather than the
// injection seam, which can only ever SET a label and never remove tracking: toggling
// whether "a" carries GetState/ChangeState services is what produces a genuine nullopt for a
// previously-tracked fqn. Neither the not-managed legs (which never touch the
// violation streak) nor the absence gaps (which hold a below-grace streak rather than
// advancing it) contribute anything on their own - only the repeated MEASURED not-active
// legs do, one real tick at a time - so this still raises, from real evidence accumulated
// across several cycles of presence rather than from any of the gaps in between.
TEST_F(LifecycleExpectationIntegrationTest, InactiveAlternatingWithNotManagedAcrossAbsenceGapsRaisesInactive) {
  set_apps({});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  std::uint64_t tick_counter = 7000;
  bool raised = false;
  for (int cycle = 0; cycle < 20 && !raised; ++cycle) {
    // MEASURED not-active leg: the node is tracked and reads a real label.
    set_managed_app("a", "/a");
    gate.update(snapshot_, ++tick_counter);
    gate.set_lifecycle_state_for_test("a", "inactive");
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);

    // A gap longer than the absence grace: the node is gone from the snapshot entirely.
    set_apps({});
    for (int i = 0; i < kDefaultAbsenceGrace + 2 && !raised; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(2ms);
      raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive) > 0;
    }
    if (raised) {
      break;
    }

    // NOT-MANAGED leg: the node is back but its lifecycle services are gone, so the
    // tracked entry is dropped and lifecycle_state_of() reads nullopt.
    set_apps_raw({{"a", "/a"}});
    gate.update(snapshot_, ++tick_counter);
    ASSERT_FALSE(gate.lifecycle_state_of("a").has_value())
        << "cycle " << cycle << ": the not-managed leg never produced a genuine nullopt";
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);

    set_apps({});
    for (int i = 0; i < kDefaultAbsenceGrace + 2 && !raised; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(2ms);
      raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive) > 0;
    }
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeInactive))
      << "a node alternating between a measured not-active read and a not-managed one, with gaps "
         "longer than the absence grace between them, was never reported under any code";
}

// The false positive this model must not have, at the tier that proves the DETECTOR's
// output: a node measured ACTIVE that then leaves the graph raises nothing at all, however
// long it stays gone. Absence continues what a node was already doing, and a healthy node
// was doing nothing.
TEST_F(LifecycleExpectationIntegrationTest, HealthyNodeThatVanishesRaisesNothingAtAll) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the node was reported while it was present AND active - nothing below would be about absence";

  set_apps({});
  for (int i = 0; i < kHoldTicks + 10; ++i) {  // far past every clock in the tracker
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "GRAPH_NODE_INACTIVE raised for a node that was measured ACTIVE and then shut down - "
         "absence must continue a violation, never start one";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeUnreadable), 0u)
      << "GRAPH_NODE_UNREADABLE raised for a node whose state WAS read, and read as active";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeNotManaged), 0u)
      << "GRAPH_NODE_NOT_MANAGED raised for a node that was a tracked lifecycle node all along";
  EXPECT_EQ(det->tracked_count_for_test(), 0u) << "an idle entry for a departed healthy node was never reclaimed";
}

// Unlike its UNREADABLE and NOT-MANAGED siblings above, GRAPH_NODE_INACTIVE no longer treats
// this shape as an evasion to close on its own: absence contributes nothing to a
// below-grace violation streak (see the tracker's own class doc), so a node that is inactive
// for one tick and then gone for a run, forever, only accumulates evidence from its PRESENT
// ticks - one per cycle here. It is still eventually reported, because it really was measured
// not-active repeatedly; it just now takes grace + 1 cycles of presence rather than grace + 1
// ticks of any kind. Closing the evasion in the ABSENCE itself is GRAPH_NODE_DISAPPEARED's job
// now (this package's own node_death detector), not this streak leaning on a departure it
// cannot corroborate.
TEST_F(LifecycleExpectationIntegrationTest, InactiveNodeInARestartLoopIsStillReported) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 5}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");

  bool raised = false;
  for (int cycle = 0; cycle < kHoldTicks + 5 && !raised; ++cycle) {
    set_apps({"a"});
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
    set_apps({});
    for (int i = 0; i < kDefaultAbsenceGrace + 2 && !raised; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(2ms);
      raised = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive) > 0;
    }
    raised = raised || count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED, kNodeInactive) > 0;
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1, kNodeInactive))
      << "a node measured inactive whenever it is present and absent the rest of the time never "
         "raised GRAPH_NODE_INACTIVE - the violation streak was discarded by every absence run";
}

// ---- configure()-level contract (no ROS scaffolding beyond a logger node) ----

// A key the detector does not read must be reported, or the README's "unknown keys are
// reported" promise is false for this detector. The misspelling chosen is the worst case:
// with `require_activ` the detector is ALSO unconfigured, so the warning must be logged
// before the zero-config early return, not after it.
TEST(LifecycleExpectationConfig, MisspeltKeyIsWarnedAboutOnce) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_misspelt");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_activ", nlohmann::json::array({"a"})}});  // misspelt on purpose
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("unknown config key 'require_activ'"), 1)
      << "expected the misspelt key logged exactly once across three ticks (0 = the sweep is missing "
         "or logged after the zero-config early return; >1 = the once-guard is gone)";
}

// The known-key set must not drift from what configure() reads: a full valid config with
// every documented key plus the plugin-injected ones produces ZERO warnings.
TEST(LifecycleExpectationConfig, EveryKnownKeyIsAcceptedWithoutWarning) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_known");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{
      {"require_active", nlohmann::json::array({"a"})}, {"grace", 3}, {"prune_grace", 10}, {"tick_interval_ms", 100}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "a fully valid config (all known keys + the plugin-injected ones) produced a warning";

  // The low end of the documented range: grace 0 (no tolerance at all) and prune_grace 0
  // (reclaim on the very next absent tick) are both valid, not merely "not yet past the
  // range" - the same endpoint the prune_grace-specific tests exercise via tracked_count,
  // pinned here on the log instead so a grace validator that quietly rejects 0 (an
  // off-by-one on the >= 0 check) cannot hide behind those.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 0}});
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "grace: 0 and prune_grace: 0 are documented, valid low endpoints and must not warn";
}

TEST(LifecycleExpectationConfig, NegativeGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_neg_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", -1}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be an integer in 0..300; keeping the default (5)"), 1)
      << "grace: -1 must warn and name the default that stays in effect";
}

TEST(LifecycleExpectationConfig, NonIntegerGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_str_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", "three"}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be an integer in 0..300"), 1) << "a non-integer grace must warn, not vanish";
}

// prune_grace is plugin-injected and exempt from the unknown-key sweep, so a bad value
// must still be validated here - and on the WIDE integer: 4294967296 truncated through
// get<int>() would arrive as 0 and reclaim bookkeeping on the first absent tick.
TEST(LifecycleExpectationConfig, InvalidPruneGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", -1}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 1);

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 2)
      << "an out-of-int-range prune_grace must be rejected on the wide value, not truncated into "
         "a tiny prune horizon";
}

// `grace` is read off the same 64-bit JSON integer prune_grace is, so it needs the same
// wide range check: get<int>() truncates FIRST, and 4294967296 arrives as 0 - a value that
// passes the >= 0 test and installs the most dangerous setting this key has.
TEST(LifecycleExpectationConfig, WideGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_wide_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be an integer in 0..300; keeping the default (5)"), 1)
      << "a grace past the int range was truncated to 0 and accepted in silence";

  // The negative twin truncates to 0 as well, so the sign check alone cannot catch it.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", -4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be an integer in 0..300; keeping the default (5)"), 2)
      << "a negative grace past the int range truncated to 0 and passed the >= 0 check";
}

// C4: kMaxGrace (lifecycle_expectation_detector.cpp) is checked at configure() time, but
// every existing grace test uses a value chosen to expose the truncation bug the WIDE
// check exists for (2^32, -2^32) - none of them sit at kMaxGrace's own boundary. Both
// documented endpoints and one value past each, since an off-by-one at either end goes
// unnoticed otherwise.
TEST(LifecycleExpectationConfig, GraceRangeEndpoints) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_grace_endpoints");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  const std::string warning = "'grace' must be an integer in 0..300; keeping the default (5)";
  constexpr std::int64_t kMaxGrace = 300;

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "grace 0 is the documented lower endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kMaxGrace}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "kMaxGrace (" << kMaxGrace << ") is the documented upper endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kMaxGrace + 1}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 1) << "the first value past kMaxGrace (" << (kMaxGrace + 1) << ") was accepted";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", -1}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 2) << "the first value below the range was accepted";

  // The value that USED to be the accepted maximum. A streak has to reach `grace` before a
  // node is confirmed, and a node that left the graph while not-active sits in the tracker's
  // pending set until it does - which withholds GRAPH_NODE_INACTIVE's clear for EVERY node
  // meanwhile. At this value that is about 24 days at the shipped 1 s cadence, i.e. the
  // detector is silent either way for longer than any deployment runs uninterrupted.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})},
                                {"grace", static_cast<std::int64_t>(std::numeric_limits<int>::max()) - 1}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 3)
      << "a grace of INT_MAX - 1 was accepted - it neither raises nor heals GRAPH_NODE_INACTIVE for weeks";
}

// The documented prune_grace range endpoints. 3600 is inside the range and must be silent;
// 3601 is the first value outside it and must warn. Without both, an off-by-one at either
// end of the bound goes unnoticed.
TEST(LifecycleExpectationConfig, PruneGraceRangeEndpoints) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune_endpoints");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 3600}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "prune_grace 3600 is the documented upper endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 3601}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 1)
      << "the first value past the documented range was accepted";
}

// The documented tracked_node_cap range, at BOTH endpoints and one value past each. 0 is
// deliberately outside the range: a cap of nothing means the detector checks nothing, which
// is the silence it exists to prevent, so it must warn rather than be clamped up to 1.
TEST(LifecycleExpectationConfig, TrackedNodeCapRangeEndpoints) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_cap_endpoints");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  const std::string warning = "'tracked_node_cap' must be an integer in 1..16384; keeping the default (512)";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 1}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "tracked_node_cap 1 is the documented lower endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 16384}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "tracked_node_cap 16384 is the documented upper endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 0}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 1) << "tracked_node_cap 0 - one below the range - was accepted in silence";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 16385}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 2) << "the first value past the documented range was accepted";

  // The wide-integer check, for the same reason grace and prune_grace need one: get<int>()
  // truncates first, so 2^32 + 1 would arrive as 1 and silently install the tightest cap
  // this key has - one tracked node for the whole fleet.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 4294967297}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 3)
      << "a tracked_node_cap past the int range truncated to 1 and installed a cap of one node";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", "many"}});
  det->tick(ctx);
  EXPECT_EQ(log.count(warning), 4) << "a non-integer tracked_node_cap must warn, not vanish";
}

// The lower endpoint IN FORCE, not merely accepted: at tracked_node_cap 1 exactly one node
// is tracked however many the entry matches. A key that parses and warns correctly but is
// never handed to the tracker would pass TrackedNodeCapRangeEndpoints above and change
// nothing at all.
TEST(LifecycleExpectationConfig, TrackedNodeCapOfOneTracksExactlyOneNode) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 1}});
  // No gate: every matched node reads nullopt, so each carries a live not-managed clock from
  // its first tick and none is ever idle - the state in which the cap is the only bound.
  auto snapshot = IntrospectionInput{};
  for (int i = 0; i < 5; ++i) {
    App app;
    app.id = "ns" + std::to_string(i) + "_a";
    app.bound_fqn = "/ns" + std::to_string(i) + "/a";
    snapshot.apps.push_back(app);
  }
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 1u)
      << "tracked_node_cap 1 was parsed but never reached the tracker - five matched nodes are being "
         "kept where the operator asked for one";
}

// The key raises the bound past the shipped default, which is the whole reason it exists: a
// deployment whose require_active legitimately matches more than 512 PRESENT nodes had no
// lever at all before. Proven at 600 - past the 512 default, affordably short of the 16384
// endpoint, and therefore impossible to pass with the default still in force.
TEST(LifecycleExpectationConfig, TrackedNodeCapAboveTheDefaultActuallyRaisesTheBound) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_cap_raised");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"tracked_node_cap", 600}});
  constexpr int kNodes = 600;
  auto snapshot = IntrospectionInput{};
  for (int i = 0; i < kNodes; ++i) {
    App app;
    app.id = "ns" + std::to_string(i) + "_a";
    app.bound_fqn = "/ns" + std::to_string(i) + "/a";
    snapshot.apps.push_back(app);
  }
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  ctx.snapshot = &snapshot;
  det->tick(ctx);
  EXPECT_EQ(det->tracked_count_for_test(), static_cast<std::size_t>(kNodes))
      << "the configured cap of 600 did not raise the bound - the shipped default of 512 is still in force";
  EXPECT_EQ(log.count("is NOT being checked"), 0)
      << "600 nodes under a configured cap of 600 saturated, so the cap the operator wrote was not the "
         "one being enforced";
}

// A require_active array of non-strings is what a ROS integer_array parameter delivers
// (`require_active: [1, 2]` in YAML). Every entry takes the warn-and-skip arm, and nothing
// may throw out of configure() - the plugin catches a throwing configure() and drops the
// detector entirely, so a typed-wrong list would silently disable the check.
TEST(LifecycleExpectationConfig, NonStringRequireActiveEntriesWarnAndAreSkipped) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_nonstring_entries");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  ASSERT_NO_THROW(det->configure(nlohmann::json{{"require_active", nlohmann::json::array({1, 2})}}));
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' entries must be non-empty node names; skipping one"), 2)
      << "a non-string entry was dropped without a word, so the operator believes a node is covered";
}

// Bounded tracking, through the detector's own config and at SCALE past the shipped cap.
// Every churned identity here is matched with no gate wired, so each reads nullopt and is
// carrying a live not-managed clock from its first tick - which means the age horizon
// reclaims none of them, deliberately (that horizon is exactly what a restart loop evaded).
// The cap is the bound - and it is met by COLLAPSING the entries for the identities that are
// gone, never by refusing the live one: the newcomer here is the only node actually in the
// graph, and refusing it would mean the detector reporting health it declined to check.
TEST(LifecycleExpectationConfig, ChurningIdentitiesCarryingEvidenceAreBoundedByCollapsingTheDeparted) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_churn_cap");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}, {"prune_grace", 2}});
  IntrospectionInput snapshot;
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  ctx.snapshot = &snapshot;  // no gate, no client: the tracker bookkeeping is the observable

  const std::size_t cap = static_cast<std::size_t>(ros2_medkit_graph_watchdog::kDefaultTrackedNodeCap);
  for (std::size_t i = 0; i < cap + 50; ++i) {
    snapshot = namespaced_snapshot_of("ns" + std::to_string(i), "a");
    det->tick(ctx);
    ASSERT_LE(det->tracked_count_for_test(), cap)
        << "iteration " << i
        << ": one bookkeeping entry is retained for every fqn ever seen, so a graph that churns "
           "node identities grows the detector's map without bound";
  }
  EXPECT_GT(det->tracked_count_for_test(), 0u) << "nothing is tracked at all, so nothing was tested";
  EXPECT_EQ(log.count("is NOT being checked"), 0)
      << "the PRESENT node was refused to make room for entries whose nodes are gone - the only node "
         "actually in the graph is the one going unchecked";
}

// Saturation is a real capacity condition, not a permanent latch: it must be reported ONCE
// per episode, and reported AGAIN when a later one happens. A latch spent by the first
// episode makes every subsequent one silent, which is the worse failure - the operator has
// resolved one and has no way of learning about the next.
//
// A cap of one against two PRESENT nodes both carrying evidence is the only shape that
// saturates now that departed entries are collapsed: nothing is idle, nothing is departed,
// so there is genuinely no slot to free. The refused node then LEAVES (ending the episode)
// and comes back (starting a second one).
TEST(LifecycleExpectationConfig, SaturationIsReportedAgainWhenItEndsAndRecurs) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_saturation_rearm");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(
      nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 0}, {"tracked_node_cap", 1}});
  // No gate: both nodes read nullopt, so each carries a live not-managed clock from its
  // first tick and neither is ever idle - the state in which a slot genuinely cannot be freed.
  auto both = snapshot_of({"a", "b"});
  auto only_a = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  ctx.snapshot = &both;
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  ASSERT_EQ(det->tracked_count_for_test(), 1u) << "a cap of one tracked more than one node";
  EXPECT_EQ(log.count("is NOT being checked"), 1) << "the first saturation episode was not reported exactly once";

  // "b" leaves: nothing is refused any more, so the episode ends and the latch re-arms.
  ctx.snapshot = &only_a;
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("is NOT being checked"), 1)
      << "the reason was repeated after the episode ended - it is once per episode, or a long "
         "saturation fills the log with the same line";

  // "b" comes back and is refused again: a SECOND episode, which must be reported.
  ctx.snapshot = &both;
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("is NOT being checked"), 2)
      << "a second, real saturation was silent because the first one spent the only warning - the "
         "operator resolves one capacity problem and never hears about the next";
}

TEST(LifecycleExpectationConfig, NonArrayRequireActiveWarnsAndIsIgnored) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_nonarray");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", "a"}});  // a plain string, not an array
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' must be a string array of node names; ignoring it"), 1)
      << "a non-array require_active was dropped without a word";
}

TEST(LifecycleExpectationConfig, EmptyRequireActiveEntryWarnsAndIsSkipped) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_empty_entry");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", ""})}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' entries must be non-empty node names; skipping one"), 1)
      << "an empty require_active entry was dropped silently - the operator believes a node is covered";
}

// The age horizon reaches IDLE bookkeeping only, and it reaches it at exactly the
// configured prune_grace - no clamp, because there is no longer anything for one to
// protect: an entry carrying evidence is exempt by construction, not by arithmetic. The
// tightest endpoint first (prune_grace 0 with grace 0: reclaimed on the FIRST absent tick),
// against a node whose lifecycle read is "active" so it is genuinely idle. The gate is
// wired here precisely because a detector with no gate reads every node as not-managed,
// which is never idle.
TEST(LifecycleExpectationConfig, IdleEntryIsReclaimedAtExactlyPruneGraceZero) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune_zero");
  std::mutex node_mutex;
  ReliabilityGate gate(/*warmup_cycles=*/1, node.get(), &node_mutex);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 0}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  ctx.gate = &gate;
  gate.set_lifecycle_state_for_test("a", "active");
  det->tick(ctx);
  ASSERT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  det->tick(ctx);  // absent 1 > prune_ticks(0): reclaimed at once
  EXPECT_EQ(det->tracked_count_for_test(), 0u)
      << "prune_grace 0 must mean 0, not a clamped-up grace + 1 - the value the operator wrote is "
         "the horizon an idle entry gets";
}

// The smallest POSITIVE prune_grace, which neither documented endpoint sweeps: 0 and 4 both
// pass an implementation that special-cases 1, rounds a positive value down to 0, or tests
// `<= prune_grace` instead of `>`. At 1 the idle entry must survive absent tick 1 exactly and
// be reclaimed on absent tick 2.
TEST(LifecycleExpectationConfig, IdleEntryIsReclaimedOnTheSecondAbsentTickAtPruneGraceOne) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune_one");
  std::mutex node_mutex;
  ReliabilityGate gate(/*warmup_cycles=*/1, node.get(), &node_mutex);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 1}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  ctx.gate = &gate;
  gate.set_lifecycle_state_for_test("a", "active");  // measured healthy, so the entry is genuinely IDLE
  det->tick(ctx);
  ASSERT_EQ(det->tracked_count_for_test(), 1u);

  snapshot.apps.clear();
  det->tick(ctx);  // absent 1, NOT past prune_ticks(1)
  EXPECT_EQ(det->tracked_count_for_test(), 1u)
      << "prune_grace 1 reclaimed on the first absent tick - the operator asked for one tick of "
         "tolerance and got none";
  det->tick(ctx);  // absent 2 > prune_ticks(1): reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u)
      << "prune_grace 1 kept the idle entry past the tick it was written to expire on";
}

// The other endpoint of the documented range, and the shape an operator actually writes: a
// wider prune_grace keeps an idle entry for exactly that many absent ticks and drops it on
// the next one.
TEST(LifecycleExpectationConfig, IdleEntrySurvivesExactlyPruneGraceAbsentTicks) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune_four");
  std::mutex node_mutex;
  ReliabilityGate gate(/*warmup_cycles=*/1, node.get(), &node_mutex);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 4}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  ctx.gate = &gate;
  gate.set_lifecycle_state_for_test("a", "active");
  det->tick(ctx);
  ASSERT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  for (int i = 0; i < 4; ++i) {
    det->tick(ctx);  // absent 1..4, none past prune_ticks(4)
  }
  EXPECT_EQ(det->tracked_count_for_test(), 1u) << "pruned before prune_grace absent ticks elapsed";
  det->tick(ctx);  // absent 5 > prune_ticks: reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
}

// Config sweep at the tightest endpoint the documented space allows - `grace: 0` together
// with `prune_grace: 0`, both at the bottom of their ranges. A node carrying a LIVE clock
// (here the not-managed one: no gate is wired, so lifecycle_state_of() is never consulted
// and the node reads nullopt from its first tick) must survive that horizon: reclaiming
// bookkeeping by AGE erases the evidence the node earned, and at this endpoint the age is
// one tick, so the node's clock would restart from zero every time it blinked.
TEST(LifecycleExpectationConfig, LiveClockIsNotErasedAtTheTightestGraceAndPruneGraceEndpoint) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 0}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;  // no gate, no client: the tracker bookkeeping is the observable
  det->tick(ctx);
  ASSERT_EQ(det->tracked_count_for_test(), 1u) << "sanity: the node must be tracked before it vanishes";

  snapshot.apps.clear();
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    EXPECT_EQ(det->tracked_count_for_test(), 1u)
        << "absent tick " << (i + 1)
        << ": a node carrying a live clock was erased by the prune horizon at grace: 0, "
           "prune_grace: 0 - its evidence is gone, so a node that touches absence periodically "
           "can never accumulate enough of it to be reported";
  }
}

// The combination the old clamp existed for - a wide `grace` next to the tightest
// `prune_grace` - is where a below-grace streak's two guarantees have to meet: it must not be
// reclaimed as IDLE (it is not - violation_streak != 0 - even though it is small), and it must
// not be silently matured by the absence run either (only a present tick may still advance it
// - see the tracker's own class doc). So the entry survives, HELD, exactly where it was: never
// confirmed while the node stays gone, never pruned either, and ready to resume - not restart
// - the moment the node returns. It is a fixture test rather than a bare configure()-level one
// because only the fake ReportFault sink can see the (absence of a) raise.
//
// The node is armed (read "active") once before anything else: this claim holds only for a
// node the presence detector could itself have reported (node_death tracks any node the gate
// has armed at least once), so the fixture has to BE one, or the absence run below would be
// proving something about a different node than the one this test names. A node that never
// arms gets absence-driven maturation instead - see
// NeverArmedBelowGraceStreakMaturesFromAbsenceInsteadOfHoldingIndefinitely below for that one.
TEST_F(LifecycleExpectationIntegrationTest, BelowGraceStreakSurvivesTheTightestPruneGraceWithoutConfirmingWhileAbsent) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}, {"prune_grace", 0}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the healthy baseline never cleared, so the node below cannot be shown to have armed";
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // streak 1, well below grace(4)
  ASSERT_EQ(det->tracked_count_for_test(), 1u);
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the node was confirmed on its first not-active tick, so grace(4) was never in force and "
         "the absence run below proves nothing about a below-grace streak";

  // "a" vanishes with a streak of 1 out of 4, at the tightest prune_grace (0) - an IDLE entry
  // would be reclaimed on the very next absent tick.
  set_apps({});
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "GRAPH_NODE_INACTIVE was confirmed from a below-grace streak while the node was absent the "
         "whole time - a fault born from evidence gathered while nobody could observe the node";
  ASSERT_EQ(det->tracked_count_for_test(), 1u)
      << "the entry was reclaimed by the tightest prune_grace despite carrying a live (if below-grace) "
         "streak - is_idle() must not treat a non-zero streak as nothing to lose";

  // It resumes rather than restarts: held at 1, three more present ticks reach grace(4), the
  // fourth crosses it - due on that exact tick and no later, so a streak that had restarted
  // from zero (needing a fifth) would still read grace here, not past it.
  set_apps({"a"});
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);  // resumed streak 2, 3, 4 (== grace, not past it)
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "resumed streak reached grace(4) but was already confirmed one tick early";
  det->tick(ctx);  // resumed streak 5 > grace: due on this exact tick
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1))
      << "resumed streak 5 > grace(4) was not confirmed on this exact tick - the streak restarted "
         "from zero on return instead of resuming where the absence had held it";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "a"))
      << "the confirmation did not name the node once it finally raised";
}

// The row the test immediately above cannot cover: a node that is NEVER armed, because it
// never once reads "active" - exactly a require_active node that comes up unconfigured and
// stays there. node_death only ever tracks a key the reliability gate has armed at least
// once, so nothing else in the plugin can ever report this node's departure; through the
// real gate and the real (fake-service-backed) fault client, a below-grace streak on such a
// node must still mature from absence alone, or the departure is reported by nothing at all.
// Same grace, same streak-at-departure, same fixture shape as
// BelowGraceStreakSurvivesTheTightestPruneGraceWithoutConfirmingWhileAbsent immediately
// above, with arming the only difference - together the two rows prove the split is keyed on
// arming and nothing else.
// The gate has no notion of `is_online`, but node_death does: it skips an app that is not
// online before it ever consults the gate. So an OFFLINE app can be armed, can be owned by the
// gate's own answer (it carries no lifecycle record, and a node with no record is the presence
// detector's outright), and can still be one node_death will never track. Latching that answer
// makes this detector hand a departure to a detector that structurally cannot report it - the
// same silence a permanently unmeasured node used to fall into.
//
// Staged in the order that reaches it: the app is offline and unmeasured while the gate arms
// it (the window where a naive `armed` latches), then reads inactive, then vanishes below
// grace. Nothing else can report that departure, so absence has to mature the violation.
TEST_F(LifecycleExpectationIntegrationTest, OfflineAppIsNeverTreatedAsOwnedByThePresenceDetector) {
  set_offline_app("a");
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // The window a naive `armed` latches in: armed by the gate, no lifecycle record at all, so
  // the gate's ownership answer is TRUE for an app node_death has already skipped.
  ASSERT_TRUE(gate.allows_raise("a")) << "the app was never armed, so the latch window this test "
                                         "is about was never open";
  ASSERT_EQ(gate.presence_ownership("a"), ros2_medkit_graph_watchdog::PresenceOwnership::kEarned)
      << "the gate must still say it OWNS an app with no lifecycle record, and own it OUTRIGHT - "
         "if it did not, this test would pass for a reason that has nothing to do with is_online";
  det->tick(ctx);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // streak 1, well below grace(4)
  ASSERT_EQ(det->tracked_count_for_test(), 1u);
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the node was confirmed already, so grace(4) was never in force and the absence run "
         "below proves nothing about a below-grace streak";

  snapshot_.apps.clear();
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a below-grace violation on an OFFLINE app did not mature from absence - node_death "
         "skips an app that is not online, so this departure is reported by nothing at all";
}

TEST_F(LifecycleExpectationIntegrationTest, NeverArmedBelowGraceStreakMaturesFromAbsenceInsteadOfHoldingIndefinitely) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Never armed: "a" reads non-active from the very first tick and never once reads "active",
  // so LifecycleWatcher::node_ok() is false for its whole life here and the gate never arms
  // it.
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // streak 1, well below grace(4)
  ASSERT_EQ(det->tracked_count_for_test(), 1u);
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the node was confirmed on its first not-active tick, so grace(4) was never in force and "
         "the absence run below proves nothing about a below-grace streak";

  // "a" vanishes with a streak of 1 out of 4. Nothing else will ever report this departure,
  // so absence itself has to mature the streak.
  set_apps({});
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a below-grace violation on a node the presence detector could never have tracked did "
         "not mature from absence alone - its departure is reported by nothing at all";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "has since left the graph"))
      << "the fault matured from absence must say the node has since left the graph, or the "
         "operator is sent looking for a node that is not there";
}
