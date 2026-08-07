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
// Drives the REAL mechanism: a target node exposes parameters over DDS, the detector
// reads them via Ros2ParameterTransport, and raises/clears through a real ReportFault
// service round-trip to a fake fault_manager. NOT a full-gateway e2e; this proves the
// detector end to end within its own scope.
//
// The detector is snapshot-driven: tick() early-returns unless ctx.snapshot is set, iterates
// ctx.snapshot->apps (id + fqn), and raises ONE aggregated fault whose source_id is always
// graph_source_id() - the constant "graph_watchdog", the entity the plugin owns. It never
// depends on what the snapshot contains. So every test seeds the snapshot via set_apps() and
// asserts on the "graph_watchdog" source, never a per-node FQN and never a component id.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::AggregatedFault;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::reliability_allows;
using ros2_medkit_graph_watchdog::ReliabilityGate;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in param_drift_detector.cpp but self-registers via
// REGISTER_DETECTOR, which runs when that .cpp is linked into this test. Pull an
// instance from the registry (no production factory needed).
std::unique_ptr<ros2_medkit_graph_watchdog::Detector> make_param_drift() {
  for (auto & d : ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all()) {
    if (d->id() == "param_drift") {
      return std::move(d);
    }
  }
  return nullptr;
}

/// Runs `fn` when it leaves scope, on EVERY exit path - including the one a failed ASSERT takes.
///
/// The teardown this file needs is not optional cleanup. A node removed from the fixture's
/// executor only on the success path is still registered with a RUNNING MultiThreadedExecutor when
/// an ASSERT above destroys it, and the process SIGABRTs on a dangling callback group: a readable
/// assertion failure turns into a crash that says nothing about what went wrong. The same applies
/// to a joinable std::thread, where the failure mode is std::terminate.
template <typename FnT>
class ScopeExit {
 public:
  explicit ScopeExit(FnT fn) : fn_(std::move(fn)) {
  }
  ~ScopeExit() {
    // A destructor must not throw: an escape here during stack unwinding from a failed assertion
    // is std::terminate, which is strictly worse than the leak this guard exists to prevent.
    try {
      fn_();
    } catch (const std::exception &) {
      // Swallowed on purpose, per the note above.
    }
  }
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&) = delete;
  ScopeExit & operator=(ScopeExit &&) = delete;

 private:
  FnT fn_;
};

template <typename FnT>
ScopeExit<FnT> on_scope_exit(FnT fn) {
  return ScopeExit<FnT>(std::move(fn));
}

/// Captures rcutils log output for as long as it is alive and restores the console handler on every
/// exit path. Leaving the process-global handler installed would swallow the log output of every
/// later case in this binary, so the restore cannot be left to the success path.
///
/// One implementation for the whole file. The rcutils handler CONTRACT hands over a caller-supplied
/// format string and a va_list - it is not a choice the handler makes - so `vsnprintf` here is the
/// one place in this file that cannot use a literal format. Written out per test, that was six
/// copies of the suppression and six guard types; written once, it is one.
///
/// The handler is a plain C function pointer with no user-data slot, so the live capture is reached
/// through a file-static. It is written by the test thread and read by whichever thread logs (the
/// detector's reader thread does), hence the atomic.
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
/// that shape once here keeps it out of every fake service below.
template <typename ServiceT, typename BodyT>
std::function<void(std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
service_callback(BodyT body) {
  return [body = std::move(body)](std::shared_ptr<typename ServiceT::Request> req,
                                  std::shared_ptr<typename ServiceT::Response> resp) {
    body(*req, *resp);
  };
}

/// The four parameter services a hand-rolled probe node needs beyond list/get, purely so the
/// transport's `AsyncParametersClient` reports ready.
///
/// wait_for_service() on that client waits for ALL SIX; a node missing any of them is answered
/// SERVICE_UNAVAILABLE and negative-cached before a single request reaches its handlers. A test
/// about what a READ costs or how long one may take is then measuring a node that was never read at
/// all - which is exactly how a first attempt at the per-read-bound case below passed for the wrong
/// reason. These four only ever have to exist; nothing in this file drives them.
std::vector<rclcpp::ServiceBase::SharedPtr> add_idle_parameter_services(const rclcpp::Node::SharedPtr & node,
                                                                        const std::string & fqn) {
  return {
      node->create_service<rcl_interfaces::srv::DescribeParameters>(
          fqn + "/describe_parameters", service_callback<rcl_interfaces::srv::DescribeParameters>(
                                            [](const rcl_interfaces::srv::DescribeParameters::Request & req,
                                               rcl_interfaces::srv::DescribeParameters::Response & resp) {
                                              for (const auto & name : req.names) {
                                                rcl_interfaces::msg::ParameterDescriptor d;
                                                d.name = name;
                                                d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                                                resp.descriptors.push_back(d);
                                              }
                                            })),
      node->create_service<rcl_interfaces::srv::GetParameterTypes>(
          fqn + "/get_parameter_types", service_callback<rcl_interfaces::srv::GetParameterTypes>(
                                            [](const rcl_interfaces::srv::GetParameterTypes::Request & req,
                                               rcl_interfaces::srv::GetParameterTypes::Response & resp) {
                                              resp.types.assign(req.names.size(),
                                                                rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
                                            })),
      node->create_service<rcl_interfaces::srv::SetParameters>(
          fqn + "/set_parameters", service_callback<rcl_interfaces::srv::SetParameters>(
                                       [](const rcl_interfaces::srv::SetParameters::Request & req,
                                          rcl_interfaces::srv::SetParameters::Response & resp) {
                                         for (std::size_t i = 0; i < req.parameters.size(); ++i) {
                                           rcl_interfaces::msg::SetParametersResult r;
                                           r.successful = false;
                                           r.reason = "probe is read-only";
                                           resp.results.push_back(r);
                                         }
                                       })),
      node->create_service<rcl_interfaces::srv::SetParametersAtomically>(
          fqn + "/set_parameters_atomically",
          service_callback<rcl_interfaces::srv::SetParametersAtomically>(
              [](const rcl_interfaces::srv::SetParametersAtomically::Request & /*req*/,
                 rcl_interfaces::srv::SetParametersAtomically::Response & resp) {
                resp.result.successful = false;
                resp.result.reason = "probe is read-only";
              })),
  };
}
}  // namespace

/// A parameter transport under the test's control. The detector's reads are blocking service
/// round-trips against a live node, which makes three behaviours unreachable from a real ROS graph:
/// a read that is still in flight when its app is de-armed, a SUCCESS response that carries no
/// parameters at all, and a transport that cannot be constructed. Each of those guards a real
/// failure mode, so each needs a stand-in to be provable.
class ScriptedTransport : public ros2_medkit_gateway::ParameterTransport {
 public:
  bool is_self_node(const std::string & /*node*/) const override {
    return false;
  }
  ros2_medkit_gateway::ParameterResult list_parameters(const std::string & node) override {
    throw_if_scripted(node);
    if (is_unreadable(node)) {
      ros2_medkit_gateway::ParameterResult r;
      r.success = false;  // a node with no working parameter service - no live node can be made to
      r.error_code = ros2_medkit_gateway::ParameterErrorCode::SERVICE_UNAVAILABLE;
      return r;  // answer this way on demand, which is why the stand-in has to
    }
    enter_read(node);
    ros2_medkit_gateway::ParameterResult r;
    r.success = true;
    {
      // Under `m_`, all of it. enter_read() releases the lock when it returns, so reading
      // empty_batch_ out here is a plain bool racing set_empty_batch() on the test thread against
      // the reader thread - which TSan fails the whole integration test for, and which is a real
      // race and not a test artefact: the stand-in stands in for a transport the reader calls
      // concurrently with the test reconfiguring it.
      std::lock_guard<std::mutex> lk(m_);
      if (empty_batch_) {
        r.data = nlohmann::json::array();  // SUCCESS carrying no view - a legitimate transport reply
        return r;
      }
      const auto it = node_params_.find(node);
      if (it != node_params_.end()) {
        r.data = nlohmann::json::array();
        for (const auto & [name, value] : it->second) {
          r.data.push_back(nlohmann::json{{"name", name}, {"value", value}, {"type", "double"}});
        }
        return r;
      }
    }
    r.data = nlohmann::json::array({{{"name", "gain"}, {"value", value_for(node)}, {"type", "double"}}});
    return r;
  }
  ros2_medkit_gateway::ParameterResult get_parameter(const std::string & node, const std::string & name) override {
    throw_if_scripted(node);
    if (is_unreadable(node)) {
      ros2_medkit_gateway::ParameterResult r;
      r.success = false;
      r.error_code = ros2_medkit_gateway::ParameterErrorCode::SERVICE_UNAVAILABLE;
      return r;
    }
    enter_read(node);
    {
      std::lock_guard<std::mutex> lk(m_);
      if (restrict_declared_) {
        const auto per_node = node_declared_.find(node);
        const auto & names = per_node != node_declared_.end() ? per_node->second : declared_;
        if (names.count(name) == 0) {
          ros2_medkit_gateway::ParameterResult r;
          r.success = false;
          r.error_code = ros2_medkit_gateway::ParameterErrorCode::NOT_FOUND;
          return r;
        }
      }
    }
    ros2_medkit_gateway::ParameterResult r;
    r.success = true;
    r.data = nlohmann::json{{"name", name}, {"value", value_for(node)}, {"type", "double"}};
    return r;
  }
  ros2_medkit_gateway::ParameterResult set_parameter(const std::string & /*node*/, const std::string & /*name*/,
                                                     const nlohmann::json & /*value*/) override {
    return {};
  }
  ros2_medkit_gateway::ParameterResult list_own_parameters() override {
    return {};
  }
  ros2_medkit_gateway::ParameterResult get_own_parameter(const std::string & /*name*/) override {
    return {};
  }
  ros2_medkit_gateway::ParameterResult get_default(const std::string & /*node*/,
                                                   const std::string & /*name*/) override {
    return {};
  }
  ros2_medkit_gateway::ParameterResult list_defaults(const std::string & /*node*/) override {
    return {};
  }
  bool is_node_available(const std::string & /*node*/) const override {
    return true;
  }
  void shutdown() override {
    std::lock_guard<std::mutex> lk(m_);
    blocked_ = false;  // never let a teardown wait on a gate the test forgot to open
    blocked_nodes_.clear();
    gate_.notify_all();
  }

  void block() {
    std::lock_guard<std::mutex> lk(m_);
    blocked_ = true;
  }
  void release() {
    std::lock_guard<std::mutex> lk(m_);
    blocked_ = false;
    gate_.notify_all();
  }
  /// Hold reads of ONE node. The reader thread is single-threaded, so blocking everything stops
  /// the whole loop: a test that needs to observe the reader moving past a held read has to leave
  /// the other targets running.
  void block_node(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    blocked_nodes_.insert(node);
  }
  void release_node(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    blocked_nodes_.erase(node);
    gate_.notify_all();
  }
  /// Wait until the reader is actually inside a read, so the test never races the thread.
  bool wait_until_reading(int at_least) {
    std::unique_lock<std::mutex> lk(m_);
    return entered_.wait_for(lk, 5s, [&] {
      return calls_ >= at_least;
    });
  }
  /// Wait until a read is actually HELD on the gate. The call counter cannot express this: a read
  /// that entered just before block_node() is counted but runs to completion, so a test keying off
  /// the count can proceed while nothing is held at all.
  bool wait_until_parked() {
    std::unique_lock<std::mutex> lk(m_);
    return parked_cv_.wait_for(lk, 5s, [this] {
      return parked_ > 0;
    });
  }
  /// Default value for every node.
  void set_value(double v) {
    value_.store(v);
  }
  /// Override the value of ONE node. Needed whenever a test drifts a single app: the shared value
  /// would drift every target at once, and the assertion would then pass on the wrong app.
  void set_value(const std::string & node, double v) {
    std::lock_guard<std::mutex> lk(m_);
    node_values_[node] = v;
  }
  /// Replace the whole parameter set ONE node answers `list_parameters` with. Left unset, the
  /// stand-in reports a single `gain`, which is enough to drift a node but nowhere near enough to
  /// fill the detector's per-app share of the aggregated description - so a test about which
  /// entries survive that cap cannot be written with it at all. Per-node for the same reason
  /// set_value and set_declared are: one shared set makes every node answer alike, and the point
  /// here is that different nodes contribute different amounts of text.
  void set_params(const std::string & node, std::map<std::string, nlohmann::json> params) {
    std::lock_guard<std::mutex> lk(m_);
    node_params_[node] = std::move(params);
  }
  /// Make one node permanently unreadable, whatever else the stand-in does.
  void set_unreadable(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    unreadable_.insert(node);
  }
  /// Undo set_unreadable. A test that proves an unreadable node cannot CLEAR a fault needs the
  /// positive control that a readable one still can, or the silence it asserts is satisfied by a
  /// detector that stopped emitting anything at all.
  void set_readable(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    unreadable_.erase(node);
  }
  /// Make one node's reads THROW. rclcpp can throw out of a read when the context is invalidated
  /// mid-call, or on a malformed service path; a live node cannot be asked to do it on demand.
  void set_throwing(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    throwing_.insert(node);
  }
  bool wait_until_threw(int at_least) {
    std::unique_lock<std::mutex> lk(m_);
    return threw_.wait_for(lk, 5s, [&] {
      return throws_ >= at_least;
    });
  }
  /// How many reads have been REJECTED because their node is unreadable. `calls()` cannot express
  /// this: an unreadable node fails before enter_read, so it is never counted there. A test whose
  /// subject is a THRESHOLD on failed attempts has to pace itself off the attempts themselves -
  /// keying off wall clock instead makes the assertion a function of the pacing budget, which is
  /// the very coupling this counter exists to let a test pin.
  int failures() {
    std::lock_guard<std::mutex> lk(m_);
    return failures_;
  }
  bool wait_until_failed(int at_least) {
    std::unique_lock<std::mutex> lk(m_);
    return failed_.wait_for(lk, 30s, [&] {
      return failures_ >= at_least;
    });
  }
  void set_empty_batch(bool on) {
    std::lock_guard<std::mutex> lk(m_);
    empty_batch_ = on;
  }
  /// Restrict which parameter names the node admits to having. Left unset, the stand-in answers
  /// any name, which is precisely the behaviour no real node has: a real one returns NOT_FOUND for
  /// anything it never declared, and that reply is deliberately not treated as a drift.
  void set_declared(std::set<std::string> names) {
    std::lock_guard<std::mutex> lk(m_);
    declared_ = std::move(names);
    restrict_declared_ = true;
  }
  /// Restrict which names ONE node admits to having, overriding the shared set for that node.
  /// Same reason set_value has a per-node form: with a single shared set every node answers alike,
  /// so a test about one node declaring a pin that the rest of the graph does not cannot be
  /// written at all - and "the sweep has not reached the node that declares it yet" is exactly the
  /// state the unmatched-pin warning must not fire in.
  void set_declared(const std::string & node, std::set<std::string> names) {
    std::lock_guard<std::mutex> lk(m_);
    node_declared_[node] = std::move(names);
    restrict_declared_ = true;
  }
  int calls() {
    std::lock_guard<std::mutex> lk(m_);
    return calls_;
  }

 private:
  std::mutex m_;
  std::condition_variable gate_;
  std::condition_variable entered_;
  std::condition_variable parked_cv_;
  std::condition_variable threw_;
  std::condition_variable failed_;
  bool blocked_ = false;
  bool empty_batch_ = false;
  std::set<std::string> unreadable_;
  std::set<std::string> blocked_nodes_;
  std::set<std::string> throwing_;
  std::set<std::string> declared_;
  std::map<std::string, std::set<std::string>> node_declared_;
  bool restrict_declared_ = false;
  std::map<std::string, double> node_values_;
  std::map<std::string, std::map<std::string, nlohmann::json>> node_params_;
  int calls_ = 0;
  int parked_ = 0;
  int throws_ = 0;
  int failures_ = 0;
  std::atomic<double> value_{1.0};

  bool is_unreadable(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    if (unreadable_.count(node) == 0) {
      return false;
    }
    ++failures_;
    failed_.notify_all();
    return true;
  }

  /// Throw out of the read itself, before anything is counted as a call. The throw has to leave
  /// the transport the way a real one would - from inside the blocking call, not around it.
  void throw_if_scripted(const std::string & node) {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (throwing_.count(node) == 0) {
        return;
      }
      ++throws_;
      threw_.notify_all();
    }
    throw std::runtime_error("scripted transport failure");
  }

  /// Count the call, then hold it for as long as the test blocks this node. `parked_` is
  /// incremented only on the path that actually waits, so wait_until_parked() means a read is
  /// held right now rather than merely started.
  void enter_read(const std::string & node) {
    std::unique_lock<std::mutex> lk(m_);
    ++calls_;
    entered_.notify_all();
    const auto held = [this, &node] {
      return blocked_ || blocked_nodes_.count(node) != 0;
    };
    if (!held()) {
      return;
    }
    ++parked_;
    parked_cv_.notify_all();
    gate_.wait(lk, [&held] {
      return !held();
    });
    --parked_;
  }

  /// Read AFTER the gate, so a value the test sets while a read is held is the one that read
  /// returns - which is how a stale in-flight sample is scripted.
  double value_for(const std::string & node) {
    std::lock_guard<std::mutex> lk(m_);
    const auto it = node_values_.find(node);
    return it != node_values_.end() ? it->second : value_.load();
  }
};

class ParamDriftIntegrationTest : public ::testing::Test {
 protected:
  // Must run before the FIRST test's fixture is constructed: `exec_` is a fixture data
  // member, so its default constructor (which touches the global rclcpp context to
  // create an interrupt guard condition) runs before SetUp(). Doing rclcpp::init() only
  // in SetUp() is too late for that first construction; SetUpTestSuite() runs earlier.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    gateway_ = std::make_shared<rclcpp::Node>("pd_it_gateway");
    target_ = std::make_shared<rclcpp::Node>("pd_it_target");
    target_->declare_parameter<double>("speed_limit", 1.0);
    sink_ = std::make_shared<rclcpp::Node>("pd_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        service_callback<ReportFault>([this](const ReportFault::Request & req, ReportFault::Response & resp) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            received_.push_back(req);
          }
          resp.accepted = true;  // ReportFault.srv response field is `bool accepted`
        }));
    client_ = gateway_->create_client<ReportFault>("/fault_manager/report_fault");
    exec_.add_node(gateway_);
    exec_.add_node(target_);
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
    exec_.remove_node(target_);
    exec_.remove_node(sink_);
    param_clients_.clear();  // safe here: the executor has stopped spinning, nothing else can touch these
    client_.reset();
    srv_.reset();
    sink_.reset();
    target_.reset();
    gateway_.reset();
  }

  DetectorContext make_ctx(DetectorMode mode) {
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.node_mutex = &node_mutex_;
    ctx.mode = mode;
    ctx.gate = nullptr;  // ungated by default; the gate-keying test wires a real gate in
    ctx.fault_client = client_;
    ctx.snapshot = &snapshot_;  // tick() early-returns without a snapshot; each test seeds set_apps()
    return ctx;
  }

  // (Re)populate the owned entity snapshot the detector reads each tick. Uses the
  // `App a; a.field = ...;` idiom (App has a RosBinding sub-struct, so designated/partial
  // init trips -Wmissing-field-initializers under our strict flags).
  //
  // The aggregated fault's source_id does NOT depend on this snapshot: it is always
  // graph_source_id(), the entity the plugin owns. SourceIdIgnoresTheHostComponent below seeds
  // a component precisely to prove that, because a host Component id would match no entity
  // scope and the fault would be reachable from no endpoint at all.
  void set_apps(const std::vector<std::pair<std::string, std::string>> & id_fqn) {
    snapshot_.apps.clear();
    for (const auto & [id, fqn] : id_fqn) {
      App a;
      a.id = id;
      a.bound_fqn = fqn;
      snapshot_.apps.push_back(a);
    }
  }

  // Wait until <node_fqn>'s parameter service is discoverable, so the detector's first
  // read succeeds and captures the baseline BEFORE any set_parameter. Without this, an
  // early negative-cache miss (service not yet discovered) could make the first
  // successful read land after the drift and capture the drifted value as the baseline.
  //
  // The client is cached per service name and kept alive in `param_clients_` instead of being
  // created and dropped on every call. The executor is spinning on `spin_` for the whole test
  // and holds its own shared_ptr to the client while it is in flight, so the last reference is
  // often released on an executor thread rather than here. `~Client()` reads the node's
  // internal rcutils_hash_map, and a throwaway client destroyed while the next `create_client`
  // call writes that same map is a data race. Caching defers destruction to TearDown(), which
  // only runs after the executor has stopped spinning.
  bool wait_param_service(const std::string & node_fqn) {
    std::lock_guard<std::mutex> lk(node_mutex_);
    const std::string service_name = node_fqn + "/get_parameters";
    auto & c = param_clients_[service_name];
    if (!c) {
      c = gateway_->create_client<rcl_interfaces::srv::GetParameters>(service_name);
    }
    return c->wait_for_service(5s);
  }

  // How many matching faults are currently in the log (for absence checks + snapshots).
  std::size_t count_faults(const std::string & source_id, uint8_t event_type) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == "GRAPH_PARAM_DRIFT" && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  // The description of the MOST RECENT FAILED report. That is what the fault store keeps: it
  // overwrites the description on every report, so it is the only text an operator can read. A
  // helper that scans the whole history can pass on a single transient report and hide the fact
  // that the current record no longer names the entry.
  std::string latest_failed_desc(const std::string & source_id) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto it = received_.rbegin(); it != received_.rend(); ++it) {
      if (it->source_id == source_id && it->fault_code == "GRAPH_PARAM_DRIFT" &&
          it->event_type == ReportFault::Request::EVENT_FAILED) {
        return it->description;
      }
    }
    return {};
  }

  // The description of the FIRST failed report that mentions `needle`, or empty if there is none.
  //
  // Deliberately NOT the latest one. Any ordering rule keyed on an app being newly affected only
  // decides the single report in which that app first appears: by the next tick the app has been
  // reported, moves into the "already reported" bucket, and every ordering agrees again. Asserting
  // on the latest record therefore passes under a wrong bucket order as soon as one more tick has
  // gone by, which is what let the pinned-versus-self-captured cross combination go unexercised.
  std::string first_failed_desc_containing(const std::string & source_id, const std::string & needle) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == "GRAPH_PARAM_DRIFT" &&
          r.event_type == ReportFault::Request::EVENT_FAILED && r.description.find(needle) != std::string::npos) {
        return r.description;
      }
    }
    return {};
  }

  // True if the CURRENT stored description names EVERY needle - used to prove one aggregated fault
  // names all the drifting apps. Deliberately the latest record and not the history: the store
  // overwrites the description on every report, so a history scan can pass on one transient report
  // while the text an operator actually reads no longer names the entry.
  bool latest_failed_desc_contains(const std::string & source_id, const std::vector<std::string> & needles) {
    const std::string desc = latest_failed_desc(source_id);
    return !desc.empty() && std::all_of(needles.begin(), needles.end(), [&desc](const std::string & needle) {
      return desc.find(needle) != std::string::npos;
    });
  }

  // Tick until a NEW matching fault appears beyond `baseline_count` (i.e. arrived AFTER the
  // trigger), or timeout. Requiring the fault to be new is what makes this a real proof: a
  // detector whose comparison is gutted but still emits a spurious raise+clear during
  // warm-up cannot satisfy it, because those land before the snapshot.
  //
  // The loop bound is 130 iterations (13s at 100ms) so the poll window EXCEEDS the
  // transport's kNegativeCacheTtlSec (10s): the detector owns its transport (built from the
  // registry) and the fixture cannot inject a shorter TTL, so a single timed-out read under
  // ASan/TSan that negative-caches the target for 10s must not outlast the window. The happy
  // path returns on first success, so responsive-node tests stay fast.
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

  // Tick up to `iters` times, returning true as soon as `pred` holds. For non-count
  // assertions (e.g. an aggregated description mentioning multiple apps).
  template <typename Pred>
  bool poll_until(Pred pred, ros2_medkit_graph_watchdog::Detector & det, DetectorContext & ctx, int iters = 130) {
    for (int i = 0; i < iters; ++i) {
      det.tick(ctx);
      std::this_thread::sleep_for(100ms);
      if (pred()) {
        return true;
      }
    }
    return false;
  }

  // EVIDENCE that every armed app has actually been read, to be taken before a test perturbs
  // anything. Ticking a fixed number of times and hoping is not the same claim.
  //
  // The detector withholds its clear while any armed app is still unread (see emit_aggregated), so
  // a PASSED event is not merely "nothing drifts" - it is the detector stating that it looked
  // everywhere and found nothing. wait_param_service() cannot give that: it waits on the GATEWAY
  // node's own client and says nothing about the transport's hidden one. Without this gate a first
  // read that misses its window negative-caches the node for ten seconds, the first successful read
  // lands after the parameter was changed, and the DRIFTED value silently becomes the baseline.
  //
  // Requires a NEW clear rather than any clear: a gate-denied app arms nothing, so a detector with
  // an empty armed set clears on every tick, and "a clear exists" would then be satisfied before
  // the app under test had been looked at even once.
  bool wait_for_baseline_evidence(ros2_medkit_graph_watchdog::Detector & det, DetectorContext & ctx) {
    const auto before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
    return poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, before, det, ctx);
  }

  rclcpp::Node::SharedPtr gateway_, target_, sink_;
  rclcpp::Service<ReportFault>::SharedPtr srv_;
  rclcpp::Client<ReportFault>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread spin_;
  std::mutex mtx_, node_mutex_;
  std::vector<ReportFault::Request> received_;
  IntrospectionInput snapshot_;  // owned entity snapshot the detector reads each tick
  // Clients created by wait_param_service(), cached per service name so none of them are ever
  // destroyed while exec_ is spinning. Released in TearDown(), after the executor has stopped.
  std::map<std::string, rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr> param_clients_;
};

TEST_F(ParamDriftIntegrationTest, RuntimeSetRaisesThenRestoreClears) {
  set_apps({{"pd_it_target", "/pd_it_target"}});
  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // defaults: baseline true, no expect
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_target"));  // guarantee the baseline read succeeds first
  // Evidence, not a fixed sleep: the clear only arrives once the target has actually been read.
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear ever arrived, so the target was never read and the change below would be "
         "measured against no baseline at all";
  // Baseline capture alone MUST be silent (kills an "always-raise" detector).
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);

  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  target_->set_parameter(rclcpp::Parameter("speed_limit", 0.2));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  target_->set_parameter(rclcpp::Parameter("speed_limit", 1.0));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// The fault must be raised under the entity the plugin OWNS, never under the host Component,
// even when the snapshot carries one - which in a real gateway it always does. A host Component
// built by HostInfoProvider never sets `external`, and a non-external component does not claim
// its own bare id in a fault scope set, so a fault raised under it is listed by no entity
// endpoint at all: not /apps/graph_watchdog/faults, not /components/<host>/faults. It would show
// up only in the flat /faults list, which is also all the e2e polls - so nothing else here can
// catch this. Every other test in this file leaves `components` empty, which is exactly why the
// regression it guards went unnoticed.
TEST_F(ParamDriftIntegrationTest, SourceIdIgnoresTheHostComponent) {
  Component host;
  host.id = "pd-it-host";
  host.name = "pd-it-host";
  host.fqn = "pd-it-host";
  snapshot_.components.push_back(host);

  set_apps({{"pd_it_target", "/pd_it_target"}});  // speed_limit is declared in SetUp

  auto det = make_param_drift();
  det->configure(nlohmann::json{{"max_reads_per_tick", 4}});
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_target"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear ever arrived, so the target was never read and no baseline exists to drift from";
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  ASSERT_EQ(failed_before, 0u) << "baseline capture must be silent for the rest of this test to mean anything";
  target_->set_parameter(rclcpp::Parameter("speed_limit", 0.2));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // The discriminator: nothing was ever reported under the component id.
  EXPECT_EQ(count_faults("pd-it-host", ReportFault::Request::EVENT_FAILED), 0u);
  EXPECT_EQ(count_faults("pd-it-host", ReportFault::Request::EVENT_PASSED), 0u);
}

TEST_F(ParamDriftIntegrationTest, ExpectRuleFlagsWrongFromStart) {
  // A separate node that comes up already violating the production pin use_sim_time=false.
  auto bad = std::make_shared<rclcpp::Node>("pd_it_badsim");
  bad->set_parameter(rclcpp::Parameter("use_sim_time", true));
  exec_.add_node(bad);
  const auto bad_guard = on_scope_exit([this, bad] {
    exec_.remove_node(bad);
  });
  set_apps({{"pd_it_badsim", "/pd_it_badsim"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json{{"baseline", false}, {"expect", {{"use_sim_time", false}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_badsim"));  // service up before we assert a read-driven raise
  // Live use_sim_time (true) != expected (false) -> a NEW raise appears with no param set,
  // on first read. A detector that never actually compares would produce no such fault.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));
}

// A node whose parameter services EXIST (discovery succeeds) but which never answers them - the
// #531 unresponsive-node scenario. What has to hold is not a property of tick(): tick() does no IO
// at all in this design, so the time it takes says nothing about a read timeout, and the same
// assertion passes with the timeout raised to an hour. The bounded read is a property of the READER
// THREAD, and the way to see it is to require the sweep to get PAST the node that will not answer.
//
// The unresponsive app sorts FIRST, so the reader's round-robin reaches it before anything else.
// With the read bounded, it times out and the healthy app behind it is served, over and over. With
// the bound removed the reader parks on the very first target for good: the healthy app is never
// read, its `expect` violation is never seen, and neither assertion below can be satisfied.
//
// Teardown is the second half. The detector is destroyed while the fake service is STILL wedged -
// the test does not release it first - because that is the shape of a real shutdown: its destructor
// has to shut the transport down and JOIN the reader with a read outstanding.
TEST_F(ParamDriftIntegrationTest, AnUnresponsiveNodeDoesNotStallTheReader) {
  const std::string kUnresp = "pd_it_aunresp";  // sorts before pd_it_target: first in the rotation
  std::atomic<bool> stop{false};
  auto unresp = std::make_shared<rclcpp::Node>(kUnresp, rclcpp::NodeOptions().start_parameter_services(false));
  // list_parameters is the round-trip both read modes make first; it never replies. It loops on
  // `stop` (not rclcpp::ok()) so the fake node can be released deterministically at the very end.
  auto list_srv = unresp->create_service<rcl_interfaces::srv::ListParameters>(
      "/" + kUnresp + "/list_parameters", service_callback<rcl_interfaces::srv::ListParameters>(
                                              [&stop](const rcl_interfaces::srv::ListParameters::Request & /*req*/,
                                                      rcl_interfaces::srv::ListParameters::Response & /*resp*/) {
                                                while (!stop.load()) {
                                                  std::this_thread::sleep_for(10ms);
                                                }
                                              }));
  // get_parameters only needs to exist in the graph so discovery (wait_param_service) succeeds.
  auto get_srv = unresp->create_service<rcl_interfaces::srv::GetParameters>(
      "/" + kUnresp + "/get_parameters", service_callback<rcl_interfaces::srv::GetParameters>(
                                             [](const rcl_interfaces::srv::GetParameters::Request & /*req*/,
                                                rcl_interfaces::srv::GetParameters::Response & /*resp*/) {}));
  rclcpp::executors::SingleThreadedExecutor unresp_exec;
  unresp_exec.add_node(unresp);
  std::thread unresp_spin([&unresp_exec] {
    unresp_exec.spin();
  });
  // Declared BEFORE the detector so it is destroyed AFTER it: the wedged handler owns the fake
  // node's only executor thread, so nothing can join that thread until `stop` is set, and the
  // detector's own teardown must happen while the node is still refusing to answer.
  const auto release_guard = on_scope_exit([&stop, &unresp_exec, &unresp_spin] {
    stop.store(true);
    unresp_exec.cancel();
    if (unresp_spin.joinable()) {
      unresp_spin.join();
    }
  });

  set_apps({{kUnresp, "/" + kUnresp}, {"pd_it_target", "/pd_it_target"}});
  auto det = make_param_drift();
  // `expect` mode: the healthy target declares speed_limit=1.0 (see SetUp) and the pin says 0.0, so
  // it violates from its very first read and no baseline warm-up is needed - which matters here,
  // because the app that would hold up a warm-up is precisely the one that never answers.
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 100}, {"max_reads_per_tick", 8}, {"expect", {{"speed_limit", 0.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/" + kUnresp));    // services discoverable (but unresponsive)
  ASSERT_TRUE(wait_param_service("/pd_it_target"));  // the healthy app behind it

  // The sweep got past the node that will not answer.
  ASSERT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc_contains("graph_watchdog", {"pd_it_target", "got=1.0"});
      },
      *det, ctx))
      << "the healthy app behind the unresponsive one was never read, so the reader is parked on a "
         "read that never returns: "
      << latest_failed_desc("graph_watchdog");

  // And it KEEPS getting past it. A description that names the new value can only come from a read
  // taken after the change, i.e. from a rotation that came round again.
  target_->set_parameter(rclcpp::Parameter("speed_limit", 7.25));
  EXPECT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc_contains("graph_watchdog", {"got=7.25"});
      },
      *det, ctx))
      << "the healthy app was read once and never again, so the sweep stopped rotating: "
      << latest_failed_desc("graph_watchdog");

  // Nothing was invented about the node that never answered: it is skipped, never raised against.
  EXPECT_EQ(latest_failed_desc("graph_watchdog").find(kUnresp), std::string::npos)
      << "the unresponsive node was reported as drifted on a read that never completed";

  // Teardown with the fake service still wedged. An unbounded read would leave the reader inside
  // rcl when the destructor joins it, and the join would never return.
  const auto teardown_started = std::chrono::steady_clock::now();
  det.reset();
  const auto teardown = std::chrono::steady_clock::now() - teardown_started;
  EXPECT_LT(teardown, 8s) << "destroying the detector while a node was refusing to answer took "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(teardown).count()
                          << " ms: the reader could not be joined out of an unbounded read";
}

// Namespaced-node gate keying, with a REAL wired gate. The discriminator is LIFECYCLE
// suppression, NOT "FQN denied": the gate has two allow-paths - a KNOWN entity is allowed iff
// armed && lifecycle-ok; an UNKNOWN source_id is allowed once past the global bringup grace.
// So an unknown FQN key past grace returns TRUE. The teeth: a known+armed entity whose
// lifecycle is non-active is suppressed (false) via its app.id key, while the SAME string used
// as an unknown FQN key is allowed (true) because it bypasses per-entity lifecycle. The detector
// keys reliability_allows() by app.id; an FQN-keying detector would slip past the suppression.
//
// The negative window is not a hand-picked constant. Silence only means something if it lasted
// longer than a raise demonstrably needs, so the positive control below is TIMED - from the moment
// the gate opens to the moment the raise lands - and the window the negative phase watched for has
// to be at least that long. A 500 ms window next to a 13 s positive poll is not a discriminator, it
// is an accident of how long the negative loop happened to run.
TEST_F(ParamDriftIntegrationTest, NamespacedNodeGateKeyingLifecycleDiscriminator) {
  // node n in namespace /ns: FQN /ns/n != app.id n
  auto nsnode = std::make_shared<rclcpp::Node>("n", "/ns");
  nsnode->declare_parameter<double>("gain", 1.0);
  exec_.add_node(nsnode);
  const auto nsnode_guard = on_scope_exit([this, nsnode] {
    exec_.remove_node(nsnode);
  });
  ASSERT_TRUE(wait_param_service("/ns/n"));
  IntrospectionInput snap;
  {
    App a;
    a.id = "n";
    a.bound_fqn = "/ns/n";
    snap.apps.push_back(a);
  }
  ReliabilityGate gate(3, gateway_.get(), &node_mutex_);
  gate.update(snap, 5);
  gate.update(snap, 8);  // 3 elapsed -> n armed, global grace met
  gate.set_lifecycle_state_for_test("n", "inactive");
  ASSERT_FALSE(reliability_allows(&gate, "n"));     // app.id key: armed but lifecycle-inactive -> SUPPRESSED
  ASSERT_TRUE(reliability_allows(&gate, "/ns/n"));  // FQN key: unknown + past grace -> allowed (the bug path)
  auto det = make_param_drift();
  det->configure(nlohmann::json{{"tick_interval_ms", 100}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  ctx.gate = &gate;
  ctx.snapshot = &snap;
  // NEGATIVE phase: correct app.id keying keeps the drifting node suppressed. Timed from the first
  // tick, because that is when an FQN-keying detector would start capturing.
  const auto negative_started = std::chrono::steady_clock::now();
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  nsnode->set_parameter(rclcpp::Parameter("gain", 0.2));
  for (int i = 0; i < 30; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  const auto negative_window = std::chrono::steady_clock::now() - negative_started;
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);
  //   ^ an FQN-keying detector would see "/ns/n" allowed -> capture + raise; 0 proves app.id keying.
  // POSITIVE control: flip lifecycle active -> the same node's drift now raises (proves the read/aggregate
  // path works, so the 0 above was genuine app.id suppression, not a dead read).
  const auto positive_started = std::chrono::steady_clock::now();
  gate.set_lifecycle_state_for_test("n", "active");
  ASSERT_TRUE(reliability_allows(&gate, "n"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "the node was never read once the gate opened, so the positive control below cannot time "
         "anything";  // captures the current value (0.2) silently
  const auto before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  nsnode->set_parameter(rclcpp::Parameter("gain", 0.9));
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, before, *det, ctx));
  const auto positive_latency = std::chrono::steady_clock::now() - positive_started;

  EXPECT_GE(negative_window, positive_latency)
      << "the suppressed phase was watched for only "
      << std::chrono::duration_cast<std::chrono::milliseconds>(negative_window).count()
      << " ms while an allowed node takes "
      << std::chrono::duration_cast<std::chrono::milliseconds>(positive_latency).count()
      << " ms to arm, be read and raise: the silence above is the window being too short, not the "
         "gate keying being right";
}

// A drift that is already reported must not be healed by the reliability gate closing on its
// app. The gate exists to hold back a RAISE about an entity that has not settled; it is not
// evidence that the parameter went back to its expected value. The aggregate's raise is gated on
// the aggregate's own source and never on the app, so dropping the app's finding when the gate
// denies it is the only thing that could clear it - straight into clear_fault, which carries no
// gate at all. A nav2 lifecycle_manager pausing a controller would then walk a still-present
// drift to HEALED and re-raise it on resume, flapping on every pause.
TEST_F(ParamDriftIntegrationTest, GateDenialDoesNotHealAPresentDrift) {
  auto node = std::make_shared<rclcpp::Node>("g", "/gd");
  node->declare_parameter<double>("gain", 1.0);
  exec_.add_node(node);
  const auto node_guard = on_scope_exit([this, node] {
    exec_.remove_node(node);
  });
  ASSERT_TRUE(wait_param_service("/gd/g"));

  IntrospectionInput snap;
  {
    App a;
    a.id = "gd_frozen";
    a.bound_fqn = "/gd/g";
    snap.apps.push_back(a);
  }
  ReliabilityGate gate(3, gateway_.get(), &node_mutex_);
  gate.update(snap, 5);
  gate.update(snap, 8);  // armed, global grace met
  gate.set_lifecycle_state_for_test("gd_frozen", "active");
  ASSERT_TRUE(reliability_allows(&gate, "gd_frozen"));

  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);
  ctx.gate = &gate;
  ctx.snapshot = &snap;

  for (int i = 0; i < 6; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  node->set_parameter(rclcpp::Parameter("gain", 0.2));
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // The gate closes while the parameter is still wrong. Nothing about the robot got better.
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  gate.set_lifecycle_state_for_test("gd_frozen", "inactive");
  ASSERT_FALSE(reliability_allows(&gate, "gd_frozen"));
  for (int i = 0; i < 8; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "the gate closing healed a drift that is still present";
  EXPECT_NE(latest_failed_desc("graph_watchdog").find("gd_frozen"), std::string::npos)
      << "the suppressed app vanished from the CURRENT description, so nothing tells the operator it "
         "is still drifting: "
      << latest_failed_desc("graph_watchdog");

  // Positive control: the drift can still be cleared the only legitimate way, by fixing it.
  gate.set_lifecycle_state_for_test("gd_frozen", "active");
  const auto passed_before2 = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  node->set_parameter(rclcpp::Parameter("gain", 1.0));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before2, *det, ctx));
}

// The other end of the freeze, and the one nothing covered. Freezing a finding is right for a
// PAUSE - the drift is still real and the gate only says "do not raise about an unsettled entity".
// It is wrong forever. A node that leaves `active` for good is never re-evaluated (not armed) and
// never pruned (still present in the graph), so without a release the aggregate keeps re-raising a
// finding nobody can act on: an operator who fixes the parameter and leaves the node deactivated
// watches GRAPH_PARAM_DRIFT stay CONFIRMED for the life of the gateway, naming a value that is no
// longer true.
//
// kFrozenHoldTicks is 60 consecutive unarmed ticks. Both ends of that are pinned here, because a
// release that fires too early is the flapping GateDenialDoesNotHealAPresentDrift forbids and a
// release that never fires is the latch above. Deleting the release leaves the suite green.
//
// The upper end is pinned by a SMALL number of further ticks rather than by a poll loop. The hold
// is a count the detector keeps privately, so the only thing that can hold the literal below to it
// is squeezing the release between "not yet after N" and "by N+3": a poll loop that ticks a
// hundred more times leaves the literal free to drift away from the constant it mirrors, and
// tripling kFrozenHoldTicks then stays green.
TEST_F(ParamDriftIntegrationTest, AFrozenFindingIsReleasedOnceTheHoldElapses) {
  constexpr int kFrozenHoldTicks = 60;  // mirrors the detector's constant; pinned from both sides below

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}});

  IntrospectionInput snap;
  {
    App a;
    a.id = "pd_it_hold";
    a.bound_fqn = "/pd_it_hold";
    snap.apps.push_back(a);
  }
  ReliabilityGate gate(3, gateway_.get(), &node_mutex_);
  gate.update(snap, 5);
  gate.update(snap, 8);  // armed, global grace met
  gate.set_lifecycle_state_for_test("pd_it_hold", "active");
  ASSERT_TRUE(reliability_allows(&gate, "pd_it_hold"));

  auto ctx = make_ctx(DetectorMode::Raise);
  ctx.gate = &gate;
  ctx.snapshot = &snap;

  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx)) << "the app was never read, so it has no baseline to drift from";
  ASSERT_NE(fake, nullptr);
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(9.0);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the drift never raised, so there is no frozen finding whose release this test can observe";

  // The gate closes and stays closed. No sleeps from here on: the hold is counted in TICKS, and
  // while the app is unarmed the reader has no targets, so there is nothing to wait for.
  gate.set_lifecycle_state_for_test("pd_it_hold", "inactive");
  ASSERT_FALSE(reliability_allows(&gate, "pd_it_hold"));
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kFrozenHoldTicks; ++i) {
    det->tick(ctx);
  }
  // Reports are service round trips, so let the in-flight ones land before asserting that none of
  // them was a clear.
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "the frozen finding was released before the hold had elapsed, so an ordinary lifecycle "
         "pause walks a still-present drift to HEALED";

  // Three more ticks, no more. The release is due on the very next one.
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_GT(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "the finding stayed frozen past kFrozenHoldTicks, so a node that leaves 'active' for good "
         "keeps GRAPH_PARAM_DRIFT raised on a parameter an operator may already have fixed";

  // Positive control: releasing the finding is not the same as going deaf. The app arms again and
  // the drift - which is still present - is re-raised from a fresh read.
  gate.set_lifecycle_state_for_test("pd_it_hold", "active");
  const auto failed_after_release = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_after_release, *det, ctx))
      << "the released finding never came back once the app re-armed, even though the parameter is "
         "still wrong: the release dropped the drift instead of re-deriving it";
}

// Node removal drives prune + clear e2e. Two nodes, one drifts and raises the aggregated
// fault; then the drifted node's App is REMOVED from the snapshot. After > kForgetGrace absent
// sweeps prune_vanished forgets it and drops it from drifted_, so the aggregate must CLEAR.
// Gutting prune_vanished to a no-op leaves the drift latched forever and fails this.
TEST_F(ParamDriftIntegrationTest, NodeRemovalPrunesAndClears) {
  auto keep = std::make_shared<rclcpp::Node>("pd_it_keep");
  keep->declare_parameter<double>("k", 1.0);
  auto gone = std::make_shared<rclcpp::Node>("pd_it_gone");
  gone->declare_parameter<double>("g", 1.0);
  exec_.add_node(keep);
  exec_.add_node(gone);
  const auto nodes_guard = on_scope_exit([this, keep, gone] {
    exec_.remove_node(keep);
    exec_.remove_node(gone);
  });
  set_apps({{"pd_it_keep", "/pd_it_keep"}, {"pd_it_gone", "/pd_it_gone"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_keep"));
  ASSERT_TRUE(wait_param_service("/pd_it_gone"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear arrived, so at least one of the two apps was never read and has no baseline";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);

  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  gone->set_parameter(rclcpp::Parameter("g", 9.0));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // Remove the drifted node from the snapshot: after > kForgetGrace sweeps it is pruned,
  // shedding its drift from the aggregate -> a clear must arrive.
  set_apps({{"pd_it_keep", "/pd_it_keep"}});
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// Forget raised path. Node drifts (aggregate raised) -> vanishes past grace (aggregate
// clears) -> reappears at a NEW healthy value distinct from its original baseline. Because
// forget() dropped its baseline, the first read after reappearance re-captures silently: NO
// stale/re-raised FAILED for it, and the aggregate stays CLEAR. If the baseline were NOT
// forgotten, the new healthy value would drift against the stale (original) baseline.
TEST_F(ParamDriftIntegrationTest, VanishedThenHealthyReappearanceStaysClear) {
  auto keep = std::make_shared<rclcpp::Node>("pd_it_tbkeep");
  keep->declare_parameter<double>("k", 1.0);
  auto reap = std::make_shared<rclcpp::Node>("pd_it_reap");
  reap->declare_parameter<double>("r", 1.0);
  exec_.add_node(keep);
  exec_.add_node(reap);
  const auto nodes_guard = on_scope_exit([this, keep, reap] {
    exec_.remove_node(keep);
    exec_.remove_node(reap);
  });
  set_apps({{"pd_it_tbkeep", "/pd_it_tbkeep"}, {"pd_it_reap", "/pd_it_reap"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_tbkeep"));
  ASSERT_TRUE(wait_param_service("/pd_it_reap"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear arrived, so at least one of the two apps was never read and has no baseline";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);

  // Drift reap -> aggregate raised.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  reap->set_parameter(rclcpp::Parameter("r", 9.0));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // Vanish reap past grace -> pruned, aggregate clears.
  set_apps({{"pd_it_tbkeep", "/pd_it_tbkeep"}});
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));

  // reap reappears at a NEW healthy value (5.0) - distinct from BOTH the original baseline
  // (1.0) and the drifted value (9.0). This is deliberate: if forget() were missing, the
  // stale baseline (1.0) would remain and 1.0 != 5.0 would raise a fresh spurious FAILED; with
  // forget() in place, the stale baseline is gone and 5.0 is silently re-captured as the new
  // baseline. Reusing 1.0 here would make the test pass vacuously even with a missing forget()
  // (stale baseline == reappearance value looks "clean" either way).
  reap->set_parameter(rclcpp::Parameter("r", 5.0));  // healthy, but distinct from the original baseline
  set_apps({{"pd_it_tbkeep", "/pd_it_tbkeep"}, {"pd_it_reap", "/pd_it_reap"}});
  const auto failed_at_reappear = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < 8; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), failed_at_reappear)
      << "stale/re-raised FAILED after healthy reappearance - baseline not forgotten on vanish";
  // The aggregate is currently clear: a fresh PASSED continues to flow.
  const auto passed_reappear = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_reappear, *det, ctx));
}

// A SUCCESS response carrying an empty parameter array is a legitimate transport reply - the
// gateway transport documents it, for instance when a parameter declaration races the list. It is
// NOT evidence that nothing drifts: evaluate() would compare against no observations at all, find
// nothing, and the tick would clear a fault that is still true. No live node can be made to answer
// that way, which is why this needs the stand-in.
TEST_F(ParamDriftIntegrationTest, AnEmptyBatchDoesNotClearARaisedDrift) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  set_apps({{"pd_it_empty", "/pd_it_empty"}});
  for (int i = 0; i < 6; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for the rest of this test to mean anything";

  // Drift it, and confirm the fault is genuinely up before testing what clears it.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(9.0);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // Now every read answers SUCCESS with nothing in it. The parameter is still 9.0.
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  fake->set_empty_batch(true);
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "an empty batch was treated as a complete view and cleared a drift that is still present";

  // Positive control: a real view showing the value restored does clear it.
  fake->set_empty_batch(false);
  fake->set_value(1.0);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// The reader releases the lock for the whole round trip, so a read can still be in flight when the
// tick thread drops its app. Publishing it anyway is not merely stale: prune_vanished has already
// forgotten the baseline and erased the cache, so the late sample lands in an empty cache and
// becomes the baseline the moment the app returns. The node is then compared against a value it
// held only while it was NOT a target - a drift no later read can heal, because the reference
// itself is wrong. This is the failure the still_a_target fence exists to prevent.
//
// Only reachable through the stand-in: a live read returns in milliseconds and nothing in a real
// graph holds one open across the three ticks the prune needs.
TEST_F(ParamDriftIntegrationTest, AReadLandingAfterItsAppWasDroppedIsNotPublished) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  // `keep` stays armed for the whole test. It is what makes the sequencing observable: the reader
  // is a single thread, so a read of `keep` completing AFTER the held read of `fence` was released
  // proves the fenced publish has already happened. With `fence` alone the loop would park on the
  // cv and the test would have to race the very publish it is trying to observe.
  const std::string kFence = "/pd_it_fence";
  const std::string kKeep = "/pd_it_keep";
  set_apps({{"pd_it_fence", kFence}, {"pd_it_keep", kKeep}});
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "both apps must be read and clean before the fence can be tested";
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for the rest of this test to mean anything";

  // Hold a read of `fence` open, and make it answer with a value the node never had while armed.
  // value_for() is read after the gate, so the held call returns 9.0 when it is finally released.
  fake->block_node(kFence);
  ASSERT_TRUE(fake->wait_until_parked()) << "no read of the fenced app was ever held";
  fake->set_value(kFence, 9.0);

  // Drop `fence` from the graph for long enough to prune it: tick 1 takes it out of `targets`,
  // tick 3 (kBaselineForgetGrace + 1) forgets its baseline and erases its cached read.
  set_apps({{"pd_it_keep", kKeep}});
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }

  // Let the held read finish, then wait for a read of `keep` to prove the reader moved past it.
  const int calls_before_release = fake->calls();
  fake->release_node(kFence);
  ASSERT_TRUE(fake->wait_until_reading(calls_before_release + 1))
      << "the reader never got past the released read, so nothing was published either way";

  // The app returns holding the value it always had. A fresh baseline is 1.0 and nothing drifts.
  // Had the late 9.0 been published, it would be the baseline instead, and this read is the drift.
  fake->set_value(kFence, 1.0);
  set_apps({{"pd_it_fence", kFence}, {"pd_it_keep", kKeep}});
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the returned app was never re-read";
  // ~2 s against a 250 ms charge per baseline visit (2 round trips x a 125 ms slot) over two
  // targets: several fresh reads of `fence`.
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "a read that completed after its app was dropped became the baseline on the app's return";
}

// A throw out of a read must not escape the reader thread. reader_loop runs on a std::thread with
// no outer handler, so an escape is std::terminate - the whole gateway process, not just the
// detector. rclcpp can throw from inside a blocking call when the context is invalidated mid-read,
// and the plugin's per-detector guard does not cover this thread.
//
// Deleting the reader's try/catch makes this test abort the test binary rather than fail it, which
// is exactly the point: today nothing in the suite ever makes a read throw.
TEST_F(ParamDriftIntegrationTest, AThrowingReadDoesNotKillTheReaderThread) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  const std::string kThrow = "/pd_it_throw";
  const std::string kKeep = "/pd_it_keep2";
  set_apps({{"pd_it_throw", kThrow}, {"pd_it_keep2", kKeep}});
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "both apps must be read and clean before the throw is introduced";
  ASSERT_NE(fake, nullptr);

  fake->set_throwing(kThrow);
  ASSERT_TRUE(fake->wait_until_threw(2)) << "the reader never issued a read that throws";

  // The surviving reader must still do its job: drift the OTHER app and require the raise. A
  // reader that died silently would leave this polling until it times out.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(kKeep, 9.0);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the reader stopped working after a read threw";
}

// The pre-shutdown callback is what stops the reader while the context is STILL valid. Without it
// the reader can be inside a blocking rcl call when rclcpp invalidates the context, and rcl aborts
// rather than throws - no try/catch reaches that. Nothing in the suite runs the callback, because
// shutting the global context down would take every remaining test with it. A context owned by
// this test drives the same code path in isolation.
TEST_F(ParamDriftIntegrationTest, ContextShutdownStopsTheReaderWhileTheDetectorIsStillAlive) {
  auto own_context = std::make_shared<rclcpp::Context>();
  own_context->init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.context(own_context);
  auto own_node = std::make_shared<rclcpp::Node>("pd_it_shutdown_gw", opts);

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);
  ctx.gateway_node = own_node.get();  // the callback is registered on THIS node's context

  set_apps({{"pd_it_shutdown", "/pd_it_shutdown"}});
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_reading(1)) << "the reader never started";

  // Returns only after the callback has waited for the reader to leave its loop, so the call
  // count is stable from here on - unless nothing stopped the reader at all.
  own_context->shutdown("test");

  const int at_shutdown = fake->calls();
  std::this_thread::sleep_for(1s);  // ~4 baseline visits at the default budget (2 round trips each)
  EXPECT_EQ(fake->calls(), at_shutdown)
      << "the reader was still issuing reads after the context shut down, so a blocking rcl call "
         "could be in flight when the context is invalidated";
}

// The budget is spent in service ROUND TRIPS at the watched node - not in node visits, and not in
// transport calls either. One `get_parameter` is three round trips there (a name-scoped list, a
// get, and an unconditional describe), and an `expect` visit makes one per pinned name, so
// charging a slot per visit lets N pins put 3N times the promised load on one node, and charging a
// slot per call still lets it put three times that load, while the knob an operator lowered to
// protect the node reports being honoured either way.
//
// Four pins and a budget of ten round trips per 1000 ms tick period: one slot is 100 ms, so a visit
// costs 4 x 3 = 12 slots and owes 1200 ms. Visits land at ~0.0 s, ~1.2 s and ~2.4 s, and a fourth
// would need ~3.6 s, so a 3 s window holds exactly three visits: twelve calls. Charging per call
// owes 400 ms and fits eight visits (32 calls); charging per visit owes 100 ms and fits about
// thirty (120 calls).
TEST_F(ParamDriftIntegrationTest, TheBudgetIsSpentInRoundTripsNotCalls) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"baseline", false},
                                {"tick_interval_ms", 1000},
                                {"max_reads_per_tick", 10},
                                {"expect", {{"a", 1.0}, {"b", 1.0}, {"c", 1.0}, {"d", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_budget", "/pd_it_budget"}});

  const auto started = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - started < 3s) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(fake, nullptr);
  const int calls = fake->calls();
  // Both bounds carry weight. Without the upper one, per-call and per-visit charging both pass.
  // Without the lower one, so does a reader that paced itself into silence after a single visit -
  // the test would then be proving that nothing happened, which any amount of over-pacing also
  // satisfies. The gap between them is one visit wide on purpose: a fourth visit inside the window
  // is the smallest observable under-charge, and it is excluded.
  EXPECT_GE(calls, 12) << "issued " << calls
                       << " calls in ~3 s: the third visit never arrived, so the budget is being "
                          "spent slower than the knob promises";
  EXPECT_LE(calls, 15) << "issued " << calls
                       << " calls in ~3 s against a budget of 10 round trips per 1000 ms: a fourth "
                          "visit fit in the window, so a visit was charged less than the three "
                          "round trips each of its four pins really costs the node";
}

// The other half of charging what a read really costs: a pin the node does not declare is answered
// from the name-scoped list alone and never reaches the get or the describe, so it costs the node
// less than a pin that resolves. `expect` is documented as checked on every node that HAS the
// parameter, so on a mixed graph most nodes answer NOT_FOUND for most pins - charging those the
// full three round trips would throttle the sweep THREE TIMES as hard as the load it creates - the
// load being one round trip, not two: the charge of two is deliberately the higher of the two
// NOT_FOUND paths, while what the node really pays on the common one is a single name-scoped list.
// The coverage latency an operator computes from the knob would be wrong in the safe-looking
// direction.
//
// Four pins, none of them declared, a budget of eight round trips per 125 ms tick period: one slot
// is 15.625 ms, so a visit costs 4 x 2 = 8 slots and owes 125 ms. The ninth visit therefore starts
// at ~1.0 s. Charging NOT_FOUND the full three owes 187.5 ms and puts it at ~1.5 s; charging it one
// owes 62.5 ms and puts it at ~0.5 s. The band below excludes both. Timing the reads rather than
// counting them in a fixed window is what buys a quarter-second of margin on each side.
TEST_F(ParamDriftIntegrationTest, AnUndeclaredPinIsChargedLessThanOneThatResolves) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  // Declared-set restriction installed in the factory: the reader starts on the first tick and the
  // very first visit must already be answered NOT_FOUND, or it is charged as a resolving read.
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_declared(std::set<std::string>{});  // the node declares none of the pins
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"baseline", false},
                                {"tick_interval_ms", 125},
                                {"max_reads_per_tick", 8},
                                {"expect", {{"a", 1.0}, {"b", 1.0}, {"c", 1.0}, {"d", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_notfound", "/pd_it_notfound"}});

  // One tick arms the reader; it then round-robins its single target on its own, so the elapsed
  // time to the 36th call is pure pacing and not the tick loop's cadence.
  const auto started = std::chrono::steady_clock::now();
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_reading(36)) << "the reader never issued nine visits, so it is paced "
                                               "far slower than any charge this test can distinguish";
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GT(ms, 750) << "nine visits took " << ms
                     << " ms: an undeclared pin was charged less than the round trip it does cost, "
                        "so the budget under-states the load the sweep puts on the node";
  EXPECT_LT(ms, 1250) << "nine visits took " << ms
                      << " ms: an undeclared pin was charged the full price of one that resolves, "
                         "so the sweep is throttled for a get and a describe it never issued";
}

// A clear says "nothing is drifting anywhere", and the only thing that can make that true is having
// LOOKED everywhere. Any horizon expressed in elapsed ticks eventually expires, and the rotation it
// is racing has no upper bound: with a 1 s tick, one read per tick and 120 apps, the derived give-up
// horizon ran out at tick 3612 while sixty apps had still never been contacted, so tick 3613
// cleared GRAPH_PARAM_DRIFT on evidence that did not exist. Bigger graph, longer silence, MORE
// confidence - exactly backwards, and worst on the graphs where the sweep is slowest.
//
// 160 apps at one round trip per 200 ms tick period: a baseline visit owes two slots, i.e. 400 ms,
// so one rotation takes a minute. The tick loop below carries no sleep and runs far past the old
// horizon inside a fraction of that, so nearly the whole graph is uncontacted throughout. Nothing
// may be cleared, and nothing may be SAID about an app the sweep has not reached either - a warning
// that "no read has succeeded" for a node nobody has tried to read is the same defect in the log.
TEST_F(ParamDriftIntegrationTest, NoClearWhileTheSweepHasNotReachedEveryApp) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 200}, {"max_reads_per_tick", 1}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  for (int i = 0; i < 160; ++i) {
    const std::string id = "pd_it_wide" + std::to_string(1000 + i);  // uniform width -> sorted == creation order
    apps.emplace_back(id, "/" + id);
  }
  set_apps(apps);
  det->tick(ctx);  // builds the transport and hands the reader all 160 targets
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_reading(1)) << "the reader never started, so nothing here is under test";

  // No sleep: this is about tick COUNT, and the point is that no number of ticks can substitute for
  // a read. 4000 is past the horizon the old derivation capped out at (3612) for any graph at all.
  for (int i = 0; i < 4000; ++i) {
    det->tick(ctx);
  }
  // The reader takes targets round-robin in order, and a baseline visit is exactly one call, so a
  // call count below the app count means the tail of the graph was never contacted. This counts
  // COVERAGE, not load - the stand-in cannot measure round trips and no bound here depends on it.
  ASSERT_LT(fake->calls(), 160) << "the sweep got round all 160 apps inside the window, so "
                                   "'uncontacted' was never the state under test";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), 0u)
      << "cleared GRAPH_PARAM_DRIFT while most of the graph had never been contacted - that is "
         "health the detector never measured, and it is what any horizon in ticks does once the "
         "rotation outlasts it";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "nothing drifted, so nothing may be raised either";
  EXPECT_EQ(log.count("consecutive parameter reads of"), 0)
      << "told the operator that reads of an app had failed when no read of it was ever attempted";
  EXPECT_EQ(log.count("giving up on"), 0)
      << "gave up on an app the sweep had not reached, on the strength of elapsed ticks alone";

  // Positive control: reduce the graph to an app that HAS been read and the clear flows at once.
  // The zeroes above are the guard doing its job, not a detector that emits nothing.
  set_apps({apps.front()});
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "no clear even once every remaining app had been read, so the guard never releases";
}

// The other half of the same rule. Silence about an app is only ever reported once we have tried
// and failed - repeatedly - to read it, and the count that decides both warnings is a count of
// ATTEMPTS, so it cannot be reached by waiting. Both warnings fire once per run of failures, in
// order, and only about the app that will not answer.
//
// Two apps at eight round trips per 10 ms tick period: one slot is 1.25 ms and both a successful
// baseline visit and a failed one are charged two slots, so the ghost is retried roughly every
// 5 ms and the sixty-first failure lands inside a second.
TEST_F(ParamDriftIntegrationTest, TheUnreadWarningsRequireFailedAttemptsNotElapsedTicks) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_mute");  // before the reader can ever answer for it
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_heard", "/pd_it_heard"}, {"pd_it_mute", "/pd_it_mute"}});

  for (int i = 0; i < 400 && log.count("giving up on") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(fake, nullptr);
  EXPECT_EQ(log.count("giving up on"), 1) << "the app that never answers was never given up on, so it "
                                             "holds back every clear from here on";
  EXPECT_EQ(log.count("consecutive parameter reads of 'pd_it_mute'"), 1)
      << "the app that never answers was never named, or was named more than once - the warning is "
         "not latched";
  EXPECT_EQ(log.count("consecutive parameter reads of"), 1)
      << "the healthy app was named alongside it, even though every read of it succeeded";

  // Both are one-shot. Keep ticking well past the point they fired.
  for (int i = 0; i < 100; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(log.count("consecutive parameter reads of"), 1)
      << "the unread warning repeats every tick once its count is passed";
  EXPECT_EQ(log.count("giving up on"), 1) << "the give-up warning repeats every tick once its count is passed";
}

// A baseline visit is one list_parameters(), and that is TWO round trips at the node: the list,
// then the batched get of every name it returned. Charging it one slot - the old per-call rate -
// puts twice the promised load on every watched node in this detector's default mode.
//
// The other half of the rule is what is deliberately NOT charged. The first list_parameters()
// against a node costs two more, because the transport primes its defaults cache before its own
// read, and that cache has no TTL: the extra pair is a one-off per node for the life of the
// gateway. Charging four on every read would throttle the whole run to pay for the first rotation.
//
// One app, a budget of eight round trips per 125 ms tick period: one slot is 15.625 ms, so a visit
// costs 2 slots and owes 31.25 ms, putting the 33rd read at ~1.0 s. One slot per visit puts it at
// ~0.5 s and the cold-start four at ~2.0 s; the band excludes both.
TEST_F(ParamDriftIntegrationTest, ABaselineVisitIsChargedTwoRoundTripsAndNotTheColdStartPair) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 125}, {"max_reads_per_tick", 8}});  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_bl", "/pd_it_bl"}});

  // One tick arms the reader; it round-robins its single target on its own from there, so the
  // elapsed time is pure pacing rather than the tick loop's cadence.
  const auto started = std::chrono::steady_clock::now();
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_reading(33)) << "the reader never issued 33 baseline visits, so it is "
                                               "paced far slower than any charge this test can distinguish";
  const auto elapsed = std::chrono::steady_clock::now() - started;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GT(ms, 750) << "33 baseline visits took " << ms
                     << " ms: a visit was charged less than the list-plus-get it really costs the "
                        "node, so the budget under-states the load by half";
  EXPECT_LT(ms, 1250) << "33 baseline visits took " << ms
                      << " ms: a visit was charged the cold-contact price of four, which is paid "
                         "once per node for the life of the gateway, not on every read";
}

// The same condition that governs a clear governs the unmatched-`expect`-pin warning, and for the
// same reason: only one node is visited per rotation slot, so a pin that exactly one node declares
// is legitimately unseen until the sweep reaches that node. On any threshold counted in elapsed
// ticks the detector announces that "no app declares" it long before it has looked at the app that
// does - a warning that tells an operator to go and fix a name that is already correct. So the
// warning is withheld until every armed app has been read (or given up on) and at least one of
// them answered.
//
// Twenty-one apps, one pin, one round trip per 10 ms tick period. An undeclared pin is charged two
// round trips, so the twenty nodes that answer NOT_FOUND owe two slots each and the one node that
// declares the pin - it sorts last - is reached after forty slots, i.e. ~400 ms. The 140 ticks
// below span well over a second, so the sweep completes several times over with nothing reported,
// which is only true if the pin was actually MATCHED rather than merely not yet due.
TEST_F(ParamDriftIntegrationTest, APinOnlyTheLastNodeDeclaresIsNotReportedAsUnmatched) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    // Only the last-sorting node has the pin; the rest answer NOT_FOUND for it from the first read.
    t->set_declared(std::set<std::string>{});
    t->set_declared("/pd_it_pinlast", {"a_pin"});
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 10}, {"max_reads_per_tick", 1}, {"expect", {{"a_pin", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  for (int i = 0; i < 20; ++i) {
    const std::string id = "pd_it_pin" + std::to_string(100 + i);  // uniform width; all sort before "pinlast"
    apps.emplace_back(id, "/" + id);
  }
  apps.emplace_back("pd_it_pinlast", "/pd_it_pinlast");
  set_apps(apps);

  for (int i = 0; i < 140; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_NE(fake, nullptr);
  EXPECT_EQ(log.count("no app declares 'a_pin'"), 0)
      << "told the operator no app declares a pin that the last node in the rotation declares "
         "perfectly well - the warning fired before the sweep had read that node";
}

// A throw while building the shared IO must leave io_ null so the next tick retries. Assigning it
// first and filling it in afterwards latches a half-built state: the plugin's per-detector guard
// logs the throw once, every later tick sees a non-null io_ and skips the setup, and the detector
// then runs forever with no reader at all - indistinguishable from a healthy one at every call site.
TEST_F(ParamDriftIntegrationTest, AFailedLazyInitRetriesOnTheNextTick) {
  ScriptedTransport * fake = nullptr;
  int attempts = 0;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test(
      [&fake, &attempts](rclcpp::Node *) -> std::unique_ptr<ros2_medkit_gateway::ParameterTransport> {
        if (++attempts == 1) {
          throw std::runtime_error("transport unavailable");
        }
        auto t = std::make_unique<ScriptedTransport>();
        fake = t.get();
        return t;
      }));
  det->configure(nlohmann::json::object());
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_retry", "/pd_it_retry"}});

  EXPECT_THROW(det->tick(ctx), std::runtime_error);  // the plugin guard swallows this in production
  for (int i = 0; i < 8; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(attempts, 2) << "the detector did not retry the transport after the first failure";
  ASSERT_NE(fake, nullptr) << "no transport was ever built, so the detector has no reader";
  EXPECT_GT(fake->calls(), 0) << "the reader never ran after the retry";
}

// The gateway must never read its own parameters: the transport spins a hidden node of its own, so
// reading the gateway would have it interrogate itself and can raise drift against the plugin. The
// skip is one line and nothing covered it.
TEST_F(ParamDriftIntegrationTest, TheGatewaysOwnNodeIsNeverRead) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json::object());
  auto ctx = make_ctx(DetectorMode::Raise);

  // Only the gateway itself: no read may happen.
  set_apps({{"pd_it_gateway", gateway_->get_fully_qualified_name()}});
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(fake, nullptr);
  EXPECT_EQ(fake->calls(), 0) << "the detector read the gateway's own parameters";

  // Positive control: zero above must mean "skipped the gateway", not "read nothing at all". Any
  // mutation that empties the armed set would also give zero, so an ordinary app has to produce
  // reads through the very same path.
  set_apps({{"pd_it_other", "/pd_it_other"}});
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(fake->calls(), 0) << "no app was ever read, so the zero above proves nothing";
}

// A config warning that never reaches the log is no warning at all. Six unit tests verify that the
// warnings vector gets populated and none of them proves it is ever surfaced, so deleting the
// logging call changed nothing. Here the detector is handed a misspelt key and the tick must reach
// the logger - asserted through the rcutils log handler rather than by reading stderr.
TEST_F(ParamDriftIntegrationTest, ConfigWarningsReachTheLogger) {
  const LogCapture log;

  auto det = make_param_drift();
  ScriptedTransport * fake = nullptr;
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  const nlohmann::json typo{{"ignore_globs", nlohmann::json::array({"*_stamp"})}};  // misspelt on purpose
  det->configure(typo);
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_warn", "/pd_it_warn"}});
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_NE(fake, nullptr);

  EXPECT_EQ(log.count("ignore_globs"), 1)
      << "expected the typo logged exactly once across ten ticks; got " << log.count("ignore_globs")
      << " (0 means it never reached the log, more than 1 means the once-guard is gone and every "
         "tick repeats it forever)";

  // The once-guard has to be RE-ARMED by configure(). The plugin re-delivers configuration to a
  // live detector, and without the reset an operator who fixes one key and mis-spells another is
  // told nothing at all about the second one - the warning is latched for the rest of the run.
  det->configure(typo);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(log.count("ignore_globs"), 2)
      << "the same typo delivered by a second configure() was never logged again, so the "
         "once-per-run latch is never released and re-configuration is silent";
}

// A pin naming a parameter no node declares is answered NOT_FOUND on every read, and NOT_FOUND is
// deliberately not a drift - a node is not at fault for a parameter it never had. The cost is that
// a misspelt pin is inert forever, and from outside inert is indistinguishable from a pin that is
// checked and passes every time. That is the silent failure this detector exists to rule out, so
// it has to say something. Exactly once, and only about the pin that is actually unmatched.
TEST_F(ParamDriftIntegrationTest, AnExpectPinNoAppDeclaresIsReportedOnce) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  // Two pins: one the node has, one misspelt. The good pin is the control - it must stay silent,
  // or the warning is just firing on every pin and proves nothing about the typo.
  det->configure(nlohmann::json{{"baseline", false}, {"expect", {{"gain", 1.0}, {"gian", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_typo", "/pd_it_typo"}});
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  fake->set_declared({"gain"});

  for (int i = 0; i < 80 && log.count("no app declares 'gian'") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(log.count("no app declares 'gian'"), 1) << "the misspelt pin was never reported";

  // Keep ticking: the report is once per pin, not once per tick.
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(log.count("no app declares 'gian'"), 1) << "the once-guard is gone and every tick repeats the warning";
  EXPECT_EQ(log.count("no app declares 'gain'"), 0) << "the pin the node actually declares was reported as unmatched";
}

// The other side of the same rule, and the one that costs a real alarm if it is wrong. DDS
// discovery drops a node from a single poll routinely, with no restart involved - WarmupTracker
// documents exactly that and absorbs such a gap. If a one-sweep gap re-baselined, the node would
// come back and its CURRENT, still drifted value would become the new reference: the confirmed
// fault heals while the parameter is wrong, and can never fire again because the baseline now IS
// the wrong value. Nothing in the log would say so.
TEST_F(ParamDriftIntegrationTest, ChurnMustNotLaunderARaisedDrift) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  const std::vector<std::pair<std::string, std::string>> present{{"pd_it_churn", "/pd_it_churn"}};
  set_apps(present);
  for (int i = 0; i < 6; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(fake, nullptr);

  // A genuine drift, confirmed up before we disturb anything.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(9.0);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // One dropped poll. The node never restarted and its parameter is still 9.0.
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  set_apps({});
  det->tick(ctx);
  std::this_thread::sleep_for(50ms);
  set_apps(present);
  for (int i = 0; i < 14; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "a single dropped discovery poll re-baselined the node on its drifted value, so the fault "
         "healed while the parameter is still wrong and can never fire again";

  // Positive control: fixing it for real still clears, so the zero above is not a dead detector.
  fake->set_value(1.0);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// The withheld clear must not become a permanent latch. Its purpose is the case where nothing is
// known yet: the round-robin sweep has not reached this app. A node that never answers AT ALL - no
// parameter service, a wedged executor - is a permanent property, and there is no per-node opt-out
// here, so without a bound one such node keeps a fixed drift reported as CONFIRMED for the whole
// lifetime of the gateway. Past the bound the app is declared unmeasurable, logged, and stops
// blocking.
//
// The bound is in FAILED ATTEMPTS, not in elapsed ticks: this app has been read sixty-one times and
// answered none of them, which is evidence, where "sixty ticks went by" is not - the sweep may
// simply not have got here. That is the whole distinction, and it is why the escape hatch survives
// the rule that an app the sweep has never attempted blocks the clear indefinitely. The budget is
// set so the sixty-first attempt lands inside a second rather than at the default pacing.
TEST_F(ParamDriftIntegrationTest, APermanentlyUnreadableAppStopsBlockingTheClear) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  // The ghost is marked unreadable inside the factory, before the transport is ever handed to the
  // reader. Marking it after the first tick is a race the reader can win, and if it does the ghost
  // has a cached read for the rest of the run, never joins the unread set, and never blocks
  // anything - the test would pass while proving nothing.
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_ghost2");
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 20}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  // The real app is served by the stand-in; the ghost has no node at all, so no read for it can
  // ever succeed. Both are armed every tick.
  set_apps({{"pd_it_real", "/pd_it_real"}, {"pd_it_ghost2", "/pd_it_ghost2"}});
  det->tick(ctx);  // builds the transport and hands the reader both targets
  ASSERT_NE(fake, nullptr);
  // Wait for EVIDENCE that the real app's baseline exists rather than sleeping a fixed time. Only
  // /pd_it_real reaches the stand-in's call counter (the ghost fails before it), so a second
  // counted read means the first one has already been published to the cache, and the tick after
  // it turns that into the baseline. A fixed window here is a race against the sweep's own pacing:
  // the ghost's failed visit is charged too, so how long the real app waits for its turn is a
  // function of the budget, and losing that race makes the drifted value the baseline instead -
  // after which no drift exists at all and the rest of the test is unreachable.
  ASSERT_TRUE(fake->wait_until_reading(2)) << "the reader never completed a read of /pd_it_real";
  det->tick(ctx);

  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(9.0);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the drift never raised, so there is nothing whose clearing this test can observe";

  // Fix it. The ghost is still silent and always will be.
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  fake->set_value(1.0);
  bool cleared = false;
  for (int i = 0; i < 120 && !cleared; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
    cleared = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED) > passed_before;
  }
  EXPECT_TRUE(cleared) << "one permanently unreadable app blocked the clear forever, so a drift that "
                          "was fixed stays CONFIRMED for the lifetime of the gateway";
}

// Restarting a node is the documented way to accept a new configuration, so a node that was gone
// long enough to have restarted must come back to a fresh baseline rather than being compared
// against values from before the change.
//
// "Long enough" is deliberately the window WarmupTracker absorbs, not a single sweep: that header
// documents that DDS discovery drops a node from one poll routinely, with no restart involved, and
// re-capturing on such a gap would take the node's current - possibly already drifted - values as
// the new reference, healing a real fault that can then never fire again.
//
// Here the app is absent for three sweeps and returns with a different value. Nothing may be
// reported: that value is the new reference.
TEST_F(ParamDriftIntegrationTest, ARestartLengthAbsenceReBaselinesInsteadOfReportingDrift) {
  set_apps({{"pd_it_target", "/pd_it_target"}});
  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_target"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear arrived, so the app was never read and there is no pre-restart baseline whose "
         "being forgotten this test could observe";
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for this test to mean anything";

  // Absent for longer than the churn window, i.e. long enough to be a restart.
  set_apps({});
  for (int i = 0; i < 4; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }

  // It comes back carrying the operator's new value.
  target_->set_parameter(rclcpp::Parameter("speed_limit", 5.0));
  set_apps({{"pd_it_target", "/pd_it_target"}});
  const auto failed_before_return = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "the returning app was never re-read, so the silence below is the silence of a detector "
         "that measured nothing";
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), failed_before_return)
      << "reported drift against the baseline from before the restart, so the README's recovery "
         "path (restart the node to re-baseline) does not work";

  // POSITIVE CONTROL. Every assertion above this line is an all-zeros one, and a detector that
  // reads nothing satisfies them exactly as well as one whose forget() works. Moving off the value
  // the node came back with must report - which is only possible if that value really was captured
  // as the new reference.
  const auto failed_before_drift = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  target_->set_parameter(rclcpp::Parameter("speed_limit", 0.25));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before_drift, *det, ctx))
      << "no drift off the re-captured baseline, so the silence above proves nothing at all";
}

// A clear says "nothing is drifting anywhere". The detector may only say that about apps it
// actually read. Here one app is real and healthy while a second app is in the snapshot with an
// FQN that answers nothing, so no read for it can ever succeed. Emitting PASSED in that state
// would report health that was never measured - and because the README tells operators to enable
// healing, those PASSED events are what would walk a genuine, still-present drift to HEALED.
//
// The healthy app is what makes this a real discriminator: without the guard, its clean reads
// leave drifted_ empty and the detector cheerfully clears every single tick.
TEST_F(ParamDriftIntegrationTest, NoClearWhileAnArmedAppWasNeverRead) {
  set_apps({{"pd_it_target", "/pd_it_target"}, {"pd_it_ghost", "/pd_it_ghost"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_target"));  // the real one is readable
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }

  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), 0u)
      << "cleared GRAPH_PARAM_DRIFT while /pd_it_ghost had never been read - that is health "
         "the detector never measured";
  // And nothing was invented either: no drift exists, so no raise should have happened.
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);
}

// An app that has been given up on must STAY given up on. The blocking set is recomputed from
// scratch every tick while the give-up is one-shot, so any bound that moves with the shape of the
// graph silently re-admits an app already logged as "no longer holding back the clear": the graph
// grows, the bound grows past the app's count, and every clear from then on is suppressed with
// nothing in the log to say why. A count of that app's OWN failed attempts cannot do that - it
// depends on nothing but the app, and it only ever grows while the app stays unread.
//
// The armed set is grown and then shrunk around a given-up ghost, and a clear is required to arrive
// on both sides. Forty extra apps at eight round trips per 10 ms tick period is a 105 ms rotation,
// so the grown graph is fully read inside a fraction of the polls below - while a bound scaled by
// the armed set would need 60 x 42 failures, which at one ghost retry per rotation is minutes away.
TEST_F(ParamDriftIntegrationTest, AGivenUpAppDoesNotReBlockWhenTheArmedSetGrowsOrShrinks) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_grow_ghost");  // before the reader can ever answer for it
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  const std::vector<std::pair<std::string, std::string>> small{{"pd_it_grow_real", "/pd_it_grow_real"},
                                                               {"pd_it_grow_ghost", "/pd_it_grow_ghost"}};
  set_apps(small);
  // The first clear IS the give-up: nothing drifts, so the only thing that can be holding the clear
  // back is the ghost, and the only thing that can release it is having failed on it often enough.
  bool gave_up = false;
  for (int i = 0; i < 400 && !gave_up; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
    gave_up = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED) > 0;
  }
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(gave_up) << "the ghost never stopped blocking, so there is no give-up to re-block";

  // GROW. Forty apps arrive at once; each of them legitimately blocks the clear until it is read,
  // and the ghost must not join them again on the way past.
  std::vector<std::pair<std::string, std::string>> grown = small;
  for (int i = 0; i < 40; ++i) {
    const std::string id = "pd_it_grow" + std::to_string(100 + i);
    grown.emplace_back(id, "/" + id);
  }
  set_apps(grown);
  auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  bool cleared_grown = false;
  for (int i = 0; i < 150 && !cleared_grown; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
    cleared_grown = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED) > passed_before;
  }
  EXPECT_TRUE(cleared_grown) << "no clear once the grown graph had been read: an app already given up "
                                "on was re-admitted to the blocking set because the graph got bigger, "
                                "and nothing in the log says so";

  // SHRINK back. Every remaining app is either read or given up on, so the very next ticks clear.
  set_apps(small);
  passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  bool cleared_small = false;
  for (int i = 0; i < 40 && !cleared_small; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
    cleared_small = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED) > passed_before;
  }
  EXPECT_TRUE(cleared_small) << "the ghost started blocking again when the graph shrank back";
}

// The stale-read fence has to be keyed on the ARMING EPISODE, not on the app. A read runs with the
// lock released and can take seconds; inside that window the app's lifecycle gate can close and
// re-open, and tick() rewrites the target list every tick, so an identity check on the app id -
// or on the fqn, which is just as stable - lets a value sampled while the node was between
// episodes land in the cache. That value then becomes what the node is measured against: a drift
// that is not real and that no later read can heal, which is the exact failure the fence exists
// for. A token issued once per episode and never reused is what tells the two apart.
//
// Only reachable through the stand-in: a live read returns in milliseconds and nothing in a real
// graph holds one open across a de-arm and a re-arm.
TEST_F(ParamDriftIntegrationTest, AReadFromAnInterruptedArmingEpisodeIsNotPublished) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  // A 500 ms tick period against eight round trips is a 62.5 ms slot, so a baseline visit owes
  // 125 ms: after the held read is released there is a comfortable margin before the reader could
  // come back round to the fenced app.
  det->configure(nlohmann::json{{"tick_interval_ms", 500}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  const std::string kEpoch = "/pd_it_epoch";
  const std::string kKeep = "/pd_it_epochkeep";
  set_apps({{"pd_it_epoch", kEpoch}, {"pd_it_epochkeep", kKeep}});
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "both apps must be read and clean before the fence can be tested";
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for the rest of this test to mean anything";

  // Hold a read of the fenced app open and arrange for it to answer 9.0 - value_for() is sampled
  // after the gate, so that is what the held call returns when it is finally released. Marking the
  // app unreadable at the same time only affects LATER reads (the held one is already past that
  // check), so nothing can overwrite whatever this one publishes, if it publishes.
  fake->block_node(kEpoch);
  ASSERT_TRUE(fake->wait_until_parked()) << "no read of the fenced app was ever held";
  fake->set_value(kEpoch, 9.0);
  fake->set_unreadable(kEpoch);

  // De-arm and re-arm inside that one read window. One tick out and one tick back in: far short of
  // the prune, so the baseline (1.0) is untouched, and the app id and the fqn are both unchanged -
  // an identity check on either sees nothing at all happen here.
  set_apps({{"pd_it_epochkeep", kKeep}});
  det->tick(ctx);
  set_apps({{"pd_it_epoch", kEpoch}, {"pd_it_epochkeep", kKeep}});
  det->tick(ctx);

  const int calls_before_release = fake->calls();
  fake->release_node(kEpoch);
  // A read of the OTHER app is the next counted one - an unreadable node fails before the counter -
  // so this means the released read has finished and either published or been fenced out.
  ASSERT_TRUE(fake->wait_until_reading(calls_before_release + 1))
      << "the reader never got past the released read, so nothing was published either way";

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "a value sampled while the app was between arming episodes was published as its reading, "
         "so the node is now measured against something it held only while it was not a target";

  // Positive control: the raise path is alive, so the zero above is the fence and not a dead
  // detector. The other app has been armed continuously throughout and drifting it must report.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value(kKeep, 9.0);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "no drift was reported for an app that plainly drifted, so nothing above proves anything";
}

// Forgetting a vanished node's baseline must be reachable at EVERY accepted prune_grace. The two
// horizons are independent - the baseline goes after a fixed three missed sweeps, the finding after
// prune_grace of them - and dropping the absence counter is what makes any later cleanup
// unreachable. With prune_grace 0 or 1, both inside the documented range, the counter was dropped
// before it could ever reach the forget, so the forget never ran for any app in any run: the
// baselines map grew without bound under node-name churn and every returning node was measured
// against values from before it left.
//
// Each case runs the same story - baseline, vanish, return holding a DIFFERENT value - and the
// return must be silent, because a forgotten baseline makes that value the new reference. The
// endpoints of the range (0 and 3600), the value below the churn window (1), the detector's own
// fallback (2), what the plugin injects when an operator sets nothing (60), and two values outside
// the range, which must be rejected, must warn, and must leave the fallback behaviour intact.
TEST_F(ParamDriftIntegrationTest, BaselineForgettingIsReachableAtEveryAcceptedPruneGrace) {
  const LogCapture log;

  const std::vector<std::pair<std::string, nlohmann::json>> cases{{"0", 0},       {"1", 1},       {"2", 2},  {"60", 60},
                                                                  {"3600", 3600}, {"4000", 4000}, {"-1", -1}};
  const int out_of_range = 2;  // the last two cases; both must warn and both must keep the fallback

  int seq = 0;
  for (const auto & [label, value] : cases) {
    SCOPED_TRACE("prune_grace=" + label);
    ++seq;
    const std::string id = "pd_it_pg" + std::to_string(seq);
    const std::string fqn = "/" + id;

    ScriptedTransport * fake = nullptr;
    auto det = make_param_drift();
    ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
      auto t = std::make_unique<ScriptedTransport>();
      fake = t.get();
      return t;
    }));
    det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}, {"prune_grace", value}});
    auto ctx = make_ctx(DetectorMode::Raise);

    // Baseline captured at the stand-in's default of 1.0. The clear proves the app was read.
    set_apps({{id, fqn}});
    const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
    ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
        << "the app was never read, so it has no baseline to forget";
    ASSERT_NE(fake, nullptr);

    // Absent for longer than either horizon can need: the churn window absorbs two missed sweeps
    // and the forget runs on the third, while the shortest configurable grace drops the counter on
    // the first.
    set_apps({});
    for (int i = 0; i < 6; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(20ms);
    }

    // It returns holding a value it never held before. With the baseline forgotten that value IS
    // the new reference; with it retained, 1.0 against 5.0 is a drift the node can never shake off.
    fake->set_value(fqn, 5.0);
    set_apps({{id, fqn}});
    const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
    for (int i = 0; i < 40; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(20ms);
    }
    EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), failed_before)
        << "the returning node was reported as drifted against a baseline from before it left, so "
           "the forget never ran at this prune_grace";

    // Positive control: the baseline it came back with is a real one, so moving off it still
    // reports. Without this the case above is satisfied by a detector that captured nothing.
    fake->set_value(fqn, 9.0);
    EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
        << "no drift off the re-captured baseline, so the silence above proves nothing";
  }

  EXPECT_EQ(log.count("'prune_grace' must be an integer"), out_of_range)
      << "an out-of-range prune_grace was accepted or rejected in silence; an operator has no other "
         "way to find out the value they wrote is not the one in force";
}

// Every per-app map in this detector is keyed by node name, and node names are not a bounded set:
// discovery has no hidden-node filter, so each `ros2` CLI invocation appears once under a name
// nothing will ever use again. A map no prune path reaches therefore grows for the life of the
// gateway process, and a leak shows up in no fault, no log line and no description - which is why
// this asserts on a count rather than on a behaviour.
//
// The churn here is the real shape: a short-lived node that is present for a single sweep, is gate-
// denied for all of it (it has not been present long enough to arm), and then is gone. That path
// creates the frozen-hold counter and nothing on it ever created anything else, so the frozen-hold
// counter was the one map prune_vanished did not clean.
TEST_F(ParamDriftIntegrationTest, PerAppBookkeepingStaysBoundedUnderNodeNameChurn) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  }));
  det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}, {"prune_grace", 2}});
  auto ctx = make_ctx(DetectorMode::Raise);

  IntrospectionInput snap;
  ReliabilityGate gate(3, gateway_.get(), &node_mutex_);
  ctx.gate = &gate;
  ctx.snapshot = &snap;

  const auto seed = [&snap](const std::vector<std::string> & ids) {
    snap.apps.clear();
    for (const auto & id : ids) {
      App a;
      a.id = id;
      a.bound_fqn = "/" + id;
      snap.apps.push_back(a);
    }
  };

  uint64_t tick_no = 0;
  const std::string stable = "pd_it_churn_stable";
  for (int i = 0; i < 200; ++i) {
    // One long-lived app plus one that exists for exactly this sweep, under a name never reused.
    seed({stable, "ros2_cli_" + std::to_string(i)});
    gate.update(snap, ++tick_no);
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_NE(fake, nullptr);

  // Let the last few transients age past the prune horizon.
  seed({stable});
  for (int i = 0; i < 10; ++i) {
    gate.update(snap, ++tick_no);
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
  }

  const std::size_t tracked = det->tracked_count_for_test();
  EXPECT_GE(tracked, 1u) << "the detector is tracking nothing at all, so the bound below is vacuous";
  EXPECT_LE(tracked, 4u) << "still holding per-app bookkeeping for " << tracked
                         << " identities after 200 one-sweep nodes came and went: a map keyed by node "
                            "name is not being pruned, and it will grow for the life of the process";
}

// A single app's share of the aggregated description is capped, and that cap is what makes going
// second worth anything at all. Without it, one node drifting on a dozen parameters spends the
// whole 480-character budget by itself and every other affected app is cut - whatever the ordering
// says, because there is no room left to order anything into.
//
// Three apps, each drifting on twelve long-valued parameters. Uncapped, the first alone renders to
// roughly 1900 characters and the other two never appear. Capped at 150 each, all three fit inside
// the budget with room to spare, so naming all three is exactly the per-app cap doing its job.
//
// The stand-in is what makes the sizing exact: a live node's parameter set is not fully under the
// test's control, and this assertion is about character counts.
TEST_F(ParamDriftIntegrationTest, OneAppsShareOfTheDescriptionIsCapped) {
  const std::string clean_value(60, 'a');
  const std::string drifted_value(60, 'b');
  const std::vector<std::string> ids{"pd_it_cap1", "pd_it_cap2", "pd_it_cap3"};

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake, &ids, &clean_value](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    // Installed in the FACTORY: a drifted first read would simply become the baseline.
    for (const auto & id : ids) {
      std::map<std::string, nlohmann::json> clean;
      for (int p = 0; p < 12; ++p) {
        clean.emplace("p" + std::to_string(p), clean_value);
      }
      t->set_params("/" + id, clean);
    }
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 100}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  apps.reserve(ids.size());
  for (const auto & id : ids) {
    apps.emplace_back(id, "/" + id);
  }
  set_apps(apps);
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx)) << "not every app was read, so nothing has a baseline yet";
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for the cap assertion below to mean anything";

  for (const auto & id : ids) {
    std::map<std::string, nlohmann::json> drifted;
    for (int p = 0; p < 12; ++p) {
      drifted.emplace("p" + std::to_string(p), drifted_value);
    }
    fake->set_params("/" + id, drifted);
  }

  ASSERT_TRUE(poll_until(
      [this, &ids]() {
        return latest_failed_desc_contains("graph_watchdog", ids);
      },
      *det, ctx))
      << "the description names " << latest_failed_desc("graph_watchdog").size()
      << " characters' worth of one or two apps and not all three: a single app is spending more "
         "than its share of the budget and the others are being cut - "
      << latest_failed_desc("graph_watchdog");

  // Belt and braces on the sizing: the whole point is that three capped apps FIT, so the aggregate
  // must be under its own cap here. If it were at the cap the three names above could be a
  // coincidence of truncation rather than the per-app budget working.
  EXPECT_LE(latest_failed_desc("graph_watchdog").size(), AggregatedFault::kMaxDescriptionChars);
}

// The description is capped, so ORDER decides what an operator actually reads. Four apps that sort
// early and drift on many parameters fill the cap between them; a node that starts drifting
// afterwards sorts last and would never appear, even though it is the one thing that just changed.
//
// Four noisy apps at the per-app cap of 150 characters overrun the 480-character budget, which is
// what makes this a real trade rather than a hypothetical one - and the assertion below refuses to
// run unless the description really is at its cap. `pd_it_zzz` sorts after all of them, so in plain
// map order it is precisely what falls off the end; ordering newly affected apps first puts it at
// the FRONT of the report in which it first appears, which is where it survives.
TEST_F(ParamDriftIntegrationTest, ANewlyDriftedAppSurvivesTheDescriptionCap) {
  const std::string clean_value(60, 'a');
  const std::string drifted_value(60, 'b');
  const std::string kLate = "pd_it_zzz";  // sorts after every pd_it_ordN
  std::vector<std::string> noisy_ids;
  for (int n = 1; n <= 4; ++n) {
    noisy_ids.push_back("pd_it_ord" + std::to_string(n));
  }

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake, &noisy_ids, &kLate, &clean_value](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    for (const auto & id : noisy_ids) {
      std::map<std::string, nlohmann::json> clean;
      for (int p = 0; p < 12; ++p) {
        clean.emplace("p" + std::to_string(p), clean_value);
      }
      t->set_params("/" + id, clean);
    }
    t->set_params("/" + kLate, {{"gain", 1.0}});
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 100}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  apps.reserve(noisy_ids.size() + 1);
  for (const auto & id : noisy_ids) {
    apps.emplace_back(id, "/" + id);
  }
  apps.emplace_back(kLate, "/" + kLate);
  set_apps(apps);
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx)) << "not every app was read, so nothing has a baseline yet";
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent, otherwise the ordering assertions below prove nothing";

  // The four early-sorting apps drift first and fill the description between them.
  for (const auto & id : noisy_ids) {
    std::map<std::string, nlohmann::json> drifted;
    for (int p = 0; p < 12; ++p) {
      drifted.emplace("p" + std::to_string(p), drifted_value);
    }
    fake->set_params("/" + id, drifted);
  }
  ASSERT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc("graph_watchdog").size() >= AggregatedFault::kMaxDescriptionChars;
      },
      *det, ctx))
      << "the description never filled up, so nothing was competing for room and the ordering could "
         "not have mattered either way: "
      << latest_failed_desc("graph_watchdog");
  ASSERT_EQ(latest_failed_desc("graph_watchdog").find(kLate), std::string::npos)
      << "the app that has not drifted yet is already in the description, so its appearance below "
         "would say nothing: "
      << latest_failed_desc("graph_watchdog");

  // Now the late app drifts. The report in which it first appears must lead with it: that is the
  // whole of the newly-affected-first rule, and it is the only thing that can keep it in a
  // description four other apps have already overrun. Every poll below reads the CURRENT record.
  fake->set_params("/" + kLate, {{"gain", 9.0}});
  EXPECT_TRUE(poll_until(
      [this, &kLate]() {
        return latest_failed_desc("graph_watchdog").rfind(kLate + ":", 0) == 0;
      },
      *det, ctx))
      << "the app that just started drifting was cut from the description to make room for apps "
         "that merely sort earlier and had already been reported: "
      << latest_failed_desc("graph_watchdog");
}

// The other half of the ordering, and the one that decides what an operator reads when the
// description is full: a violation of a pin the operator WROTE outranks drift the detector
// captured by itself. The two are not equally interesting. A self-captured entry says a value moved
// and the detector has no idea whether that matters; an `expect` entry says a value an operator
// declared is wrong. Cutting the declared one to make room for four apps' worth of the other is the
// wrong trade, and the cap makes it a real trade rather than a hypothetical one.
//
// Nothing exercised this. Every `expect` configuration in this file also sets `baseline: false`, so
// a pinned violation and self-captured drift never coexisted in a single run and the pinned buckets
// of emit_order() were never reached with anything in them. With `baseline: true` the pins are
// checked against the same list_parameters() the capture uses, so one read produces both kinds -
// which is what makes the combination reachable at all.
//
// The setup is sized so the ordering is the only thing that can decide the outcome. Four noisy apps
// drift on six parameters each, which overruns the detector's per-app share of the description, so
// each contributes exactly that share - and four of them are needed to overflow the cap, not three:
// three capped entries and their separators are 150 + 2 + 150 + 2 + 150 = 454 characters against a
// budget of 480, and it is the fourth that pushes it over. The pinned app sorts LAST of the five,
// so in plain map order it is precisely what falls off the end. And the
// assertion is taken only after every app has been reported at least once, so the separate "newly
// affected first" rule (ANewlyDriftedAppSurvivesTheDescriptionCap) cannot be what keeps it in.
TEST_F(ParamDriftIntegrationTest, AnOperatorPinnedViolationOutranksSelfCapturedDrift) {
  const std::string kPin = "pinned_gain";
  const std::string kPinnedApp = "pd_ord_zzz";  // sorts after every pd_ord_noisyN
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake, &kPin](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    // Installed in the FACTORY so the very first read of every noisy node is the clean one. A
    // drifted first read would simply become that node's baseline and nothing would ever be
    // reported, leaving the pinned app alone in a description with room to spare.
    for (int n = 1; n <= 4; ++n) {
      std::map<std::string, nlohmann::json> clean;
      for (int p = 0; p < 6; ++p) {
        clean.emplace("p" + std::to_string(p), 1.0);
      }
      t->set_params("/pd_ord_noisy" + std::to_string(n), clean);
    }
    // The pinned app violates its pin from the first read and can carry nothing else: a pinned
    // parameter is excluded from the capture set, so its whole contribution is the one entry an
    // operator declared.
    t->set_params("/pd_ord_zzz", {{kPin, 9.0}});
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  // baseline stays true - that is the whole point - and the budget only keeps the sweep brisk.
  det->configure(nlohmann::json{{"tick_interval_ms", 100}, {"max_reads_per_tick", 8}, {"expect", {{kPin, 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  for (int n = 1; n <= 4; ++n) {
    const std::string id = "pd_ord_noisy" + std::to_string(n);
    apps.emplace_back(id, "/" + id);
  }
  apps.emplace_back(kPinnedApp, "/" + kPinnedApp);
  set_apps(apps);

  det->tick(ctx);  // builds the transport and hands the reader all five targets
  ASSERT_NE(fake, nullptr);
  // The sixth read STARTING means the first five have completed and been published - the reader is
  // a single thread round-robinning five targets - so the tick after it captures a clean baseline
  // for every noisy app.
  ASSERT_TRUE(fake->wait_until_reading(6)) << "the reader never got round all five apps";
  det->tick(ctx);

  // Now every noisy app drifts on all six of its parameters.
  for (int n = 1; n <= 4; ++n) {
    std::map<std::string, nlohmann::json> drifted;
    for (int p = 0; p < 6; ++p) {
      drifted.emplace("p" + std::to_string(p), 9.0);
    }
    fake->set_params("/pd_ord_noisy" + std::to_string(n), drifted);
  }
  ASSERT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc("graph_watchdog").size() >= AggregatedFault::kMaxDescriptionChars;
      },
      *det, ctx))
      << "the description never filled up, so nothing was competing for room and the ordering "
         "could not have mattered either way";

  // Keep ticking well past that point: several more full rotations, so every app has been reported
  // at least once and none of them is "newly affected" any more.
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }

  const std::string latest = latest_failed_desc("graph_watchdog");
  ASSERT_GE(latest.size(), AggregatedFault::kMaxDescriptionChars)
      << "the description is no longer at its cap, so the assertions below prove nothing: " << latest;
  ASSERT_NE(latest.find("pd_ord_noisy1"), std::string::npos)
      << "no self-captured drift is in the description at all, so the pinned entry had the cap to "
         "itself and its presence says nothing about priority: "
      << latest;
  EXPECT_NE(latest.find(kPinnedApp + ":\"" + kPin + "\" expected="), std::string::npos)
      << "the violation of a pin an operator WROTE was cut from the description an operator reads, "
         "to make room for self-captured drift on apps that merely sort earlier: "
      << latest;
  EXPECT_EQ(latest.rfind(kPinnedApp + ":", 0), std::size_t{0})
      << "the pinned violation is in the description but not at the front of it, so it survived on "
         "where its app happens to sort rather than on being pinned: "
      << latest;
}

// Aggregation: two nodes drift -> ONE GRAPH_PARAM_DRIFT (source "graph_watchdog") whose
// description names BOTH apps; revert one -> the aggregate stays raised (the other still
// drifts, no clear); revert both -> the aggregate clears.
TEST_F(ParamDriftIntegrationTest, MultiNodeAggregationOneFaultNamesBothThenClears) {
  auto a1 = std::make_shared<rclcpp::Node>("pd_it_agg1");
  a1->declare_parameter<double>("alpha", 1.0);
  auto a2 = std::make_shared<rclcpp::Node>("pd_it_agg2");
  a2->declare_parameter<double>("beta", 2.0);
  exec_.add_node(a1);
  exec_.add_node(a2);
  const auto nodes_guard = on_scope_exit([this, a1, a2] {
    exec_.remove_node(a1);
    exec_.remove_node(a2);
  });
  set_apps({{"pd_it_agg1", "/pd_it_agg1"}, {"pd_it_agg2", "/pd_it_agg2"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json::object());  // baseline mode
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_agg1"));
  ASSERT_TRUE(wait_param_service("/pd_it_agg2"));
  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx))
      << "no clear arrived, so at least one of the two apps was never read and has no baseline";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u);

  // Drift both -> a single aggregated FAILED naming both apps. Asserted on the CURRENT record, not
  // on the history: what an operator reads is the description the store last overwrote.
  a1->set_parameter(rclcpp::Parameter("alpha", 0.1));
  a2->set_parameter(rclcpp::Parameter("beta", 0.2));
  EXPECT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc_contains("graph_watchdog", {"pd_it_agg1", "pd_it_agg2"});
      },
      *det, ctx))
      << "one aggregated fault naming both apps never appeared: " << latest_failed_desc("graph_watchdog");

  // Revert ONE: the aggregate must STAY raised, because agg2 still drifts.
  //
  // The snapshot for that is taken only after the revert has SETTLED - long enough for agg1's
  // clean re-read to have landed and dropped it from the finding set. Counting raises from before
  // that point is the hole in "some raise exists": every raise emitted while agg1 was still in the
  // set satisfies it, so a detector that goes quiet the moment the set shrinks passes.
  a1->set_parameter(rclcpp::Parameter("alpha", 1.0));
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  const auto failed_settled = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_settled, *det, ctx))
      << "no raise arrived once the reverted node had dropped out of the finding set, so the "
         "aggregate went quiet while pd_it_agg2 is still drifting: "
      << latest_failed_desc("graph_watchdog");
  EXPECT_TRUE(latest_failed_desc_contains("graph_watchdog", {"pd_it_agg2"}))
      << "the current description no longer names the app that is still drifting: "
      << latest_failed_desc("graph_watchdog");
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "reverting one node cleared the aggregate while another still drifts";

  // Revert BOTH -> the aggregate clears.
  const auto passed_before2 = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  a2->set_parameter(rclcpp::Parameter("beta", 2.0));
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before2, *det, ctx));
}

// Guards the atomic expect-read fix. Config: baseline:false, 3 expect pins, max_reads_per_tick:6,
// two expect nodes each costing 3 pins x 3 round trips = 9. One node violates its LAST (sorted) pin
// z_bad from the start. Because expect is a std::map (keys sorted a_ok, b_ok, z_bad), a budget that
// truncated a visit at 6 would stop after b_ok, drop the drift from `observed`, and spuriously
// CLEAR a real, unfixed fault. The atomic read fetches the whole expect set for a visited node
// regardless of budget, so the drift stays raised and never flaps. Assert the aggregated FAILED
// stays raised with NO interleaved PASSED clear.
//
// The 250 ms tick is what keeps the assertion meaningful and the test honest at the same time. A
// visit owes 9 slots of read_gap(250, 6) = 41.7 ms, i.e. 375 ms, so a two-node rotation is 750 ms:
// the absence loop below spans more than two full rotations, and a bud1 visit that truncated its
// expect set has several chances to clear inside it. It also keeps the retry cadence short enough
// that a first read lost to discovery still recovers well inside poll_for_new's 13 s bound, which
// was sized to outlast the transport's 10 s negative cache.
TEST_F(ParamDriftIntegrationTest, ExpectBudgetTruncationDoesNotFlap) {
  auto bud1 = std::make_shared<rclcpp::Node>("pd_it_bud1");
  bud1->declare_parameter<double>("a_ok", 1.0);
  bud1->declare_parameter<double>("b_ok", 2.0);
  bud1->declare_parameter<double>("z_bad", 9.0);  // violates the LAST sorted pin from the start
  auto bud2 = std::make_shared<rclcpp::Node>("pd_it_bud2");
  bud2->declare_parameter<double>("a_ok", 1.0);
  bud2->declare_parameter<double>("b_ok", 2.0);
  bud2->declare_parameter<double>("z_bad", 3.0);  // all pins satisfied
  exec_.add_node(bud1);
  exec_.add_node(bud2);
  const auto nodes_guard = on_scope_exit([this, bud1, bud2] {
    exec_.remove_node(bud1);
    exec_.remove_node(bud2);
  });
  set_apps({{"pd_it_bud1", "/pd_it_bud1"}, {"pd_it_bud2", "/pd_it_bud2"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json{{"baseline", false},
                                {"tick_interval_ms", 250},
                                {"max_reads_per_tick", 6},
                                {"expect", {{"a_ok", 1.0}, {"b_ok", 2.0}, {"z_bad", 3.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/pd_it_bud1"));
  ASSERT_TRUE(wait_param_service("/pd_it_bud2"));

  // Establish the raised state (tolerates initial discovery misses). Once bud1's z_bad drift is
  // observed it stays in drifted_ (a failed read never erases it), so the clear count freezes.
  const auto failed0 = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed0, *det, ctx));
  const auto passed_after_raise = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  const auto failed_after_raise = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);

  // Tick across several full round-robin rotations. A visit owes 375 ms and there are two nodes,
  // so a rotation is 750 ms and this 2 s loop covers more than two of them - bud1 is re-read
  // several times inside it. The clear count must NOT advance: z_bad is caught atomically on every
  // bud1 visit.
  for (int i = 0; i < 40; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
    ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_after_raise)
        << "spurious clear at rotation tick " << i << " - expect set truncated by a per-call budget";
  }
  EXPECT_GT(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), failed_after_raise);
}

// The other side of the same rule, in the OTHER read mode. A visit in `expect` mode makes one
// get_parameter per pin and any of them can come back with something that is not NOT_FOUND - a
// wedged executor, parameter services torn down mid-sweep. Publishing what was collected up to that
// point would be reporting a partial view as a complete one: evaluate() finds nothing wrong in a
// view that is simply missing the parameter that is wrong, and the tick clears a fault that is
// still true. NOT_FOUND is the one failure that is deliberately carried on with, because a node is
// not at fault for a parameter it never declared.
//
// No live node can be asked to answer a get with SERVICE_UNAVAILABLE on demand, which is why this
// needs the stand-in.
TEST_F(ParamDriftIntegrationTest, AFailedExpectReadDoesNotClearARaisedDrift) {
  const std::string kFqn = "/pd_it_expfail";

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 20}, {"max_reads_per_tick", 8}, {"expect", {{"gain", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_expfail", kFqn}});

  // The stand-in answers 9.0, so the pin is violated from the first read.
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  fake->set_value(kFqn, 9.0);
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the pinned violation never raised, so there is no fault whose clearing this test can watch";

  // Every read of the pin now fails with SERVICE_UNAVAILABLE. The parameter is still 9.0.
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  fake->set_unreadable(kFqn);
  for (int i = 0; i < 40; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "a failed expect read was published as a complete view of the node, and the pinned "
         "violation it does not contain was cleared while the parameter is still wrong";

  // Positive control: a read that succeeds AND satisfies the pin does clear it, so the silence
  // above is the guard and not a detector that has stopped emitting.
  fake->set_value(kFqn, 1.0);
  fake->set_readable(kFqn);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the fault never cleared even once the pin was satisfied on a readable node";
}

// tick_interval_ms is injected by the plugin and is half of what paces the reader: read_gap divides
// the TICK PERIOD by the budget, so a detector that took the budget and ignored the period would
// pace every deployment at its own default no matter what an operator configured - and the coverage
// latency the README tells them to compute would be wrong by whatever factor they had changed the
// tick by. Nothing asserted on the key directly.
//
// Two runs differing ONLY in tick_interval_ms. At a budget of eight, one slot is period/8 and a
// baseline visit owes two of them, so sixteen visits after the first owe 4 x period: ~1.6 s at a
// 400 ms period and ~0.4 s at a 100 ms one. Both bands exclude the ignored-key behaviour, which
// paces both runs at the 1000 ms default and takes ~4 s.
TEST_F(ParamDriftIntegrationTest, TheInjectedTickIntervalPacesTheReader) {
  const auto time_seventeen_visits = [this](int tick_interval_ms) {
    ScriptedTransport * fake = nullptr;
    auto det = make_param_drift();
    EXPECT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
      auto t = std::make_unique<ScriptedTransport>();
      fake = t.get();
      return t;
    }));
    det->configure(nlohmann::json{{"tick_interval_ms", tick_interval_ms}, {"max_reads_per_tick", 8}});
    auto ctx = make_ctx(DetectorMode::Raise);
    set_apps({{"pd_it_pace", "/pd_it_pace"}});

    // One tick arms the reader; it round-robins its single target on its own from there, so the
    // elapsed time is pure pacing rather than the tick loop's cadence.
    const auto started = std::chrono::steady_clock::now();
    det->tick(ctx);
    EXPECT_NE(fake, nullptr);
    EXPECT_TRUE(fake != nullptr && fake->wait_until_reading(17))
        << "the reader never issued seventeen visits at a " << tick_interval_ms << " ms tick period";
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
  };

  const auto slow_ms = time_seventeen_visits(400);
  EXPECT_GT(slow_ms, 1200) << "seventeen visits at a 400 ms tick period took " << slow_ms
                           << " ms: the reader is pacing faster than the configured period allows";
  EXPECT_LT(slow_ms, 2400) << "seventeen visits at a 400 ms tick period took " << slow_ms
                           << " ms, which is the default 1000 ms period - the injected value was ignored";

  const auto fast_ms = time_seventeen_visits(100);
  EXPECT_LT(fast_ms, 800) << "seventeen visits at a 100 ms tick period took " << fast_ms
                          << " ms: shortening the tick period did not shorten the reader's slot, so "
                             "the injected value is not reaching read_gap";
  EXPECT_GT(fast_ms, 200) << "seventeen visits at a 100 ms tick period took " << fast_ms
                          << " ms: the reader is not honouring the budget at all";
}

// ================================================================================================
// Evidence for a clear, continued: the two states in which the detector knows NOTHING about an app
// and previously said "nothing is drifting anywhere" anyway.
// ================================================================================================

// A gate-denied app never reaches the armed set, so it never reaches the unread set and never joins
// the blocking one either. With nothing armed at all - which is the state of EVERY app for the
// first `warmup_cycles` ticks of every gateway or plugin start - the finding set is empty and the
// blocking set is empty, and that pair is exactly what emit_aggregated reads as "nothing is
// drifting anywhere". clear_fault carries no gate of its own (by design: a fault must always be
// clearable), so the clear goes out. With `healing_enabled`, which this package's README tells
// operators to switch on, those clears walk a persisted CONFIRMED GRAPH_PARAM_DRIFT toward HEALED
// before the detector has issued a single parameter round trip.
//
// The evidence rule does not care WHY an app has no usable read. "The round-robin has not reached
// it" and "the gate has not armed it" are the same state: nothing is known about the app, so
// nothing may be said about it.
TEST_F(ParamDriftIntegrationTest, NoClearWhileTheGateHasNotArmedAnyApp) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}});

  IntrospectionInput snap;
  {
    App a;
    a.id = "pd_it_warmup";
    a.bound_fqn = "/pd_it_warmup";
    snap.apps.push_back(a);
  }
  // A REAL gate, fed exactly one snapshot. warmup_cycles is the shipped default of 5 and one tick
  // has elapsed, so the app is KNOWN and NOT armed - the state every app is in immediately after a
  // restart, reached here without having to drive five seconds of wall clock.
  ReliabilityGate gate(5, gateway_.get(), &node_mutex_);
  gate.update(snap, 1);
  ASSERT_FALSE(reliability_allows(&gate, "pd_it_warmup"));

  auto ctx = make_ctx(DetectorMode::Raise);
  ctx.gate = &gate;
  ctx.snapshot = &snap;

  for (int i = 0; i < 40; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(300ms);  // reports are service round trips; let the in-flight ones land
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), 0u)
      << "cleared GRAPH_PARAM_DRIFT while the only app in the graph was still warming up and had "
         "never been read once - health the detector never measured, emitted on every tick of "
         "every restart";

  // Positive control: the guard releases the moment the app arms and is actually read. Without it
  // the zero above is satisfied just as well by a detector that emits nothing at all.
  gate.update(snap, 20);
  ASSERT_TRUE(reliability_allows(&gate, "pd_it_warmup"));
  ASSERT_NE(fake, nullptr);
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "no clear even once the app had armed and been read, so the guard never releases";
}

// The other state: an app that answered ONCE and then stopped for good. Its cached read keeps it in
// the measured set forever - it never joins the unread set, so it never blocks a clear and neither
// unread warning can ever fire about it. The detector then reports "nothing is drifting anywhere"
// off a snapshot of a moment that has passed, indefinitely, and says nothing at all about the fact.
//
// A cached read is evidence about the instant it was taken, not a standing licence. The moment a
// read of an app FAILS, what is cached about it stops being a measurement of the app's current
// state, and the detector is back to knowing nothing.
TEST_F(ParamDriftIntegrationTest, AnAppThatAnsweredOnceAndThenStoppedIsNoLongerCountedAsMeasured) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  // The default tick period against the default budget: one slot is 125 ms and a baseline visit,
  // successful or failed, is charged two of them - so a failed attempt lands every 250 ms and the
  // tenth arrives inside three seconds, well short of the sixty-first that ends the blocking.
  det->configure(nlohmann::json{{"tick_interval_ms", 1000}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_onceonly", "/pd_it_onceonly"}});

  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the app was never read, so it was never 'measured' and there is nothing here to go stale";
  ASSERT_NE(fake, nullptr);

  // And now it stops answering, permanently. Nothing about its parameters is known from here on.
  //
  // The baseline for the clear count is taken once a read has actually FAILED and the detector has
  // seen it, not the instant the stand-in is reconfigured: a clear emitted while the last completed
  // read was still a success is a clear made on evidence that was current, and this test is about
  // what happens afterwards.
  fake->set_unreadable("/pd_it_onceonly");
  ASSERT_TRUE(fake->wait_until_failed(1)) << "no read of the silenced app ever failed";
  det->tick(ctx);
  std::this_thread::sleep_for(200ms);  // let a report issued before that tick land
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 90 && log.count("consecutive parameter reads of 'pd_it_onceonly'") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(log.count("consecutive parameter reads of 'pd_it_onceonly'"), 1)
      << "ten consecutive reads of the app failed and nothing was said about it, because a cached "
         "read from before the node went silent keeps it out of the unread set entirely";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), passed_before)
      << "kept clearing GRAPH_PARAM_DRIFT off a cached read taken before the node stopped "
         "answering, so a drift introduced since then is reported as health";

  // Positive control: the app is not written off, it is simply unmeasured. Answering again puts it
  // straight back into the measured set and the clear flows.
  fake->set_readable("/pd_it_onceonly");
  const auto passed_again = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_again, *det, ctx))
      << "the app answered again and the clear never came back, so the guard latches instead of "
         "tracking what is currently known";
}

// Counting the give-up in FAILED ATTEMPTS rather than elapsed ticks is right - a tick horizon
// expires on exactly the large graphs where the rotation is slowest - but attempts alone are not a
// duration. The reader paces itself at `max_reads_per_tick` round trips per tick period, so an
// attempt costs read_gap x its charge in wall clock, and the whole horizon scales with the load
// knob. At the accepted ceiling of 100000 the gap floors at one microsecond, a failed baseline
// visit owes two of them, and sixty-one attempts are spent in a couple of milliseconds: the very
// first ticks of a run write off any node whose parameter service DDS has not finished discovering
// yet. The README's own remedy for a slow sweep - raise max_reads_per_tick - is what collapses it.
//
// So the horizon needs a floor that the budget cannot move. Ticks are that floor: they are the one
// quantity the load knob does not scale. Used as a CONJUNCTION with the attempt count it can only
// ever delay the give-up, never bring it forward, so it does not reintroduce the failure mode that
// counting attempts was adopted to fix.
TEST_F(ParamDriftIntegrationTest, GivingUpOnAnAppNeedsElapsedTicksAsWellAsFailedAttempts) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_slowstart");  // before the reader can ever answer for it
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 1000}, {"max_reads_per_tick", 100000}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_slowstart", "/pd_it_slowstart"}});

  det->tick(ctx);  // builds the transport and hands the reader its single target
  ASSERT_NE(fake, nullptr);
  // Far more failed attempts than the horizon counts, inside a handful of ticks. This is the whole
  // point: at this budget the ATTEMPT count is exhausted before the run has properly begun.
  ASSERT_TRUE(fake->wait_until_failed(300)) << "the reader never got anywhere near the attempt horizon at "
                                               "the ceiling budget, so this test is not measuring what it claims";
  for (int i = 0; i < 4; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(log.count("giving up on"), 0)
      << "wrote the app off as unmeasurable within five ticks of the run starting, purely because "
         "the load knob was raised: a node that is merely slow to advertise its parameter services "
         "is declared unmeasurable before DDS has finished discovering it";
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED), 0u)
      << "cleared GRAPH_PARAM_DRIFT on the strength of that write-off";

  // Positive control: the give-up is floored, not removed. Keep ticking and it arrives - otherwise
  // one node with no parameter service holds the fault CONFIRMED for the life of the gateway.
  for (int i = 0; i < 300 && log.count("giving up on") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(log.count("giving up on"), 1) << "the app never stopped blocking the clear, so the floor became a latch";
}

// An app id whose BINDING moves to a different node is a different node, whatever the id says. The
// reader already treats it that way - a target's arming token is keyed on (app_id, fqn), so a read
// in flight from the old binding can never publish into the new one - but nothing resets the state
// the old binding left behind: the captured baseline, the cached read, the failure count. The new
// node's first reading is then compared against values a DIFFERENT node happened to hold, which is
// a drift no later read can heal, because the reference itself belongs to somewhere else.
TEST_F(ParamDriftIntegrationTest, AnFqnReBindStartsFromAFreshBaseline) {
  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_value("/pd_it_bind_a", 1.0);
    t->set_value("/pd_it_bind_b", 5.0);
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 20}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);

  set_apps({{"pd_it_bind", "/pd_it_bind_a"}});
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the app was never read under its first binding, so there is no stale baseline to carry over";
  ASSERT_NE(fake, nullptr);

  // The same app id, now bound to a DIFFERENT node holding a different value.
  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  set_apps({{"pd_it_bind", "/pd_it_bind_b"}});
  for (int i = 0; i < 30; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), failed_before)
      << "reported the new node's value as drift against a baseline captured from a DIFFERENT node: "
      << latest_failed_desc("graph_watchdog");

  // Positive control: the new node's value really was captured as the reference, so moving off it
  // still reports. Without this the silence above is satisfied by a detector that captured nothing.
  const auto failed_before_drift = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  fake->set_value("/pd_it_bind_b", 9.0);
  EXPECT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before_drift, *det, ctx))
      << "no drift off the re-captured baseline, so the silence above proves nothing at all";
}

// The unmatched-pin warning waits for the sweep to cover the graph, which raises the question of
// whether it can still fire on a graph big enough to take many ticks to cover, and on one where
// short-lived nodes keep arriving. Both are answered here: sixty apps, and a run of one-sweep
// transients on top of them. The warning has to arrive in both.
//
// Sixty apps at eight round trips per 10 ms tick period is a 1.25 ms slot; an undeclared pin is
// charged two, so a rotation is 150 ms and the loop below spans many of them. The transients are
// the shape a real graph churns in - one `ros2` CLI invocation, present for a sweep under a name
// nothing will use again - and each of them legitimately holds the warning back while it is there.
// What must not happen is that they hold it back for good.
TEST_F(ParamDriftIntegrationTest, AnUnmatchedPinIsStillReportedOnALargeAndChurningGraph) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_declared(std::set<std::string>{"gain"});  // no node declares the misspelt pin
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 10}, {"max_reads_per_tick", 8}, {"expect", {{"gian", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  for (int i = 0; i < 60; ++i) {
    const std::string id = "pd_it_wide2_" + std::to_string(100 + i);  // uniform width -> sorted == creation order
    apps.emplace_back(id, "/" + id);
  }

  // Churn phase: every tick one short-lived app arrives under a name never reused, on top of the
  // sixty that stay. Nothing may be reported while such an app has not been looked at.
  for (int i = 0; i < 60; ++i) {
    auto churned = apps;
    churned.emplace_back("pd_it_cli_" + std::to_string(i), "/pd_it_cli_" + std::to_string(i));
    set_apps(churned);
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_NE(fake, nullptr);

  // The churn stops, the sixty stay. The sweep can now finish, and the misspelt pin must be named.
  set_apps(apps);
  for (int i = 0; i < 300 && log.count("no app declares 'gian'") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(log.count("no app declares 'gian'"), 1)
      << "the misspelt pin was never named on a sixty-app graph that had just been churning, so the "
         "warning is dead on exactly the graphs a silent pin is easiest to miss on";
  EXPECT_EQ(log.count("no app declares 'gain'"), 0) << "the pin every node declares was reported as unmatched";
}

// ================================================================================================
// Pins for behaviour that a mutation of the detector's own constants used to leave green.
// ================================================================================================

// The per-read timeout is a BOUND, and a bound is only real if a read that overruns it is
// abandoned. Nothing here could see that: tick() does no IO, so its duration says nothing, and the
// unresponsive-node test only requires the sweep to get PAST a node that never answers at all -
// which it does whether the bound is half a second or half an hour.
//
// The discriminator is a node that answers LATE rather than never. Its parameter service takes
// 1.5 s to reply, which is past the 0.5 s per-read bound and inside anything meaningfully larger.
// At the real bound the read is abandoned and the node is never read, so it can never appear in a
// description; raise the bound and the same reply lands and it does. The counter on the service
// handler is what keeps that absence honest: it proves the read was actually issued and the node
// actually served it late, rather than the transport never having found the service at all.
TEST_F(ParamDriftIntegrationTest, AReadThatOverrunsThePerReadBoundIsAbandoned) {
  const std::string kSlow = "pd_it_to_slow";  // sorts after pd_it_to_ok: read second in the rotation
  const std::string kOk = "pd_it_to_ok";
  std::atomic<int> slow_calls{0};

  auto ok = std::make_shared<rclcpp::Node>(kOk);
  ok->declare_parameter<double>("speed_limit", 3.5);  // violates the pin below from the first read
  exec_.add_node(ok);
  const auto ok_guard = on_scope_exit([this, ok] {
    exec_.remove_node(ok);
  });

  // The late node. list_parameters is the round trip an `expect` read makes first, so holding it is
  // what makes the whole read overrun; get_parameters exists so discovery finds a complete node.
  auto slow = std::make_shared<rclcpp::Node>(kSlow, rclcpp::NodeOptions().start_parameter_services(false));
  auto slow_list = slow->create_service<rcl_interfaces::srv::ListParameters>(
      "/" + kSlow + "/list_parameters", service_callback<rcl_interfaces::srv::ListParameters>(
                                            [&slow_calls](const rcl_interfaces::srv::ListParameters::Request & /*req*/,
                                                          rcl_interfaces::srv::ListParameters::Response & resp) {
                                              ++slow_calls;
                                              std::this_thread::sleep_for(1500ms);
                                              resp.result.names = {"speed_limit"};
                                            }));
  auto slow_get = slow->create_service<rcl_interfaces::srv::GetParameters>(
      "/" + kSlow + "/get_parameters",
      service_callback<rcl_interfaces::srv::GetParameters>([](const rcl_interfaces::srv::GetParameters::Request & req,
                                                              rcl_interfaces::srv::GetParameters::Response & resp) {
        for (std::size_t i = 0; i < req.names.size(); ++i) {
          rcl_interfaces::msg::ParameterValue v;
          v.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          v.double_value = 7.5;  // would violate the pin, if it were ever read
          resp.values.push_back(v);
        }
      }));
  const auto slow_idle = add_idle_parameter_services(slow, "/" + kSlow);
  // Its own executor: the 1.5 s sleep owns whatever thread serves it, and blocking the fixture's
  // executor would stall the healthy node and the fake fault service with it.
  rclcpp::executors::SingleThreadedExecutor slow_exec;
  slow_exec.add_node(slow);
  std::thread slow_spin([&slow_exec] {
    slow_exec.spin();
  });
  const auto slow_guard = on_scope_exit([&slow_exec, &slow_spin] {
    slow_exec.cancel();
    if (slow_spin.joinable()) {
      slow_spin.join();
    }
  });

  set_apps({{kOk, "/" + kOk}, {kSlow, "/" + kSlow}});
  auto det = make_param_drift();
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 100}, {"max_reads_per_tick", 8}, {"expect", {{"speed_limit", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);

  ASSERT_TRUE(wait_param_service("/" + kOk));
  ASSERT_TRUE(wait_param_service("/" + kSlow));

  // Positive control FIRST: the healthy node's pinned violation is reported, which proves the
  // transport, the reader and the aggregate are all alive on this graph.
  ASSERT_TRUE(poll_until(
      [this, &kOk]() {
        return latest_failed_desc_contains("graph_watchdog", {kOk, "got=3.5"});
      },
      *det, ctx))
      << "the healthy node's pinned violation never reported, so the absence below proves nothing: "
      << latest_failed_desc("graph_watchdog");

  // And the late node WAS reached: it served the request, it just served it too late.
  for (int i = 0; i < 200 && slow_calls.load() == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_GT(slow_calls.load(), 0) << "the late node's parameter service was never called at all, so nothing "
                                     "here exercised the per-read bound";

  // Give a read that was NOT abandoned every chance to land: the reply is 1.5 s behind the request.
  for (int i = 0; i < 40; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_EQ(latest_failed_desc("graph_watchdog").find(kSlow), std::string::npos)
      << "a node whose parameter service took 1.5 s to answer was read anyway, so the per-read "
         "bound is no longer half a second and one slow node can hold the sweep for as long as it "
         "likes: "
      << latest_failed_desc("graph_watchdog");
}

// The unread warning is worth something only if it means what it says. Firing it at the first
// failed read makes it a bringup notice: one dropped reply on a busy graph is not "this node does
// not answer", and an operator who is told it every time stops reading the log. Both sides of the
// threshold are pinned here, off the ATTEMPT counter rather than off wall clock, because the
// attempt count is the thing under test and pacing it by time would make the assertion a function
// of the budget instead.
TEST_F(ParamDriftIntegrationTest, TheUnreadWarningWaitsForTenFailedAttemptsNotOne) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_tenth");
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  // One slot is 125 ms and a failed baseline visit is charged two, so attempts arrive every 250 ms:
  // slow enough that the assertion below cannot race the attempt it is asserting about.
  det->configure(nlohmann::json{{"tick_interval_ms", 1000}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_tenth", "/pd_it_tenth"}});

  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_failed(5)) << "the reader never made five attempts against the silent app";
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("consecutive parameter reads of"), 0)
      << "named the app after a handful of failed reads: a single dropped reply is not evidence "
         "that a node has stopped answering, and a warning that fires on one is noise";

  for (int i = 0; i < 200 && log.count("consecutive parameter reads of 'pd_it_tenth'") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_EQ(log.count("consecutive parameter reads of 'pd_it_tenth'"), 1)
      << "the app that never answers was never named, so there is no threshold here to pin";
  EXPECT_GE(fake->failures(), 10) << "the warning fired after only " << fake->failures()
                                  << " failed attempts, short of the ten it documents";
}

// A per-app detail that overruns its share is cut from the MIDDLE, keeping both ends. The head
// names the app and its first drifted parameter; the tail is what stops the entry ending mid-token
// and is the only place the last of a long run of drifted parameters can survive. Head-only
// truncation renders the whole tail unreachable and leaves every existing assertion green, because
// they all look at the front of the entry.
TEST_F(ParamDriftIntegrationTest, AnOverlongPerAppDetailKeepsItsTailAsWellAsItsHead) {
  const std::string kApp = "pd_it_tail";
  // Six drifted parameters render to about 230 characters, comfortably past the 150 an app may
  // spend, and the last of them carries a name nothing else in the text can produce.
  const std::vector<std::string> names{"p1_aaaa", "p2_bbbb", "p3_cccc", "p4_dddd", "p5_eeee", "zz_lastone"};

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake, &kApp, &names](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    std::map<std::string, nlohmann::json> clean;
    for (const auto & n : names) {
      clean.emplace(n, 1.0);
    }
    t->set_params("/" + kApp, clean);  // installed in the factory: the first read must be the clean one
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 100}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{kApp, "/" + kApp}});

  ASSERT_TRUE(wait_for_baseline_evidence(*det, ctx)) << "the app was never read, so it has no baseline to drift from";
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED), 0u)
      << "baseline capture must be silent for the assertions below to mean anything";

  std::map<std::string, nlohmann::json> drifted;
  for (const auto & n : names) {
    drifted.emplace(n, 9.0);
  }
  fake->set_params("/" + kApp, drifted);
  ASSERT_TRUE(poll_until(
      [this]() {
        return latest_failed_desc("graph_watchdog").find("...") != std::string::npos;
      },
      *det, ctx))
      << "the entry never overran its share, so there was nothing to trim and this test would pass "
         "under any trimming rule at all: "
      << latest_failed_desc("graph_watchdog");

  const std::string desc = latest_failed_desc("graph_watchdog");
  EXPECT_EQ(desc.rfind(kApp + ":\"p1_aaaa\"", 0), std::size_t{0})
      << "the head no longer names the app and its first drifted parameter: " << desc;
  EXPECT_NE(desc.find("zz_lastone"), std::string::npos)
      << "the trimmed entry keeps only its head, so the last of a long run of drifted parameters is "
         "unreachable however many of them there are: "
      << desc;
  EXPECT_EQ(desc.find("p3_cccc"), std::string::npos)
      << "nothing was actually cut, so 'both ends' is not what is being tested here: " << desc;
}

// A graph on which every armed app has been GIVEN UP on is swept in name only. Nothing answered, so
// nothing is known about which parameters exist anywhere, and a pin cannot be called unmatched on
// the strength of it - that is a warning telling an operator to go and fix a name which may well be
// correct, produced by a detector that read nothing.
TEST_F(ParamDriftIntegrationTest, AnUnmatchedPinIsNotReportedWhenEveryAppWasGivenUpOn) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_allmute");
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{
      {"baseline", false}, {"tick_interval_ms", 10}, {"max_reads_per_tick", 8}, {"expect", {{"gian", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_allmute", "/pd_it_allmute"}});

  // Tick well past the point the only app is given up on: from there the sweep is "complete" in the
  // sense the clear uses, and nothing but the answered-something guard stands between the pin and a
  // warning about it.
  for (int i = 0; i < 400 && log.count("giving up on") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(log.count("giving up on"), 1) << "the app was never given up on, so the swept-in-name-only state this "
                                             "test is about was never reached";
  for (int i = 0; i < 60; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(log.count("no app declares 'gian'"), 0)
      << "called the pin unmatched on a graph where not one app ever answered a read, which says "
         "nothing whatever about which parameters the graph declares";

  // Positive control: once an app does answer and still does not declare the pin, the warning is
  // due and must arrive. Otherwise the zero above is a detector that never reports anything.
  fake->set_declared(std::set<std::string>{"gain"});
  fake->set_readable("/pd_it_allmute");
  for (int i = 0; i < 300 && log.count("no app declares 'gian'") == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(log.count("no app declares 'gian'"), 1)
      << "the pin was never reported even once an app had answered and did not declare it";
}

// The unread warnings are one-shot per RUN OF FAILURES, not once per process. An app that goes
// quiet, comes back, and goes quiet again has failed twice, and the second time is worth saying:
// the operator who fixed it needs to know it has broken again. What makes that possible is erasing
// the latch when the app leaves the unread set - and leaving the latch behind is invisible, because
// silence is what a working latch looks like too.
TEST_F(ParamDriftIntegrationTest, TheUnreadWarningFiresAgainAfterTheAppRecoversAndFailsOnceMore) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_unreadable("/pd_it_relapse");
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 20}, {"max_reads_per_tick", 8}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{"pd_it_relapse", "/pd_it_relapse"}});

  const std::string needle = "consecutive parameter reads of 'pd_it_relapse'";
  for (int i = 0; i < 400 && log.count(needle) == 0; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(log.count(needle), 1) << "the app that never answered was never named the first time";

  // It recovers. The reader reads it, the failure count is dropped, and so must the latch be.
  fake->set_readable("/pd_it_relapse");
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the recovered app was never read again, so nothing could have reset its latch either way";

  // And it breaks again.
  fake->set_unreadable("/pd_it_relapse");
  for (int i = 0; i < 400 && log.count(needle) < 2; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(log.count(needle), 2)
      << "the app broke a second time and nothing was said, because the one-shot latch from the "
         "first run of failures was never erased - so from the first failure onward this app is "
         "silent for the life of the gateway";
}

// The description's ordering has four buckets and the cross combination is what decides a real
// report: a pinned violation that has already been reported against self-captured drift that is
// brand new. Priority runs pinned-before-self-captured FIRST and newly-affected-before-reported
// only WITHIN a bucket, so the pinned entry leads even though the other one is the thing that just
// changed - an operator who declared a value cares that it is still wrong more than about a value
// the detector itself decided to watch.
//
// The two rules were only ever exercised one at a time, so swapping the middle two buckets left the
// whole suite green. The assertion is taken on the FIRST report the new app appears in, because
// that is the only report in which the two orderings differ at all.
TEST_F(ParamDriftIntegrationTest, AReportedPinnedViolationStillOutranksBrandNewSelfCapturedDrift) {
  const std::string kPin = "pinned_gain";
  const std::string kPinned = "pd_ord_x_pinned";  // sorts AFTER the self-captured app
  const std::string kSelf = "pd_ord_a_self";

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake, &kPin, &kPinned, &kSelf](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    t->set_params("/" + kPinned, {{kPin, 9.0}});  // violates its pin from the very first read
    t->set_params("/" + kSelf, {{"gain", 1.0}});  // clean, so its first read is a silent capture
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 50}, {"max_reads_per_tick", 8}, {"expect", {{kPin, 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  set_apps({{kSelf, "/" + kSelf}, {kPinned, "/" + kPinned}});

  // The pinned violation is reported on its own for a while: by the time the other app drifts it is
  // firmly in the already-reported bucket, which is the whole point of the combination.
  ASSERT_TRUE(poll_until(
      [this, &kPinned]() {
        return latest_failed_desc_contains("graph_watchdog", {kPinned});
      },
      *det, ctx))
      << "the pinned violation never reported, so there is no reported-pinned entry to rank";
  ASSERT_NE(fake, nullptr);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_EQ(latest_failed_desc("graph_watchdog").find(kSelf), std::string::npos)
      << "the self-capturing app is in the description already, so its arrival below says nothing: "
      << latest_failed_desc("graph_watchdog");

  // Now the self-capturing app drifts. It is brand new to the report and it sorts first, so plain
  // map order and newly-affected-first both put it ahead; only pinned-first keeps the pin in front.
  fake->set_params("/" + kSelf, {{"gain", 9.0}});
  ASSERT_TRUE(poll_until(
      [this, &kSelf]() {
        return !first_failed_desc_containing("graph_watchdog", kSelf).empty();
      },
      *det, ctx))
      << "the self-captured drift was never reported at all";
  const std::string first = first_failed_desc_containing("graph_watchdog", kSelf);
  EXPECT_EQ(first.rfind(kPinned + ":", 0), std::size_t{0})
      << "brand-new self-captured drift was put ahead of a violation of a pin an operator wrote, so "
         "on a full description the declared value is the one that gets cut: "
      << first;
}

// The captured baseline is dropped on the THIRD consecutive missed sweep, and both ends of that
// matter. Dropping it sooner re-captures a node on its already-drifted value after an ordinary
// dropped discovery poll - a confirmed fault heals while the parameter is still wrong, and can
// never fire again because the new baseline IS the wrong value. Dropping it later means a restart,
// which is the documented way to accept a new configuration, keeps reporting the operator's own
// change as drift.
//
// Existing coverage only pinned the short side. `prune_grace` is set well above the horizon here so
// the FINDING prune cannot reach it: the baseline forget is the only thing that can be under test.
TEST_F(ParamDriftIntegrationTest, TheBaselineForgetHorizonIsExactlyThreeMissedSweeps) {
  struct Case {
    int absences;
    bool expect_drift;
    const char * why;
  };
  const std::vector<Case> cases{
      {2, true,
       "two missed sweeps re-baselined the node, so one dropped discovery poll away from that "
       "launders a real drift into the new reference"},
      {3, false,
       "three missed sweeps did NOT re-baseline the node, so a restart - the documented way to "
       "apply a new configuration - reports the operator's own change as drift"},
  };

  int seq = 0;
  for (const auto & [absences, expect_drift, why] : cases) {
    SCOPED_TRACE("absences=" + std::to_string(absences));
    ++seq;
    const std::string id = "pd_it_forget" + std::to_string(seq);
    const std::string fqn = "/" + id;

    ScriptedTransport * fake = nullptr;
    auto det = make_param_drift();
    ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
      auto t = std::make_unique<ScriptedTransport>();
      fake = t.get();
      return t;
    })) << "the transport factory hook did not reach param_drift";
    // prune_grace far above the absence counts below, so `dropping` is never true and the finding
    // prune cannot stand in for the baseline forget.
    det->configure(nlohmann::json{{"tick_interval_ms", 10}, {"max_reads_per_tick", 8}, {"prune_grace", 60}});
    auto ctx = make_ctx(DetectorMode::Raise);

    set_apps({{id, fqn}});
    const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
    ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
        << "the app was never read, so it has no baseline whose fate this case could observe";
    ASSERT_NE(fake, nullptr);

    // Exactly `absences` sweeps without it. prune_vanished counts one missed sweep per tick.
    set_apps({});
    for (int i = 0; i < absences; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(20ms);
    }

    // It comes back holding a value it never held while it was present.
    fake->set_value(fqn, 5.0);
    set_apps({{id, fqn}});
    const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
    bool drifted = false;
    for (int i = 0; i < 60 && !drifted; ++i) {
      det->tick(ctx);
      std::this_thread::sleep_for(20ms);
      drifted = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED) > failed_before;
    }
    EXPECT_EQ(drifted, expect_drift) << why << ": " << latest_failed_desc("graph_watchdog");
  }
}

// Withholding a clear is correct and, from outside the process, completely invisible. No fault is
// raised, no fault is cleared, and a GRAPH_PARAM_DRIFT already in the store simply never heals -
// which is exactly what a detector that is working and finding nothing looks like. Worse, the apps
// doing the holding are usually ones the per-app unread warning may NOT name: nothing has been
// attempted against them, because they are waiting their turn or the gate has not armed them, and
// naming those would be a bringup warning about healthy nodes. So the aggregate has to speak for
// itself: once per episode, how long it has been withheld and how many apps are behind it.
//
// 160 apps at one round trip per 200 ms tick period is a rotation of a minute, so the tail of the
// graph stays uncontacted for the whole run. No sleeps: the horizon is counted in TICKS.
TEST_F(ParamDriftIntegrationTest, AWithheldClearSaysSoInTheLog) {
  const LogCapture log;

  ScriptedTransport * fake = nullptr;
  auto det = make_param_drift();
  ASSERT_TRUE(det->set_parameter_transport_factory_for_test([&fake](rclcpp::Node *) {
    auto t = std::make_unique<ScriptedTransport>();
    fake = t.get();
    return t;
  })) << "the transport factory hook did not reach param_drift";
  det->configure(nlohmann::json{{"tick_interval_ms", 200}, {"max_reads_per_tick", 1}});
  auto ctx = make_ctx(DetectorMode::Raise);

  std::vector<std::pair<std::string, std::string>> apps;
  for (int i = 0; i < 160; ++i) {
    const std::string id = "pd_it_held" + std::to_string(1000 + i);
    apps.emplace_back(id, "/" + id);
  }
  set_apps(apps);
  det->tick(ctx);
  ASSERT_NE(fake, nullptr);
  ASSERT_TRUE(fake->wait_until_reading(1)) << "the reader never started, so nothing here is under test";

  const std::string needle = "is being withheld from clearing";
  for (int i = 0; i < 30; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 0) << "announced a withheld clear after thirty ticks: an ordinary rotation on a "
                                     "large graph takes longer than that, so this fires during normal bringup";

  for (int i = 0; i < 200; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1)
      << "the clear has been withheld for over two hundred ticks and nothing in the log says so, "
         "so an operator watching GRAPH_PARAM_DRIFT refuse to heal has no way to tell a graph the "
         "sweep cannot cover from a detector that is working normally (got "
      << log.count(needle) << " lines)";

  // The episode ends when the graph is actually covered, and the warning must re-arm with it: a
  // one-shot-per-process latch would say nothing at all the next time coverage breaks down.
  set_apps({apps.front()});
  const auto passed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_PASSED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "no clear even once the graph was one already-read app, so the episode never ended";

  set_apps(apps);
  for (int i = 0; i < 300; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 2)
      << "coverage broke down a second time and nothing was said, so the warning is one-shot per "
         "process rather than per episode";
}

// An `expect` pin on a parameter the node DECLARED and never SET.
//
// Not a corner case and not the NOT_FOUND carve-out. rclcpp lets a node declare a dynamically typed
// parameter with no value, and its parameter service answers that name with a PARAMETER_NOT_SET
// VALUE rather than an absent one - so the transport gets a parameter back and returns SUCCESS
// carrying null. The pin therefore DOES fire, which is right: a pin says the value must be 1.0, and
// a node holding no value at all is not holding 1.0. Nothing asserted that in either direction.
//
// What the fault SAYS has to survive the round trip too. A NaN double dumps as `null` as well, so
// one description said the same thing about "the node never set this parameter" and "the value is
// NaN" - two states with different remedies, told apart by nothing. Both are pinned here, on the
// same node and in the same description, so the text has to distinguish them or this fails.
//
// Deliberately a LIVE node and the real transport, not the scripted stand-in. The premise is a
// reply shape rclcpp produces; a stand-in scripted to produce it would restate the premise instead
// of testing it.
TEST_F(ParamDriftIntegrationTest, AnExpectPinOnADeclaredButUnsetParameterRaisesAndReadsAsUnset) {
  auto node = std::make_shared<rclcpp::Node>("pd_it_unset");
  rcl_interfaces::msg::ParameterDescriptor dynamic_typing;
  dynamic_typing.dynamic_typing = true;  // a statically typed declaration demands a value or an override
  node->declare_parameter("threshold", rclcpp::ParameterValue{}, dynamic_typing);         // declared, never set
  node->declare_parameter<double>("headroom", std::numeric_limits<double>::quiet_NaN());  // a value, and it is NaN
  exec_.add_node(node);
  const auto node_guard = on_scope_exit([this, node] {
    exec_.remove_node(node);
  });
  set_apps({{"pd_it_unset", "/pd_it_unset"}});

  auto det = make_param_drift();
  det->configure(nlohmann::json{{"baseline", false}, {"expect", {{"threshold", 1.0}, {"headroom", 1.0}}}});
  auto ctx = make_ctx(DetectorMode::Raise);
  ASSERT_TRUE(wait_param_service("/pd_it_unset"));  // service up before a read-driven raise is asserted

  const auto failed_before = count_faults("graph_watchdog", ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new("graph_watchdog", ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "an expect pin on a parameter the node declared and never set raised nothing at all, so the "
         "operator is told the pin is satisfied by a node holding no value for it";

  const std::string desc = latest_failed_desc("graph_watchdog");
  ASSERT_NE(desc.find("threshold"), std::string::npos) << "the unset parameter is not named: " << desc;
  ASSERT_NE(desc.find("headroom"), std::string::npos) << "the NaN parameter is not named: " << desc;

  // Each entry's own `got=` field, cut at the entry separator so the two cannot be confused.
  const auto got_of = [&desc](const std::string & param) {
    const auto name_at = desc.find('"' + param + '"');
    if (name_at == std::string::npos) {
      return std::string{};
    }
    const auto got_at = desc.find(" got=", name_at);
    if (got_at == std::string::npos) {
      return std::string{};
    }
    const auto end = desc.find(';', got_at);
    return desc.substr(got_at + 5, end == std::string::npos ? std::string::npos : end - got_at - 5);
  };
  const std::string unset_got = got_of("threshold");
  const std::string nan_got = got_of("headroom");
  ASSERT_FALSE(unset_got.empty()) << "no got= field for the unset parameter in: " << desc;
  ASSERT_FALSE(nan_got.empty()) << "no got= field for the NaN parameter in: " << desc;

  EXPECT_EQ(unset_got, "<unset>") << "a parameter the node declared and never set reads as '" << unset_got
                                  << "' in the fault an operator sees: " << desc;
  EXPECT_NE(unset_got, nan_got) << "the unset parameter and the NaN one both read as '" << unset_got
                                << "', so the description cannot tell an operator which fault this is: " << desc;
}
