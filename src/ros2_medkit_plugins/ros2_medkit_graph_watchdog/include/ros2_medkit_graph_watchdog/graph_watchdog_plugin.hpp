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
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/watchdog_clock.hpp"

namespace ros2_medkit_graph_watchdog {

class ReliabilityGate;  // defined in reliability_gate.hpp; kept out of this header (see gate_ below).

/// Gateway plugin: hosts the silent-fault detector fleet. It wires the fault client
/// and the tick loop, and fans each tick out to every registered detector.
///
/// Also an IntrospectionProvider, for one reason: it publishes the single App that
/// every GRAPH_* fault is scoped to (see graph_watchdog_entity_id()). Without an entity
/// it owns, a graph-level fault is reachable from no scoped endpoint at all - see
/// aggregated_fault.hpp for the full story.
class GraphWatchdogPlugin : public ros2_medkit_gateway::GatewayPlugin,
                            public ros2_medkit_gateway::IntrospectionProvider {
 public:
  // Out-of-line (even though = default) so every TU calling `new GraphWatchdogPlugin()`
  // does not need to instantiate ~unique_ptr<ReliabilityGate>() for the constructor's
  // exception-unwind path (gate_ is only forward-declared here; ReliabilityGate is
  // complete in graph_watchdog_plugin.cpp, where this is actually defined).
  GraphWatchdogPlugin();
  ~GraphWatchdogPlugin() override;
  GraphWatchdogPlugin(const GraphWatchdogPlugin &) = delete;
  GraphWatchdogPlugin & operator=(const GraphWatchdogPlugin &) = delete;
  GraphWatchdogPlugin(GraphWatchdogPlugin &&) = delete;
  GraphWatchdogPlugin & operator=(GraphWatchdogPlugin &&) = delete;

  std::string name() const override {
    return "graph_watchdog";
  }
  void configure(const nlohmann::json & config) override;
  /// PRECONDITION: called exactly once, before any tick. It builds `gate_` and the
  /// detector list and then spawns the tick thread, so everything the tick thread reads is
  /// published to it by that thread creation and needs no lock afterwards. A second call
  /// while the tick thread is running would rebuild both under a live reader with no
  /// synchronisation at all. The gateway honours this (one call site, gateway_node.cpp),
  /// which is why nothing here defends against the other case - stated rather than
  /// enforced, because a guard would be untestable through any caller that respects it.
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  std::vector<PluginRoute> get_routes() override;
  void shutdown() override;

  /// Publishes exactly one entity: the App that owns every GRAPH_* fault. It is attached
  /// to the host Component when the snapshot carries one, so the faults also roll up to
  /// `/components/<host>/faults` (a Component resolves its scope through its child apps).
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  uint64_t tick_count() const {
    return tick_count_.load();
  }

  /// Test-only: how many ReportFault responses the plugin's fault client is still waiting
  /// on. The client is fire-and-forget (DetectorContext::raise_fault discards the future),
  /// and rclcpp keeps per-request state until an executor processes the response, so this
  /// must fall back to 0 between ticks or the plugin leaks one entry per raised fault for
  /// the life of the process. DESTRUCTIVE: rclcpp exposes no non-mutating count, so this
  /// clears the pending map as it reads it.
  std::size_t prune_pending_fault_requests_for_test();

  /// Test-only: exposes the otherwise-private compute_departed_retention_ticks() so a unit
  /// test can drive its config-validation directly (malformed/oversized miss_grace or
  /// prune_grace) without constructing a full ROS gate/node - it has no ROS dependency of
  /// its own, only tick_interval_ms_/prune_grace_ (set via configure()+load_parameters())
  /// and the JSON it is handed.
  int compute_departed_retention_ticks_for_test(const nlohmann::json & config_snapshot) const {
    return compute_departed_retention_ticks(config_snapshot);
  }

 private:
  void load_parameters();
  void run_tick_loop();  ///< Body of tick_thread_: tick() + interruptible wait, until shutdown.
  /// Run both private executors once, each guarded so a throw from one cannot escape the
  /// tick thread or cost the other its drain.
  void drain_private_executors();
  /// Wait out one tick interval on tick_cv_, in short slices, draining the private
  /// executors between them: the gate's lifecycle ~/transition_event subscriptions and the
  /// fault client's responses. Both live in callback groups no gateway executor collects
  /// (see lifecycle_watcher.hpp and set_context()), so this poll is the only thing that
  /// runs them - and it deliberately runs on the tick thread, the same thread that creates
  /// and destroys them.
  void wait_for_next_tick();
  void tick();
  /// Departed-lifecycle retention horizon (ticks) for the ReliabilityGate's
  /// LifecycleWatcher: prune_ticks + node_death's own miss_grace + 1, where prune_ticks
  /// = max(prune_grace_, miss_grace + 1) mirrors node_death's own prune_ticks_ clamp.
  /// Must outlive node_death's own reclaim tick (T_depart + miss_grace + prune_ticks),
  /// not just its first suppression-evaluation tick, so a durable
  /// lifecycle_clean_shutdown veto is silently RECLAIMED before the cached label
  /// expires rather than lapsing into a permanent re-raise - see the .cpp for the full
  /// derivation. Reads "detectors.node_death.miss_grace" directly off
  /// `config_snapshot` (node_death's own configure() has not run yet at the point this
  /// is needed - see set_context()).
  int compute_departed_retention_ticks(const nlohmann::json & config_snapshot) const;

  ros2_medkit_gateway::RosPluginContext * ctx_ = nullptr;
  nlohmann::json config_;
  std::mutex config_mutex_;

  int tick_interval_ms_ = 1000;
  int warmup_cycles_ = 5;
  // Plugin-scope prune grace (ticks) injected into every detector's own config as
  // "prune_grace" (extract_detector_config() otherwise strips plugin-scope keys) -
  // see graph_watchdog_plugin.cpp's configure loop.
  int prune_grace_ = 60;

  std::unique_ptr<WatchdogClock> clock_;
  std::unique_ptr<ReliabilityGate> gate_;  ///< Incomplete type here is fine: dtor is out-of-line (.cpp).
  /// The fault client's own callback group, created with automatic executor registration
  /// DISABLED so no executor the gateway node is added to ever collects the client - that
  /// is what keeps every reference to it, and therefore ~Client, on this plugin's threads.
  rclcpp::CallbackGroup::SharedPtr fault_client_group_;
  /// The only executor that group is ever added to, pumped from the tick thread. It is
  /// what processes the ReportFault responses nobody reads; without it the client's
  /// pending-request map would grow without bound.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> fault_executor_;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr fault_client_;

  // The tick runs on a DEDICATED plugin thread, NOT a gateway wall-timer/executor
  // callback. Detectors do blocking parameter/service reads; the gateway's small
  // executor is safe only because blocking never happens on an executor thread (see
  // the gateway's main.cpp). shutdown() joins this thread BEFORE tearing down members,
  // so tick() needs no lock against teardown; tick_mutex_ then only serializes the
  // get_routes() status handler against that teardown.
  std::thread tick_thread_;
  std::condition_variable tick_cv_;
  std::mutex tick_cv_mutex_;
  bool tick_loop_exited_ = false;  ///< set (under tick_cv_mutex_) when run_tick_loop() leaves its loop
  // A pre-shutdown callback stops the tick thread while the rclcpp context is STILL valid (it runs
  // before context invalidation, even on a SIGINT). Detectors do rcl graph/service reads on the tick
  // thread; if the context invalidates mid-read, rcl ABORTS (SIGABRT) rather than throwing, so it
  // cannot be caught - the thread must be quiesced first. shutdown() (called from teardown) alone is
  // too late: on SIGINT the context is already dead by then.
  rclcpp::Context::SharedPtr context_;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;
  std::mutex node_mutex_;
  std::mutex tick_mutex_;  ///< Serializes the get_routes() handler against shutdown()'s member teardown.

  std::vector<std::pair<std::unique_ptr<Detector>, DetectorMode>> detectors_;

  std::atomic<uint64_t> tick_count_{0};
  std::atomic<bool> shutdown_requested_{false};  ///< stop signal for the tick loop; set by BOTH the
                                                 ///< pre-shutdown callback and shutdown()
  std::atomic<bool> teardown_done_{false};       ///< idempotency guard for shutdown()'s member teardown.
                                                 ///< Kept separate from shutdown_requested_ so the
                                                 ///< pre-shutdown callback stopping the tick loop cannot
                                                 ///< trip shutdown() into skipping the thread join.
};

}  // namespace ros2_medkit_graph_watchdog
