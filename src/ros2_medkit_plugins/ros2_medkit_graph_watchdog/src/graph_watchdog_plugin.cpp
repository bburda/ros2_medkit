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
#include "ros2_medkit_graph_watchdog/graph_watchdog_plugin.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <set>
#include <string>
#include <utility>
#include <vector>

// rclcpp's create_client() does not spell its QoS parameter the same way on every distro
// this package builds for: Humble takes only rmw_qos_profile_t, Lyrical and Rolling take
// only rclcpp::QoS, and Jazzy takes both but marks the rmw_qos_profile_t overload
// deprecated. Humble's rclcpp ships no <rclcpp/version.h> at all, so the absence of that
// header is what identifies Humble below.
#if defined(__has_include)
#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#endif
#endif

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"  // kGraphWatchdogEntityId
#include "ros2_medkit_graph_watchdog/detector_config.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"  // min_node_death_miss_grace
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Mirrors NodeDeathDetector's own kDefaultMissGrace (node_death_detector.cpp, anonymous
// namespace, not reachable from here) - kept in sync by convention since this is only a
// fallback used when config carries no explicit "detectors.node_death.miss_grace".
constexpr int kDefaultNodeDeathMissGrace = 2;

/// How often the tick thread comes back to drain the plugin's private executors while it
/// waits out the tick interval. Both the lifecycle watcher's ~/transition_event
/// subscriptions and the fault client are deliberately kept out of every gateway executor
/// (see lifecycle_watcher.hpp and set_context() below), so this poll is the ONLY thing
/// that runs them. Draining once per tick instead would delay every transition by up to a
/// full tick interval (1 s by default) and let a bringup burst overrun the subscription's
/// KeepLast(10) queue, losing the intermediate transitions the watcher's saw_transition /
/// errorprocessing logic reads.
constexpr std::chrono::milliseconds kPrivateDrainInterval{20};

/// Wall-clock cap on a single drain of one private executor. spin_some() never blocks
/// waiting for work, so this only bounds a pathological burst of already-queued work; it
/// exists so a flood cannot push the next tick out indefinitely.
constexpr std::chrono::milliseconds kPrivateDrainBudget{5};

/// create_client() with an explicit callback group, spelled the way each supported distro
/// wants it (see the <rclcpp/version.h> note above). Jazzy is rclcpp 28, Lyrical 32+;
/// anything older only offers the rmw_qos_profile_t form, where it is not deprecated.
template <typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr create_client_in_group(rclcpp::Node * node, const std::string & name,
                                                                    const rclcpp::CallbackGroup::SharedPtr & group) {
#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
  return node->create_client<ServiceT>(name, rclcpp::ServicesQoS(), group);
#else
  return node->create_client<ServiceT>(name, rmw_qos_profile_services_default, group);
#endif
}

/// What node_death's own configure() will resolve `prune_grace` to, mirroring that
/// detector's exact validation (a non-negative integer no larger than
/// kMaxNodeDeathGraceTicks) so this function and the config actually injected into
/// node_death's own dcfg (set_context()'s detector loop, below) can never disagree - both
/// call this, so there is exactly one place that decision is made.
///
/// Falls back to `plugin_prune_grace` (clamped into node_death's own accepted range - a
/// defensive clamp, not another validation pass: load_parameters() only checks
/// `>= 0` for its own field) exactly when node_death's own configure() would ALSO fall back:
/// the per-detector value is absent, the wrong JSON type, negative, or past
/// kMaxNodeDeathGraceTicks. Before this existed, the plugin's own retention math used this
/// same fallback while set_context() injected the plugin default only when the per-detector
/// key was ABSENT - so a PRESENT but malformed `detectors.node_death.prune_grace` reached
/// node_death's configure() unfiltered, which falls back to ITS OWN hardcoded default
/// instead of `plugin_prune_grace`. The two fallbacks coincide at their shared default (60)
/// and only diverge when an operator's plugin-scope `prune_grace` differs from it - which is
/// exactly the gap a test that never varies the plugin-scope value off 60 cannot catch.
int resolve_node_death_prune_grace(const nlohmann::json & config_snapshot, int plugin_prune_grace) {
  if (config_snapshot.contains("detectors") && config_snapshot["detectors"].is_object()) {
    const auto & detectors = config_snapshot["detectors"];
    if (detectors.contains("node_death") && detectors["node_death"].is_object()) {
      const auto & node_death_cfg = detectors["node_death"];
      // ROS/JSON integers are wide: read int64_t and range-check it BEFORE narrowing to int,
      // mirroring node_death_detector.cpp's own identical read of this same field. A value
      // above kMaxNodeDeathGraceTicks (or above INT_MAX) narrowed first can wrap back into
      // an accepted-looking band and pass silently.
      if (node_death_cfg.contains("prune_grace") && node_death_cfg["prune_grace"].is_number_integer()) {
        const std::int64_t wide = node_death_cfg["prune_grace"].get<std::int64_t>();
        if (wide >= 0 && wide <= kMaxNodeDeathGraceTicks) {
          return static_cast<int>(wide);
        }
      }
    }
  }
  return std::min(plugin_prune_grace, static_cast<int>(kMaxNodeDeathGraceTicks));
}
}  // namespace

ros2_medkit_gateway::IntrospectionResult
GraphWatchdogPlugin::introspect(const ros2_medkit_gateway::IntrospectionInput & input) {
  ros2_medkit_gateway::IntrospectionResult result;
  ros2_medkit_gateway::App app;
  app.id = kGraphWatchdogEntityId;
  app.name = "Graph watchdog";
  app.description = "Graph-level silent-fault detectors; every GRAPH_* fault is scoped here";
  // This App stands for the graph itself, not for a ROS node, so it has no binding to
  // derive an FQN from. Without `external` the fault scope resolves it to an empty FQN
  // and drops it (fault_scope.cpp's resolve_app_source_fqn), which would leave every
  // GRAPH_* fault exactly as unreachable as it was under the host Component.
  app.external = true;
  app.is_online = true;
  app.source = "plugin";
  // Attach to the host Component so the faults also roll up to /components/<host>/faults
  // (a Component resolves its scope through its child apps). With no Component in the
  // snapshot the App still stands alone and /apps/graph_watchdog/faults works.
  if (!input.components.empty() && !input.components.front().id.empty()) {
    app.component_id = input.components.front().id;
  }
  result.new_entities.apps.push_back(std::move(app));
  return result;
}

GraphWatchdogPlugin::GraphWatchdogPlugin() = default;
GraphWatchdogPlugin::~GraphWatchdogPlugin() {
  shutdown();
}

void GraphWatchdogPlugin::configure(const nlohmann::json & config) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
}

void GraphWatchdogPlugin::load_parameters() {
  std::lock_guard<std::mutex> lock(config_mutex_);
  if (config_.contains("tick_interval_ms") && config_["tick_interval_ms"].is_number_integer()) {
    const int candidate = config_["tick_interval_ms"].get<int>();
    if (candidate > 0) {
      tick_interval_ms_ = candidate;
    } else {
      log_warn("ignoring non-positive tick_interval_ms=" + std::to_string(candidate) + "; keeping " +
               std::to_string(tick_interval_ms_));
    }
  }
  if (config_.contains("warmup_cycles") && config_["warmup_cycles"].is_number_integer()) {
    const int candidate = config_["warmup_cycles"].get<int>();
    if (candidate >= 0) {
      warmup_cycles_ = candidate;
    } else {
      log_warn("ignoring negative warmup_cycles=" + std::to_string(candidate));
    }
  }
  if (config_.contains("prune_grace") && config_["prune_grace"].is_number_integer()) {
    const int candidate = config_["prune_grace"].get<int>();
    if (candidate >= 0) {
      prune_grace_ = candidate;
    } else {
      log_warn("ignoring negative prune_grace=" + std::to_string(candidate));
    }
  }
}

int GraphWatchdogPlugin::compute_departed_retention_ticks(const nlohmann::json & config_snapshot) const {
  // Read "detectors.node_death.miss_grace" directly off the full config tree (mirroring
  // node_death_detector.cpp's own nested read of the same field) - node_death's own
  // configure() has not run yet at the point set_context() needs this value, so its
  // own kDefaultMissGrace/clamping cannot be reused directly here.
  int node_death_miss_grace = kDefaultNodeDeathMissGrace;
  if (config_snapshot.contains("detectors") && config_snapshot["detectors"].is_object()) {
    const auto & detectors = config_snapshot["detectors"];
    if (detectors.contains("node_death") && detectors["node_death"].is_object()) {
      const auto & node_death_cfg = detectors["node_death"];
      // ROS/JSON integers are wide: read int64_t and range-check it BEFORE narrowing to int,
      // mirroring node_death_detector.cpp's own identical read of this same field. A
      // value above kMaxNodeDeathGraceTicks (or above INT_MAX) narrowed first can wrap back
      // into an accepted-looking band and pass the ">= 0" check silently - node_death's own
      // configure() would reject that same value and keep its default, so a plugin that
      // narrowed first would size this window from a number the detector never actually
      // uses, under- or over-running the retention this function exists to get right.
      if (node_death_cfg.contains("miss_grace") && node_death_cfg["miss_grace"].is_number_integer()) {
        const std::int64_t wide = node_death_cfg["miss_grace"].get<std::int64_t>();
        if (wide >= 0 && wide <= kMaxNodeDeathGraceTicks) {
          node_death_miss_grace = static_cast<int>(wide);
        }
      }
    }
  }
  // prune_grace is a per-detector key whose plugin-scope value is only a default, so the
  // retention window must be computed from whatever node_death will ACTUALLY clamp its
  // prune_ticks_ to - see resolve_node_death_prune_grace()'s own doc for why this must be
  // the SAME function set_context() uses to inject node_death's own dcfg, not a parallel
  // re-derivation of the same rule.
  const int node_death_prune_grace = resolve_node_death_prune_grace(config_snapshot, prune_grace_);
  // node_death's own configure() unconditionally raises miss_grace to its wall-clock floor
  // (min_node_death_miss_grace(), detector_config_keys.hpp) whenever the configured tick is fast
  // enough to need it - "unconditionally" because that floor applies to node_death's
  // DEFAULT miss_grace too, not only an operator-set one. A fast tick is exactly this
  // plugin's own tick_interval_ms_, already loaded by the time set_context() calls this
  // (load_parameters() runs first). Applied here too, and outside the "detectors.node_death
  // exists" check above: mirroring only the RAW config value - or only flooring it when an
  // operator happened to configure node_death at all - would leave this window sized for a
  // miss_grace smaller than the one node_death will actually use, which is the exact
  // under-run the rest of this function's comment describes.
  node_death_miss_grace = std::max(node_death_miss_grace, min_node_death_miss_grace(tick_interval_ms_));
  // prune_ticks mirrors node_death's OWN prune_ticks_ clamp (node_death_detector.cpp's
  // configure()): max(prune_grace, miss_grace + 1). The departed-lifecycle label must
  // survive not just past the FIRST tick node_death evaluates suppression on
  // (T_depart + miss_grace, see test_node_death_integration.cpp's
  // CleanShutdownDepartureAtMissGraceBoundaryIsSuppressed), but all the way through
  // node_death's own RECLAIM tick (T_depart + miss_grace + prune_ticks): only a durable
  // suppressor (lifecycle_clean_shutdown IS one, see suppressor.hpp) may feed
  // NodeLivenessTracker::prune(), and that reclaim can only happen while the suppressor
  // still actively vetoes the id - i.e. while the label is still cached. A retention window
  // anchored at T_depart instead and merely as long as prune_ticks itself (`max(prune_grace,
  // miss_grace + 1)`, with no `+ node_death_miss_grace + 1` on top) would expire miss_grace
  // ticks too early: the suppressor would then abstain right at the reclaim tick, node_death
  // would RAISE instead of silently reclaiming, and - because the label is now gone for good
  // - the id could never be suppressed again: a permanent false GRAPH_NODE_DISAPPEARED for a
  // cleanly-shut-down node. The extra "+1" below keeps one full tick of margin past the
  // reclaim tick itself. See
  // test_node_death_integration.cpp's CleanShutdownDepartureIsReclaimedNotReRaisedPastRetention
  // for the regression this formula fixes.
  const int prune_ticks = std::max(node_death_prune_grace, node_death_miss_grace + 1);
  return prune_ticks + node_death_miss_grace + 1;
}

void GraphWatchdogPlugin::set_context(ros2_medkit_gateway::PluginContext & context) {
  ctx_ = ros2_medkit_gateway::as_ros_plugin_context(context);
  load_parameters();
  if (!ctx_ || !ctx_->node()) {
    log_error("no ROS node in context; graph_watchdog disabled");
    return;
  }
  auto * node = ctx_->node();
  clock_ = std::make_unique<WatchdogClock>(node);

  nlohmann::json config_snapshot;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_snapshot = config_;
  }

  // Computed BEFORE gate_ is constructed: node_death's own configure() (which normally
  // owns its miss_grace default) has not run yet at this point.
  const int departed_retention_ticks = compute_departed_retention_ticks(config_snapshot);
  gate_ = std::make_unique<ReliabilityGate>(warmup_cycles_, node, &node_mutex_, departed_retention_ticks);

  // The fault client gets the same treatment as the lifecycle subscriptions: its own
  // callback group, automatic executor registration disabled, added only to a private
  // executor this plugin pumps from the tick thread. rclcpp::AnyExecutable holds a strong
  // reference to the CLIENT it dispatches just as it does for a subscription, and it is a
  // local of the executor thread, so a client left in the node's default group can have
  // its last reference released - and ~Client run, mutating the node's entity registry -
  // on a gateway executor thread at an arbitrary moment. Creating it under node_mutex_
  // and never handing it to a gateway executor is what keeps every reference to it on
  // this plugin's own threads.
  //
  // The client is used strictly fire-and-forget (see DetectorContext::raise_fault: it
  // guards on service_is_ready() and discards the future), so nothing consumes the
  // responses - but rclcpp keeps per-request state until an executor processes the
  // response, so the private executor MUST still be drained or pending_requests_ grows for
  // the life of the process. That drain is wait_for_next_tick()'s second job.
  {
    std::lock_guard<std::mutex> node_lock(node_mutex_);
    fault_client_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                                      /*automatically_add_to_executor_with_node=*/false);
    fault_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    fault_executor_->add_callback_group(fault_client_group_, node->get_node_base_interface());
    fault_client_ = create_client_in_group<ros2_medkit_msgs::srv::ReportFault>(node, "/fault_manager/report_fault",
                                                                               fault_client_group_);
  }

  std::set<std::string> seen_ids;
  for (auto & detector : DetectorRegistry::instance().create_all()) {
    const std::string id = detector->id();
    if (!seen_ids.insert(id).second) {
      log_warn("duplicate detector id '" + id + "'; skipping the duplicate");
      continue;
    }
    std::vector<std::string> mode_warnings;
    const DetectorMode mode = parse_detector_mode(config_snapshot, id, &mode_warnings);
    for (const auto & warning : mode_warnings) {
      log_warn(warning);
    }
    if (mode == DetectorMode::Off) {
      continue;
    }
    try {
      // extract_detector_config() isolates each detector to config["detectors"][id], so a
      // plugin-scope knob never reaches a detector on its own. Inject them under names every
      // detector treats as plugin plumbing rather than operator keys (see
      // plugin_injected_detector_keys()): node_death needs the tick period to turn its
      // tick-counted grace into a wall-clock window, and every prune-aware detector reads
      // prune_grace.
      auto dcfg = extract_detector_config(config_snapshot, id);
      dcfg["tick_interval_ms"] = tick_interval_ms_;
      // prune_grace is a plugin-scope DEFAULT, not an override: every prune-aware detector
      // documents it in its own key table and reads it off the config it is handed, so
      // `detectors.tf_stale.prune_grace` must win over the plugin-scope value. Injecting it
      // unconditionally silently discarded the per-detector setting.
      if (!dcfg.contains("prune_grace")) {
        dcfg["prune_grace"] = prune_grace_;
      }
      if (id == "node_death") {
        // node_death is the one detector this plugin also predicts the behaviour of BEFORE
        // configure() runs (compute_departed_retention_ticks(), above, sizes the lifecycle
        // watcher's own retention off it) - so what actually reaches its configure() here
        // must be byte-for-byte the value that prediction assumed, not merely "the operator's
        // value when well-formed, else whatever node_death happens to default to on its own."
        // Re-resolving (rather than trusting the injection above) also covers a PRESENT but
        // malformed detectors.node_death.prune_grace, which the absence check above leaves
        // untouched even though node_death's own configure() will still reject it.
        dcfg["prune_grace"] = resolve_node_death_prune_grace(config_snapshot, prune_grace_);
      }
      detector->configure(dcfg);
    } catch (const std::exception & e) {
      log_error("detector '" + id + "' configure() threw: " + std::string(e.what()) + "; skipping");
      continue;
    }
    detectors_.emplace_back(std::move(detector), mode);
  }

  // A typo in the detector ID itself is the worst of the config typos: the whole block
  // becomes a no-op, so `detectors.param_drfit.mode: off` leaves a detector the operator
  // explicitly disabled still raising on the robot. Nothing downstream can catch it -
  // extract_detector_config only ever looks up ids it already knows - so compare the
  // configured ids against the registered set here, where both are in hand.
  if (config_snapshot.contains("detectors") && config_snapshot["detectors"].is_object()) {
    std::string registered;
    for (const auto & id : seen_ids) {
      if (!registered.empty()) {
        registered += ", ";
      }
      registered += id;
    }
    for (const auto & item : config_snapshot["detectors"].items()) {
      if (seen_ids.count(item.key()) == 0) {
        log_warn("config block 'detectors." + item.key() + "' matches no registered detector and has no effect" +
                 " (registered: " + registered + ")");
      }
    }
  }

  // Run the tick on a dedicated thread, NOT a gateway wall-timer: detectors do blocking
  // parameter/service reads, and the gateway's small executor is only safe because
  // blocking never runs on an executor thread (see the gateway's main.cpp). This keeps
  // the blocking off the executor's callback group entirely.
  tick_thread_ = std::thread([this] {
    run_tick_loop();
  });

  // Quiesce the tick thread BEFORE rclcpp invalidates the context (this runs even on a SIGINT,
  // ahead of context invalidation). It signals shutdown and waits, bounded, for run_tick_loop() to
  // leave its loop so no detector is mid-rcl-read when the context dies. It does NOT join the
  // thread (shutdown()/the destructor own that) - it only makes the thread quiescent.
  context_ = node->get_node_base_interface()->get_context();
  pre_shutdown_handle_ = context_->add_pre_shutdown_callback([this] {
    {
      std::lock_guard<std::mutex> lock(tick_cv_mutex_);
      shutdown_requested_.store(true);
    }
    tick_cv_.notify_all();
    std::unique_lock<std::mutex> lock(tick_cv_mutex_);
    tick_cv_.wait_for(lock, std::chrono::seconds(2), [this] {
      return tick_loop_exited_;
    });
  });

  log_info("graph_watchdog up: " + std::to_string(detectors_.size()) + " detector(s)");
}

void GraphWatchdogPlugin::run_tick_loop() {
  while (!shutdown_requested_.load()) {
    tick();
    wait_for_next_tick();
  }
  // Announce that no more ticks (and so no more rcl reads) will run. The pre-shutdown callback
  // waits on this so the context is not invalidated while a tick is mid-flight.
  {
    std::lock_guard<std::mutex> lock(tick_cv_mutex_);
    tick_loop_exited_ = true;
  }
  tick_cv_.notify_all();
}

void GraphWatchdogPlugin::drain_private_executors() {
  // Each drain is guarded separately: an escape here would unwind out of the thread
  // function and std::terminate the gateway (the same reason tick() catches the gate
  // update), and a throw from one executor must not cost the other its drain.
  try {
    if (gate_) {
      gate_->pump_lifecycle_events(kPrivateDrainBudget);
    }
  } catch (const std::exception & e) {
    log_error("graph_watchdog lifecycle pump threw: " + std::string(e.what()));
  }
  try {
    if (fault_executor_) {
      fault_executor_->spin_some(kPrivateDrainBudget);
    }
  } catch (const std::exception & e) {
    log_error("graph_watchdog fault-response pump threw: " + std::string(e.what()));
  }
}

void GraphWatchdogPlugin::wait_for_next_tick() {
  // The gap between two ticks is not dead time any more. The lifecycle watcher's
  // ~/transition_event subscriptions and the fault client both sit in callback groups that
  // no gateway executor collects, so the tick thread - the only thread allowed anywhere
  // near them - has to run them itself. Poll instead of sleeping the whole interval in one
  // go: otherwise a transition event sits unread for a full tick, and every fault reported
  // during a tick keeps its pending-request entry until the next one.
  //
  // The condition variable still owns the sleeping, so shutdown latency is unchanged: a
  // shutdown request wakes the wait immediately and both loop conditions re-test it.
  const auto next_tick = std::chrono::steady_clock::now() + std::chrono::milliseconds(tick_interval_ms_);
  while (!shutdown_requested_.load()) {
    drain_private_executors();
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_tick) {
      return;
    }
    const auto slice = std::min(std::chrono::duration_cast<std::chrono::nanoseconds>(next_tick - now),
                                std::chrono::duration_cast<std::chrono::nanoseconds>(kPrivateDrainInterval));
    std::unique_lock<std::mutex> lock(tick_cv_mutex_);
    tick_cv_.wait_for(lock, slice, [this] {
      return shutdown_requested_.load();
    });
  }
}

void GraphWatchdogPlugin::tick() {
  if (shutdown_requested_.load()) {
    return;
  }
  // No tick_mutex_ here: shutdown() joins this thread before tearing down any member, so
  // tick() can never race teardown. Running lock-free is what keeps the blocking reads
  // below from stalling the get_routes() status handler (which does take tick_mutex_).
  tick_count_.fetch_add(1);
  // The gate update runs first and can throw: get_entity_snapshot()'s contract is not
  // enforced, and LifecycleWatcher::update() calls create_subscription() (invalid topic
  // name, RCLError, bad_alloc). An escape would kill the tick thread and every detector,
  // so guard it like each detector->tick() below.
  ros2_medkit_gateway::IntrospectionInput snapshot;
  try {
    clock_->mark_tick();
    if (ctx_) {
      snapshot = ctx_->get_entity_snapshot();
    }
    if (gate_) {
      gate_->update(snapshot, tick_count_.load());
    }
  } catch (const std::exception & e) {
    log_error("graph_watchdog gate update threw: " + std::string(e.what()));
  }
  auto * node = ctx_ ? ctx_->node() : nullptr;
  for (auto & [detector, mode] : detectors_) {
    // Bail between detectors so shutdown() (which joins this thread) is not held for a full
    // sweep of blocking reads once teardown has been requested.
    if (shutdown_requested_.load()) {
      return;
    }
    DetectorContext dctx;
    dctx.gateway_node = node;
    dctx.node_mutex = &node_mutex_;
    dctx.clock = clock_.get();
    dctx.gate = gate_.get();
    dctx.mode = mode;
    dctx.fault_client = fault_client_;
    dctx.snapshot = &snapshot;
    dctx.cancelled = &shutdown_requested_;
    try {
      detector->tick(dctx);
    } catch (const std::exception & e) {
      log_error("detector '" + detector->id() + "' threw: " + e.what());
    }
  }
}

void GraphWatchdogPlugin::shutdown() {
  // Guard on teardown_done_, NOT shutdown_requested_: the pre-shutdown callback may already have set
  // shutdown_requested_ to quiesce the tick loop, and guarding on that would make this early-return
  // WITHOUT joining tick_thread_, leaving a joinable std::thread to std::terminate() at destruction.
  if (teardown_done_.exchange(true)) {
    return;
  }
  // Deregister the pre-shutdown callback (it captures `this`): a no-op if it already ran during a
  // normal rclcpp shutdown, but it prevents a stale callback firing on a destroyed plugin when the
  // plugin is torn down without an rclcpp shutdown.
  if (context_) {
    context_->remove_pre_shutdown_callback(pre_shutdown_handle_);
  }
  // Wake the tick loop out of its sleep and JOIN it before touching any member: after the
  // join no tick runs, so the teardown below cannot race an in-flight tick. (A tick
  // in progress finishes first; its blocking reads are bounded by the parameter
  // transport's own service timeout, so the join cannot hang on an unresponsive node.)
  // Set the stop flag under tick_cv_mutex_ before notifying so the signal cannot fall into the gap
  // between the tick loop testing shutdown_requested_ and entering wait_for(): re-acquiring the mutex
  // the waiter holds while it re-checks the predicate guarantees it observes the flag we just set,
  // bounding shutdown latency to well under one tick interval. This is a latency optimization, not
  // a correctness requirement - wait_for()'s timeout and predicate already backstop a missed wakeup.
  {
    std::lock_guard<std::mutex> lock(tick_cv_mutex_);
    shutdown_requested_.store(true);
  }
  tick_cv_.notify_all();
  if (tick_thread_.joinable()) {
    tick_thread_.join();
  }
  // Fence the get_routes() status handler (the only other member reader) against teardown.
  std::lock_guard<std::mutex> lock(tick_mutex_);
  // Detectors first: each tick hands them a copy of fault_client_ in a DetectorContext, so
  // clearing them makes sure no detector-held copy outlives the reset below.
  detectors_.clear();
  {
    // Destroy the private executor BEFORE the client, for the same reason
    // LifecycleWatcher::reset() destroys its own executor before clearing the
    // subscriptions: while the executor lives it can hold its own strong reference to the
    // client, so resetting our handle alone would not necessarily release the last one and
    // ~Client would run at some later, less obvious point. Doing both here, under
    // node_mutex_, keeps the destruction on this thread and ordered against the entity
    // creation this plugin does on the same node. The scope matters: gate_->reset() below
    // takes node_mutex_ itself, and std::mutex is not recursive.
    std::lock_guard<std::mutex> node_lock(node_mutex_);
    fault_executor_.reset();
    fault_client_.reset();
    fault_client_group_.reset();
  }
  clock_.reset();
  if (gate_) {
    gate_->reset();
  }
  gate_.reset();
}

std::size_t GraphWatchdogPlugin::prune_pending_fault_requests_for_test() {
  // rclcpp guards pending_requests_ with its own mutex, so this is safe to call while the
  // tick thread is sending.
  return fault_client_ ? fault_client_->prune_pending_requests() : 0u;
}

std::vector<GraphWatchdogPlugin::PluginRoute> GraphWatchdogPlugin::get_routes() {
  auto handler = [this](const ros2_medkit_gateway::PluginRequest &, ros2_medkit_gateway::PluginResponse & res) {
    // tick_mutex_ guards gate_ against shutdown()'s teardown (which also holds it after
    // joining the tick thread), so the pointer cannot be reset mid-read. The concurrent
    // tick-thread gate_->update() is made safe against this status read by the gate's OWN
    // internal gate_mutex_ (status_json() and update() both take it); status_json() does
    // no blocking I/O, so this handler never stalls behind the detectors' blocking reads.
    std::lock_guard<std::mutex> lock(tick_mutex_);
    if (!gate_) {
      res.send_error(503, ros2_medkit_gateway::ERR_SERVICE_UNAVAILABLE,
                     "graph_watchdog reliability gate not initialized");
      return;
    }
    auto payload = gate_->status_json();
    // Detector-scoped status, beside the gate's own. `detectors_` is only ever mutated by
    // set_context() and by shutdown() (which holds tick_mutex_, taken above), and each
    // detector's own status_json() is required to be safe against a concurrent tick - see
    // Detector::status_json.
    nlohmann::json detectors = nlohmann::json::object();
    for (const auto & [detector, mode] : detectors_) {
      (void)mode;
      auto status = detector->status_json();
      if (!status.is_null()) {
        detectors[detector->id()] = std::move(status);
      }
    }
    if (!detectors.empty()) {
      payload["x-medkit-watchdog"]["detectors"] = std::move(detectors);
    }
    res.send_json(payload);
  };
  std::vector<PluginRoute> routes;
  routes.push_back({"GET", R"(x-medkit-watchdog)", std::move(handler)});
  return routes;
}

}  // namespace ros2_medkit_graph_watchdog
