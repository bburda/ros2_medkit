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
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/param_drift_policy.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// The detector reports the drift and does not grade its impact: whether a changed parameter
// matters is a question about the robot, not about the graph, so triage belongs to whoever
// reads the fault. WARN keeps it visible without claiming the run is broken.
constexpr std::uint8_t kParamDriftSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
constexpr double kParamServiceTimeoutSec = 0.5;  // per-read timeout, same as the lifecycle GetState read
constexpr double kNegativeCacheTtlSec = 10.0;    // unreachable-node backoff
constexpr int kForgetGrace = 2;                  // missed-sweep grace before forgetting, mirrors WarmupTracker

// What one transport call costs the WATCHED node, in parameter-service round trips.
//
// Nothing in the build ties these literals to the transport they describe. Compiling the gateway's
// Ros2ParameterTransport into the plugin (see PARAMETER_TRANSPORT_SOURCES in CMakeLists.txt) only
// guarantees that the constants and the code are the same VERSION - a round trip added inside the
// transport would leave every one of them stale and still compile.
//
// Two DIFFERENT tests are needed to hold them true, and neither can do the other's job:
//
// - test/e2e/test_param_roundtrip_e2e.test.py measures what the TRANSPORT really costs, by running
//   it against a node that answers the parameter services by hand and counts every request it is
//   asked to serve. It reads none of the literals below - it drives the two transport functions
//   over the configurations endpoint with this detector switched OFF - so it catches a round trip
//   being added to or removed from the transport, and it catches nothing about these numbers. Set
//   kBaselineReadRoundTrips to 4 and it still passes.
// - the CHARGE, i.e. that these literals still match that measurement, is held by the timing cases
//   in test/test_param_drift_integration.cpp (ABaselineVisitIsChargedTwoRoundTripsAndNotTheColdStartPair,
//   TheBudgetIsSpentInRoundTripsNotCalls, AnUndeclaredPinIsChargedLessThanOneThatResolves). They
//   time the reader against a configured budget, which is a function of what it charges, so a
//   literal moved here moves the elapsed time and they go red.
//
// Every OTHER test in this package proves the detector's arithmetic GIVEN these numbers, and the
// stand-in transport they use counts CALLS.
//
// A baseline visit is one list_parameters(): a list request, then one batched get for every name
// it returned (ros2_parameter_transport.cpp:397 and :420).
constexpr int kBaselineReadRoundTrips = 2;
// The FIRST list_parameters() against a given node costs two MORE, because it primes the
// transport's defaults cache before doing its own read (:364 -> :1158 list + :1190 get). That
// cache has no TTL (:1163 - entries are only evicted under LRU pressure), so the extra pair is a
// one-off per node per gateway lifetime and is deliberately NOT charged here: this knob bounds a
// SUSTAINED sweep, and charging every read for a cost paid once per node would throttle the whole
// run to cover the first rotation. The README states the cold-contact excess instead.
//
// An expect visit is one get_parameter() per pinned name, and each of those is three round trips:
// a name-scoped list, a get, and a describe (:520, :553, :585). The describe is unconditional on
// the success path - the block at :584 has no enclosing guard, only the USE of its result at :586
// is guarded - so it is part of the price whether or not the descriptor is wanted. There is no
// cold-contact pair here: get_parameter never calls cache_default_values, so a `baseline: false`
// sweep pays the same three on its first read of a node as on every later one.
constexpr int kExpectReadRoundTrips = 3;
// A pin the node does not declare costs ONE round trip: the name-scoped list comes back empty and
// the transport returns NOT_FOUND without reaching the get or the describe (:538). `expect` pins
// are documented as checked on every node that HAS the parameter, so on a mixed graph most nodes
// answer NOT_FOUND for most pins, and charging those the full success price would throttle the
// sweep three times harder than the load it creates. NOT_FOUND has a second, rarer path that costs
// two (:571): the name-scoped list DOES find the name and the get then comes back with no value
// for it, which the GetParameters contract allows, so the transport stops one round trip later.
// The error code is the same on both paths and the caller cannot tell which it took, so the CHARGE
// below is the higher of them and deliberately overstates the common case: a bound may overstate a
// cost, it must never understate one. Both are measured in test_param_roundtrip_e2e.
//
// A parameter the node DECLARES and has not set is not either of those. rclcpp answers one with a
// PARAMETER_NOT_SET value rather than an absent one, so the transport gets a parameter back, pays
// for the unconditional describe and returns SUCCESS carrying null - three round trips, charged as
// kExpectReadRoundTrips. Charging it two would understate what the node pays.
constexpr int kExpectNotFoundRoundTrips = 2;

// Completed, FAILED read attempts against one armed app before we say so. Counted in attempts and
// never in elapsed ticks. An app the round-robin sweep has not reached yet has made no attempt at
// all, and on a large graph it waits many ticks for its turn: reporting that "no read has
// succeeded" for it would be a bringup warning about a perfectly healthy node, which is what an
// elapsed-tick threshold produced. Ten consecutive failures is past any single dropped reply and
// squarely into "this node does not answer".
constexpr int kUnreadReportAfterFailures = 10;
// A FLOOR in ticks, spent alongside the attempt counts and never instead of them.
//
// An attempt is not a duration. The reader paces itself at max_reads_per_tick round trips per tick
// period, so what an attempt costs in wall clock is read_gap x its charge - and the whole horizon
// therefore scales with the load knob. At the accepted ceiling of 100000 the gap floors at one
// microsecond and sixty-one failed attempts are spent in a couple of milliseconds, so the very
// first ticks of a run write off any node whose parameter service DDS has not finished discovering
// yet. The README's own remedy for a slow sweep - raise max_reads_per_tick - is exactly what
// collapses it.
//
// Ticks are the one quantity the budget does not scale, so they are the floor. Used as a
// CONJUNCTION with the attempt count this can only ever DELAY a report or a give-up, never bring
// one forward, so it does not reintroduce the failure mode that counting attempts was adopted to
// fix: a tick horizon that EXPIRES on a large graph. The values mirror the attempt counts, which at
// the default tick period is ten seconds before an app is named and a minute before it stops
// holding back a clear.
constexpr int kUnreadReportAfterTicks = kUnreadReportAfterFailures;
// Missed sweeps a node may be absent for before we treat it as restarted and drop its captured
// baseline. Deliberately the SAME window WarmupTracker absorbs (kDefaultForgetGrace): that header
// documents that DDS discovery routinely drops a node from a single poll without it restarting, so
// re-baselining on the first miss would re-capture the node's CURRENT - possibly already drifted -
// values on ordinary churn. A confirmed fault would then heal while the parameter is still wrong
// and could never fire again, because the new baseline IS the wrong value. The cost of waiting is
// that a respawn faster than this window keeps the old baseline and reports the operator's change
// as drift until they revert it or the node stays away longer.
constexpr int kBaselineForgetGrace = 2;
// Upper bound on the configurable prune grace. At the 1 s default tick this is already an hour of
// absence before a finding is dropped; beyond it the value is a typo or a unit mix-up, not a choice.
constexpr std::int64_t kMaxPruneGrace = 3600;
// Completed, FAILED read attempts after which an app stops holding back a clear.
//
// A clear asserts that nothing is drifting anywhere, and that is only true if we actually looked
// everywhere. An app the sweep has never ATTEMPTED therefore blocks the clear INDEFINITELY: no
// amount of elapsed time turns an unmeasured node into a measured one, and a horizon in ticks is
// guaranteed to expire on a graph whose rotation is longer than the horizon - the detector would
// then report health it never measured, on exactly the large graphs where the sweep is slowest.
//
// What does justify giving up is evidence: this many attempts were made against the node and every
// one of them failed. That is a node with no working parameter service (a non-rclcpp participant,
// parameter services disabled, a wedged executor), and there is no per-node opt-out for this
// detector, so without a bound one such node would keep a fixed drift reported as CONFIRMED for
// the lifetime of the gateway, with a description naming something that is no longer true. Past
// the bound the app is declared unmeasurable, logged once, and excluded from the blocking set.
//
// The count only grows while the app stays unread and is dropped the moment a read succeeds, so an
// app already given up on can never silently re-enter the blocking set - and one that does become
// readable leaves it through the front door, by being read.
constexpr int kUnmeasurableAfterFailures = 60;
// The tick floor for the give-up, spent alongside the attempt count. See kUnreadReportAfterTicks.
constexpr int kUnmeasurableAfterTicks = kUnmeasurableAfterFailures;
// Consecutive ticks a clear may be withheld before the reason is put in the log.
//
// Withholding is the right thing to do - health that was never measured is not health - but it is
// indistinguishable from the outside from a detector that is simply working and finding nothing:
// no fault is raised, no fault is cleared, and a GRAPH_PARAM_DRIFT already in the store just never
// heals. An operator has no way to tell "the graph is being covered" from "the graph has not been
// covered for the last twenty minutes and never will be", so the detector has to say which. Once
// per episode, and only past a horizon a healthy rotation on a large graph comfortably fits inside.
constexpr int kWithheldClearReportTicks = 60;
// Consecutive ticks an app may stay non-armed while holding a frozen finding. Freezing is right for
// a pause - the drift is still real and the gate only means "do not raise about an unsettled
// entity". But a node that leaves `active` for good is never re-evaluated (not armed) and never
// pruned (still present), so without this the aggregate re-raises its stale finding forever, even
// after an operator fixes the parameter. Mirrors the prune horizon.
constexpr int kFrozenHoldTicks = 60;
// Characters one app may contribute to the aggregated description. Without a per-app bound a single
// node drifting on many parameters fills the whole 480-char cap by itself, and every other app -
// including one whose violation an operator explicitly pinned - is cut regardless of the ordering.
// Ordering decides who goes first; this is what makes going second still mean something. Three apps
// fit comfortably, which is the case worth optimising for: the aggregate exists to say WHICH nodes
// drifted, and the per-parameter detail is a hint, not the record.
constexpr std::size_t kMaxAppDetailChars = 150;

/// One app the reader has been told to read, tagged with the token of the arming EPISODE it
/// belongs to.
///
/// The token, not the app id and not the fqn, is what the reader compares before publishing. Both
/// of those are stable across a de-arm and a re-arm, and tick() rewrites `targets` from scratch
/// every tick, so an app whose lifecycle gate closed and re-opened inside one (blocking, seconds
/// long) read window would pass an identity check and have a value sampled mid-transition published
/// as its next reading - which becomes its BASELINE, a false drift no later read can heal, and
/// exactly what the fence exists to prevent. A token is issued once per arming episode from a
/// monotonic counter and never reused, so an interrupted episode can never be mistaken for the one
/// that followed it.
struct ReadTarget {
  std::string app_id;
  std::string fqn;
  std::uint64_t token = 0;
};

/// An armed app with no usable read, and the EVIDENCE we have about why.
///
/// `failed_attempts` is the number of consecutive completed read attempts against this app that
/// failed. Zero means the round-robin sweep has not reached it yet - the app is waiting its turn,
/// nothing is known about it, and nothing may be said about it either.
struct UnreadApp {
  std::string app_id;
  int failed_attempts = 0;
};

/// One-shot latches for the two things we say about an app that will not answer, plus how long the
/// app has been in that state. Kept per app and dropped the moment the app leaves the unread set,
/// so a run of silence is reported once rather than every tick, and an app that starts answering
/// again can be reported afresh if it later stops.
struct UnreadLatch {
  bool reported = false;  ///< the "no read has succeeded" warning has been logged
  bool gave_up = false;   ///< the unmeasurable warning has been logged
  /// Consecutive ticks this app has been armed with no usable read. The FLOOR half of both
  /// horizons: see kUnreadReportAfterTicks for why an attempt count alone is not a duration.
  int ticks = 0;
};

// Reading a node's parameters is a BLOCKING service round-trip (up to kParamServiceTimeoutSec
// each, and observed to hang PAST that on a real Nav2 node stuck mid-transition, since the
// underlying DDS recv does not always honour the spin timeout). Doing it inline on the tick
// thread stalls the WHOLE detection loop - node_death, tf_stale, everything - the moment ONE
// node's parameter service is slow. So the reads run on a dedicated background thread and the
// tick thread only ever reads the cache. State shared between the two threads lives here,
// behind a shared_ptr so a reader still inside a read keeps it alive. Teardown joins the
// thread rather than detaching it, and shuts the transport down first so a blocking read
// returns instead of holding the join (see the destructor); a detached reader would still be
// making service calls while rclcpp tears the context down, which aborts rather than throws.
struct SharedIo {
  std::mutex mutex;
  std::condition_variable cv;
  bool stop = false;
  bool reader_stopped = false;            ///< set by the reader when it leaves the loop (no more rcl ops)
  bool baseline_capture = true;           ///< snapshot of policy config (set-once at reader start)
  std::vector<std::string> expect_names;  ///< pinned params to read when baseline_capture is false
  /// The ROS-neutral base, not the concrete ROS 2 transport. The detector builds the real one by
  /// default; a test can inject a stand-in to control read latency, force an empty batch, or fail
  /// construction outright - none of which is reachable through a live node.
  std::unique_ptr<ros2_medkit_gateway::ParameterTransport> transport;     // reader-thread only
  std::vector<ReadTarget> targets;                                        // armed apps to read, this episode
  std::uint64_t next_token = 1;                                           ///< monotonic; never reused
  std::map<std::string, std::map<std::string, nlohmann::json>> observed;  // app_id -> last usable read
  std::set<std::string> have;  ///< app_ids with a usable cached read (distinct from "read empty")
  /// app_id -> consecutive read attempts that COMPLETED and failed. Written by the reader under
  /// this mutex, read by the tick thread under it. An app absent from this map and absent from
  /// `have` has never been attempted at all, and that is the distinction the clear rests on: `have`
  /// says "we looked and it answered", an entry here says "we looked and it did not", and neither
  /// says "we have not looked yet". Erased on the first successful read.
  std::map<std::string, int> failed_reads;
  /// Pacing for the reader thread (set-once at reader start, alongside baseline_capture).
  /// `max_reads_per_tick` service round trips at the watched nodes per `tick_interval_ms` - one
  /// transport call is several of those, see reader_loop().
  int max_reads_per_tick = 8;
  int tick_interval_ms = kDefaultTickIntervalMs;
};

/// Blocking read of `fqn`'s parameters into `observed`. Runs ONLY on the reader thread (it is
/// the sole user of io.transport, so the transport needs no lock of its own here). Returns true
/// when `observed` is a usable, complete view; false when the node should be skipped (a non-
/// NOT_FOUND transport error, or a stop mid-read) so evaluate() never sees a partial view and
/// spuriously clears a real drift. Mirrors the original inline read_params semantics exactly.
///
/// `round_trips` accumulates what this visit is CHARGED against the WATCHED node in
/// parameter-service round trips - not transport calls, which are 1-3 round trips each, and never
/// below what the node really paid (see kExpectNotFoundRoundTrips). reader_loop turns it into the
/// pacing debt for the visit, so the knob bounds the load a node really sees.
bool read_params_io(SharedIo & io, const std::string & fqn, std::map<std::string, nlohmann::json> & observed,
                    int & round_trips) {
  if (io.baseline_capture) {
    round_trips += kBaselineReadRoundTrips;
    const auto result = io.transport->list_parameters(fqn);  // one list, then one batched get
    if (!result.success || !result.data.is_array() || result.data.empty()) {
      // An empty array is a SUCCESSFUL response that carries no view - the transport documents it
      // as legitimate, e.g. when a parameter declaration races the list. Treating it as "nothing
      // drifts here" is what the contract above exists to prevent: evaluate() would compare
      // against no observations at all, find nothing, and the tick would clear a real drift.
      // Every ROS 2 node declares use_sim_time, so a genuinely empty parameter set is not a case
      // worth trading that away for.
      return false;
    }
    for (const auto & p : result.data) {
      if (p.contains("name") && p.contains("value")) {
        // Through param_entry_value, not straight off "value": the entry also carries the TYPE, and
        // for a parameter the node declared and never set that is the only thing separating it from
        // a NaN double - both of which are `null` by the time a description is written. See
        // param_entry_value() and unset_param_value().
        observed.emplace(p["name"].get<std::string>(), param_entry_value(p));
      }
    }
    return true;
  }
  for (const auto & name : io.expect_names) {
    {
      std::lock_guard<std::mutex> lk(io.mutex);
      if (io.stop) {
        return false;  // shutting down mid-read: incomplete view -> skip (no spurious clear)
      }
    }
    const auto result = io.transport->get_parameter(fqn, name);
    // Charged from the OUTCOME rather than blind, because a name the node never declared costs it
    // a fraction of a successful read (see kExpectNotFoundRoundTrips). A call that throws is not
    // charged at all - reader_loop's own floor of one slot covers that, and a throw here means the
    // context is being torn down, not that the sweep is running hot.
    round_trips += result.error_code == ros2_medkit_gateway::ParameterErrorCode::NOT_FOUND ? kExpectNotFoundRoundTrips
                                                                                           : kExpectReadRoundTrips;
    if (result.success && result.data.contains("value")) {
      // Same reason as the baseline branch above: a SUCCESS carrying null is a parameter the node
      // declared and never set, and it must not reach the description as the `null` a NaN prints.
      observed.emplace(name, param_entry_value(result.data));
    } else if (result.error_code != ros2_medkit_gateway::ParameterErrorCode::NOT_FOUND) {
      return false;  // unreadable this tick -> skip, do not risk a spurious clear
    }
  }
  return true;
}

/// Background loop: round-robins the tick thread's current `targets`, doing one blocking read
/// at a time and publishing the result into the cache. A hung read stalls only THIS thread, not
/// the tick loop; on stop it shuts the transport down and exits, releasing the shared_ptr.
///
/// PACED at max_reads_per_tick service ROUND TRIPS per tick period - the unit the watched node
/// actually pays in, not transport calls, which are 1-3 round trips each (see
/// kBaselineReadRoundTrips / kExpectReadRoundTrips / kExpectNotFoundRoundTrips). Without pacing
/// this loop issues round-trips
/// back to back for the whole life of the gateway - tick() re-arms `targets` with every present
/// app on every tick, so there is always work and the cv predicate is always satisfied. Every one
/// of those round trips is served on the customer node's own executor. The short-circuit paths
/// turn it into a pure busy spin: a node in the transport's negative cache returns
/// SERVICE_UNAVAILABLE immediately, and `baseline: false` with an empty `expect` does zero IO and
/// returns true, so with all targets in either state the thread burns a core doing nothing. The
/// budget is also what the documented coverage latency of
/// ceil(round_trips_per_visit * node_count / max_reads_per_tick) ticks is computed from.
///
/// The cursor is POSITIONAL, not an identity. `idx` indexes a vector the tick thread rewrites from
/// scratch every tick, in sorted app-id order, so a target's slot is not stable: an app arriving
/// that sorts earlier shifts every later one down a slot, and one leaving shifts them up, which
/// re-reads an app just visited or jumps past the app that was due. Nobody is starved by that -
/// `idx` still advances by one per read, so every index of the CURRENT list is visited within
/// targets.size() reads - but the rotation is not a guaranteed round of exactly one read per app,
/// and under continuous node-name churn an individual app's turn can keep moving. Nothing the
/// detector SAYS about an app is allowed to rest on how long that app has gone unread; see
/// kUnreadReportAfterFailures and kUnmeasurableAfterFailures, which count failed ATTEMPTS.
///
/// Every completed visit also records its OUTCOME against the app (see `failed_reads`), which is
/// what lets the tick thread tell "this node will not answer" from "the sweep has not got to it
/// yet" without appealing to how much time has passed.
void reader_loop(std::shared_ptr<SharedIo> io) {
  std::size_t idx = 0;
  for (;;) {
    ReadTarget target;
    std::chrono::microseconds min_gap{0};
    {
      std::unique_lock<std::mutex> lk(io->mutex);
      io->cv.wait(lk, [&] {
        return io->stop || !io->targets.empty();
      });
      if (io->stop) {
        break;
      }
      target = io->targets[idx % io->targets.size()];
      ++idx;
      min_gap = read_gap(io->tick_interval_ms, io->max_reads_per_tick);
    }
    const auto read_started = std::chrono::steady_clock::now();
    std::map<std::string, nlohmann::json> observed;
    bool ok = false;
    int round_trips = 0;
    try {
      ok = read_params_io(*io, target.fqn, observed, round_trips);
    } catch (const std::exception &) {
      // A transport/rclcpp call can throw when the context is invalidated during shutdown, or
      // on a malformed service path. An escape would std::terminate the process (the reader has
      // no outer handler unlike the tick loop), so swallow it: drop this read and let the loop
      // re-check stop. Matches the guard the plugin puts around each detector->tick().
      ok = false;
    }
    {
      std::unique_lock<std::mutex> lk(io->mutex);
      if (io->stop) {
        break;
      }
      // Record the outcome only if this arming EPISODE is still current. The read ran with the
      // lock released and can take seconds, during which the tick thread may have dropped this
      // app - because its lifecycle gate closed, or because prune_vanished forgot it - and may
      // even have re-armed it since. Publishing anyway is not merely stale, it is harmful in
      // three ways: a value sampled mid-transition becomes the app's baseline when it re-arms,
      // which is a false drift that can never heal; a read landing after the prune re-inserts an
      // entry whose miss counter is gone, so no later prune can reach it and the maps grow
      // without bound under node-name churn; and a failure charged to an episode that is over
      // pushes an app that is now perfectly readable towards being given up on.
      //
      // Comparing the token rather than the app id is what closes the de-arm/re-arm case: the id
      // and the fqn are both stable across it, so either would let a mid-transition sample land.
      const bool current_episode = std::any_of(io->targets.begin(), io->targets.end(), [&target](const ReadTarget & t) {
        return t.token == target.token;
      });
      if (current_episode) {
        if (ok) {
          io->observed[target.app_id] = std::move(observed);
          io->have.insert(target.app_id);
          io->failed_reads.erase(target.app_id);
        } else {
          // An ATTEMPT that completed and failed. This - not elapsed time - is the only thing
          // that ever justifies giving up on an app and letting a clear through without it.
          ++io->failed_reads[target.app_id];
          // And the app stops counting as MEASURED. A cached read is evidence about the instant it
          // was taken, not a standing licence: once a read fails, what is cached says nothing about
          // the node's current parameters. Leaving the app in `have` kept it in the evaluated set
          // for ever off a snapshot from before it went silent, so it never joined the unread set,
          // never held back a clear, and neither unread warning could ever fire about it - the
          // detector reported "nothing is drifting anywhere" on a node it had not read in hours.
          // `observed` is deliberately kept: it is only ever read together with `have`, and the
          // next successful read overwrites it.
          io->have.erase(target.app_id);
        }
      }
      // Hold the budget: wait out whatever is left of this visit's slots. The budget is spent in
      // service ROUND TRIPS at the watched node - not in node visits, and not in transport calls
      // either: one list_parameters() is two round trips there and one get_parameter() is three on
      // a name the node declares, and an `expect` visit makes one of the latter per pinned name.
      // Charging a slot per visit,
      // or even per call, would let the sweep put several times the promised load on a node while
      // the knob an operator lowered to protect it reports being honoured. Charge what was
      // actually spent. Waiting on the cv (not sleeping) keeps stop responsive, so shutdown is not
      // delayed by the pacing.
      const auto owed = min_gap * std::max(1, round_trips);
      const auto elapsed = std::chrono::steady_clock::now() - read_started;
      if (elapsed < owed) {
        io->cv.wait_for(lk, owed - elapsed, [&] {
          return io->stop;
        });
      }
      if (io->stop) {
        break;
      }
    }
  }
  // Signal that the reader has left the loop and will do no more rcl ops. A pre-shutdown
  // callback waits on this (see tick()) so the thread is idle BEFORE rclcpp invalidates the
  // context - a blocking rcl call caught mid-context-teardown would SIGABRT, not throw, so it
  // cannot merely be try/catch'd. The transport itself is torn down later, in the destructor.
  std::lock_guard<std::mutex> lk(io->mutex);
  io->reader_stopped = true;
  io->cv.notify_all();
}
}  // namespace

/// Self-capturing parameter-drift detector. See the package design doc.
/// Parameter reads run on a background thread (see SharedIo) so a slow/stuck node's blocking
/// read can never stall the tick loop; tick() only evaluates the cache.
class ParamDriftDetector : public Detector {
 public:
  ParamDriftDetector() : policy_(ParamDriftConfig{}) {
  }

  bool set_parameter_transport_factory_for_test(const ParameterTransportFactory & factory) override {
    transport_factory_ = factory;
    return true;
  }
  ~ParamDriftDetector() override {
    if (io_) {
      // Deregister the pre-shutdown callback first (it holds a weak_ptr to io_, so it is safe
      // either way, but leaving a stale callback on the context is untidy). No-op if it already
      // ran during a normal shutdown.
      if (context_) {
        context_->remove_pre_shutdown_callback(pre_shutdown_handle_);
      }
      {
        std::lock_guard<std::mutex> lk(io_->mutex);
        io_->stop = true;
      }
      io_->cv.notify_all();
      // Shut the transport down FIRST: it interrupts any in-flight blocking read (its spin has
      // a bounded timeout, see Ros2ParameterTransport::shutdown) so the reader returns promptly.
      // Then JOIN - never detach: a detached reader still doing a service call races rclcpp
      // teardown and SIGABRTs. The join here runs before the node/context are torn down (the
      // plugin destroys detectors before rclcpp shutdown), so the reader finishes cleanly.
      if (io_->transport) {
        io_->transport->shutdown();
      }
      if (reader_thread_.joinable()) {
        reader_thread_.join();
      }
    }
  }
  ParamDriftDetector(const ParamDriftDetector &) = delete;
  ParamDriftDetector & operator=(const ParamDriftDetector &) = delete;
  ParamDriftDetector(ParamDriftDetector &&) = delete;
  ParamDriftDetector & operator=(ParamDriftDetector &&) = delete;

  std::string id() const override {
    return "param_drift";
  }

  /// Distinct app ids this detector holds ANY per-app bookkeeping for, across both its own maps
  /// and the ones it shares with the reader thread.
  ///
  /// Every one of these is keyed by app id, and node names are not a bounded set: discovery has no
  /// hidden-node filter, so each `ros2` CLI invocation appears once under a name nothing will ever
  /// use again. A map that no prune path reaches therefore grows for the life of the process, and
  /// nothing about a leak is visible in a fault, a log line or a description - which is why it
  /// needs a number a test can assert on rather than a behaviour a test can observe.
  std::size_t tracked_count_for_test() const override {
    std::set<std::string> ids;
    const auto collect = [&ids](const auto & container) {
      for (const auto & entry : container) {
        ids.insert(entry.first);
      }
    };
    collect(misses_);
    collect(drifted_);
    collect(reported_);
    collect(pinned_counts_);
    collect(unread_latches_);
    collect(unarmed_ticks_);
    collect(bound_fqn_);
    if (io_) {
      std::lock_guard<std::mutex> lk(io_->mutex);
      collect(io_->observed);
      collect(io_->failed_reads);
      ids.insert(io_->have.begin(), io_->have.end());
      for (const auto & t : io_->targets) {
        ids.insert(t.app_id);
      }
    }
    return ids.size();
  }

  void configure(const nlohmann::json & config) override {
    policy_ = ParamDriftPolicy{ParamDriftConfig::from_json(config)};
    // prune_grace is injected into EVERY detector's config by the plugin and is exempt from the
    // unknown-key warning, so ignoring it here would accept the key with no warning and no
    // effect - the exact silent failure the README says cannot happen.
    own_warnings_.clear();
    prune_grace_ = kForgetGrace;
    if (config.contains("prune_grace")) {
      // Range-check on the WIDE value, like max_reads_per_tick. get<int>() truncates first, so
      // 4294967296 would arrive as 0 and drop findings on the first missed sweep - the flapping
      // prune_grace exists to prevent - and 2^63-1 would arrive as -1 and be dropped in silence.
      const auto & value = config["prune_grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxPruneGrace) {
        prune_grace_ = static_cast<int>(wide);
      } else {
        own_warnings_.push_back("'prune_grace' must be an integer in 0.." + std::to_string(kMaxPruneGrace) +
                                "; keeping " + std::to_string(kForgetGrace));
      }
    }
    // Plugin-injected: the reader thread paces itself at max_reads_per_tick service round trips
    // per tick period, so it needs to know how long a tick period is.
    if (config.contains("tick_interval_ms") && config["tick_interval_ms"].is_number_integer() &&
        config["tick_interval_ms"].get<int>() > 0) {
      tick_interval_ms_ = config["tick_interval_ms"].get<int>();
    }
    warnings_logged_ = false;
  }

  void tick(DetectorContext & ctx) override {
    if (!ctx.gateway_node || !ctx.snapshot) {
      return;
    }
    // Lazily build the shared IO + reader thread on the first tick (configure() has no node).
    //
    // `io_` is assigned LAST, once the transport and the thread both exist. Assigning it first
    // and filling it in afterwards means any throw from this block - std::thread under resource
    // pressure, an rclcpp throw while building the transport's hidden node, bad_alloc - leaves a
    // half-built `io_` behind. The plugin's per-detector guard logs that throw once, and every
    // later tick then sees a non-null `io_`, skips this block, and runs against a detector that
    // has no reader at all: `have` stays empty forever. Nothing distinguishes that from a healthy
    // detector at any call site. Leaving `io_` null instead means the next tick simply retries.
    if (!io_) {
      auto fresh = std::make_shared<SharedIo>();
      fresh->baseline_capture = policy_.config().baseline_capture;
      fresh->max_reads_per_tick = policy_.config().max_reads_per_tick;
      fresh->tick_interval_ms = tick_interval_ms_;
      for (const auto & [name, _] : policy_.config().expect) {
        fresh->expect_names.push_back(name);
      }
      fresh->transport = transport_factory_ ? transport_factory_(ctx.gateway_node)
                                            : std::make_unique<ros2_medkit_gateway::ros2::Ros2ParameterTransport>(
                                                  ctx.gateway_node, kParamServiceTimeoutSec, kNegativeCacheTtlSec);
      reader_thread_ = std::thread(reader_loop, fresh);
      io_ = std::move(fresh);
      // Register a pre-shutdown callback: it runs while the context is STILL valid (before
      // rclcpp invalidates it, even on a SIGINT), signals the reader to stop and waits (bounded)
      // for it to leave its loop. That guarantees the reader is not mid-blocking-rcl-call when
      // the context dies - which would SIGABRT (rcl aborts, it does not throw, so it cannot be
      // caught). The bounded wait keeps a genuinely hung read from hanging shutdown forever.
      context_ = ctx.gateway_node->get_node_base_interface()->get_context();
      std::weak_ptr<SharedIo> weak_io = io_;
      pre_shutdown_handle_ = context_->add_pre_shutdown_callback([weak_io] {
        auto io = weak_io.lock();
        if (!io) {
          return;
        }
        std::unique_lock<std::mutex> lk(io->mutex);
        io->stop = true;
        io->cv.notify_all();
        io->cv.wait_for(lk, std::chrono::seconds(2), [&] {
          return io->reader_stopped;
        });
      });
    }

    log_config_warnings_once(ctx);

    // Present apps (id + fqn) from the entity snapshot.
    std::vector<std::pair<std::string, std::string>> apps;  // (app.id, fqn)
    for (const auto & a : ctx.snapshot->apps) {
      const std::string fqn = a.effective_fqn();  // bound_fqn or derived; see app.hpp
      if (!fqn.empty()) {
        apps.emplace_back(a.id, fqn);
      }
    }
    std::sort(apps.begin(), apps.end());

    forget_rebound_apps(apps);
    prune_vanished(apps);

    // Armed targets (fast, NO I/O): only capture/evaluate a node that is armed + lifecycle-ok
    // The blocking read for each is done off-thread by reader_loop.
    const std::string self_fqn = ctx.gateway_node->get_fully_qualified_name();
    std::vector<std::pair<std::string, std::string>> armed;
    std::vector<std::string> denied;  // gate-denied apps still inside the frozen hold
    for (const auto & [app_id, fqn] : apps) {
      if (fqn == self_fqn) {
        // The GATEWAY's own node, matched on its fully qualified name. Reading it would have the
        // process interrogate itself through the hidden client node the transport spins, and its
        // own parameters would then start reporting drift.
        continue;
      }
      if (reliability_allows(ctx.gate, app_id)) {
        unarmed_ticks_.erase(app_id);
        armed.emplace_back(app_id, fqn);
      } else {
        if (++unarmed_ticks_[app_id] > kFrozenHoldTicks) {
          // Held long enough that this is not a pause: release the finding so a fixed parameter can
          // stop being reported. It re-raises from a fresh read if the app ever arms again. Past
          // this point the app also stops holding back a clear: the hold is what bounds an app the
          // gate will never open for, and without a bound one permanently inactive node would keep
          // GRAPH_PARAM_DRIFT from ever healing, with no read it could ever make to get out of it.
          drifted_.erase(app_id);
          pinned_counts_.erase(app_id);
        } else {
          denied.push_back(app_id);
        }
        // Do NOT erase this app's drift. The aggregate raise is gated on the aggregate's own
        // source, never on app_id, so an unarmed app's finding is not "suppressed anyway" -
        // dropping it here is the only thing that would suppress it, and dropping it feeds the
        // CLEAR path, which carries no gate at all. A routine lifecycle pause would then heal a
        // drift that is still present. The entry stays frozen: not re-evaluated while unarmed,
        // not removed either. Removal is the prune path's job, when the app actually goes away.
        continue;
      }
    }

    // Hand the reader thread the current armed set, then evaluate drift from the CACHED reads.
    std::map<std::string, std::map<std::string, nlohmann::json>> cached;
    std::vector<UnreadApp> unread;  // armed apps with no usable read yet - see emit_aggregated
    /// Gate-denied apps with no usable read either. The gate decides whether we may RAISE about an
    /// entity; it says nothing about whether we have measured it, and an app that is denied is one
    /// the reader is not even allowed to target. "The round-robin has not reached it" and "the gate
    /// has not armed it" are the same state - nothing is known - and the clear may not be emitted
    /// out of either. Without this, every app is denied for the first `warmup_cycles` ticks after a
    /// gateway or plugin start, the armed set is empty, and the detector clears on every one of
    /// those ticks before it has issued a single round trip.
    std::vector<std::string> denied_unmeasured;
    {
      std::lock_guard<std::mutex> lk(io_->mutex);
      // Carry an app's arming token forward only while it stays armed WITHOUT interruption. An app
      // missing from the previous target list is starting a new episode and gets a fresh token, so
      // a read still in flight from its last one cannot publish into it. The fqn is part of the
      // key for the same reason: a re-bind to a different node under the same app id is a new
      // episode too, even though the id never changed.
      std::map<std::pair<std::string, std::string>, std::uint64_t> previous;
      for (const auto & t : io_->targets) {
        previous.emplace(std::make_pair(t.app_id, t.fqn), t.token);
      }
      std::vector<ReadTarget> next;
      next.reserve(armed.size());
      for (const auto & [app_id, fqn] : armed) {
        const auto pit = previous.find(std::make_pair(app_id, fqn));
        next.push_back(ReadTarget{app_id, fqn, pit != previous.end() ? pit->second : io_->next_token++});
      }
      io_->targets = std::move(next);
      for (const auto & [app_id, fqn] : armed) {
        (void)fqn;
        auto it = io_->observed.find(app_id);
        if (it != io_->observed.end() && io_->have.count(app_id) > 0) {
          cached[app_id] = it->second;  // copy out under the lock; evaluate below without it held
        } else {
          const auto fit = io_->failed_reads.find(app_id);
          unread.push_back(UnreadApp{app_id, fit != io_->failed_reads.end() ? fit->second : 0});
        }
      }
      for (const auto & app_id : denied) {
        if (io_->have.count(app_id) == 0) {
          denied_unmeasured.push_back(app_id);
        }
      }
    }
    io_->cv.notify_one();

    // Advance the per-app unread bookkeeping BEFORE the blocking set is built: the give-up is a
    // conjunction of failed attempts and elapsed ticks, so the tick count has to already exist when
    // the set that depends on it is computed.
    advance_unread_latches(unread);

    // Apps that hold back a clear: no usable read, and no grounds to stop waiting for one. An app
    // the sweep has not attempted (failed_attempts == 0) is in here for as long as that stays true,
    // however long the rotation takes - see kUnmeasurableAfterFailures.
    std::vector<std::string> blocking;
    for (const auto & app : unread) {
      if (!given_up_on(app)) {
        blocking.push_back(app.app_id);
      }
    }
    blocking.insert(blocking.end(), denied_unmeasured.begin(), denied_unmeasured.end());

    for (const auto & [app_id, observed] : cached) {
      std::size_t pinned = 0;
      const auto d = policy_.evaluate(app_id, observed, /*capture_allowed=*/true, &pinned);
      if (!d.empty()) {
        drifted_[app_id] = d;
        pinned_counts_[app_id] = pinned;
      } else {
        drifted_.erase(app_id);
        pinned_counts_.erase(app_id);
      }
    }

    report_unmatched_pins_once(ctx, cached, blocking.empty());
    emit_aggregated(ctx, unread, blocking);
  }

 private:
  /// Name `expect` pins that no app declares, once each. A pin whose parameter name is misspelled
  /// comes back NOT_FOUND from every read, and that is deliberately carved out as "not a drift" -
  /// a node is not at fault for a parameter it never had. The cost is that such a pin is inert
  /// forever, and at runtime inert is indistinguishable from a pin that is checked and passes
  /// every time. That is the same silent failure this detector exists to rule out, so it has to
  /// say something.
  ///
  /// Said only once the sweep has COVERED the graph, for the same reason a clear is: only one node
  /// is visited per rotation slot, so a perfectly good pin is legitimately unseen for as long as
  /// the sweep has not reached the node that declares it. Telling an operator to go and fix a name
  /// that is already correct is the same defect as clearing a fault we never measured, and a
  /// threshold in ticks fails here in exactly the same way - it expires on the large graphs where
  /// the rotation is longest.
  ///
  /// `swept` is the clear's own condition: every armed app has either been read or been given up
  /// on after repeated failures. `cached` non-empty is still required on top of it, because a graph
  /// where every app was given up on is swept in name only and says nothing about any pin.
  void report_unmatched_pins_once(const DetectorContext & ctx,
                                  const std::map<std::string, std::map<std::string, nlohmann::json>> & cached,
                                  bool swept) {
    if (cached.empty() || !swept) {
      return;
    }
    for (const auto & [pin, expected] : policy_.config().expect) {
      (void)expected;
      const bool declared = std::any_of(cached.begin(), cached.end(), [&pin](const auto & entry) {
        return entry.second.count(pin) != 0;
      });
      if (declared) {
        unmatched_pins_reported_.erase(pin);  // a later disappearance is worth saying again
        continue;
      }
      // Node check FIRST: latching without logging would silence the pin for the whole run.
      if (!ctx.gateway_node || !unmatched_pins_reported_.insert(pin).second) {
        continue;
      }
      RCLCPP_WARN(ctx.gateway_node->get_logger(),
                  "param_drift: no app declares '%s', the parameter pinned under 'expect', so that "
                  "pin can never report a drift; check the name against the node's own parameters",
                  pin.c_str());
    }
  }

  void log_config_warnings_once(const DetectorContext & ctx) {
    if (warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    warnings_logged_ = true;
    for (const auto & w : policy_.config().warnings) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "param_drift config: %s", w.c_str());
    }
    // Keys the DETECTOR parses rather than the policy (prune_grace) surface here, so an operator
    // finds them in the same place and a rejected value is never silent.
    for (const auto & w : own_warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "param_drift config: %s", w.c_str());
    }
  }

  /// Per-app unread bookkeeping for THIS tick: drop the entries of apps that have left the unread
  /// set, and advance the tick counter of the ones still in it.
  ///
  /// The drop is load-bearing and is why this must run on every tick, including ticks where nothing
  /// is unread. The latches are one-shot per RUN of failures, not once per process: an app that
  /// goes quiet, recovers and goes quiet again has failed twice, and the second time is worth
  /// saying to the operator who fixed it the first time. A latch left behind silences that app for
  /// the life of the gateway, and silence is also what a working latch looks like.
  void advance_unread_latches(const std::vector<UnreadApp> & unread) {
    std::set<std::string> still_unread;
    for (const auto & app : unread) {
      still_unread.insert(app.app_id);
    }
    for (auto it = unread_latches_.begin(); it != unread_latches_.end();) {
      it = still_unread.count(it->first) == 0 ? unread_latches_.erase(it) : std::next(it);
    }
    for (const auto & app : unread) {
      ++unread_latches_[app.app_id].ticks;
    }
  }

  /// Consecutive ticks `app_id` has been armed with no usable read. Zero when it is not in the
  /// unread set at all.
  int unread_ticks(const std::string & app_id) const {
    const auto it = unread_latches_.find(app_id);
    return it != unread_latches_.end() ? it->second.ticks : 0;
  }

  /// Whether this app has stopped holding back a clear. BOTH counters have to be past the bound:
  /// the attempts, because only evidence justifies giving up, and the ticks, because an attempt is
  /// not a duration and the attempt horizon alone is scaled by the load knob (kUnreadReportAfterTicks).
  /// Strictly past both, matching the give-up warning: at exactly the bound the app still blocks.
  bool given_up_on(const UnreadApp & app) const {
    return app.failed_attempts > kUnmeasurableAfterFailures && unread_ticks(app.app_id) > kUnmeasurableAfterTicks;
  }

  /// Name the apps that are holding back a clear, once each, so the reason a GRAPH_PARAM_DRIFT
  /// is not healing is readable instead of having to be inferred.
  ///
  /// Only apps we ATTEMPTED and failed to read are ever named. An app the round-robin has not
  /// reached yet is silent for a reason that is entirely ours, and saying "no parameter read has
  /// succeeded for it" would be a bringup warning about a healthy node on every graph whose
  /// rotation outlasts the threshold. Reported once per app per run of failures; the latches are
  /// created and dropped by advance_unread_latches, which tick() runs first.
  void report_unread_once(const DetectorContext & ctx, const std::vector<UnreadApp> & unread) {
    for (const auto & app : unread) {
      if (app.failed_attempts <= 0) {
        continue;  // never attempted: there is nothing to report and nothing to give up on
      }
      const auto lit = unread_latches_.find(app.app_id);
      if (lit == unread_latches_.end()) {
        continue;  // advance_unread_latches owns this map; it always has an entry for an unread app
      }
      auto & latch = lit->second;
      if (app.failed_attempts >= kUnreadReportAfterFailures && latch.ticks >= kUnreadReportAfterTicks &&
          !latch.reported && ctx.gateway_node) {
        latch.reported = true;
        RCLCPP_WARN(ctx.gateway_node->get_logger(),
                    "param_drift: %d consecutive parameter reads of '%s' have failed; its parameters "
                    "are unverified and GRAPH_PARAM_DRIFT will not clear while that is true",
                    app.failed_attempts, app.app_id.c_str());
      }
      // Strictly past the bound, matching the blocking set built in tick(): at exactly the bound
      // the app still blocks, so announcing that we gave up on it would be premature.
      if (given_up_on(app) && !latch.gave_up && ctx.gateway_node) {
        latch.gave_up = true;
        RCLCPP_WARN(ctx.gateway_node->get_logger(),
                    "param_drift: giving up on '%s' after %d failed parameter reads; it no longer "
                    "holds back GRAPH_PARAM_DRIFT from clearing, and its parameters stay unverified",
                    app.app_id.c_str(), kUnmeasurableAfterFailures);
      }
    }
  }

  /// Say, once per episode, that a clear is being withheld and why.
  ///
  /// Withholding is correct - health that was never measured is not health - but from the outside
  /// it is indistinguishable from a detector that is working and finding nothing: no raise, no
  /// clear, and a GRAPH_PARAM_DRIFT already in the store simply never heals. The apps holding it
  /// back are frequently ones report_unread_once may not name, because nothing has been ATTEMPTED
  /// against them (they are waiting their turn, or the gate has not armed them), so without this
  /// the log is silent about the one thing an operator needs in order to act.
  void report_withheld_clear(const DetectorContext & ctx, const std::vector<std::string> & blocking) {
    ++withheld_ticks_;
    if (withheld_ticks_ <= kWithheldClearReportTicks || withheld_reported_ || !ctx.gateway_node) {
      return;
    }
    withheld_reported_ = true;
    RCLCPP_WARN(ctx.gateway_node->get_logger(),
                "param_drift: nothing has been found drifting for %d consecutive ticks, but "
                "GRAPH_PARAM_DRIFT is being withheld from clearing because %zu app(s) still have no "
                "usable parameter read, starting with '%s'. Until every armed app has been read or "
                "given up on, the graph has not been measured and no clear is emitted; raise "
                "'max_reads_per_tick' or shorten 'tick_interval_ms' if the sweep is simply too slow "
                "for the size of this graph",
                withheld_ticks_, blocking.size(), blocking.front().c_str());
  }

  /// Drop everything an app's PREVIOUS binding left behind when its fqn changes underneath it.
  ///
  /// The reader already treats (app_id, fqn) as the identity of an arming episode, so a read in
  /// flight from the old binding cannot publish into the new one - but nothing reset the state the
  /// old binding had already published. The new node's first reading was then compared against a
  /// baseline a DIFFERENT node happened to hold, which is a drift no later read can heal because
  /// the reference itself belongs somewhere else, and it inherited the old node's failure count,
  /// so a perfectly readable node could be given up on after a handful of its own attempts.
  void forget_rebound_apps(const std::vector<std::pair<std::string, std::string>> & apps) {
    for (const auto & [app_id, fqn] : apps) {
      const auto it = bound_fqn_.find(app_id);
      if (it == bound_fqn_.end()) {
        bound_fqn_.emplace(app_id, fqn);
        continue;
      }
      if (it->second == fqn) {
        continue;
      }
      it->second = fqn;
      policy_.forget(app_id);
      drifted_.erase(app_id);
      pinned_counts_.erase(app_id);
      unread_latches_.erase(app_id);
      if (io_) {
        std::lock_guard<std::mutex> lk(io_->mutex);
        io_->observed.erase(app_id);
        io_->have.erase(app_id);
        io_->failed_reads.erase(app_id);
        io_->targets.erase(std::remove_if(io_->targets.begin(), io_->targets.end(),
                                          [&app_id](const ReadTarget & t) {
                                            return t.app_id == app_id;
                                          }),
                           io_->targets.end());
      }
    }
  }

  /// Two different horizons, on purpose.
  ///
  /// The BASELINE is dropped once the app has been absent from kBaselineForgetGrace + 1
  /// consecutive sweeps - long enough that the absence is no longer explained by the discovery
  /// churn WarmupTracker documents. Dropping it on the FIRST missed sweep would re-capture the
  /// node's current, possibly already drifted values on an ordinary dropped poll: the confirmed
  /// fault would heal while the parameter is still wrong and could never fire again, because the
  /// new baseline IS the wrong value. The cost is the other direction - a respawn faster than the
  /// window keeps the pre-restart baseline and reports the operator's own change as drift until
  /// they revert it. See kBaselineForgetGrace, and the README's recovery-path paragraph.
  ///
  /// The FINDING is dropped only after prune_grace consecutive absences. Removing it on the first
  /// missed sweep would clear the aggregated fault on any one-tick discovery gap, which is
  /// flapping, not healing.
  ///
  /// The two horizons are INDEPENDENTLY configurable and prune_grace can be the shorter one - `0`
  /// and `1` are both inside the documented range - so the baseline forget cannot simply wait for
  /// the churn window and hope the counter survives that long. Dropping the counter is what makes
  /// any later cleanup unreachable: with prune_grace below the churn window the forget would never
  /// run for any app, ever, leaving both maps growing without bound under node-name churn and every
  /// returning node compared against a stale baseline. So the forget happens at the churn window OR
  /// on the tick the counter is dropped, whichever comes first, and a latch keeps it one-shot.
  void prune_vanished(const std::vector<std::pair<std::string, std::string>> & apps) {
    std::set<std::string> present;
    for (const auto & [id, fqn] : apps) {
      present.insert(id);
    }
    for (auto it = misses_.begin(); it != misses_.end();) {
      if (present.count(it->first) > 0) {
        it->second.misses = 0;
        it->second.baseline_forgotten = false;
        ++it;
        continue;
      }
      ++it->second.misses;
      const bool dropping = it->second.misses > prune_grace_;
      if (!it->second.baseline_forgotten && (it->second.misses >= kBaselineForgetGrace + 1 || dropping)) {
        // Past the churn-absorbing window (or out of ticks in which to do it at all), so treat this
        // as a restart rather than a dropped poll: forget the baseline and the cached read so the
        // node's return starts from a fresh capture.
        it->second.baseline_forgotten = true;
        policy_.forget(it->first);
        if (io_) {
          std::lock_guard<std::mutex> lk(io_->mutex);
          io_->observed.erase(it->first);
          io_->have.erase(it->first);
          io_->failed_reads.erase(it->first);
          // And from targets, in the SAME critical section. tick() rewrites targets further down,
          // so leaving the app there would let a read completing inside this tick republish the
          // values just erased - they would become the baseline on return, which is the false,
          // unhealable drift the reader's episode fence exists to prevent.
          io_->targets.erase(std::remove_if(io_->targets.begin(), io_->targets.end(),
                                            [&it](const ReadTarget & t) {
                                              return t.app_id == it->first;
                                            }),
                             io_->targets.end());
        }
      }
      if (dropping) {
        drifted_.erase(it->first);
        pinned_counts_.erase(it->first);
        // The frozen-hold counter goes with the finding it holds. Nothing else erases it off the
        // arm path, and discovery has no hidden-node filter, so every short-lived `ros2` CLI node
        // would otherwise leave one entry behind for the life of the process.
        unarmed_ticks_.erase(it->first);
        // Same for the remembered binding: it is written for every present app and nothing else
        // reaches it, so it leaks under node-name churn exactly as the counter above did.
        bound_fqn_.erase(it->first);
        it = misses_.erase(it);
      } else {
        ++it;
      }
    }
    for (const auto & id : present) {
      misses_.emplace(id, AbsenceCounter{});
    }
  }

  /// Level-triggered aggregation: one graph-level fault carrying every app's drift, through the
  /// same helper qos_mismatch and orphan use. Going through AggregatedFault rather than building
  /// the text here is what keeps three properties this detector previously got wrong: the fault
  /// is raised under graph_source_id() (a constant, owned entity - a host Component id would
  /// match no entity scope at all, so the fault would be reachable from no endpoint), the
  /// description stops being built once it exceeds the cap instead of concatenating the whole
  /// graph first, and the cap itself is the helper's constant rather than a second copy of 480.
  ///
  /// `order` carries the priority: entries an operator explicitly pinned with `expect` come
  /// first, then self-captured drift, and within each group the newly detected apps precede the
  /// ones already reported. Without that, a capped description can hide exactly the two things
  /// worth reading - the pin the operator declared, and whatever just changed.
  void emit_aggregated(DetectorContext & ctx, const std::vector<UnreadApp> & unread,
                       const std::vector<std::string> & blocking) {
    // A clear asserts that nothing is drifting anywhere. That is only true if we actually LOOKED
    // everywhere. drifted_ is built from cached reads alone, so an app whose parameter service
    // never answers contributes nothing to it - and without this guard an empty drifted_ would
    // be reported as health the detector never measured, every tick, forever. AggregatedFault
    // decides raise-versus-clear from emptiness alone, so the guard has to live here: when there
    // is nothing to report AND something we could not read, emit nothing at all.
    //
    // A raise is never withheld: a drift found on the apps that did answer is real either way.
    // Every tick, including ticks with nothing unread: that call is what resets the latches.
    report_unread_once(ctx, unread);
    if (drifted_.empty() && !blocking.empty()) {
      report_withheld_clear(ctx, blocking);
      return;
    }
    withheld_ticks_ = 0;
    withheld_reported_ = false;
    std::map<std::string, std::string> affected;
    for (const auto & [app, items] : drifted_) {
      std::string detail;
      for (const auto & item : items) {
        detail += (detail.empty() ? "" : "; ") + item;
      }
      if (detail.size() > kMaxAppDetailChars) {
        // Keep both ends: the head names the app and its first drifted parameter, the tail keeps the
        // count of what followed readable rather than ending mid-token.
        const std::size_t keep = kMaxAppDetailChars - 3;
        const std::size_t head = (keep + 1) / 2;
        detail = detail.substr(0, head) + "..." + detail.substr(detail.size() - (keep - head));
      }
      affected.emplace(app, detail);
    }
    aggregated_.emit_ordered(ctx, affected, emit_order());
    reported_ = drifted_;
  }

  /// App ids in the order their details should appear in the description. Keys of `affected`
  /// missing here are appended by the helper, so a gap degrades the ordering rather than
  /// dropping an app.
  std::vector<std::string> emit_order() const {
    std::vector<std::string> pinned_new, pinned_old, baseline_new, baseline_old;
    for (const auto & [app, items] : drifted_) {
      (void)items;
      const bool is_new = reported_.count(app) == 0;
      const auto pit = pinned_counts_.find(app);
      const bool pinned = pit != pinned_counts_.end() && pit->second > 0;
      auto & bucket = pinned ? (is_new ? pinned_new : pinned_old) : (is_new ? baseline_new : baseline_old);
      bucket.push_back(app);
    }
    std::vector<std::string> order;
    order.reserve(drifted_.size());
    for (const auto * bucket : {&pinned_new, &pinned_old, &baseline_new, &baseline_old}) {
      order.insert(order.end(), bucket->begin(), bucket->end());
    }
    return order;
  }

  ParamDriftPolicy policy_;
  int tick_interval_ms_ = kDefaultTickIntervalMs;  // plugin-injected; paces the reader thread
  std::shared_ptr<SharedIo> io_;                   // shared with the background reader thread
  std::thread reader_thread_;
  rclcpp::Context::SharedPtr context_;  // for the pre-shutdown callback
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;
  /// Consecutive missing sweeps for one app, plus the one-shot latch that keeps the baseline
  /// forget from repeating once the churn window has passed.
  struct AbsenceCounter {
    int misses = 0;
    bool baseline_forgotten = false;
  };

  std::map<std::string, AbsenceCounter> misses_;              // app.id -> absence bookkeeping
  std::map<std::string, std::vector<std::string>> drifted_;   // app.id -> its drifted descriptors
  std::map<std::string, std::vector<std::string>> reported_;  // drifted_ as of the previous emit
  std::map<std::string, std::size_t> pinned_counts_;          // app.id -> leading `expect` descriptors
  std::map<std::string, UnreadLatch> unread_latches_;         // app.id -> which unread warnings were logged
  std::set<std::string> unmatched_pins_reported_;             // expect pins already reported as declared by no app
  std::map<std::string, int> unarmed_ticks_;                  // app.id -> consecutive ticks the gate denied it
  std::map<std::string, std::string> bound_fqn_;              // app.id -> the fqn it was last seen bound to
  int withheld_ticks_ = 0;                                    // consecutive ticks a clear has been held back
  bool withheld_reported_ = false;                            // the withheld-clear warning fired for this episode
  int prune_grace_ = kForgetGrace;                            // plugin-injected; absences before a finding is dropped
  std::vector<std::string> own_warnings_;                     // config problems the detector itself found
  ParameterTransportFactory transport_factory_;               // empty = build the real ROS 2 transport
  AggregatedFault aggregated_{graph_fault_codes::kParamDrift, kParamDriftSeverity};
  bool warnings_logged_ = false;
};

REGISTER_DETECTOR(ParamDriftDetector, "param_drift")

}  // namespace ros2_medkit_graph_watchdog
