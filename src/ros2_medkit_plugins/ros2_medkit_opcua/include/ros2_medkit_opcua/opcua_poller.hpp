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

#pragma once

#include "ros2_medkit_opcua/alarm_state_machine.hpp"
#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

/// Snapshot of all polled values at a point in time
struct PollSnapshot {
  std::chrono::system_clock::time_point timestamp;
  std::unordered_map<std::string, OpcuaValue> values;  // node_id_str -> value
  std::unordered_map<std::string, bool> alarms;        // fault_code -> active
  bool connected{false};
  uint64_t poll_count{0};
  uint64_t error_count{0};
};

/// Callback when a fault-detection signal transitions (for fault reporting).
/// Fires once per raise/clear edge with the hosting entity and the shared
/// ``FaultSignal`` (fault code, severity, message, active) produced by the
/// shared evaluator. Threshold, status-bit and enum modes all use this path.
using AlarmChangeCallback =
    std::function<void(const std::string & entity_id, const ros2_medkit::fault_detection::FaultSignal & signal)>;

/// Callback when a native OPC-UA AlarmCondition lifecycle transitions to a
/// new SOVD status (issue #386). Fires at most once per logical transition;
/// suppressed events do not invoke the callback.
struct AlarmEventDelivery {
  std::string fault_code;
  std::string entity_id;
  SovdAlarmStatus next_status;
  AlarmAction action;
  uint16_t severity{0};           // raw OPC-UA Severity 1-1000
  std::string severity_override;  // resolved severity bucket override (issue #389), empty = derive from severity
  std::string message;            // event Message field (or resolved override)
  std::string condition_id;       // string form of OPC-UA ConditionId
};
using EventAlarmCallback = std::function<void(const AlarmEventDelivery & delivery)>;

/// Looked-up runtime state for a unique OPC-UA Condition instance. The
/// poller keeps one entry per distinct ConditionId observed; the entry
/// outlives ack/confirm round-trips so the PR3 ``acknowledge_fault`` SOVD
/// operation can resolve a fault_code to the live ConditionId NodeId and
/// the latest ``EventId`` ByteString (required for spec-compliant Ack).
struct ConditionRuntime {
  opcua::NodeId condition_id;
  opcua::ByteString latest_event_id;
  std::string entity_id;
  std::string fault_code;
  /// String form of the alarm source NodeId that owns this condition. Used by
  /// the read-based reconcile to skip clearing conditions whose source scan
  /// failed (issue #479).
  std::string source_id;
  SovdAlarmStatus last_status{SovdAlarmStatus::Suppressed};
};

/// Callback after each poll cycle (for publishing values to ROS 2 topics)
using PollCallback = std::function<void(const PollSnapshot & snapshot)>;

/// Strategy for replaying already-active OPC-UA conditions on (re)subscribe
/// (issue #389). Some servers (e.g. Siemens S7-1500) reject the standard
/// ConditionRefresh method with BadNodeIdUnknown, so the active alarm set is
/// otherwise lost across a reconnect / gateway restart.
///   - Method: call ConditionRefresh only (Part 9 §5.5.7 standard path).
///   - Read:   skip ConditionRefresh; browse the alarm sources and read each
///             condition's current state, then reconcile the fault set.
///   - Auto:   try ConditionRefresh first; on rejection fall back to Read.
///   - Off:    no replay (only live transitions surface).
enum class ConditionReplayStrategy { Method, Read, Auto, Off };

/// Configuration for the poller
struct PollerConfig {
  bool prefer_subscriptions{false};  // poll mode by default (subscriptions need event loop)
  double subscription_interval_ms{500.0};
  std::chrono::milliseconds poll_interval{1000};
  std::chrono::milliseconds reconnect_interval{5000};
  /// Active-condition replay strategy on (re)subscribe (issue #389).
  /// Default Auto: ConditionRefresh with a read-based fallback so hardened
  /// servers that reject the method still recover their active alarms.
  ConditionReplayStrategy condition_replay_strategy{ConditionReplayStrategy::Auto};
  /// When true (default) a native AlarmCondition auto-clears only after BOTH
  /// Acknowledge and Confirm (OPC-UA Part 9 §5.7). Set false for servers that
  /// do not implement the optional Confirm transition (e.g. Siemens S7-1500,
  /// which supports Acknowledge / ConditionRefresh / AddComment but not
  /// Confirm) so the alarm clears on Acknowledge alone instead of latching
  /// forever. Default true preserves the spec-strict behaviour. Issue #478;
  /// needs real-S7-1500 validation.
  bool require_confirm_for_clear{true};
  /// Issue #496: raise a single component-scoped ``PLC_COMMS_LOST`` fault when
  /// the OPC-UA connection stays down for ``comms_lost_debounce`` continuously
  /// (so a brief blip during a normal reconnect does not flap a fault), and
  /// clear it on the next successful reconnect. ``enabled`` default true;
  /// ``severity`` is the SOVD severity bucket for the fault.
  bool comms_lost_fault_enabled{true};
  std::chrono::milliseconds comms_lost_debounce{5000};
  std::string comms_lost_severity{"ERROR"};
  /// Optional warn-level log sink for operator-visible failures inside the
  /// poll thread. Set by the plugin owning the poller to its log_warn
  /// helper so events like ``ConditionRefresh failed`` reach the ROS 2 log
  /// instead of stderr only. Empty by default - the poller falls back to
  /// stderr in that case.
  std::function<void(const std::string &)> log_warn;
  /// Optional readiness probe for the fault sink (ReportFault service). The poll
  /// thread only latches the comms-lost fault once this returns true, so a
  /// fire-and-forget report is never dropped-and-forgotten while the sink is
  /// unmatched - it retries on the next poll instead. Empty => assume ready.
  std::function<bool()> report_sink_ready;
  /// Optional endpoint rediscovery, bound by a plugin running config-less
  /// network discovery with no endpoint configured. Called from the reconnect
  /// arm - so only while no session is up - and expected to rate-limit itself.
  /// Returns a new endpoint URL to reconnect against, nullopt to keep the
  /// current one. Without it the reconnect loop retries the same endpoint
  /// forever, which strands a gateway that scanned before its PLC had booted.
  std::function<std::optional<std::string>()> rediscover_endpoint;
};

/// Fault code of the component-scoped OPC-UA connection fault the poller raises
/// on a sustained outage and clears on the next successful connect (issue #496).
/// Named here so the plugin can clear the same code from its own connect path
/// without keeping a second copy of the literal.
inline constexpr const char * kCommsLostFaultCode = "PLC_COMMS_LOST";

/// Manages OPC-UA data collection via subscriptions (preferred) or polling
class OpcuaPoller {
 public:
  OpcuaPoller(OpcuaClient & client, const NodeMap & node_map);
  ~OpcuaPoller();

  OpcuaPoller(const OpcuaPoller &) = delete;
  OpcuaPoller & operator=(const OpcuaPoller &) = delete;
  OpcuaPoller(OpcuaPoller &&) = delete;
  OpcuaPoller & operator=(OpcuaPoller &&) = delete;

  /// Start the poller thread
  void start(const PollerConfig & config);

  /// Stop the poller thread
  void stop();

  /// Get current snapshot (thread-safe copy)
  PollSnapshot snapshot() const;

  /// Get value for a specific node (thread-safe)
  std::optional<OpcuaValue> get_value(const std::string & node_id_str) const;

  /// Set callback for alarm state changes
  void set_alarm_callback(AlarmChangeCallback callback);

  /// Set callback for native AlarmCondition event lifecycle transitions
  /// (issue #386). Must be called before ``start()``.
  void set_event_alarm_callback(EventAlarmCallback callback);

  /// Set callback fired after each poll cycle (for value bridging)
  void set_poll_callback(PollCallback callback);

  /// Check if using subscriptions (vs polling)
  bool using_subscriptions() const {
    return using_subscriptions_.load();
  }

  /// Look up a live condition by ``(entity_id, fault_code)``. Used by the
  /// SOVD ``acknowledge_fault`` / ``confirm_fault`` operation handlers to
  /// resolve which OPC-UA ConditionId should receive the Method call. The
  /// returned snapshot is a copy, so the caller can release any locks
  /// before performing the OPC-UA round-trip.
  ///
  /// Returns ``std::nullopt`` if no condition with that fault_code is
  /// currently active for the entity.
  std::optional<ConditionRuntime> lookup_condition(const std::string & entity_id, const std::string & fault_code) const;

  /// Parse a replay-strategy name ("method", "read", "auto", "off").
  /// Case-insensitive; unknown input falls back to Auto.
  static ConditionReplayStrategy parse_replay_strategy(const std::string & name);

  /// How a read-scan condition snapshot participates in the read-based replay.
  ///   - Feed:     interesting + reliable -> mark seen and drive the state
  ///               machine (mirrors a ConditionRefresh replay event).
  ///   - KeepOnly: present but unreliable (transient read/browse failure) ->
  ///               mark seen so reconcile does not clear it, but do NOT feed an
  ///               unreliable state into the state machine.
  ///   - Skip:     not interesting (Disabled, or Retain=false) -> neither
  ///               marked seen nor fed, so a stale tracked fault can reconcile.
  enum class ReadReplayDisposition { Feed, KeepOnly, Skip };

  /// Classify one read-scan condition snapshot for reconnect replay (issue
  /// #478). Mirrors ConditionRefresh's interesting-state filter: a Disabled
  /// condition (EnabledState/Id=false) is never active, and only conditions
  /// with Retain==true are replayed; a transient read failure is kept but not
  /// trusted. Pure / static so the filter is unit-testable without a server.
  static ReadReplayDisposition classify_read_snapshot(const OpcuaClient::ConditionStateSnapshot & snap);

  /// Decide whether a tracked condition should be cleared after a read-based
  /// replay scan (issue #479 / #478). A condition is cleared only when it was
  /// active, was NOT observed this scan (``seen``), its owning source scan
  /// succeeded (not in ``failed_sources``), AND its source is positively known
  /// to model Condition instances as address-space nodes (in
  /// ``modeled_sources``). The last gate is the #478 safety guarantee: an
  /// EventNotifier-only server (e.g. S7-1500) exposes no per-condition nodes,
  /// so its empty read scan must never wipe the tracked fault set.
  /// Pure and static so the false-clear guard is unit-testable without a server.
  static bool should_clear_condition(SovdAlarmStatus last_status, const std::string & condition_id,
                                     const std::string & source_id, const std::set<std::string> & seen,
                                     const std::set<std::string> & failed_sources,
                                     const std::set<std::string> & modeled_sources);

  /// Decide whether a tracked condition should be cleared after a completed
  /// ConditionRefresh burst (issue #480). Cleared only when it was active
  /// (Confirmed/Healed) and its ConditionId was NOT among the ones the server
  /// replayed during the burst (``seen``). A delivered RefreshEnd is an
  /// authoritative, subscription-wide replay, so - unlike the read fallback -
  /// no per-source modeling gate is needed. Pure and static so it is
  /// unit-testable without a server.
  static bool should_clear_after_refresh(SovdAlarmStatus last_status, const std::string & condition_id,
                                         const std::set<std::string> & seen);

  /// Decide whether the debounced comms-lost fault should be raised now
  /// (issue #496). Raised only when the fault is enabled, is not already
  /// raised, and the connection has been continuously down since
  /// ``down_since`` for at least ``debounce``. Pure and static so the
  /// debounce gate is unit-testable without a live connection.
  static bool comms_lost_should_raise(bool enabled, bool already_raised,
                                      std::chrono::steady_clock::time_point down_since,
                                      std::chrono::steady_clock::time_point now, std::chrono::milliseconds debounce);

  /// Endpoint the next reconnect attempt should target. Asks
  /// ``rediscover_endpoint`` (when bound) for a freshly discovered server and
  /// returns it only when it names a DIFFERENT endpoint than ``current``.
  /// nullopt means "keep the current one", which is also the answer when no
  /// callback is bound, when the callback declines, or when it hands back an
  /// empty string. Pure and static (the callback is injected) so the adoption
  /// rule is unit-testable without a network.
  static std::optional<std::string>
  adopt_rediscovered_endpoint(const std::string & current,
                              const std::function<std::optional<std::string>()> & rediscover);

  /// Zero-config native A&C (``auto_alarms``): the alarm sources that should
  /// actually be subscribed / replayed, i.e. every explicit ``event_alarms``
  /// entry plus (when ``auto_cfg.enabled`` and no explicit entry already
  /// targets the same source) one synthetic, mapping-less
  /// ``AlarmEventConfig`` for ``auto_cfg.source_node_id``. Because the
  /// synthetic entry carries no ``fault_code``/``mappings``,
  /// ``NodeMap::resolve_alarm`` always reports it unmatched, which is
  /// exactly what routes every event on that source through the
  /// auto-derivation branch in ``on_event`` / ``read_fallback_replay``
  /// (explicit mappings on a shared source are tried first and still win -
  /// precedence). Pure and static so subscription-source selection and the
  /// precedence rule are unit-testable without a server.
  static std::vector<AlarmEventConfig> effective_alarm_sources(const std::vector<AlarmEventConfig> & explicit_sources,
                                                               const AutoAlarmsConfig & auto_cfg);

  /// True when two OPC-UA NodeId strings denote the same node once parsed to
  /// canonical form, so equivalent spellings are recognized as one node:
  /// ``i=2253`` (the default numeric Server object) matches an explicit
  /// ``ns=0;i=2253``, and a default vs explicit namespace agree. Used to
  /// dedupe alarm subscriptions and to route the notifier hierarchy so a given
  /// physical node is not subscribed / auto-derived twice. Falls back to raw
  /// string equality when a spelling is unparseable (distinct raw strings stay
  /// distinct). Pure and static so it is unit-testable without a server.
  static bool node_ids_equivalent(const std::string & a, const std::string & b);

  /// True only for a real OPC-UA Condition event. Per Part 9 §5.5.2.13 the
  /// ConditionId SAO resolves to a non-null NodeId only for AlarmConditionType
  /// (and subtype) instances; a plain BaseEvent / SystemEvent notification -
  /// e.g. a Siemens Server-object "CPU not in RUN" system message delivered on
  /// the same EventNotifier (i=2253) an alarm source subscribes to - carries no
  /// ConditionId. ``on_event`` drops such an event for EVERY alarm source,
  /// explicit ``event_alarms`` and zero-config ``auto_alarms`` alike, before the
  /// mapping/auto-derivation split, so it never becomes a fault. Pure and static
  /// so the guard is unit-testable without a server.
  static bool is_condition_event(const opcua::NodeId & condition_id);

 private:
  void poll_loop();
  void event_pump_loop();
  bool same_code_active_elsewhere_locked(const std::string & fault_code, const std::string & condition_id_str) const;
  void do_poll();
  /// Issue #496: emit the component-scoped comms-lost raise/clear edge through
  /// the alarm callback (fault_code ``PLC_COMMS_LOST``, scoped to the node
  /// map's root component).
  void emit_comms_lost(bool active);
  void setup_subscriptions();
  void evaluate_alarms();
  void on_data_change(const std::string & node_id, const OpcuaValue & value);

  // Issue #386 helpers.
  void setup_event_subscriptions();
  void on_event(const AlarmEventConfig & cfg, const std::vector<opcua::Variant> & values,
                const opcua::NodeId & source_node, const opcua::NodeId & event_type,
                const opcua::NodeId & condition_id);

  /// True when native alarm events (explicit ``event_alarms`` or
  /// ``auto_alarms``) are configured at all - gates whether the poller
  /// bothers subscribing / re-subscribing on (re)connect.
  bool has_alarm_sources() const {
    return !node_map_.event_alarms().empty() || node_map_.auto_alarms().enabled;
  }

  /// Run the configured active-condition replay (issue #389). Dispatches to
  /// ConditionRefresh and/or the read-based fallback per
  /// ``condition_replay_strategy``.
  void replay_active_conditions();
  /// Call OPC-UA ConditionRefresh. Returns true when the server accepted it.
  bool try_condition_refresh();
  /// Read-based fallback: browse each alarm source, read current condition
  /// state, drive the state machine, and reconcile the fault set.
  void read_fallback_replay();
  /// Clear faults for conditions that were active before the replay but are
  /// no longer present/active in the read scan (``seen`` = condition ids
  /// observed this scan). Conditions whose owning source is in
  /// ``failed_sources`` (its browse failed this scan) are left untouched so a
  /// transient disconnect cannot falsely clear live alarms (issue #479).
  /// ``modeled_sources`` lists the sources positively known to expose Condition
  /// instance nodes; conditions whose source is absent from it are never
  /// cleared, so an EventNotifier-only server cannot wipe live alarms (#478).
  void reconcile_after_read(const std::set<std::string> & seen, const std::set<std::string> & failed_sources,
                            const std::set<std::string> & modeled_sources);

  /// Reconcile the tracked fault set after a completed ConditionRefresh burst
  /// (issue #480). ConditionRefresh (Part 9 §5.5.7) replays only Retain=true
  /// conditions between RefreshStart and RefreshEnd, so a condition that fully
  /// cleared while we were offline is silently absent from the burst and would
  /// otherwise stay latched. ``seen`` is the set of ConditionIds the server
  /// replayed during the burst; any tracked-active condition NOT in it cleared
  /// offline and is cleared here. Unlike the read fallback this needs no
  /// per-source modeling gate: a delivered RefreshEnd is an authoritative,
  /// subscription-wide replay (works on EventNotifier-only servers too).
  void reconcile_after_refresh(const std::set<std::string> & seen);

  /// Dispatch a batch of ClearFault deliveries to the event-alarm callback.
  /// Shared by the read- and refresh-based reconcile paths. Must be called with
  /// no poller lock held (invokes the user callback).
  void dispatch_condition_clears(const std::vector<AlarmEventDelivery> & clears);

  /// Emit an operator-visible warning via the configured ``log_warn`` sink,
  /// falling back to the poller's ROS 2 logger.
  void warn_operator(const std::string & msg);

  /// Apply one condition state observation (from a live event or a read scan)
  /// to the tracked condition map + state machine, dispatching the resulting
  /// fault action. ``event_id`` is the live EventId for ack/confirm (null for
  /// read-based observations). ``require_confirm_for_clear`` overrides
  /// ``config_.require_confirm_for_clear`` for this one observation - for an
  /// auto-derived alarm (see on_event), the caller passes ``false`` when
  /// ``auto_alarms.auto_clear`` is true (both Acked and Confirmed forced
  /// open so a zero-config alarm can clear without an operator), otherwise
  /// it passes the poller-wide ``config_.require_confirm_for_clear`` as-is.
  void apply_condition_state(const AlarmEventConfig & cfg, const opcua::NodeId & condition_id,
                             const AlarmEventInput & input, uint16_t severity, const std::string & message,
                             const opcua::ByteString * event_id, bool require_confirm_for_clear);

  OpcuaClient & client_;
  const NodeMap & node_map_;
  PollerConfig config_;

  std::thread poll_thread_;
  /// Dedicated subscription pump. Publish responses are only processed inside
  /// a runIterate, and the poll thread's single iterate per cycle loses the
  /// client mutex to sync API traffic (fleet health checker, UI, freeze-frame
  /// reads) long enough for outstanding publish requests to hit the client
  /// requestTimeout - all pending publishes then expire in one burst and
  /// AlarmCondition events silently stop. A short iterate every 100 ms from
  /// its own thread keeps the publish channel serviced regardless of poll
  /// cadence or API contention.
  std::thread event_pump_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> using_subscriptions_{false};

  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  mutable std::mutex snapshot_mutex_;
  PollSnapshot snapshot_;

  mutable std::mutex alarm_mutex_;
  AlarmChangeCallback alarm_callback_;
  // Shared raise/clear edge detector across all detection modes, keyed by
  // fault_code. Replaces the previous threshold-only last-state map.
  ros2_medkit::fault_detection::FaultTransitionTracker alarm_tracker_;

  // Issue #386: event-mode AlarmCondition state.
  EventAlarmCallback event_alarm_callback_;
  std::mutex event_alarm_callback_mutex_;

  // Atomic: written by poll/reconnect and setup, read by the event pump thread.
  std::atomic<uint32_t> event_subscription_id_{0};
  std::vector<uint32_t> event_monitored_item_ids_;

  mutable std::shared_mutex conditions_mutex_;
  std::unordered_map<std::string, ConditionRuntime> conditions_;  // ConditionId stringForm -> runtime

  // Issue #478 read-fallback safety state. Touched only from the replay path
  // (start() before the poll thread exists, then the poll thread on reconnect),
  // never concurrently, so no extra lock is needed.
  //
  // ``read_modeled_sources_``: alarm sources positively known to expose
  // Condition instances as address-space nodes (a read scan yielded >=1
  // ActiveState/Id condition). Only such sources are eligible for read-based
  // clearing; EventNotifier-only servers (S7-1500) never enter this set, so
  // their tracked faults survive an empty read scan.
  std::set<std::string> read_modeled_sources_;
  // Sources already warned about as read-fallback-unsupported (warn once each).
  std::set<std::string> read_unsupported_warned_sources_;
  // Configured alarm sources already warned that a non-condition (null
  // ConditionId) event was dropped despite their explicit fault_code/mappings
  // (warn once each; see the guard in on_event).
  std::set<std::string> noncondition_drop_warned_sources_;

  // ConditionRefresh bracketing state. open62541 sends the buffered historical
  // condition burst between RefreshStartEvent and RefreshEndEvent; we apply each
  // event during the burst as normal. On RefreshStart we clear ``refresh_seen_``
  // and accumulate every replayed ConditionId there; on RefreshEnd we reconcile
  // (clear tracked-active conditions the server did NOT replay - they cleared
  // offline). Per Part 9 §5.5.7 ConditionRefresh replays only Retain=true /
  // current-branch conditions; the state machine already drops non-current
  // branches via BranchId. Touched only on the poll thread (the burst is
  // delivered via run_iterate there), same convention as read_modeled_sources_.
  std::atomic<bool> condition_refresh_in_progress_{false};
  std::set<std::string> refresh_seen_;

  // Throttle the warn-level log emitted from condition_refresh() failures.
  // Reset to false on each fresh subscribe in setup_event_subscriptions()
  // so a transient server error (BadMethodInvalid on cold-start, recovers
  // later) gets one log per connect, not one per re-subscribe attempt.
  bool condition_refresh_warned_{false};

  // Issue #496: comms-lost debounce state, touched only on the poll thread.
  // ``comms_down_since_`` is set the first poll iteration the connection is
  // observed down and cleared on reconnect; ``comms_lost_raised_`` guards the
  // one-shot raise / matching clear so the fault is idempotent.
  std::optional<std::chrono::steady_clock::time_point> comms_down_since_;
  bool comms_lost_raised_{false};

  // Thread safety: must be set via set_poll_callback() before start().
  // Not modified after start(), so safe to read from the poll thread without a mutex.
  PollCallback poll_callback_;
};

}  // namespace ros2_medkit_gateway
