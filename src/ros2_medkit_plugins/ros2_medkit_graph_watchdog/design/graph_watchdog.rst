graph_watchdog design
=====================

Role
----
Detects silent faults in the ROS 2 graph. A ``GatewayPlugin``
hosting a fleet of detectors; each detector observes the ROS 2 graph and raises
faults via a ``ReportFault`` service client on the gateway node. Faults reach the
SOVD ``/faults`` API via FaultManager - the plugin injects no entities. Its only
HTTP surface is a read-only reliability status route (see `Reliability core`_
below).

Structure
---------
- **Plugin shell** (``GraphWatchdogPlugin``): loads via the gateway plugin ABI
  (v7). In ``set_context`` it casts the context with ``as_ros_plugin_context``,
  creates one ``rclcpp::Client<ReportFault>`` on the gateway node, and starts a
  tick wall timer on that node. No child node, no extra executor/thread - the
  gateway spins its own node.
- **Detector pattern**: ``Detector`` + ``DetectorContext``; each detector is one
  self-registering file under ``src/detectors/`` (``REGISTER_DETECTOR``), globbed
  by CMake so parallel authors never edit a shared file. A detector's source
  needs no CMake or registry edit; adding a per-detector unit test is the one
  shared touch - it adds an ``ament_add_gtest`` entry to the test block.
- **Fault-code contract**: the frozen ``GRAPH_*`` namespace.
- **mode seam**: ``raise`` / ``advisory`` / ``off`` per detector.
- **Reliability core**: see the dedicated section below - central warmup and
  lifecycle gating enforced inside ``raise_fault``, detector-consulted clock
  validity, and the ``x-medkit-watchdog`` status route.

Reliability core
-----------------
- **ReliabilityGate** composes two independent trackers and is the single entry
  point both the tick loop and the HTTP route go through:

  - ``WarmupTracker`` (pure, no ROS): an entity arms once continuously present
    for ``warmup_cycles`` ticks. A disappearance longer than a short forget
    grace followed by a reappearance (a real mid-run restart) re-warms it from
    scratch; a transient one-tick discovery gap is absorbed and does not
    re-warm, so recurring DDS churn cannot re-arm (and thus permanently
    suppress) a still-running entity. Unknown ``source_id``\ s (e.g. a topic,
    not a tracked app) fall back to a global bringup grace window keyed off the
    first tick the graph was seen non-empty, which re-arms whenever the graph
    empties out so a full-stack restart gets the grace again.
  - ``LifecycleWatcher`` (event-driven): discovers managed ``rclcpp_lifecycle``
    nodes from the introspection snapshot, seeds their state via a
    ``GetState`` service call, then keeps it fresh by subscribing to each
    node's ``~/transition_event`` topic (reliable + volatile, matching the
    ``rcl_lifecycle`` publisher). A node still cached non-active shortly after
    discovery is briefly re-seeded via ``GetState`` so an ``active`` transition
    lost during the subscription's DDS endpoint-matching window self-heals
    rather than suppressing the node for the process lifetime; the blocking
    seeds are bounded per tick so a batch bringup cannot stall the tick loop.
    The subscription callback holds only a ``weak_ptr`` to the watcher's shared
    state, so a callback in flight during teardown bails instead of touching
    freed memory. Non-managed nodes are never gated - ``node_ok`` returns true
    for them unconditionally.
- **Central enforcement.** ``DetectorContext::raise_fault`` runs every raise
  through ``reliability_allows(gate, source_id)`` before the fault client
  sends anything; a detector raising about a still-warming-up entity or a
  lifecycle-inactive node is silently suppressed, transparent to the
  detector. ``clear_fault`` is never gated - a fault can always be cleared. A
  null gate (not yet wired) always allows, so unit tests that construct a
  bare ``DetectorContext`` are unaffected.
- **Clock validity is detector-consulted, not central.**
  ``WatchdogClock::time_is_valid()`` flags a paused or absent ``/clock`` under
  ``use_sim_time`` by comparing wall-clock advancement against sim-time
  advancement on each ``mark_tick()``. Unlike warmup/lifecycle, this is not
  enforced inside ``raise_fault`` - a time-based detector must check it
  itself and skip its own age/grace-period math for the tick when it is
  false.
- **tf_static QoS helper.** ``tf_static_qos()`` returns a depth-1
  ``transient_local`` ``rclcpp::QoS``, available to detectors alongside
  ``time_is_valid()``. ``/tf_static`` publishers latch with transient-local
  durability, so a late subscriber must match it; the actual ``/tf_static``
  subscription belongs to a later detector.
- **Status endpoint.** ``GraphWatchdogPlugin::get_routes()`` registers
  ``GET x-medkit-watchdog`` (mounted at ``/api/v1/x-medkit-watchdog`` by the
  gateway), returning ``ReliabilityGate::status_json()``: schema version,
  ``warmup_cycles``, a ``global_state`` (``armed``/``warming_up``), and one
  entry per known entity (``id``, ``first_seen_tick``, ``armed``, ``state``,
  ``lifecycle``). Returns 503 with ``ERR_SERVICE_UNAVAILABLE`` if the gate has
  not been constructed yet or has already been torn down by ``shutdown()``.
- **Build note.** ``LifecycleWatcher`` reuses the gateway's own
  lifecycle-state helpers (``lifecycle_status_helpers.cpp``,
  ``ros2_lifecycle_state_reader.cpp``) compiled in via ``GATEWAY_SRC_DIR`` -
  the same non-header-only reuse pattern other gateway plugins use - rather than reimplementing lifecycle-state
  parsing.

Detectors
---------
``qos_mismatch`` is the detector this package ships first. The remaining silent-fault
classes land in follow-up changes, each against its own issue.

``qos_mismatch`` is the second detector and raises ``GRAPH_QOS_MISMATCH``. It
watches every topic's publisher/subscriber QoS pairs rather than parameter values: each
tick it enumerates every topic via ``get_topic_names_and_types()`` and, for each one,
every currently-connected publisher and subscriber (``get_publishers_info_by_topic`` /
``get_subscriptions_info_by_topic``, which report the RESOLVED live profile - never a
hand-built one), checking each pub x sub pair for incompatibility.

**Two levels behind one fault code.** Per-pair incompatibility is not reported uniformly.
A subscriber incompatible with EVERY publisher on its topic receives nothing and raises at
``SEVERITY_ERROR``. A subscriber incompatible with some publisher but not all raises at
``SEVERITY_WARN``: DDS refuses that one pair, so that producer's data is silently
discarded while the topic keeps looking alive - a real silent fault, since an
RxO-incompatible pair is a match DDS has already computed, not a heuristic. One fault code
carries one severity, so the emitted fault reflects the worst finding in the sweep. Before
this split the partial case was indistinguishable from healthy, which meant e.g. a second
``/tf`` broadcaster or a hand-rolled ``BEST_EFFORT`` publisher on ``/diagnostics`` went
unreported forever.

**RxO rule.** ``qos_policy.hpp``'s ``qos_incompatibility(pub, sub)`` implements RxO
(Request <= Offered) compatibility for the QoS policies that silently starve a
subscriber with no error surfaced anywhere in the graph: reliability, durability,
liveliness kind, deadline, and liveliness lease duration. Publisher = offered,
subscriber = requested; a pair is incompatible only in the one strict direction per
policy - a ``BEST_EFFORT`` publisher against a ``RELIABLE`` subscriber, a ``VOLATILE``
publisher against a ``TRANSIENT_LOCAL`` subscriber, an ``AUTOMATIC``-liveliness
publisher against a ``MANUAL_BY_TOPIC`` subscriber, or an offered deadline / lease
duration greater than the requested one. The reverse direction (e.g. a ``RELIABLE``
publisher feeding a ``BEST_EFFORT`` subscriber) is compatible by design - the subscriber
asked for no more than it is offered - even though the QoS profiles differ, which is
exactly the discriminator the integration and e2e tests each pin with a positive control
(see "Test tiers" below). For deadline and lease, an unspecified/infinite offer is
unbounded (fails any finite request), while an unspecified subscriber value does not
constrain the policy (always compatible); history and depth are not RxO dimensions and
are not checked. The checks match only concrete incompatible enum pairs, so
``SYSTEM_DEFAULT``/``UNKNOWN`` (never
reported by a live endpoint, which always carries the resolved profile) never raise.

**Aggregation via the shared helper.** ``qos_mismatch`` is the first detector to use
the new ``AggregatedFault`` helper (``aggregated_fault.hpp``), factored out of
a shared helper so later detectors do not each reimplement the same
level-triggered raise/clear pattern. The rationale is
(see "Aggregation rationale" above): the fault_manager identifies a fault by
``fault_code`` alone, so one ``GRAPH_QOS_MISMATCH`` per mismatched topic would collide
into a single record under the shared code. The detector keeps one ``AggregatedFault``
instance for the whole graph and, each tick, hands it every currently-mismatched
topic's description (keyed by topic name so a repeat mismatch on the same topic
overwrites rather than duplicates); an empty map on a clean tick clears
(``EVENT_PASSED``), level-triggered semantics (see
"Closing the loop" in the README for the ``healing_enabled`` requirement to actually
reach HEALED). ``graph_source_id()``, also in ``aggregated_fault.hpp``, is the same
host-Component-id-or-literal-fallback logic - the detectors'
aggregated faults land under the same ``source_id``.

**Coverage is exhaustive, not budgeted.** Unlike a call-budgeted
round-robin sweep (bounded by a parameter-service transport call budget),
``qos_mismatch`` reads no external service - ``get_topic_names_and_types()`` and the
two ``get_*_info_by_topic()`` calls are local graph-cache queries, so every topic is
checked every tick with no coverage-latency trade-off to configure or budget knob to
tune. ``/rosout`` and ``/parameter_events`` are skipped - ROS 2 system topics with
their own well-known QoS conventions, not useful signal for this detector. The sweep
polls ``ctx.cancelled`` between topics so a shutdown mid-sweep bails promptly, the same
shutdown-responsiveness contract every detector honors.

**Test tiers.** Three tiers each prove a different layer, deliberately not
overlapping:

1. **Unit** (``test_qos_policy.cpp``): pure ``qos_incompatibility()`` logic against
   hand-built ``rmw_qos_profile_t`` values - the RxO trio (reliability, durability,
   liveliness), the reverse-direction compatible case, identical-profile compatibility,
   and the never-raise guarantee for ``SYSTEM_DEFAULT``/``UNKNOWN`` enums a live
   endpoint never actually reports.
2. **Integration** (``test_qos_mismatch_integration.cpp``): a real ``rclcpp::Node``
   publisher/subscriber pair over real DDS, read through the actual
   ``get_publishers_info_by_topic``/``get_subscriptions_info_by_topic`` API - proving
   the detector reads the RESOLVED live profile correctly, not just that the pure
   comparison function is correct - against a fake ``ReportFault`` service. A second,
   concurrent RxO-compatible-but-different-QoS topic pair proves the detector does not
   over-fire on "QoS differs somewhere in the graph".
3. **E2e** (``test/e2e/test_qos_e2e.test.py``): the Acceptance gate - a
   real gateway process with the plugin ``.so`` loaded, a real fault_manager, and the
   operator-visible ``GET /api/v1/faults`` surface, proving the whole raise/clear story
   reaches a SOVD fault through the real tick timer, not just the detector's own
   ``tick()`` called directly.


Status
---------------
The plugin loads, ticks the graph, and shuts down cleanly. The reliability core is
real and already ticking, and all six silent-fault detector classes are live and
raising through it. The remaining silent-fault classes land in follow-up changes,
each against its own issue.
