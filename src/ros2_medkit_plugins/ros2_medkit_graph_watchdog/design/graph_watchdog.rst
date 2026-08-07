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
  dedicated tick thread. The tick is deliberately NOT a gateway wall timer:
  detectors do blocking parameter/service reads, and the gateway's small
  executor is only safe because blocking never runs on an executor thread. No
  child node.

  Every ROS entity the plugin owns - the fault client here and the
  ``LifecycleWatcher``'s ``~/transition_event`` subscriptions below - is created
  in a callback group of its own with
  ``automatically_add_to_executor_with_node = false``, and added only to a
  private ``SingleThreadedExecutor`` that the tick thread pumps between ticks.
  Neither executor is ever spun on a thread of its own. The reason is the same in
  both cases and is a correctness requirement, not tuning:
  ``rclcpp::AnyExecutable`` holds a strong reference to the subscription or
  client it dispatches and is a local of the executor thread, so an entity left
  in the node's default group can have its last reference released - and its
  destructor run, mutating the node's rcl entity registry - on a gateway executor
  thread, concurrently with this plugin creating an entity on the same node. No
  lock the plugin holds can reach a gateway executor thread, so the fix is to
  keep the entities out of every gateway executor; that puts creation,
  destruction and callback execution on the tick thread alone.

  For the fault client the drain is load-bearing for a second reason:
  ``DetectorContext::raise_fault`` is fire-and-forget (it guards on
  ``service_is_ready()`` and discards the future), and rclcpp keeps per-request
  state until an executor processes the response, so an unpumped client would
  leak one pending entry per raised fault for the life of the process.
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

    Those subscriptions follow the private-callback-group rule described in the
    plugin shell bullet above: created on the shared gateway node, but in the
    watcher's own group and pumped only by the watcher's own executor, so they
    are created, run and destroyed on the tick thread alone. This is also why the
    tick thread drains in short slices rather than once per tick - a transition
    would otherwise be delayed by up to a whole tick interval, and a bringup
    burst could overrun the subscription's queue and lose the intermediate
    transitions the departure classification reads.
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
``qos_mismatch``, ``orphan`` and ``param_drift`` are the detectors this package ships so
far. The remaining silent-fault classes land in follow-up changes, each against its own
issue.

``qos_mismatch`` raises ``GRAPH_QOS_MISMATCH``. It
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

**Aggregation via the shared helper.** ``qos_mismatch`` was the first detector to use the
``AggregatedFault`` helper (``aggregated_fault.hpp``); ``orphan`` and ``param_drift`` go
through the same one, so no detector reimplements the level-triggered raise/clear
pattern. The rationale: the fault_manager identifies a fault by
``fault_code`` alone, so one ``GRAPH_QOS_MISMATCH`` per mismatched topic would collide
into a single record under the shared code. The detector keeps one ``AggregatedFault``
instance for the whole graph and, each tick, hands it every currently-mismatched
topic's description (keyed by topic name so a repeat mismatch on the same topic
overwrites rather than duplicates); an empty map on a clean tick clears
(``EVENT_PASSED``), level-triggered semantics (see
"Closing the loop" in the README for the ``healing_enabled`` requirement to actually
reach HEALED). ``graph_source_id()``, also in ``aggregated_fault.hpp``, returns the
constant ``kGraphWatchdogEntityId`` (``"graph_watchdog"``) and nothing else, so every
detector's aggregated faults land under the same ``source_id``. It deliberately does NOT
fall back to the host Component id: a runtime host Component built by ``HostInfoProvider``
never sets ``external``, ``collect_component_app_fqns`` only puts a Component's bare id in
the fault scope set when it does, and a fault raised under it is therefore listed by no
entity endpoint at all. The ``ctx`` argument is unused and kept only because every call
site already has it in hand.

**Coverage is exhaustive, not budgeted.** Unlike a budgeted
round-robin sweep (bounded by a parameter-service round-trip budget),
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



``orphan`` raises ``GRAPH_ORPHAN``. It looks for the mistake no ROS 2 tool
reports: a topic that exists but has no counterpart, because one side was spelled
slightly differently. Each tick it counts publishers and subscribers per topic, keeps
the one-sided ones, and pairs a publisher-only topic with a subscriber-only topic when
their names are within ``max_edit_distance`` Levenshtein edits of each other. Only a
pair is reported, never a lone one-sided topic: a publisher with no subscriber is
normal on almost every robot, so it carries no signal on its own.

**Three guards keep this from crying wolf.**

- Names must sit in the same parent namespace. ``/a/scan`` and ``/b/scan`` differ by one
  character but belong to two different robots, and pairing them would be nonsense. The
  leaf and the namespace carry separate edit budgets and the namespace one defaults to
  zero, so this is an exact match unless an operator raises
  ``namespace_edit_distance``. Raising it is what makes a misspelled namespace
  (``/robott/scan`` against ``/robot/scan``) visible at all, at the price of reporting
  every robot of a fleet whose namespaces are themselves near-misses, which is why it is
  opt-in rather than tuned.
- A pair that matches once every run of digits collapses to one placeholder is an
  enumeration, not a typo. ``/lidar_1`` next to ``/lidar_2``, each one-sided, is the
  ordinary state of a multi-sensor machine, and reporting it would fire on every such
  machine. Collapsing runs rather than comparing character positions also covers
  ``/lidar_9`` next to ``/lidar_10``, where the index changes length. The price is a real
  typo in a digit going unreported, which is a deliberate trade.
- ``grace`` consecutive sweeps must agree before anything is raised. During bringup one
  side of a healthy pair is routinely missing for a moment, which looks exactly like a
  typo until the other side appears.

An operator who still gets a false alarm puts the topic in ``allowlist``. What this
detector cannot see at all is a namespace added or dropped by accident: ``/scan``
against ``/robot/scan`` is far past any sensible edit distance.

**Test tiers.** ``test_orphan_policy.cpp`` pins the pure matching rules, including each
guard and the cases it deliberately lets through. ``test_orphan_integration.cpp`` drives
the detector over a real ``rclcpp`` graph and a fake ``ReportFault`` service, covering
the grace counter, the clear on repair, and the allowlist. ``test/e2e/test_orphan_e2e.test.py``
is the acceptance gate: a real gateway with the plugin loaded, a real fault_manager, and
the fault read back from ``GET /api/v1/faults``. It carries three pairs at once - one
reported, one allowlisted, one visible only under a raised ``namespace_edit_distance`` -
so it also proves a string array and an integer survive the parameter path into
``configure()``, which no C++ tier can show.


``param_drift`` raises ``GRAPH_PARAM_DRIFT``. It reads node parameters over the
parameter service and reports a value that no longer matches its reference. The
reference is either self-captured, the value the parameter had when the node armed, or
pinned in config through ``expect``. A node absent for longer than the window the
reliability gate absorbs loses its captured baseline, so a restart - the documented way to apply
a new configuration - re-captures instead of reporting the change as drift. A shorter gap keeps
the baseline on purpose: DDS discovery drops a node from a single poll routinely, and re-capturing
on that would take the node's current, possibly already drifted, value as the new reference and
heal a real fault that could then never fire again. Only the pinned form can catch a value that was
already wrong at startup; self-capture by construction treats whatever it first sees as
correct.

**A budgeted sweep, unlike every other detector here.** ``qos_mismatch`` and ``orphan``
read the local graph cache and are therefore exhaustive every tick. This one makes real
service calls to other nodes, so it is bounded by ``max_reads_per_tick`` and sweeps the
graph round-robin. The budget is spent in service ROUND TRIPS at the watched node rather
than in transport calls, because one call is several requests there: a self-capture read
is a list plus a batched get, and an ``expect`` pin the node declares is a list, a get and a
descriptor read. A pin the node does NOT declare stops at the list, and is charged two rather
than the one it costs, because a second and rarer NOT_FOUND path does cost two and the error
code cannot tell them apart - a load bound may overstate, never understate. That buys a bound
on the load the sweep puts on watched nodes and pays with
coverage latency: a drift is invisible until its node's turn comes, and so is a repair.
The reads run on a thread the plugin owns rather than the gateway executor, so a node
that stops answering stalls this sweep only.

**Three horizons for a stale entry.** A node missing from one sweep is not forgotten
immediately. Its captured BASELINE is dropped on the third consecutive sweep without it - the
two the warmup tracker's forget grace absorbs, plus one - so that a one-tick discovery gap does
not silently reset the baseline and hide a real drift behind a fresh capture. Its FINDING
survives ``prune_grace`` consecutive absences and is dropped on the next one, so that the same
gap does not clear the aggregated fault either. Only the finding horizon is configurable:
``prune_grace`` is a per-detector config key, while the baseline horizon is the compile-time
``kBaselineForgetGrace``. A ``prune_grace`` below 2 makes the finding the shorter of the two and
the baseline then goes on the same sweep, because the absence counter it rides on is dropped
with the finding.

The third horizon is not about absence at all. A node that stays PRESENT but leaves the
reliability gate's ``active`` state is never re-evaluated (it is not armed) and never pruned (it
is still there), so its finding would otherwise be re-raised for ever - an operator who fixes
the parameter and leaves the node deactivated would watch ``GRAPH_PARAM_DRIFT`` stay CONFIRMED
naming a value that is no longer true. ``kFrozenHoldTicks`` (60 consecutive gate-denied ticks,
one minute at the default tick period) bounds that: past it the finding is released and the app
also stops holding back a clear, and both re-derive from a fresh read if it ever arms again.
Because it is a compile-time constant and counted in ticks rather than in rotations, neither
``max_reads_per_tick`` nor ``tick_interval_ms`` shortens it.

**Severity is WARN, deliberately.** The detector knows a value moved; it does not know
whether that breaks the robot. Grading the consequence needs knowledge of the machine
that the graph does not carry.

**Nothing is cleared that was not measured.** A clear asserts that nothing is drifting anywhere,
which is only true if every app was actually looked at, so the aggregate emits nothing at all while
any app still has no usable read. That covers three states which are all "nothing is known about
this app": the round-robin has not reached it, its reads keep failing, or the reliability gate has
not armed it - and the last of those is the ordinary state of every app for the first
``warmup_cycles`` ticks of a run, so without it a restart clears before issuing a single round trip.
A cached read counts as evidence only until a read of that app FAILS, after which the app is
unmeasured again. Each hold is bounded from the other side: an app is given up on after both sixty
failed attempts and sixty ticks unread, and a gate-denied one after the frozen hold. The bound on
attempts alone would not do, because an attempt is not a duration - the reader paces itself off
``max_reads_per_tick``, so at the accepted ceiling sixty-one attempts are spent in milliseconds and
a node still being discovered would be written off. Finally, a clear withheld for a minute of ticks
is logged once with the count of apps behind it, because from outside the process a correctly
withheld clear and a detector that has nothing to report look exactly the same.

**Test tiers.** ``test_param_drift_policy.cpp`` pins the pure comparison, rendering and
config rules. ``test_param_drift_integration.cpp`` drives real nodes with real parameters over
the real parameter service against a fake ``ReportFault`` service, including a node that
never answers, which must not hang the tick, and a node that answers past the per-read bound,
which must not be read at all.
``test/e2e/test_param_drift_e2e.test.py`` is the acceptance gate: one gateway launch carrying
the whole raise-and-clear story for the detector's default self-capturing mode, with a real
fault_manager and both ends read back from ``GET /api/v1/faults``.
``test/e2e/test_param_roundtrip_e2e.test.py`` measures what one parameter read really costs the
node that serves it, against a node that answers the parameter services by hand and counts every
request - so a round trip added to or removed from the TRANSPORT is caught, instead of quietly
invalidating the round-trip figures the budget is built on. Nothing else in the package can
notice that: every other test proves the arithmetic GIVEN those figures, and the stand-in
transport they use counts calls. It does not, however, read the detector's own charge constants
at all - it runs with ``param_drift`` switched off - so what holds those to the measurement is
the timing suite in ``test_param_drift_integration.cpp``, which times the reader against a
configured budget and therefore moves when a charge does. Cost and charge are pinned separately
and on purpose.
``test/e2e/test_config_plumbing_e2e.test.py`` drives three further gateway launches covering the
nested ``expect`` sub-object reader, ``mode: "off"``, and the bare ``mode: off`` that the
ROS parser types as a YAML 1.1 boolean; those three prove configuration delivery and
suppression only, and assert no clear.


Status
---------------
The plugin loads, ticks the graph, and shuts down cleanly. The reliability core is real
and already ticking. Three silent-fault detector classes raise through it today,
``qos_mismatch``, ``orphan`` and ``param_drift``. The remaining classes land in follow-up
changes, each against its own issue.
