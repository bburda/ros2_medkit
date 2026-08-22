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
    An entry's identity is its BINDING - the pair (fqn, ``GetState`` path) - not
    its ``App::id``, which can survive a graph sweep while pointing at a
    different node. When either half moves, the entry is dropped (the old
    binding recorded as departed under its own fqn) and re-seeded from scratch,
    so a moved binding can never keep enforcing the old node's label.
    The subscription callback holds only a ``weak_ptr`` to the watcher's shared
    state, so a callback in flight during teardown bails instead of touching
    freed memory. Non-managed nodes are never gated - ``node_ok`` returns true
    for them unconditionally, and so is a managed node whose ``GetState`` has
    never answered: its label stays empty, and ``node_ok`` reads an unread label
    as permission on purpose, since gating on it would silence every detector
    for a node whose lifecycle service is broken. Only a KNOWN non-active label
    suppresses. ``presence_ownership()`` asks the STRICTER question the presence
    class needs, without changing that permissive answer, and answers it with a
    GROUND rather than a boolean: EARNED for a state read as ``active`` or for a
    node with no lifecycle at all, PROVISIONAL for one whose state was asked for
    as often as it ever will be and never came (``measurement_pending()``, which
    reads the per-node GetState re-seed budget charged only for reads that
    actually ran), and neither while the asking is still going on. Only the first
    is permanent - see the ``node_death`` section.

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

  Beside the gate's own state the payload carries a ``detectors`` object, one block per
  detector whose ``Detector::status_json()`` returns something, omitted entirely when none
  does. It exists for a condition that belongs to a SINGLE detector and would otherwise live
  only in the gateway log, which no HTTP client and no e2e assertion can read -
  ``lifecycle_expectation`` reports ``tracking_saturated`` (it has refused to track a
  required node) beside the live ``tracked_nodes`` count and the ``tracked_node_cap`` in
  force. The handler runs on an HTTP thread while ``tick()`` runs on the tick thread and
  holds no lock a detector takes, so a ``status_json()`` implementation must build its
  payload from atomics and must not block.
- **Build note.** ``LifecycleWatcher`` reuses the gateway's own
  lifecycle-state helpers (``lifecycle_status_helpers.cpp``,
  ``ros2_lifecycle_state_reader.cpp``) compiled in via ``GATEWAY_SRC_DIR`` -
  the same non-header-only reuse pattern other gateway plugins use - rather than reimplementing lifecycle-state
  parsing.

Detectors
---------
``qos_mismatch``, ``orphan``, ``param_drift``, ``lifecycle_expectation`` and
``node_death`` are the detectors this package ships so far. Two silent-fault classes
remain undelivered, ``GRAPH_TF_STALE`` and ``GRAPH_LATENCY_BUDGET``; they land in
follow-up changes, each against its own issue.

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


``lifecycle_expectation`` raises three independent faults, ``GRAPH_NODE_INACTIVE``,
``GRAPH_NODE_UNREADABLE`` and ``GRAPH_NODE_NOT_MANAGED``. It watches an operator-declared
set of ``require_active`` node names rather than topics, parameters, or presence. Each
tick it matches every configured entry against the live apps by ``App::id`` OR the
stable ``effective_fqn()`` OR that fqn's bare leaf name - ``App::id`` alone is
recomputed each sweep and gets namespaced on a same-bare-name collision, so an id-only
match would silently stop covering a multi-robot graph; a bare name therefore matches
that node in every namespace (use a full FQN to pin one). For each matched present node
it reads the node's raw lifecycle label via ``ReliabilityGate::lifecycle_state_of()``
and hands the matches - a ``std::vector<LifecycleMatch>`` of (entry, node fqn, state) -
to ``lifecycle_expectation_tracker.hpp``'s ``LifecycleExpectationTracker``, the
detector's pure, ROS-free core, independently unit-tested before the detector ever
touches a live gate.

**The model: one observed state per node per tick, two clocks - not three fault codes
each running their own bookkeeping.** All three faults are outputs of a SINGLE per-node
state machine inside the tracker. Every tick, a matched node is classified into exactly
one of ``ACTIVE``, ``INACTIVE`` (non-empty label, not ``active``), ``UNREADABLE`` (a
managed node whose label has never been read - ``optional("")``), or ``NOT_MANAGED``
(``nullopt`` - no tracked lifecycle at all); a node not matched at all this tick is a
fifth case, ``ABSENT``, handled by a wholly separate path. Two clocks track history:

- The **violation streak** advances on ``INACTIVE``, resets ONLY on ``ACTIVE``. Past
  ``grace`` the node is ``GRAPH_NODE_INACTIVE``'s content.
- The **unmeasured clock** advances on ``UNREADABLE`` OR ``NOT_MANAGED``, resets ONLY on
  a real measurement (``ACTIVE`` or ``INACTIVE``). It is deliberately BLIND to which of
  the two causes it is seeing on any given tick - a node alternating between "matched but
  never read" and "not a tracked lifecycle node at all" keeps this ONE clock climbing
  instead of each cause resetting a separate counter of its own. That blindness closes an
  entire class of alternation, not one more special case for it: three review rounds
  each found a pair of "I cannot measure this" causes whose separate counters could erase
  each other's progress, leaving a node invisible to every code that existed. Past
  ``unmeasured_hold_ticks`` (a fixed 60 ticks, not configurable), the clock MATURES and
  takes ownership of the node.

**Ownership is exclusive.** A node is content of at most one of the three faults at a
time. The moment the unmeasured clock matures, the violation streak is RELEASED -
actually reset to 0, not merely excluded from ``GRAPH_NODE_INACTIVE``'s content this
tick - so that fault's clear is free to flow immediately; a node returning to
``INACTIVE`` afterwards re-earns ``grace`` from zero. Which of the two unmeasured codes
a matured node reports under is STICKY: it keeps reporting under whichever code the
clock matured under even if the LIVE cause later flips, and only a real measurement
(resetting the whole clock) changes it. The rejected alternative, current-cause-wins,
is more literal but makes a node that flaps between the two causes flap the fault
surface too - a raise/clear/raise churn across two codes for a node whose actual
situation never changed.

**Absence continues the last CORROBORATED observation, it never erases.** A node not matched
at all this tick is ABSENT, a fact separate from any observed state (there is no label to
classify). For up to ``absence_grace`` (a fixed 3 ticks) consecutive absent ticks a node's
whole state is held unchanged - the blink tolerance. Past ``absence_grace`` absence resets
nothing, but what it ADVANCES depends on whether the node's SETTLED observation is already
CONTENT, and - for a below-grace violation streak only - on whether the presence detector ever
OWNED the node: a settled-``INACTIVE`` node ALREADY reported under ``GRAPH_NODE_INACTIVE``
keeps its streak exactly as it was, so the fault stays raised, regardless of ownership. One not
yet past ``grace`` splits on that fact: a node the presence detector COULD have tracked
(admitted for ownership at least once) is HELD - neither advanced (no fault born from evidence
gathered while nobody could observe the node, since ``node_death`` is able to report this exact
departure instead) nor erased (it
resumes rather than restarts on return) - while a node the presence detector could NEVER have
tracked keeps climbing on absence exactly as it would on a present tick, because nothing else
in the plugin will ever report its departure either. A settled-``UNREADABLE``/``NOT_MANAGED``
node keeps climbing its unmeasured clock under that same cause regardless of maturity or
ownership - that clock's own absence behaviour is unrelated to this distinction. A
settled-``ACTIVE`` node advances nothing at all - anything it had started but not corroborated
is released, so the entry becomes idle and is reclaimed silently. So a departure never heals a
fault it has already earned, and never starts one out of evidence gathered while the node
could not be observed AND could be reported some other way; what it changes, for an
already-raised fault, is the detail phrase, which then says the node has since left the graph.

**What "settled" means, and why an unmeasured reading needs corroborating.** A real
measurement (``ACTIVE`` or a non-active label) settles at once - a label is a fact about the
node, and no discovery artifact invents one. An UNMEASURED reading settles at once too on a
node nothing has ever measured, since it is then the only thing known about it (this is what
keeps a crash-looping node accumulating toward its report rather than being released on every
absence). Only when an unmeasured reading would OVERRIDE a real measurement must it first
hold for ``kDefaultObservationSettleTicks`` (6) consecutive matched ticks. The transient it
guards against is real and cheap to hit: ``LifecycleWatcher::update()`` drops a tracked id
whose ``get_state`` path is absent from the current sweep, and ``discover_apps()`` can yield
an app with no services when a sweep races service enumeration - so a present, healthy,
managed node reads ``NOT_MANAGED`` for a tick. If that tick is the last one before a clean
shutdown, continuing "the last observation" literally would mature a healthy departure into a
permanent ``GRAPH_NODE_NOT_MANAGED``. Six is the same bar the typo warning already sets
against the same transient, and a tenth of the 60-tick unmeasured hold, so R30 is untouched: a
node that is genuinely unmeasurable when it leaves corroborated that long before the hold that
reports it.

**Where "a departure never heals a fault" ends.** It holds within one gateway lifetime. Across
a RESTART it does not: the restarted tracker has no measurements, the departed node is not in
the graph, and its ``require_active`` entry matches nothing - which this detector cannot tell
apart from a misspelt entry, because the only component that knows the difference is the fault
store and it is not read at startup. The never-matched hold is deliberately bounded (a typo
must not block healing forever), so once it lapses the level-triggered clear flows and the
record heals with nothing having been measured. Re-seeding the tracker from the fault store at
startup would change that and is not implemented; the boundary is pinned by the
``restart_departed`` e2e scenario rather than left to be discovered.

Why the erasure went away rather than moving, for the two UNMEASURED causes: every erasure
horizon is an evasion for a node that touches it periodically. A node in a restart loop -
start, crash, respawn delay, start - touches absence by construction, and discarding its
evidence would let it alternate ``(UNREADABLE, ABSENT x N)`` or ``(NOT_MANAGED, ABSENT x N)``
forever without ever accumulating enough of anything to be reported.

The VIOLATION streak needed the identical rule for the identical reason once - a node
alternating ``(INACTIVE, ABSENT x N)`` would otherwise never accumulate ``grace`` + 1 either.
Where the presence class CAN also report the same departure, it no longer does:
``GRAPH_NODE_DISAPPEARED`` (this package's own ``node_death`` detector) independently
reports it, whether or not the node was ever measured not-active first. But ``node_death``
only ever tracks an App once the reliability gate says it OWNS that App's departure, on one of
one of the two grounds ``ReliabilityGate::presence_ownership()`` accepts - so a ``require_active``
node that never reaches ``active`` is never tracked by ``node_death``, and neither is a managed
node whose ``GetState`` has not answered while the watcher is still asking. The gate would
permit that second one to RAISE - ``LifecycleWatcher::node_ok()`` treats an unread label as
permission, deliberately, or a broken lifecycle service would silence ``qos_mismatch``,
``orphan`` and ``param_drift`` too - but permission is not knowledge. Once the asking stops the
node IS tracked, provisionally, and a label arriving afterwards over ``~/transition_event`` and
reading non-active takes it straight back out again. A node that is merely RE-WARMING is not
that: warmup says nothing about who a node belongs to, so the gate distinguishes ``kUnclaimed``
(nothing known yet) from ``kDisowned`` (measured as another detector's) and only the second
withdraws a key already held.

That withdrawal happens inside ``node_death``'s own tick, and only on a tick that still sees
the app present, so it leaves a window of about one entity-cache refresh in which a node that
dies right after its label arrives is reported by the presence code anyway (measured: 210 ms
between the label reaching the status route and the release). A withdrawal that HAS happened
is final, which needed its own guard: a dying managed node loses its lifecycle services from
the snapshot before it loses its App entry, and a node with no managed record answers
``kEarned`` - so dying erased the very measurement that disowned the node and re-admitted the
key on the tick before it departed. A released key is therefore readmitted only on a label
reading ``active``, never on the absence of a label. It is a mis-attribution of a TRUE
report, not a false positive, and it cannot be closed at report time: the departed record keeps
only the last label, which reads ``inactive`` both for a node earned and then deactivated - a
death this detector must report - and for one only ever held provisionally. Separating them
after the fact needs per-key ownership history surviving the departure, which is the unbounded
state the tracked-key prune exists to prevent. So
the split is on whether the presence detector EVER owned the node - a fact read from the
reliability gate itself, not guessed from the observed label: once owned, a below-``grace``
streak that goes absent is simply HELD rather than matured on the strength of the absence
alone - two codes standing at once for the same node is not a problem to fix, it is
``GRAPH_NODE_INACTIVE`` and ``GRAPH_NODE_DISAPPEARED`` saying different, both-true things. A
node the presence detector never owned gets no such backstop - ``GRAPH_NODE_DISAPPEARED``
structurally cannot report it - so absence keeps advancing its streak exactly as a present tick
would. Reporting a HEALTHY node that left remains ``GRAPH_NODE_DISAPPEARED``'s job as well, for
either kind of node - a healthy departure never starts a violation regardless of ownership.

**Content follows the clocks, not the snapshot.** Whether a node was in this tick's matches
decides nothing about what it reports: a node past ``grace`` stays in
``GRAPH_NODE_INACTIVE``'s content while it blinks, and a matured node stays in its own
code's content. A fault's content is what has been MEASURED about the node, and a missing
snapshot entry measures nothing either way.

**Presence is not enough here - and absence is someone else's fault.** A
present-but-non-active managed node (``inactive``, ``unconfigured``, ``finalized``) is
exactly the case ``GRAPH_NODE_INACTIVE`` exists to catch: on a Nav2 stack, a
``controller_server`` stuck ``inactive`` means the robot silently will not act, with
no crash and no signal on ``/diagnostics`` or ``/rosout``. A node that vanishes having
only ever been measured HEALTHY is the presence fault class's business, and this detector
raises nothing for it; a node that vanishes while already reported under any of the three
faults KEEPS that fault, because leaving the graph answers nothing the operator asked -
the integration test's ``AbsenceAfterRaiseKeepsTheFaultAndSaysTheNodeIsGone`` and
``HealthyNodeThatVanishesRaisesNothingAtAll`` pin the two halves of that split.
**Safe default: off.** ``require_active`` is
empty by default, so the detector emits nothing at all - not even clears - until an
operator opts specific nodes in (zero false positives, the same config-scoped posture
``param_drift``'s ``expect`` uses).

**Keyed by node, not by config entry.** The bare-name form of ``require_active`` is
deliberately fleet-wide ("all ``controller_server``\ s must be active"), so one entry
legitimately covers N nodes. The tracker therefore keys every fact by the node's
stable fqn: two namesakes are two report entries (one cannot silently replace the
other, and a healthy one cannot mask a broken one), and the fault description names
each offending node's FQN with the entry that demanded it as context - the
description is the only carrier, since every ``GRAPH_*`` fault shares one
``source_id``. Two entries naming the SAME node advance its clocks once per tick, not
once per matching entry.

**Why grace, not the reliability gate (the design decision this detector turns on).**
Every detector's raise is subject to the central ``reliability_allows(gate,
source_id)`` gate inside ``ctx.raise_fault()``, which ANDs the entity's warmup state
with ``LifecycleWatcher::node_ok()`` - true only when a managed node's lifecycle is
``active`` (or it is not tracked at all). Gating THIS detector's raise the same way on
the required node's OWN lifecycle would be self-defeating: ``node_ok()`` is exactly
FALSE for the inactive node this detector exists to report, so the central gate would
suppress the signal forever. Instead the tracker counts consecutive not-active ticks
itself, per node, entirely independent of ``reliability_allows``. This does not bypass
bringup-quiesce entirely: the aggregated fault's own outer ``ctx.raise_fault`` call is
still subject to the central gate for the aggregate's own ``source_id``
(``graph_watchdog``) - only the per-node lifecycle check bypasses it, because that check
IS what this detector reports on.

**Bounded by evidence, not by age.** The map is NOT simply bounded by the
operator-declared ``require_active`` set: keys are node fqns, and one bare-name entry
matches a node of that name in every namespace, so identity churn (nodes reappearing under
ever-new fqns) would grow it without a bound. Since absence no longer erases anything, an
entry carrying a live clock is not reclaimed by age either - those are the same defect seen
twice, and moving a horizon rather than removing it leaves the evasion in place at a
different N. So ``prune_ticks`` (the operator's ``prune_grace``, used as written - there is
no ``grace + 1`` clamp any more, because there is nothing left for one to protect) reclaims
IDLE entries only: both clocks at zero and no matured ownership, i.e. nothing to lose, and
still atomically - ONE map entry per node, gone in the same tick, never partially. A
non-idle entry is never pruned by age. An entry whose UNMEASURED clock is still climbing
cannot grow without bound in time either: past the absence grace it advances every tick, so
it matures within at most ``60 + absence_grace + 1`` ticks and is reported - the longest
``GRAPH_NODE_UNREADABLE`` or ``GRAPH_NODE_NOT_MANAGED``'s clear can be withheld by one
departed node. A below-``grace`` VIOLATION streak on a node the presence detector never owned
shares that same bound, for the same reason it advances at all while absent: it matures within
at most ``grace + absence_grace + 1`` ticks, because nothing else will ever report that node's
departure either. Only once a node HAS been owned does its below-grace streak lose the
bound: absence then holds it rather than advancing it, so it neither matures nor becomes
idle for as long as the node is away. That is not a new way to withhold
``GRAPH_NODE_INACTIVE``'s clear - the clear was already gated on every required node's
status being settled, so one node this indecisive already blocked it; what changes is only
that the fault never NAMES an OWNED node's departure, since that node's evidence belongs to
``GRAPH_NODE_DISAPPEARED`` instead. ``grace`` is still capped at 300 instead of being
accepted up to ``INT_MAX - 1``, because it independently
bounds how long a PRESENT node may go unreported and how long a returning node takes to
re-mature: at the old maximum that PRESENT-side bound was roughly 24 days at the shipped
cadence, with no warning - silence indistinguishable from a working detector finding
nothing.

What bounds the map is ``tracked_node_cap`` (default 512, ``kDefaultTrackedNodeCap``,
accepted range 1..16384): at the cap idle entries are reclaimed first, then entries for
DEPARTED nodes are collapsed into per-code COUNTS - lexicographically last first, keeping at
most ``kMaxNamedDepartedEntries`` (3) named, since three maximally-long details are all one
480-character description holds - and only if every tracked node is PRESENT and carrying
evidence is the NEWCOMER refused, never a live violation evicted.

Collapsing follows from the fault being keyed by CODE rather than by node: five hundred
entries for dead identities keep the same one fault raised that a single entry would, so
holding them buys nothing while the slots they occupy can cost total blindness. Under
identity churn a cap full of the dead would refuse a genuinely broken PRESENT node - which
then never reaches ``affected`` or ``pending``, is not covered by the never-matched hold
either (its entry HAS matched), and so ``GRAPH_NODE_INACTIVE`` would emit a level-triggered
CLEAR every tick while that node sat there not-active. The count is content, so collapsing an
entry heals nothing; it is ordered ahead of the individually named entries but behind
anything crossing on this tick, so a node that just broke is never displaced by it. It only
grows within one tracker lifetime: a collapsed entry's fqn is no longer known, so a node
returning under it is tracked and measured afresh.

A refused node is a required node going unchecked, so refusing one is never silent: it
withholds ``GRAPH_NODE_INACTIVE``'s clear for as long as it lasts, is logged once per
saturation EPISODE (the latch re-arms when the episode ends, so a later, real saturation is
not silenced by an earlier one), and is reported on ``GET /x-medkit-watchdog`` under
``detectors.lifecycle_expectation``. That withhold is deliberately UNBOUNDED, unlike the
never-matched hold beside it: the never-matched hold is bounded because a typo must not block
healing forever, while saturation - once departed entries can no longer crowd out present
ones - means genuinely more required PRESENT nodes than the cap allows, a capacity condition
the operator resolves rather than a transient that resolves itself.

**The clear is withheld until the required set has actually been measured - and this is
entirely about** ``GRAPH_NODE_INACTIVE``. The other two faults have no withheld-clear
guard of their own; see "Three independent faults, not one shared record" below for why
they don't need one. A clear asserts that every required node is free of a CONFIRMED
violation. That is the restart-heal hazard: a gateway restart brings every detector
counter back to zero while the ``GRAPH_NODE_INACTIVE`` raised before it is still in the
fault_manager's store (a separate process), so a clear emitted on the strength of an
empty affected map heals a fault that is still real. TWO things produce that empty map
without the assertion being true, both the ordinary state of bookkeeping that just
started over (a restarted gateway, a reconfigure - ``configure()`` rebuilds the tracker
- or a node that respawned stuck):

- **Not matched yet.** Before the entity snapshot catches up with the graph a
  ``require_active`` entry matches nothing at all, so the clear is about a node the
  detector has never once looked at. The plugin ticks as soon as it is loaded, so every
  restart passes through this window. Bounded the same way node-keyed state is (60
  ticks), so a misspelt entry cannot block healing for the process lifetime.
- **A node's status is UNSETTLED** (the tracker's own ``pending`` set). A violation streak
  that has not yet passed ``grace``, an unmeasured clock still climbing under EITHER cause,
  or a streak HELD while the node is inside an unmeasured spell. Absence never puts a node
  here on its own - content follows the clocks, so a node already past ``grace`` stays in
  the fault's content through a blink rather than dropping into a withheld limbo. A node whose
  unmeasured clock has MATURED is deliberately NOT in this set - ownership passed to its
  own fault code and the violation streak was released, so it stops counting toward this
  withhold the exact tick it stops being uncertain, rather than continuing to poison
  ``GRAPH_NODE_INACTIVE``'s withhold decision the way a shared record used to.

Either reason withholds the emission entirely, neither raise nor clear. A raise is never
withheld: a violation read from the nodes that DID answer is real regardless of the
unmeasured ones. The never-matched leg and both unmeasured causes are bounded: they release
after 60 consecutive ticks (a minute at the shipped cadence, mirroring ``param_drift``'s
frozen hold), whether the node is present or gone. The pending leg releases the same
way - as soon as the node reads ``active``, or a PRESENT tick pushes its streak past grace
and the fault is raised again - but while the node stays absent that release has no timeout
of its own: absence holds a below-grace streak rather than advancing it, so the hold lasts
for exactly as long as the node does not return. Every hold releases by SETTLING the node's
status, never by giving up on it. Because a correctly
withheld clear and a detector with nothing to report look identical from outside, a hold
that lives past 10 consecutive ticks is explained in the log once per episode, naming
every reason in force and, for the node-keyed ones, the count behind each and one node
by name - the not-managed and unreadable reasons are still named separately even though
both now release the same way (into their own fault code), since an operator reading the
log wants to know WHICH of the two is happening.

**Three independent faults, not one shared record.** ``GRAPH_NODE_INACTIVE``,
``GRAPH_NODE_UNREADABLE`` and ``GRAPH_NODE_NOT_MANAGED`` are each raised through the
shared ``AggregatedFault`` helper every ``GRAPH_*`` detector uses (one graph-level
record per code, since the fault_manager identifies a fault by ``fault_code`` alone),
but as three SEPARATE, fixed-severity class members - ``GRAPH_NODE_INACTIVE`` always
``SEVERITY_ERROR``, the other two always ``SEVERITY_WARN`` - the same shape
``orphan_detector`` and ``param_drift_detector`` use, rather than one record whose
severity is chosen from that tick's mixed content the way
``qos_mismatch_detector``'s ``any_starved ? kStarvedSeverity : kPartialSeverity`` does
for its own single code. Raises are fully independent: each fault's content comes from
its own measurement, and one raising, healing, or changing severity never forces,
blocks, or reflects onto another. ``GRAPH_NODE_INACTIVE``'s own clear is not simply
"nothing CONFIRMED non-active this tick" though - it is withheld exactly as described
above; that is the ORIGINAL withheld-clear guarantee this detector always gave, scoped
to ``GRAPH_NODE_INACTIVE`` alone. The other two have no such guard: each one's own
clear needs nothing beyond its own content going empty, because once the unmeasured
clock has matured "still cannot be measured" is a settled fact, not a pending one. A
node is content of at most one of the three at a time, and healing one never forces,
blocks, or changes the severity of another. Content under either unmeasured code
survives for as long as a node stays that way, with no further bound past the initial
hold.

**A re-bind is a fresh binding.** ``LifecycleWatcher`` keys its tracked map by
``App::id``, and an id can survive a graph sweep while pointing at a DIFFERENT node
(id assignment shifts under bare-name collisions) - an entry kept across such a move
would keep enforcing the old node's label, and its old ``~/transition_event``
subscription, against the new binding. An entry's binding identity is its fqn plus
its ``GetState`` service path, both captured at first sighting, and ``update()``
re-checks that identity every tick. A moved binding is two events at once: the OLD
binding departed (recorded under ITS OWN fqn in ``recently_departed_``, same record
and retention as a vanish), and the id is new again (erased and re-seeded through the
ordinary new-node path: fresh ``GetState``, fresh subscription, fresh self-heal
budget). For this detector the consequence is that a re-bind is never enforced with the
departed node's label: the new binding starts unknown (benign) until its own label is
read.

No straggler from the old subscription has to be filtered out of the new entry. Erasing
the entry destroys that subscription, on the tick thread: the private executor that
runs these callbacks is pumped by the same thread that runs ``update()`` (see the
plugin-shell bullet above), so nothing else holds a reference to it and no message it
had queued is delivered afterwards. The same property removes the other ordering
question this seam used to carry - a re-seed's blocking ``GetState`` cannot be
overtaken by a ``~/transition_event``, because while ``update()`` blocks nothing is
pumping the events at all.

**New violations are named first, not alphabetically - in EACH fault's own description
independently.** The description used to list affected nodes in fqn order
(``AggregatedFault::emit``'s default), which is fine when every entry is equally
interesting but not once the cap is full: a fleet sharing
``require_active: ["controller_server"]`` across a dozen robots fills the 480-char cap
from the alphabetically-earliest ones, and a THIRTEENTH robot going inactive afterward
would be silently invisible forever - one shared ``fault_code``, one record, no way to
tell the operator which of thirteen actually broke. The tracker reports which fqns
entered EACH fault's content on THIS tick - ``newly_affected``, ``newly_unreadable``,
``newly_not_managed`` - and each list orders only its OWN fault's
``AggregatedFault::emit_ordered`` call, since the three faults never share a
description: a fresh entry is named FIRST in whichever fault it belongs to; every other
affected node in that same fault still appears, in the same fqn order as before, once
the fresh ones are placed. When every node crosses on the same tick (``grace: 0``
during a bringup burst, for instance), there is nothing to distinguish them by and the
order degrades to fqn order. Two budgets protect each description, applied before it
ever reaches the 480-char cap: the lifecycle label - which arrives verbatim off a
remote ``~/transition_event`` and is therefore untrusted and unbounded, and only ever
appears in ``GRAPH_NODE_INACTIVE``'s own detail, never the two unmeasured faults' -
is trimmed to 32 characters before it is interpolated (more than double the longest
label a conforming implementation produces, ``errorprocessing`` at 15 characters), and
the whole per-node detail is then capped at 150 characters as a backstop against a
pathological fqn or a long ``require_active`` "required by" list, sized so at least
three worst-case details still fit inside the 480-char cap
(``3 * 150 + 2 * 2 = 454 <= 480``).

**Test tiers.** Three tiers each prove a different layer, deliberately not
overlapping - plus the shared-watcher seam the re-bind behaviour lives in:

1. **Unit** (``test_lifecycle_expectation_tracker.cpp``): pure
   ``LifecycleExpectationTracker`` logic over hand-built matches. Covers the violation
   streak (stuck-inactive past grace raising keyed by the node with the entry as
   context, active never raising, reaching active within grace resetting the streak,
   both namesakes reported separately, two entries not halving the grace, a violating
   read winning a duplicate match's tie-break); the unmeasured clock shared by
   UNREADABLE and NOT_MANAGED (neither ever confirms a violation while climbing; each
   matures into its OWN fault at the exact hold boundary; the clock resets ONLY on a
   real measurement, in both directions; maturity RELEASES the violation streak
   entirely, so a node returning to inactive re-earns grace from zero; the sticky-cause
   decision, pinned directly against the rejected current-cause-wins alternative); the
   alternation this redesign closes - a node alternating between unreadable and
   not-managed, indefinitely and on every single tick, still matures the shared clock;
   the ``INACTIVE``/``NOT_MANAGED`` alternation across absence gaps longer than the
   absence grace, now counting only the MEASURED not-active legs; the violation streak
   surviving non-maturing unmeasured ticks and RESUMING rather than restarting, counted
   exactly and read through ``pending_violation``; absence CONTINUING an already-matured
   fault exactly as it was on its last present tick - a blink holds either clock unchanged;
   past the blink tolerance an unmeasured clock still climbing keeps climbing and matures on
   absence alone, while a below-grace violation streak is instead HELD and RESUMES rather
   than restarts once the node returns, proven both from one long absence and from many
   short ones interleaved with matched reads; a matured violation fault survives a
   departure, a node measured ACTIVE that vanishes raises nothing and is reclaimed, and the
   two ``(UNREADABLE/NOT_MANAGED, ABSENT x N)`` restart-loop shapes are swept at the
   absence grace and past it (their ``INACTIVE`` sibling pinned to the opposite claim at the
   same two N: it never crosses grace from absence alone, for a node the presence detector
   owned at some point - a node it never owned gets the opposite result instead, maturing
   from absence alone within ``grace + absence_grace + 1`` ticks exactly like the unmeasured
   clock does, and resuming rather than restarting on return the same way an owned node's
   held streak does); the ``pending`` set and its
   per-reason breakdown; new-first
   ordering for all three fault-shaped maps; the remote-supplied label's own trim budget
   ahead of the whole-detail backstop, reapplied to a matured unreadable node's detail
   (which carries no label, only a fqn and a "required by" list); the age horizon
   reclaiming IDLE entries only, atomically, and never one that carries evidence even
   when it undercuts the absence grace; the SETTLING rule that decides what absence may
   continue - an uncorroborated unmeasured run before a healthy departure raising nothing,
   swept across every run length below the bound and with the entry released rather than
   left holding the clear hostage, the same run one tick longer still being reported, and a
   single MEASURED not-active read before a departure still confirming; and the tracked-node
   cap - idle entries reclaimed first, departed entries collapsed into a count so a present
   broken node is always admitted and reported, at most three left named, the count
   surviving into the description as content, a node returning after its entry was collapsed
   measured afresh, the newcomer refused only when every entry is PRESENT and carrying
   evidence, and saturation reported as a LEVEL on every refused tick with its edge
   re-arming when an episode ends - swept at a shrunk cap and again at the real shipped 512.
   The re-bind seam is shared infrastructure and is pinned separately in
   ``test_lifecycle_watcher.cpp``.
2. **Integration** (``test_lifecycle_expectation_integration.cpp``): the detector
   driven against a fake ``ReportFault`` service, with a REAL ``ReliabilityGate``
   arming the global bringup grace and feeding the detector its labels through
   ``lifecycle_state_of()`` - injected via ``set_lifecycle_state_for_test()`` strictly
   AFTER the last ``gate.update()`` call for most cases, or, for the alternation and
   not-managed cases (which need a genuine ``nullopt`` the injection seam cannot
   produce - it can only ever SET a tracked value, never remove tracking), by toggling
   whether the matched app carries lifecycle services and driving the gate's real
   discovery path instead. The fixture cases cover the ``GRAPH_NODE_INACTIVE``
   raise/clear round trip naming the stuck node; the active-from-arming positive
   control; the fault SURVIVING the required node's departure while a healthy node's
   departure raises nothing at all; bare-name and full-FQN matching against a
   namespaced app; the zero-config default measured as zero fault_manager requests of
   any kind; the unmanaged-entry and no-match warnings; the withheld-clear guard's
   releases and its once-per-episode log line; the blink-plus-unread-re-seed sequence;
   a filler batch sized (from the real detail-building code) to exceed the 480-char cap
   aggregating into one fault with the fresh crossing named first; a PRESENT node crossing
   grace fresh while a batch of already-departed, already-matured ones sits
   absent-and-content, named ahead of them rather than truncated away by them; a required
   node appearing mid-run; a re-bind under the same
   ``App::id``; and the reconfigure/config-validation edge cases. The unmeasured clock's own split is pinned directly,
   for BOTH codes symmetrically: a managed node whose ``GetState`` genuinely never
   answers (through ``set_managed_app`` and a real, failing seed - proven by asserting
   ``lifecycle_state_of()`` actually returns ``optional("")`` first, not assumed) is
   reported under ``GRAPH_NODE_UNREADABLE``; a node with no tracked lifecycle at all is
   reported under ``GRAPH_NODE_NOT_MANAGED`` (no longer released into silence, the
   deliberate behaviour change this redesign makes); either hold releases into its
   report on the exact tick past its bound; a node already reported under one of the
   two clears once genuinely read, returning to ordinary ``GRAPH_NODE_INACTIVE``
   tracking; a confirmed node healing while an unreadable OR not-managed sibling stays
   present clears ``GRAPH_NODE_INACTIVE`` promptly without touching either unmeasured
   fault; content under either code survives with no window and no expiry; 25 nodes of
   either cause aggregate into one capped fault at ``SEVERITY_WARN``; a node whose hold
   expires opens its fault's own description over an already-reported filler batch with
   new-first ordering; an already-reported node of either cause KEEPING its own record
   once it vanishes, with its description switching to say the node has left the graph; a
   node returning from that absence staying reported with no clear/re-raise churn; each of
   the two UNMEASURED restart-loop shapes raising its own code from absence alone (their
   ``INACTIVE`` sibling instead pinned to counting only real reads, never crossing from any
   of the interleaved absence gaps), and the ``inactive``/``not-managed``
   alternation across absence gaps now counting only the MEASURED not-active legs; a
   below-grace streak surviving the tightest ``prune_grace`` without being confirmed while
   the node stays absent, and resuming (not restarting) once it returns; a withheld
   ``GRAPH_NODE_INACTIVE`` clear releasing when
   the absent node's clock MATURES into its sibling's content rather than when the node is
   given up on; the two wire strings pinned as hand-typed literals; and the independence
   claim in both directions.

   The ``configure()``-level cases pin the config contract: the unknown-key warning for
   ``require_activ`` (the worst-case typo - it also leaves the detector unconfigured, so
   the warning must precede the zero-config early return), a fully-valid config
   producing zero warnings (including ``grace: 0`` and ``prune_grace: 0``, the
   documented low endpoints), negative, non-integer, past-the-int-range and
   exact-``kMaxGrace``-boundary ``grace`` (both sides), ``prune_grace`` out of 0..3600
   rejected on the WIDE integer (never truncated into a hair-trigger prune) plus both
   range endpoints accepted, non-array ``require_active`` and empty-string and
   non-string entries each warning, the unclamped prune horizon reaching idle bookkeeping
   at exactly the configured ``prune_grace`` (0, 1 and 4 - the smallest positive value
   included, since neither documented endpoint sweeps it) while a node carrying evidence
   survives it - including the ``grace: 0, prune_grace: 0`` corner (an UNMEASURED clock, the
   only kind that keeps climbing while absent) and a wide ``grace`` beside the tightest
   ``prune_grace`` for a below-grace VIOLATION streak, whose instrument is that it is neither
   confirmed nor pruned while the node stays absent, and resumes rather than restarts once it
   returns - ``tracked_node_cap`` validated at both range endpoints and one value past each
   with the key proven IN FORCE at both ends, boundedness under identity churn at the real
   512-node cap by collapsing the departed rather than refusing the live node, and saturation
   reported once per EPISODE with a second episode reported again after the first ends.
3. **E2e** (``test/e2e/test_lifecycle_expectation_e2e.test.py``): the acceptance
   gate - one source file, TWELVE CTest targets (the config-plumbing pattern: the
   plugin reads its config once at ``set_context()``, so different configs need
   different gateway launches), each bringing up a real gateway with the plugin
   ``.so`` and a real fault_manager, asserting on the operator-visible
   ``GET /api/v1/faults`` surface. Six scenarios drive the ``managed_lifecycle``
   demo node (a real ``rclcpp_lifecycle::LifecycleNode``, which always answers
   ``GetState``); two, ``unreadable`` and ``departure_keeps``, drive a fixture built to
   never answer it (see below); the last two, ``not_managed`` and ``restart_loop``, drive
   ``calibration`` - a PLAIN demo service node with no lifecycle interface at all, so no
   purpose-built fixture was needed for ``GRAPH_NODE_NOT_MANAGED``, unlike
   ``GRAPH_NODE_UNREADABLE``.
   The main scenario proves raise-survives-CONFIGURE-survives-RESTART-heals-on-ACTIVATE
   through real ``lifecycle_msgs/srv/ChangeState`` transitions, including the
   entity-scoped ``/apps/graph_watchdog/faults`` surface. The restart leg is the
   withheld-clear guard at the only tier that can reach it: the gateway is SIGTERMed,
   its port is waited down, the relaunched process is gated on being armed again, and
   the fault about the still-inactive node must come back with ``last_passed`` unset.
   The default-config scenario launches with no ``lifecycle_expectation`` config at all
   against the same inactive node and holds a sustained silence window, and the
   negative control does the same with the self-activating variant of the same
   executable; the discriminating variable is the node's actual lifecycle state. Both
   silence scenarios gate on three facts before asserting absence: the plugin is
   armed; ``GET /faults`` answers 200 in THIS launch; and the target's label was
   actually READ, pinned through the plugin's own ``GET /x-medkit-watchdog`` route.

   The fourth scenario, ``healing_threshold``, is the withheld-clear guard's pending
   leg at the only tier that runs the REAL fault_manager debounce state machine at
   all. The required node, already reported stuck, is SIGTERM'd and respawned twice
   under the same name (a real snapshot blink, not a lifecycle transition), each one
   comfortably inside the tracker's fixed absence grace, against a
   ``healing_threshold`` of 1. This same run is also this package's only e2e coverage
   for a node oscillating readable/unreadable producing no ``GRAPH_NODE_INACTIVE``
   event churn, read raw off ``GET /faults/stream``: no ``fault_cleared`` frame for
   ``GRAPH_NODE_INACTIVE`` at any point, and every ``fault_confirmed``/
   ``fault_updated`` frame carrying ``severity_label: "ERROR"`` - its only possible
   value now that severity is fixed per code.

   The fifth scenario, ``unreadable``, is where the 60-consecutive-tick failure that
   matures ``GRAPH_NODE_UNREADABLE`` is actually reached. It launches
   ``unreadable_lifecycle_node.cpp``: a plain ``rclcpp::Node`` that advertises
   ``get_state``/``change_state`` with the service TYPES ``find_lifecycle_get_state_path``
   actually checks for, whose ``get_state`` handler stores every request and never
   answers until its own ``start_answering`` parameter is set true. Against that
   fixture the scenario proves the node is present and matched with its lifecycle
   label read as ``""``; ``GRAPH_NODE_INACTIVE`` stays silent for it throughout;
   ``GRAPH_NODE_UNREADABLE`` raises once the hold expires, names the node, and carries
   ``SEVERITY_WARN`` - a node is content of at most one of the three, never more than
   one; and once the fixture starts answering ``"active"`` the watcher reads it through
   a real ``GetState`` round trip and ``GRAPH_NODE_UNREADABLE`` clears.

   The sixth scenario, ``departure_keeps``, proves what a DEPARTURE does to an
   already-reported unmeasured fault: nothing. The same fixture is SIGTERM'd permanently
   once ``GRAPH_NODE_UNREADABLE`` has raised, confirmed gone from ``GET /apps``, and past
   every horizon that could have discarded its evidence the fault is still active, has
   never once been reported PASSED (``last_passed``, which catches even a transient
   clear), and its description now says the node has left the graph. The seventh,
   ``not_managed``, is the sibling proof for the OTHER cause: ``calibration`` is present
   and matched from launch, ``GRAPH_NODE_NOT_MANAGED`` raises once its hold expires naming
   the node at ``SEVERITY_WARN``, and neither ``GRAPH_NODE_INACTIVE`` nor
   ``GRAPH_NODE_UNREADABLE`` ever raises for it - the one live proof that the NOT-MANAGED
   cause specifically never bleeds into the UNREADABLE code it shares a clock with. Its
   departure leg mirrors ``departure_keeps``'s own shape.

   The eighth scenario, ``restart_loop``, is the acceptance gate for the whole
   evidence-retention model. The same plain ``calibration`` node, but respawning, is
   SIGTERM'd over and over on a cadence that never lets it accumulate the 60 consecutive
   PRESENT ticks the unmeasured hold would otherwise need. Every cycle's absence is proven
   rather than assumed - ``GET /apps`` is polled until the node is gone and again until it
   is back, and the measured gap must EXCEED the 3-tick absence grace, on top of launch's
   own enforced ``respawn_delay`` floor - so every cycle genuinely crosses the horizon past
   which evidence used to be discarded. ``GRAPH_NODE_NOT_MANAGED`` must raise anyway and
   survive the restarts that follow. A required node in a crash loop is the case this
   detector most exists to catch, and the one a design that discarded evidence on absence
   made permanently silent.

   The last four scenarios are about the bounds themselves. ``cap_pressure`` runs
   ``tracked_node_cap: 1`` against TWO required nodes, so one is refused on every tick -
   reachable only because the cap is a config key; against a compile-time 512 it would need
   513 real lifecycle nodes. It proves the refusal is visible on ``GET /x-medkit-watchdog``,
   that it WITHHOLDS ``GRAPH_NODE_INACTIVE``'s clear (measured through ``last_passed``, which
   catches even a transient clear, at the moment the tracked node's unmeasured clock matures
   and that fault's content goes empty), and that the entry for a node that then LEAVES is
   collapsed so the present, still-broken node is admitted and named.
   ``unsettled_departure`` runs two healthy managed nodes built from this package's own
   ``droppable_lifecycle_node.cpp`` - which looks managed, answers a chosen label, and stops
   advertising its lifecycle services on command - and drops the services of each: one is
   killed immediately (a single missed sweep before a clean shutdown, about which nothing may
   ever be reported) and the other holds the dropped state past the settling budget before
   being killed (genuinely not managed when it left, so it must still be reported). The first
   leg's window is MEASURED against the settling budget rather than assumed, so it cannot
   silently turn into the second. ``wide_grace`` configures the value that used to be the
   accepted ``grace`` maximum and proves it is refused and the documented default applied -
   under it the detector could neither raise nor heal for days. ``restart_departed`` records
   where "a departure never heals a fault" ends: the required node is killed while its fault
   is outstanding, the gateway is restarted, and the record then heals - because the restarted
   detector cannot tell an entry for a departed node from a misspelt one.


``node_death`` raises ``GRAPH_NODE_DISAPPEARED``. It watches every ARMED App in the graph
for one that goes offline - zero-config, unlike ``lifecycle_expectation``'s
operator-declared ``require_active`` list, because every armed App is a candidate.
Liveness is ``App::is_online``, never mere membership in the entity snapshot: in
runtime-only discovery the two happen to coincide (a dead node's App leaves the snapshot
entirely), but a manifest keeps a bound App present with only ``is_online`` cleared once
its ROS binding disappears, and hybrid discovery inherits that shape - counting snapshot
membership alone would make a manifest-declared node immortal. A managed lifecycle node
that merely deactivates keeps ``is_online: true`` (its process is still running, only its
ROS 2 lifecycle state changed), so a deactivation is never mistaken for a death either -
that is ``lifecycle_expectation``'s concern, not this one's. Tracking is keyed on the
STABLE fqn (``App::effective_fqn()``), never ``App::id``: an id is recomputed every sweep
and only gains a namespace prefix once a same-bare-name collision currently exists
anywhere in the graph, so a live node's id can change out from under a key built from it.

**What is never tracked.** A peer-aggregated app (``app.source`` starting ``peer:``)
carries no ROS binding of its own, so ``effective_fqn()`` is empty for every one of them;
tracking them would collapse an entire peer fleet onto one ``""`` key. A
``_ros2cli_<pid>`` node - rcl's own hidden-node naming convention for every ``ros2`` CLI
invocation, matched structurally rather than guessed at - is excluded too, or every CLI
invocation would accumulate one permanently "dead" entry for the life of the gateway. An
App that never comes online is skipped by the ``is_online`` filter above and so is never
admitted to tracking in the first place - the identical protection a node still inside
``warmup_cycles`` gets. A MANAGED App whose lifecycle state has not been read YET is excluded
too, and for a different reason: the gate's raise permission for it is deliberately PERMISSIVE
(an unread label must not silence ``qos_mismatch``, ``orphan`` or ``param_drift``), but this
detector reports an ABSENCE and cannot attribute the departure of a node whose state nothing
has read - so tracking asks ``presence_ownership()`` instead.

That exclusion is BOUNDED, and the readmission that follows it is PROVISIONAL. The watcher
charges a GetState re-seed budget per node and charges it only for a read that actually ran, so
an empty label with attempts left is "we have not finished asking" and an empty label with none
left is "we asked and failed". Past that point the node is admitted here after all - nothing
will ASK again, and ``lifecycle_expectation`` returns before it looks at anything unless an
operator named the node in ``require_active``, which defaults to empty, so refusing forever
would mean a whole class of deaths reported by nobody. But asking is not the only channel: the
``~/transition_event`` subscription is created independently of the seed budget and is never
torn down when it runs out, so a label can still arrive unasked. One that reads non-active says
the node was ``lifecycle_expectation``'s all along, and this detector releases the key while the
node is still alive rather than reporting a death it was never entitled to. Ownership EARNED
from a measurement is not released that way: a node that ran and then deactivated is still a
death when it stops running. Stickiness is earned by knowledge, never by ignorance.

What that does NOT close: an App counts as managed only once the snapshot shows it advertising
a ``GetState``-typed service, and until then it is indistinguishable from a plain node, so a
managed node whose services have not yet been discovered can still be taken for one. That
window is bounded by the arming warmup, which already needs several consecutive ticks of
presence, and node and services come from the same introspection sweep. Nor does anything here
close the case of a node that appears, lives fewer than ``warmup_cycles`` ticks and vanishes:
the gate has always required completed warmup before it arms anything, so such a node is owned
by nobody and reported by nobody. That is the bringup-quiesce trade-off working as designed,
not a gap this boundary introduces. Once
tracked, though, a node's continued life-or-death judgement rests on PRESENCE alone, not
on staying armed: a tracked node that later goes lifecycle-inactive is not thereby
mistaken for dead. Composable/component nodes hosted in one process die together - if the
container process dies, every node it hosted leaves the snapshot on the same tick, and all
of them are named in the one aggregated fault rather than raising one fault each.

**The wall-clock floor on** ``miss_grace``. The entity snapshot a tick reads is not
rebuilt every tick: runtime discovery rebuilds it off a graph event debounced to about one
refresh per second, so several consecutive ticks between two refreshes see the IDENTICAL
snapshot. A ``miss_grace`` counted purely in ticks does not count independent samples of
the graph - shorten ``tick_interval_ms`` enough and one stale cache generation gets
re-counted as several misses. The floor raises ``miss_grace``, for the configured
``tick_interval_ms``, to the smallest value whose ``(miss_grace + 1) * tick_interval_ms``
window is still at least 3000 ms - one graph-cache refresh cycle plus margin. At the
shipped 1000 ms tick the floor is exactly the shipped default (``miss_grace: 2``), so the
default configuration is unaffected; only a faster-than-default tick ever raises the
effective grace, with a warning naming the window it actually spans.

**Suppression: opt-in, and only by name.** Nothing is suppressed unless the operator
names the mechanism in ``suppress`` - a configured ``allowlist`` that ``suppress`` does
not name has NO effect, and a startup warning says so. Two mechanisms exist, activated
independently of each other:

- ``"allowlist"`` builds an ``AllowlistSuppressor`` over the configured ``allowlist`` set
  - the same three-way match (fqn, bare leaf, ``App::id``) ``lifecycle_expectation``'s
  ``require_active`` uses, so an operator who has one working can expect the other to
  accept the same shapes. Exact match only, never a prefix or a shared suffix.
- ``"lifecycle"`` builds a (stateless) ``LifecycleShutdownSuppressor``: it suppresses a
  departure the reliability gate classifies as a clean managed-lifecycle shutdown - see
  "Clean-shutdown suppression" below for what counts as clean.

Every field is re-validated from scratch on every ``configure()`` call, so a malformed
``allowlist`` entry, an unrecognized ``suppress`` name, or a wrongly-typed entry each
produce their own named warning rather than being silently dropped. A candidate is
suppressed the moment ANY active mechanism votes yes, order-independent - which one runs
first never changes the result. Suppression is independent of ``mode``: a suppressed key
never becomes content in the first place, while ``mode`` separately decides whether
non-suppressed content is actually sent.

**Durability, and why only a durable veto may reclaim bookkeeping.** Whether a suppressor
is durable answers whether its "yes" is a standing fact about the key rather than a
condition that can later stop holding. Both mechanisms here are durable: an
operator-declared allowlist entry does not start matching and then stop on its own, and a
departure's observed shutdown shape does not change after the fact. Durability is what
makes reclaiming a suppressed key's tracker bookkeeping (``prune_grace`` consecutive
suppressed ticks) sound - a NON-durable veto could lift later even after its key's
bookkeeping was discarded, leaving a live, unsuppressed death with nothing left to report
it: a false heal that silently outlives the very condition that produced it. A durable
veto never lifts for a given key once it has fired, so reclaiming under it loses nothing.
``Suppressor::durable()`` defaults to false, the safe assumption for a suppressor nobody
has reasoned about yet; both suppressors here override it explicitly to true. Reclaiming
is re-checked against the durable suppressors specifically, never merely "no longer in
this tick's report", because a key can be dropped from a tick's report by ANY suppressor
in the chain while a durable one also happens to cover it - whether a key may be reclaimed
depends on WHICH suppressor vetoed it, not on whether it survived the filter. The generic
chain is reusable (``Suppressor`` plus the free function ``apply_suppressors()``,
independently unit-tested), but this detector's own filtering is hand-inlined rather than
a call to that helper, because it has to interleave the id-form check above - answerable
only while the node is still present - with the ordinary fqn/leaf dispatch the helper's
plain string-keyed signature does not have room for.

**Clean-shutdown suppression - what counts as clean.** ``LifecycleShutdownSuppressor``
reads the lifecycle watcher's cached label for the departed node's LAST observed
transition. ``shuttingdown`` alone suppresses - it is only reachable via a deliberate
SHUTDOWN transition, so the label alone is enough. ``finalized`` suppresses only WITH an
observed transition and never through the error branch; without an observed transition,
or reached through the error branch, it does not, because ``finalized`` is also where a
node's ``on_error`` override lands after ``ON_ERROR_FAILURE``/``ON_ERROR_ERROR`` out of
``errorprocessing`` - the standard way a driver reports a hardware fault it cannot
recover from, which is precisely the death an operator needs reported, not silenced.
``unconfigured`` is deliberately excluded even though it looks quiet: it is also the
resting state of a node whose ``configure()`` failed or that never activated at all, and
suppressing on it would hide exactly that startup failure. Any other label, or no
departure on record at all, abstains. The mechanism is stateless by construction - a
detector's ``configure()`` runs before any per-tick context exists, so it stores nothing
at construction and reads the gate fresh on every call - and its retention window is not
this detector's own to size: before this detector's own ``configure()`` has run, the
plugin predicts the same ``prune_ticks_`` (``max(prune_grace, miss_grace + 1)``) this
detector will compute, then sizes the lifecycle watcher's departed-node retention from it
with enough margin that a clean-shutdown label is still cached at this detector's own
reclaim tick, one tick to spare
(``GraphWatchdogPlugin::compute_departed_retention_ticks()`` - the resulting window is
larger than ``prune_ticks_`` alone).

**Bounded by evidence, not by age - and why this cap, unlike the sibling's, cannot
actually saturate.** This detector is zero-config over every armed App in the graph - a
strictly larger scope than ``lifecycle_expectation``'s named ``require_active`` set - so
identity churn would grow the tracker's map without a bound if nothing capped it. An
unsuppressed, still-dead entry is never reclaimed by age (``prune_grace`` only ever
reclaims a DURABLY suppressed key), so ``tracked_node_cap`` (default 512, accepted range
1..16384) is what bounds memory. At the cap: idle entries (present, carrying no evidence)
are evicted first - free, since a still-armed one is simply re-tracked a moment later -
then departed entries are collapsed into one synthetic count, keeping at most three
individually named, and only if even that leaves no room is every departed entry
collapsed. Unlike ``LifecycleExpectationTracker``, whose two clocks let a node be
simultaneously present and mid-violation (neither idle nor departed, and so un-evictable -
exactly what lets ITS cap genuinely saturate and withhold ``GRAPH_NODE_INACTIVE``'s
clear), ``NodeLivenessTracker`` carries exactly one piece of per-key state, so every
tracked identity is always either idle or departed; collapsing every departed entry always
empties enough room, so a newcomer is never actually refused while the cap is at least 1
(``NodeLivenessTrackerCap.SaturationNeverFiresBecauseEveryKeyIsEitherIdleOrCollapsible``
pins this directly). ``tracking_saturated`` and ``tracked_node_cap`` still appear in this
detector's own ``GET /x-medkit-watchdog`` block, in the same shape the sibling reports
them, but the field is never observed true here - the shared shape exists for consistency
with the sibling detector, not because saturation is reachable in this one.

**The boundary with** ``lifecycle_expectation``. ``GRAPH_NODE_INACTIVE`` and
``GRAPH_NODE_DISAPPEARED`` can both be raised for the same node at the same time, and that
is CORRECT, not a defect this detector removes: ``GRAPH_NODE_INACTIVE`` says a required
node is present but not active, ``GRAPH_NODE_DISAPPEARED`` says a node is gone, and an
operator facing each has a different repair - reactivate it, or find out why it left. A
node CONFIRMED ``GRAPH_NODE_INACTIVE`` that then departs keeps that fault (its description
switches to say the node has since left the graph) AND now also raises
``GRAPH_NODE_DISAPPEARED``, since this detector tracks every App it owns independently of
whatever ``lifecycle_expectation`` thinks of it.

What changed is narrower than that, and it only applies to a node this detector could ever
have tracked. Before this detector existed, ``lifecycle_expectation``'s own violation streak
let a SUSTAINED absence mature an unconfirmed (below-``grace``) streak into a confirmed
``GRAPH_NODE_INACTIVE`` - the only way a node stuck in a restart loop, never observed long
enough to cross ``grace`` while present, would ever be reported at all. Where this detector
can independently report the same departure - which needs the presence detector to have OWNED
the node at least once, since it only ever tracks an App the reliability gate has admitted for
ownership, and ownership needs a MANAGED node's lifecycle state known to read ``active`` or
asked for as often as it ever will be, plus the node being online - that absence-alone maturity
is gone:
absence CONTINUES a violation that has already matured past ``grace`` (the fault stays
raised, still naming the node), but no longer CREATES one that has not. A node that was
briefly non-active and then died is reported as gone (``GRAPH_NODE_DISAPPEARED``) rather
than also acquiring an inactive fault born from ticks gathered while nobody could observe
it. A ``require_active`` node that never reaches ``active`` is never owned, is never tracked
here, and its departure can never raise ``GRAPH_NODE_DISAPPEARED``; the same holds for an app
that is not online. For those, ``lifecycle_expectation`` keeps the older behaviour: absence
still matures a below-``grace`` streak, because it is structurally the only detector that will
ever get to report them. See
``lifecycle_expectation``'s own "Absence continues the last CORROBORATED observation, it never
erases" section above for the mechanism this rests on.

**Repeated failures: what** ``occurrence_count`` **and the captured evidence answer, and
don't.** An operator asking "how many times did this node die in the last hour" reads it
off the fault record itself, not off this detector: ``GRAPH_NODE_DISAPPEARED``'s own
``occurrence_count`` starts at 1 on the first raise and increments by exactly one each
time a FAILED report reactivates a record that was CLEARED - never merely re-raised while
still CONFIRMED, and never merely HEALED. Healing (the fault manager's debounce counter
crossing ``healing_threshold`` on clean sweeps, see "Closing the loop" in the README) and
clearing are different things: ``DELETE
/api/v1/apps/graph_watchdog/faults/GRAPH_NODE_DISAPPEARED`` - the fault manager's own
``~/clear_fault`` underneath - is what acknowledges an occurrence and closes its cycle. A
still-CONFIRMED, or still-HEALED-but-unacknowledged, fault that fires FAILED again is the
SAME occurrence continuing, not a new one, so restarting the dead node fast enough to heal
it before anyone acknowledges it will not move the count.

The honest limits, read off this branch's fault-manager storage rather than assumed: the
per-fault rosbag store enforces ``fault_code`` as UNIQUE, so a fault can hold at most one
recording at a time - a later confirmation's capture replaces the earlier one on disk
rather than accumulating a history. The freeze frame is the same shape: one row per
``fault_code``, overwritten on every capture, so a fifth occurrence's captured values
overwrite the first's. What survives every occurrence by default is the count itself; a
hash-chained record of every raise/clear/heal transition also exists, but only once the
fault manager's own ``audit_log.enabled`` is turned on, which it is not by default.
Recordings and the freeze frame do not accumulate a per-occurrence history either way.

**A second death while the first is still outstanding gets no evidence of its own.** This
detector folds every currently-affected node into ONE ``GRAPH_NODE_DISAPPEARED`` record via
``AggregatedFault::emit()`` (see "The boundary with" ``lifecycle_expectation`` above): a
second node dying while the fault is still CONFIRMED is added to the description on the next
tick, but the ``ReportFault`` call that carries it lands on ``fault_storage.cpp``'s
already-CONFIRMED, still-FAILED branch - the same re-report-not-a-new-occurrence path
"Repeated failures" above describes for one node dying twice, reached here by two different
nodes sharing one code instead. ``occurrence_count`` does not move, the fault manager
publishes ``EVENT_UPDATED`` rather than ``EVENT_CONFIRMED``, and ``just_confirmed`` in
``fault_manager_node.cpp`` stays false - which is what gates ``capture_pool_``, so neither a
freeze frame nor a recording is captured for the second node. Acknowledging between the two
deaths avoids this: the DELETE route's ``~/clear_fault`` closes the first occurrence, so the
second node's next FAILED report reactivates a CLEARED record instead of updating a CONFIRMED
one - a genuine new occurrence, with its own ``occurrence_count`` and its own capture.

Two fixes were measured and rejected here, so this is not open for reconsideration without
new evidence:

- **Clear-then-raise from inside the detector does nothing.** ``AggregatedFault`` already has
  both halves available - ``emit()`` sends a PASSED-shaped ``clear_fault()`` when ``affected``
  is empty, a FAILED-shaped ``raise_fault()`` otherwise - so sending one right after the other
  on the same tick looks like a free fix. It is not: ``compute_debounce_status()``'s
  hysteresis latch holds a CONFIRMED (or HEALED) status until the debounce counter itself
  crosses the OPPOSITE threshold, and a single PASSED event only moves that counter by one
  step. Unless the fault happened to sit exactly one step short of ``healing_threshold``
  already, the clear changes nothing the status can see, and the raise that follows lands
  right back on the same already-CONFIRMED branch as before.
- **Calling the real** ``~/clear_fault`` **service instead is worse than doing nothing.**
  ``SqliteFaultStorage::clear_fault()`` runs ``DELETE FROM snapshots WHERE fault_code = ?`` -
  it deletes the per-topic readings captured for the fault it clears, keeping only the
  freeze-frame row. And ``ClearFault.srv``'s own ``skip_correlation_auto_clear`` defaults to
  false, so clearing a root cause also clears every symptom the correlation engine attributes
  to it unless the caller explicitly opts out (the gateway's own REST DELETE routes do; a
  detector reaching for this service directly would have to remember to as well). Having the
  detector call this automatically the moment a second node dies would destroy the FIRST
  node's evidence - snapshots and correlated symptoms alike - to chase evidence for the
  second, before anyone has necessarily seen the first.

A correct fix does not belong in this detector at all: it needs an operation in the fault
manager that re-confirms a CONFIRMED record - bumping ``occurrence_count``, publishing
``EVENT_CONFIRMED`` again, and re-arming ``just_confirmed`` for a fresh capture - without
routing through CLEARED and without deleting anything the first occurrence already earned.

**Test tiers.** Three tiers each prove a different layer, deliberately not overlapping,
plus the suppression chain the detector shares no code with any sibling for:

1. **Unit** (``test_node_liveness_tracker.cpp``): the pure presence/absence state machine
   - a key present but never armed is never tracked; once armed, presence alone (not
   continued arming) keeps a tracked key's miss counter at zero; a key is reported dead
   only once misses exceed ``miss_grace``; freshness ordering keeps a brand-new death out
   of a capped description's blind spot; ``prune()`` reclaims a key only once it has been
   suppressed for MORE than ``prune_ticks`` CONSECUTIVE calls, resets the streak the
   moment a veto lifts even once, and never reclaims an unsuppressed death no matter how
   long it stays dead; the cap evicts idle entries before collapsing departed ones, keeps
   the collapsed count monotone within one tracker lifetime, and - the property most at
   odds with a naive read of the sibling detector - can never actually report
   ``tracking_saturated`` at all, since every tracked key here is always either idle or
   collapsible, never both at once the way the sibling's two clocks allow.
   ``test_suppressor.cpp`` pins the ``Suppressor`` interface and the free
   ``apply_suppressors()`` helper (order-independent, null-safe, returns the dropped
   count); ``test_allowlist_suppressor.cpp`` and ``test_lifecycle_shutdown_suppressor.cpp``
   pin each suppressor's own matching rules independently of any detector.
2. **Integration** (``test_node_death_integration.cpp``): the full config contract
   (``miss_grace``, ``prune_grace`` and ``tick_interval_ms`` range checks and their floor
   interaction; malformed ``allowlist``/``suppress`` entries; ``tracked_node_cap`` range
   checks, including a value far past ``INT_MAX`` rejected rather than wrapped; an unknown
   top-level key), that a peer-aggregated app and an ``is_online: false`` app are never
   tracked, that the tracked count stays bounded and shrinks after a reclaim under
   sustained identity churn, that a clean managed-lifecycle departure is suppressed
   exactly AT the ``miss_grace`` boundary and stays reclaimed - not re-raised - past the
   retention window the plugin sizes for it, that the allowlist's id-form suppresses only
   the colliding node and not its namesake, and the ungated-clear guard's four properties:
   a stored fault stays unhealed while an unrelated node arms alone, a clear flows once
   this process instance has genuinely raised, the guard tracks DELIVERY rather than
   intent, and a reconfigure while the node is absent does not withhold a clear the
   process had already earned.
3. **E2e**, four files, each launching its own real gateway + fault_manager + demo-node
   stack: ``test/e2e/test_node_death_e2e.test.py`` (ten scenarios covering the raise/clear
   round trip, no-heal-standalone, the lifecycle-deactivate/manifest-never-online/ros2cli
   non-cases, a bare-name collision naming only the node that exited, a fast tick alone
   raising nothing, a three-cycle restart loop reaching ``occurrence_count: 3``, and a
   fault surviving a gateway restart); ``test/e2e/test_node_death_suppression_e2e.test.py``
   (five scenarios covering the allowlist, its inertness when unnamed in ``suppress``, the
   clean-shutdown/still-active contrast, opt-in suppression, and pruning without a false
   heal); and ``test/e2e/test_node_death_boundary_e2e.test.py`` (eight scenarios proving
   the seam with ``lifecycle_expectation`` directly: a node stuck inactive but never gone
   raises ``GRAPH_NODE_INACTIVE`` alone, an ARMED node killed below ``grace`` raises
   ``GRAPH_NODE_DISAPPEARED`` alone, a node killed AFTER ``GRAPH_NODE_INACTIVE`` has
   confirmed raises BOTH at once, a healthy node that is simply killed raises
   ``GRAPH_NODE_DISAPPEARED`` alone, a restart-looping required node is caught every cycle
   regardless of lifecycle maturity, a NEVER-armed node killed below ``grace`` raises
   ``GRAPH_NODE_INACTIVE`` alone instead - node_death cannot track a node the gate never
   admitted for ownership, so absence has to mature it here - a large ``miss_grace`` delays
   but does not swallow the report, and a restarted gateway's warmup window never produces a
   spurious PASSED); and ``test/e2e/test_presence_ownership_e2e.test.py`` (seven scenarios
   against the fixture whose ``GetState`` never answers, so the unread state is permanent
   rather than the race the boundary file's B6 row has to live with: a managed node the
   watcher asked and could not read, killed, raises BOTH ``GRAPH_NODE_DISAPPEARED`` and
   ``GRAPH_NODE_UNREADABLE``; the same death with no ``require_active`` entry anywhere - the
   shipped default, where nothing else is watching at all - still raises
   ``GRAPH_NODE_DISAPPEARED``, and that pair also rules out an implementation deciding
   ownership from ``require_active`` membership; the SAME node told to start answering,
   measured ``active``, then killed, IS reported too, under the identical configuration as
   the first row so the only variable is the lifecycle state; a node measured ``active`` that
   then LOSES its lifecycle services and is killed is still reported; and with an unmeasured
   managed node in the graph ``GRAPH_PARAM_DRIFT`` still names it - the one leg of the three
   that passes through the app-keyed gate at all - while the topic-keyed
   ``GRAPH_QOS_MISMATCH`` and ``GRAPH_ORPHAN`` still report too; and the one row that asserts
   an ABSENCE, where the same never-answering node is owned provisionally, then announces
   ``inactive`` over ~/transition_event, then dies: ``GRAPH_NODE_INACTIVE`` names it and
   ``GRAPH_NODE_DISAPPEARED`` never appears, which is also what rejects a detector wired to
   the permissive ``reliability_allows()``; and a crash loop shorter than the warmup, where the
   node is killed, reported, brought back for less than one re-warm and killed again - the
   second death must still be reported, and no PASSED may be emitted while it is dead).


Status
---------------
The plugin loads, ticks the graph, and shuts down cleanly. The reliability core is real
and already ticking. Five silent-fault detector classes raise through it today,
``qos_mismatch``, ``orphan``, ``param_drift``, ``lifecycle_expectation`` and
``node_death``. Two classes remain undelivered, ``GRAPH_TF_STALE`` and
``GRAPH_LATENCY_BUDGET``; they land in follow-up changes, each against its own issue.
