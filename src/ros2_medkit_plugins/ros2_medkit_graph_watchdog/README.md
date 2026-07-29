# ros2_medkit_graph_watchdog

Gateway plugin that detects silent faults in the ROS 2 graph: failures where every
node is up, nothing logs an error, and the robot is still broken.

Detectors read the graph and raise faults through a `ReportFault` service client on the
gateway node; the faults surface via FaultManager on the gateway `/faults` API.

This package carries the plugin skeleton, the central reliability gate that holds raises
until the graph has quiesced, and two detectors, `qos_mismatch` and `orphan`. The remaining
silent-fault classes land in follow-up changes, each against its own issue; their fault
codes are already reserved in the frozen `GRAPH_*` namespace (see "Fault codes").

## Build

With the gateway built and sourced:

    colcon build --packages-select ros2_medkit_graph_watchdog
    colcon test --packages-select ros2_medkit_graph_watchdog && colcon test-result --verbose

## Load into the gateway

    ros2 run ros2_medkit_gateway gateway_node --ros-args \
      -p plugins:="[graph_watchdog]" \
      -p "plugins.graph_watchdog.path:=$(ros2 pkg prefix ros2_medkit_graph_watchdog)/lib/ros2_medkit_graph_watchdog/libros2_medkit_graph_watchdog.so"

## Configuration (`plugins.graph_watchdog.<key>`)

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `tick_interval_ms` | int | `1000` | Detector tick cadence. |
| `warmup_cycles` | int | `5` | An entity must be continuously present for this many ticks before it arms. A mid-run restart re-warms it. Enforced centrally, see "Reliability (bringup-quiesce)" below. |
| `prune_grace` | int | `60` | Default injected into every detector's own config before `configure()`; a per-detector `detectors.<id>.prune_grace` overrides it. Used by detectors that keep per-key bookkeeping. |
| `detectors.<id>.mode` | string \| bool | `raise` | `raise` = push faults; `advisory` = observe, do not push; `off` = disabled. Bare `off`/`no` (YAML booleans) also disable; bare `on`/`yes` mean `raise`. |
| `detectors.<id>.<field>` | any | - | Per-detector thresholds, passed to that detector's `configure()`. |

**Nested delivery.** Keys are written dotted in YAML or `--ros-args`, and the gateway
delivers them to the plugin as a nested object. A bare `off` is a YAML 1.1 boolean, so
`mode: off` arrives as `false`, not the string `"off"`; both forms disable the detector.

**Unknown keys are reported.** Anything under `detectors.<id>` that the detector does not
read produces one startup warning naming the key and listing the ones that exist, so a
typo like `allow_list:` for `allowlist:` cannot silently do nothing. A typo in the detector
id itself is warned about the same way, with the registered ids listed.

### `orphan` keys

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string | `raise` | As above. |
| `grace` | int | `10` | Consecutive sweeps a candidate pair must hold before it is reported. A staged bringup or a restart makes one side missing for a short time, which looks exactly like a typo until the other side appears. |
| `max_edit_distance` | int | `1` | How many character edits the LEAF (the part after the last `/`) may differ by and still count as the same intended name. Values outside 1..8 keep the default: past a few edits a "near miss" starts matching unrelated topics. |
| `namespace_edit_distance` | int | `0` | The same budget for the namespace, spent separately. `0` means the namespace must match to the character, which is what makes `/robot1/scan` and `/robot2/scan` two robots rather than a typo. Raise it to catch a misspelled namespace, and read the warning under the table first. Values outside 0..8 keep the default. |
| `allowlist` | string[] | `[]` | Topic names never reported. Exact match only, never a prefix, so allowlisting `/r1/scan` does not also silence `/r2/scan`. |

**What this detector will not catch, on purpose.** A pair whose names match once every run of
digits is collapsed is treated as an enumeration of sensors, not a typo: `/lidar_1` next to
`/lidar_2`, each one-sided, is an ordinary state on a multi-sensor robot and reporting it would
be a false alarm on every such machine. Collapsing runs rather than comparing character
positions is what also covers `/lidar_9` next to `/lidar_10`. The cost is that a typo which
*is* a digit stops being reported. An appended digit is not that case: `/scan1` collapses to
`/scan#`, which is not `/scan`, so it is still reported.

**A misspelled namespace is opt-in, and it can be expensive.** `/robott/scan` against
`/robot/scan` is one edit, but in the namespace, so by default it is not reported. Setting
`namespace_edit_distance: 1` reports it. Check the naming scheme before doing that: on a fleet
named with letters, `/amr_a` and `/amr_b` are also one edit apart, and every robot in the
fleet will be reported as a typo. A numbered fleet is safe, since `/robot1` against `/robot2`
differs only in its numeric field and the rule above already spares it.

**What it cannot catch at all.** A namespace added or dropped by mistake, `/scan` against
`/robot/scan`, is six edits apart, so no budget that is still specific will reach it. Do not
rely on this detector for that class.

### `qos_mismatch` keys

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string | `raise` | As above. |
| `grace` | int | `3` | Consecutive ticks a topic must stay affected before it is reported. Endpoint discovery is not atomic, so a subscriber can be visible before a publisher's QoS is, which reads as starvation for a tick or two. |
| `allowlist` | string[] | `[]` | Subscriber FQNs never reported. Exact match only. |


## Where GRAPH_* faults hang

Every `GRAPH_*` fault is raised with `source_id: graph_watchdog`. That is an App the
plugin publishes itself, from its own `IntrospectionProvider`, marked `external: true`.
So:

- `GET /api/v1/apps/graph_watchdog/faults` lists them.
- `GET /api/v1/components/<host>/faults` lists them too, because the App is attached to
  the host Component and a Component resolves its fault scope through its child apps.

It has to be an entity the plugin owns. Scoping these faults to the host Component
directly - the obvious choice - makes them reachable from no endpoint at all:
`collect_component_app_fqns` only puts a Component's own id in the scope set when
`external` is true, and a runtime host Component built by `HostInfoProvider` never sets
it. There is no server-level `/faults/{code}` route either, so the flat `/faults` list
was the only place these faults existed.

Hanging each fault on the FQN of the node it is ABOUT does not work either: fault
identity in the store is `fault_code` alone (`fault_code TEXT PRIMARY KEY`) while
`reporting_sources` accumulates into a set on that one record, and `fault_in_source_scope`
requires EVERY source to be in scope - so two dead nodes would hide
`GRAPH_NODE_DISAPPEARED` from both of their `/apps/<fqn>/faults` pages. One owned entity
per code keeps exactly one reporting source per record; the affected nodes are named in
the description.

### Detectors

#### `qos_mismatch` (GRAPH_QOS_MISMATCH)

Watches every topic's publisher/subscriber QoS pairs and raises `GRAPH_QOS_MISMATCH`
when a pair is RxO-incompatible on reliability, durability, liveliness kind, deadline, or
liveliness lease duration - the policies that silently starve a subscriber (no data ever
arrives, no error surfaces anywhere).

**Two levels behind one fault code.** A subscriber incompatible with EVERY publisher on
its topic receives nothing at all and raises the fault at `SEVERITY_ERROR`. A subscriber
incompatible with *some* publisher but not all is reported at `SEVERITY_WARN`: DDS refuses
that one pair, so that producer's data is discarded while the topic still looks alive and
nothing anywhere reports it. That is a silent fault, not noise - an RxO-incompatible pair
is a match DDS has already refused, not a heuristic. Since one fault code carries one
severity, the emitted fault takes the worst finding in the sweep.

The partial case is easy to hit: `/diagnostics` with several publishers, one of them
hand-rolled `BEST_EFFORT`, and a `RELIABLE` aggregator - that publisher's diagnostics are
dropped forever while everyone else's keep arriving.

**The fault names the starved subscriber.** Each reason lists the node FQNs that hit it,
e.g. `/tf: reliability: publisher BEST_EFFORT vs subscriber RELIABLE (receives nothing:
/planner_server, /rviz)`, so finding the affected node does not need an ssh session and
`ros2 topic info -v`.

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string \| bool | `raise` | `raise` / `advisory` / `off` (framework seam). |
| `grace` | int | `3` | Consecutive ticks a topic must stay affected before it is reported. |
| `allowlist` | string[] | `[]` | Topic names never reported. Exact match. |

`grace` exists because a single sweep used to raise, and the shipped fault_manager config
carries `confirmation_threshold: -1` - so one sweep became a CONFIRMED ERROR immediately
and, with `snapshots.rosbag.enabled`, pulled a black-box capture with it. Endpoint
discovery is not atomic, so during bringup a subscriber can be visible before a
publisher's QoS is, which reads exactly like starvation for a tick or two. A real QoS
mismatch is static and still there N ticks later, so this costs almost no detection
latency.

`allowlist` exists because an operator's own tooling is not a robot fault. rviz2 builds
every display's subscription from `rclcpp::QoS(5)` - RELIABLE - and does not negotiate
down to what a `BEST_EFFORT` sensor publisher offers, so opening an Image or PointCloud2
display on a driver topic reads as starvation. (`ros2 topic echo` and `ros2 bag record`
are not affected: both inspect the publisher endpoints and fall back to `BEST_EFFORT`.)

```yaml
plugins:
  graph_watchdog:
    detectors:
      qos_mismatch:
        grace: 3
        allowlist: ["/camera/image_raw"]   # an rviz2 display lives here
```

**RxO rule (Request <= Offered).** Publisher = offered, subscriber = requested; a pair
is incompatible only in the one strict direction per policy:

| Policy | Incompatible when |
|--------|--------------------|
| Reliability | publisher `BEST_EFFORT`, subscriber `RELIABLE` |
| Durability | publisher `VOLATILE`, subscriber `TRANSIENT_LOCAL` |
| Liveliness kind | publisher `AUTOMATIC`, subscriber `MANUAL_BY_TOPIC` |
| Deadline | offered deadline > requested deadline (an unset/infinite publisher deadline is unbounded, so it fails any finite subscriber request) |
| Liveliness lease | offered lease duration > requested lease duration (same unset/infinite handling) |

The reverse direction (e.g. a `RELIABLE` publisher feeding a `BEST_EFFORT` subscriber)
is compatible by design - the subscriber asked for no more than it is offered - even
though the QoS profiles differ. For deadline and lease, an unspecified subscriber value
means the subscriber does not constrain that policy, so it is always compatible. History
and depth are not RxO-compatibility dimensions and are deliberately not checked.

**Aggregated fault, not per-topic**, via the shared `AggregatedFault` helper
(`aggregated_fault.hpp`): the fault_manager
identifies a fault by `fault_code` alone, so one `GRAPH_QOS_MISMATCH` per mismatched
topic would collide into a single record under the shared code. One graph-level fault
enumerates every currently-mismatched topic; it clears (`EVENT_PASSED`) on every tick
where nothing is mismatched, subject to the same `healing_enabled` requirement
described in "Closing the loop" above to actually reach HEALED.

**Exhaustive, not budgeted.** `qos_mismatch` reads no external service - `get_topic_names_and_types()` and
`get_publishers_info_by_topic()` / `get_subscriptions_info_by_topic()` are local
graph-cache queries, so every topic is checked every tick with no coverage-latency
knob to configure. `/rosout` and `/parameter_events` are skipped (ROS 2 system topics
with their own QoS conventions, not useful signal here).

**Test tiers** (deliberately non-overlapping proof at each layer):

1. **Unit** (`test/test_qos_policy.cpp`): pure `qos_incompatibility()` logic against
   hand-built `rmw_qos_profile_t` values - the RxO trio, the reverse-direction
   compatible case, identical-profile compatibility, and the never-raise guarantee for
   `SYSTEM_DEFAULT`/`UNKNOWN` enums a live endpoint never actually reports.
2. **Integration** (`test/test_qos_mismatch_integration.cpp`): a real publisher and
   subscriber over real DDS, read through the actual
   `get_publishers_info_by_topic()`/`get_subscriptions_info_by_topic()` API against a
   fake `ReportFault` service - proving the detector reads the RESOLVED live profile
   correctly, not just that the comparison function is correct. A second, concurrent
   RxO-compatible-but-different-QoS topic pair proves the detector does not over-fire
   on "QoS differs somewhere in the graph".
3. **E2e** (`test/e2e/test_qos_e2e.test.py`): the Acceptance gate - a
   real gateway process with the plugin `.so` loaded, a real fault_manager, and the
   operator-visible `GET /api/v1/faults` surface, proving the whole raise/clear story
   reaches a SOVD fault through the real tick timer against a reliability mismatch
   (raise + clear round trip), a deadline mismatch, and a liveliness-lease-duration
   mismatch (the description names each policy).

#### `orphan` (GRAPH_ORPHAN)

Watches every topic's publisher/subscriber counts and raises `GRAPH_ORPHAN` when a
one-sided endpoint (a topic with publishers but no subscribers, or vice versa) has a
same-type, same-namespace near-miss counterpart carrying the complementary side - the
signature of a remap / topic-name typo (e.g. a node publishes on `/scann` while every
would-be subscriber listens on `/scan`; neither side ever notices, and no `/diagnostics`
or `/rosout` signal fires). "Near-miss" is a small Levenshtein edit distance, default `1`
(config-overridable via `max_edit_distance`), which catches single-character typos. Numeric
siblings are spared by a separate rule, described under "Digit guard" below, because raising
the distance is not what separates them from a typo.

**Names both sides, suggests neither.** A finding's `reason` names BOTH the pub-only and
sub-only topic; the detector deliberately gives no directional rename recommendation -
it cannot know which of the two names is the canonical one, so the operator decides. Each
unordered near-miss pair is reported exactly once, not once per side.

**A candidate, not a verdict - and it has to hold.** The pub-only/sub-only shape is not
unique to a typo. Any node that publishes one name and subscribes a near-identical one of
the same type - a CAN/serial bridge on `/can_rx` + `/can_tx`, any relay that republishes a
one-character variant - produces exactly this shape the moment its peer exits, whether that
is a crash, a supervisor restart, or staged bringup. So the fault is worded as a candidate
("either a remap typo or a departed counterpart"), and a pair must hold for `grace`
consecutive ticks before it is reported at all.

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string \| bool | `raise` | `raise` / `advisory` / `off` (framework seam). |
| `max_edit_distance` | int | `1` | Levenshtein distance that still counts as a near miss (1..8). |
| `grace` | int | `10` | Consecutive ticks a candidate pair must hold before it is reported. |
| `namespace_edit_distance` | int | `0` | Edit budget for the namespace, spent separately from the leaf (0..8). |
| `allowlist` | string[] | `[]` | Topic names never reported, matched exactly and never by prefix. |

`grace` costs no meaningful detection latency, because a real name typo is static and still
there N ticks later. The reliability gate does not cover this case: `graph_watchdog` is the
plugin's own entity, not the app being restarted, so `allows_raise()` is already armed and
gives a restarting node no settle window. At the 1s default tick the default outlives a
supervisor restart; it does NOT outlive a slow staged bringup, which is why the wording
stays a candidate rather than an instruction to rename something.

**Same-namespace guard.** The leaf and the namespace get separate edit budgets, and the
namespace budget defaults to `0`, so by default both topics must share the same parent
namespace (everything up to the final `/`). `/scann` vs `/scan` (same namespace, leaf edit
distance 1) matches; `/robot1/scan` vs `/robot2/scan` (different namespaces, also edit
distance 1) does not - that is a fleet layout, not a typo.

`namespace_edit_distance` opts out of that. At `1`, `/robott/scan` vs `/robot/scan` is
reported - a real remap mistake that is invisible by default. The budgets stay independent, so
spending the namespace one does not buy extra leaf edits, and the numeric-field guard still applies
to the whole name, which keeps a numbered fleet quiet even here. What it does NOT keep quiet is a
fleet named with letters: `/amr_a` and `/amr_b` are one edit apart, so every robot pairs with
its neighbour. Set this only if the namespaces on the robot are not near-misses of each
other.

**Numeric-field guard.** A pair whose names are equal once every run of digits collapses to a
single placeholder is an enumeration, not a typo:
`/lidar_1` next to `/lidar_2`, each one-sided, is the ordinary state of a multi-sensor
machine, and reporting it would fire on every such machine. Collapsing runs rather than
comparing character positions also covers `/lidar_9` vs `/lidar_10`, where the index changes
length. This is checked independently of `max_edit_distance`, so lowering the distance does not
help and raising it does not hurt here. The price is that a typo which is itself a digit stops being reported. That is a
deliberate trade: a fleet-wide false alarm is worse than one missed class of typo.

**What this detector cannot see at all** is a namespace added or dropped by accident.
`/scan` against `/robot/scan` is 6 edits apart, far beyond any distance that would still be
specific, so do not rely on this detector for that class.

**When it is still wrong, allowlist the topic.** Any node that publishes one name and
subscribes a near-identical one of the same type produces the orphan shape permanently once
its peer is gone for good, and no threshold separates that from a typo. Name either side of
the pair; matching is exact, so allowlisting `/r1/scan` does not also silence `/r2/scan`.

```yaml
plugins:
  graph_watchdog:
    detectors:
      orphan:
        grace: 10
        max_edit_distance: 1
        namespace_edit_distance: 0   # 1 also catches /robto/scan vs /robot/scan; see above
        allowlist: ["/can_rx"]       # a bridge whose peer is intentionally absent
```

**Aggregated fault, not per-pair**, via the same `AggregatedFault` helper `qos_mismatch`
uses - the fault_manager identifies a fault by `fault_code` alone, so one `GRAPH_ORPHAN`
per near-miss pair would collide into a single record under the shared code. One
graph-level fault enumerates every currently-orphaned pair, keyed by the canonical
`<publisher_topic> <-> <subscriber_topic>` string; it clears (`EVENT_PASSED`) on every
tick where nothing is orphaned, subject to the same `healing_enabled` requirement
described in "Closing the loop" above to actually reach HEALED.

**Exhaustive, not budgeted**, the same rationale as `qos_mismatch`: no external service is
read - `get_topic_names_and_types()` and `count_publishers()`/`count_subscribers()` are
local graph-cache queries, so every topic is checked every tick. `/rosout` and
`/parameter_events` are skipped.

**Test tiers:**

1. **Unit** (`test/test_orphan_policy.cpp`): pure `find_orphans()` logic against
   hand-built `TopicEndpointCounts` - the typo pair reported once naming both sides, a
   lone topic with no counterpart, a near-miss pair of DIFFERENT types, a fully-connected
   topic, the default-vs-opt-in edit-distance boundary (`/scan` vs `/scan_1`), the
   same-namespace guard (`/robot1/scan` vs `/robot2/scan`), and the numeric-field guard - what
   it spares (`/lidar_1` vs `/lidar_2`, and `/lidar_9` vs `/lidar_10`), what it knowingly
   drops (a typo in a digit), and the appended digit it still reports (`/scan1` vs `/scan`),
   with a letter difference of the same shape still reported to keep the guard honest. The
   namespace budget is pinned on both sides of its default and at the bound above it, along
   with the two properties that make it safe to ship: it does not widen the leaf budget, and
   it does not defeat the digit guard.
2. **Integration** (`test/test_orphan_integration.cpp`): a real publisher on a topic-name
   typo and a real subscriber on the intended target, both `sensor_msgs/msg/LaserScan`
   over real DDS, read through the actual `get_topic_names_and_types()` +
   `count_publishers()`/`count_subscribers()` API against a fake `ReportFault` service. A
   concurrent lone pub-only topic proves the detector does not over-fire on "any one-sided
   topic". Clearing destroys the typo publisher - adding a publisher on the target topic
   instead would leave the typo topic permanently orphaned. A third case allowlists a topic
   and drives 12 ticks past `grace` to prove it is never reported.
3. **E2e** (`test/e2e/test_orphan_e2e.test.py`): the acceptance gate - a
   real gateway process with the plugin `.so` loaded, a real fault_manager, and the
   operator-visible `GET /api/v1/faults` surface, proving the whole raise/clear story
   reaches a SOVD fault through the real tick timer. It runs a second, allowlisted near-miss
   pair alongside the reported one, plus a namespace-typo pair that is reported ONLY because
   the launch raises `namespace_edit_distance`. Together they are the only place a string array
   and an integer are proven to survive the YAML -> ROS parameter -> nested-config path into
   `configure()`; both C++ tiers hand `configure()` a JSON object directly.


## Reliability (bringup-quiesce)

Silent-fault detectors are prone to bringup noise: a node joining the graph, a
topic still being discovered, or a lifecycle node mid-transition all look like
"something is wrong" for the first few ticks. The plugin enforces
bringup-quiesce centrally so no individual detector has to reimplement it.

- **Central gate.** `ctx.raise_fault(...)` runs every raise through a
  `ReliabilityGate` before the fault client sends anything. A detector that
  raises about a `source_id` that is still warming up, or a managed lifecycle
  node that is not `active`, is silently suppressed - the detector's own logic
  never has to know. This is transparent to detectors: nothing changes
  in how they call `raise_fault`/`clear_fault`. **`clear_fault` is never
  gated** - a fault can always be cleared regardless of warmup or lifecycle
  state.
- **Warmup.** `warmup_cycles` is live: an entity arms only once it has been
  continuously present for `warmup_cycles` ticks. An entity that vanishes for
  more than a short grace (a real mid-run restart) and reappears re-warms from
  scratch; a transient one-tick discovery gap (DDS churn) is tolerated and does
  not re-warm, so recurring churn cannot suppress an entity forever. Unknown
  `source_id`s (e.g. a topic, not a tracked app) fall back to a global bringup
  grace period keyed off the first tick the graph was seen non-empty; that grace
  re-arms whenever the graph empties out, so a full-stack restart is covered too.
- **Lifecycle.** Managed ROS 2 lifecycle nodes that are not in the `active`
  state are suppressed. State is seeded via a `GetState` service call when a
  lifecycle node is first discovered, then kept fresh via its
  `~/transition_event` topic. A node still cached non-active shortly after
  discovery is briefly re-seeded, so an `active` transition lost during the
  subscription's endpoint-matching window self-heals instead of suppressing the
  node forever. Seeds are bounded per tick so a batch bringup cannot stall the
  tick loop. Non-managed nodes are never gated.
- **Clock validity.** `ctx.clock->time_is_valid()` flags a paused or absent
  `/clock` (e.g. a bag pauses or a sim crashes under `use_sim_time`). This is
  **detector-consulted, not centrally enforced**: a time-based detector
  (age/staleness/grace-period math) should check `time_is_valid()` and skip
  its own math for the tick rather than raise a false positive on a frozen
  clock. Detectors that don't do time math can ignore it.

**Status endpoint.** `GET /api/v1/x-medkit-watchdog` returns the reliability
core's state so an operator can tell "armed and quiet" (nothing wrong) apart
from "still warming up" (nothing raised yet because the gate hasn't armed)
apart from a dead watchdog (503 if the gate was never initialized):

```json
{
  "x-medkit-watchdog": {
    "schema_version": "1.0.0",
    "warmup_cycles": 5,
    "global_state": "armed",
    "entities": [
      {
        "id": "/nav/planner",
        "first_seen_tick": 3,
        "armed": true,
        "state": "armed",
        "lifecycle": "active"
      }
    ]
  }
}
```

`state` is `"armed"` only when both warmup is done and lifecycle (if managed)
is `active`; otherwise it is `"warming_up"`. `lifecycle` is the raw lifecycle
state label, or `null` for entities with no tracked lifecycle state.

**What detectors see.** Nothing changes in how a detector raises or
clears a fault - the gate is applied transparently inside `ctx.raise_fault()`
itself. The two things a detector opts into explicitly are
`ctx.clock->time_is_valid()` (skip age/grace math on a stalled clock) and
`tf_static_qos()` (a `transient_local` QoS helper for subscribing to
`/tf_static`, whose publishers latch so a late subscriber must match the
durability to receive them).

## Adding a detector 

1. Create `src/detectors/<id>.cpp`.
2. Subclass `Detector`; implement `id()` and `tick(DetectorContext&)`. Read the
   graph via `ctx.gateway_node`; create any subscription on `ctx.gateway_node`
   while holding `*ctx.node_mutex`. Raise/clear faults with
   `ctx.raise_fault(code, severity, desc, source_id)` /
   `ctx.clear_fault(code, source_id)` using a code from `graph_fault_codes.hpp`
   and a non-empty `source_id` (the affected node/entity).
3. Add `REGISTER_DETECTOR(YourDetector, "<id>")` at file scope.

A detector's source needs no CMake or registry edit (sources are globbed);
adding a per-detector unit test is the one shared touch - it adds an
`ament_add_gtest` entry to the test block.

## Fault codes

Frozen in `include/ros2_medkit_graph_watchdog/graph_fault_codes.hpp`:
`GRAPH_QOS_MISMATCH`, `GRAPH_ORPHAN`, `GRAPH_NODE_DISAPPEARED`, `GRAPH_TF_STALE`,
`GRAPH_PARAM_DRIFT`, `GRAPH_LATENCY_BUDGET`, plus one extension of the frozen
namespace for a new capability beyond the original six: `GRAPH_NODE_INACTIVE`.

