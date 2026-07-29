# ros2_medkit_graph_watchdog

Gateway plugin that detects silent faults in the ROS 2 graph: failures where every
node is up, nothing logs an error, and the robot is still broken.

Detectors read the graph and raise faults through a `ReportFault` service client on the
gateway node; the faults surface via FaultManager on the gateway `/faults` API.

This package carries the plugin skeleton, the central reliability gate that holds raises
until the graph has quiesced, and the first detector, `qos_mismatch`. The remaining
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

