# ros2_medkit_graph_watchdog

Gateway plugin that detects silent faults in the ROS 2 graph: failures where every
node is up, nothing logs an error, and the robot is still broken.

Detectors read the graph and raise faults through a `ReportFault` service client on the
gateway node; the faults surface via FaultManager on the gateway `/faults` API. That
client, like the lifecycle subscriptions below, lives in the plugin's own callback group
and is driven by the plugin's own executor from the tick thread, never by the gateway's
ROS executor.

This package carries the plugin skeleton, the central reliability gate that holds raises
until the graph has quiesced, and three detectors, `qos_mismatch`, `orphan` and
`param_drift`. The remaining
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

### `param_drift` keys

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string \| bool | `raise` | As above. A bare `off` arrives as a YAML boolean; both forms disable the detector. |
| `baseline` | bool | `true` | Self-capture the value of every parameter not listed in `expect`, then flag later changes to it. `false` checks only what `expect` pins, and skips reading the full parameter list. |
| `expect.<param>` | any | - | Absolute expected value for `<param>`, checked on every node that has it. This is the only rule that catches a value that was wrong from the first tick, which self-capture cannot see. A node that does not declare `<param>` answers NOT_FOUND, which is not a drift - so a misspelt name checks nothing at all. Once the sweep has covered every armed app - each one read, or given up on after repeated failed reads - and at least one of them answered, a pin that no app declares is warned about once, naming it. |
| `ignore` | string[] | `[]` | Parameter-name globs never flagged. Applies to the self-captured set only, so it cannot silence an `expect` pin. A pattern of nothing but `*` matches every name and suppresses self-capture wholesale, which is indistinguishable at runtime from finding nothing; that combination warns at startup. |
| `max_reads_per_tick` | int | `8` | Parameter-service ROUND TRIPS this detector may spend per tick, round-robin over nodes. Round trips, not calls: one call is several requests at the watched node, and the node pays for each of them. A `baseline` visit is charged 2 (a list, then one batched get) whatever the node's parameter count; each `expect` pin is charged 3 (a list, a get and a descriptor read), or 2 on a node that does not declare it - see the table under "Coverage latency" for what each of those really costs the node. So a self-capture visit spends 2 of the budget and pinning N parameters spends up to 3N. It is an AMORTISED rate, not a per-tick cap: there is no pacing INSIDE a visit, so a node's whole read set is issued back to back and the reader waits out the debt afterwards. Four pins against a budget of 8 fire 12 round trips inside one tick period and then idle for the rest of the next one; what the knob bounds is the sustained rate, not the burst. Accepted range 1..100000; anything else keeps the default and warns. |
| `prune_grace` | int | `60` | Consecutive sweeps an app may be missing before its finding is dropped: the finding survives `prune_grace` absences and goes on the next one. Injected for every detector at plugin scope and overridable per detector; a value outside 0..3600 is rejected with a warning and this detector falls back to 2, not to the plugin default. The captured BASELINE has a shorter horizon of its own, and that one is NOT configurable - it is the compile-time `kBaselineForgetGrace`: the baseline is dropped on the third consecutive missed sweep, the two the reliability gate absorbs plus one, so an ordinary dropped discovery poll cannot re-capture a node on its already-drifted value. Setting `prune_grace` below 2 pulls that forward onto the sweep the finding itself is dropped on, so the two then go together. |

### `qos_mismatch` keys

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string | `raise` | As above. |
| `grace` | int | `3` | Consecutive ticks a topic must stay affected before it is reported. Endpoint discovery is not atomic, so a subscriber can be visible before a publisher's QoS is, which reads as starvation for a tick or two. |
| `allowlist` | string[] | `[]` | Subscriber FQNs never reported. Exact match only. |


## Closing the loop: healing config is required

Every detector here reports level-triggered: it re-raises FAILED while the condition holds
and emits PASSED on every clean sweep. That is what lets the fault_manager's debounce
counter walk from CONFIRMED back to HEALED. The fault_manager only counts PASSED events
toward healing when `healing_enabled` is true for that fault, and it defaults to `false`.
Without it, a `GRAPH_*` fault that has genuinely cleared stays CONFIRMED forever. Enable
healing globally on the fault_manager node:

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p healing_enabled:=true \
  -p healing_threshold:=1  # 2 consecutive clean sweeps to heal (confirmation_threshold defaults to -1)
```

or scope it to this plugin only, by longest-prefix match on `source_id`, which is always
`graph_watchdog`:

```yaml
# entity_thresholds.yaml, loaded via the fault_manager's entity_thresholds.config_file param
graph_watchdog:
  healing_enabled: true
  healing_threshold: 1  # 2 consecutive clean sweeps to heal (confirmation_threshold defaults to -1)
```

Healing time does not depend on how long the fault was active. The debounce counter is
clamped to `[confirmation_threshold, healing_threshold]` on every event, so a FAILED event
can push it down to `confirmation_threshold` and no further; a long-running fault does not
dig a hole that later clean sweeps have to climb out of. After the fix, each clean sweep
steps the counter up by one, so healing takes at most
`healing_threshold - confirmation_threshold` consecutive clean sweeps - exactly that many once
the counter has walked down to the floor, fewer if the fault was raised only briefly.

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


#### `param_drift` (GRAPH_PARAM_DRIFT)

Watches ROS 2 node parameters and raises `GRAPH_PARAM_DRIFT` when a live value diverges
from its reference. It needs no configuration to be useful: it captures each parameter's
value once the node arms and flags later runtime changes. Pinning a parameter to an
absolute value with `expect` additionally catches the case self-capture cannot see, a value
that was already wrong when the node started.

**A changed parameter is not the same as a broken robot.** The detector reports that a
value moved and does not grade the consequence. Whether the new value matters is a question
about the machine, not about the graph, so it raises at WARN and leaves the judgement to
whoever reads the fault.

**Aggregated fault, not per-node.** All drift in the graph is one graph-level
`GRAPH_PARAM_DRIFT`, not a fault per owning node, for the same reason as the other
detectors: the fault_manager identifies a fault by `fault_code` alone, so per-node faults
would collide into a single record. The description enumerates every currently-drifted
`(node, parameter)` pair, up to a cap of 480 characters - past that the text is truncated. Each
app's own contribution is trimmed to 150 characters before that, so one node drifting on many
parameters cannot fill the 480 by itself, and the entries are ordered `expect` violations first,
then self-captured drift, with newly detected apps ahead of the ones already reported inside each
group, so the cap cannot hide the two things worth reading. It clears
(`EVENT_PASSED`) on every sweep where nothing drifts, so one node reverting is not enough while
another still drifts, subject to the same `healing_enabled` requirement described in "Closing the
loop" above. A clear is withheld while any app still has no usable read - one the sweep has not
reached, one whose reads keep failing, or one the reliability gate has not armed - because health
that was never measured is not reported as health. That withholding is bounded in both directions:
an app whose reads keep failing stops holding the clear back once it has been given up on (see
below), and a gate-denied app stops once the frozen hold has elapsed. An app whose reads keep
failing is named in the log once ten consecutive attempts have failed; one the sweep has not yet
attempted is not named, because nothing is known about it yet - instead, a clear that has been
withheld for a whole minute of ticks is logged once, saying how many apps are behind it and naming
one, so an operator watching a fault refuse to heal can tell a graph the sweep cannot cover from a
detector that is simply working.

```yaml
plugins:
  graph_watchdog:
    detectors:
      param_drift:
        expect:
          use_sim_time: false     # flag any node running on sim time in production
        ignore: ["*_stamp"]       # only filters self-captured params, so keep baseline enabled
```

`baseline` and `expect` combine: an empty `expect` with `baseline: true` is pure
self-capture; a sparse `expect` with `baseline: true` is pins plus self-capture;
`baseline: false` checks only the pinned parameters and moves far less data, because it never
reads the full parameter list. It is not cheaper against the budget, though: a self-capture visit
is charged 2 round trips no matter how many parameters the node has, while a single pin the node
declares is charged 3.

**Coverage latency.** The sweep is budgeted by service ROUND TRIPS at the watched nodes - not by
node count, and not by transport calls either. `max_reads_per_tick` bounds how many round trips
happen per tick period, and the reader paces itself to that rate rather than reading back to back.
A node's whole read set is read in one visit, and the visit is then charged in round trips rather
than in calls. What it is charged is not always what the node paid:

| Visit | Round trips charged | What the node actually pays |
|-------|---------------------|-----------------------------|
| `baseline: true` | 2 | 2 - one list, one batched get, whatever the parameter count. 4 on the very first contact; see below. |
| one `expect` pin the node declares | 3 | 3 - a name-scoped list, a get, and a descriptor read the transport makes unconditionally on the success path |
| one `expect` pin the node does not declare | 2 | 1 - the name-scoped list comes back empty and the transport stops there. The charge is the higher of the two NOT_FOUND paths: a name the list DOES find and the get then returns no value for costs 2, and the error code cannot tell them apart, so the bound overstates rather than understates. |
| one `expect` pin the node declares but has not set | 3 | 3 - not a NOT_FOUND at all. rclcpp answers an unset parameter with a `PARAMETER_NOT_SET` value rather than an absent one, so the read succeeds, pays for the descriptor like any other resolved pin, and the pin fires: a node holding no value at all does not satisfy a pin that names one. It is reported as `got=<unset>`, not as `got=null`, because a NaN-valued parameter reads as `null` too and the two are different faults. |

The right-hand column is measured rather than asserted: `test/e2e/test_param_roundtrip_e2e.test.py`
drives the real transport against a node that answers the parameter services by hand and counts
every request it is asked to serve.

Visiting every node once therefore takes at least about

- `ceil(2 * node_count / max_reads_per_tick)` tick periods with `baseline: true`, and
- with `baseline: false`, `ceil(3 * pin_count * node_count / max_reads_per_tick)` where every node
  declares every pin, down to `ceil(2 * pin_count * node_count / max_reads_per_tick)` where none of
  them does,

and a drift on a node not yet visited is invisible until then. At the defaults (budget 8, 1 s
tick) a 20-node graph is about 5 ticks in self-capture mode; the same graph with four pins every
node declares is about 30, and about 20 if no node declares any of them. Treat that as a floor
rather than a bound: a read that overruns its slot is not compensated, so a few slow nodes stretch
the rotation further.

In `baseline: true` mode the very FIRST read of a given node costs two round trips more than the
table charges, because `list_parameters` primes the transport's defaults cache before doing its own
read. That excess is deliberately not charged: the cache has no expiry, so the pair is paid once
per node for the life of the gateway, and charging it on every read would throttle the whole run to
cover one rotation. The practical effect is that the first rotation after startup puts up to twice
the budgeted load on the nodes in it, and every rotation after that stays within budget.
`baseline: false` has no cold surcharge at all: `get_parameter` never primes that cache, so the
first pinned read of a node costs the same three round trips as every later one.

Coverage latency governs repairs as well as detections: a node that reverted but has not been read
again yet keeps its stale entry, so the aggregated fault can stay raised for up to one rotation
after the underlying fix. Raise `max_reads_per_tick`, or lower the tick interval, to shorten that
window on a big graph.

There is one stale entry those two knobs do NOT shorten. A node that is present but has left the
reliability gate's `active` state is not read at all, so its finding is frozen rather than
re-derived - which is right for a pause, and wrong for good. `kFrozenHoldTicks`, a compile-time 60
consecutive gate-denied ticks (a minute at the default tick period), bounds it: past that the
finding is released and the app also stops holding back a clear, and both come back from a fresh
read if it ever arms again. Because that horizon is counted in TICKS and not in rotations, raising
`max_reads_per_tick` does nothing to it and shortening `tick_interval_ms` shortens it only
incidentally, by making a tick shorter.

What is NOT derived from the rotation is anything the detector says about an app that will not
answer. Both the log line and the point at which it stops holding back a clear rest primarily on
completed, FAILED read attempts and not on elapsed ticks, so an app that is merely waiting its turn
is never announced as silent: the log names an app once ten consecutive reads of it have failed,
and the detector stops letting it hold back a clear only once more than sixty have. Each of those
carries an additional FLOOR in ticks, of the same size, which the app must also have spent
unread - an attempt is not a duration, and without the floor `max_reads_per_tick: 100000` spends
sixty-one attempts in a couple of milliseconds and writes off a node that has merely not finished
being discovered. The floor is a conjunction, so it can only ever delay the two, never bring them
forward. An armed app the sweep has never ATTEMPTED blocks the clear for as long as that stays
true, however long the rotation takes; a cached read is not a substitute, so an app whose reads
start failing blocks the clear again from the first failure.

The reads run on a thread the plugin owns, never on the gateway's ROS executor, so a node
that stops answering stalls only this detector's sweep and not the gateway's request
handling.

**Getting back to clean after a deliberate change.** The detector cannot tell an
intentional runtime change from a misconfiguration; it only knows the value moved away from
what it last saw. Reverting the parameter always works: the aggregate clears on the first sweep
where nothing drifts and every app has either been read, been given up on after repeated failed
reads, or exhausted the frozen hold with the gate still shut.

Restarting the node only works if the restart is slow enough to be SEEN. A captured baseline
is dropped on the third consecutive sweep its app is absent from (sooner only if
`prune_grace` is set below 2, see its row above), and only then does the node's return start from
a fresh capture. That window is deliberate. DDS discovery drops a
node from a single poll routinely, with no restart involved, so a one-sweep window would
re-baseline that node on its already-drifted value and heal a true positive that could then
never fire again, because the new baseline IS the wrong value. The cost runs the other way: a
node that comes back inside the window keeps its pre-restart baseline and goes on being
reported as drifted until someone reverts the change. `respawn` in a launch file defaults to
no delay, so the usual edit-the-YAML-and-restart flow is frequently never observed as an
absence at all: a respawn that completes inside one tick leaves the node present on every
sweep, and no window, however short, catches that. To make a restart re-baseline, keep the
node out of the graph long enough to miss three sweeps: a `respawn_delay`, or a stop, a pause,
then a start. Accepting a new value as the baseline without a restart does not exist today.

**Test tiers:**

1. **Unit** (`test/test_param_drift_policy.cpp`): the pure comparison, rendering and
   configuration rules against hand-built values - the glob matching, the exact
   integer-versus-float comparison and the NaN carve-out, the `baseline` and `expect`
   combinations, the config range checks, the ASCII-only elided rendering the capped description
   depends on, and the `read_gap` spacing the pacing budget is built on.
2. **Integration** (`test/test_param_drift_integration.cpp`): real `rclcpp` nodes with real
   parameters, read over the real parameter service against a fake `ReportFault` service, plus a
   scripted stand-in transport for the replies no live node can be made to produce on demand - a
   read still in flight when its app is de-armed, a SUCCESS carrying no parameters at all, a read
   that throws, a node that never answers. It covers the grace-based prune when a node vanishes,
   the re-baseline after an absence long enough to be a restart together with the single dropped
   poll that must NOT re-baseline, that the baseline forget is reachable at every accepted
   `prune_grace`, that the baseline horizon is exactly three missed sweeps and not two or four,
   the clear path, that a closing reliability gate does not heal a present drift, and every state
   in which a clear must be withheld because nothing is known about an app: one the sweep has not
   reached, one the gate has not armed, and one that answered once and then stopped. It also
   covers that both unread warnings need FAILED attempts AND elapsed ticks rather than either
   alone, that an app given up on stops blocking the clear, that a withheld clear says so in the
   log once, that a read overrunning the per-read bound is abandoned, that an app re-bound to a
   different node starts from a fresh baseline, the per-app share of the description and that
   trimming it keeps both ends, and the ordering that decides what survives the capped
   description: a newly drifted app ahead of the ones already reported, and an `expect` violation
   ahead of self-captured drift - including the cross case, where a pinned violation already
   reported still outranks brand-new self-captured drift. The pacing budget is covered in both
   tiers - `read_gap`'s arithmetic in the unit tests above, and the rate the reader actually
   achieves against a configured budget here.
3. **E2e** (`test/e2e/test_param_drift_e2e.test.py`,
   `test/e2e/test_param_roundtrip_e2e.test.py`,
   `test/e2e/test_config_plumbing_e2e.test.py`): five real gateway launches.
   The first is the acceptance gate: it drives the detector's DEFAULT self-capturing mode end to
   end - a real node whose parameter the test changes over the real parameter service, a real
   fault_manager, and both the raise and the heal read back from the operator-visible
   `GET /api/v1/faults`.
   The second is what makes the round-trip table above a measurement rather than an assertion. It
   runs with `param_drift` switched OFF and drives the same two transport functions the detector
   uses over `GET /{entity}/configurations`, against a node that answers the parameter services by
   hand and counts every request it is asked to serve: a cold and a warm `baseline` visit, a
   resolved pin, both NOT_FOUND paths (the unlisted name that costs 1 and the listed one with no
   value that costs 2, which is what the charge of 2 is sized for), a declared-but-unset pin that
   is a success costing 3, and a first pinned read of an untouched node. Nothing else in the
   package can notice a round trip being added to or removed from the TRANSPORT - every other test
   proves the arithmetic GIVEN those numbers, and the stand-in transport they use counts CALLS.
   It does not read the detector's charge constants either, though: those are held to the
   measurement by the timing cases in the integration suite, which pace the reader off them.
   The last three are the only proof that a nested SUB-OBJECT under a detector key survives the
   delivery path: `expect.<param>` arrives from the gateway as nested JSON and has to be flattened
   back to the dotted parameter name. (Plain per-detector scalars and arrays are proven by the
   orphan e2e.) Every C++ test above calls `configure()` directly with hand-built JSON and never
   touches the delivery path. One launch proves the nested
   `expect` reader, one proves `mode: "off"`, and one proves the bare `mode: off` that the
   ROS parser types as a YAML 1.1 boolean rather than a string, which is the form operators
   actually write.

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
  tick loop. Those `~/transition_event` subscriptions are kept out of the
  gateway's ROS executor (own callback group, own single-threaded executor that
  the plugin's tick thread drains between ticks), so they are only ever created,
  run and destroyed on that one thread. Non-managed nodes are never gated.
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
