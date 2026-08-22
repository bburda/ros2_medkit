# ros2_medkit_graph_watchdog

Gateway plugin that detects silent faults in the ROS 2 graph: failures where every
node is up, nothing logs an error, and the robot is still broken.

Detectors read the graph and raise faults through a `ReportFault` service client on the
gateway node; the faults surface via FaultManager on the gateway `/faults` API. That
client, like the lifecycle subscriptions below, lives in the plugin's own callback group
and is driven by the plugin's own executor from the tick thread, never by the gateway's
ROS executor.

This package carries the plugin skeleton, the central reliability gate that holds raises
until the graph has quiesced, and five detectors, `qos_mismatch`, `orphan`,
`param_drift`, `lifecycle_expectation` and `node_death`. Two silent-fault classes remain
undelivered, `GRAPH_TF_STALE` and `GRAPH_LATENCY_BUDGET`; they land in follow-up changes,
each against its own issue, and their fault codes are already reserved in the frozen
`GRAPH_*` namespace (see "Fault codes").

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

### `lifecycle_expectation` keys

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string \| bool | `raise` | As above. |
| `require_active` | string[] | `[]` | Node names that must be in the `active` lifecycle state, each matched against a live node by its `App::id`, its full FQN (`/ns/name`), or the bare leaf of that FQN. A bare name matches that node in EVERY namespace (a fleet-wide "all `controller_server`s must be active"); use a full FQN to pin one robot's. Both the bare and the FQN form match against the node's stable FQN, so they survive the `App::id` renaming that a same-bare-name collision triggers; an entry written as an `App::id` also works, but can stop matching on a multi-robot graph for exactly that reason. Empty = the detector checks nothing and emits nothing. A `require_active` that is not a string array warns and is ignored; an empty-string entry warns and is skipped - never dropped silently, since the operator would go on believing the node is covered. |
| `grace` | int | `5` | Consecutive not-active ticks a required node tolerates before being reported inactive - bringup time for a managed node to reach `active` (configure + activate) before the expectation is enforced. Counted per NODE, not per matching entry: a node named by both a bare-name and a full-FQN entry still advances its streak once per tick, so mixing the two documented forms cannot halve the grace that was configured. An ALREADY-CONFIRMED streak keeps its content while the node is ABSENT, so a node that leaves the graph after being confirmed stays confirmed; a streak that has not yet crossed `grace` on a node the presence detector could also report (it was armed at least once) is HELD rather than advanced while absent, so a node is never confirmed out of ticks gathered while nobody could observe it - only a PRESENT tick may still cross `grace` for that node. A node that was NEVER armed gets no such hold: `node_death` can never track it either, so absence keeps advancing its streak too, the same as a present tick would. Accepted range 0..300 - five minutes at the shipped 1 s cadence, and already an extravagant allowance for a managed node to reach `active`. The upper end is a real bound rather than a formality: `grace` decides how long a PRESENT node may go unreported, and how long a node returning from a departure still inactive takes to (re-)mature, so at the old maximum of `INT_MAX - 1` that PRESENT-side silence lasted about 24 days at the shipped cadence with no warning. Whether it also bounds how long a node that leaves the graph BELOW `grace` sits unsettled depends on the same split: for an armed node that lasts for as long as the node stays gone, independent of `grace` (see "Bounded by evidence, not by age" below); for a never-armed one `grace` bounds that too, since absence matures it within `grace + absence_grace + 1` ticks either way. The check runs on the wide integer, so a value that only fits after truncation is rejected rather than silently turned into a hair-trigger. Anything outside the range warns and keeps the default. |
| `prune_grace` | int | `60` | Consecutive ticks an IDLE tracked node - both clocks at zero and no matured ownership, so nothing to lose - may stay ABSENT from the graph before its bookkeeping is reclaimed. A node carrying evidence is never reclaimed by age at all, however long it stays gone, so this key cannot erase a fault's own state; the map is bounded instead by `tracked_node_cap` (see "Bounded by evidence, not by age"). Injected for every detector at plugin scope and overridable per detector; a value outside 0..3600 warns and this detector keeps its own default of 60. The range check runs on the wide integer, so an out-of-int-range value is rejected rather than truncated. The value is used as written - there is no `grace + 1` clamp, because there is no longer anything for one to protect. |
| `tracked_node_cap` | int | `512` | The most nodes this detector keeps bookkeeping for at once. It is what bounds the map, since evidence is never reclaimed by age (see "Bounded by evidence, not by age"): at the cap, idle entries are reclaimed first, then entries for DEPARTED nodes are collapsed into a count, and only if every tracked node is PRESENT and carrying evidence is a newly matched node refused - which withholds `GRAPH_NODE_INACTIVE`'s clear and is reported both in the log and on `GET /x-medkit-watchdog`. 512 is comfortably above every node in a realistic graph (a full Nav2 stack plus perception is roughly a hundred nodes; a ten-robot fleet sharing one domain a few hundred), so raising it is only needed where `require_active` legitimately matches more PRESENT nodes than that. Accepted range 1..16384 - 16384 is about 8 MB of bookkeeping at roughly 500 bytes per tracked node, and a deployment needing more distinct required-node identities alive at once has identity churn rather than a large fleet. Anything outside the range, including 0 (a cap of nothing would mean checking nothing), warns and keeps the default; the check runs on the wide integer, so an out-of-int-range value is rejected rather than truncated. |

### `node_death` keys

Zero-config, unlike every detector above: there is no `require_active`-style list of nodes
to watch, because every armed App in the graph is a candidate.

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `mode` | string \| bool | `raise` | As above. |
| `miss_grace` | int | `2` | Consecutive missed ticks a tracked node tolerates before being reported dead - a death is reported once misses EXCEED this, i.e. after `miss_grace + 1` ticks. Accepted range 0..3600, and additionally floored to whatever wall-clock window the configured `tick_interval_ms` needs (see "The wall-clock floor" below); a value below that floor is silently raised to it, with a warning naming the ms window it actually spans. |
| `prune_grace` | int | `60` | Plugin-injected default (overridable here); how long a key being DURABLY suppressed (see "Suppression" below) may sit unreported before its bookkeeping is reclaimed. Accepted range 0..3600. The value actually used is `max(prune_grace, miss_grace + 1)`, a silent internal floor rather than a rejection: without it, a durably-suppressed key could be reclaimed the very tick it would first have become eligible to report, losing the report rather than merely suppressing it. |
| `allowlist` | string[] | `[]` | Node identities never reported dead. A dead key matches if it - or a name the same node is also known by - is present verbatim: the key's own fqn, its bare leaf, or (captured while the node was still present) its `App::id`. Has no effect unless named in `suppress`; naming it without a configured list is a no-op. |
| `suppress` | string[] | `[]` | Suppression mechanisms to activate: `"allowlist"` and `"lifecycle"`. Any other entry warns as unrecognized and is ignored; a non-string or non-array entry warns the same way. |
| `tracked_node_cap` | int | `512` | The most DEPARTED (not-yet-confirmed-dead or already-collapsed) identities this detector keeps individual bookkeeping for at once - a present/armed node is never counted against it and never refused tracking. Accepted range 1..16384; `0` is refused rather than clamped, since a cap of nothing would mean tracking nothing. See "Bounded by evidence, not by age" below for what it actually bounds and why, unlike the sibling detector's identical-looking cap, a newcomer here is never the thing refused. |

`tick_interval_ms` is not an own key of this detector, but this is the one detector that
reads the plugin-injected value directly, to compute `miss_grace`'s wall-clock floor below.

**Liveness, and why membership in the snapshot is not enough.** `App::is_online` is what
this detector tracks, never mere presence in the entity snapshot. In runtime-only discovery
the two happen to coincide - a dead node's App leaves the snapshot entirely - but a manifest
keeps a bound App present with only `is_online` cleared once its ROS binding disappears, and
hybrid discovery inherits that same shape; counting snapshot membership alone would make a
manifest-declared node immortal. A managed lifecycle node that merely deactivates keeps
`is_online: true` - its process is still running, only its ROS 2 lifecycle state changed -
so a deactivation is never mistaken for a death either; that is `lifecycle_expectation`'s
concern, not this one's (see "The boundary with `lifecycle_expectation`" below).

Tracking is keyed on the STABLE fqn (`App::effective_fqn()`), never `App::id`: an id is
recomputed every sweep and only gains a namespace prefix once a same-bare-name collision
currently exists anywhere in the graph, so a live node's id can change out from under a key
built from it.

**What is never tracked.**

| Excluded | Why |
|---|---|
| Peer-aggregated apps (`app.source` starting `peer:`) | A peer app carries no ROS binding of its own, so `effective_fqn()` is empty for every one of them; tracking them would collapse an entire peer fleet onto one `""` key, and one online peer app would mask every other peer app's departure. |
| `_ros2cli_<pid>` nodes | Every `ros2` CLI invocation (`ros2 topic echo`, `ros2 param get`, ...) spins up a real, short-lived node under this prefix - rcl's own hidden-node naming convention, matched structurally rather than guessed at. Letting these through would accumulate one permanently "dead" entry per CLI invocation for the life of the gateway. |
| An App that never comes online | A key is admitted to tracking only once the reliability gate has said this detector OWNS its departure - armed at least once, and on one of the two GROUNDS ownership can rest on (a lifecycle state read as `active`, or no lifecycle to read at all; the third case, a state nobody could read, is admitted PROVISIONALLY and is described in the row below). A manifest App whose binding never starts is never armed and so can never be falsely called dead; a node still inside its `warmup_cycles` window gets the identical protection. Once tracked, though, a node's continued life-or-death judgement rests on PRESENCE alone, not on staying armed - a tracked node that later goes lifecycle-inactive is not thereby mistaken for dead. |
| A managed App whose lifecycle state has not been read YET | Not tracked, and only for as long as that is still true. The gate's own raise permission is answered PERMISSIVELY for a managed node whose `GetState` has not answered (`LifecycleWatcher::node_ok()`) - without that, `qos_mismatch`, `orphan` and `param_drift` would all fall silent about a node whose lifecycle service is broken. Permission is not knowledge, though, so tracking is withheld while a measurement may still arrive. `LifecycleWatcher` charges a GetState re-seed budget per node, and only for a read that actually ran, so "still asking" is a fact rather than a guess. Once that budget is spent the node is tracked here after all: nothing will ASK again, and `lifecycle_expectation` looks at nothing unless an operator named the node in `require_active` (empty by default), so withholding it any longer would mean the death is reported by nobody. That admission is PROVISIONAL, not final. The `~/transition_event` subscription outlives the seed budget, so a label can still arrive without being asked for, and one that reads non-active says the node was the lifecycle detector's all along - so the presence detector hands it back on the spot, while it is still alive. Waiting for the budget alone would only have delayed the wrong report, not prevented it. |

Composable/component nodes hosted in one process die together: if the container process
dies, every node it hosted leaves the snapshot on the same tick, and all of them are folded
into the one aggregated fault rather than raising one fault each - individually named up to
the description's 480-character cap and up to 3 at once under `tracked_node_cap` pressure,
whichever runs out first; past either limit the remainder still count toward the fault, just
not by name (see "Bounding the tracked-key map" below).

**The wall-clock floor on `miss_grace`.** The entity snapshot a tick reads is not rebuilt
every tick: runtime discovery rebuilds it off a graph event debounced to about one refresh
per second, so several consecutive ticks between two refreshes see the IDENTICAL snapshot. A
`miss_grace` counted purely in ticks does not count independent samples of the graph -
shorten `tick_interval_ms` enough and one stale cache generation gets re-counted as several
misses. The floor raises `miss_grace`, for the configured `tick_interval_ms`, to the
smallest value whose `(miss_grace + 1) * tick_interval_ms` window is still at least 3000 ms -
one graph-cache refresh cycle plus margin. At the shipped 1000 ms tick the floor is exactly
the shipped default (`miss_grace: 2`), so the default configuration is unaffected; only a
faster-than-default tick ever raises the effective grace, and it does so with a warning
naming the window it actually spans.

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

#### `lifecycle_expectation` (GRAPH_NODE_INACTIVE, GRAPH_NODE_UNREADABLE, GRAPH_NODE_NOT_MANAGED)

A configurable "this node must be active" check. The process is ALIVE and in the graph,
but a node the operator declared critical-active is either sitting in a non-active
lifecycle state (`inactive`/`unconfigured`/`finalized`), or its lifecycle promise could
not be MEASURED at all - a managed node whose label has never been read, or a node with
no tracked lifecycle whatsoever. On a Nav2 stack a `controller_server` stuck inactive
means the robot silently will not act - no crash, no log, nothing on `/diagnostics` or
`/rosout`. Presence itself is a different fault class (`GRAPH_NODE_DISAPPEARED`, this
package's own `node_death` detector): this detector only ever STARTS reporting a
node it has measured while PRESENT. A node that leaves the graph having only ever been
measured healthy is left entirely to the presence class - but one that leaves while already
under one of these three faults keeps it, because a node the operator declared must-be-
active going away is not an answer. See "Absence continues, it never erases" below.

**The model: one observed state per node per tick, two clocks.** All three fault codes
are outputs of a single per-node state machine
(`LifecycleExpectationTracker`, `lifecycle_expectation_tracker.hpp`) - not three
independent trackers that happen to share a detector. Every tick, a matched node is
classified into exactly one of `ACTIVE` (label `"active"`), `INACTIVE` (label non-empty
and not `"active"`), `UNREADABLE` (a managed node whose label has never been read -
`lifecycle_state_of()` returned `optional("")`), or `NOT_MANAGED` (`lifecycle_state_of()`
returned `nullopt` - no tracked lifecycle at all); a node not matched at all this tick is
a fifth, separate case, `ABSENT`. Two clocks track that history, and only two:

- The **violation streak** advances on `INACTIVE` and resets ONLY on `ACTIVE`. Past
  `grace`, the node is `GRAPH_NODE_INACTIVE`'s content.
- The **unmeasured clock** advances on `UNREADABLE` OR `NOT_MANAGED` and resets ONLY on a
  real measurement (`ACTIVE` or `INACTIVE`). It is deliberately BLIND to which of the two
  causes it is seeing on any given tick - a node alternating between "matched but never
  read" and "not a tracked lifecycle node at all" keeps this one clock climbing instead of
  each cause resetting a separate counter of its own. That blindness is what closes an
  entire class of alternation: three review rounds each found one pair of "I cannot
  measure this" causes that could erase each other's progress and leave a node invisible
  to every code that exists, and the fix is not a fourth special case, it is refusing to
  keep separate counters for causes the operator's actual question ("can I trust this
  node's state?") does not distinguish. Past `unmeasured_hold_ticks` (a fixed 60 ticks,
  not configurable), the clock MATURES and takes ownership of the node.

**Ownership is exclusive.** A node is content of at most one of the three fault codes at
a time. The moment the unmeasured clock matures, the violation streak is RELEASED -
actually reset to 0, not merely excluded from `GRAPH_NODE_INACTIVE`'s content this tick -
so that fault's clear is free to flow immediately. A node returning to `INACTIVE`
afterwards therefore re-earns `grace` from zero, exactly like a node that had never gone
unmeasured. Which of the two unmeasured codes a matured node reports under is STICKY:
`GRAPH_NODE_UNREADABLE` or `GRAPH_NODE_NOT_MANAGED`, whichever the clock matured under,
keeps reporting under that code even if the LIVE cause later flips - a node's own maturity
never re-derives its fault code from the current tick's read, only from a real
measurement resetting the clock entirely. The rejected alternative, current-cause-wins,
is more literal ("report exactly what is true right now") but makes a node that flaps
between the two causes flap the fault surface too - a raise/clear/raise churn across two
different fault codes for a node whose actual situation (still cannot be measured, either
way) never changed.

**Absence continues the last CORROBORATED observation, it never erases.** A node not
matched at all this tick is ABSENT, a separate fact from any observed state (there is no
label to classify). For up to `absence_grace` (a fixed 3 ticks) consecutive absent ticks a
node's whole state - both clocks, whichever fault (if any) currently owns it - is held
unchanged, so a node blinking out of one snapshot in every few is not indistinguishable
from one that was never measured. PAST `absence_grace` absence advances whichever clock the
node's SETTLED observation had started, and resets nothing:

| Settled observation | What sustained absence does |
|---|---|
| `inactive`, ALREADY reported under `GRAPH_NODE_INACTIVE` | the streak continues exactly as it did on its last present tick, so the fault stays raised and keeps naming a node that is no longer there |
| `inactive`, not yet past `grace`, node the presence detector OWNED at some point | the streak is HELD - neither advanced (no fault is born while nobody can observe the node) nor erased (a node that returns still inactive resumes rather than re-earning `grace`). The presence detector could report this exact departure instead (`node_death` tracks any node the reliability gate has admitted for presence ownership at least once), so this detector does not need to. |
| `inactive`, not yet past `grace`, node the presence detector NEVER owned | the streak keeps climbing on absence exactly as it would on a present tick. `node_death` only ever tracks a node the gate has admitted for ownership AND that is online, so a node that never reached that bar - one that never reached `active`, or one whose ROS binding never started - is structurally invisible to it no matter what happens afterwards. If this detector held the streak too, the departure would be reported by nothing at all. |
| unread label, or no tracked lifecycle | the unmeasured clock keeps climbing under that same cause, so the node is eventually reported under `GRAPH_NODE_UNREADABLE` / `GRAPH_NODE_NOT_MANAGED` |
| `active` | nothing. Anything the node had started but not corroborated is released, the entry becomes idle and is reclaimed silently |

**What "settled" means.** A real measurement - `active` or a non-active label - settles
immediately: a lifecycle label is a fact about the node, and no discovery artifact invents
one. An UNMEASURED observation settles immediately too on a node nothing has ever measured,
because it is then the only thing known about it (this is what keeps a crash-looping node
that is unreadable whenever it is up accumulating toward its report). It is only when an
unmeasured reading would OVERRIDE a real measurement that it must first hold for 6
consecutive matched ticks. The reason is that "no tracked lifecycle" is producible by the
discovery layer missing a node's service path for a single sweep, on a node that is present
and perfectly healthy - `LifecycleWatcher::update()` drops a tracked id whose path is absent
from the current sweep, and `discover_apps()` can yield an app with no services when a sweep
races service enumeration. Without corroboration, one such sweep immediately before a clean
shutdown would mature a healthy departure into a permanent `GRAPH_NODE_NOT_MANAGED`. Six is
the same bar the "probably a typo" warning below already sets against the same transient,
and a tenth of the 60-tick unmeasured hold, so a node that is genuinely unmeasurable when it
leaves has corroborated that long before the hold that reports it.

Three consequences worth stating plainly:

- **A departure never heals a fault it has already earned, within one gateway lifetime.** A
  node CONFIRMED not-active, or corroborated as unmeasurable, that then leaves the graph
  keeps its fault - the operator declared it must be ACTIVE, it was not, and being gone is
  not an answer. The fault's description switches to saying the node has since left the
  graph, so nobody is sent looking for it. A node that had NOT yet been confirmed when it
  left earns nothing from leaving either - see "Absence continues the last CORROBORATED
  observation" above. **Across a gateway restart it does not hold**, and that is a real
  boundary rather than an oversight: the restarted detector has no measurements at all, the
  departed node is not in the graph, and its `require_active` entry therefore matches
  nothing - which is indistinguishable from a misspelt entry, since the only component that
  knows the difference is the fault store and this detector does not read it at startup. The
  hold for a never-matched entry is deliberately bounded (a typo must not block healing
  forever), so once it expires the level-triggered clear flows and the record heals without
  anything having been measured. Re-seeding the tracker from the fault store at startup
  would change this; it is not implemented. The e2e scenario `restart_departed` pins the
  behaviour as it actually is.
- **A departure never STARTS one.** A node measured `active` that shuts down raises
  nothing, and neither does one whose lifecycle services went missing from a sweep or two
  immediately before it did. Reporting a healthy node that left is the presence class's job
  (`GRAPH_NODE_DISAPPEARED`, this package's own `node_death` detector), not this one's -
  a much narrower boundary than "a node absent past the grace is invisible whichever clock
  it was on".
- **A departed node never crowds out a present one.** Entries for departed nodes are
  collapsed into a count when the tracked-node cap needs the slot - see "Bounded by
  evidence, not by age" below.

Why, for the two UNMEASURED causes: every erasure horizon is an evasion for a node that
touches it periodically. A node in a restart loop - start, crash, respawn delay, start -
touches absence by construction, and discarding its evidence on absence would let it
alternate `(unreadable, absent x N)` or `(not-managed, absent x N)` forever without ever
accumulating enough of anything to be reported.

The VIOLATION streak used to need the identical rule for the identical reason - a node
alternating `(inactive, absent x N)` would otherwise never accumulate `grace` + 1 either.
Where the presence class CAN also report the same departure, that is no longer the only
thing standing between such a node and invisibility: `GRAPH_NODE_DISAPPEARED` (this
package's `node_death` detector) independently reports it, whether or not the node was
ever measured not-active first. But `node_death` only ever tracks an App once the
reliability gate says it OWNS that App's departure, and ownership needs the node's lifecycle
state either KNOWN not to be a managed-non-active one (no managed record at all, or a record
reading `active`) or asked for as often as it ever will be. So a `require_active` node that
never reaches `active` is never tracked by `node_death`, and neither is an app that is not
online. A managed node whose `GetState` has not answered is withheld only while the watcher is
still asking: the gate would let it raise throughout (`LifecycleWatcher::node_ok()` treats an
unread label as permission, deliberately), and once the re-seed budget is spent the node is
`node_death`'s after all, because nothing else would ever report it. The tracker therefore
splits on
whether the presence detector EVER owned a node, read from the reliability gate itself rather
than guessed from the observed label: once owned, a below-`grace` streak that goes absent is
simply HELD - maturing it would raise `GRAPH_NODE_INACTIVE` from ticks gathered while nobody
could observe the node, a fault its presence never earned, and `GRAPH_NODE_DISAPPEARED` is
there to report the departure instead. A node the presence detector never owned gets no such
backstop - `GRAPH_NODE_DISAPPEARED` structurally cannot report it - so absence keeps advancing
its streak exactly as a present tick would, maturing it within the same bound an owned node's
UNMEASURED clock already has (`grace` + `absence_grace` + 1 ticks). An ALREADY-matured
streak still continues through absence exactly as before, for either kind of node: two
codes standing at once for the same node (`GRAPH_NODE_INACTIVE` because it was measured
not-active, `GRAPH_NODE_DISAPPEARED` because it is gone) is not a problem to fix - they say
different, both-true things.

**Content follows the clocks, not the snapshot.** Whether a node happened to be in this
tick's matches decides nothing about what it reports: a node whose streak is past `grace`
stays in `GRAPH_NODE_INACTIVE`'s content while it blinks, and a matured node stays in its
own code's content. A fault's content is what the detector has MEASURED, and a missing
snapshot entry measures nothing either way.

**Safe default: off.** `require_active` is empty by default, so nothing is checked and
nothing is emitted - no raises AND no clears, not a single fault_manager request - until
an operator opts specific nodes in. Zero false positives out of the box, the same
config-scoped posture as `param_drift`'s `expect`.

```yaml
plugins:
  graph_watchdog:
    detectors:
      lifecycle_expectation:
        require_active: ["controller_server", "planner_server"]
        grace: 5
```

**The fault names the NODE, not the config entry.** The bare-name form is deliberately
fleet-wide, so one entry legitimately covers N nodes - and keying the violation on the
entry would leave the fault unable to say which of them broke. On a dual-arm robot
`require_active: ["controller_server"]` would produce `node controller_server expected
active but is inactive`, with nothing pointing at `/left` or `/right`, and with two
offenders only one would survive into the report at all. `source_id` cannot
disambiguate either (every `GRAPH_*` fault shares one), so the description is the only
carrier. Each offending node is its own entry, named by its stable FQN, with the entry
that demanded it as context. Two entries naming the SAME node (the documented bare-name
plus pinned-FQN mix) advance its clocks once per tick, not once per matching entry -
counting per match would silently halve whatever `grace` or the unmeasured hold the
operator configured.

**A single-snapshot blink does not restart anything.** A matched node that drops out of
the snapshot has its WHOLE state (both clocks, whichever fault currently owns it) held
unchanged for a small absence grace (3 ticks, fixed); discarding it on the first blink
would mean a genuinely stuck node that drops out of one snapshot in every few never
accumulates enough consecutive ticks on either clock to be reported at all. That is
reachable at the shipped cadence: the tick is 1s and the entity cache is graph-event
driven with a 1s debounce, so on a churning graph the snapshot turns over roughly once
per tick. Past the absence grace the clocks resume advancing rather than restarting - see
"Absence continues, it never erases" above.

**An entry that has never matched anything is reported.** It would otherwise be silent
in every path: it never reaches any of the three fault codes, and a presence detector
cannot backfill it either, because it only ever tracks nodes that were present at least
once. So a misspelt entry - or a required node that crashed during launch, or was down
when the gateway started - would produce no fault and no log line anywhere. After 10
consecutive ticks matching nothing, the detector says so in the log, once per entry per
configuration. An entry whose node WAS matched and later left the graph is deliberately
not accused: for it both halves of that sentence are false, and a departed node is
precisely what the presence class reports.

**The typo warning does not latch.** An entry that matches present nodes but none with
a tracked lifecycle state is probably a typo, or names a plain (non-lifecycle) node -
so the detector warns about it, but only after more than 5 CONSECUTIVE such ticks, and
the warning clears if the entry resolves. One transient tick is not evidence: the gateway's
`discover_apps()` wraps per-node service enumeration in a try/catch and pushes the app
regardless, so a sweep that races service discovery yields an app with no services and
no tracked state. The once-per-entry bookkeeping is reset on every `configure()`, so an
entry removed and re-added by a reconfigure warns again - which is exactly when the
operator wants to hear that it still names nothing managed. The warning is suppressed
when no reliability gate is wired, since then no entry has a lifecycle state to begin
with. This is a SEPARATE, log-only mechanism from `GRAPH_NODE_NOT_MANAGED` below: the
warning fires after 5 ticks and never raises a fault; the fault code fires only after
the same 60-tick unmeasured hold as its `GRAPH_NODE_UNREADABLE` sibling.

**Bounded by evidence, not by age.** One bare-name entry can match a node of that name in
every namespace, so under identity churn (nodes reappearing under ever-new FQNs) the
tracker's map would grow without a bound. Since absence no longer erases anything, an
entry carrying a live clock is not reclaimed by age either - the two are the same defect
seen twice, and moving a horizon rather than removing it would leave the evasion in place
at a different N. So:

- **`prune_grace` reclaims IDLE bookkeeping only** - both clocks at zero and no matured
  ownership, i.e. an entry with nothing to lose. An idle node absent for more than
  `prune_grace` consecutive ticks is reclaimed atomically: one node, one map entry, gone
  in the same tick, never partially. There is no `grace + 1` clamp any more, and none is
  needed: a node carrying evidence is exempt by construction rather than by arithmetic, so
  `prune_grace: 0` means exactly 0.
- **A non-idle entry is never pruned by age at all.** An entry whose UNMEASURED clock is
  still climbing cannot grow without bound in time either: past the absence grace it
  advances every tick, so it matures within at most `60 + absence_grace + 1` ticks and gets
  reported - the longest `GRAPH_NODE_UNREADABLE` or `GRAPH_NODE_NOT_MANAGED`'s clear can be
  withheld by one departed node. A below-`grace` VIOLATION streak on a node that was NEVER
  armed shares that same bound, for the same reason it advances at all while absent: it
  matures within at most `grace + absence_grace + 1` ticks, because nothing else will ever
  report that node's departure either. Only once a node HAS been armed does its below-grace
  streak lose the bound: absence then holds it rather than advancing it, so it neither
  matures nor becomes idle for as long as the node is away, however long that is. That is
  not a new way to withhold `GRAPH_NODE_INACTIVE`'s clear - the clear was already gated on
  every required node's status being settled, so one node this indecisive already blocked
  it before this design; what changes is only that the fault never NAMES an ARMED node's
  departure, since that node's evidence belongs to `GRAPH_NODE_DISAPPEARED` instead. `grace`
  is still capped at 300 rather than accepted up to the int maximum, because it independently
  bounds how long a PRESENT node may
  go unreported and how long a returning node takes to re-mature: at the old maximum of
  `INT_MAX - 1` that PRESENT-side bound was about 24 days at the shipped cadence, with no
  warning and no way to tell a silently-withheld detector from one finding nothing.
- **The map is bounded by `tracked_node_cap`** (default 512, accepted range 1..16384). At
  the cap, idle entries are reclaimed first (free - they carry nothing), then entries for
  DEPARTED nodes are collapsed into a count so a PRESENT node always wins a slot; only if
  every tracked node is PRESENT and carrying evidence is the NEWCOMER refused - never a live
  violation evicted. 512 is comfortably above every node in a realistic graph (a full Nav2
  stack plus perception is roughly a hundred nodes; a ten-robot fleet sharing one domain a
  few hundred), so only identity churn or a genuinely larger required set reaches it.

  Collapsing rather than holding follows from the fault being keyed by CODE, not by node:
  five hundred entries for dead identities keep the same one fault raised that a single entry
  would, and the description can only ever name a handful of them - so holding them buys
  nothing, while the slots they occupy can cost total blindness. Under identity churn a cap
  full of the dead would refuse a genuinely broken PRESENT node, which then never enters the
  detector's content at all, and `GRAPH_NODE_INACTIVE` would emit a level-triggered CLEAR
  every tick while that node sat there not-active. At most three departed entries stay named
  (three maximally-long details are all one 480-character description holds, so a fourth name
  could never be shown anyway); the rest become one line reading "and N more required node(s)
  left the graph ...". That line IS content, so collapsing an entry never heals its fault -
  and it is ordered ahead of the individually named entries but behind anything crossing on
  this tick, so a node that just broke is never displaced by it. The count only grows within
  one gateway lifetime: once collapsed, an entry's fqn is no longer known, so a node
  returning under it is tracked and measured afresh and cannot decrement the count. A
  non-zero count therefore says two things at once - those nodes left carrying evidence, and
  `require_active` is matching more identities than `tracked_node_cap` can hold.

  Refusing a newcomer means a required node goes unchecked, which is why it is never silent:
  it withholds `GRAPH_NODE_INACTIVE`'s clear for as long as it lasts (a detector that
  declined to check a required node cannot assert that every required node is healthy), it is
  logged once per saturation EPISODE - the latch re-arms when saturation ends, so a later,
  real one is not silenced by an earlier one - and it is reported on `GET /x-medkit-watchdog`
  under `detectors.lifecycle_expectation.tracking_saturated`, beside the current
  `tracked_nodes` count and the `tracked_node_cap` in force. Unlike the never-matched hold,
  this withhold is NOT bounded: that hold is bounded because a typo must not block healing
  forever, while saturation - once departed entries can no longer crowd out present ones -
  means genuinely more required PRESENT nodes than the cap allows, which is a capacity
  condition the operator resolves by pinning fewer identities or raising the key.

**Why grace, not the reliability gate.** Every detector's raise passes through
`ctx.raise_fault()`'s central `reliability_allows(gate, source_id)` gate, which ANDs
the entity's warmup state with `LifecycleWatcher::node_ok()` - true only when a managed
node's lifecycle is `active` (or it has no tracked lifecycle at all). Gating THIS
detector's raise the same way on the required node's own lifecycle would be
self-defeating: `node_ok()` is exactly FALSE for the inactive node this detector exists
to catch, so the central gate would suppress the signal forever. Instead the detector's
own pure, ROS-free core (`LifecycleExpectationTracker`, independently unit-tested in
`test_lifecycle_expectation_tracker.cpp`) counts consecutive not-active ticks itself
and raises only past its own `grace` - completely independent of `reliability_allows`.
The aggregated fault's outer `ctx.raise_fault` call is still subject to the central
gate for its own `source_id` (`graph_watchdog`), same as every other detector - see
"Reliability (bringup-quiesce)" below - just not on the required node's lifecycle
state, which is this detector's own job.

**A clear is withheld until the required set has actually been measured - this section
is entirely about `GRAPH_NODE_INACTIVE`.** `GRAPH_NODE_UNREADABLE` and
`GRAPH_NODE_NOT_MANAGED` have no withheld-clear guard of their own - see "Three
independent faults, not one shared record" below for why they don't need one. A clear
asserts that every required node is free of a CONFIRMED violation, and the
`GRAPH_NODE_INACTIVE` raised before a gateway restart is still in the store while the
restarted detector counts from zero - so a clear emitted on the strength of an empty
affected map would spuriously HEAL a still-real fault. Two things produce that empty map
without the assertion being true, both the ordinary state of bookkeeping that just
started over (a restarted gateway, a reconfigure, a node that respawned stuck):

1. **An entry has not matched anything yet.** Before the entity snapshot catches up with
   the graph, a `require_active` entry matches no node at all - so a clear would be about
   a node the detector has never once looked at. Every restart passes through this
   window: the plugin starts ticking as soon as it is loaded. Bounded the same way the
   node-keyed state below is (60 ticks), so a misspelt entry cannot block healing for the
   process lifetime.
2. **A node's status is UNSETTLED.** The tracker's own `pending` set - a violation streak
   that has not yet passed `grace`, an unmeasured clock still climbing (whichever of the
   two causes), or a streak HELD while the node is inside an unmeasured spell. A node
   whose unmeasured clock has MATURED is deliberately NOT in this set: ownership passed to
   its own fault code and the violation streak was released (see "Ownership is exclusive"
   above), so a node stops counting toward this withhold the exact tick it stops being
   uncertain - it does not linger as a reason to keep `GRAPH_NODE_INACTIVE` waiting once
   the question about it has a real answer somewhere else. Absence never puts a node here
   on its own: content follows the clocks, so a node already past `grace` stays in the
   fault's content through a blink rather than dropping into a withheld limbo.

Either reason withholds the emission entirely, neither raise nor clear, so a fault
already in the store keeps its state instead of being healed by a detector that has not
re-established the node is fine.

What never blocks `GRAPH_NODE_INACTIVE`'s raise: a RAISE is never withheld (a violation
read from the nodes that did answer is real regardless of the unmeasured ones). An entry
that HAS matched and later stops matching does not re-enter reason 1 either. The
never-matched hold and both unmeasured-clock causes are bounded (60 consecutive ticks, a
minute at the shipped cadence) and release by SETTLING the node's status rather than by
giving up on it, whether the node is present or gone. A below-`grace` violation streak
releases the same way - as soon as the node reads `active`, or a PRESENT tick pushes its
streak past `grace` and the fault is raised - but while the node stays absent that release
has no timeout of its own: absence holds the streak rather than advancing it, so the hold
lasts for exactly as long as the node does not return.
Because a withheld clear is indistinguishable from a detector that is working and
finding nothing, a hold that lives past 10 consecutive ticks is explained in the log once
per episode, naming every reason in force and, for the node-keyed ones, how many nodes
are behind each and one of them by name - the not-managed and unreadable reasons are
named separately even though both now release the same way (into their own fault code),
since an operator reading the log wants to know WHICH of the two is happening, not just
that one of them is.

**Three independent faults, not one shared record.** `GRAPH_NODE_INACTIVE`,
`GRAPH_NODE_UNREADABLE` and `GRAPH_NODE_NOT_MANAGED` are each raised through the shared
`AggregatedFault` helper (see "Aggregated fault, not per-node" below), but as three
SEPARATE, fixed-severity members - `GRAPH_NODE_INACTIVE` always `SEVERITY_ERROR`, the
other two always `SEVERITY_WARN` - the same shape `orphan` and `param_drift` use, not a
single record whose severity has to be picked from that tick's mixed content. Raises are
fully independent: each fault's content comes from its own measurement, and one raising,
healing, or changing severity never forces, blocks, or reflects onto the others.
`GRAPH_NODE_INACTIVE`'s own clear is not simply "nothing CONFIRMED non-active this
tick", though - it is withheld exactly as described under "A clear is withheld..."
above; that is the ORIGINAL withheld-clear guarantee this detector always gave, scoped
to `GRAPH_NODE_INACTIVE` alone. `GRAPH_NODE_UNREADABLE` and `GRAPH_NODE_NOT_MANAGED`
have no such guard: each one's own clear needs nothing beyond its own content going
empty, because once the unmeasured clock has matured "still cannot be measured" is a
settled fact, not a pending one - there is nothing left to wait on. A node is content of
at most one of the three at a time, and healing one never forces, blocks, or changes the
severity of another. Content under either unmeasured code survives for as long as a node
stays that way, with no further bound past the initial hold.

**A re-bind is a fresh binding.** The lifecycle label cache is keyed by `App::id`, and
an id can survive a graph sweep while pointing at a DIFFERENT node (id assignment
shifts under bare-name collisions). The shared `LifecycleWatcher` therefore re-checks
every tracked id's binding identity - its FQN and its `GetState` service path, both
captured at first sighting - on every tick. A moved binding is two events at once: the
OLD binding is recorded as departed under ITS OWN FQN (same record and retention as a
vanish), and the id is re-seeded through the ordinary new-node path (fresh `GetState`,
fresh `~/transition_event` subscription). For this detector that means the departed
node's label is never enforced against the node the id now binds: the new binding
starts unknown (benign) until its own label is read. No straggler from the old
subscription has to be filtered out - dropping the entry destroys that subscription on
the tick thread, the only thread that ever pumps these callbacks, so nothing it had
queued is delivered afterwards.

**Aggregated fault, not per-node**, via the shared `AggregatedFault` helper - the same
rationale as every other detector here: the fault_manager identifies a fault by
`fault_code` alone, so one `GRAPH_NODE_INACTIVE` per stuck node would collide into a
single record under the shared code. Three graph-level faults, each enumerating every
currently affected node for ITS OWN code, each description capped independently at 480
characters with a truncation marker. Like `orphan` and `param_drift`, all three are
fixed-severity `AggregatedFault` members at class scope (`aggregated_inactive_`,
`aggregated_unreadable_`, `aggregated_not_managed_`) rather than being rebuilt per tick:
unlike `qos_mismatch_detector`'s `any_starved ? kStarvedSeverity : kPartialSeverity`,
there is no record anywhere in this detector whose severity has to be picked from mixed
content. `GRAPH_NODE_INACTIVE` clears (`EVENT_PASSED`) on a tick where nothing is
CONFIRMED non-active AND the withheld-clear guard above is satisfied - not on every tick
where the tracker's affected map happens to be empty. The other two clear on any tick
where no matched node is owned by them, with no guard of their own to satisfy. All three
reaching HEALED needs the same `healing_enabled` requirement described in "Closing the
loop" above.

**New violations are named first, not alphabetically - in EACH fault's own description
independently.** The description used to list affected nodes in fqn order
(`AggregatedFault::emit`'s default), which is fine when every entry is equally
interesting but not once the cap is full: a fleet sharing
`require_active: ["controller_server"]` across a dozen robots fills the 480-char cap from
the alphabetically-earliest ones, and a THIRTEENTH robot going inactive afterward would
be silently invisible forever - one shared `fault_code`, one record, no way to tell the
operator which of thirteen actually broke. The tracker reports which fqns entered EACH
fault's content on THIS tick - `newly_affected`, `newly_unreadable`, `newly_not_managed` -
and each list orders only its OWN fault's `AggregatedFault::emit_ordered` call, since the
three faults never share a description: a fresh entry is named FIRST in whichever fault
it belongs to, and every other affected node in that same fault still appears, in the
same fqn order as before, once the fresh ones are placed. When every node crosses on the
same tick (`grace: 0` during a bringup burst, for instance), there is nothing to
distinguish them by and the order degrades to fqn order. Two budgets protect each
description, applied before it ever reaches the 480-char cap: the lifecycle label -
which arrives verbatim off a remote `~/transition_event` and is therefore untrusted and
unbounded, and only ever appears in `GRAPH_NODE_INACTIVE`'s own detail, never the two
unmeasured faults' (there is no live label to show for either of them) - is trimmed to 32
characters before it is interpolated (more than double the longest label a conforming
implementation produces, `errorprocessing` at 15 characters), and the whole per-node
detail is then capped at 150 characters as a backstop against a pathological fqn or a
long `require_active` "required by" list, sized so at least three worst-case details
still fit inside the 480-char cap (`3 * 150 + 2 * 2 = 454 <= 480`).

**Test tiers:**

1. **Unit** (`test/test_lifecycle_expectation_tracker.cpp`): pure
   `LifecycleExpectationTracker` logic over hand-built matches. Covers the violation
   streak (stuck-inactive past grace raising keyed by the node with the entry as context,
   active never raising, reaching active within grace resetting the streak, both
   namesakes reported separately and a healthy namesake not masking a broken one, two
   entries naming one node not halving its grace, a violating read winning a duplicate
   match's tie-break); the unmeasured clock shared by UNREADABLE and NOT_MANAGED (neither
   ever confirms a violation while climbing; each matures into its OWN fault at the exact
   hold boundary, tick-exact; the clock resets ONLY on a real measurement, in both
   directions; maturity RELEASES the violation streak entirely, so a node returning to
   inactive re-earns grace from zero rather than re-crossing immediately; the sticky-cause
   design decision - a node stays reported under whichever code it matured under even
   after the LIVE cause flips, pinned directly against the rejected current-cause-wins
   alternative); the alternation this slice's redesign exists to close - a node
   alternating between unreadable and not-managed, indefinitely and on every single tick,
   still matures the shared clock, closing the exact hole that let a node evade both
   codes when they were counted separately; the alternation between a MEASURED not-active
   read and a not-managed one across absence gaps longer than the absence grace; that the
   violation streak survives non-maturing unmeasured ticks and RESUMES rather than
   restarting (counted exactly, and read through `pending_violation` - the only field that
   differs during the climbing window); absence CONTINUING an already-matured fault exactly
   as it was on its last present tick (a blink holds either clock unchanged; past the blink
   tolerance an unmeasured clock still climbing keeps climbing and matures on absence alone,
   its detail then saying the node has left the graph, while a below-grace violation streak
   is instead HELD - neither advanced nor erased - and RESUMES rather than restarts once the
   node returns, proven both from a single long absence and from many short ones
   interleaved with matched reads), a matured violation fault surviving a departure, a node
   measured ACTIVE that vanishes raising nothing and being reclaimed, and the two
   `(unreadable/not-managed, absent x N)` restart-loop shapes for N at the absence grace and
   past it (their `inactive` sibling now pinned to the opposite claim, at the same two N: it
   never crosses grace from absence alone, for a node that was armed at some point - a node
   that was NEVER armed instead matures from absence alone within `grace + absence_grace + 1`
   ticks, exactly like the unmeasured clock does, and resumes rather than restarts on return
   the same way an armed node's held streak does); the
   `pending` set and its per-reason breakdown; new-first ordering for all three
   fault-shaped maps, including the lexicographic tie-break when several cross together;
   the remote-supplied label's own trim budget ahead of the whole-detail backstop, and the
   same backstop reapplied to a matured unreadable node's detail (which carries no label,
   only a fqn and a "required by" list); the age horizon now reclaiming IDLE entries only,
   atomically, and never touching one that carries evidence even when it undercuts the
   absence grace; the SETTLING rule that decides what absence may continue - an
   uncorroborated unmeasured run before a healthy departure raising nothing, swept across
   every run length below the bound, with the entry released rather than left holding the
   clear hostage; the same run one tick longer still being reported; and a single MEASURED
   not-active read before a departure still confirming, since a label needs no corroborating;
   and the tracked-node cap - idle entries reclaimed first, departed entries collapsed into a
   count so a present broken node is always admitted and reported, at most three of them left
   named, the count surviving into the description as content, a node returning after its
   entry was collapsed being measured afresh, the newcomer refused only when every entry is
   PRESENT and carrying evidence, and saturation reported as a LEVEL on every refused tick with
   its edge re-arming when an episode ends - swept at a shrunk cap and again at the real
   shipped 512.
2. **Integration** (`test/test_lifecycle_expectation_integration.cpp`): the detector
   driven against a fake `ReportFault` service, with a REAL `ReliabilityGate` arming the
   global bringup grace and feeding it labels through
   `ReliabilityGate::lifecycle_state_of()` (injected via the gate's test seam strictly
   after the last `gate.update()` - see the file header for the ordering rationale, and
   for the alternation/not-managed cases, which instead toggle whether the matched app
   carries lifecycle services and drive the gate's real discovery path, since the
   injection seam can only ever SET a tracked value, never remove tracking to produce a
   genuine `nullopt`). Covers the `GRAPH_NODE_INACTIVE` raise/clear round trip naming the
   node, the active-from-arming positive control, the absence-after-raise clear, bare-name
   and full-FQN matching against a namespaced app, the zero-config silence measured as
   zero fault_manager requests of any kind, the once-per-configuration unmanaged-entry
   warning, the withheld-clear guard's releases and its once-per-episode log line, the
   blink-plus-unread-re-seed sequence, the no-match warning, a filler batch sized (from
   the real detail-building code) to exceed the 480-char cap aggregating into one fault
   with the fresh crossing named first, a PRESENT node crossing grace fresh while a batch of
   already-departed, already-matured ones sits absent-and-content being named ahead of them
   (and not truncated away by them), a required
   node appearing mid-run, a re-bind under the same `App::id`, and the
   reconfigure/config-validation edge cases. The
   unmeasured clock's own split is pinned directly, for BOTH codes symmetrically: a
   managed node whose `GetState` genuinely never answers (through `set_managed_app` and a
   real, failing seed - proven by asserting `lifecycle_state_of()` actually returns
   `optional("")` first, not assumed) is reported under `GRAPH_NODE_UNREADABLE`; a node
   with no tracked lifecycle at all is reported under `GRAPH_NODE_NOT_MANAGED` (no longer
   released into silence, the deliberate behaviour change this slice makes); either hold
   releases into its report on the exact tick past its bound, never a tick early; a node
   already reported under one of the two clears once genuinely read, returning to
   ordinary `GRAPH_NODE_INACTIVE` tracking from there; a confirmed node healing while an
   unreadable OR not-managed sibling stays present clears `GRAPH_NODE_INACTIVE` promptly
   without touching either unmeasured fault; content under either code survives for as
   long as the node stays that way, with no window and no expiry; 25 nodes of either
   cause aggregate into one capped fault, at `SEVERITY_WARN`, never `GRAPH_NODE_INACTIVE`;
   a node whose hold expires opens its fault's own description over an already-reported
   filler batch, the same new-first ordering; an already-reported node of either cause
   KEEPING its own record once it vanishes (and its description switching to say the node
   has left the graph), a node returning from that absence staying reported with no
   clear/re-raise churn, each of the two UNMEASURED restart-loop shapes raising its own code
   from absence alone (their `inactive` sibling instead pinned to counting only real reads,
   never crossing from any of the interleaved absence gaps), the
   `inactive` <-> `not-managed` alternation across absence gaps now counting only the
   MEASURED not-active legs, a node measured ACTIVE
   that vanishes raising nothing under any of the three, and a withheld `GRAPH_NODE_INACTIVE`
   clear releasing when the absent node's clock MATURES into its sibling's content rather
   than when the node is given up on; the two wire strings pinned as hand-typed literals
   never read from their constants; and the independence claim in both directions -
   clearing one unmeasured fault never disturbs a concurrently raised
   `GRAPH_NODE_INACTIVE` or the OTHER unmeasured fault.

   Alongside the fixture, the `configure()`-level cases pin the config
   contract: the unknown-key warning, a fully-valid config producing zero warnings
   (including the documented low endpoints), `grace` and `prune_grace` validation
   (negative, non-integer, past the int range, and both range endpoints), `require_active`
   validation, `tracked_node_cap` validation at both range endpoints and one value past each
   (with the key proven IN FORCE at both ends, not merely accepted), the unclamped prune
   horizon reaching idle bookkeeping at exactly the configured `prune_grace` (0, 1 and 4 -
   the smallest positive value included, since neither documented endpoint sweeps it) while a
   node carrying evidence survives it - including the `grace: 0, prune_grace: 0` corner and a
   wide `grace` beside the tightest `prune_grace`, whose instrument is the CONFIRMATION rather
   than the map size - boundedness under identity churn at the real 512-node cap by collapsing
   the departed rather than refusing the live node, and saturation reported once per EPISODE
   with a second episode reported again after the first ends.
3. **E2e** (`test/e2e/test_lifecycle_expectation_e2e.test.py`): the acceptance gate -
   one file, twelve CTest targets, each launching its own real gateway + plugin `.so` +
   fault_manager stack. Six scenarios drive the `managed_lifecycle` demo node (a real
   `rclcpp_lifecycle::LifecycleNode`, which always answers `GetState`); two drive
   `unreadable_lifecycle_node.cpp`, a fixture built specifically to never answer it (see
   below); two drive `calibration`, a PLAIN demo service node with no lifecycle
   interface at all - no purpose-built fixture was needed for `GRAPH_NODE_NOT_MANAGED`,
   unlike `GRAPH_NODE_UNREADABLE`; and two drive this package's own
   `test/e2e/droppable_lifecycle_node.cpp`, which looks managed, answers a chosen lifecycle
   label, and stops advertising its lifecycle services on command - the only way to reach a
   node that leaves the managed set without either becoming healthy or leaving the graph.

   The `cap_pressure` scenario runs `tracked_node_cap: 1` against TWO required nodes, so one
   is refused on every tick: it proves the refusal is visible on `GET /x-medkit-watchdog`,
   that it WITHHOLDS `GRAPH_NODE_INACTIVE`'s clear (measured through `last_passed`, which
   catches even a transient clear), and that the entry for a node that then LEAVES is
   collapsed so the present, still-broken node is admitted and named. The
   `unsettled_departure` scenario runs two healthy managed nodes and drops the lifecycle
   services of each: one is killed immediately (a single missed sweep before a clean
   shutdown - nothing may ever be reported about it) and the other holds the dropped state
   past the settling budget before being killed (genuinely not managed when it left - it must
   still be reported), with the first leg's own window MEASURED against the settling budget so
   it cannot silently become the second. `wide_grace` configures the value that used to be
   the accepted `grace` maximum and proves it is refused and the default applied - under it
   the detector could neither raise nor heal for days. `restart_departed` records where "a
   departure never heals a fault" ends: it kills the required node while its fault is
   outstanding, restarts the gateway, and pins that the record then heals, because a restarted
   detector cannot tell an entry for a departed node from a misspelt one. The main scenario drives real
   `lifecycle_msgs/srv/ChangeState` transitions: the required node in its launch-default
   unconfigured state raises `GRAPH_NODE_INACTIVE` on `GET /api/v1/faults` (and on the
   entity-scoped `/apps/graph_watchdog/faults`), a real CONFIGURE leaves the fault raised
   (inactive is still not active), a real gateway RESTART - SIGTERM, the port going down,
   the relaunched process arming again - must not produce a single PASSED for the
   still-inactive node, which is the withheld-clear guard at the only tier that can reach
   it, and a real ACTIVATE finally arms the node's per-entity gate and heals it. The
   default-config scenario launches with NO `lifecycle_expectation` config at all against
   the same inactive node and holds a sustained silence window - the zero-false-positive
   default, falsifiable nowhere else. The negative control runs the self-activating
   variant of the same executable with the same grace and cadence, its `require_active`
   naming that active variant, and holds the same window; the discriminating variable is
   the node's actual lifecycle state. Both silence scenarios gate on three facts before
   asserting absence and re-pin the last of them afterwards: the plugin is armed, `GET
   /faults` answers 200 in this launch (a dead fault_manager answers 503 and polls to the
   same `None` the assertion wants), and the target's label was actually READ via the
   plugin's own `GET /x-medkit-watchdog` route - an unread label would produce the same
   silence for the wrong reason. The `healing_threshold` scenario proves the
   withheld-clear guard's pending leg against the REAL fault_manager debounce state
   machine, not a fake sink: with `healing_threshold` set to 1 (the README's own
   recommended value, see "Closing the loop"), the required node - already reported stuck
   - is SIGTERM'd and respawned twice under the same name (a real snapshot blink, not a
   lifecycle transition), each one comfortably inside the tracker's absence grace. The
   instrument is the fault's `last_passed` and `status` fields read from `GET
   /api/v1/faults`, the same instrument test_02b uses for the restart leg above: nothing
   below e2e ever exercises the real debounce counter, only a fake `ReportFault` sink, so
   only this tier can prove a blink never moves it. The same run is also this package's
   only e2e coverage of a node oscillating readable/unreadable producing no
   `GRAPH_NODE_INACTIVE` event churn: `GET /faults/stream` is read raw for the whole
   blink sequence (each blink briefly re-seeds the required node's label, present but
   unread, before it settles back) and checked for a `fault_cleared` frame for
   `GRAPH_NODE_INACTIVE` (there must be none) and for every `fault_confirmed`/
   `fault_updated` frame for `GRAPH_NODE_INACTIVE` carrying `severity_label: "ERROR"` -
   its only possible value now that severity is fixed per code rather than chosen from a
   tick's mixed content. This scenario's blinks are far too short to ever convert the
   required node into `GRAPH_NODE_UNREADABLE` content (that needs a REAL, sustained
   60-consecutive-tick `GetState` failure), so no `GRAPH_NODE_UNREADABLE` frame is
   expected on this stream either.

   The fifth scenario, `unreadable`, is where that 60-tick failure is actually reached.
   It launches `unreadable_lifecycle_node.cpp`: a plain `rclcpp::Node` that advertises
   `get_state`/`change_state` with the service TYPES `find_lifecycle_get_state_path`
   checks for, but whose `get_state` handler stores every request (via rclcpp's
   deferred-response service callback) and never answers until its own
   `start_answering` parameter is set true. Against that fixture the scenario proves,
   through `GET /api/v1/faults` on the real stack: the node is present and matched with
   its lifecycle label read as `""` (never answered, not "no data yet"); `GRAPH_NODE_INACTIVE`
   stays silent for it the whole time - impossible by construction, proven live rather
   than trusted from the source; `GRAPH_NODE_UNREADABLE` raises once the 60-tick hold
   expires, names the node, and carries `SEVERITY_WARN`, with `GRAPH_NODE_INACTIVE` still
   silent at that same moment; and once the fixture starts answering `"active"` the
   watcher reads it through a real GetState round trip and `GRAPH_NODE_UNREADABLE` clears.

   The sixth scenario, `departure_keeps`, proves what a DEPARTURE does to an
   already-reported unmeasured fault: nothing. It drives the same
   `unreadable_lifecycle_node.cpp` fixture up to the same raised `GRAPH_NODE_UNREADABLE`,
   then SIGTERMs the fixture process - permanently, no respawn - and confirms it is really
   gone from `GET /apps` (the same `ThreadSafeEntityCache` the detector's own per-tick
   snapshot reads) before asserting anything about the fault. Past every horizon that could
   have discarded the node's evidence, the fault is still active, has never once been
   reported PASSED (`last_passed`, which catches even a transient clear), and its
   description now says the node has left the graph. `GRAPH_NODE_INACTIVE` never raises for
   it at any point, before or after the kill, because the fixture is never told to start
   answering anywhere in this scenario. Only this tier can show that the real discovery
   layer noticing a process leave DDS does not walk the real fault_manager's debounce
   counter toward HEALED.

   The seventh scenario, `not_managed`, is `GRAPH_NODE_NOT_MANAGED`'s own acceptance
   gate, proving the sibling cause the unmeasured clock is blind to on the real stack:
   `calibration` (`DEMO_NODE_REGISTRY`'s plain `demo_calibration_service`, no lifecycle
   interface whatsoever) is present and matched from launch, `GRAPH_NODE_NOT_MANAGED`
   raises once the 60-tick hold expires naming the node at `SEVERITY_WARN`, and neither
   `GRAPH_NODE_INACTIVE` nor `GRAPH_NODE_UNREADABLE` ever raises for it - a node is
   content of at most one of the three, and this is the one live proof that the
   NOT-MANAGED cause specifically never bleeds into the UNREADABLE code it shares a clock
   with. The departure leg mirrors `departure_keeps`'s own shape: SIGTERM the fixture,
   confirm it is gone from `GET /apps`, and the fault is still there afterwards, never
   PASSED, now describing a node that has left the graph.

   The eighth scenario, `restart_loop`, is the acceptance gate for the whole
   evidence-retention model: the same plain `calibration` node, but respawning, SIGTERM'd
   over and over on a cadence that never lets it accumulate the 60 consecutive PRESENT
   ticks the unmeasured hold would otherwise need. Every cycle's absence is proven rather
   than assumed - `GET /apps` is polled until the node is gone and again until it is back,
   and the measured gap must exceed the 3-tick absence grace, on top of launch's own
   enforced `respawn_delay` floor - so every cycle genuinely crosses the horizon past which
   evidence used to be discarded. `GRAPH_NODE_NOT_MANAGED` must raise anyway, naming the
   node at `SEVERITY_WARN`, and must survive the restarts that follow. A required node in a
   crash loop is the case this detector most exists to catch, and it is the one that a
   design discarding evidence on absence makes permanently silent.

#### `node_death` (GRAPH_NODE_DISAPPEARED)

Watches every armed App in the graph for one that goes offline, and raises
`GRAPH_NODE_DISAPPEARED` naming it. Zero-config, unlike `lifecycle_expectation`'s
operator-declared `require_active` list: every armed App is a candidate. See the
`node_death` key table and the "Liveness", "What is never tracked" and "The wall-clock
floor on `miss_grace`" notes above the `orphan` key table for what counts as alive, what
this detector never tracks, and why `miss_grace` has a floor - none of that is repeated
here.

**Suppression.** Nothing is suppressed unless the operator names the mechanism in
`suppress` - a configured `allowlist` that `suppress` does not name has NO effect, and a
startup warning says so. Two mechanisms exist, activated independently of each other:

| Name | Mechanism | Durable |
|---|---|---|
| `"allowlist"` | `AllowlistSuppressor` over the configured `allowlist` set - the same three-way match (fqn, bare leaf, `App::id`) `lifecycle_expectation`'s `require_active` uses, so an operator who has one working can expect the other to accept the same shapes. Exact match only, never a prefix or a shared suffix. | Yes |
| `"lifecycle"` | `LifecycleShutdownSuppressor` - suppresses a departure that the reliability gate classifies as a clean managed-lifecycle shutdown. See "Clean-shutdown suppression" below for what counts as clean. | Yes |

Every field is re-validated from scratch on every `configure()` call: a malformed
`allowlist` entry, an unrecognized `suppress` name, or a `suppress`/`allowlist` entry of the
wrong type each produce their own named warning rather than being silently dropped. A
candidate is suppressed the moment ANY active mechanism votes yes, order-independent -
which one runs first never changes the result. Suppression is independent of `mode`: a
suppressed key never becomes content in the first place, while `mode` separately decides
whether non-suppressed content is actually sent (`advisory` observes without sending,
`off` disables).

**Durability, and why only a durable veto may reclaim bookkeeping.** Whether a suppressor is
durable answers whether its "yes" is a standing fact about the key rather than a condition
that can later stop holding. Both mechanisms here are durable: an operator-declared allowlist
entry does not start matching and then stop on its own, and a departure's observed shutdown
shape does not change after the fact. Durability is what makes reclaiming a suppressed key's
tracker bookkeeping (`prune_grace` consecutive suppressed ticks) sound - a NON-durable veto
could lift later even after its key's bookkeeping was discarded, leaving a live, unsuppressed
death with nothing left to report it: a false heal that silently outlives the very condition
that produced it. A durable veto never lifts for a given key once it has fired, so reclaiming
under it loses nothing. Durable defaults to false, the safe assumption for a suppressor
nobody has reasoned about yet; both suppressors here override it explicitly to true.
Reclaiming is re-checked against the durable suppressors specifically, never merely "no
longer in this tick's report", because a key can be dropped from a tick's report by ANY
suppressor in the chain while a durable one also happens to cover it - whether a key may be
reclaimed depends on WHICH suppressor vetoed it, not on whether it survived the filter.

**Clean-shutdown suppression - what counts as clean.** `LifecycleShutdownSuppressor` reads
the lifecycle watcher's cached label for the departed node's LAST observed transition:

| Last observed label | Suppressed? |
|---|---|
| `shuttingdown` | Yes - only reachable via a deliberate SHUTDOWN transition, so the label alone is enough. |
| `finalized`, with an observed transition and never through the error branch | Yes. |
| `finalized`, with no observed transition, or reached through the error branch | No. |
| `unconfigured` | No, deliberately - it is also the resting state of a node whose `configure()` failed or that never activated at all, and suppressing on it would hide exactly that startup failure. |
| Any other label, or no departure on record at all | No (abstains). |

**Which bindings the mechanism can see.** The departed-lifecycle record it reads is written
when a tracked app's `get_state` path stops appearing in `App::services`, which is what the
lifecycle watcher builds its managed set from. `App::is_online` plays no part in that, and the
two do not have to coincide: a manifest-declared App outlives the node it binds, keeping its id
and fqn while only its runtime-derived collections empty out. That case still works - the
services go with the node, so the departure is recorded - because a manifest cannot declare
services at all: `ManifestParser::parse_app` reads `id`, `name`, `description`,
`is_located_on`, `depends_on`, `tags`, `external` and `ros_binding`, and nothing else, so an
app's services can only ever come from a live runtime match. The case the mechanism genuinely
cannot cover is `discovery.inherit_runtime_resources: false`, which restores the manifest's own
(empty) collections on every sweep: such an app is never lifecycle-tracked in the first place,
so it has no lifecycle label, `node_ok()` never gates it, and there is nothing for this
suppressor to classify. That is the documented meaning of the flag - the app exposes only what
the manifest declared - rather than a gap here.

`finalized` needs the extra corroboration because it is also where a node's `on_error`
override lands after `ON_ERROR_FAILURE`/`ON_ERROR_ERROR` out of `errorprocessing` - the
standard way a driver reports a hardware fault it cannot recover from, which is precisely the
death an operator needs reported, not silenced. Without an observed transition history the
departure is unclassified and stays reported - the safe direction. This mechanism is
stateless by construction: a detector's `configure()` (where the suppressor chain is built)
runs before any per-tick context exists, so it stores nothing at construction and reads the
gate fresh on every call. Its retention window is not this detector's own to size, either:
before this detector's own `configure()` has run, the plugin predicts the same
`prune_ticks_` (`max(prune_grace, miss_grace + 1)`) this detector will compute, then sizes
the lifecycle watcher's departed-node retention from it with enough margin that a
clean-shutdown label is still cached at this detector's own reclaim tick, one tick to
spare - see `GraphWatchdogPlugin::compute_departed_retention_ticks()` for the exact
arithmetic, which is larger than `prune_ticks_` alone.

**Bounded by evidence, not by age.** This detector is zero-config over every armed App in
the graph - a strictly larger scope than `lifecycle_expectation`'s named `require_active`
set - so identity churn (nodes reappearing under ever-new names, one per run or per
namespace) would grow the tracker's map without a bound if nothing capped it. An
unsuppressed, still-dead entry is never reclaimed by age - `prune_grace` is the only reclaim
path there is, and it only ever applies to a durably-suppressed key - so `tracked_node_cap`
is what actually bounds memory: specifically, the DEPARTED subset of it (identities carrying
a nonzero miss count). A PRESENT entry never counts against the cap and is never evicted to
make room for anything, at any map size: it costs nothing to keep (an idle entry is simply
re-tracked a moment later if it is still armed), and it is bounded by the live graph, not by
churn, which is bounded by reality rather than by this cap. So a graph carrying far more
nodes than `tracked_node_cap` is not, by itself, capacity pressure at all - every one of them
is still tracked and every genuine death among them still reported; only a departed set that
is itself larger than the cap is.

At that point, the cap collapses departed entries into one synthetic count, keeping at most
three individually named - but ONLY entries that have actually crossed `miss_grace` (a
confirmed death). An entry that is merely mid-grace is never a collapse candidate: folding it
into the count would report a death the node has not earned, permanently, since collapsing
erases the identity and a node returning cannot un-report what was never real. Under
sustained pressure from still-maturing churn the departed set may therefore exceed
`tracked_node_cap` for a few ticks - bounded by how fast new departures arrive times
`miss_grace`, not by how long the process has been running, which is the growth this cap
exists to prevent in the first place. `tracking_saturated` reports exactly this: the departed
set still exceeds `tracked_node_cap` even after collapsing every confirmed death it safely
could, because immature entries alone account for the excess.

Unlike `lifecycle_expectation`'s identical-looking cap, a PRESENT/armed key here is never the
thing evicted to free a slot. `LifecycleExpectationTracker` carries two clocks per node, so a
node can be simultaneously present and mid-violation - a third state this tracker does not
have, and what makes evicting a present entry there lose nothing (the evidence lives in the
clocks). `NodeLivenessTracker` carries exactly one piece of per-key state (a miss count), so
a tracked identity is always either present-and-empty or absent-and-evidential; evicting a
present one here would only ever discard a key that could simply be re-admitted next tick
anyway, at the cost of losing any death that happens to land in the eviction window - only
ONLINE nodes re-enter `armed`, so an evicted-while-present node that then dies is never
re-admitted at all. The sibling's eviction order does not transfer.

**The boundary with `lifecycle_expectation`.** `GRAPH_NODE_INACTIVE` and
`GRAPH_NODE_DISAPPEARED` can both be raised for the same node at the same time, and that is
CORRECT, not a defect this detector removes: `GRAPH_NODE_INACTIVE` says a required node is
present but not active, `GRAPH_NODE_DISAPPEARED` says a node is gone, and an operator facing
each has a different repair - reactivate it, or find out why it left. A node CONFIRMED
`GRAPH_NODE_INACTIVE` that then departs keeps that fault (its description switches to say
the node has since left the graph) AND now also raises `GRAPH_NODE_DISAPPEARED`, since this
detector tracks every App it owns independently of whatever `lifecycle_expectation` thinks of
it.

What changed is narrower than that, and it only applies to a node this detector could ever
have tracked. Before this detector existed, `lifecycle_expectation`'s own violation streak
let a SUSTAINED absence mature an unconfirmed (below-`grace`) streak into a confirmed
`GRAPH_NODE_INACTIVE` - the only way a node stuck in a restart loop, never observed long
enough to cross `grace` while present, would ever be reported at all. Where this detector
can independently report the same departure - which needs the presence detector to have OWNED
the node at least once, since `node_death` only ever tracks an App the reliability gate has
admitted for ownership, and ownership of a MANAGED node is EARNED only by a lifecycle state
read as `active` (a state nobody could read earns nothing: it is granted provisionally and given
up again the moment a label arrives) - that absence-alone maturity is gone: absence CONTINUES a violation that has
already matured past `grace` (the fault stays raised, still naming the node), but no longer
CREATES one that has not. A node that was briefly non-active and then died is reported as gone
(`GRAPH_NODE_DISAPPEARED`) rather than also acquiring an inactive fault born from ticks
gathered while nobody could observe it. A `require_active` node that never reaches `active` is
never owned, is never tracked here, and its departure can never raise
`GRAPH_NODE_DISAPPEARED`; the same holds for an app that is not online. For those,
`lifecycle_expectation` keeps the older behaviour: absence still matures a below-`grace`
streak, because it is structurally the only detector that will ever get to report them. See
`lifecycle_expectation`'s own "Absence continues the last CORROBORATED observation, it never
erases" section above for the mechanism.

**Repeated failures: what `occurrence_count` and the captured evidence answer, and don't.**
An operator asking "how many times did this node die in the last hour" reads it off the
fault record itself, not off this detector: `GRAPH_NODE_DISAPPEARED`'s own
`occurrence_count` (`GET /api/v1/apps/graph_watchdog/faults`, or the scoped
`GET /api/v1/apps/graph_watchdog/faults/GRAPH_NODE_DISAPPEARED`) starts at 1 on the first
raise and increments by exactly one each time a FAILED report reactivates a record that was
CLEARED - never merely re-raised while still CONFIRMED, and never merely HEALED. Healing
(the fault manager's debounce counter crossing `healing_threshold` on clean sweeps, see
"Closing the loop" above) and clearing are different things: `DELETE
/api/v1/apps/graph_watchdog/faults/GRAPH_NODE_DISAPPEARED` - the fault manager's own
`~/clear_fault` underneath - is what acknowledges an occurrence and closes its cycle. A
still-CONFIRMED, or still-HEALED-but-unacknowledged, fault that fires FAILED again is the
SAME occurrence continuing, not a new one, so restarting the dead node fast enough to heal it
before anyone acknowledges it will not move the count.

The honest limits, read off this branch's fault-manager storage rather than assumed: the
per-fault rosbag store enforces `fault_code` as UNIQUE, so a fault can hold at most one
recording at a time - a later confirmation's capture replaces the earlier one on disk rather
than accumulating a history. The freeze frame is the same shape: one row per `fault_code`,
overwritten on every capture, so a fifth occurrence's captured values overwrite the first's.
What survives every occurrence by default is the count itself; a hash-chained record of
every raise/clear/heal transition also exists, but only once the fault manager's own
`audit_log.enabled` is turned on, which it is not by default. Recordings and the freeze
frame do not accumulate a per-occurrence history either way.

**A second death while the first is still outstanding gets no evidence of its own.** This
detector folds every currently-affected node into ONE `GRAPH_NODE_DISAPPEARED` record (see
"The boundary with `lifecycle_expectation`" above) - so a node dying while another's death is
still CONFIRMED is added to the description, but the report that adds it is a re-report on an
already-active fault, not a new occurrence: the same distinction "Repeated failures" above
draws for one node dying twice applies just as much across two different nodes sharing the
one code. `occurrence_count` does not move, no state transition fires, and - for the same
reason "The honest limits" above gives - neither a freeze frame nor a recording is captured
for the second node; only whichever death actually confirmed the record has either.
Acknowledging between the two deaths avoids this: `DELETE
/api/v1/apps/graph_watchdog/faults/GRAPH_NODE_DISAPPEARED` closes the first occurrence, so the
second node's next report reactivates a CLEARED record instead of updating a CONFIRMED one - a
genuine new occurrence, with its own `occurrence_count` and its own capture.

**A fault outlives a gateway restart even if the node comes back, and only an acknowledgement
closes it.** Within one process this detector heals on return: the node reappears, the dead set
empties, a PASSED flows and the fault heals. Across a restart it does not, in either direction.
The restarted instance may clear `GRAPH_NODE_DISAPPEARED` only once it has ITSELF put a FAILED
on the wire for it, because until then it cannot tell "the node came back" from "I never saw
that node" - and clearing on the second reading is the ungated heal the guard exists to
prevent. So a fault CONFIRMED before a restart stays CONFIRMED afterwards whether or not the
node returned, and with the sqlite backend it survives every reboot until
`DELETE /api/v1/apps/graph_watchdog/faults/GRAPH_NODE_DISAPPEARED` acknowledges it. That is the
acknowledge model doing what it is for, not an accident: an occurrence is closed by an
operator, not by the passage of a restart.

Re-seeding the tracker from the fault store at startup would change it, and today it cannot be
done. `/fault_manager/list_faults` would tell this detector that a `GRAPH_NODE_DISAPPEARED`
record is outstanding and that it is among its reporting sources - but not WHICH nodes it
names, which is the only thing that would make a fresh instance's silence meaningful. The fault
manager keeps one record per `fault_code`, and this detector folds every dead key into that one
record's description, capped at `kMaxDescriptionChars` with the remainder collapsed into a
count, so the key list is not recoverable even by parsing prose. `lifecycle_expectation` has the
same boundary and resolves it differently (a bounded hold, then the clear flows) because its
`require_active` set is enumerable from config alone; this detector is zero-config over the
whole graph and has no such set. The integration test
`RestartedInstanceNeverClearsAFaultItDidNotRaiseEvenAsTheNodeReturns` pins the behaviour so it
cannot change silently.

**Test tiers.**

1. **Unit**: `test_node_liveness_tracker.cpp` pins the pure presence/absence state machine -
   a key present but never armed is never tracked; once armed, presence alone (not
   continued arming) keeps a tracked key's miss counter at zero; a key is reported dead only
   once misses exceed `miss_grace`; freshness ordering keeps a brand-new death out of a
   capped description's blind spot; `prune()` reclaims a key only once it has been
   suppressed for MORE than `prune_ticks` CONSECUTIVE calls, resets the streak the moment a
   veto lifts even once, and never reclaims an unsuppressed death no matter how long it
   stays dead. On the cap: a graph carrying ten times `tracked_node_cap` present keys is
   never refused or evicted, and every genuine death among them is still individually
   reported; two entries carrying only a below-grace miss each are never collapsed or
   reported under cap pressure, however much the departed count exceeds the cap, and
   `tracking_saturated` reports exactly that condition; matured (confirmed-dead) entries ARE
   collapsed into a monotonically-accumulating count once the departed set exceeds the cap
   and no present or immature entry is available to spare; and at `miss_grace: 0`, where
   every departure matures instantly, saturation never fires at all - a narrower, honestly-
   scoped property than a blanket "can never saturate" claim. `test_suppressor.cpp`
   pins the `Suppressor` interface (`durable()` defaults false) and the free
   `apply_suppressors()` helper (order-independent, null-safe, returns the dropped count) -
   this detector's own filtering is hand-inlined rather than a call to that helper, because
   it layers an id-form check the helper's plain string-keyed signature cannot express.
   `test_allowlist_suppressor.cpp` and `test_lifecycle_shutdown_suppressor.cpp` pin each
   suppressor's own matching rules independently of any detector.
2. **Integration** (`test_node_death_integration.cpp`): the full config contract
   (`miss_grace`, `prune_grace` and `tick_interval_ms` range checks and their floor
   interaction; malformed `allowlist`/`suppress` entries; `tracked_node_cap` range checks,
   including a value far past `INT_MAX` rejected rather than wrapped; an unknown top-level
   key), that a peer-aggregated app and an `is_online: false` app are never tracked, that
   the tracked count stays bounded and shrinks after a reclaim under sustained identity
   churn (and, without any suppression configured at all, that `tracked_node_cap` still
   bounds memory under unsuppressed churn while the fault stays raised), that the
   `prune_grace` clamp is not merely never-reclaimed-yet but actually reclaims once its
   clamped horizon passes, that Advisory mode keeps accumulating misses (provable by
   switching to Raise mode without letting the node return and observing an immediate raise)
   rather than merely declining to send, that a clean managed-lifecycle departure is
   suppressed exactly AT the `miss_grace` boundary and stays reclaimed - not re-raised - past
   the retention window the plugin sizes for it, that the allowlist's id-form suppresses only
   the colliding node and not its namesake, and the ungated-clear guard's four properties: a
   stored fault stays unhealed while an unrelated node arms alone, a clear flows once this
   process instance has itself handed a FAILED request to the fault client, the guard tracks
   that handoff rather than intent (an attempted-but-never-sent raise never earns a clear),
   and a reconfigure while the node is absent does not withhold a clear the process had
   already earned. Three more rows sit past the cap specifically: 513 armed nodes against the
   default `tracked_node_cap` (512) still report the one that dies; three below-grace entries
   under `tracked_node_cap: 2` never fabricate a death and still mature into a genuine one
   once the pressure passes; and a cap-forced collapse stays raised through a genuine
   recovery but clears once reconfigured, mirroring the ungated-clear guard's own reconfigure
   test for an ordinary (uncollapsed) fault.
3. **E2e**, four files, each launching its own real gateway + fault_manager + demo-node
   stack:
   - `test/e2e/test_node_death_e2e.test.py` (ten scenarios): raise and name the node; clear
     once it returns; no heal with the fault manager's healing disabled; a lifecycle
     DEACTIVATE is never mistaken for a death; a manifest app that never comes online is
     never called dead; a `_ros2cli_*`-prefixed node is never tracked, checked against
     ros2cli's own naming constant; a bare-name collision names only the node that actually
     exited; a fast tick alone, nothing else perturbing the graph, raises nothing; a
     three-cycle restart loop, acknowledged between cycles, reaches `occurrence_count: 3`;
     and a fault confirmed before a gateway restart stays CONFIRMED across it.
   - `test/e2e/test_node_death_suppression_e2e.test.py` (five scenarios): the allowlist
     suppresses the node it names; the SAME allowlist left unnamed in `suppress` is inert,
     and a startup warning says so; a managed lifecycle node that reached a clean shutdown
     is never named while an active sibling that was simply killed still is; naming a node
     on the allowlist alone, with `suppress` unset, does not suppress it; and an ALLOWLISTED
     death (the only shape pruning ever applies to - see "Bounded by evidence, not by age")
     held well past `prune_grace` is actually reclaimed (`tracked_count` on
     `GET /x-medkit-watchdog` drops to 0, not merely "no fault ever appeared", which a
     deleted `prune()` would also produce), while never once raising `GRAPH_NODE_DISAPPEARED`
     for it - proving pruning a suppressed key's bookkeeping is not itself an event the
     aggregate reacts to.
   - `test/e2e/test_node_death_boundary_e2e.test.py` (eight scenarios, the seam with
     `lifecycle_expectation`): a node stuck inactive but never gone is
     `GRAPH_NODE_INACTIVE`'s alone; the same node, ARMED first, then killed while still below
     `grace`, raises `GRAPH_NODE_DISAPPEARED` alone, with no `GRAPH_NODE_INACTIVE` ever born
     from the departure; a node killed AFTER `GRAPH_NODE_INACTIVE` has confirmed raises BOTH
     codes at once, the confirmed one still naming the node; a healthy node that is simply
     killed raises `GRAPH_NODE_DISAPPEARED` alone; a restart-looping required node is caught
     every cycle regardless of whether it ever matures under `lifecycle_expectation`; a node
     that is NEVER armed, killed while still below `grace`, raises `GRAPH_NODE_INACTIVE`
     alone instead - `node_death` cannot track a node the gate never admitted for ownership,
     so absence has to mature the violation here; a large `miss_grace` delays the report but does not swallow
     it; and a restarted gateway's own warmup window never produces a spurious PASSED for a
     node it has not yet re-measured.
   - `test/e2e/test_presence_ownership_e2e.test.py` (seven scenarios, presence ownership
     against a node whose `GetState` never answers - the boundary file's B6 row covers the
     measured-not-active side, where the unread window is a race rather than a permanent
     state): a managed node the watcher asked and could not read, killed, raises BOTH
     `GRAPH_NODE_DISAPPEARED` and `GRAPH_NODE_UNREADABLE`; the same death with NO
     `require_active` entry anywhere - the shipped default, where nothing else is watching -
     still raises `GRAPH_NODE_DISAPPEARED`, and that pair also rules out an implementation
     deciding ownership from `require_active` membership; the same node told to start
     answering, measured `active`, then killed, is reported too, under the identical
     configuration as the first row so the only variable is the lifecycle state; a node
     measured `active` that then LOSES its lifecycle services and is killed is still
     reported; and with an unmeasured managed node in the graph `GRAPH_PARAM_DRIFT` still
     names it (the one leg of the three that passes through the app-keyed gate at all),
     while `GRAPH_QOS_MISMATCH` and `GRAPH_ORPHAN`, both keyed by topic, still report too.
     The sixth is the one that asserts an ABSENCE: the same never-answering node, owned
     provisionally once the asking stops, is then told to announce `inactive` and killed -
     `GRAPH_NODE_INACTIVE` names it and `GRAPH_NODE_DISAPPEARED` never appears, which is also
     the row that rejects a detector wired to the permissive `reliability_allows()`. The
     seventh is a crash loop shorter than the warmup: killed, reported, back for less than one
     re-warm, killed again - the second death must still be reported, and no PASSED may be
     emitted while the node is dead, which is what a key handed back for merely re-warming
     would produce.

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
  tick loop. A tracked id whose binding moved (a different node behind the same
  app id) is dropped and re-seeded from scratch rather than keeping the old
  node's label. Those `~/transition_event`
  subscriptions are kept out of the gateway's ROS executor (own callback group,
  own single-threaded executor that the plugin's tick thread drains between
  ticks), so they are only ever created, run and destroyed on that one thread.
  Non-managed nodes are never gated. Neither is a managed node whose `GetState`
  has never answered: its label stays EMPTY, and the gate answers PERMISSIVELY
  for it on purpose, because gating on an unread label would silence every
  detector for a node whose lifecycle service is broken - `qos_mismatch`,
  `orphan` and `param_drift` included. Only a KNOWN non-active label suppresses.
- **Presence ownership** is a second, stricter question the same gate answers,
  and only `node_death` and `lifecycle_expectation` ask it. "May this entity
  raise" is not "will the presence detector be able to report this node's
  departure". The latter is true for a node with no managed record at all (a
  plain node) and for one reading `active`. An unread label is ignorance rather
  than a state, so it answers no - but only while that ignorance can still
  resolve. `LifecycleWatcher` charges a bounded GetState re-seed budget per node,
  and charges it only for a read that actually ran, so an empty label with
  attempts left means "we have not finished asking" and an empty label with the
  budget spent means "we asked and failed". Past that point the answer flips back
  to yes, because `lifecycle_expectation` is the only other detector that could
  report such a node and it looks at nothing unless an operator named the node in
  `require_active`, which is empty by default: withholding ownership forever
  would be a silence, not a handover.
  That readmission is PROVISIONAL, and the distinction is the whole of the rule:
  **stickiness is earned by knowledge, never by ignorance.** The
  `~/transition_event` subscription outlives the seed budget, so a label can
  still arrive without being asked for; one that reads non-active says the node
  was `lifecycle_expectation`'s all along, and `node_death` releases it while it
  is still alive rather than reporting a death later. Ownership EARNED from a
  measurement (or from a node with no lifecycle at all) is never released that
  way: a node that went `active`, deactivated and then died is still
  `node_death`'s to report. `lifecycle_expectation` latches only the EARNED
  ground, and ANDs it with the node being ONLINE, because `node_death` skips an
  offline app before it ever consults the gate. Both detectors read the one
  shared answer on the same tick rather than each guessing from a label.
  The two NEGATIVE answers are kept apart for the same reason the positive ones
  are: `unclaimed` means nothing is known yet (still warming up, or still being
  asked about) and `disowned` means the graph has said whose node this is. Only
  `disowned` may take a key away from a detector already holding it, and the
  label decides it before arming does - a node that restarts is un-armed for its
  whole re-warm, and reading that as a verdict would drop the key just in time
  for the node's next death to be reported by nobody.
  **The hand-back is not instantaneous, and the gap is stated rather than
  hidden.** `node_death` releases a provisionally owned key inside its own tick,
  and only on a tick that still sees the app present, so a node that dies within
  about one entity-cache refresh of its label arriving is reported by the
  presence code after all - measured at 210 ms between the label reaching
  `GET /x-medkit-watchdog` and the release. Once the release HAS happened it
  holds, and that took fixing: a dying managed node loses its lifecycle services
  from the snapshot a sweep before the App itself goes, and a node with no
  managed record reads as a plain node, which is `kEarned`. The act of dying was
  therefore erasing the measurement that disowned the node and walking the key
  straight back in. A released key now waits for the graph to read `active`
  again; the disappearance of what disowned it is not news that the node became
  the presence detector's. The report is TRUE (the node did
  disappear); what the window costs is attribution, and where `require_active`
  names the node both codes stand, which the boundary already accepts elsewhere.
  Closing it would mean deciding at REPORT time, and that cannot be done from
  what survives a departure: the departed record carries only the last label,
  which reads `inactive` both for a node that was earned and then deactivated
  (must be reported) and for one that was only ever provisional (must not), so
  telling them apart needs per-key ownership history outliving the departure -
  the unbounded state the tracked-key prune exists to prevent.
  What this does NOT close: an
  App counts as managed only once the snapshot shows it advertising a
  `GetState`-typed service, and until then it is indistinguishable from a plain
  node, so a managed node whose services have not yet been discovered can still
  be taken for a plain one. That window is bounded by the arming warmup, which
  already requires several consecutive ticks of presence, and both the node and
  its services come from the same introspection sweep. The window this DOES
  close is the unbounded one: a managed node whose state stays unmeasured for as
  long as its `GetState` goes unanswered, which is not a startup artifact and
  does not close on its own. Nor does any of it change what arming already
  decided: a node that appears, lives fewer than `warmup_cycles` ticks and
  vanishes was never armed, so it is owned by nobody and reported by nobody -
  the bringup-quiesce trade-off working as designed, not a gap this boundary
  introduces.
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
    ],
    "detectors": {
      "lifecycle_expectation": {
        "tracking_saturated": false,
        "tracked_nodes": 12,
        "tracked_node_cap": 512
      }
    }
  }
}
```

`state` is `"armed"` only when both warmup is done and lifecycle (if managed)
is `active`; otherwise it is `"warming_up"`. `lifecycle` is the raw lifecycle
state label, or `null` for entities with no tracked lifecycle state.

`detectors` carries one block per detector that has something of its own to report, and is
omitted entirely when none does - for a condition that belongs to a single detector and
would otherwise exist only in the gateway log. `lifecycle_expectation` reports whether its
tracked-node cap is SATURATED, i.e. whether it has refused to track a required node because
every slot is held by a present node carrying evidence. `tracking_saturated: true` means a
required node is going unchecked and `GRAPH_NODE_INACTIVE`'s clear is withheld for as long
as it lasts; the fix is either a `require_active` entry matching fewer identities (a bare
name on a graph whose nodes respawn under ever-new namespaces matches an unbounded set) or a
larger `tracked_node_cap`, which is why the cap and the live count are reported beside it.

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
`GRAPH_PARAM_DRIFT`, `GRAPH_LATENCY_BUDGET`, plus three extensions of the frozen
namespace beyond the original six, all raised by `lifecycle_expectation`:
`GRAPH_NODE_INACTIVE` (a required node CONFIRMED non-active), `GRAPH_NODE_UNREADABLE`
(a required, managed node whose lifecycle label has never been read), and
`GRAPH_NODE_NOT_MANAGED` (a required node with no tracked lifecycle at all). The latter
two share one cause-blind unmeasured clock internally (see the detector section above)
but are always two DISTINCT codes on the wire - the clock's blindness to which cause it
is seeing never leaks into which fault code a node ends up reported under.

Of these nine, seven currently have a detector raising them: `qos_mismatch`
(`GRAPH_QOS_MISMATCH`), `orphan` (`GRAPH_ORPHAN`), `param_drift` (`GRAPH_PARAM_DRIFT`),
`node_death` (`GRAPH_NODE_DISAPPEARED`), and `lifecycle_expectation`'s three above. Two
remain undelivered: `GRAPH_TF_STALE` and `GRAPH_LATENCY_BUDGET`.

Splitting the unmeasured cases out of `GRAPH_NODE_INACTIVE` has two consequences nothing
else states. Anything downstream that filters or correlates on `GRAPH_NODE_INACTIVE`
alone no longer sees either unmeasured case at all - they live under
`GRAPH_NODE_UNREADABLE`/`GRAPH_NODE_NOT_MANAGED` now, different codes, not a
WARN-severity instance of the first. And a node moving from confirmed-inactive to
unmeasured (or back), or from one unmeasured cause to the other while its clock is still
climbing (before maturity), produces a HEAL on one code and a RAISE on another for the
SAME node, on adjacent ticks - separate fault_manager events, not one transition,
because none of the three faults share a record to transition within. A node whose
unmeasured clock has already MATURED under one cause does not flip fault codes merely
because the LIVE cause changes later, though (the sticky-cause design decision, see
above) - only a real measurement resetting the clock entirely can move it off the code
it matured under.
