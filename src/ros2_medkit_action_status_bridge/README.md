# ros2_medkit_action_status_bridge

Generic drop-in bridge that turns terminal ROS 2 action goal states into
structured medkit faults. It watches the `/<action>/_action/status`
(`action_msgs/msg/GoalStatusArray`) topic that **every** action server
publishes, so it works across Nav2, MoveIt 2, ros2_control's controller
actions, and any custom action with **no per-project code**.

## Why this matters

For the core ROS 2 league the authoritative "the goal failed" verdict lives on
the action-result channel, not on `/diagnostics` or `/rosout`:

- Nav2 `NavigateToPose` -> `GoalStatus = ABORTED`
- MoveIt `MoveGroup` -> `GoalStatus = ABORTED` (with `MoveItErrorCode` in the result)
- ros2_control controller actions -> aborted goals

Neither the diagnostic bridge nor the log bridge sees this. This bridge does,
generically, by observing the goal-status topic.

## What it does

- Discovers every action on the graph by scanning for `*/_action/status`
  topics (re-scanned on a timer to catch actions that appear later, to drop
  actions that vanish, e.g. a Nav2 lifecycle deactivate or a one-shot node, and
  to retry any fault whose delivery to the FaultManager was deferred).
- `ABORTED (6)` -> fault (`SEVERITY_ERROR` by default).
- `CANCELED (5)` -> fault only if `canceled_is_fault` (off by default; cancel is
  usually intentional). When enabled it emits a `_CANCELED` code.
- `SUCCEEDED (4)` -> `PASSED` to heal the action's fault code (if enabled).
- `fault_code` is `<PREFIX>_<ACTION>_ABORTED` (or `_CANCELED`), e.g.
  `ACTION_NAVIGATE_TO_POSE_ABORTED`. `source_id` is the action **server's node
  FQN** (resolved from the status topic's publisher, e.g. `/bt_navigator`), so
  the fault associates with that node's SOVD entity; it falls back to the action
  name only if no publisher is visible. When the FQN is not yet discovered at the
  first report, the fault fires on time under the action-name fallback and is
  then **re-attributed** to the FQN once discovery resolves - the bridge
  re-reports it with `ReportFault.supersedes_source_id` set to the fallback, so
  the FaultManager drops the provisional source and the fault lands on the server
  node's entity (the fallback source alone would keep it out under the strict-AND
  per-entity scope filter).

## Per-action state, not per-goal

The fault is a property of the **action**, not of an individual goal. On every
status message the whole `GoalStatusArray` is scanned for the net state:

- if **any** goal is `ABORTED` (or `CANCELED` when `canceled_is_fault`), the
  action is **failed** (order of goals in the array does not matter);
- otherwise, if the array has terminal goals and none are failing, the action is
  **healthy**.

A fault is raised only on the `healthy -> failed` transition and healed only on
`failed -> healthy`. This means one failed goal cannot heal an action while
another goal is still failed, and a dropped terminal message cannot leave a
fault stuck. Per-goal dedup (keyed on `goal_id:status`) only suppresses
duplicate log lines; it never gates the transition.

The bridge tracks the latest observed state (the *desired* state) separately
from what the FaultManager was actually told. The transition is committed only
once the report is delivered: if the FaultManager service is not discovered yet
- e.g. an action's status topic is `transient_local` (latched) and its terminal
status reaches the bridge during the startup discovery window, before the report
client connects - the transition stays pending and is retried, rather than the
high-severity fault being silently dropped. (The latched status is delivered to
a subscription only once, so without this retry the dropped report would never
recover.) See [Delivery latency and freeze-frame](#delivery-latency-and-freeze-frame).

## Scope: the terminal verdict, not the reason

This bridge delivers the generic "it aborted" event. The action-specific
*reason* (e.g. `MoveItErrorCode.val = -26 START_STATE_IN_COLLISION`, Nav2
`error_code` on Iron/Jazzy) lives in the action result message and is a separate
enrichment concern (a future action-result reader using runtime message
introspection / dynmsg). Per-project plugins are only needed for human-readable
labels, not to surface the fault.

## Delivery latency and freeze-frame

The FaultManager captures the freeze-frame (topic snapshot, rosbag) **when it
receives the report**, so the value of that context decays with delivery
latency: a late fault is reported against state that has already moved on. The
bridge therefore optimises for delivering a fault to the FaultManager as soon as
physically possible.

- **Happy path (service discovered):** the fault is reported synchronously in
  the status callback, the instant the terminal status arrives. This is the
  normal runtime case and is already ASAP - nothing is deferred.
- **Degraded path (service not yet discovered):** a fault cannot physically be
  delivered before the FaultManager `report_fault` service is discovered (startup
  before the FaultManager is up, or a FaultManager restart). The transition is
  kept pending and retried on a dedicated **fast retry timer** (`retry_period_sec`,
  default 50 ms), decoupled from the slow discovery `rescan`. So the fault lands
  within one short tick of the service appearing - keeping the freeze-frame as
  contemporaneous as the discovery floor allows.

We deliberately do **not** block the (single-threaded) executor waiting for the
service: blocking would stall detection of *other* actions' faults during an
outage to shave the last few milliseconds off one fault - a bad trade when the
freeze-frame is already only ~tens of ms fresher. The retry timer is armed only
while a delivery is pending, so an idle bridge does no periodic work.

**Known boundaries** (narrow, and shrunk further by the fast retry; both are
consequences of the level-triggered, net-state model, not silent loss in the
common case):

- *Flap entirely within an outage:* if an action both fails **and** recovers
  while the report channel is undiscovered, the net observed state collapses to
  healthy and no retroactive raise+heal pair is emitted (there is nothing to
  freeze-frame after the fact anyway). A flap while the channel is up still
  produces a raise+heal per cycle as normal.
- *Vanish during an outage:* if a still-failed action disappears from the graph
  (its state is dropped, so it cannot be retried) at the exact moment the service
  is undiscovered, its heal cannot be delivered and the fault may remain active
  in the FaultManager. This is logged as a warning rather than dropped silently.
- *Latched historical fault at startup:* a goal that aborted before the bridge
  existed delivers its latched status on first subscribe; its freeze-frame
  reflects post-hoc startup state regardless of delivery speed, so prompt
  delivery just records that it happened.

## Run it

```bash
ros2 launch ros2_medkit_action_status_bridge action_status_bridge.launch.py
```

## Configuration (`config/action_status_bridge.yaml`)

| Param | Default | Meaning |
|-------|---------|---------|
| `aborted_severity` | `2` (ERROR) | severity of an aborted goal |
| `canceled_is_fault` | `false` | treat CANCELED as a fault |
| `heal_on_succeeded` | `true` | send PASSED on a successful goal |
| `rescan_period_sec` | `2.0` | how often to look for new actions |
| `retry_period_sec` | `0.05` | fast retry cadence for reports deferred while the FaultManager service is undiscovered (armed only while pending) |
| `code_prefix` | `ACTION` | prefix for generated codes |
| `exclude_actions` | `[]` | action-name substrings to skip |
| `include_only_actions` | `[]` | if set, only watch these |
| `dedup_capacity` | `4096` | remembered goal/status log keys |

`exclude_actions` / `include_only_actions` match as **unanchored substrings** of
the fully-qualified action name (e.g. `nav` matches `/navigate_to_pose`). Use a
longer fragment (or the full name) for exact targeting.

`aborted_severity`, `code_prefix` and `dedup_capacity` are range-checked and
normalized at load (out-of-range severity clamps to ERROR, a non-snake-case
`code_prefix` is upper-snake-cased, a non-positive `dedup_capacity` falls back to
the default), with a warning.

## Limitations

- **Flapping**: a retry loop that issues a fresh `goal_id` each attempt does not
  re-raise while the action stays failed (only net-state changes transition),
  but a true flap (fail -> succeed -> fail) does produce one raise + heal per
  cycle. There is no per-code rate throttle in this bridge; lower noise via the
  FaultReporter local filter if needed.
- **No per-code throttle**: every action-level transition is forwarded.
- **CANCELED heal semantics**: with `canceled_is_fault`, a canceled action heals
  on the next non-failing terminal (a later SUCCEEDED), or when the canceled goal
  ages out of the action server's retained status array and a non-failed terminal
  remains. It is not healed by an explicit "uncancel" because none exists.
- **QoS**: the bridge requests the standard action status QoS (reliable +
  transient_local). A non-standard server (volatile/best-effort) is logged with
  an incompatible-QoS warning rather than silently yielding zero faults.
- **Vanish while failed**: if an action that is currently failed disappears from
  the graph (lifecycle deactivate, one-shot node), the bridge heals its fault
  before forgetting it (when `heal_on_succeeded` is set), so it is not left stuck
  active. With healing disabled the fault persists by design.
- **Healing threshold**: the bridge emits one `PASSED` per recovery (a discrete
  event, not a stream), so a fault reaches `HEALED` only when the FaultManager's
  `healing_threshold` is `0`. With a higher threshold the fault stays `CONFIRMED`
  after the action recovers.
