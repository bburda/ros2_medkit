# ros2_medkit_plc_msgs

Message definitions for exchanging PLC alarms and aggregate diagnostics state with ROS 2.

## Overview

A machine that combines a PLC and ROS 2 has two fault stories that never meet. The PLC
has a mature alarm system: numbered alarms, coming and going states, acknowledgement, and
values latched at the moment the alarm fired. ROS 2 has faults, freeze-frames and a
diagnostics tree.

This package defines the interface between them. Alarms declared in a PLC project arrive
in ROS 2 as structured events and become faults; the diagnostics side sends back one small
status word for the signal column and for machine logic.

Two directions, deliberately asymmetric:

- **Alarms flow out in full** - every alarm event becomes a `PlcAlarm` message.
- **Only a status word flows back in** - `PlcDiagnosticsStatus` carries four fixed values,
  no fault list and no strings the PLC would have to build.

Fault text is dynamic and unbounded, and string handling inside a PLC costs determinism.
A fault list inside the PLC would also be a second copy that drifts from the first. What
the machine needs from the diagnostics side is a condition it can act on; everything
richer is read over REST by whoever needs it.

## Messages

### PlcAlarm.msg

One alarm event as reported by a PLC alarm system.

| Field | Type | Description |
|-------|------|-------------|
| `alarm_id` | uint32 | Numeric alarm identifier from the PLC project |
| `source` | string | Block or instance that declared the alarm |
| `alarm_class` | string | Alarm class from the project, e.g. whether acknowledgement is required |
| `state` | uint8 | `STATE_COMING` / `STATE_GOING` |
| `ack_state` | uint8 | `ACK_UNACKNOWLEDGED` / `ACK_ACKNOWLEDGED` |
| `plc_timestamp` | builtin_interfaces/Time | Event time from the PLC clock, UTC |
| `associated_values` | float64[] | Values latched when the alarm fired |

Alarm texts are not carried at runtime. The mapping from alarm number to text is static,
exported once from the engineering project, and resolved on the ROS 2 side.

A `STATE_GOING` event is not a delete. It moves the fault towards healing, the same way an
alarm system keeps a condition until it is acknowledged, so the history survives for the
audit trail.

`ack_state` is the shared acknowledgement state. Whoever acknowledges first, every other
consumer sees the same value rather than keeping a private copy.

### PlcDiagnosticsStatus.msg

Aggregate diagnostics state sent towards a PLC.

| Field | Type | Description |
|-------|------|-------------|
| `heartbeat` | uint16 | Liveness counter of the diagnostics layer, about 1 Hz |
| `aggregate_severity` | uint8 | Highest severity in scope, `Fault` SEVERITY_* semantics |
| `active_fault_count` | uint16 | Active faults in scope |
| `class_bitmask` | uint16 | Coarse fault classes for machine logic |
| `scope` | string | Which entity in the diagnostics tree this status covers |

A stalled `heartbeat` lets a watchdog in the PLC raise its own alarm, so the plant sees
that diagnostics are offline instead of a stale healthy state.

`scope` lets one diagnostics instance serve several cells without them seeing each other's
faults.

## Related packages

- `ros2_medkit_msgs` - the core fault model these events are mapped onto
- `ros2_medkit_fault_manager` - fault aggregation, debounce and freeze-frame capture
- `ros2_medkit_diagnostic_bridge` - the same bridging pattern for ROS 2 `/diagnostics`

## License

Apache-2.0
