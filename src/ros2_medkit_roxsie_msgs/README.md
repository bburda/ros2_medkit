# ros2_medkit_roxsie_msgs

Messages for the ROXSIE integration: PLC alarm events into ROS 2, and aggregate diagnostics
state back into the PLC.

## Overview

ROXSIE generates the PLC blocks and a ROS 2 package for the data you declare in the YAML
config. That covers process values. It does not cover alarms, because alarms live in the CPU
message system and not in a data block.

So on a machine with a SIMATIC PLC and ROS 2 you still have two separate lists of what is
wrong. To join them today you either read alarm bits over a protocol and rebuild their
meaning from an engineering export, or someone copies the alarms by hand into a second list
on the panel. Both drift.

This package holds the two message definitions for that exchange. Alarms declared in the TIA
project arrive in ROS 2 as faults, with the values the alarm system latched when the alarm
fired. Back the other way, the PLC gets four numbers for the signal column and for machine
logic.

Those four numbers cover every active fault in scope, not only the ones that came from the
PLC. A navigation fault on the robot moves the same severity and the same bits.

The two directions are not the same size, on purpose. Every alarm goes out with its full
context. Back the other way we send four numbers.

Alarm text has no fixed length and changes with the project. A PLC that handles strings
loses determinism, and determinism is why it is there. Writing the fault list into the PLC
would give us two lists, and two lists drift.

The PLC only needs something it can switch on: a lamp for the operator, a bit for an
interlock. Whoever wants the detail reads it over REST.

## Messages

### AlarmEvent.msg

One alarm event from the CPU alarm system, published by the ROXSIE generated node.

| Field | Type | Description |
|-------|------|-------------|
| `alarm_id` | uint32 | Numeric alarm identifier from the TIA project |
| `source` | string | Block or instance that declared the alarm |
| `alarm_class` | string | Alarm class from the project, e.g. whether it needs an ack |
| `state` | uint8 | `STATE_COMING` / `STATE_GOING` |
| `ack_state` | uint8 | `ACK_UNACKNOWLEDGED` / `ACK_ACKNOWLEDGED` |
| `plc_timestamp` | builtin_interfaces/Time | Event time from the PLC clock, UTC |
| `associated_values` | float64[] | Values the alarm system latched when the alarm fired |

Alarm text does not travel at runtime. The generator writes an alarm number to text table
next to the code, and the bridge looks the text up on the ROS 2 side.

`STATE_GOING` moves the fault towards healing. We keep the fault, so the history stays for
the audit trail, the same way the alarm system keeps a condition until someone acks it.

`ack_state` is one shared value. The operator acks on the panel or a client acks over REST,
and both sides see the same thing.

### DiagnosticsStatus.msg

Aggregate diagnostics state towards the PLC, consumed by the ROXSIE generated node and
written into a consumed data block.

| Field | Type | Description |
|-------|------|-------------|
| `heartbeat` | uint16 | Liveness counter of the diagnostics layer, about 1 Hz |
| `aggregate_severity` | uint8 | Highest severity in scope, `Fault` SEVERITY_* semantics |
| `active_fault_count` | uint16 | Active faults in scope |
| `class_bitmask` | uint16 | Coarse fault classes for machine logic |
| `scope` | string | Which entity in the SOVD tree this status covers |

When `heartbeat` stops, a watchdog in the PLC program raises its own alarm. The plant sees
that diagnostics are down instead of a green light that means nothing.

`scope` lets one diagnostics instance serve several cells without them seeing each other's
faults.

## Related packages

- `ros2_medkit_msgs` - the core fault model these events are mapped onto
- `ros2_medkit_fault_manager` - fault aggregation, debounce and freeze-frame capture
- `ros2_medkit_diagnostic_bridge` - the same bridging pattern for ROS 2 `/diagnostics`

## License

Apache-2.0
