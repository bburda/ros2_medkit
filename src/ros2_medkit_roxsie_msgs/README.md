# ros2_medkit_roxsie_msgs

Messages for the ROXSIE integration: PLC alarms into ROS 2, and one status value back
into the PLC.

## Overview

ROXSIE generates the PLC blocks and a ROS 2 package for the data you declare in the YAML
config, exchanged over shared memory on the host. That covers process values. It does not
cover alarms, because alarms live in the CPU message system and not in a data block.

So on a machine with a SIMATIC PLC and ROS 2 you still have two separate lists of what is
wrong. To join them today you either read alarm bits over a protocol and rebuild their
meaning from an engineering export, or someone copies the alarms by hand into a second
list on the panel. Both drift.

This package holds the messages for that exchange. Alarms in the PLC arrive in ROS 2 and
become faults with freeze-frames. Back the other way, the PLC gets one value it can act on.

## A picture, not a stream

`AlarmList` carries every alarm that is active or waiting for acknowledgement, published
cyclically. That is a deliberate choice.

The transport is cyclic anyway: the PLC-side node reads a data block at a fixed rate, so a
picture is what it naturally has.

And a picture survives a restart. Whoever restarts, the next cycle carries the full truth
again. An event stream loses whatever happened while it was away, and the consumer cannot
tell an empty stream from a healthy machine. That gap is not hypothetical: reading alarms
over OPC UA has the same hole, because no table of pending alarms is served on reconnect.

The bridge compares consecutive pictures. An entry that appears becomes a fault, an entry
that disappears heals it.

## Two ways to fill the list

**From bits.** The customer declares alarm bits in a data block and ROXSIE carries them
like any other data. Nothing new is needed on the generator side, so this works today.
`alarm_class`, `priority`, `producer`, `plc_timestamp` and `associated_values` stay empty,
and the freeze-frame comes from the process topics instead. Acknowledgement is a bit, so
either side can set it.

**From the CPU alarm system.** Generated code reads the alarm system with `Get_Alarm` and
mirrors it into the data block. Still no OPC UA and no license. This fills everything: the
event time from the CPU clock, the alarm class, the priority the plant already engineers
with, and the values latched when the alarm fired.

It also picks up alarms nobody declared. The CPU message system carries system diagnostics,
ProDiag, GRAPH, motion and security alarms, and `producer` says which one an entry came
from. A module fault or a drive alarm arrives without any extra engineering.

The trade is acknowledgement. The CPU owns the acknowledged state and has no per-alarm
acknowledge instruction. `Ack_Alarms` takes no alarm id, `MODE = 1` confirms every pending
alarm at once, in batches of 100. So the bridge never calls it. Acknowledging one alarm
needs the panel or OPC UA, and we mirror the state rather than write it.

## Messages

### Alarm.msg

One alarm, as one entry of the list.

| Field | Type | Description |
|-------|------|-------------|
| `alarm_id` | uint32 | Numeric alarm identifier from the PLC project, unique within a CPU, keyed with the PLC |
| `source` | string | Block or instance that declared the alarm |
| `alarm_class` | string | Alarm class from the project, empty in the bit path |
| `priority` | uint8 | 0 to 16 as the PLC knows it, higher is more important |
| `producer` | string | `user_program`, `system_diagnostics`, `prodiag`, `graph`, `motion`, `security` |
| `active` | bool | Whether the condition is currently present |
| `acknowledged` | bool | Whether the alarm has been acknowledged |
| `plc_timestamp` | builtin_interfaces/Time | Last state change, from the PLC clock, UTC |
| `associated_values` | float64[] | Values latched when the alarm fired, 512 bytes total in the CPU |

Two flags rather than one enum, because an alarm that is gone but unacknowledged is a real
state and stays in the list. Those are the same two bits the CPU alarm system tracks.

Alarm text does not travel. The text belongs to the project, changes with the project, and
is resolved on the ROS 2 side from a table that ships with the deployment. That table comes
from a TIA Openness export of the project's PLC alarm text lists, and the bridge mapping is
generated from the same export.

### AlarmList.msg

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | builtin_interfaces/Time | When this picture was taken |
| `catalog_version` | string | Hash of the alarm catalog the list was built from, compared by the bridge against its mapping |
| `alarms` | Alarm[] | Alarms active or not yet acknowledged, 64 in the current generator, configurable in the next |
| `truncated` | bool | The CPU had more pending alarms than the block holds |

### DiagnosticsStatus.msg

| Field | Type | Description |
|-------|------|-------------|
| `heartbeat` | uint16 | Liveness counter of the diagnostics layer, about 1 Hz |
| `condition` | uint8 | `OK`, `ABNORMAL`, `ACTION_REQUIRED`, `STOP` |
| `class_bitmask` | uint16 | Optional, coarse fault classes for interlocks |
| `scope` | string | Which entity in the SOVD tree this covers |

`condition` says what we are asking of the machine, not what to light up. Machines already
have a state machine, and where PackML is used, a block that turns machine state into
signal column bits. We hand over a condition and let that logic decide.

When `heartbeat` stops, a watchdog in the PLC program raises its own alarm, so the plant
sees that diagnostics are down instead of a green light that means nothing.

`condition` covers every active fault in scope, not only the ones that came from the PLC. A
navigation fault on the robot moves the same value.

## Related packages

- `ros2_medkit_msgs` - the core fault model these alarms are mapped onto
- `ros2_medkit_fault_manager` - fault aggregation, debounce and freeze-frame capture
- `ros2_medkit_diagnostic_bridge` - the same bridging pattern for ROS 2 `/diagnostics`

## License

Apache-2.0
