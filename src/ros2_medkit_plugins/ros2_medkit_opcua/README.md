# ros2_medkit_opcua

Gateway plugin that bridges OPC-UA capable PLCs (OpenPLC, Siemens S7, Beckhoff, Allen-Bradley, etc.) into the SOVD entity tree. Enables unified diagnostics for mixed ROS 2 + industrial PLC deployments through a single REST API, with PLC alarms routed to `ros2_medkit_fault_manager` and numeric PLC values optionally bridged to ROS 2 `std_msgs/Float32` topics.

Follows the same plugin pattern as `ros2_medkit_graph_provider`: implements `GatewayPlugin` + `IntrospectionProvider` against the `get_routes()` plugin API, loaded at runtime by `ros2_medkit_gateway` via `dlopen`.

## What it does

- Connects to any OPC-UA capable PLC server over `opc.tcp`
- Emits SOVD entities (area, component, apps) from a YAML-driven node map, or
  discovers them automatically by browsing the live address space
  (`auto_browse`, see below) - zero node-map config required
- Exposes PLC values as the `x-plc-data` vendor collection
- Allows writing setpoints via `x-plc-operations` with type-aware coercion and range validation
- Reports the connection state and poll metrics via `x-plc-status`
- Maps threshold-based PLC alarms to SOVD faults on the owning entity
- Optionally publishes numeric PLC values to ROS 2 `std_msgs/Float32` topics

## Architecture

```
                    OPC-UA (TCP :4840)
PLC Runtime  <─────────────────────────>  OPC-UA Plugin (.so)
  IEC 61131-3 program                      │
  Cyclic execution (100ms)                 │  Polls all configured nodes
  Variables exposed as OPC-UA nodes        │  Maps to SOVD entity tree
                                           │  Alarm thresholds -> fault reporting
                                           │
                                           ▼
                                    ros2_medkit Gateway
                                      REST API :8080
                                           │
                                    ┌──────┴──────┐
                                    │             │
                              SOVD REST      Fleet Gateway
                              (direct)       (aggregation)
                                    │             │
                                Dashboard    Multi-device view
```

The plugin connects to any PLC with an OPC-UA server over TCP. No ROS 2 dependency between the plugin and the PLC - communication is pure OPC-UA. The plugin is loaded by the gateway at runtime via `dlopen()` and registers vendor REST endpoints for PLC data access and control.

## SOVD Entity Model

The plugin creates a hierarchical entity tree from a YAML node map configuration:

```
Area: plc_systems
  └── Component: openplc_runtime
        ├── App: tank_process
        │     Data: tank_level (mm), tank_temperature (C), tank_pressure (bar)
        │     Faults: PLC_HIGH_TEMP, PLC_LOW_LEVEL, PLC_OVERPRESSURE
        ├── App: fill_pump
        │     Data: pump_speed (%)
        │     Operations: set_pump_speed
        └── App: drain_valve
              Data: valve_position (%)
              Operations: set_valve_position
```

## Asset identity (x-medkit.identity)

The plugin auto-populates the PLC component's asset-identity nameplate from the
live server and serves it as `x-medkit.identity` on the component (see
`docs/api/rest.rst` for the JSON shape). Two sources are read, best-effort:

- `Server/ServerStatus/BuildInfo` (present on every compliant server):
  `ManufacturerName`, `ProductName`, `SoftwareVersion`, `BuildNumber` (carried
  as the `extra.buildNumber` key).
- The OPC UA DI companion nameplate (`http://opcfoundation.org/UA/DI/`, only
  when the server exposes that namespace): `Manufacturer`, `Model`,
  `SerialNumber`, `HardwareRevision`, `SoftwareRevision` from the first device
  under `Objects/DeviceSet` that carries identification properties. DI values
  are device-specific and win over the server-level BuildInfo per field.

The read happens once per session - on the first introspect after a connect -
and is refreshed after every reconnect: the cached nameplate is tied to the
OPC UA session, so a PLC reboot, firmware update, or device swap is picked up
on the next discovery refresh instead of latching the first read forever.

**Trust-gated precedence.** How the live nameplate ranks against a
hand-authored manifest `identity:` block depends on whether the session
authenticates the server:

- Secured channel (`security_mode: Sign`/`SignAndEncrypt`) **and**
  `reject_untrusted: true`: the component is tagged with the protocol source
  `opcua`, which outranks `manifest` in the identity merge - an authenticated
  live read overrides stale manifest fields.
- Anything else (`SecurityPolicy=None`, or `reject_untrusted: false` /
  accept-any cert): the nameplate is spoofable by a rogue endpoint, so the
  component keeps the generic `plugin` source, which ranks below `manifest` -
  the read fills fields the operator left empty but never overrides them.

Per-field `_provenance` records `opcua` in both cases, so consumers always see
where a value was read even when it merged with low authority.

Note: identity (model, serial number, firmware/software versions) is served
unauthenticated on the default gateway configuration (`auth.enabled` defaults
to `false`) like every other discovery resource - it is a device fingerprint,
useful for reconnaissance. Enable gateway authentication (`auth.enabled: true`)
or front the API with an authenticating proxy to gate access to it.

## REST API

### Vendor Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/apps/{id}/x-plc-data` | All OPC-UA values for entity (with units, types, timestamps) |
| GET | `/apps/{id}/x-plc-data/{name}` | Single data point value |
| POST | `/apps/{id}/x-plc-operations/set_{name}` | Write value to PLC (`{"value": 75.0}`) |
| GET | `/components/{id}/x-plc-status` | Connection state, poll stats, active alarms |

### Standard SOVD (provided by gateway)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/v1/areas` | Lists `plc_systems` area |
| GET | `/api/v1/components` | Lists `openplc_runtime` component |
| GET | `/api/v1/apps` | Lists PLC applications (tank_process, fill_pump, etc.) |
| GET | `/api/v1/apps/{id}/faults` | Active PLC alarms mapped to SOVD faults |

### Example Responses

**Live PLC data:**
```json
GET /api/v1/apps/tank_process/x-plc-data

{
  "entity_id": "tank_process",
  "connected": true,
  "timestamp": 1774185903,
  "items": [
    {"name": "tank_level", "value": 742.5, "unit": "mm", "data_type": "float", "writable": true},
    {"name": "tank_temperature", "value": 31.8, "unit": "C", "data_type": "float", "writable": true},
    {"name": "tank_pressure", "value": 2.95, "unit": "bar", "data_type": "float", "writable": false}
  ]
}
```

**Write to PLC:**
```json
POST /api/v1/apps/fill_pump/x-plc-operations/set_pump_speed
{"value": 80.0}

{"status": "ok", "operation": "set_pump_speed", "node_id": "ns=2;i=4", "value_written": 80}
```

**PLC connection status:**
```json
GET /api/v1/components/openplc_runtime/x-plc-status

{
  "component_id": "openplc_runtime",
  "connected": true,
  "endpoint_url": "opc.tcp://openplc:4840/openplc/opcua",
  "server_description": "OpenPLC Runtime",
  "mode": "poll",
  "poll_count": 142,
  "error_count": 0,
  "node_count": 5,
  "active_alarms": []
}
```

## Finding Node IDs on your PLC

The plugin identifies PLC tags by OPC-UA node IDs in the canonical string
format (`ns=N;i=M` numeric, `ns=N;s=tag` string, `ns=N;g=...` GUID, or
`ns=N;b=...` opaque). To discover the correct node IDs for a real PLC
without guessing, use one of the standard OPC-UA browser tools:

- **UaExpert** - free GUI browser from Unified Automation. Download at
  <https://www.unified-automation.com/downloads/uaexpert.html>. Connect
  to your PLC's `opc.tcp://` endpoint, navigate the address space tree,
  right-click any Variable node, copy the NodeId property into the YAML
  map below.

- **`asyncua` command line** - `pip install asyncua` and then
  `python -m asyncua.tools.uals -u opc.tcp://your-plc:4840` walks the
  address space from a terminal, no GUI required.

- **Vendor toolchains** - Siemens TIA Portal's OPC-UA configuration
  exports DB/variable node IDs in the `ns=3;s="..."` format. Beckhoff
  TwinCAT 3 XAE displays them as `ns=4;s=MAIN.Tank.level`. Allen-Bradley
  users typically deploy Kepware or Ignition as an OPC-UA gateway which
  auto-maps tag names.

`config/tank_demo_nodes.yaml` ships a commented example with
ready-to-paste templates for OpenPLC, Siemens S7-1500, Beckhoff TwinCAT,
Allen-Bradley via Kepware and KUKA KR C5. Copy one of those blocks as a
starting point.

## Configuration

### Node Map (YAML)

Maps OPC-UA NodeIds to SOVD entities. One file per PLC setup.

```yaml
area_id: plc_systems
area_name: PLC Systems
component_id: openplc_runtime
component_name: OpenPLC Runtime

nodes:
  - node_id: "ns=2;i=1"          # OPC-UA NodeId (numeric or string)
    entity_id: tank_process       # SOVD app this belongs to
    data_name: tank_level         # Data point name in REST API
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: true                # Allow writes via x-plc-operations
    min_value: 0.0                # Optional: range validation for writes
    max_value: 100.0
    alarm:                        # Optional: numeric threshold -> SOVD fault
      fault_code: PLC_LOW_LEVEL
      severity: WARNING
      message: Tank level below minimum
      threshold: 100.0
      above_threshold: false      # Alarm when value < threshold

  # Status-word bit decode: one integer register, one fault per named bit.
  # Mutually exclusive per node with `alarm:` / `fault_enum:`.
  - node_id: "ns=2;s=Pump.StatusWord"
    entity_id: pump
    data_name: status_word
    data_type: int
    status_word_width: 16          # optional: source register width in bits.
                                   # Masks off sign-extended high bits so a
                                   # signed 16-bit word read with its top bit
                                   # set does not fire spurious faults above
                                   # bit 15. Range 1..64. See the note below.
    status_bits:
      - bit: 3                     # bit position, 0 = least significant
        fault_code: PUMP_OVERLOAD
        severity: ERROR
        message: Pump motor overload
      - bit: 7
        fault_code: FILTER_DIRTY
        severity: WARNING          # message defaults to the fault_code

  # Fault-code enum: a register value selects at most one fault + text.
  - node_id: "ns=2;s=Vfd.FaultCode"
    entity_id: vfd
    data_name: fault_code
    data_type: int
    fault_enum:
      ok_value: 0                  # value meaning "no fault" (default 0)
      # Catch-all for a non-ok register value that matches no listed code
      # (firmware revisions routinely add codes the config has not enumerated).
      # All three keys are optional:
      #   unknown_fault_code: code raised for the unmapped value. Defaults to a
      #                       deterministic <ENTITY>_<DATA_NAME>_UNMAPPED.
      #   unknown_severity:   SOVD bucket for it (default ERROR).
      #   unknown_message:    text (default "unmapped fault code <N>").
      unknown_fault_code: VFD_UNKNOWN_FAULT
      unknown_severity: WARNING
      unknown_message: Undocumented VFD fault code
      codes:
        - code: 10
          fault_code: VFD_OVERVOLTAGE
          severity: ERROR
          message: DC bus overvoltage
        - code: 11
          fault_code: VFD_OVERCURRENT
          severity: ERROR

# The three modes (`alarm`, `status_bits`, `fault_enum`) are evaluated by the
# shared `ros2_medkit_fault_detection` module so every protocol plugin maps raw
# values to faults identically. They are mutually exclusive on a single node.
#
# Status words: prefer exposing the register as an UNSIGNED OPC-UA type (Byte /
# UInt16 / UInt32 / UInt64). A signed type (Int16/Int32) whose top bit is set is
# sign-extended when widened to the 64-bit decode register, which sets every bit
# above the register width and raises spurious bit faults. When you cannot change
# the server type, set `status_word_width:` (1..64) on the node instead: the
# loader masks the raw value to that many low bits before decoding, so the
# sign-extended high bits are dropped in-config. `bit:` positions are 0-based and
# must be < 64 (and < `status_word_width` when set); higher positions are dead
# config, so the loader logs a warning and skips that bit rule while still loading
# the rest of the config.
#
# Fault codes must be globally unique across ALL fault sources - every `alarm` /
# `status_bits` / `fault_enum` entry AND every `event_alarms` entry (including
# each of its `mappings[].fault_code`) - regardless of which entity owns them. The fault manager keys and clears faults by
# fault_code alone, so a code reused on two sources (even on different entities,
# even one polled and one event-driven) would flap raise/clear or clear the other
# source's fault. The loader rejects the whole file at load with an actionable
# error naming both sources.

# Native OPC-UA AlarmConditionType events (issue #386). Subscribes to alarms
# defined inside the PLC (Siemens Program_Alarm / ProDiag, Beckhoff TF6100,
# CodeSys 3.5+, Rockwell via FactoryTalk Linx). Mutually exclusive per entry
# with the threshold-based alarm form above.
event_alarms:
  # Simple form: one source -> one fault.
  - alarm_source: "ns=4;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_TANK_OVERPRESSURE

  # Multi-alarm form (issue #389): one source emits many conditions (e.g. the
  # Server object as a catch-all, or one Object owning several Program_Alarms)
  # routed to distinct faults by condition identity. `mappings` are evaluated
  # in order; the first whose non-empty match fields all match the observed
  # event wins (`match_message` is a substring match, the rest equality),
  # falling back to the source-level `fault_code` (if set).
  - alarm_source: "ns=0;i=2253"          # Server object (system-wide)
    entity_id: line_1
    fault_code: PLC_GENERIC_ALARM        # catch-all fallback (optional)
    mappings:
      - condition_name: "Overpressure"   # match ConditionType.ConditionName
        fault_code: PLC_OVERPRESSURE
        severity_override: ERROR
        message: "Tank overpressure"
      - source_node: "ns=3;s=PumpA"      # or match by event SourceNode
        event_type: "ns=0;i=2915"        # and/or EventType (AND-combined)
        fault_code: PLC_PUMP_A_FAULT
      - match_message: "Overtemperature"  # or match a Message substring - the
        fault_code: PLC_OVERTEMP          # practical discriminator for PLCs (e.g.
        severity_override: ERROR          # Siemens S7-1500) whose Program_Alarms
                                          # share one EventType + SourceNode and
                                          # differ only by alarm text
    # Append vendor associated values (Siemens Program_Alarm SD_1..SD_n) to the
    # fault description as "label=value".
    associated_values:
      - "SD_1"
      - name: "SD_2"
        label: "Setpoint"
```

Mapping precedence: a mapping with more (non-empty) match fields is matched
only when all of them match the observed event; declaration order breaks ties.
`condition_name` / `source_node` / `event_type` are equality matches;
`match_message` is a case-sensitive substring match on the event Message.
An event matching no mapping uses the source-level `fault_code`; if that is also
empty the event is ignored. Exception: a non-condition event (a plain BaseEvent
/ system message whose `ConditionId` resolves to null - e.g. a SIMATIC "CPU not
in RUN" arriving on a `ns=0;i=2253` catch-all) is dropped before mapping
resolution runs, so it never falls through to the source-level `fault_code` and
that catch-all code is never raised for it (see "System messages" under
`auto_alarms` below). A mapping-level `severity_override` / `message` overrides
the source-level one; otherwise the source-level value is inherited.

The plugin auto-registers `acknowledge_fault` and `confirm_fault` operations
on every entity that has at least one `event_alarms` entry. Invoke them with:

```bash
curl -X POST http://localhost:8080/api/v1/apps/tank_process/operations/acknowledge_fault/executions \
     -H 'Content-Type: application/json' \
     -d '{"fault_code":"PLC_OVERPRESSURE","comment":"operator on radio"}'
```

See `design/index.rst` for the full state machine table and vendor matrix.

### auto_browse (zero-config discovery)

Instead of (or on top of) a hand-written `nodes:` list, the plugin can build
the SOVD entity tree itself by recursively browsing the connected server's
address space. This is what makes a discovered endpoint (see the discovery
plugin) usable with no PLC-specific config at all: `endpoint_url` +
`auto_browse: true` is enough.

The walk starts at the standard `Objects` folder (`i=85`, which is where the
OPC-UA DI companion namespace also hangs `DeviceSet`) and recurses forward
along hierarchical references:

- **Object** nodes become a path segment; an Object becomes its own SOVD App
  only once it (directly) owns at least one supported Variable - a pure
  folder Object with only child Objects is walked through but never shows up
  as an entity of its own. Hierarchy is preserved in the generated
  `entity_id`/name as a path (e.g. `plc_1_db_test` / `PLC_1 / DB_Test`), not
  as a separate parent-entity field - the plugin's SOVD model stays the flat
  Area -> Component -> App tree it already produces from a hand-written node
  map.
- **Variable** nodes become data points. Their DataType is read and mapped to
  a medkit `data_type` (`Boolean`->`bool`; `SByte`/`Byte`/`Int16`/`UInt16`/
  `Int32`/`UInt32`/`Int64`/`UInt64`->`int`; `Float`/`Double`->`float`;
  `String`/`DateTime`->`string`); anything else (structured/enumerated/vendor
  types) is skipped, not guessed at.
- Noise filtering: ns=0 (the standard/infrastructure namespace - the `Server`
  object, `ServerCapabilities`, `Types`, `Views`, ...) is denied by default so
  it never enters the tree, while the DI/vendor namespaces underneath
  `Objects` are walked normally. A depth limit and a total-node budget bound
  the walk against a pathological or very large address space; hitting either
  logs an operator warning that the resulting tree may be incomplete.
- **Read-only.** auto_browse never writes to the server, and every
  auto-discovered data point loads with `writable: false` - promoting a
  specific point to writable is a deliberate, reviewed decision made via an
  explicit `nodes:` entry.
- **Explicit config always wins.** An auto-browsed entry for a NodeId that
  already has a hand-written `nodes:` entry is dropped; auto_browse only fills
  in what the node map does not already cover (or the whole tree, when there
  is no node map at all).

Enable it either in the node-map YAML:

```yaml
auto_browse: true   # all-default knobs (root=Objects folder, depth 8, 5000 nodes, deny ns=0)

# or with explicit knobs:
auto_browse:
  enabled: true
  root_nodes: ["i=85"]        # default: Objects folder. Point elsewhere (e.g. a
                               # specific DeviceSet/Object) to scope the walk.
  max_depth: 8                # recursion depth cap from a root (1..64)
  max_nodes: 5000              # total node budget for the walk (1..200000)
  namespace_allow: []          # non-empty = only these namespaces are eligible
  namespace_deny: [0]          # ignored when namespace_allow is set
  read_initial_values: true    # read each candidate Variable once; drop it if
                                # the read fails (reachability check)
```

or purely via the plugin's ROS param / JSON config, which is what lets a
discovered endpoint go straight to a populated tree with no node-map file:

```yaml
plugins.opcua.endpoint_url: "opc.tcp://192.168.1.10:4840"
plugins.opcua.auto_browse: true   # or the same map form as above
```

The JSON/ROS-param form takes precedence over whatever the node-map YAML's
`auto_browse:` block set, mirroring how environment variables override the
rest of the plugin's YAML config.

### Zero-config native A&C (`auto_alarms`)

`event_alarms:` above requires a hand-written mapping per alarm. `auto_alarms`
needs none: enable it and every AlarmConditionType event on the subscribed
source (default the Server object, `i=2253` - the same system-wide catch-all
used in the `event_alarms` examples) becomes a fault automatically, with no
`condition_name` / `source_node` / `fault_code` mapping to write.

```yaml
# Bare boolean - all defaults.
auto_alarms: true
```

```yaml
# Map form - every key optional.
auto_alarms:
  enabled: true
  source_node_id: "i=2253"       # EventNotifier to subscribe (default: Server object)
  entity_id: "plc_alarms"        # fallback SOVD entity (default: "<component_id>_alarms")
  auto_clear: true               # clear immediately on ActiveState=false, bypassing
                                  # Acknowledge/Confirm (see below)
  severity_bands:                # OPC-UA Severity (1-1000) -> SOVD bucket
    critical: 801
    error: 501
    warning: 201
  include: []                    # substring allow-list on ConditionName/SourceName/Message
  exclude: ["CPU not in RUN"]    # substring deny-list, checked first
```

Default `off`. Unknown keys are warned and ignored, not silently dropped.

Instead of (or on top of) the node-map YAML, `auto_alarms` can be set purely
via the plugin's ROS param / JSON config, which is what lets a discovered
endpoint go straight to native faults with no node-map file at all:

```yaml
plugins.opcua.endpoint_url: "opc.tcp://192.168.1.10:4840"
plugins.opcua.auto_alarms: true   # or the same map form as above
```

The JSON/ROS-param form takes precedence over whatever the node-map YAML's
`auto_alarms:` block set (same precedence env vars use for the rest of the
plugin config). Explicit `event_alarms:` mappings still win over
auto-derivation at the poller regardless of how `auto_alarms` was enabled.
This completes the zero-config chain: discovery (endpoint) ->
`auto_browse` (SOVD tree) -> `auto_alarms` (native faults), each param-driven,
no node-map file required.

**Fault derivation** (no config, per observed event):
- `fault_code`: `PLC_ALARM_<slug>_<hash(SourceNode)>` from `ConditionName` when
  present, else from `SourceName`, else `PLC_ALARM_<hash>` of
  SourceNode + EventType + Message. The SourceNode is folded into every tier so
  two distinct conditions sharing a ConditionName/SourceName but sitting on
  different sources (e.g. two identical FB instances each raising
  "Overpressure") never collapse onto one code - the fault manager keys/clears
  by code alone, so a collision would let one condition's clear wipe the
  other's still-active fault. Message is NOT folded into the slug tiers, so a
  condition whose Message differs between its active and inactive notifications
  still maps to one code. The hash tier additionally folds in the Message
  deliberately - a real Siemens S7-1500 multiplexes every `Program_Alarm` of
  one FB through a single SourceNode (e.g. `i=1845`) with no
  ConditionName/SourceName, distinguished only by Message text ("pa" vs "pa2");
  without the Message in the hash those two alarms would collapse onto one
  fault_code. A given condition derives its code once (at first observation)
  and keeps it for its whole lifecycle, so its raise and clear always agree.
- `entity`: the node-map entity that owns the event's SourceNode when it
  matches a `nodes:` entry, else `auto_alarms.entity_id`.
- `severity`: the event's raw `Severity` (1-1000) mapped through
  `severity_bands`.
- `description`: the event's `Message`, verbatim.

**System messages:** some servers deliver non-alarm notifications on the same
EventNotifier as real alarms - a Siemens Server object also emits SIMATIC
system messages (e.g. `"CPU not in RUN"`) over `i=2253`. Per OPC-UA Part 9
§5.5.2.13 only a true Condition instance carries a ConditionId; a system
message resolves it to `NodeId.Null`. The bridge drops those events for every
alarm source - explicit `event_alarms` (including a source-level `fault_code`
catch-all) and `auto_alarms` alike - before any mapping resolution or
auto-derivation, so they never surface as a fault. On an explicit source it also
logs a one-time warning (a real alarm that genuinely lacks a ConditionId would
otherwise be dropped silently). Use `exclude` for anything that still needs
filtering by text (e.g. a server that does set a ConditionId on system
events).

**Precedence:** explicit `event_alarms` mappings are tried first; `auto_alarms`
covers whatever they do not match. On the same source (e.g. both configured
on `i=2253`) no duplicate subscription is created - unmatched events on an
`event_alarms` source simply fall through to auto-derivation when
`auto_alarms` is enabled for that source.

**Acknowledge/Confirm:** `auto_clear` (default `true`) clears an auto-derived
alarm as soon as it goes inactive, bypassing the Acknowledge/Confirm workflow
entirely - there is no `acknowledge_fault` SOVD operation for a zero-config
alarm to receive an Ack through, so without this it would latch forever on a
server that requires one (as most do). Set `auto_clear: false` to require the
poller's normal Acknowledge/Confirm gating before it clears - since a
zero-config alarm has no SOVD `acknowledge_fault` operation of its own,
Acked/Confirmed must come from the OPC-UA server side instead (an HMI/SCADA
system or another OPC-UA client acknowledging the condition directly).

### Gateway Parameters

```yaml
ros2_medkit_gateway:
  ros__parameters:
    plugins: ["opcua"]
    plugins.opcua.endpoint_url: "opc.tcp://plc-host:4840/path"
    plugins.opcua.node_map_path: "/path/to/nodes.yaml"
    plugins.opcua.poll_interval_ms: 1000
    plugins.opcua.prefer_subscriptions: false
    plugins.opcua.auto_browse: true
    plugins.opcua.auto_alarms: true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `endpoint_url` | `opc.tcp://localhost:4840` | OPC-UA server endpoint |
| `node_map_path` | (none) | Path to node map YAML (optional when `auto_browse` or `auto_alarms` is enabled - see above) |
| `auto_browse` | `false` | Boolean or object; recursively discover the SOVD tree from the live address space instead of (or in addition to) `node_map_path` - see above |
| `auto_alarms` | `false` | Boolean or object; subscribe a server EventNotifier and auto-derive a fault per AlarmCondition with no per-alarm `event_alarms` mapping - the zero-config native A&C surface (see above) |
| `poll_interval_ms` | `1000` | Polling interval in ms (clamped to [100, 60000]) |
| `prefer_subscriptions` | `false` | Use OPC-UA subscriptions instead of polling |
| `subscription_interval_ms` | `500` | Publishing interval for OPC-UA subscriptions when `prefer_subscriptions: true` |
| `condition_replay_strategy` | `auto` | Active-condition replay on reconnect: `method`, `read`, `auto`, `off` (see below) |
| `require_confirm_for_clear` | `true` | Require both Acknowledge AND Confirm before a native alarm auto-clears. Set `false` for Confirm-less servers (e.g. Siemens S7-1500) so alarms clear on Acknowledge alone (see below) |
| `comms_lost_fault_enabled` | `true` | Raise a component-scoped `PLC_COMMS_LOST` fault when the connection stays down, and clear it on every successful connect (issue #496) |
| `comms_lost_debounce_ms` | `5000` | Continuous down time before `PLC_COMMS_LOST` is raised (debounces reconnect blips; clamped to [0, 3600000] ms) |
| `comms_lost_severity` | `ERROR` | SOVD severity bucket for the `PLC_COMMS_LOST` fault |
| `discovery.enabled` | `false` | Opt-in read-only PLC network discovery (auto endpoint). See below |

### OPC-UA client security (SecurityPolicy, certificates, user auth)

By default the client connects with `SecurityPolicy=None` and an `Anonymous`
user (unencrypted, unauthenticated). This is fine for an isolated lab LAN but
not for a hardened server. The following parameters enable a signed/encrypted
SecureChannel with a client application-instance certificate, a server trust
store, and a user identity. All are opt-in; leaving them unset reproduces the
legacy behaviour.

```yaml
plugins.opcua.security_policy: "Basic256Sha256"       # None | Basic256Sha256 | Aes128Sha256RsaOaep | Aes256Sha256RsaPss
plugins.opcua.security_mode: "SignAndEncrypt"          # None | Sign | SignAndEncrypt
plugins.opcua.client_cert_path: "/etc/ros2_medkit/opcua/client_cert.der"  # X.509 v3, DER
plugins.opcua.client_key_path:  "/etc/ros2_medkit/opcua/client_key.pem"   # private key, PEM
plugins.opcua.application_uri:  "urn:selfpatch:medkit:opcua-client"        # MUST match the cert SAN URI
plugins.opcua.trust_list_paths: ["/etc/ros2_medkit/opcua/server_cert.der"]  # DER trust store
plugins.opcua.reject_untrusted: true                   # false = accept ANY server cert (INSECURE, lab only; not trust-on-first-use, nothing pinned)
plugins.opcua.user_auth_mode: "UsernamePassword"       # Anonymous | UsernamePassword | X509
plugins.opcua.username: "medkit"
plugins.opcua.password: "..."                           # inject from a secret store at deploy time
plugins.opcua.user_cert_path: "/etc/ros2_medkit/opcua/user_cert.der"  # X509 user token only
```

Notes:
- A non-None `security_policy` requires `client_cert_path` + `client_key_path`;
  a secured connection with no cert fails fast (before contacting the server).
- `application_uri` must equal the URI entry in the client certificate's
  SubjectAltName or the server rejects the channel with
  `BadCertificateUriInvalid`.
- The client cert (its public part) must be added to the server's trusted
  client list, and the server cert (DER) added to `trust_list_paths`, unless
  `reject_untrusted: false`.
- The encryption backend (OpenSSL) is compiled in; a build without it logs an
  error and refuses any secured profile.

### Read-only PLC network discovery (auto endpoint)

A diag-box plugged into a plant LAN can auto-find an OPC-UA PLC instead of
requiring a hand-configured `endpoint_url`. Discovery is **opt-in and disabled
by default**; enabling it runs a bounded, read-only active scan and hands the
best OPC-UA data server to the normal connect + introspect path.

```yaml
plugins.opcua.discovery:
  enabled: false               # master switch (default false)
  mode: "active"               # active | passive | both (only active implemented)
  subnets: ["192.168.1.0/24"]  # CIDR list; empty => derive the host's local /24
  ports: [4840]                # TCP ports probed (4840 = OPC-UA; others = leads)
  connect_timeout_ms: 600      # per-port TCP connect timeout
  scan_concurrency: 100        # bounded, polite concurrent connect count
  identify_timeout_ms: 6000    # per GetEndpoints identify
  # re-scan cadence while disconnected. Omit the key for the built-in 30 s,
  # or set it to 0 to keep discovery on but never re-scan (start-up scan only).
  interval_s: 30
  anonymous_none_only: true    # only auto-connect None/Anonymous servers
```

Environment overrides (Docker / appliance): `OPCUA_DISCOVERY_ENABLED`,
`OPCUA_DISCOVERY_SUBNETS` (comma-separated CIDRs), `OPCUA_DISCOVERY_INTERVAL_S`.
Leaving `interval_s` (and `OPCUA_DISCOVERY_INTERVAL_S`) unset means "no cadence
stated" and takes the 30 s default. An explicit `0` is honoured as written and
turns the recurring sweep off. A negative value is refused with a warning and
leaves the cadence unset.

How it works:
1. Bounded concurrent TCP connect sweep of the configured ports across the
   subnet (nothing beyond a connect - no payload sent to non-OPC-UA ports).
2. For each open `:4840`, a read-only OPC-UA `GetEndpoints` identify extracts
   the endpoint URL, ApplicationUri, ProductUri, security policies, and whether
   a None/Anonymous endpoint is offered.
3. Results are deduplicated by ApplicationUri (fallback ip:port) and classified:
   OPC-UA **data server** (ApplicationType 0/2, browseable) vs **discovery
   server / LDS** (type 3, never browsed) vs **non-OPC-UA lead** (e.g. an open
   `:102` S7comm port, recorded but not identified).
4. When discovery is on **and no `endpoint_url` is configured**, the plugin
   auto-selects the best None/Anonymous data server (deterministic, lowest
   ip:port) and connects to the **scanned ip:port** - not the advertised
   EndpointUrl, which a server may report as a non-resolvable hostname.
5. While no session is established, the reconnect loop scans again every
   `interval_s` (default 30 s), measured from the END of the previous sweep, and
   adopts a newly found server for its next connect attempt, logging the swap at
   INFO. The re-scan is consulted once per reconnect attempt, and those are
   spaced by an exponential backoff, so the backoff ceiling is capped at
   `interval_s` while discovery is re-scanning - otherwise the real cadence
   would be `max(interval_s, backoff)` rather than the stated one. This covers
   the common race where the gateway and the PLC boot together: the startup scan
   finds nothing because the PLC is still coming up, and without a re-scan the
   plugin would retry the fallback endpoint until someone restarted it.
6. On the first session after such an adoption, a config-less deployment (no
   node map) re-derives the SOVD component identity from the device itself, so
   the component stops being served under the provisional `opcua-<host>` name it
   got when nothing answered. The change is logged at INFO.

Re-scanning stops as soon as a session is up, and never starts at all when an
`endpoint_url` is configured.

Safety / OT posture:
- Everything is read-only: TCP connect + `GetEndpoints` only. No writes, no
  subscriptions, no second long-lived session.
- The scan is NOT one-shot: while the plugin has no session it repeats every
  `interval_s` (default 30 s) for as long as it stays disconnected. Set
  `interval_s: 0` (or `OPCUA_DISCOVERY_INTERVAL_S=0`) to keep discovery on with
  the start-up scan only, or `enabled: false` to switch it off entirely.
- A sweep is cancellable, so a stop does not have to wait out a subnet the size
  of a /16. The start-up sweep runs while the gateway node is still being
  constructed, so what ends it is `SIGINT` / `SIGTERM`, which the plugin sees
  through `rclcpp::ok()`. A re-scan sweep runs on the poll thread and is ended
  by either that or the plugin's own `shutdown()`.
- An explicitly configured `endpoint_url` (or `OPCUA_ENDPOINT_URL`) always wins;
  discovery then does nothing, so it never opens a second session on a PLC the
  plugin already polls.
- Secured-only servers (no None/Anonymous endpoint) are surfaced in the log as
  leads requiring operator credentials - never auto-connected or probed. A
  re-scan whose outcome has not changed reports at DEBUG instead of repeating
  the whole report, so a recurring sweep does not bury the rest of the log.
- The scan is bounded (short connect timeout, capped concurrency) and CIDRs
  wider than /16 are rejected to prevent an accidental broad sweep.

Note on passive discovery: a stock Siemens S7-1500 neither multicast-announces
(mDNS `_opcua-tcp._tcp`) nor registers with an OPC-UA LDS, so passive sources
find nothing there; the active scan is what discovers it. Passive mDNS / LDS
`FindServers` sources (useful on Kepware / Prosys / GDS estates) and
multi-endpoint registration are planned follow-ups. This iteration delivers the
active-scan core and a single "auto endpoint" mode.

### Active-condition replay on reconnect (issue #389/#478)

When the gateway (re)subscribes after a drop or restart, conditions that are
already active on the server would otherwise not be re-reported (only live
transitions flow). The standard recovery is OPC-UA `ConditionRefresh`. Per
Part 9 §5.5.7 it is a method of the **ConditionType** node (`i=2782`), so the
Call is issued there (not the Server object) - calling it on the Server object
was the root cause of `BadNodeIdUnknown` rejections, including on Siemens
S7-1500, which documents `ConditionRefresh` as supported (only the optional
`ConditionRefresh2` is not). With the corrected target, `ConditionRefresh` is
expected to work on conformant servers and the read fallback should rarely be
needed. The `condition_replay_strategy` parameter controls recovery:

| Value | Behaviour |
|-------|-----------|
| `method` | Call `ConditionRefresh` only (warns if rejected) |
| `read` | Skip the method; browse each `alarm_source`, read current condition state, reconcile the fault set |
| `auto` (default) | Try `ConditionRefresh`; on rejection fall back to `read` |
| `off` | No replay (only live transitions surface) |

The read fallback browses each `alarm_source` for AlarmCondition instances -
both hierarchical Object children and targets of the `HasCondition` reference
(`i=9006`), recursing one level - and reads their state. It mirrors
`ConditionRefresh` semantics: only conditions with `Retain==true` form the
active set, a `Disabled` condition (`EnabledState/Id=false`) is never treated
as active, and a transient read failure keeps the condition rather than
dropping it.

**Safety guarantee (#478):** the read fallback NEVER clears a tracked fault for
a source that has not positively exposed at least one Condition instance node.
An EventNotifier-only server (e.g. S7-1500, which models no per-condition
instance nodes) makes the browse return zero conditions; rather than wiping the
active alarm set, the gateway preserves the last-known faults, logs an operator
warning that read-fallback is unsupported on this server, and keeps live
transitions flowing. Use `ConditionRefresh` (`method`/`auto`) on such servers.
The read fallback does not recover the live `EventId`, so an operator
ack/confirm may need the next live event before it succeeds.

### Confirm-less servers and `require_confirm_for_clear` (issue #478)

By default a native alarm auto-clears only after the operator has both
Acknowledged AND Confirmed it (OPC-UA Part 9 §5.7). Some servers do not
implement the optional `Confirm` transition - Siemens S7-1500 supports
`Acknowledge` / `ConditionRefresh` / `AddComment` but not `Confirm`, so
`ConfirmedState` never becomes true and an inactive alarm would latch forever.
Set `require_confirm_for_clear: false` (or `OPCUA_REQUIRE_CONFIRM_FOR_CLEAR=0`)
so the alarm clears on `Acknowledge` alone. The default (`true`) is unchanged
and spec-strict; the relaxed path still requires acknowledgement and needs
real-S7-1500 validation.

### Connection loss and `PLC_COMMS_LOST` (issue #496)

When the OPC-UA connection stays down for `comms_lost_debounce_ms` continuously,
the plugin raises one component-scoped `PLC_COMMS_LOST` fault (a shorter blip
during a normal reconnect does not flap it).

The fault is cleared on **every** successful connect, both the initial one and
every later reconnect, whether or not this process was the one that raised it.
The fault manager keys faults by fault code and persists them, so a fault raised
before a gateway restart is still standing while the new process has no memory
of it. Clearing only what the running process remembered left exactly that fault
CONFIRMED for good. The clear is fire-and-forget, so a clear for a fault that is
not there is harmless.

Node map entries also support an optional `ros2_topic` field to override the auto-generated ROS 2 topic name for the PLC value bridge:

```yaml
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank_process
    data_name: tank_level
    ros2_topic: /custom/plc/tank_level   # optional, overrides auto-generated /plc/tank_process/tank_level
```

### Operation Naming Convention

Write operations use the `set_` prefix convention:
- Node map defines `data_name: pump_speed`
- REST operation: `POST /apps/fill_pump/x-plc-operations/set_pump_speed`
- The `set_` prefix is stripped to find the matching node map entry
- Operations without `set_` prefix also work (matches `data_name` directly)

### Environment Variables (override YAML config)

| Variable | Description |
|----------|-------------|
| `OPCUA_ENDPOINT_URL` | OPC-UA server URL |
| `OPCUA_NODE_MAP_PATH` | Path to node map YAML |
| `OPCUA_SECURITY_POLICY` | `None` / `Basic256Sha256` / `Aes128Sha256RsaOaep` / `Aes256Sha256RsaPss` |
| `OPCUA_SECURITY_MODE` | `None` / `Sign` / `SignAndEncrypt` |
| `OPCUA_CLIENT_CERT` / `OPCUA_CLIENT_KEY` | Client cert (DER) / key (PEM) paths |
| `OPCUA_APPLICATION_URI` | Client application URI (must match cert SAN) |
| `OPCUA_TRUST_LIST` | Colon-separated DER trust-store paths |
| `OPCUA_REJECT_UNTRUSTED` | `false`/`0`/`no` to accept any server cert (lab only) |
| `OPCUA_USER_AUTH` | `Anonymous` / `UsernamePassword` / `X509` |
| `OPCUA_USERNAME` / `OPCUA_PASSWORD` | Username-password identity |
| `OPCUA_USER_CERT` | X.509 user-token cert (DER) |
| `OPCUA_CONDITION_REPLAY` | `method` / `read` / `auto` / `off` |
| `OPCUA_REQUIRE_CONFIRM_FOR_CLEAR` | `0`/`false`/`no`/`off` to clear native alarms on Acknowledge alone (Confirm-less servers) |
| `OPCUA_COMMS_LOST_ENABLED` | `0`/`false`/`no`/`off` to disable the `PLC_COMMS_LOST` fault |
| `OPCUA_COMMS_LOST_DEBOUNCE_MS` | Continuous down time (ms) before `PLC_COMMS_LOST` is raised (clamped to [0, 3600000] ms; non-numeric / out-of-range keeps the existing value) |

## Hardware Deployment

### Robot + PLC on the same LAN

```
┌──────────────┐  WiFi/ETH  ┌──────────────┐  ETH  ┌──────────────┐
│  Robot       │◄───────────►│  Edge RPi    │◄─────►│  PLC         │
│  (Jetson)   │             │              │       │  (Siemens/   │
│              │             │  Fleet GW    │       │   Beckhoff/  │
│  ros2_medkit │             │  :9090       │       │   OpenPLC)   │
│  :8080       │             │              │       │              │
└──────────────┘             │  ros2_medkit │       │  OPC-UA      │
                             │  + OPC-UA    │       │  :4840       │
                             │  plugin      │       └──────────────┘
                             │  :8080       │
                             └──────────────┘
```

Fleet gateway aggregates robot and PLC diagnostics. Operator sees both in one dashboard.

### Robot directly connected to PLC

```
┌──────────────────────┐  ETH  ┌──────────────┐
│  Robot               │◄─────►│  PLC         │
│                      │       │  OPC-UA :4840│
│  ros2_medkit gateway │       └──────────────┘
│  + OPC-UA plugin     │
│  :8080               │
│                      │
│  ROS 2 nodes + PLC   │
│  in one entity tree  │
└──────────────────────┘
```

Plugin runs on the robot itself. ROS 2 faults and PLC alarms appear side by side.

### Compatible PLCs

Any PLC with an OPC-UA server works out of the box:

| PLC | OPC-UA Support | Notes |
|-----|---------------|-------|
| Siemens S7-1500 | Built-in | Most common in EU industry |
| Allen-Bradley CompactLogix | Built-in | Common in US |
| Beckhoff TwinCAT | Built-in | Popular in motion control |
| Schneider M340/M580 | Built-in | Process automation |
| OpenPLC v4 | Plugin (asyncua) | Open-source, used in our demo |
| Older PLCs (S7-300, etc.) | Via Modbus->OPC-UA bridge | Requires additional software |

### What you need

1. PLC with OPC-UA server enabled (most modern PLCs have this)
2. Network connectivity between robot/edge device and PLC (Ethernet, same subnet)
3. Node map YAML matching your PLC program variables
4. See Security Limitations below regarding OPC-UA auth

### What you DON'T need

- No ROS 2 on the PLC
- No modification to the PLC program
- No special hardware adapters
- No proprietary PLC software licenses

## Development

### Build

```bash
# From ros2_medkit repo root
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_medkit_opcua
colcon test --packages-select ros2_medkit_opcua
```

### Docker Integration Tests

The plugin ships a self-contained OpenPLC tank demo in `docker/` that exercises the full stack end-to-end. CI runs this suite on every PR that touches the plugin; it is also runnable locally from any developer laptop.

```bash
cd src/ros2_medkit_plugins/ros2_medkit_opcua/docker

# Start OpenPLC + gateway (builds everything)
bash scripts/start.sh

# Manual testing
curl -s http://localhost:8080/api/v1/apps/tank_process/x-plc-data | jq .

# Automated tests (16 assertions)
bash scripts/run_integration_tests.sh

# Stop
bash scripts/stop.sh
```

A separate scenario covers the config-less discovery start-up race, which the
suite above cannot see because it pins `OPCUA_ENDPOINT_URL` and so
short-circuits discovery. It starts the gateway before any server, with
discovery on and no endpoint configured, then brings a server up and asserts
the gateway adopts it without a restart:

```bash
bash src/ros2_medkit_plugins/ros2_medkit_opcua/docker/scripts/run_discovery_race_test.sh
```

### Test Coverage

| Category | Tests | What it validates |
|----------|-------|-------------------|
| Entity discovery | 5 | Areas, components, apps from PLC node map |
| PLC connection | 2 | OPC-UA connected, zero errors |
| Live data | 3 | Tank level, temperature, pressure have values |
| Write control | 2 | Pump speed, valve position written to PLC |
| Error handling | 3 | Unknown entity, unknown operation, invalid JSON |
| **Total** | **16** | |

## Security

The client supports a signed/encrypted SecureChannel (`SecurityPolicy`
Basic256Sha256 / Aes128 / Aes256, `MessageSecurityMode` Sign / SignAndEncrypt),
a client application-instance certificate, a server trust store with
reject-untrusted, and user identity (Anonymous / Username-Password / X.509).
See "OPC-UA client security" above for configuration.

**The default remains `SecurityPolicy=None` + `Anonymous` (unencrypted,
unauthenticated)** for backward compatibility and isolated-LAN demos; enable a
secured profile for any network exposed to untrusted traffic. The plugin logs
the effective security profile at startup and warns when running unsecured or
with `reject_untrusted: false`.

Northbound (gateway REST) hardening is documented separately in the gateway's
`design/hardening.rst` and the `gateway_params.secure.yaml` field profile.

Write operations include configurable `min_value`/`max_value` range validation to prevent out-of-range values being sent to PLC actuators.

## Alarm-to-Fault Bridge

PLC alarms (threshold-based) are automatically mapped to SOVD faults:

```
PLC variable (e.g., TankTemperature = 95C)
    │
    ▼  threshold check (> 80C)
Alarm active: PLC_HIGH_TEMP
    │
    ▼  ROS 2 service call
ros2_medkit fault_manager
    │
    ▼  appears in SOVD API
GET /api/v1/apps/tank_process/faults
  -> [{fault_code: "PLC_HIGH_TEMP", severity: "ERROR", ...}]
```

When the value returns below threshold, the fault is automatically cleared.

## Key Design Decisions

- **Poll mode by default** - OPC-UA subscriptions require a client event loop thread. Polling at 1s interval is simpler and sufficient for diagnostics use cases.
- **Type-aware writes** - Plugin reads the OPC-UA node's data type before writing to avoid type mismatches (e.g., writing float32 to a REAL node, not float64).
- **Node map driven** - All entity mapping is in YAML config, not code. Same plugin binary works with any PLC by changing the config file.
- **Env var overrides** - `OPCUA_ENDPOINT_URL` and `OPCUA_NODE_MAP_PATH` override YAML config for Docker deployment flexibility.

## License

Apache License 2.0. See the `LICENSE` file at the repository root for the full text.

Copyright 2026 mfaferek93

## Third-party Dependencies

| Library | License | Notes |
|---------|---------|-------|
| [open62541pp](https://github.com/open62541pp/open62541pp) v0.16.0 | MPLv2 | OPC-UA C++ client, linked as library (no source modification) |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | JSON serialization |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | MIT | Node map YAML parser |
| [OpenSSL](https://www.openssl.org/) | Apache-2.0 | TLS support for OPC-UA secure channels |

MPLv2 is a weak copyleft license that permits linking from Apache-2.0 code without triggering source disclosure obligations for the linking application, as long as the MPLv2-licensed library itself is not modified.

## Contributing

Issues and pull requests welcome through the main [ros2_medkit](https://github.com/selfpatch/ros2_medkit) repository. See the project-level `CONTRIBUTING.md` for coding style, test expectations, and review process.
