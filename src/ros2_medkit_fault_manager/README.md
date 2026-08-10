# ros2_medkit_fault_manager

Central fault manager node for the ros2_medkit fault management system.

## Overview

The FaultManager node provides a central point for fault aggregation and lifecycle management.
It receives fault reports from multiple sources, aggregates them by `fault_code`, and provides
query and clearing interfaces.

## Quick Start

By default, faults are confirmed immediately when reported - no additional configuration needed.

```bash
# Start the fault manager
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# Report a fault - it's immediately CONFIRMED
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'MOTOR_OVERHEAT', event_type: 0, severity: 2, description: 'Motor temp exceeded', source_id: '/motor_node'}"

# Query faults
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED']}"

# Clear a fault (cascade-clears correlated symptoms by default)
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT', skip_correlation_auto_clear: false}"

# Clear without touching correlated symptoms
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT', skip_correlation_auto_clear: true}"
```

> **Note:** The `skip_correlation_auto_clear` request field was added post-0.4.0. Adding a request field changes the service type hash, so callers built against `ros2_medkit_msgs` 0.4.0 or earlier must rebuild to keep talking to `fault_manager`.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/report_fault` | `ros2_medkit_msgs/srv/ReportFault` | Report a fault occurrence |
| `~/list_faults` | `ros2_medkit_msgs/srv/ListFaults` | Query faults with filtering |
| `~/clear_fault` | `ros2_medkit_msgs/srv/ClearFault` | Clear/acknowledge a fault |
| `~/get_snapshots` | `ros2_medkit_msgs/srv/GetSnapshots` | Get topic snapshots for a fault |

## Features

- **Multi-source aggregation**: Same `fault_code` from different sources creates a single fault
- **Occurrence tracking**: Counts total reports and tracks all reporting sources
- **Severity escalation**: Fault severity is updated if a higher severity is reported
- **Persistent storage**: SQLite backend ensures faults survive node restarts
- **Debounce filtering** (optional): AUTOSAR DEM-style counter-based fault confirmation with per-entity threshold overrides
- **Snapshot capture**: Captures topic data when faults are confirmed for debugging (snapshots are deleted when fault is cleared)
- **Freeze-frame retention**: One compact JSON freeze-frame per fault code, retained across `clear_fault` (see below)
- **Fault correlation** (optional): Root cause analysis with symptom muting and auto-clear
- **Tamper-evident audit log** (optional): Append-only, hash-chained record of fault state transitions for verifiable history

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `storage_type` | string | `"sqlite"` | Storage backend: `"sqlite"` or `"memory"` |
| `database_path` | string | `"/var/lib/ros2_medkit/faults.db"` | Path to SQLite database file |
| `confirmation_threshold` | int | `-1` | Counter value at which faults are confirmed |
| `healing_enabled` | bool | `false` | Enable automatic healing via PASSED events |
| `healing_threshold` | int | `3` | Counter value at which faults are healed |
| `auto_confirm_after_sec` | double | `0.0` | Auto-confirm PREFAILED faults after timeout (0 = disabled) |
| `entity_thresholds.config_file` | string | `""` | Path to YAML file with per-entity debounce threshold overrides |

### Snapshot Parameters

Snapshots capture topic data when faults are confirmed for post-mortem debugging.

Each confirm also writes a **freeze-frame**: a single compact JSON object mapping every captured topic to its value at confirmation time, keyed by fault code. It differs from per-topic snapshots in two ways: snapshots are deleted when the fault is cleared, while the freeze-frame is retained across `clear_fault` (once the snapshots are gone, `~/get_fault` serves the retained frame so the confirmed-state record stays available after acknowledgement); and a re-confirm that captures nothing (e.g. source publishers down) never overwrites an existing non-empty frame. A fault code with no configured capture set gets no freeze-frame row; a configured capture that samples nothing on its first run records an empty `{}` frame. Freeze-frame storage is bounded by the number of distinct fault codes (one row per code, replaced in place) and rows are never evicted.

Under a fault storm, captures are bounded by a worker pool (`capture_pool_size`) draining a bounded queue (`capture_queue_depth`); excess captures are dropped per `capture_queue_full_policy` and logged (throttled). The pool is shared and is created when snapshots **or** rosbag is enabled, so these parameters bound both. `capture_pool_size` parallelizes freeze-frame snapshot capture only - rosbag stays single-writer regardless of pool size, and correlated faults confirming inside one post-roll window share a single recording.

That single-writer property also shapes what each fault of a burst gets. Nothing is buffered while a post-fault window is open (messages go straight into the open bag), and the flush that opened that bag already emptied the ring buffer, so a fault confirming right *after* the window closes has no pre-fault history available. It gets a **post-fault-only bag**: its own recording holding just its `duration_after_sec` window, entered through the same post-roll state machine, so later faults of the burst attach to it normally. With `duration_after_sec: 0` there is no window to record into and such a fault gets no bag; if the bag cannot be written at all, no recording is opened and no metadata row is stored. The `duration_sec` on a stored bag is the span the recording was open rather than the configured windows, so a post-fault-only bag usually reports roughly `duration_after_sec` where a full one reports its buffered history too. It is a recording span, not a content span: a window during which nothing was published still reports the seconds it covered. It can also exceed `duration_sec + duration_after_sec`, because the ring buffer is pruned only when a message arrives - a topic that stops publishing keeps its last window buffered until the next confirmation flushes it, which is deliberate for a black box. See `docs/config/fault-manager.rst` for the full lifecycle.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `snapshots.enabled` | bool | `true` | Enable/disable snapshot capture |
| `snapshots.background_capture` | bool | `false` | Use background subscriptions (caches latest message) vs on-demand capture |
| `snapshots.timeout_sec` | double | `1.0` | Timeout waiting for topic message (on-demand mode) |
| `snapshots.max_message_size` | int | `65536` | Maximum message size in bytes (larger messages skipped) |
| `snapshots.default_topics` | string[] | `[]` | Topics to capture for all faults |
| `snapshots.config_file` | string | `""` | Path to YAML config for `fault_specific` and `patterns` |
| `snapshots.recapture_cooldown_sec` | double | `60.0` | Min seconds between captures for the same fault code. |
| `snapshots.max_per_fault` | int | `10` | Max snapshots retained per fault. |
| `snapshots.capture_pool_size` | int | `2` | Max concurrent capture threads under a fault storm (>= 1). Parallelizes snapshot capture only; rosbag stays single-writer. |
| `snapshots.capture_queue_depth` | int | `16` | Max pending captures before the full-queue policy applies (>= 1). |
| `snapshots.capture_queue_full_policy` | string | `reject_newest` | Policy when the queue is full: `reject_newest` or `drop_oldest`. |

**Topic Resolution Priority:**
1. `fault_specific` - Exact match for fault code (configured via YAML config file)
2. `patterns` - Regex pattern match (configured via YAML config file)
3. `default_topics` - Fallback for all faults

**Example YAML config file** (`snapshots.yaml`):
```yaml
fault_specific:
  MOTOR_OVERHEAT:
    - /joint_states
    - /motor/temperature
patterns:
  "MOTOR_.*":
    - /joint_states
    - /cmd_vel
```

### Storage Backends

**SQLite (default)**: Faults are persisted to disk and survive node restarts. Uses WAL mode for optimal performance.

**Memory**: Faults are stored in memory only. Useful for testing or when persistence is not required.

## Advanced: Tamper-Evident Audit Log

An optional append-only, hash-chained audit log records every fault state transition (`occurred`, `confirmed`, `healed`, `cleared`) so the fault history is independently verifiable. Auto-recovery (a fault reaching the healing threshold via PASSED events) is recorded as a distinct `healed` row with source `auto_heal`, so the fault's END is in the timeline and is not confused with a manual `cleared`. The manager has no acknowledge action separate from clearing, so `~/clear_fault` is recorded as `cleared` (clear == ack); there is no `ack` kind. The log also records its own lifecycle with `logging_activated` / `logging_deactivated` markers at start and stop. It is **off by default** because it adds a write and storage cost per transition.

Each transition appends one immutable row holding `record_hash = sha256(prev_hash + canonical(event))` (OpenSSL EVP SHA-256), the `prev_hash` it links to, and a monotonic `seq`. The hash is computed once at insert and never recomputed. A persisted chain head lets the chain resume across restarts. The log is stored in its own SQLite database (separate from the fault store) and is treated as append-only: the manager only ever inserts rows, and `BEFORE UPDATE` / `BEFORE DELETE` triggers reject out-of-band edits (the guarded rotation prune excepted).

**Completeness is an integrity property.** `verify()` proves nothing was *deleted* from the chain, but it cannot prove a transition that was *never appended*. So a silently dropped append is a hole `verify()` can never see. Every transition on the write path is therefore audited (occurred, timer/threshold confirmations, auto-heal, and clears), and an append failure is never swallowed silently: it increments a dropped-writes counter and clears an "audit healthy" flag. **These are in-process signals only** (C++ getters on the node). This revision exposes no service/REST/health endpoint that surfaces audit health or lets an operator run `verify()` at runtime, so the signals are **not operator-observable at runtime yet** - a runtime read/verify/health surface is future work. With `audit_log.fail_closed` set, an append failure is re-raised as a **fail-FAST** error so a compliance-strict deployment learns the audit broke. This does **not** roll back the fault-state change that already committed: the audit log and the fault store are **separate SQLite databases**, so there is no cross-DB atomicity, and `fail_closed` is a broken-audit alarm requiring operator action, not a rollback. The default (`fail_closed=false`) keeps fault processing running; either way the in-process signals record the gap.

`verify()` walks the persisted chain oldest-first and recomputes every link: editing a row breaks its `record_hash`, and deleting a row breaks the next row's `prev_hash` linkage. Deleting the newest row *while leaving the head untouched* is caught by the persisted-head check (the head is read straight from the DB). However, deleting the newest row(s) **and** repointing the head with a single `UPDATE audit_chain_head SET seq=..., record_hash=...` to the prior row's values costs no more than any other casual edit - it is **not** the "recompute the entire chain" the threat model below might suggest - and the truncated chain still verifies. There is no external record that a later `seq` ever existed, so this tail-truncation is undetectable by design.

**Threat model (read this).** The chain is **unkeyed**, and the head and segment anchors live in the **same writable SQLite file** as the rows. `verify()` therefore catches edits or deletions that did **not** also recompute the chain - that is, casual or accidental tampering, and the bookkeeping bugs that would otherwise lose records. The append-only triggers are defense-in-depth: `audit_log` rejects out-of-band UPDATE/DELETE, `audit_anchors` carries the same guard-gated triggers so an out-of-band INSERT/UPDATE/DELETE of an anchor is rejected too, and the rotation-prune guard (`audit_prune_guard`) is itself protected by a trigger so an external writer cannot simply flip it open and then delete a prefix (or forge an anchor) - that flip is only permitted from the in-process connection that holds a per-connection temp marker. The single-row chain head (`audit_chain_head`) is intentionally **not** trigger-protected (a trigger there would block the legitimate head update inside the append transaction); a casual edit or delete of the head is instead caught by `verify()` via the seq/hash/head-mismatch checks. None of this stops an attacker with write access to the file: such an attacker can create the same temp marker or drop the triggers, and recompute the entire chain (head and anchors included) to forge a self-consistent history - and cheaper still, the tail-truncation above and the forged prefix-truncation below need no recompute at all. The triggers are **not** a security boundary - this is tamper-**evident**, not tamper-**proof**. True tamper-*proofing* requires a key or signature over the head (so it cannot be recomputed without the key) or external anchoring of the head hash to an append-only store you do not control; both are out of scope here and belong to the audit-log exporter / signing follow-up.

**Retention/rotation**: when more than `audit_log.retention_max_records` rows are retained, the oldest segment is *sealed* (its final `seq` + hash are persisted as an anchor) and then pruned. The surviving tail still verifies because the oldest retained row links back to the sealed anchor. Only the anchor at the current prune boundary is kept - the same rotation drops older anchors - so `audit_anchors` stays bounded (one row) instead of growing one row per rotation. Because `verify()` treats any matching sealed anchor as a valid tail root, a **forged** prefix-truncation (an out-of-band actor deletes a prefix and inserts a matching anchor) is **indistinguishable** from legitimate pruning: "the surviving tail still verifies" therefore covers a forged truncation exactly as well as a real one. The guard-gated `audit_anchors` triggers raise the bar for this (casual/accidental only) but, like every trigger here, a write-capable adversary can drop them - so this stays tamper-**evident**, not tamper-**proof**.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `audit_log.enabled` | bool | `false` | Enable the tamper-evident audit log |
| `audit_log.transitions` | string | `"all"` | Which transitions to record: `"all"` (occurred/confirmed/healed/cleared) or `"confirmed_only"`. Lifecycle markers are always recorded. |
| `audit_log.database_path` | string | `""` | SQLite path. Empty => sibling `fault_audit.db` next to the fault DB (or `:memory:` for in-memory fault stores) |
| `audit_log.retention_max_records` | int | `0` | Seal + prune the oldest segment beyond this many retained records (0 = unlimited) |
| `audit_log.fail_closed` | bool | `false` | When `true`, an audit append failure is re-raised as a fail-FAST error signalling the audit chain is broken and needs operator action. It does **not** roll back the already-committed fault-state change (the fault store is a separate DB - no cross-DB atomicity). When `false`, the failure is logged and counted and fault processing continues. Either way the gap is recorded via the in-process dropped-writes / audit-healthy signals (not operator-observable at runtime yet). |

## Usage

### Launch

```bash
# Default (SQLite storage, immediate confirmation)
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# With custom database path
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p database_path:=/custom/path/faults.db

# With in-memory storage (no persistence)
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p storage_type:=memory
```

## Advanced: Debounce Filtering

For systems that need to filter transient faults, enable debounce filtering by setting a lower `confirmation_threshold`.

### Configuration

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p confirmation_threshold:=-3 \
  -p healing_enabled:=true \
  -p healing_threshold:=3
```

### How It Works

The fault manager uses an AUTOSAR DEM-style debounce model:

- **FAILED events** (fault detected): Decrement the internal counter
- **PASSED events** (fault cleared): Increment the internal counter
- Fault becomes **CONFIRMED** when counter reaches `confirmation_threshold`
- Fault becomes **HEALED** when counter reaches `healing_threshold` (if enabled)

The counter is always clamped to `[confirmation_threshold, healing_threshold]`, so a long run of
one-sided events cannot push it out to the integer limits and delay the opposite transition.
`confirmation_threshold < 0 <= healing_threshold` is required (`healing_threshold = 0` heals on a
single PASSED event); invalid thresholds fall back to safe defaults with a warning.

`CONFIRMED` and `HEALED` are **latched** (hysteresis): once reached, the status holds until the
counter reaches the opposite threshold, so a single opposite-direction event cannot flip it. As a
result a fault that becomes active again can take up to `healing_threshold - confirmation_threshold`
events to return to the default (CONFIRMED-only) list; `occurrence_count` and `last_occurred` still
update meanwhile.

### Fault Lifecycle with Debounce

```
     FAILED events          PASSED events
          |                      |
          v                      v
   [counter--]             [counter++]
          |                      |
          v                      v
PREFAILED -----> CONFIRMED -----> HEALED (retained)
  (counter     (counter <=     (counter >=
   < 0)         threshold)      healing)
                    |
                    v
                CLEARED (manual via ~/clear_fault)
```

### Status Reference

| Status | Description |
|--------|-------------|
| `PREFAILED` | Debounce counter < 0, not yet confirmed |
| `CONFIRMED` | Fault is active and verified |
| `HEALED` | Resolved via PASSED events (if healing enabled) |
| `CLEARED` | Manually acknowledged via `~/clear_fault` |

### Testing Debounce

```bash
# Report FAILED events (need 3 to confirm with threshold=-3)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 0, severity: 2, description: 'Sensor timeout', source_id: '/sensor'}"

# Report PASSED event (fault condition cleared)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 1, severity: 0, description: '', source_id: '/sensor'}"

# Query all statuses including PREFAILED
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['PREFAILED', 'CONFIRMED', 'HEALED']}"
```

Event types: `0` = EVENT_FAILED, `1` = EVENT_PASSED

### Immediate Confirmation

CRITICAL severity faults bypass debounce and are immediately CONFIRMED, regardless of threshold.

## Advanced: Fault Correlation

Fault correlation reduces noise by identifying relationships between faults. When enabled, symptom faults
(effects of a root cause) can be muted and auto-cleared when the root cause is resolved.

### Correlation Modes

**Hierarchical**: Defines explicit root cause → symptoms relationships. When a root cause fault occurs,
subsequent matching symptom faults within a time window are correlated and optionally muted.

**Auto-Cluster**: Automatically groups related faults that match a pattern within a time window.
Useful for detecting "storms" of related faults (e.g., communication errors).

### Configuration

Enable correlation by providing a YAML configuration file:

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p correlation.config_file:=/path/to/correlation.yaml
```

### Correlation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `correlation.config_file` | string | `""` | Path to correlation YAML config (empty = disabled) |
| `correlation.cleanup_interval_sec` | double | `5.0` | Interval for cleaning up expired pending correlations (seconds) |

### Configuration File Format

```yaml
correlation:
  enabled: true
  default_window_ms: 500  # Default time window for symptom detection

  # Reusable fault patterns (supports wildcards with *)
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    drive_faults:
      codes: ["DRIVE_*"]
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]

  rules:
    # Hierarchical rule: E-Stop causes motor and drive faults
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001", "ESTOP_002"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000           # Symptoms within 1s of root cause
      mute_symptoms: true       # Don't publish symptom events
      auto_clear_with_root: true # Clear symptoms when root cause clears

    # Auto-cluster rule: Group communication errors
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3              # Need 3 faults to form cluster
      window_ms: 500            # Within 500ms
      show_as_single: true      # Only show representative fault
      representative: highest_severity  # first | most_recent | highest_severity
```

### Pattern Wildcards

Patterns support `*` wildcard matching:
- `MOTOR_*` matches `MOTOR_COMM`, `MOTOR_TIMEOUT`, `MOTOR_DRIVE_FAULT`
- `*_COMM_*` matches `MOTOR_COMM_FL`, `SENSOR_COMM_TIMEOUT`
- `*_TIMEOUT` matches `MOTOR_TIMEOUT`, `SENSOR_TIMEOUT`

### Querying Correlation Data

Use `include_muted` and `include_clusters` to retrieve correlation information:

```bash
# Get faults with muted fault details
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED'], include_muted: true, include_clusters: true}"
```

Response includes:
- `muted_count`: Number of muted symptom faults
- `cluster_count`: Number of active fault clusters
- `muted_faults[]`: Details of muted faults (when `include_muted=true`)
- `clusters[]`: Details of active clusters (when `include_clusters=true`)

### REST API (via Gateway)

Query parameters for GET `/api/v1/faults`:
- `include_muted=true`: Include muted fault details in response
- `include_clusters=true`: Include cluster details in response

Response fields:
```json
{
  "faults": [...],
  "count": 5,
  "muted_count": 2,
  "cluster_count": 1,
  "muted_faults": [
    {
      "fault_code": "MOTOR_COMM_FL",
      "root_cause_code": "ESTOP_001",
      "rule_id": "estop_cascade",
      "delay_ms": 50
    }
  ],
  "clusters": [
    {
      "cluster_id": "comm_storm_1",
      "rule_id": "comm_storm",
      "rule_name": "Communication Storm",
      "representative_code": "SENSOR_TIMEOUT",
      "representative_severity": "CRITICAL",
      "fault_codes": ["MOTOR_COMM_FL", "SENSOR_TIMEOUT", "DRIVE_COMM_ERR"],
      "count": 3,
      "first_at": 1705678901.123,
      "last_at": 1705678901.456
    }
  ]
}
```

When clearing a root cause fault, `auto_cleared_codes` lists symptoms that were auto-cleared:
```json
{
  "status": "success",
  "fault_code": "ESTOP_001",
  "message": "Fault cleared",
  "auto_cleared_codes": ["MOTOR_COMM_FL", "MOTOR_COMM_FR", "DRIVE_FAULT"]
}
```

### Example: E-Stop Cascade

1. E-Stop is triggered → `ESTOP_001` fault reported
2. Motors lose power → `MOTOR_COMM_FL`, `MOTOR_COMM_FR` faults reported
3. Correlation engine detects motor faults are symptoms of E-Stop
4. Motor faults are muted (not published as events, but stored)
5. Dashboard shows only `ESTOP_001` (root cause)
6. When E-Stop is cleared → Motor faults are auto-cleared

## Namespaced Deployment

The fault manager can run in a custom ROS 2 namespace. The gateway resolves service and topic
names automatically via the `fault_manager.namespace` parameter:

```yaml
# gateway_params.yaml
fault_manager:
  namespace: "robot1"          # -> /robot1/fault_manager/list_faults
  service_timeout_sec: 5.0
```

Launch the fault manager in a namespace:

```bash
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py \
  namespace:=robot1
```

Leading slashes are optional - `"robot1"` and `"/robot1"` are equivalent.

## Building

```bash
colcon build --packages-select ros2_medkit_fault_manager
source install/setup.bash
```

## Testing

```bash
colcon test --packages-select ros2_medkit_fault_manager
colcon test-result --verbose
```

## License

Apache-2.0
