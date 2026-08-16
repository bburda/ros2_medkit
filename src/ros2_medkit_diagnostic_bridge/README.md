# ros2_medkit_diagnostic_bridge

Bridge node that converts ROS2 `/diagnostics` messages to FaultManager faults.

## Overview

This package provides backwards compatibility with existing ROS2 diagnostic infrastructure.
It subscribes to the standard `/diagnostics` topic and forwards diagnostic status messages
to the FaultManager as faults.

## Severity Mapping

| DiagnosticStatus Level | Fault Severity | Action |
|------------------------|----------------|--------|
| OK (0) | - | Sends PASSED event (healing) |
| WARN (1) | WARN (1) | Reports fault |
| ERROR (2) | ERROR (2) | Reports fault |
| STALE (3) | CRITICAL (3) | Reports fault |

## Quick Start

```bash
# Start FaultManager first
ros2 run ros2_medkit_fault_manager fault_manager_node

# Start DiagnosticBridge
ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge_node
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `diagnostics_topic` | string | `/diagnostics` | Topic to subscribe to |
| `auto_generate_codes` | bool | `true` | Auto-generate fault codes from diagnostic names |
| `use_hardware_id_as_source_id` | bool | `false` | Use slash-containing diagnostic `hardware_id` values as fault `source_id` |
| `max_tracked_sources` | integer | `512` | Maximum number of per-source FaultReporter instances retained by the bridge |
| `name_to_code.<name>` | string | - | Custom mapping from diagnostic name to fault code |
| `keyvalue_codes` | string[] | - | List of keys used to search the diagnostic values for the fault code |

### Example Configuration

```yaml
diagnostic_bridge:
  ros__parameters:
    diagnostics_topic: "/diagnostics"
    auto_generate_codes: true
    # Opt in to source attribution from diagnostic hardware_id values
    use_hardware_id_as_source_id: false
    max_tracked_sources: 512

    # Custom mappings (optional)
    # Format: "name_to_code.<diagnostic_name>": "<FAULT_CODE>"
    "name_to_code.motor_controller: Temperature": "MOTOR_OVERHEAT"
    "name_to_code.battery_monitor: Voltage": "BATTERY_LOW"
```

### Fault Code Generation

When `auto_generate_codes` is enabled, diagnostic names are converted to fault codes:

| Diagnostic Name | Generated Fault Code |
|-----------------|---------------------|
| `motor temp` | `MOTOR_TEMP` |
| `motor: Status` | `MOTOR_STATUS` |
| `/robot/sensor` | `ROBOT_SENSOR` |

Custom mappings in `name_to_code` take priority over auto-generation.

### Fault Source Attribution

By default, faults reported by this bridge use the bridge node's fully-qualified
name (`/diagnostic_bridge`) as their `source_id`. This preserves the behavior of
existing deployments because `DiagnosticStatus.hardware_id` commonly contains a
serial number, device path, or no value rather than a ROS node name.

Set `use_hardware_id_as_source_id` to `true` to opt in to hardware ID attribution.
When enabled, a non-empty `hardware_id` is used exactly as provided only when it
contains `/`, which is treated as a heuristic for a node/FQN-like identifier.
Empty or non-slash hardware IDs fall back to the bridge FQN. This option does not
perform manifest lookup or normalize hardware IDs; the accepted value must already
match the runtime entity source ID used by the gateway.

The bridge keeps one `FaultReporter` per active source so local filtering remains
isolated by source. `max_tracked_sources` bounds this cache with least-recently-used
eviction; values below `1` are clamped to `1`.

## Launch

```bash
# Using launch file with default config
ros2 launch ros2_medkit_diagnostic_bridge diagnostic_bridge.launch.py

# With custom config
ros2 launch ros2_medkit_diagnostic_bridge diagnostic_bridge.launch.py \
    config_file:=/path/to/custom_config.yaml
```

## Integration with FaultManager

For full healing support, configure FaultManager with:

```yaml
fault_manager:
  ros__parameters:
    healing_enabled: true
    healing_threshold: 1
```

When a diagnostic transitions from ERROR/STALE to OK, the bridge sends a PASSED event,
allowing the fault to heal in FaultManager.

## Building

```bash
colcon build --packages-select ros2_medkit_diagnostic_bridge
```

## Testing

```bash
colcon test --packages-select ros2_medkit_diagnostic_bridge
colcon test-result --verbose
```

## License

Apache-2.0
