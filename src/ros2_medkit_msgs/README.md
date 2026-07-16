# ros2_medkit_msgs

ROS 2 message and service definitions for the ros2_medkit fault management system.

## Overview

This package provides the interface definitions used by the fault management components:

- **FaultManager** (`ros2_medkit_fault_manager`) - Central fault aggregation and lifecycle management
- **FaultReporter** (`ros2_medkit_fault_reporter`) - Client library for fault reporting
- **Gateway** (`ros2_medkit_gateway`) - REST API endpoints for fault access

## Messages

### Fault.msg

Core fault data model representing an aggregated fault condition with AUTOSAR DEM-style debounce filtering.

| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | Global fault identifier (e.g., "MOTOR_OVERHEAT") |
| `severity` | uint8 | Severity level (use SEVERITY_* constants) |
| `description` | string | Human-readable description |
| `first_occurred` | builtin_interfaces/Time | When fault was first reported |
| `last_occurred` | builtin_interfaces/Time | When fault was last reported (FAILED or PASSED) |
| `occurrence_count` | uint32 | Total FAILED events aggregated across all sources |
| `status` | string | Current status (see STATUS_* constants) |
| `reporting_sources` | string[] | List of source identifiers that reported this fault |

**Severity Levels:**
| Constant | Value | Description |
|----------|-------|-------------|
| `SEVERITY_INFO` | 0 | Informational, no action required |
| `SEVERITY_WARN` | 1 | May require attention, no impact on functionality |
| `SEVERITY_ERROR` | 2 | Impacts functionality, requires intervention |
| `SEVERITY_CRITICAL` | 3 | Severe, may compromise safety or system operation. Bypasses debounce. |

**Status Constants:**
| Constant | Description |
|----------|-------------|
| `STATUS_PREFAILED` | Debounce counter < 0 but above confirmation threshold |
| `STATUS_PREPASSED` | Debounce counter > 0 but below healing threshold |
| `STATUS_CONFIRMED` | Fault confirmed (counter <= threshold, e.g., -3) |
| `STATUS_HEALED` | Fault healed by PASSED events (if healing enabled) |
| `STATUS_CLEARED` | Fault manually cleared via ClearFault service |

**Status Lifecycle (Debounce Model):**
```
PREFAILED ←→ PREPASSED → HEALED (retained)
    ↓
CONFIRMED → CLEARED (manual)
```
- FAILED events decrement counter (towards confirmation)
- PASSED events increment counter (towards healing)
- CRITICAL severity bypasses debounce and confirms immediately

### FaultEvent.msg

Real-time fault event notification for SSE streaming (published on `/fault_manager/events`).

| Field | Type | Description |
|-------|------|-------------|
| `event_type` | string | Event type (see constants below) |
| `fault` | Fault | The fault data (state after event) |
| `timestamp` | builtin_interfaces/Time | When the event occurred |

**Event Types:**
| Constant | Trigger |
|----------|---------|
| `EVENT_CONFIRMED` | Fault transitions PREFAILED → CONFIRMED |
| `EVENT_CLEARED` | Fault transitions to CLEARED |
| `EVENT_UPDATED` | Fault data changes without status transition |

## Services

### ReportFault.srv

Report a fault event (FAILED or PASSED) to the FaultManager.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | Global identifier (UPPER_SNAKE_CASE, max 64 chars) |
| `event_type` | uint8 | Event type: EVENT_FAILED (0) or EVENT_PASSED (1) |
| `severity` | uint8 | Severity level (0-3, only for FAILED events) |
| `description` | string | Human-readable description (only for FAILED events) |
| `source_id` | string | Reporting node FQN (e.g., "/powertrain/engine/temp_sensor") |
| `supersedes_source_id` | string | Optional. A previously-reported source to drop from `reporting_sources` before `source_id` is added (empty = no supersede). Corrects a provisional attribution so the fault resolves to the right entity under the strict-AND per-entity scope filter. |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `accepted` | bool | True if the event was accepted |

**Event Types:**
- `EVENT_FAILED` (0): Fault condition detected - decrements debounce counter
- `EVENT_PASSED` (1): Fault condition cleared - increments debounce counter

### ListFaults.srv

Query faults with optional filtering.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `filter_by_severity` | bool | If true, filter by severity field; if false, return all severities |
| `severity` | uint8 | Severity level (0-3), only used if filter_by_severity is true |
| `statuses` | string[] | Statuses to include (empty = CONFIRMED only) |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `faults` | Fault[] | Matching faults |

**Examples:**
- Default (active faults): `filter_by_severity=false, statuses=[]` → all CONFIRMED faults
- Only errors: `filter_by_severity=true, severity=2, statuses=[]` → CONFIRMED with ERROR
- All faults: `filter_by_severity=false, statuses=["PREFAILED", "CONFIRMED", "CLEARED"]`
- Historical: `filter_by_severity=false, statuses=["CLEARED"]`

### ClearFault.srv

Clear/acknowledge a fault. Cleared faults are retained and queryable with `statuses=["CLEARED"]`.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | The fault to clear |
| `skip_correlation_auto_clear` | bool | When `true`, only the requested fault_code is cleared; symptom faults that the correlation engine would normally auto-clear via `auto_clear_with_root` rules are left untouched. Default `false` (cascade clear). The gateway sets this to `true` on per-entity `DELETE /{entity-path}/faults/{fault_code}` so that an operator with access to one entity cannot cascade-clear correlated symptoms reported by apps in other entities. |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | True if fault was cleared |
| `message` | string | Status or error message |
| `auto_cleared_codes` | string[] | Symptom fault codes auto-cleared with the root cause (empty when `skip_correlation_auto_clear=true`) |

> **Note:** `skip_correlation_auto_clear` was added in `ros2_medkit_msgs` post-0.4.0. Adding a request field changes the service type hash, so out-of-tree callers that invoke `/fault_manager/clear_fault` directly (via `ros2 service call` or a generated client) must rebuild against the new `ros2_medkit_msgs` release to keep talking to `fault_manager`.

## Usage

### C++

```cpp
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

// Create a fault message
ros2_medkit_msgs::msg::Fault fault;
fault.fault_code = "MOTOR_OVERHEAT";
fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
```

### Python

```python
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

# Create a fault message
fault = Fault()
fault.fault_code = "MOTOR_OVERHEAT"
fault.severity = Fault.SEVERITY_ERROR
fault.status = Fault.STATUS_CONFIRMED
```

## Building

```bash
colcon build --packages-select ros2_medkit_msgs
source install/setup.bash  # or setup.zsh for zsh users
```

## Verifying

```bash
ros2 interface show ros2_medkit_msgs/msg/Fault
ros2 interface show ros2_medkit_msgs/srv/ReportFault
```

## License

Apache-2.0
