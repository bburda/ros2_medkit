# Copyright 2025 mfaferek93
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Fault.msg - Core fault data model for ros2_medkit fault management system.
#
# A Fault represents an aggregated fault condition identified by a global fault_code.
# Multiple sources can report the same fault_code, and they are aggregated into a
# single Fault with tracked occurrence_count and reporting_sources.
#
# Debounce model (AUTOSAR DEM-style):
# - FAILED events decrement internal counter (towards confirmation)
# - PASSED events increment internal counter (towards healing)
# - Status reflects the current debounce state

# Global fault identifier (e.g., "MOTOR_OVERHEAT", "SENSOR_FAILURE_001")
string fault_code

# Fault severity level (use SEVERITY_* constants)
uint8 severity

# Human-readable description of the fault condition
string description

# Timestamp when this fault was first reported
builtin_interfaces/Time first_occurred

# Timestamp when this fault last occurred (FAILED events only; a PASSED
# event is the fault ending, not occurring, and does not touch this field)
builtin_interfaces/Time last_occurred

# Timestamp of the last PASSED event reported for this fault (zero = never)
builtin_interfaces/Time last_passed

# Total number of FAILED events aggregated across all sources
uint32 occurrence_count

# Current fault status (use STATUS_* constants)
string status

# List of source identifiers that have reported this fault
string[] reporting_sources

# Severity level constants
# Informational message: No action required, used for logging or tracking.
uint8 SEVERITY_INFO = 0
# Warning: May require attention, but does not impact core functionality.
uint8 SEVERITY_WARN = 1
# Error: Impacts functionality and requires intervention.
uint8 SEVERITY_ERROR = 2
# Critical: Severe error requiring immediate attention; may compromise safety or system operation.
# CRITICAL severity bypasses debounce filtering and confirms immediately.
uint8 SEVERITY_CRITICAL = 3

# Status constants
# Fault status lifecycle with debounce filtering:
#
#   PREFAILED ←────────────────→ PREPASSED
#       │      (counter crosses 0)    │
#       ▼                             ▼
#   CONFIRMED                      HEALED
#       │                          (retained)
#       ▼
#   CLEARED (manual)
#
# - PREFAILED: Debounce counter < 0 but above confirmation threshold.
#   Fault condition detected but not yet confirmed. More FAILED events push towards CONFIRMED.
# - PREPASSED: Debounce counter > 0 but below healing threshold.
#   More PASSED than FAILED events - fault condition trending towards resolution.
# - CONFIRMED: Debounce counter <= confirmation_threshold (e.g., -3).
#   Fault is active and verified. Appears in default ListFaults queries.
# - HEALED: Debounce counter >= healing_threshold (e.g., +3).
#   Fault resolved by sufficient PASSED events. Retained in storage, queryable with statuses=["HEALED"].
# - CLEARED: Fault manually acknowledged/resolved via ClearFault service.
#   Retained for historical analysis, queryable with statuses=["CLEARED"].
string STATUS_PREFAILED = "PREFAILED"
string STATUS_PREPASSED = "PREPASSED"
string STATUS_CONFIRMED = "CONFIRMED"
string STATUS_HEALED = "HEALED"
string STATUS_CLEARED = "CLEARED"
