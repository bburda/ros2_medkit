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
# FaultEvent.msg - Real-time fault event notifications.
#
# Published by FaultManager on /fault_manager/events when fault state changes occur.
# Used for SSE (Server-Sent Events) streaming via the gateway REST API.

# Event type identifier (use EVENT_* constants)
string event_type

# The fault data associated with this event (contains current state after the event)
Fault fault

# Timestamp when this event was generated
builtin_interfaces/Time timestamp

# Event type constants
# Emitted when a fault transitions from PREFAILED to CONFIRMED status
# (e.g., after debounce counter reaches confirmation threshold).
string EVENT_CONFIRMED = "fault_confirmed"
# Emitted when a fault ends: CLEARED via the ClearFault service, or HEALED when
# PASSED events drive it past the healing threshold. Check fault.status to tell
# the two apart.
string EVENT_CLEARED = "fault_cleared"
# Emitted when fault data changes without status transition (e.g., last_occurred
# advanced, severity escalated, new source added to reporting_sources).
# occurrence_count is not one of them: it only changes on an occurrence edge
# (the first FAILED, or a FAILED that arrives while the fault is CLEARED).
string EVENT_UPDATED = "fault_updated"

# Correlation-related fields (populated when correlation is enabled)

# List of symptom fault codes that were automatically cleared along with a root cause.
# Only populated for EVENT_CLEARED when the cleared fault was a root cause with
# correlated symptoms and auto_clear_with_root=true.
string[] auto_cleared_codes
