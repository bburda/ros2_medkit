# Copyright 2026 mfaferek93
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
# DiagnosticsStatus.msg - aggregate diagnostics state sent towards the PLC.
#
# Published by ros2_medkit_roxsie_bridge, consumed by the ROXSIE generated node,
# which writes the values into a consumed data block the PLC program reads.
#
# Design intent: this is the whole of what the diagnostics side puts inside the
# PLC. Faults stay on the ROS 2 side and are read over REST by whoever needs
# them. What the PLC genuinely needs is a lamp and, optionally, a condition it
# can interlock on. Four fixed words, no strings, no lists.

# Liveness counter of the diagnostics layer, incremented about once per second.
#
# A watchdog in the PLC program turns a stalled counter into its own alarm, so
# the plant sees "diagnostics offline" instead of a stale healthy state. The
# failure mode of the diagnostics layer must never look like a healthy machine.
uint16 heartbeat

# Highest severity among the active faults in scope.
# Values follow ros2_medkit_msgs/Fault SEVERITY_* semantics: 0 INFO, 1 WARN,
# 2 ERROR, 3 CRITICAL. Typically drives the signal column.
uint8 aggregate_severity

# Number of active faults in scope. Optional indicator on the panel.
uint16 active_fault_count

# Coarse fault classes currently active, one bit per class.
#
# Intended for machine logic, for example an interlock that must not depend on
# reading a fault list. The mapping from fault codes to bits is deployment
# configuration on the ROS 2 side, not a fixed catalogue in this message.
uint16 class_bitmask

# Which part of the plant this status covers, expressed as an entity in the SOVD
# tree, e.g. an area or a component id. Lets one diagnostics instance serve
# several cells without them seeing each other's faults.
string scope
