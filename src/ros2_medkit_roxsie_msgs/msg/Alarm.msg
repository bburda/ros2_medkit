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
# Alarm.msg - one alarm in the PLC, as one entry of AlarmList.
#
# Not an event. An entry stays in the list while the alarm is active or waiting
# for acknowledgement, and drops out once it is gone and acknowledged. The two
# flags below mirror how the CPU alarm system tracks a condition.
#
# Fields the bit path cannot fill stay empty. A consumer that finds an empty
# alarm_class or no associated values is looking at a bit-filled alarm, not at
# an error.

# Numeric alarm identifier from the PLC project.
#
# Unique within one CPU (Siemens, 2026-09-03), so inside one PLC the number is
# the key. Two CPUs can use the same number, so the bridge keys on
# (PLC, alarm_id). Not stable: copying a block and recompiling can renumber,
# which is what AlarmList.catalog_version guards against.
uint32 alarm_id

# Block or instance that declared the alarm. Static knowledge of the generator.
# Also the natural handle for placing the alarm under an entity in the tree.
string source

# Alarm class from the project, e.g. whether the alarm needs acknowledgement.
# Empty when the alarm comes from bits, which carry no class.
string alarm_class

# Priority as the PLC knows it: 0 to 16, higher is more important. This is the
# scale the plant already engineers with, so we carry it instead of inventing
# one. 0 when unknown.
uint8 priority

# Where the alarm came from inside the CPU: "user_program", "system_diagnostics",
# "prodiag", "graph", "motion", "security". Empty for bit-filled alarms.
#
# Worth carrying because the CPU message system produces more than user alarms.
# A module fault or a drive alarm arrives here without anyone declaring it.
string producer

# Whether the condition is currently present.
bool active

# Whether the alarm has been acknowledged.
#
# Both flags together, not one enum: an alarm that is gone but unacknowledged is
# a real state and stays in the list. Same two bits the CPU alarm system tracks.
#
# Who owns this depends on how the list is filled. Bits: shared, either side can
# set it. CPU alarm system: the CPU owns it and we mirror it. The CPU has no
# per-alarm acknowledge instruction: Ack_Alarms takes no alarm id, and MODE = 1
# confirms every pending alarm at once, in batches of 100 (Siemens, 2026-09-03).
# The bridge never calls it. Acknowledging one alarm needs the panel or OPC UA.
bool acknowledged

# Time the condition last changed, from the PLC clock, UTC.
#
# The source time, so a consumer can compare it against its own clock and spot a
# PLC that is not time-synchronised. That happens often on plant networks. Empty
# in the bit path, where nobody recorded when it happened.
builtin_interfaces/Time plc_timestamp

# Values the alarm system latched when the alarm fired. They become the
# freeze-frame of the fault.
#
# The CPU caps these at 512 bytes across all of them. In the bit path there are
# none, and the freeze-frame comes from the process topics instead.
float64[] associated_values
