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
# PlcAlarm.msg - one alarm event as reported by a PLC alarm system.
#
# Published by a vendor bridge or by generated PLC glue code, consumed by
# ros2_medkit_plc_bridge, which maps each event onto a SOVD fault.
#
# Design intent: the payload is fixed-size and free of runtime strings that the
# PLC would have to build. Alarm texts are not carried here. A generator or an
# engineering export provides a static alarm_id -> text table that the bridge
# resolves on the ROS 2 side. String handling in a PLC costs determinism, and
# alarm texts do not change at runtime.

# Numeric alarm identifier as declared in the PLC project.
#
# OPEN QUESTION: is this identifier unique per CPU, or only per function block?
# If it is only unique per block, the unique key must be (source, alarm_id) and
# consumers cannot key on alarm_id alone.
uint32 alarm_id

# Origin of the alarm inside the PLC program, e.g. the function block or
# instance name that declared it. Static knowledge of the generator or the
# engineering export, not read from the PLC at runtime.
# Required when alarm_id is not globally unique (see above).
string source

# Alarm class as declared in the PLC project, e.g. whether the alarm requires
# acknowledgement. Carried as an opaque string because the class catalogue is
# project and vendor specific.
string alarm_class

# Whether the alarm condition appeared or disappeared. Use STATE_* constants.
#
# The event is not a level: a GOING event does not delete the fault, it moves
# it towards healing. The fault stays visible for the audit trail until it is
# cleared, which mirrors how PLC alarm systems latch conditions.
uint8 state
uint8 STATE_COMING = 0
uint8 STATE_GOING = 1

# Acknowledgement state as seen by the PLC alarm system. Use ACK_* constants.
#
# This is the shared acknowledgement truth. Whoever acknowledges first - an
# operator on the panel or a consumer on the ROS 2 side - every other consumer
# sees the same state instead of keeping a private copy.
uint8 ack_state
uint8 ACK_UNACKNOWLEDGED = 0
uint8 ACK_ACKNOWLEDGED = 1

# Time of the event as taken from the PLC clock, in UTC.
#
# Deliberately the source time, not the time the message was received. A
# consumer can compare it against its own clock to detect a PLC that is not
# time-synchronised, which is common on plant networks.
builtin_interfaces/Time plc_timestamp

# Process values latched by the alarm system at the moment the alarm fired.
# They become the freeze-frame of the resulting fault.
#
# OPEN QUESTION: numeric only, or a typed variant? Alarm systems can attach
# non-numeric values. Numeric keeps the message fixed-size and cheap on the PLC
# side; a typed variant is richer but reintroduces strings.
float64[] associated_values
