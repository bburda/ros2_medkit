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
# AlarmEvent.msg - one alarm event from the PLC alarm system, carried onto the
# ROS 2 graph by the node that Siemens SIMATIC ROS Connector (ROXSIE) generates.
#
# Published by the ROXSIE generated node, consumed by ros2_medkit_roxsie_bridge,
# which maps each event onto a SOVD fault with a freeze-frame and an audit trail.
#
# The payload is fixed-size and carries no strings the PLC would have to build.
# Alarm text stays out of it: the generator writes an alarm_id -> text table next
# to the code, and the bridge looks the text up on the ROS 2 side. A PLC that
# handles strings loses determinism, and alarm text does not change while the
# machine runs.

# Numeric alarm identifier as declared in the TIA project.
#
# OPEN QUESTION: is this identifier unique per CPU, or only per function block?
# If it is only unique per block, the unique key must be (source, alarm_id) and
# the bridge cannot key on alarm_id alone.
uint32 alarm_id

# Origin of the alarm inside the PLC program, e.g. the function block or
# instance that declared it. Static knowledge of the generator, not read from
# the PLC at runtime. Required when alarm_id is not unique per CPU (see above).
string source

# Alarm class as declared in the TIA project, e.g. whether the alarm requires
# acknowledgement. Carried as an opaque string because the class catalogue is
# project specific.
string alarm_class

# Whether the alarm condition appeared or disappeared. Use STATE_* constants.
#
# This is an event, not a level. GOING moves the fault towards healing; we keep
# the fault until it is cleared, so the history stays for the audit trail. The
# alarm system latches conditions the same way.
uint8 state
uint8 STATE_COMING = 0
uint8 STATE_GOING = 1

# Acknowledgement state as seen by the PLC alarm system. Use ACK_* constants.
#
# One shared value. The operator acks on the panel or a client acks over REST,
# and both sides see the same thing.
uint8 ack_state
uint8 ACK_UNACKNOWLEDGED = 0
uint8 ACK_ACKNOWLEDGED = 1

# Time of the event taken from the PLC clock, in UTC.
#
# The source time, so a consumer can compare it against its own clock and spot a
# PLC that is not time-synchronised. That happens often on plant networks.
builtin_interfaces/Time plc_timestamp

# Values latched by the alarm system at the moment the alarm fired, the
# associated values of the alarm. They become the freeze-frame of the fault.
#
# OPEN QUESTION: numeric only, or a typed variant? The alarm system can attach
# non-numeric values. Numeric keeps the message fixed-size and cheap on the PLC
# side; a typed variant is richer but reintroduces strings.
float64[] associated_values
