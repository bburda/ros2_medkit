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
# DiagnosticsStatus.msg - what the diagnostics side asks of the machine.
#
# Written into a consumed data block by the PLC-side node. This is everything we
# put inside the PLC. Faults stay on the ROS 2 side and whoever wants them reads
# them over REST.
#
# It says what we are asking for, not what to display. Machines already have a
# state machine and, where PackML is used, a block that turns machine state into
# signal column bits. We do not drive lamps; we hand the machine a condition its
# own logic can act on. Two writers on one lamp is a fault of its own.

# Liveness counter of the diagnostics layer, incremented about once per second.
#
# When it stops, a watchdog in the PLC program raises its own alarm. The plant
# then sees that diagnostics are down instead of a green light that means
# nothing.
uint16 heartbeat

# What we are asking the machine to do about the faults in scope.
#
# The wording follows the colour semantics the signalling standards already use,
# so an integrator can wire it without a translation table.
uint8 condition
uint8 CONDITION_OK = 0
uint8 CONDITION_ABNORMAL = 1
uint8 CONDITION_ACTION_REQUIRED = 2
uint8 CONDITION_STOP = 3

# Optional, for machine logic only, one bit per coarse fault class. Meant for an
# interlock that must not depend on reading a fault list. Which fault sets which
# bit is deployment configuration on the ROS 2 side. Zero when unused.
uint16 class_bitmask

# Which part of the plant this status covers, as an entity in the SOVD tree, e.g.
# an area or a component id. Lets one diagnostics instance serve several cells
# without them seeing each other's faults.
string scope
