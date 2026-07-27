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
# AlarmList.msg - every alarm in the PLC that is active or waiting for
# acknowledgement, published cyclically.
#
# A picture of the current state, not a stream of events. Two reasons.
#
# The transport is cyclic: the PLC-side node reads a data block at a fixed rate,
# so a picture is what it naturally has. An event stream would have to be built
# on top of it and would then need its own delivery guarantees.
#
# A picture survives a restart. Whoever restarts - the PLC-side node, the bridge,
# the whole diagnostics stack - the next cycle carries the full truth again. An
# event stream loses whatever happened while it was away, and the consumer has no
# way to tell an empty stream from a healthy machine. This is the same gap we hit
# reading alarms over OPC UA, where no table of pending alarms is served on
# reconnect.
#
# The bridge compares consecutive pictures: an entry that appears is reported as a
# fault, an entry that disappears heals it.

# When this picture was taken, from the PLC clock.
builtin_interfaces/Time stamp

# Alarms that are active or not yet acknowledged. Order is not significant.
#
# OPEN QUESTION: how many entries can the generated data block hold? A plant can
# declare hundreds of alarms, and the array is fixed size once generated. If the
# ceiling is low, the fill has to prioritise, and then it also has to say that it
# truncated.
Alarm[] alarms
